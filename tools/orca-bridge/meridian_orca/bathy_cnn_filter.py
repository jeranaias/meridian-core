"""CNN-backed particle filter — uses the learned matcher as the weight function.

Drop-in replacement for `PatchMatchFilter` that scores each particle with
the trained `BathyMatcherNet` instead of hand-coded Gaussian/MCC likelihood.

Performance expectation (per published LSTM/CNN-RBPF literature):
- 2-5 m median error on fine-resolution charts
- Better out-of-distribution generalization than the hand-coded filter
  (CNN learns to look at texture, not point values)

Runtime
-------
The CNN evaluates all particles in a single batch per step. On CPU torch
2.9.1, 1000 particles × 3×16×16 input ≈ 30 ms per step — fits comfortably
in the 1 Hz bathy update rate.
"""
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple

import numpy as np

try:
    import torch
    import torch.nn.functional as F
    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False

from .bathy_match import (
    BathymetricChart, GridChart, RealisticChart,
    METERS_PER_DEG_LAT, meters_per_deg_lon,
)
from .bathy_patch import DepthHistory
from .bathy_cnn import BathyMatcherNet, load_matcher, build_patch, build_patch_grid


# ===========================================================================
# Config
# ===========================================================================

@dataclass
class CNNFilterConfig:
    n_particles: int = 800            # CNN is slower, use fewer particles
    init_spread_m: float = 40.0
    process_noise_m_per_sqrt_s: float = 0.25
    regularization_m: float = 0.3
    resample_threshold: float = 0.5
    min_chart_depth: float = 0.3
    patch_window_s: float = 8.0
    patch_max_samples: int = 8
    patch_cells: int = 16
    patch_size_m: float = 80.0
    # Log-weight scaling — tune in [1, 10]. Higher = sharper particle
    # distribution, lower = more conservative
    cnn_log_weight_scale: float = 3.0


@dataclass
class CNNFilterEstimate:
    lat: float
    lon: float
    spread_m: float
    n_effective: float
    healthy: bool
    multimodal: bool


# ===========================================================================
# Filter
# ===========================================================================

class BathyCNNFilter:
    """Particle filter with CNN-based weighting."""

    def __init__(self, chart: BathymetricChart, model_path: str,
                 cfg: Optional[CNNFilterConfig] = None,
                 seed: int = 0xB474) -> None:
        if not _HAS_TORCH:
            raise RuntimeError("torch required for CNN filter")
        self.chart = chart
        self.cfg = cfg or CNNFilterConfig()
        self.rng = np.random.default_rng(seed)
        self.net = load_matcher(model_path, patch_cells=self.cfg.patch_cells)
        self.pos: Optional[np.ndarray] = None
        self.w:   Optional[np.ndarray] = None
        self.history = DepthHistory(
            window_s=self.cfg.patch_window_s,
            max_samples=max(50, self.cfg.patch_max_samples * 10),
        )
        self._t = 0.0
        # Cache grid params for the vectorized patch builder (50× speedup)
        if isinstance(chart, GridChart):
            self._grid_data = np.asarray(chart.data, dtype=np.float64)
            self._grid_lat_min = chart.lat_min
            self._grid_lon_min = chart.lon_min
            self._grid_dlat = chart.dlat
            self._grid_dlon = chart.dlon
            self._grid_n_rows = chart.n_rows
            self._grid_n_cols = chart.n_cols
        else:
            self._grid_data = None

    def initialize(self, lat: float, lon: float,
                   spread_m: Optional[float] = None) -> None:
        cfg = self.cfg
        N = cfg.n_particles
        s = spread_m if spread_m is not None else cfg.init_spread_m
        lat_scale = s / METERS_PER_DEG_LAT
        lon_scale = s / meters_per_deg_lon(lat)
        self.pos = np.empty((N, 2))
        self.pos[:, 0] = lat + self.rng.normal(0.0, lat_scale, N)
        self.pos[:, 1] = lon + self.rng.normal(0.0, lon_scale, N)
        self.w = np.full(N, 1.0 / N)

    def step(self, vel_n_mps: float, vel_e_mps: float, dt_s: float,
             observed_depth_m: float,
             ais_observations=None) -> CNNFilterEstimate:
        if self.pos is None:
            raise RuntimeError("call initialize() first")
        cfg = self.cfg
        self._t += dt_s

        # 1. Advance cumulative motion + push sample
        dn_m = vel_n_mps * dt_s
        de_m = vel_e_mps * dt_s
        self.history.advance(dn_m, de_m)
        self.history.push(self._t, observed_depth_m)

        # 2. Propagate particles
        sigma = cfg.process_noise_m_per_sqrt_s * math.sqrt(dt_s)
        m_per_lon = meters_per_deg_lon(float(self.pos[:, 0].mean()))
        noise = self.rng.normal(0.0, sigma, (cfg.n_particles, 2))
        self.pos[:, 0] += (dn_m + noise[:, 0]) / METERS_PER_DEG_LAT
        self.pos[:, 1] += (de_m + noise[:, 1]) / m_per_lon

        # 3. Build the depth-history track (observations + offsets from NOW)
        patch = self.history.offsets_to_now()
        if len(patch) > cfg.patch_max_samples:
            idx = np.linspace(0, len(patch) - 1, cfg.patch_max_samples).astype(int)
            patch = [patch[i] for i in idx]

        if not patch:
            # No depth history yet — bathy log-likelihood is uniform.
            # AIS observations can still lock position (useful on cold-start).
            log_w = np.zeros(cfg.n_particles)
            if ais_observations:
                from .ais_fusion import ais_log_likelihood_batch
                spread_prior = getattr(self, "_last_spread_m", cfg.init_spread_m)
                for obs in ais_observations:
                    log_w = log_w + ais_log_likelihood_batch(
                        self.pos[:, 0], self.pos[:, 1], obs,
                        current_spread_m=spread_prior)
                log_w -= log_w.max()
                w = np.exp(log_w)
                s = w.sum()
                self.w = w / s if s > 1e-30 else np.full(cfg.n_particles,
                                                          1.0 / cfg.n_particles)
            else:
                self.w[:] = 1.0 / cfg.n_particles
        else:
            # 4. Build 3-channel patch for each particle (N × 3 × H × W)
            N = cfg.n_particles
            xs = np.zeros((N, 3, cfg.patch_cells, cfg.patch_cells), dtype=np.float32)
            if self._grid_data is not None:
                # Fast vectorized path — 50× faster per particle
                for i in range(N):
                    xs[i] = build_patch_grid(
                        self._grid_data,
                        self._grid_lat_min, self._grid_lon_min,
                        self._grid_dlat, self._grid_dlon,
                        self._grid_n_rows, self._grid_n_cols,
                        self.pos[i, 0], self.pos[i, 1], patch,
                        patch_cells=cfg.patch_cells,
                        patch_size_m=cfg.patch_size_m,
                    )
            else:
                # Scalar fallback for non-grid charts
                for i in range(N):
                    xs[i] = build_patch(
                        self.chart.depth, self.pos[i, 0], self.pos[i, 1],
                        patch, patch_cells=cfg.patch_cells,
                        patch_size_m=cfg.patch_size_m,
                    )
            # 5. Score via CNN
            with torch.no_grad():
                logits = self.net(torch.from_numpy(xs))
                log_probs = F.logsigmoid(logits).numpy()
            # Scale to avoid overly peaked or flat weights
            log_w = log_probs * cfg.cnn_log_weight_scale

            # 5b. AIS fusion — additive log-likelihood per observation
            if ais_observations:
                from .ais_fusion import ais_log_likelihood_batch
                spread_prior = getattr(self, "_last_spread_m", cfg.init_spread_m)
                for obs in ais_observations:
                    log_w = log_w + ais_log_likelihood_batch(
                        self.pos[:, 0], self.pos[:, 1], obs,
                        current_spread_m=spread_prior)

            log_w -= log_w.max()
            w = np.exp(log_w)
            s = w.sum()
            if s < 1e-30:
                w = np.full_like(w, 1.0 / N)
            else:
                w /= s
            self.w = w

        # 6. Resample if degenerate
        n_eff = 1.0 / (self.w ** 2).sum()
        if n_eff < cfg.resample_threshold * cfg.n_particles:
            self._systematic_resample()
            # Boost regularization on AIS-fusion steps to preserve diversity.
            reg_scale = 3.0 if ais_observations else 1.0
            reg_m = cfg.regularization_m * reg_scale
            reg_lat = reg_m / METERS_PER_DEG_LAT
            reg_lon = reg_m / m_per_lon
            self.pos[:, 0] += self.rng.normal(0.0, reg_lat, cfg.n_particles)
            self.pos[:, 1] += self.rng.normal(0.0, reg_lon, cfg.n_particles)

        # 7. Estimate + spread
        est_lat = float(np.sum(self.pos[:, 0] * self.w))
        est_lon = float(np.sum(self.pos[:, 1] * self.w))
        dn = (self.pos[:, 0] - est_lat) * METERS_PER_DEG_LAT
        de = (self.pos[:, 1] - est_lon) * meters_per_deg_lon(est_lat)
        spread_m = float(math.sqrt(np.sum(self.w * (dn ** 2 + de ** 2))))

        heavy = self.w > (2.0 / cfg.n_particles)
        in_primary = np.sum(heavy & (np.sqrt(dn ** 2 + de ** 2) < spread_m))
        multimodal = (in_primary / cfg.n_particles) < 0.4
        healthy = (n_eff / cfg.n_particles > 0.1 and
                   spread_m < cfg.init_spread_m * 1.5 and
                   not multimodal)
        self._last_spread_m = spread_m
        return CNNFilterEstimate(est_lat, est_lon, spread_m, n_eff,
                                 healthy, multimodal)

    def _systematic_resample(self) -> None:
        N = self.cfg.n_particles
        u = (self.rng.uniform(0, 1) + np.arange(N)) / N
        cumsum = np.cumsum(self.w)
        idx = np.searchsorted(cumsum, u)
        idx = np.clip(idx, 0, N - 1)
        self.pos = self.pos[idx].copy()
        self.w = np.full(N, 1.0 / N)


# ===========================================================================
# Demo — head-to-head vs. bootstrap and patch
# ===========================================================================

def _demo():
    import math, random, statistics
    from meridian_orca.bathy_match import (BathyMatch, BathyMatchConfig,
        RealisticChart, GridChart)
    from meridian_orca.bathy_patch import PatchMatchFilter, PatchFilterConfig

    chart = GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        -33.980, 151.190, -33.960, 151.210, resolution_m=5.0,
    )
    MODEL = r"D:\projects\meridian\tools\orca-bridge\tests\fixtures\bathy_matcher.pt"

    def trial(kind, seed, outlier=None):
        rng = random.Random(seed)
        lat = -33.970 + rng.uniform(-0.002, 0.002)
        lon = 151.198 + rng.uniform(-0.002, 0.002)
        h = rng.uniform(0, 2 * math.pi); s = rng.uniform(1.5, 2.5)
        vn, ve = s * math.cos(h), s * math.sin(h)
        ang = rng.uniform(0, 2 * math.pi)
        seed_lat = lat + 30 * math.cos(ang) / METERS_PER_DEG_LAT
        seed_lon = lon + 30 * math.sin(ang) / meters_per_deg_lon(lat)

        if kind == "bootstrap":
            cfg = BathyMatchConfig(n_particles=2000, init_spread_m=40.0,
                process_noise_m_per_sqrt_s=0.3, depth_noise_m=0.5,
                mcc_enabled=True, regularization_m=0.5, resample_threshold=0.6)
            pf = BathyMatch(chart, cfg, seed=seed)
        elif kind == "patch":
            cfg = PatchFilterConfig(n_particles=1000, init_spread_m=40.0,
                process_noise_m_per_sqrt_s=0.25, depth_noise_m=0.5,
                mcc_enabled=True, regularization_m=0.3, resample_threshold=0.5,
                patch_window_s=8.0, patch_max_samples=8)
            pf = PatchMatchFilter(chart, cfg, seed=seed)
        else:  # cnn
            cfg = CNNFilterConfig()
            pf = BathyCNNFilter(chart, MODEL, cfg, seed=seed)

        pf.initialize(seed_lat, seed_lon)
        errs = []
        for t in range(60):
            lat += vn / METERS_PER_DEG_LAT
            lon += ve / meters_per_deg_lon(lat)
            d = chart.depth(lat, lon)
            if math.isnan(d) or d < 0.5: break
            obs = d + rng.gauss(0, 0.3)
            if outlier and t % outlier == 0:
                obs += rng.choice([-3.0, 3.0])
            est = pf.step(vn, ve, 1.0, obs)
            err = math.hypot(
                (lat - est.lat) * METERS_PER_DEG_LAT,
                (lon - est.lon) * meters_per_deg_lon(lat),
            )
            errs.append(err)
        if len(errs) < 30: return None
        return statistics.median(errs[30:])

    print(f"{'Condition':<15} {'Bootstrap':>11} {'Patch':>11} {'CNN':>11}")
    for label, outlier in [("clean", None), ("outlier/10", 10), ("harsh/5", 5)]:
        b = [trial("bootstrap", 0x4000 + i, outlier=outlier) for i in range(8)]
        p = [trial("patch",     0x4000 + i, outlier=outlier) for i in range(8)]
        c = [trial("cnn",       0x4000 + i, outlier=outlier) for i in range(8)]
        b = [v for v in b if v]; p = [v for v in p if v]; c = [v for v in c if v]
        bm = statistics.median(b) if b else float('nan')
        pm = statistics.median(p) if p else float('nan')
        cm = statistics.median(c) if c else float('nan')
        print(f"{label:<15} {bm:9.1f}m  {pm:9.1f}m  {cm:9.1f}m")


if __name__ == "__main__":
    _demo()
