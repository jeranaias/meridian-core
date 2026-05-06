"""Patch-matching bathymetric particle filter.

Classical Stage-2 upgrade: instead of comparing single `depth_observation`
to single `chart_depth_at_particle`, compare a *time-windowed history* of
observations along the boat's recent track to the chart values *along
that same track for each particle*.

Why this works
--------------
A single depth reading is ambiguous — there are many locations with 5.2 m
depth. But a sequence "5.2, 5.7, 6.4, 6.1, 5.9" over 5 seconds of motion
is a *signature* — very few points on the chart match it.

This is the classical Bergman TCM (terrain contour matching) algorithm,
adapted for a particle filter. Each particle evaluates:

    for each depth_k in recent history:
        predicted_depth_k = chart(particle_pos - velocity * (t_now - t_k))
        weight *= likelihood(predicted_depth_k, observed_depth_k)

Dramatic improvement in feature-ambiguous areas (flat seabeds, symmetric
harbors, repeating shipping channels).

References
----------
- Bergman 1999, "Recursive Bayesian estimation"
- Anonsen & Hagen 2010, "An analysis of real-time TAN results from HUGIN"
- Zhang 2023 (RBPF + patch): ~2-3x tighter in flat-terrain conditions
"""
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple

import numpy as np

from .bathy_match import (
    BathymetricChart, GridChart, RealisticChart,
    METERS_PER_DEG_LAT, meters_per_deg_lon,
)


# ===========================================================================
# Depth history (time-stamped ring buffer)
# ===========================================================================

@dataclass
class DepthSample:
    t: float        # simulation time
    depth: float    # measured depth (m)
    # Position-delta back to the track-time point, relative to current
    # particle position at t_now. Populated at match time per particle.


class DepthHistory:
    """Ring buffer of recent (time, depth) measurements plus cumulative
    motion (so we can reconstruct the offset from now to past samples)."""

    def __init__(self, window_s: float = 10.0, max_samples: int = 200) -> None:
        self.window_s = window_s
        self.max_samples = max_samples
        # Each entry: (t, depth, cum_north_m, cum_east_m)
        # cum_* is meters traveled since the filter started — used as relative
        # displacement between samples.
        self._buf: Deque[Tuple[float, float, float, float]] = deque(maxlen=max_samples)
        self._cum_n = 0.0
        self._cum_e = 0.0

    def advance(self, dn_m: float, de_m: float) -> None:
        """Advance the cumulative-motion counter (called each step)."""
        self._cum_n += dn_m
        self._cum_e += de_m

    def push(self, t: float, depth: float) -> None:
        self._buf.append((t, depth, self._cum_n, self._cum_e))

    def offsets_to_now(self) -> List[Tuple[float, float, float]]:
        """For each sample in the window, return (depth_observed, dn_back, de_back)
        where dn_back/de_back are meters NORTH/EAST from the *current* position
        back to where that sample was taken.

        This assumes the boat's path is well-characterized by the cumulative
        motion counter (i.e., we're trusting the velocity integration between
        samples). This is accurate for short windows (< 10 s) even with
        imperfect velocity.
        """
        if not self._buf:
            return []
        now_cum_n, now_cum_e = self._cum_n, self._cum_e
        t_now = self._buf[-1][0]
        out = []
        for t, depth, cum_n, cum_e in self._buf:
            if t_now - t > self.window_s:
                continue
            # Offset from current pos back to this sample's pos (negative of motion)
            dn_back = -(now_cum_n - cum_n)
            de_back = -(now_cum_e - cum_e)
            out.append((depth, dn_back, de_back))
        return out


# ===========================================================================
# Patch-match particle filter
# ===========================================================================

@dataclass
class PatchFilterConfig:
    n_particles: int = 1000
    init_spread_m: float = 40.0
    process_noise_m_per_sqrt_s: float = 0.25
    depth_noise_m: float = 0.5
    regularization_m: float = 0.3
    resample_threshold: float = 0.5
    min_chart_depth: float = 0.3

    # Patch window
    patch_window_s: float = 8.0
    patch_max_samples: int = 8        # we subsample the history evenly

    # MCC (Maximum Correlation Entropy) outlier-robust weighting
    mcc_enabled: bool = True
    mcc_kernel_sigma_mult: float = 3.0


@dataclass
class PatchEstimate:
    lat: float
    lon: float
    spread_m: float
    n_effective: float
    healthy: bool
    multimodal: bool


class PatchMatchFilter:
    """Terrain-contour matching particle filter with numpy acceleration."""

    def __init__(self, chart: BathymetricChart,
                 cfg: Optional[PatchFilterConfig] = None,
                 seed: int = 0xB474) -> None:
        self.chart = chart
        self.cfg = cfg or PatchFilterConfig()
        self.rng = np.random.default_rng(seed)
        self.pos: Optional[np.ndarray] = None    # (N, 2)
        self.w: Optional[np.ndarray] = None      # (N,)
        self.history = DepthHistory(
            window_s=self.cfg.patch_window_s,
            max_samples=max(50, self.cfg.patch_max_samples * 10),
        )
        self._t = 0.0

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
             observed_depth_m: float) -> PatchEstimate:
        if self.pos is None:
            raise RuntimeError("call initialize() first")
        cfg = self.cfg
        self._t += dt_s

        # 1. Advance cumulative motion and push this sample
        dn_m = vel_n_mps * dt_s
        de_m = vel_e_mps * dt_s
        self.history.advance(dn_m, de_m)
        self.history.push(self._t, observed_depth_m)

        # 2. Propagate particles
        sigma = cfg.process_noise_m_per_sqrt_s * math.sqrt(dt_s)
        m_per_lat = METERS_PER_DEG_LAT
        m_per_lon = meters_per_deg_lon(float(self.pos[:, 0].mean()))
        noise = self.rng.normal(0.0, sigma, (cfg.n_particles, 2))
        self.pos[:, 0] += (dn_m + noise[:, 0]) / m_per_lat
        self.pos[:, 1] += (de_m + noise[:, 1]) / m_per_lon

        # 3. Build the patch: subsample history to `patch_max_samples`
        patch = self.history.offsets_to_now()
        if len(patch) > cfg.patch_max_samples:
            # Subsample evenly — oldest first, newest always included
            idx = np.linspace(0, len(patch) - 1, cfg.patch_max_samples).astype(int)
            patch = [patch[i] for i in idx]

        # 4. Score each particle by likelihood of the whole patch
        inv2var = 1.0 / (2.0 * cfg.depth_noise_m ** 2)
        sigma_c2 = (cfg.mcc_kernel_sigma_mult * cfg.depth_noise_m) ** 2
        log_w = np.zeros(cfg.n_particles)
        any_sample_scored = False
        for (obs_depth, dn_back, de_back) in patch:
            # For each particle, query chart at (particle_pos + back_offset)
            # back_offset converts meters to degree offset per particle's latitude
            lat_offset = dn_back / m_per_lat
            lon_offset = de_back / m_per_lon
            depths = np.empty(cfg.n_particles)
            for i in range(cfg.n_particles):
                try:
                    d = self.chart.depth(
                        self.pos[i, 0] + lat_offset,
                        self.pos[i, 1] + lon_offset,
                    )
                except Exception:
                    d = float('nan')
                depths[i] = d

            invalid = np.isnan(depths) | (depths < cfg.min_chart_depth)
            err2 = (depths - obs_depth) ** 2
            if cfg.mcc_enabled:
                kernel = np.exp(-err2 / (2.0 * sigma_c2))
                contribution = -err2 * inv2var * kernel
            else:
                contribution = -err2 * inv2var
            contribution[invalid] = -1e9
            log_w += contribution
            any_sample_scored = True

        if not any_sample_scored:
            # No history yet — uniform weights
            self.w[:] = 1.0 / cfg.n_particles
        else:
            # Normalize
            log_w -= log_w.max()
            w = np.exp(log_w)
            s = w.sum()
            if s < 1e-30:
                w = np.full_like(w, 1.0 / cfg.n_particles)
            else:
                w /= s
            self.w = w

        # 5. Resample if degenerate
        n_eff = 1.0 / (self.w ** 2).sum()
        if n_eff < cfg.resample_threshold * cfg.n_particles:
            self._systematic_resample()
            # Regularize
            reg_lat = cfg.regularization_m / m_per_lat
            reg_lon = cfg.regularization_m / m_per_lon
            self.pos[:, 0] += self.rng.normal(0.0, reg_lat, cfg.n_particles)
            self.pos[:, 1] += self.rng.normal(0.0, reg_lon, cfg.n_particles)

        # 6. Estimate
        est_lat = float(np.sum(self.pos[:, 0] * self.w))
        est_lon = float(np.sum(self.pos[:, 1] * self.w))
        dn = (self.pos[:, 0] - est_lat) * m_per_lat
        de = (self.pos[:, 1] - est_lon) * meters_per_deg_lon(est_lat)
        spread_m = float(math.sqrt(np.sum(self.w * (dn ** 2 + de ** 2))))

        heavy = self.w > (2.0 / cfg.n_particles)
        in_primary = np.sum(heavy & (np.sqrt(dn ** 2 + de ** 2) < spread_m))
        multimodal = (in_primary / cfg.n_particles) < 0.4
        healthy = (n_eff / cfg.n_particles > 0.1 and
                   spread_m < cfg.init_spread_m * 1.5 and
                   not multimodal)

        return PatchEstimate(est_lat, est_lon, spread_m, n_eff,
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
# Demo
# ===========================================================================

def _demo(verbose: bool = True) -> float:
    import random as _r
    chart = GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        -33.980, 151.190, -33.960, 151.210, resolution_m=5.0,
    )
    cfg = PatchFilterConfig(n_particles=1000, init_spread_m=40.0,
                            patch_window_s=8.0, patch_max_samples=8)
    pf = PatchMatchFilter(chart, cfg)

    true_lat, true_lon = -33.970, 151.198
    pf.initialize(true_lat + 0.0002, true_lon + 0.0001)

    vel_n_truth, vel_e_truth = 0.0, 2.0
    dt = 1.0
    total_err = 0.0
    samples = 0
    rng = _r.Random(0xCAFE)
    if verbose:
        print(f"{'t':>4} {'err':>6} {'spread':>7} {'n_eff':>6}")
    for t in range(60):
        true_lat += vel_n_truth * dt / METERS_PER_DEG_LAT
        true_lon += vel_e_truth * dt / meters_per_deg_lon(true_lat)
        d = chart.depth(true_lat, true_lon) + rng.gauss(0, 0.3)
        if t % 13 == 0:
            d += rng.choice([-3.0, 3.0])
        est = pf.step(vel_n_truth, vel_e_truth, dt, d)
        err = math.hypot(
            (true_lat - est.lat) * METERS_PER_DEG_LAT,
            (true_lon - est.lon) * meters_per_deg_lon(true_lat),
        )
        if t > 15:
            total_err += err
            samples += 1
        if verbose:
            print(f"{t:4d} {err:6.1f} {est.spread_m:7.1f} {est.n_effective:6.0f}")
    mean = total_err / max(1, samples)
    if verbose:
        print(f"\nmean steady-state: {mean:.2f} m  (patch-match, 1000 particles, 8-sample patch)")
    return mean


if __name__ == "__main__":
    _demo()
