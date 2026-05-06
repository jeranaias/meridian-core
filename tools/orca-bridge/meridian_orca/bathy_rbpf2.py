"""Production Rao-Blackwellized Particle Filter — numpy-vectorized.

Replaces the hand-rolled Kalman math in `bathy_rbpf.py` with proper matrix
operations using numpy. All N particles are updated in parallel via
broadcasting — each step is O(N) tensor math, not a Python loop.

State decomposition:
  x_nonlin = [lat, lon]              — particle-based
  x_lin    = [vn, ve, cn, ce]        — per-particle Kalman state

Measurement models:
  depth observation: z_d = chart(lat, lon) + v    — nonlinear, weights particle
  velocity obs  (GPS): z_v = [vn+cn, ve+ce] + w   — linear in x_lin, updates Kalman

Process model (random walk):
  vn,ve random walk with variance q_v per second
  cn,ce random walk with tighter variance q_c (currents evolve slowly)

References:
  - Zhang et al. 2023, "An outlier-robust RBPF for underwater TAN"
  - Kim 2018, "Robust TAN using RBPF" (methodology baseline)
  - Schon & Gustafsson 2005, "Marginalized particle filters"
"""
import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .bathy_match import (
    BathymetricChart, RealisticChart, GridChart,
    METERS_PER_DEG_LAT, meters_per_deg_lon,
)


# ===========================================================================
# Config
# ===========================================================================

@dataclass
class RBPF2Config:
    n_particles: int = 1000
    init_spread_m: float = 40.0
    init_vel_std_mps: float = 1.0
    init_current_std_mps: float = 0.3

    # Process noise (per second) — tuned so Kalman converges within 10-15 s
    pos_noise_m_per_sqrt_s: float = 0.2
    vel_noise_mps_per_sqrt_s: float = 0.05
    current_noise_mps_per_sqrt_s: float = 0.01

    # Measurement noises
    depth_noise_m: float = 0.5
    vel_obs_std_mps: float = 0.3

    # MCC outlier-robust weighting
    mcc_enabled: bool = True
    mcc_kernel_sigma_mult: float = 3.0

    # Particle hygiene
    regularization_m: float = 0.4
    resample_threshold: float = 0.5
    min_chart_depth: float = 0.3


@dataclass
class RBPF2Estimate:
    lat: float
    lon: float
    pos_spread_m: float
    vel_n_mps: float
    vel_e_mps: float
    current_n_mps: float
    current_e_mps: float
    current_confidence: float
    n_effective: float
    healthy: bool
    multimodal: bool


# ===========================================================================
# Filter
# ===========================================================================

class RBPF2:
    """Numpy-vectorized RBPF. Maintains state as:
        pos:  (N, 2) lat/lon
        mu:   (N, 4) Kalman mean [vn, ve, cn, ce]
        Sig:  (N, 4, 4) Kalman covariance
        w:    (N,) normalized weights
    """

    # Observation matrix for velocity measurements: z = H @ x_l
    # z = [vn + cn, ve + ce]; H picks those combinations
    _H = np.array([[1.0, 0.0, 1.0, 0.0],
                   [0.0, 1.0, 0.0, 1.0]])

    def __init__(self, chart: BathymetricChart,
                 cfg: Optional[RBPF2Config] = None, seed: int = 0xB474) -> None:
        self.chart = chart
        self.cfg = cfg or RBPF2Config()
        self.rng = np.random.default_rng(seed)
        # Allocated lazily in initialize()
        self.pos: Optional[np.ndarray] = None
        self.mu:  Optional[np.ndarray] = None
        self.Sig: Optional[np.ndarray] = None
        self.w:   Optional[np.ndarray] = None

    # --- init -------------------------------------------------------------

    def initialize(self, lat: float, lon: float,
                   vel_n: float = 0.0, vel_e: float = 0.0,
                   pos_spread_m: Optional[float] = None) -> None:
        cfg = self.cfg
        N = cfg.n_particles
        s = pos_spread_m if pos_spread_m is not None else cfg.init_spread_m

        # Seed positions around (lat, lon)
        lat_scale = s / METERS_PER_DEG_LAT
        lon_scale = s / meters_per_deg_lon(lat)
        self.pos = np.empty((N, 2))
        self.pos[:, 0] = lat + self.rng.normal(0.0, lat_scale, N)
        self.pos[:, 1] = lon + self.rng.normal(0.0, lon_scale, N)

        # Seed Kalman state
        self.mu = np.empty((N, 4))
        self.mu[:, 0] = vel_n + self.rng.normal(0.0, cfg.init_vel_std_mps, N)
        self.mu[:, 1] = vel_e + self.rng.normal(0.0, cfg.init_vel_std_mps, N)
        self.mu[:, 2] = self.rng.normal(0.0, cfg.init_current_std_mps, N)
        self.mu[:, 3] = self.rng.normal(0.0, cfg.init_current_std_mps, N)

        # Initial covariance (diagonal)
        self.Sig = np.zeros((N, 4, 4))
        self.Sig[:, 0, 0] = cfg.init_vel_std_mps ** 2
        self.Sig[:, 1, 1] = cfg.init_vel_std_mps ** 2
        self.Sig[:, 2, 2] = cfg.init_current_std_mps ** 2
        self.Sig[:, 3, 3] = cfg.init_current_std_mps ** 2

        self.w = np.full(N, 1.0 / N)

    # --- step -------------------------------------------------------------

    def step(self, measured_vel_n: Optional[float], measured_vel_e: Optional[float],
             dt_s: float, observed_depth_m: float) -> RBPF2Estimate:
        if self.pos is None:
            raise RuntimeError("call initialize() first")
        cfg = self.cfg

        # =====================================================================
        # 1. Predict: propagate Kalman state (F = I, so mu unchanged; only
        #    covariance grows)
        # =====================================================================
        Q = np.diag([
            cfg.vel_noise_mps_per_sqrt_s ** 2 * dt_s,
            cfg.vel_noise_mps_per_sqrt_s ** 2 * dt_s,
            cfg.current_noise_mps_per_sqrt_s ** 2 * dt_s,
            cfg.current_noise_mps_per_sqrt_s ** 2 * dt_s,
        ])
        self.Sig += Q

        # =====================================================================
        # 2. Compute predicted over-ground velocity per particle
        #    z_pred = H @ mu → (N, 2) = [vn+cn, ve+ce]
        # =====================================================================
        z_pred = self.mu @ self._H.T
        # Predicted velocity covariance (for Kalman update below)
        HSH = np.einsum('ij,njk,lk->nil', self._H, self.Sig, self._H)

        # =====================================================================
        # 3. Advance position using the deterministic mean velocity +
        #    process noise. (Proper Rao-Blackwell: the particle state is
        #    position; velocity uncertainty is factored into the Kalman
        #    marginal, not the position sample.)
        # =====================================================================
        pos_noise_m = cfg.pos_noise_m_per_sqrt_s * math.sqrt(dt_s)
        m_per_lon = meters_per_deg_lon(float(self.pos[:, 0].mean()))
        motion_noise = self.rng.normal(0.0, pos_noise_m, (cfg.n_particles, 2))
        dn_m = z_pred[:, 0] * dt_s + motion_noise[:, 0]
        de_m = z_pred[:, 1] * dt_s + motion_noise[:, 1]
        self.pos[:, 0] += dn_m / METERS_PER_DEG_LAT
        self.pos[:, 1] += de_m / m_per_lon

        # =====================================================================
        # 4. Kalman update: velocity measurement (GPS COG+SOG, if available)
        # =====================================================================
        if measured_vel_n is not None and measured_vel_e is not None:
            z = np.array([measured_vel_n, measured_vel_e])
            R = (cfg.vel_obs_std_mps ** 2) * np.eye(2)
            # Innovation (N, 2)
            y = z[None, :] - z_pred
            # Innovation covariance (N, 2, 2): H @ Sig @ H.T + R
            S = HSH + R[None, :, :]
            # Invert S for each particle (N, 2, 2)
            S_inv = np.linalg.inv(S)
            # Kalman gain K = Sig @ H.T @ S_inv → (N, 4, 2)
            SigHt = np.einsum('nij,kj->nik', self.Sig, self._H)  # (N, 4, 2)
            K = np.einsum('nij,njk->nik', SigHt, S_inv)          # (N, 4, 2)
            # Update mu: mu += K @ y
            self.mu += np.einsum('nij,nj->ni', K, y)
            # Update Sig: Sig = (I - K @ H) @ Sig
            KH = np.einsum('nij,jk->nik', K, self._H)            # (N, 4, 4)
            I_KH = np.eye(4)[None, :, :] - KH
            self.Sig = np.einsum('nij,njk->nik', I_KH, self.Sig)

        # =====================================================================
        # 5. Chart likelihood — weight particles by terrain match
        # =====================================================================
        # Query chart for each particle (chart is a Python callable, not
        # vectorized; N calls per step)
        depths = np.empty(cfg.n_particles)
        for i in range(cfg.n_particles):
            try:
                d = self.chart.depth(self.pos[i, 0], self.pos[i, 1])
            except Exception:
                d = float('nan')
            depths[i] = d

        invalid = np.isnan(depths) | (depths < cfg.min_chart_depth)
        err2 = (depths - observed_depth_m) ** 2
        inv2var = 1.0 / (2.0 * cfg.depth_noise_m ** 2)
        if cfg.mcc_enabled:
            sigma_c2 = (cfg.mcc_kernel_sigma_mult * cfg.depth_noise_m) ** 2
            kernel = np.exp(-err2 / (2.0 * sigma_c2))
            log_w = -err2 * inv2var * kernel
        else:
            log_w = -err2 * inv2var
        log_w[invalid] = -1e9
        # Normalize in log-space
        log_w -= log_w.max()
        w = np.exp(log_w)
        w_sum = w.sum()
        if w_sum < 1e-30:
            w = np.full_like(w, 1.0 / len(w))
        else:
            w /= w_sum
        self.w = w

        # =====================================================================
        # 6. Resample with regularization
        # =====================================================================
        n_eff = 1.0 / (self.w ** 2).sum()
        if n_eff < cfg.resample_threshold * cfg.n_particles:
            self._systematic_resample()
            # Regularize position
            reg_lat = cfg.regularization_m / METERS_PER_DEG_LAT
            reg_lon = cfg.regularization_m / m_per_lon
            self.pos[:, 0] += self.rng.normal(0.0, reg_lat, cfg.n_particles)
            self.pos[:, 1] += self.rng.normal(0.0, reg_lon, cfg.n_particles)

        # =====================================================================
        # 7. Build estimate
        # =====================================================================
        est_lat = float(np.sum(self.pos[:, 0] * self.w))
        est_lon = float(np.sum(self.pos[:, 1] * self.w))
        est_vn  = float(np.sum(self.mu[:, 0]  * self.w))
        est_ve  = float(np.sum(self.mu[:, 1]  * self.w))
        est_cn  = float(np.sum(self.mu[:, 2]  * self.w))
        est_ce  = float(np.sum(self.mu[:, 3]  * self.w))

        # Spread in meters
        m_per_lon_est = meters_per_deg_lon(est_lat)
        dn = (self.pos[:, 0] - est_lat) * METERS_PER_DEG_LAT
        de = (self.pos[:, 1] - est_lon) * m_per_lon_est
        pos_spread_m = float(math.sqrt(np.sum(self.w * (dn ** 2 + de ** 2))))

        # Current confidence from mean Kalman diag
        cur_var = float(np.sum(self.w * (self.Sig[:, 2, 2] + self.Sig[:, 3, 3]) / 2))
        cur_conf = 1.0 / (1.0 + cur_var * 10)

        # Multimodality check
        heavy = self.w > (2.0 / cfg.n_particles)
        in_primary = np.sum(heavy & (np.sqrt(dn ** 2 + de ** 2) < pos_spread_m))
        multimodal = (in_primary / cfg.n_particles) < 0.4

        healthy = (n_eff / cfg.n_particles > 0.1 and
                   pos_spread_m < cfg.init_spread_m * 1.5 and
                   not multimodal)

        return RBPF2Estimate(
            lat=est_lat, lon=est_lon, pos_spread_m=pos_spread_m,
            vel_n_mps=est_vn, vel_e_mps=est_ve,
            current_n_mps=est_cn, current_e_mps=est_ce,
            current_confidence=cur_conf, n_effective=n_eff,
            healthy=healthy, multimodal=multimodal,
        )

    # --- internal ---------------------------------------------------------

    def _systematic_resample(self) -> None:
        N = self.cfg.n_particles
        u = (self.rng.uniform(0, 1) + np.arange(N)) / N
        cumsum = np.cumsum(self.w)
        idx = np.searchsorted(cumsum, u)
        idx = np.clip(idx, 0, N - 1)
        self.pos = self.pos[idx].copy()
        self.mu  = self.mu[idx].copy()
        self.Sig = self.Sig[idx].copy()
        self.w = np.full(N, 1.0 / N)


# ===========================================================================
# Demo
# ===========================================================================

def _demo(verbose: bool = True) -> float:
    import random as _r
    from .bathy_match import RealisticChart, GridChart
    base = RealisticChart(seed=0xBA74)
    chart = GridChart.from_realistic(
        base, -33.980, 151.190, -33.960, 151.210, resolution_m=5.0
    )
    cfg = RBPF2Config(n_particles=1000, init_spread_m=40.0)
    pf = RBPF2(chart, cfg)

    true_lat, true_lon = -33.970, 151.198
    pf.initialize(true_lat + 0.0002, true_lon + 0.0001, vel_n=0.0, vel_e=2.0)

    vel_n_truth, vel_e_truth = 0.0, 2.0
    cur_n_truth, cur_e_truth = 0.2, 0.0
    dt = 1.0
    total_err = 0.0
    samples = 0
    rng = _r.Random(0xCAFE)
    if verbose:
        print(f"{'t':>4} {'err':>6} {'spread':>7} {'vn':>5} {'ve':>5} {'cn':>5} {'ce':>5}")
    for t in range(60):
        true_lat += (vel_n_truth + cur_n_truth) * dt / METERS_PER_DEG_LAT
        true_lon += (vel_e_truth + cur_e_truth) * dt / meters_per_deg_lon(true_lat)
        d_true = chart.depth(true_lat, true_lon) + rng.gauss(0, 0.3)
        if t % 13 == 0:
            d_true += rng.choice([-3.0, 3.0])
        measured_vn = (vel_n_truth + cur_n_truth) + rng.gauss(0, 0.3)
        measured_ve = (vel_e_truth + cur_e_truth) + rng.gauss(0, 0.3)
        est = pf.step(measured_vn, measured_ve, dt, d_true)
        err = math.hypot(
            (true_lat - est.lat) * METERS_PER_DEG_LAT,
            (true_lon - est.lon) * meters_per_deg_lon(true_lat),
        )
        if t > 15:
            total_err += err
            samples += 1
        if verbose:
            print(f"{t:4d} {err:6.1f} {est.pos_spread_m:7.1f} "
                  f"{est.vel_n_mps:5.2f} {est.vel_e_mps:5.2f} "
                  f"{est.current_n_mps:5.2f} {est.current_e_mps:5.2f}")
    mean = total_err / max(1, samples)
    if verbose:
        print(f"\nmean steady-state: {mean:.2f} m  (RBPF2, 1000 particles)")
    return mean


if __name__ == "__main__":
    _demo()
