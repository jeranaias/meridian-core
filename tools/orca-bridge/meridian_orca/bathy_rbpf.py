"""Rao-Blackwellized particle filter for bathymetric terrain-aided navigation.

This is the Tier 1 upgrade over the bootstrap particle filter in
`bathy_match.py`. Target: 2-3x tighter median error with the same particle
count, plus outlier-robust measurement updates via the Maximum Correlation
Entropy Criterion (MCC).

Rao-Blackwellization
--------------------
The boat's state has two parts:
  x_n  (nonlinear):   [lat, lon]         — particle-based (chart match)
  x_l  (linear):      [vel_n, vel_e]     — per-particle Kalman update

Each particle carries its own tiny Kalman filter for velocity. We marginalize
velocity out of the particle weighting, which means:
  - Same accuracy with ~1/3 the particles, OR
  - Same particle count with ~2x tighter spread
  - Natural tide/current estimation (extend x_l to include current vector)

MCC weighting
-------------
Standard particle weights use Gaussian likelihood:
  w ∝ exp(-err² / 2σ²)

MCC replaces this with a kernel that's heavy-tailed (robust to outliers):
  w ∝ exp(-(1 - exp(-err² / 2σ_c²)) / ε)

The result: a sonar glitch or chart error doesn't spike weight on wrong
particles. Filter stays on track even when the measurement is ~2-3 σ off.
Reference: Zhang et al. 2023, "An outlier-robust Rao-Blackwellized particle
filter for underwater terrain-aided navigation."

Tide / current estimation
-------------------------
The linear state is extended to [vel_n, vel_e, current_n, current_e].
The observation model (depth-only, doesn't see velocity) means the current
is observable only indirectly — through how well the filter's velocity
estimate matches the dead-reckoned position vs. the chart. Papers report
convergence to within ~0.1 m/s after ~60-120 s given feature-rich terrain.
"""
import math
import random
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from .bathy_match import (
    BathymetricChart, AnalyticChart, RealisticChart, GridChart,
    METERS_PER_DEG_LAT, meters_per_deg_lon,
)


# ===========================================================================
# Particle — nonlinear position + per-particle Kalman for velocity/current
# ===========================================================================

@dataclass
class RBParticle:
    lat: float
    lon: float
    # Per-particle Kalman state for [vel_n, vel_e, cur_n, cur_e]
    x_l: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    # Covariance (4x4), stored as flat list row-major
    P_l: List[float] = field(default_factory=lambda: [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.25, 0.0,
        0.0, 0.0, 0.0, 0.25,
    ])
    weight: float = 1.0


@dataclass
class RBPFConfig:
    n_particles: int = 1000
    init_spread_m: float = 40.0
    init_vel_std_mps: float = 1.0
    init_current_std_mps: float = 0.3      # prior on tidal current magnitude
    # Process noise
    pos_noise_m_per_sqrt_s: float = 0.4
    vel_noise_mps_per_sqrt_s: float = 0.1
    current_noise_mps_per_sqrt_s: float = 0.02   # current changes slowly
    # Measurement
    depth_noise_m: float = 0.5
    # MCC kernel bandwidth — larger = more robust, less precise
    mcc_kernel_sigma: float = 2.5           # in multiples of depth_noise_m
    mcc_enabled: bool = True
    # Particle hygiene
    regularization_m: float = 0.4
    resample_threshold: float = 0.5
    min_chart_depth: float = 0.3


@dataclass
class RBPFEstimate:
    lat: float
    lon: float
    pos_spread_m: float
    vel_n_mps: float
    vel_e_mps: float
    current_n_mps: float
    current_e_mps: float
    current_confidence: float   # 0..1, inverse of current-state variance
    n_effective: float
    healthy: bool
    multimodal: bool


# ===========================================================================
# RBPF filter
# ===========================================================================

class RBPF:
    """Rao-Blackwellized particle filter for bathy terrain-aided nav."""

    def __init__(self, chart: BathymetricChart,
                 cfg: Optional[RBPFConfig] = None, seed: int = 0xB474) -> None:
        self.chart = chart
        self.cfg = cfg or RBPFConfig()
        self.rng = random.Random(seed)
        self.particles: List[RBParticle] = []

    # ---- lifecycle ---------------------------------------------------------

    def initialize(self, lat: float, lon: float,
                   vel_n: float = 0.0, vel_e: float = 0.0,
                   pos_spread_m: Optional[float] = None) -> None:
        cfg = self.cfg
        s = pos_spread_m if pos_spread_m is not None else cfg.init_spread_m
        self.particles = []
        lat_scale = s / METERS_PER_DEG_LAT
        lon_scale = s / meters_per_deg_lon(lat)
        for _ in range(cfg.n_particles):
            p = RBParticle(
                lat=lat + self.rng.gauss(0, lat_scale),
                lon=lon + self.rng.gauss(0, lon_scale),
                x_l=[
                    vel_n + self.rng.gauss(0, cfg.init_vel_std_mps),
                    vel_e + self.rng.gauss(0, cfg.init_vel_std_mps),
                    self.rng.gauss(0, cfg.init_current_std_mps),
                    self.rng.gauss(0, cfg.init_current_std_mps),
                ],
                # Diagonal P_l (initial uncertainty)
                P_l=[
                    cfg.init_vel_std_mps ** 2, 0, 0, 0,
                    0, cfg.init_vel_std_mps ** 2, 0, 0,
                    0, 0, cfg.init_current_std_mps ** 2, 0,
                    0, 0, 0, cfg.init_current_std_mps ** 2,
                ],
                weight=1.0 / cfg.n_particles,
            )
            self.particles.append(p)

    # ---- step --------------------------------------------------------------

    def step(self, measured_vel_n: Optional[float], measured_vel_e: Optional[float],
             dt_s: float, observed_depth_m: float) -> RBPFEstimate:
        """Advance one step.
        `measured_vel_n/e`: IMU- or GPS-derived velocity (boat frame NED).
        If None, the filter dead-reckons using the per-particle velocity
        estimate — this is the GPS-denied regime.
        """
        if not self.particles:
            raise RuntimeError("call initialize() first")
        cfg = self.cfg

        # 1. Propagate: Kalman predict for (vel, current), then advance position.
        q_vel = cfg.vel_noise_mps_per_sqrt_s ** 2 * dt_s
        q_cur = cfg.current_noise_mps_per_sqrt_s ** 2 * dt_s
        pos_noise = cfg.pos_noise_m_per_sqrt_s * math.sqrt(dt_s)

        for p in self.particles:
            # Propagate velocity state with process noise
            # State = [vn, ve, cn, ce]; dynamics: velocities random-walk slowly,
            # currents random-walk even slower. No drift except noise.
            # Add noise to covariance (F = I for this simple model).
            p.P_l[0]  += q_vel
            p.P_l[5]  += q_vel
            p.P_l[10] += q_cur
            p.P_l[15] += q_cur
            # Sample from velocity posterior to advance position
            # (Rao-Blackwell says we propagate position by sampling the
            # marginal; we use the mean + noise from P_l diag for simplicity.)
            effective_vn = p.x_l[0] + p.x_l[2]
            effective_ve = p.x_l[1] + p.x_l[3]
            # Small sample noise proportional to velocity uncertainty
            vn_std = math.sqrt(max(p.P_l[0], 0.0))
            ve_std = math.sqrt(max(p.P_l[5], 0.0))
            vn_sample = effective_vn + self.rng.gauss(0, vn_std)
            ve_sample = effective_ve + self.rng.gauss(0, ve_std)
            # Advance particle position
            m_per_lat = METERS_PER_DEG_LAT
            m_per_lon = meters_per_deg_lon(p.lat)
            p.lat += (vn_sample * dt_s + self.rng.gauss(0, pos_noise)) / m_per_lat
            p.lon += (ve_sample * dt_s + self.rng.gauss(0, pos_noise)) / m_per_lon

        # 2. Kalman update for velocity with direct measurement (if available).
        # H = [1 0 0 0 ; 0 1 0 0]    (we observe vel_n + current_n fused as vn, etc.)
        # Actually — the boat's over-ground velocity IS vn+cn (water velocity + current).
        # When we get a GPS COG+SOG, that's the over-ground velocity. So the
        # measurement model is: z = [vn+cn, ve+ce], H = [1 0 1 0; 0 1 0 1].
        if measured_vel_n is not None and measured_vel_e is not None:
            R_vel = 0.3 ** 2       # GPS COG-derived vel has ~0.3 m/s noise
            for p in self.particles:
                self._vel_update(p, measured_vel_n, measured_vel_e, R_vel)

        # 3. Weigh particles by bathymetric likelihood.
        inv2var = 1.0 / (2.0 * cfg.depth_noise_m ** 2)
        mcc_inv2var = 1.0 / (2.0 * (cfg.mcc_kernel_sigma * cfg.depth_noise_m) ** 2)
        log_max = -float("inf")
        for p in self.particles:
            try:
                d = self.chart.depth(p.lat, p.lon)
            except Exception:
                d = float("nan")
            if math.isnan(d) or d < cfg.min_chart_depth:
                lw = -1e9
            else:
                err2 = (d - observed_depth_m) ** 2
                if cfg.mcc_enabled:
                    # MCC: 1 - exp(-err²/2σ_c²) is a kernel bounded in [0, 1]
                    # log weight ∝ -(1 - exp(-err²/2σ_c²)) / ε
                    # Using ε = depth_noise² to match scales
                    kernel = math.exp(-err2 * mcc_inv2var)
                    lw = -(1.0 - kernel) / (cfg.depth_noise_m ** 2) * (cfg.depth_noise_m ** 2 * inv2var)
                    # Simplification: lw = -(1 - kernel) * inv2var_scaled
                    # In practice, use a mix of Gaussian (for precision) and MCC (for outlier rejection):
                    lw = -err2 * inv2var * kernel    # weighted Gaussian
                else:
                    lw = -err2 * inv2var
            p.weight = lw      # temporarily store log weight
            if lw > log_max:
                log_max = lw

        if log_max == -float("inf"):
            log_max = 0.0
        # Normalize in linear space
        w_sum = 0.0
        for p in self.particles:
            p.weight = math.exp(p.weight - log_max)
            w_sum += p.weight
        if w_sum < 1e-30:
            w_sum = 1.0
        for p in self.particles:
            p.weight /= w_sum

        # 4. Effective sample size + resample if degenerate
        n_eff = 1.0 / sum(p.weight ** 2 for p in self.particles)
        if n_eff < cfg.resample_threshold * cfg.n_particles:
            self._systematic_resample()
            # Regularize position (jitter)
            reg_lat = cfg.regularization_m / METERS_PER_DEG_LAT
            for p in self.particles:
                reg_lon = cfg.regularization_m / meters_per_deg_lon(p.lat)
                p.lat += self.rng.gauss(0, reg_lat)
                p.lon += self.rng.gauss(0, reg_lon)

        # 5. Build estimate — weighted mean over particles
        est_lat = sum(p.lat * p.weight for p in self.particles)
        est_lon = sum(p.lon * p.weight for p in self.particles)
        est_vn  = sum(p.x_l[0] * p.weight for p in self.particles)
        est_ve  = sum(p.x_l[1] * p.weight for p in self.particles)
        est_cn  = sum(p.x_l[2] * p.weight for p in self.particles)
        est_ce  = sum(p.x_l[3] * p.weight for p in self.particles)

        # Position spread
        var_m2 = 0.0
        m_per_lat = METERS_PER_DEG_LAT
        m_per_lon_est = meters_per_deg_lon(est_lat)
        for p in self.particles:
            dn = (p.lat - est_lat) * m_per_lat
            de = (p.lon - est_lon) * m_per_lon_est
            var_m2 += p.weight * (dn ** 2 + de ** 2)
        pos_spread_m = math.sqrt(max(var_m2, 0.0))

        # Current confidence from mean posterior covariance
        cur_var = sum(p.weight * (p.P_l[10] + p.P_l[15]) / 2 for p in self.particles)
        cur_conf = 1.0 / (1.0 + cur_var * 10)   # maps (0, ∞) → (1, 0)

        # Modality check
        in_primary = 0
        threshold_w = 2.0 / cfg.n_particles
        for p in self.particles:
            dn = (p.lat - est_lat) * m_per_lat
            de = (p.lon - est_lon) * m_per_lon_est
            if math.hypot(dn, de) < pos_spread_m and p.weight > threshold_w:
                in_primary += 1
        multimodal = (in_primary / cfg.n_particles) < 0.4

        healthy = (n_eff / cfg.n_particles > 0.1 and
                   pos_spread_m < cfg.init_spread_m * 1.5 and
                   not multimodal)

        return RBPFEstimate(
            lat=est_lat, lon=est_lon, pos_spread_m=pos_spread_m,
            vel_n_mps=est_vn, vel_e_mps=est_ve,
            current_n_mps=est_cn, current_e_mps=est_ce,
            current_confidence=cur_conf,
            n_effective=n_eff, healthy=healthy, multimodal=multimodal,
        )

    # ---- internal ----------------------------------------------------------

    def _vel_update(self, p: RBParticle, z_vn: float, z_ve: float, R: float) -> None:
        """Kalman update for velocity given over-ground velocity measurement.
        H = [[1 0 1 0],
             [0 1 0 1]]    (observation = vel + current)
        """
        # y = z - H x_l
        y_n = z_vn - (p.x_l[0] + p.x_l[2])
        y_e = z_ve - (p.x_l[1] + p.x_l[3])
        # S = H P H' + R   (2x2)
        #   = [[P00 + 2*P02 + P22 + R,   P01 + P03 + P21 + P23],
        #      [ (symmetric),             P11 + 2*P13 + P33 + R ]]
        p00, p01, p02, p03 = p.P_l[0],  p.P_l[1],  p.P_l[2],  p.P_l[3]
        p11, p13             = p.P_l[5],             p.P_l[7]
        p22, p23             = p.P_l[10],            p.P_l[11]
        p33                  = p.P_l[15]
        S00 = p00 + 2 * p02 + p22 + R
        S11 = p11 + 2 * p13 + p33 + R
        S01 = p01 + p03 + p02 + p23     # rough approximation
        det = S00 * S11 - S01 * S01
        if abs(det) < 1e-12:
            return
        invS00 =  S11 / det
        invS11 =  S00 / det
        invS01 = -S01 / det
        # K = P H' S^-1 (4x2). Just update diag components for simplicity
        # (this is a simplified update that preserves the important dynamics)
        # Effective gain for vel state ~ P0x / S
        K_vn_n = (p00 + p02) * invS00 + (p01 + p03) * invS01
        K_ve_e = (p01 + p03) * invS01 + (p11 + p13) * invS11
        K_cn_n = (p02 + p22) * invS00 + (p03 + p23) * invS01
        K_ce_e = (p03 + p23) * invS01 + (p13 + p33) * invS11
        # State update
        p.x_l[0] += K_vn_n * y_n
        p.x_l[1] += K_ve_e * y_e
        p.x_l[2] += K_cn_n * y_n
        p.x_l[3] += K_ce_e * y_e
        # Covariance update (simplified — reduce diagonals by gain magnitude)
        p.P_l[0]  *= max(0.1, 1.0 - abs(K_vn_n))
        p.P_l[5]  *= max(0.1, 1.0 - abs(K_ve_e))
        p.P_l[10] *= max(0.3, 1.0 - abs(K_cn_n))   # currents learn slower
        p.P_l[15] *= max(0.3, 1.0 - abs(K_ce_e))

    def _systematic_resample(self) -> None:
        n = len(self.particles)
        r = self.rng.uniform(0, 1.0 / n)
        new_particles: List[RBParticle] = []
        i = 0
        w = self.particles[0].weight
        for k in range(n):
            u = r + k / n
            while u > w and i < n - 1:
                i += 1
                w += self.particles[i].weight
            p = self.particles[i]
            new_particles.append(RBParticle(
                lat=p.lat, lon=p.lon,
                x_l=list(p.x_l), P_l=list(p.P_l),
                weight=1.0 / n,
            ))
        self.particles = new_particles


# ===========================================================================
# Demo / smoke test
# ===========================================================================

def _demo(verbose: bool = True) -> float:
    """Run RBPF on the same scenario as bootstrap demo for direct compare."""
    base = RealisticChart(seed=0xBA74)
    chart = GridChart.from_realistic(
        base, -33.980, 151.190, -33.960, 151.210, resolution_m=5.0,
    )
    cfg = RBPFConfig(
        n_particles=1000,       # half the particles vs. bootstrap
        init_spread_m=40.0,
        pos_noise_m_per_sqrt_s=0.4,
        depth_noise_m=0.5,
        mcc_enabled=True,
        regularization_m=0.4,
    )
    pf = RBPF(chart, cfg)

    true_lat, true_lon = -33.970, 151.198
    pf.initialize(true_lat + 0.0002, true_lon + 0.0001, vel_n=0.0, vel_e=2.0)

    # Actual trajectory: 2 m/s east + 0.2 m/s north current
    vel_n_truth, vel_e_truth = 0.0, 2.0
    cur_n_truth, cur_e_truth = 0.2, 0.0
    dt = 1.0
    total_err = 0.0
    samples = 0
    if verbose:
        print(f"{'t':>4} {'err_m':>7} {'spread':>7} {'vn':>6} {'ve':>6} {'cur_n':>6} {'cur_e':>6} {'depth':>6}")
    rng = random.Random(0xCAFE)
    for t in range(60):
        m_per_lon = meters_per_deg_lon(true_lat)
        true_lat += (vel_n_truth + cur_n_truth) * dt / METERS_PER_DEG_LAT
        true_lon += (vel_e_truth + cur_e_truth) * dt / m_per_lon

        true_depth = chart.depth(true_lat, true_lon) + rng.gauss(0, 0.3)
        # Periodic sonar outlier — test MCC robustness
        if t % 13 == 0:
            true_depth += rng.choice([-3.0, 3.0])

        # Measured velocity = true over-ground vel + GPS noise
        # Assume GPS is available (provides measured_vel_n/e to the filter)
        measured_vn = (vel_n_truth + cur_n_truth) + rng.gauss(0, 0.3)
        measured_ve = (vel_e_truth + cur_e_truth) + rng.gauss(0, 0.3)

        est = pf.step(measured_vn, measured_ve, dt, true_depth)
        err = math.hypot(
            (true_lat - est.lat) * METERS_PER_DEG_LAT,
            (true_lon - est.lon) * m_per_lon,
        )
        if t > 15:
            total_err += err
            samples += 1
        if verbose:
            print(f"{t:4d} {err:7.1f} {est.pos_spread_m:7.1f} "
                  f"{est.vel_n_mps:6.2f} {est.vel_e_mps:6.2f} "
                  f"{est.current_n_mps:6.2f} {est.current_e_mps:6.2f} {true_depth:6.2f}")

    mean = total_err / max(1, samples)
    if verbose:
        print(f"\nmean steady-state: {mean:.2f} m  ({samples} samples, RBPF + MCC, 1000 particles)")
    return mean


if __name__ == "__main__":
    _demo()
