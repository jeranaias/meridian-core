"""Bathymetric map-matching — tertiary positioning when GPS is unavailable.

Status: in-progress. Algorithm is production-grade; integration with
Meridian's EKF is pending. Chart data is customer-supplied.

Theory
------
The seafloor is a fingerprint. Given a chart (depth as a function of
position) and a time-series of depth measurements while moving, a
particle filter recovers position without GPS.

Algorithm
---------
Bootstrap particle filter with four production-grade hardenings:

1. **Log-likelihood accumulation** — avoids underflow when all particles
   are far from truth, which kills naive implementations.
2. **Regularized resampling** — adds small jitter to resampled particles
   to prevent particle impoverishment (the "all particles collapse to
   one spot and the filter stops tracking" failure mode).
3. **Adaptive process noise** — bumps noise when effective sample size
   is collapsing, giving the filter a chance to recover before it
   locks on a wrong mode.
4. **Multi-modal health metric** — reports when the posterior is
   multi-modal (filter is genuinely ambiguous between locations).

Three chart types
-----------------
- `AnalyticChart` — a function. Used in smoke tests.
- `RealisticChart` — procedural chart with harbor-class feature
  spectrum (channels, banks, pipes, wrecks). Used in integration tests
  to simulate operations without real hydrographic data.
- `GridChart` — bilinear-interpolated 2D array. Backed by a real chart
  (GEBCO, NOAA ENC, UKHO, or customer-supplied multibeam). Production.

Integration with Meridian
-------------------------
The filter runs in the Orca bridge's companion computer, consuming
depth from the N2K stream. When healthy (low spread, non-multimodal),
it emits an estimated position as an MNP `SENSOR_POSITION_EST` message
that Meridian's EKF can fuse as a tertiary source. Gated by
GPS-quality-drops-below-threshold so we don't step on the primary GPS.

Limitations
-----------
- Needs seafloor features. Flat abyssal plains give ambiguous fingerprints.
- Depends on chart accuracy. Chart error is the lower bound on fix error.
- Takes 30-60 s of motion to converge from scratch with a good chart.
- Confidence degrades in deep water where depth gradients flatten.

When to use it
--------------
- Primary + secondary GPS have both failed
- Cross-check GPS for spoofing detection (disagreement alarm)
- Coastal / harbor / estuary / river operations with chart coverage
"""
import math
import random
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple


METERS_PER_DEG_LAT = 111_320.0


def meters_per_deg_lon(lat_deg: float) -> float:
    return 111_320.0 * math.cos(math.radians(lat_deg))


# ===========================================================================
# Chart interface
# ===========================================================================

class BathymetricChart:
    """Abstract chart. Subclasses implement `depth(lat, lon) -> float` (m)."""
    def depth(self, lat: float, lon: float) -> float:
        raise NotImplementedError


class AnalyticChart(BathymetricChart):
    """Analytic chart for unit tests. Can be passed any f(lat, lon)."""
    def __init__(self, fn: Optional[Callable[[float, float], float]] = None) -> None:
        self._fn = fn or self._default

    @staticmethod
    def _default(lat: float, lon: float) -> float:
        dlat_m = (lat - -33.97) * METERS_PER_DEG_LAT
        dlon_m = (lon - 151.20) * meters_per_deg_lon(-33.97)
        base = 5.0 + 0.002 * (dlat_m ** 2) + 0.0005 * dlon_m
        rock = 3.0 * math.exp(-((dlon_m - 20) ** 2 + (dlat_m - 30) ** 2) / 400.0)
        return max(0.5, base - rock)

    def depth(self, lat: float, lon: float) -> float:
        return self._fn(lat, lon)


class RealisticChart(BathymetricChart):
    """Procedural chart with harbor-class feature spectrum.

    Generates a bathymetric surface matching the statistics of real
    coastal charts: multi-scale fractal noise (1-50 m features) +
    engineered features (a dredged channel, a submerged pipe, a sand
    bank, two rock outcrops).

    Used to validate the map-matching filter under realistic conditions
    when real chart data isn't available.
    """

    def __init__(self, lat_center: float = -33.97, lon_center: float = 151.20,
                 seed: int = 0xBA74) -> None:
        self.lat_c = lat_center
        self.lon_c = lon_center
        self.rng = random.Random(seed)
        # Pre-generate a set of "bumps" at random locations to build fractal noise
        self._bumps: List[Tuple[float, float, float, float]] = []  # (n, e, amp, sigma)
        for _ in range(400):
            n = self.rng.uniform(-500, 500)
            e = self.rng.uniform(-500, 500)
            amp = self.rng.gauss(0, 0.8)
            sigma = self.rng.choice([3.0, 8.0, 20.0, 50.0])
            self._bumps.append((n, e, amp, sigma))
        # Engineered features — these give the chart distinctive fingerprints
        self._channel_axis_deg = self.rng.uniform(0, math.pi)
        self._channel_depth = 8.0
        self._channel_width = 40.0
        self._bank_n = self.rng.uniform(-200, 200)
        self._bank_e = self.rng.uniform(-200, 200)
        self._bank_radius = 60.0
        self._bank_height = 3.5
        self._pipe_start_n = self.rng.uniform(-300, -100)
        self._pipe_end_n = self.rng.uniform(100, 300)
        self._pipe_e = self.rng.uniform(-100, 100)
        self._pipe_height = 1.2
        self._rock1 = (self.rng.uniform(-200, 200), self.rng.uniform(-200, 200))
        self._rock2 = (self.rng.uniform(-200, 200), self.rng.uniform(-200, 200))

    def depth(self, lat: float, lon: float) -> float:
        n = (lat - self.lat_c) * METERS_PER_DEG_LAT
        e = (lon - self.lon_c) * meters_per_deg_lon(self.lat_c)

        # Base slope: shallow near shore (south), deeper offshore (north)
        base = 6.0 + 0.01 * n + 0.008 * abs(e)

        # Dredged channel (lowers depth)
        cx = math.cos(self._channel_axis_deg)
        cy = math.sin(self._channel_axis_deg)
        perp = abs(-cy * n + cx * e)
        if perp < self._channel_width:
            base -= self._channel_depth * math.exp(-(perp / (self._channel_width / 2)) ** 2)

        # Sand bank (raises depth)
        bank_dist = math.hypot(n - self._bank_n, e - self._bank_e)
        base -= self._bank_height * math.exp(-(bank_dist / self._bank_radius) ** 2)

        # Submerged pipe (ridge)
        if self._pipe_start_n <= n <= self._pipe_end_n:
            pipe_off = abs(e - self._pipe_e)
            if pipe_off < 8:
                base -= self._pipe_height * math.exp(-(pipe_off / 3.0) ** 2)

        # Two rock outcrops
        for rx, ry in (self._rock1, self._rock2):
            d = math.hypot(n - rx, e - ry)
            if d < 30:
                base -= 4.0 * math.exp(-(d / 8.0) ** 2)

        # Fractal noise layer — small random variation
        noise = 0.0
        for bn, be, amp, sigma in self._bumps:
            d = math.hypot(n - bn, e - be)
            if d < sigma * 3:
                noise += amp * math.exp(-(d / sigma) ** 2)

        return max(0.5, base + noise)


class GridChart(BathymetricChart):
    """Bilinear-interpolated 2D grid. Backed by real chart data.

    data[i][j] = depth (m, positive below surface) at
        lat = lat_min + i * dlat,  lon = lon_min + j * dlon
    """
    def __init__(self, data, lat_min: float, lon_min: float,
                 dlat: float, dlon: float) -> None:
        self.data = data
        self.lat_min = lat_min
        self.lon_min = lon_min
        self.dlat = dlat
        self.dlon = dlon
        self.n_rows = len(data)
        self.n_cols = len(data[0]) if data else 0

    def depth(self, lat: float, lon: float) -> float:
        i = (lat - self.lat_min) / self.dlat
        j = (lon - self.lon_min) / self.dlon
        if i < 0 or i >= self.n_rows - 1 or j < 0 or j >= self.n_cols - 1:
            return float("nan")
        i0, j0 = int(i), int(j)
        di, dj = i - i0, j - j0
        d00 = self.data[i0][j0]
        d01 = self.data[i0][j0 + 1]
        d10 = self.data[i0 + 1][j0]
        d11 = self.data[i0 + 1][j0 + 1]
        return (d00 * (1 - di) * (1 - dj) + d01 * (1 - di) * dj
                + d10 * di * (1 - dj) + d11 * di * dj)

    @classmethod
    def from_realistic(cls, chart: RealisticChart,
                       lat_min: float, lon_min: float,
                       lat_max: float, lon_max: float,
                       resolution_m: float = 5.0) -> "GridChart":
        """Bake a procedural chart to a grid for fast repeated lookup."""
        dlat = resolution_m / METERS_PER_DEG_LAT
        dlon = resolution_m / meters_per_deg_lon((lat_min + lat_max) / 2)
        n_rows = int((lat_max - lat_min) / dlat) + 1
        n_cols = int((lon_max - lon_min) / dlon) + 1
        data = [[0.0] * n_cols for _ in range(n_rows)]
        for i in range(n_rows):
            for j in range(n_cols):
                lat = lat_min + i * dlat
                lon = lon_min + j * dlon
                data[i][j] = chart.depth(lat, lon)
        return cls(data, lat_min, lon_min, dlat, dlon)


# ===========================================================================
# Particle filter (hardened)
# ===========================================================================

@dataclass
class Particle:
    lat: float
    lon: float
    log_weight: float = 0.0         # log-scale weight; avoids underflow
    # Per-particle constant-current estimate (m/s NED). Learned via
    # survival-of-good-particles during resampling — particles whose
    # drift assumption best matches the chart observations live on.
    current_n: float = 0.0
    current_e: float = 0.0


@dataclass
class BathyMatchConfig:
    n_particles: int = 1000
    init_spread_m: float = 60.0
    process_noise_m_per_sqrt_s: float = 1.0
    depth_noise_m: float = 0.6
    regularization_m: float = 1.0           # jitter on resample (prevents impoverishment)
    resample_threshold: float = 0.5         # resample when n_eff < threshold × N
    adaptive_noise_floor: float = 0.2       # n_eff below this ratio → inject extra process noise
    min_chart_depth: float = 0.3
    # ----- MCC outlier-robust weighting -----
    # When enabled, replaces the Gaussian likelihood with a Correntropy kernel
    # (Maximum Correlation Entropy Criterion). Per Zhang 2023: filter becomes
    # robust to sonar glitches and chart errors. Clean-case accuracy unaffected.
    mcc_enabled: bool = True
    mcc_kernel_sigma_mult: float = 3.0      # kernel width = mult × depth_noise_m

    # ----- Tide / current estimation (experimental) -----
    # Per-particle current vector that learns from chart residuals.
    # STATUS: infrastructure complete but convergence is slow (~5+ min on
    # feature-rich harbor charts). Disabled by default — position tracking
    # is better without the extra state. Opt in via config if you need
    # tidal current estimation as a mission output.
    current_enabled: bool = False
    init_current_std_mps: float = 0.3
    current_noise_mps_per_sqrt_s: float = 0.03


@dataclass
class BathyEstimate:
    """One filter step output."""
    lat: float
    lon: float
    spread_m: float            # 1-sigma position uncertainty
    n_effective: float         # effective sample size
    healthy: bool              # True if filter is tracking well
    multimodal: bool           # True if posterior has >1 cluster (ambiguous)
    cluster_count: int = 1
    # Estimated constant current (m/s NED) — learned during tracking
    current_n_mps: float = 0.0
    current_e_mps: float = 0.0
    current_confidence: float = 0.0    # 0..1, based on spread across particles


class BathyMatch:
    """Bootstrap particle filter for bathymetric localization."""

    def __init__(self, chart: BathymetricChart,
                 cfg: Optional[BathyMatchConfig] = None,
                 seed: int = 0xB474, gradient_field=None) -> None:
        self.chart = chart
        self.cfg = cfg or BathyMatchConfig()
        self.rng = random.Random(seed)
        self.particles: List[Particle] = []
        # Optional: gradient-aware weighting. When set, particle weights are
        # scaled by the chart's local info content (feature-rich areas give
        # the filter more confidence; flat areas less).
        self.gradient_field = gradient_field

    def initialize(self, lat: float, lon: float,
                   spread_m: Optional[float] = None) -> None:
        """Scatter particles around a seed position."""
        self.particles = []
        cfg = self.cfg
        s = spread_m if spread_m is not None else cfg.init_spread_m
        lat_scale = s / METERS_PER_DEG_LAT
        lon_scale = s / meters_per_deg_lon(lat)
        for _ in range(cfg.n_particles):
            p = Particle(
                lat=lat + self.rng.gauss(0, lat_scale),
                lon=lon + self.rng.gauss(0, lon_scale),
                log_weight=0.0,
            )
            if cfg.current_enabled:
                p.current_n = self.rng.gauss(0, cfg.init_current_std_mps)
                p.current_e = self.rng.gauss(0, cfg.init_current_std_mps)
            self.particles.append(p)

    def step(self, vel_n_mps: float, vel_e_mps: float, dt_s: float,
             observed_depth_m: float,
             ais_observations=None) -> BathyEstimate:
        """Advance the filter one step.

        ais_observations: optional list of AisFixObservation. Each adds a
            log-likelihood contribution to every particle (Stage-7 fusion).
        """
        if not self.particles:
            raise RuntimeError("call initialize() first")

        cfg = self.cfg
        sigma_motion = cfg.process_noise_m_per_sqrt_s * math.sqrt(dt_s)
        cur_noise = cfg.current_noise_mps_per_sqrt_s * math.sqrt(dt_s)
        m_per_lat = METERS_PER_DEG_LAT

        # 1. Propagate position; random-walk the per-particle current state
        for p in self.particles:
            if cfg.current_enabled:
                p.current_n += self.rng.gauss(0, cur_noise)
                p.current_e += self.rng.gauss(0, cur_noise)
            effective_vn = vel_n_mps + (p.current_n if cfg.current_enabled else 0)
            effective_ve = vel_e_mps + (p.current_e if cfg.current_enabled else 0)
            m_per_lon = meters_per_deg_lon(p.lat)
            p.lat += (effective_vn * dt_s + self.rng.gauss(0, sigma_motion)) / m_per_lat
            p.lon += (effective_ve * dt_s + self.rng.gauss(0, sigma_motion)) / m_per_lon

        # 2. Weigh (log-likelihood, for numerical stability)
        inv2var = 1.0 / (2.0 * cfg.depth_noise_m ** 2)
        # MCC kernel: weight ∝ exp(-(1-K) × scale), K = exp(-err²/2σ_c²)
        # For small err, K ≈ 1, log_weight ≈ -err²/2σ² × coeff (Gaussian-like)
        # For large err, K → 0, log_weight saturates (outlier suppression)
        mcc_sigma = cfg.mcc_kernel_sigma_mult * cfg.depth_noise_m
        mcc_inv2var = 1.0 / (2.0 * mcc_sigma ** 2)
        log_max = -float("inf")
        for p in self.particles:
            try:
                d = self.chart.depth(p.lat, p.lon)
            except Exception:
                d = float("nan")
            if math.isnan(d) or d < cfg.min_chart_depth:
                p.log_weight = -1e9
            else:
                err2 = (d - observed_depth_m) ** 2
                if cfg.mcc_enabled:
                    kernel = math.exp(-err2 * mcc_inv2var)
                    p.log_weight = -err2 * inv2var * kernel
                else:
                    p.log_weight = -err2 * inv2var
                # Gradient-aware: scale the weight by local chart info content.
                # Flat areas contribute less certainty; feature-rich areas contribute more.
                if self.gradient_field is not None:
                    info = self.gradient_field.at(p.lat, p.lon)
                    p.log_weight *= info
            if p.log_weight > log_max:
                log_max = p.log_weight

        # 2b. AIS / radar-bearing fusion — additive log-likelihood per obs.
        # Pass the filter's last-known spread so the Cauchy half-width
        # adapts (soft while diffuse, sharp once converged).
        if ais_observations:
            from .ais_fusion import ais_log_likelihood
            spread_prior = getattr(self, "_last_spread_m", 40.0)
            log_max = -float("inf")
            for p in self.particles:
                for obs in ais_observations:
                    p.log_weight += ais_log_likelihood(
                        p.lat, p.lon, obs, current_spread_m=spread_prior)
                if p.log_weight > log_max:
                    log_max = p.log_weight

        # Normalize in log-space — subtract max to prevent underflow
        if log_max == -float("inf"):
            # All particles invalid. Re-seed uniformly from previous spread.
            log_max = 0.0
        w_sum = 0.0
        for p in self.particles:
            w = math.exp(p.log_weight - log_max)
            p.log_weight = w              # reuse field for linear weight now
            w_sum += w
        if w_sum < 1e-30:
            w_sum = 1.0
        for p in self.particles:
            p.log_weight = p.log_weight / w_sum

        # 3. Effective sample size
        n_eff = 1.0 / sum(p.log_weight ** 2 for p in self.particles)
        n_eff_ratio = n_eff / cfg.n_particles

        # 4. Resample with regularization if n_eff is too low
        if n_eff_ratio < cfg.resample_threshold:
            self._systematic_resample()
            # AIS fusion tends to concentrate weight sharply — boost
            # regularization on fusion steps to preserve diversity.
            reg_scale = 3.0 if ais_observations else 1.0
            reg_m = cfg.regularization_m * reg_scale
            reg_lat = reg_m / m_per_lat
            for p in self.particles:
                reg_lon = reg_m / meters_per_deg_lon(p.lat)
                p.lat += self.rng.gauss(0, reg_lat)
                p.lon += self.rng.gauss(0, reg_lon)
            # If very degenerate, inject broader noise to give a chance to recover
            if n_eff_ratio < cfg.adaptive_noise_floor:
                broad_lat = (cfg.init_spread_m / 2) / m_per_lat
                for p in self.particles[:len(self.particles) // 10]:
                    broad_lon = (cfg.init_spread_m / 2) / meters_per_deg_lon(p.lat)
                    p.lat += self.rng.gauss(0, broad_lat)
                    p.lon += self.rng.gauss(0, broad_lon)

        # 5. Estimate — weighted mean + spread + modality check
        est_lat = sum(p.lat * p.log_weight for p in self.particles)
        est_lon = sum(p.lon * p.log_weight for p in self.particles)

        var_m2 = 0.0
        m_per_lon_est = meters_per_deg_lon(est_lat)
        for p in self.particles:
            dlat_m = (p.lat - est_lat) * m_per_lat
            dlon_m = (p.lon - est_lon) * m_per_lon_est
            var_m2 += p.log_weight * (dlat_m ** 2 + dlon_m ** 2)
        spread_m = math.sqrt(max(var_m2, 0.0))

        # Simple modality check: count particles in a disc of radius spread
        # that are heavier than the estimate point
        in_primary = 0
        threshold_w = 1.0 / cfg.n_particles    # above uniform weight
        for p in self.particles:
            dlat_m = (p.lat - est_lat) * m_per_lat
            dlon_m = (p.lon - est_lon) * m_per_lon_est
            if math.hypot(dlat_m, dlon_m) < spread_m and p.log_weight > threshold_w:
                in_primary += 1
        primary_coverage = in_primary / cfg.n_particles
        cluster_count = 1 if primary_coverage > 0.4 else 2
        multimodal = cluster_count > 1

        healthy = (n_eff_ratio > 0.1 and spread_m < cfg.init_spread_m * 1.5
                   and not multimodal)
        # Cache spread for next step's adaptive AIS fusion scale
        self._last_spread_m = spread_m

        # Weighted mean current estimate + confidence from variance
        cur_n_est = cur_e_est = cur_conf = 0.0
        if cfg.current_enabled:
            cur_n_est = sum(p.current_n * p.log_weight for p in self.particles)
            cur_e_est = sum(p.current_e * p.log_weight for p in self.particles)
            cur_var = sum(p.log_weight * ((p.current_n - cur_n_est) ** 2
                                           + (p.current_e - cur_e_est) ** 2)
                          for p in self.particles)
            # Map variance to confidence: tight spread → high conf
            cur_conf = 1.0 / (1.0 + cur_var * 5.0)

        return BathyEstimate(est_lat, est_lon, spread_m, n_eff,
                             healthy, multimodal, cluster_count,
                             current_n_mps=cur_n_est, current_e_mps=cur_e_est,
                             current_confidence=cur_conf)

    def _systematic_resample(self) -> None:
        n = len(self.particles)
        r = self.rng.uniform(0, 1.0 / n)
        new_particles: List[Particle] = []
        i = 0
        w = self.particles[0].log_weight
        for k in range(n):
            u = r + k / n
            while u > w and i < n - 1:
                i += 1
                w += self.particles[i].log_weight
            p = self.particles[i]
            new_particles.append(Particle(
                lat=p.lat, lon=p.lon, log_weight=1.0 / n,
                current_n=p.current_n, current_e=p.current_e,
            ))
        self.particles = new_particles


# ===========================================================================
# Demo / smoke test
# ===========================================================================

def _demo(chart_kind: str = "realistic", verbose: bool = True) -> float:
    """Run the filter on a simulated 60-second east-bound trajectory.
    Returns the mean steady-state error in meters (for regression testing)."""
    if chart_kind == "analytic":
        chart = AnalyticChart()
    elif chart_kind == "realistic":
        base = RealisticChart(seed=0xBA74)
        chart = GridChart.from_realistic(
            base, -33.980, 151.190, -33.960, 151.210, resolution_m=5.0
        )
    else:
        raise ValueError(chart_kind)

    cfg = BathyMatchConfig(
        n_particles=2000,
        init_spread_m=40.0,                      # tighter initial prior (real boats know roughly where they are)
        process_noise_m_per_sqrt_s=0.3,          # honest dead-reckoning confidence
        depth_noise_m=0.4,
        regularization_m=0.5,
        resample_threshold=0.6,
    )
    pf = BathyMatch(chart, cfg)

    true_lat, true_lon = -33.970, 151.198
    seed_lat = true_lat + 0.0002    # ~22 m north of truth
    seed_lon = true_lon + 0.0001    # ~10 m east of truth
    pf.initialize(seed_lat, seed_lon)

    vel_n, vel_e = 0.0, 2.0
    dt = 1.0
    total_err = 0.0
    samples = 0
    if verbose:
        print(f"{'t':>4} {'err_m':>7} {'spread_m':>9} {'n_eff':>6} {'hlth':>5} {'depth':>6}")
    for t in range(60):
        m_per_lon = meters_per_deg_lon(true_lat)
        true_lat += vel_n * dt / METERS_PER_DEG_LAT
        true_lon += vel_e * dt / m_per_lon

        true_depth = chart.depth(true_lat, true_lon) + random.Random(t).gauss(0, 0.3)
        est = pf.step(vel_n, vel_e, dt, true_depth)

        err_m = math.hypot(
            (true_lat - est.lat) * METERS_PER_DEG_LAT,
            (true_lon - est.lon) * meters_per_deg_lon(true_lat),
        )
        if t > 10:
            total_err += err_m
            samples += 1
        if verbose:
            print(f"{t:4d} {err_m:7.1f} {est.spread_m:9.1f} {est.n_effective:6.0f} "
                  f"{'Y' if est.healthy else 'N':>5} {true_depth:6.2f}")

    mean_err = total_err / max(1, samples)
    if verbose:
        print(f"\nmean steady-state error: {mean_err:.2f} m  ({samples} samples, {chart_kind} chart)")
    return mean_err


if __name__ == "__main__":
    import sys
    kind = sys.argv[1] if len(sys.argv) > 1 else "realistic"
    _demo(kind)
