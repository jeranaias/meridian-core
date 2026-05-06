"""SKPF-style tide/current estimator.

Standalone module that observes (GPS over-ground velocity) vs (commanded
or model-predicted water-relative velocity) and recovers the constant
current vector from the residual.

Why a separate module
---------------------
My first attempt embedded per-particle current state in the bathy
filter. Convergence was slow because the current state had to share
selection pressure with position and the observation model was
indirect (chart-depth match).

The clean version: current is observable *directly* from the residual
between GPS-measured over-ground velocity and estimated-water-relative
velocity. That's a straightforward Kalman problem on its own:

  z_over_ground = v_boat + c_current
  → given z (from GPS) and v_boat (from commanded thrust, or speed-through-water sensor)
    current = z - v_boat + noise

If v_boat is known (or closely estimated), we get the current vector
with Kalman precision, no particles needed. The "smooth kernel"
component smooths it over time to reject noise.

Usage
-----
The bridge ingests GPS velocity + VESC RPM-derived forward speed
(Phase 2) or commanded throttle (Phase 1 fallback), and feeds both into
this estimator. It publishes a tidal current vector updated ~1 Hz.

Output
------
Continuous Kalman estimate of (cn, ce) with covariance, readable as
a `CurrentEstimate` structure. Updated at the rate GPS provides
velocity (typically 10 Hz).
"""
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class CurrentEstimate:
    current_n_mps: float = 0.0
    current_e_mps: float = 0.0
    std_n_mps: float = 1.0          # 1-sigma uncertainty
    std_e_mps: float = 1.0
    n_updates: int = 0

    @property
    def magnitude(self) -> float:
        return math.hypot(self.current_n_mps, self.current_e_mps)

    @property
    def direction_deg(self) -> float:
        """Bearing TOWARDS which current flows, degrees true (0 = north)."""
        return (math.degrees(math.atan2(self.current_e_mps, self.current_n_mps)) + 360) % 360

    @property
    def confidence(self) -> float:
        """0..1, based on std. >0.7 = confident, <0.3 = seeding."""
        unc = max(self.std_n_mps, self.std_e_mps)
        return 1.0 / (1.0 + unc * 5.0)


@dataclass
class CurrentEstimatorConfig:
    # Initial prior uncertainty (harbor currents typically < ±1 m/s)
    init_std_mps: float = 0.5
    # Process noise (how fast currents evolve — tidal cycles are 6h, so slow)
    process_noise_mps_per_sqrt_s: float = 0.02
    # Measurement noise (GPS COG+SOG) — typical 0.2-0.5 m/s
    measurement_noise_mps: float = 0.3
    # Residual threshold for outlier gating
    outlier_mahalanobis: float = 4.0


class CurrentEstimator:
    """Kalman filter for constant tidal/river current given
    repeated observations of (over_ground_vel, water_vel)."""

    def __init__(self, cfg: Optional[CurrentEstimatorConfig] = None) -> None:
        self.cfg = cfg or CurrentEstimatorConfig()
        # State: [cn, ce]
        self.mu = np.zeros(2)
        self.Sig = np.eye(2) * self.cfg.init_std_mps ** 2
        self.n_updates = 0
        self._last_update_t: Optional[float] = None

    def reset(self) -> None:
        self.mu = np.zeros(2)
        self.Sig = np.eye(2) * self.cfg.init_std_mps ** 2
        self.n_updates = 0
        self._last_update_t = None

    def update(self, over_ground_vn: float, over_ground_ve: float,
               water_vn: float, water_ve: float, t: float) -> CurrentEstimate:
        """One measurement update. Times in seconds, velocities in m/s NED.

        water_vn, water_ve: the vessel's through-water velocity (what the
            hull is doing relative to the water). Obtained from commanded
            thrust / hull model, or a speed-through-water sensor, or VESC
            RPM × thrust-to-speed calibration.

        over_ground_vn, over_ground_ve: GPS COG+SOG-derived NED velocity.
        """
        # Time update (process noise)
        if self._last_update_t is not None:
            dt = t - self._last_update_t
            if dt > 0:
                q = self.cfg.process_noise_mps_per_sqrt_s ** 2 * dt
                self.Sig += np.eye(2) * q
        self._last_update_t = t

        # Observation: z = [cn_obs, ce_obs] where
        #   cn_obs = over_ground_vn - water_vn
        #   ce_obs = over_ground_ve - water_ve
        z = np.array([
            over_ground_vn - water_vn,
            over_ground_ve - water_ve,
        ])
        # H = I, R = measurement_noise_mps² × I
        R = (self.cfg.measurement_noise_mps ** 2) * np.eye(2)
        # Innovation
        y = z - self.mu
        S = self.Sig + R
        # Outlier gate (Mahalanobis distance)
        try:
            S_inv = np.linalg.inv(S)
            mahal2 = float(y @ S_inv @ y)
        except np.linalg.LinAlgError:
            mahal2 = 0.0
        if mahal2 < self.cfg.outlier_mahalanobis ** 2:
            # Standard Kalman update
            K = self.Sig @ S_inv
            self.mu = self.mu + K @ y
            self.Sig = (np.eye(2) - K) @ self.Sig
            self.n_updates += 1
        # else: reject as outlier, keep prior

        return self.snapshot()

    def snapshot(self) -> CurrentEstimate:
        return CurrentEstimate(
            current_n_mps=float(self.mu[0]),
            current_e_mps=float(self.mu[1]),
            std_n_mps=float(math.sqrt(max(0.0, self.Sig[0, 0]))),
            std_e_mps=float(math.sqrt(max(0.0, self.Sig[1, 1]))),
            n_updates=self.n_updates,
        )


# ===========================================================================
# Demo
# ===========================================================================

def _demo() -> None:
    """Simulate 2 minutes of motion in a 0.3 m/s N, 0.1 m/s E current.
    Feed the estimator noisy GPS + exact water-vel; watch it converge."""
    import random as _r
    rng = _r.Random(0xCAFE)
    est = CurrentEstimator()

    # Truth: current flows NE at (0.3, 0.1) m/s
    cur_n_true, cur_e_true = 0.3, 0.1
    # Vessel going 2 m/s east through water
    water_vn, water_ve = 0.0, 2.0

    print(f"{'t':>4} {'cn':>6} {'ce':>6} {'|c|':>6} {'dir':>5} {'conf':>5}")
    for step in range(120):
        t = step * 1.0
        # Measured over-ground = water vel + current + GPS noise
        og_vn = water_vn + cur_n_true + rng.gauss(0, 0.2)
        og_ve = water_ve + cur_e_true + rng.gauss(0, 0.2)
        e = est.update(og_vn, og_ve, water_vn, water_ve, t)
        if step % 10 == 0:
            print(f"{step:4d} {e.current_n_mps:6.3f} {e.current_e_mps:6.3f} "
                  f"{e.magnitude:6.3f} {e.direction_deg:5.0f} {e.confidence:5.2f}")

    print(f"\nTruth: cn=0.300, ce=0.100, |c|=0.316, dir=18°")
    print(f"Est :  cn={est.mu[0]:.3f}, ce={est.mu[1]:.3f}, "
          f"mag={est.snapshot().magnitude:.3f}, dir={est.snapshot().direction_deg:.0f}°")


if __name__ == "__main__":
    _demo()
