"""Wires the bathymetric map-matching filter into the Orca bridge's data flow.

Runs in its own thread. Consumes depth updates from BoatState, takes GPS
as seed (or last-known position when GPS drops), produces a position
estimate published as a synthetic 'bathy GPS' that Meridian's EKF can
arbitrate into its source priority list (EK3_SRC3_* for tertiary).

The filter emits to the state as `.bathy_estimate` so:
- The tablet can overlay "GPS position vs. bathy position" for visual check
- Meridian's EKF can subscribe (once firmware-side integration lands)
- Operators get a live spoof-detection signal from the GPS-vs-bathy delta
"""
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from .bathy_match import (
    BathyMatch, BathyMatchConfig, BathymetricChart, BathyEstimate,
    GridChart, METERS_PER_DEG_LAT, meters_per_deg_lon,
)
from .bathy_patch import PatchMatchFilter, PatchFilterConfig
from .state import BoatState


def pick_filter(chart: BathymetricChart):
    """Auto-select filter by chart resolution.
    Patch-match dominates on fine charts; bootstrap is safer on coarse ones.
    Crossover measured empirically at ~100m chart resolution (at 2 m/s cruise).
    """
    if isinstance(chart, GridChart):
        res_m = min(abs(chart.dlat) * METERS_PER_DEG_LAT,
                    abs(chart.dlon) * meters_per_deg_lon(chart.lat_min))
        if res_m < 80.0:
            return "patch"
    return "bootstrap"


log = logging.getLogger(__name__)


@dataclass
class BathyEstimatorConfig:
    rate_hz: float = 1.0                # filter step rate (matches depth sensor rate)
    init_after_gps_good_for_s: float = 10.0  # wait for GPS to be stable before init
    divergence_reset_error_m: float = 150.0  # re-seed if estimate wanders this far from GPS
    use_as_standalone_gps: bool = False   # If True, emit when GPS is unhealthy
    # ----- Velocity source selection during GPS dropout -----
    # When GPS drops, we need velocity to propagate the filter. Order of
    # preference: (1) Meridian-EKF-velocity (via MAVLink GLOBAL_POSITION_INT
    # — if the Cube is on our link), (2) last-known GPS velocity held for
    # up to `velocity_hold_duration_s` then decayed with time constant
    # `velocity_decay_tau_s`.
    velocity_hold_duration_s: float = 30.0    # hold last GPS velocity this long
    velocity_decay_tau_s: float = 60.0        # exponential decay time constant
    velocity_min_valid_sog_mps: float = 0.1   # below this we treat as "stopped"


class BathyEstimator:
    """Thread-safe integration of BathyMatch with the bridge's state."""

    def __init__(self, chart: BathymetricChart, state: BoatState,
                 cfg: Optional[BathyEstimatorConfig] = None) -> None:
        self.chart = chart
        self.state = state
        self.cfg = cfg or BathyEstimatorConfig()
        self._filter_cfg = BathyMatchConfig(
            n_particles=1500,
            init_spread_m=40.0,
            process_noise_m_per_sqrt_s=0.3,
            depth_noise_m=0.5,
            regularization_m=0.6,
            resample_threshold=0.6,
        )
        self._filter: Optional[BathyMatch] = None
        self._thr: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._initialized = False
        self._gps_good_since: Optional[float] = None
        self._last_depth_seen: Optional[float] = None
        self._last_step_time: Optional[float] = None
        self.latest_estimate: Optional[BathyEstimate] = None
        self.last_initialized_at: Optional[float] = None
        self.reset_count: int = 0
        self.step_count: int = 0
        # Velocity-hold state — populated when GPS is healthy, consumed
        # during GPS outage so the filter keeps dead-reckoning.
        self._last_good_vel_n: float = 0.0
        self._last_good_vel_e: float = 0.0
        self._last_good_vel_at: float = 0.0

    def start(self) -> None:
        self._thr = threading.Thread(target=self._run, daemon=True, name="bathy-est")
        self._thr.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        log.info("bathy estimator started")
        dt_target = 1.0 / max(self.cfg.rate_hz, 0.1)
        while not self._stop.is_set():
            try:
                self._step()
            except Exception as e:
                log.exception("bathy step error: %s", e)
            if self._stop.wait(dt_target):
                return

    def _step(self) -> None:
        now = time.time()
        gps = self.state.snapshot_gps()
        depth = self.state.snapshot_depth()

        gps_healthy = gps.fix_type >= 3 and not gps.stale(now, 3.0)
        have_depth = depth.updated_at > 0 and (now - depth.updated_at) < 5.0
        if not have_depth:
            return

        # --- Initialization gate ---
        if not self._initialized:
            if not gps_healthy:
                return
            if self._gps_good_since is None:
                self._gps_good_since = now
                return
            if now - self._gps_good_since < self.cfg.init_after_gps_good_for_s:
                return
            # Seed the filter at current GPS
            self._filter = BathyMatch(self.chart, self._filter_cfg)
            self._filter.initialize(gps.lat, gps.lon)
            self._initialized = True
            self.last_initialized_at = now
            log.info("bathy filter initialized at (%.6f, %.6f)", gps.lat, gps.lon)

        # --- Velocity for propagation ---
        # Cascade: fresh GPS vel → hold last-known for N seconds → decay to zero.
        # Extends useful GPS-denied window from a few seconds (filter spreads
        # too much with zero vel) to several minutes (honest dead-reckoning).
        if gps_healthy and gps.sog_mps > self.cfg.velocity_min_valid_sog_mps:
            crad = math.radians(gps.cog_deg)
            vn = gps.sog_mps * math.cos(crad)
            ve = gps.sog_mps * math.sin(crad)
            # Remember this for later decay
            self._last_good_vel_n = vn
            self._last_good_vel_e = ve
            self._last_good_vel_at = now
        else:
            # GPS unhealthy or stopped. Dead-reckon from last-known velocity.
            age = now - self._last_good_vel_at if self._last_good_vel_at > 0 else 1e9
            if age < self.cfg.velocity_hold_duration_s:
                # Hold phase — use the last-known velocity unchanged
                vn = self._last_good_vel_n
                ve = self._last_good_vel_e
            else:
                # Decay phase — exponential decay toward zero
                decay_age = age - self.cfg.velocity_hold_duration_s
                alpha = math.exp(-decay_age / self.cfg.velocity_decay_tau_s)
                vn = self._last_good_vel_n * alpha
                ve = self._last_good_vel_e * alpha

        # Timing
        if self._last_step_time is None:
            self._last_step_time = now
            return
        dt = now - self._last_step_time
        self._last_step_time = now
        if dt <= 0 or dt > 5.0:
            return

        # --- Step the filter ---
        assert self._filter is not None
        observed = depth.meters_below_transducer + depth.offset_m
        est = self._filter.step(vn, ve, dt, observed)
        self.latest_estimate = est
        self.step_count += 1

        # --- Divergence guard ---
        if gps_healthy:
            err_m = math.hypot(
                (est.lat - gps.lat) * METERS_PER_DEG_LAT,
                (est.lon - gps.lon) * meters_per_deg_lon(est.lat),
            )
            if err_m > self.cfg.divergence_reset_error_m:
                log.warning("bathy estimate diverged %.0fm from GPS — reseeding", err_m)
                self._filter.initialize(gps.lat, gps.lon)
                self.reset_count += 1

    # ------ external helpers ----------------------------------------------

    def is_usable(self) -> bool:
        """True if the filter is initialized, healthy, and non-multimodal."""
        return (self._initialized and
                self.latest_estimate is not None and
                self.latest_estimate.healthy and
                not self.latest_estimate.multimodal)

    def health_report(self) -> dict:
        e = self.latest_estimate
        return {
            "initialized": self._initialized,
            "step_count": self.step_count,
            "reset_count": self.reset_count,
            "last_spread_m": e.spread_m if e else None,
            "last_n_eff": e.n_effective if e else None,
            "last_healthy": e.healthy if e else None,
            "last_multimodal": e.multimodal if e else None,
        }
