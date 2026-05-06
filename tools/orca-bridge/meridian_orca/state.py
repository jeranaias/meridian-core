"""Running EKF-input state. Holds the latest values from Orca that we care
about. Thread-safe via a single update lock. Every field has a timestamp so
consumers (MAVLink emitter) can detect staleness.
"""
from dataclasses import dataclass, field
from threading import RLock
from typing import Dict, Optional
import time


@dataclass
class GpsFix:
    lat: float = 0.0              # degrees
    lon: float = 0.0              # degrees
    alt_m: float = 0.0            # above MSL
    sog_mps: float = 0.0
    cog_deg: float = 0.0
    hdop: float = 99.0
    vdop: float = 99.0
    fix_type: int = 0             # 0=none, 2=2D, 3=3D, 4=DGPS, 5=RTK-float, 6=RTK-fix
    satellites: int = 0
    updated_at: float = 0.0

    def stale(self, now: float, timeout: float = 3.0) -> bool:
        return now - self.updated_at > timeout


@dataclass
class Heading:
    true_rad: float = 0.0         # true heading in radians, 0 = north
    rate_rad_s: float = 0.0
    mag_variation_rad: float = 0.0
    updated_at: float = 0.0


@dataclass
class Depth:
    meters_below_transducer: float = 0.0
    offset_m: float = 0.0         # transducer-to-keel or transducer-to-waterline
    range_m: float = 100.0        # max detectable range
    updated_at: float = 0.0


@dataclass
class Wind:
    speed_mps: float = 0.0
    direction_rad: float = 0.0    # true bearing
    reference: str = "true"       # "true" or "apparent"
    updated_at: float = 0.0


@dataclass
class AisContact:
    mmsi: int
    lat: float
    lon: float
    cog_deg: float = 0.0
    sog_mps: float = 0.0
    heading_deg: float = 0.0
    nav_status: int = 15
    ship_type: int = 0
    name: str = ""
    call_sign: str = ""
    length_m: float = 0.0
    beam_m: float = 0.0
    class_letter: str = "B"       # "A" or "B"
    updated_at: float = 0.0


class BoatState:
    """The single running state the bridge maintains. All getters + updaters
    take the lock briefly. Readers get a snapshot copy, never a live reference.
    """
    def __init__(self) -> None:
        self._lock = RLock()
        self.gps: GpsFix = GpsFix()
        self.heading: Heading = Heading()
        self.depth: Depth = Depth()
        self.wind: Wind = Wind()
        self._ais: Dict[int, AisContact] = {}   # mmsi -> AisContact
        self.packets_seen: int = 0
        self.packets_decoded: int = 0
        self.last_any_update_at: float = 0.0

    # --- updaters -----------------------------------------------------------
    def update_gps(self, **kw) -> None:
        with self._lock:
            now = time.time()
            for k, v in kw.items():
                if hasattr(self.gps, k):
                    setattr(self.gps, k, v)
            self.gps.updated_at = now
            self.last_any_update_at = now

    def update_heading(self, **kw) -> None:
        with self._lock:
            now = time.time()
            for k, v in kw.items():
                if hasattr(self.heading, k):
                    setattr(self.heading, k, v)
            self.heading.updated_at = now
            self.last_any_update_at = now

    def update_depth(self, **kw) -> None:
        with self._lock:
            now = time.time()
            for k, v in kw.items():
                if hasattr(self.depth, k):
                    setattr(self.depth, k, v)
            self.depth.updated_at = now
            self.last_any_update_at = now

    def update_wind(self, **kw) -> None:
        with self._lock:
            now = time.time()
            for k, v in kw.items():
                if hasattr(self.wind, k):
                    setattr(self.wind, k, v)
            self.wind.updated_at = now
            self.last_any_update_at = now

    def update_ais(self, contact: AisContact) -> None:
        with self._lock:
            contact.updated_at = time.time()
            self._ais[contact.mmsi] = contact
            self.last_any_update_at = contact.updated_at

    # --- readers (snapshots) ------------------------------------------------
    def snapshot_gps(self) -> GpsFix:
        with self._lock:
            return GpsFix(**self.gps.__dict__)

    def snapshot_heading(self) -> Heading:
        with self._lock:
            return Heading(**self.heading.__dict__)

    def snapshot_depth(self) -> Depth:
        with self._lock:
            return Depth(**self.depth.__dict__)

    def snapshot_wind(self) -> Wind:
        with self._lock:
            return Wind(**self.wind.__dict__)

    def snapshot_ais(self) -> Dict[int, AisContact]:
        with self._lock:
            return {m: AisContact(**c.__dict__) for m, c in self._ais.items()}

    def prune_stale_ais(self, max_age_s: float = 600.0) -> int:
        """Drop AIS contacts not updated in the last `max_age_s` seconds."""
        now = time.time()
        dropped = 0
        with self._lock:
            stale = [m for m, c in self._ais.items() if now - c.updated_at > max_age_s]
            for m in stale:
                del self._ais[m]
                dropped += 1
        return dropped

    # --- stats --------------------------------------------------------------
    def stats(self) -> dict:
        with self._lock:
            return {
                "packets_seen":    self.packets_seen,
                "packets_decoded": self.packets_decoded,
                "gps_age_s":       (time.time() - self.gps.updated_at) if self.gps.updated_at else None,
                "hdg_age_s":       (time.time() - self.heading.updated_at) if self.heading.updated_at else None,
                "depth_age_s":     (time.time() - self.depth.updated_at) if self.depth.updated_at else None,
                "wind_age_s":      (time.time() - self.wind.updated_at) if self.wind.updated_at else None,
                "ais_contacts":    len(self._ais),
            }
