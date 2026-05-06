"""MAVLink output: reads BoatState, emits GPS_INPUT / DISTANCE_SENSOR /
WIND_COV / AHRS2 to the Cube at configured rates.

Uses pymavlink. Connects to the Cube over whatever endpoint the config
specifies (serial:/dev/ttyACM0:115200 or udpout:127.0.0.1:14550 etc).

The GPS_INPUT message is the core of the zero-GPS-puck play — it lets the
Orca's internal GPS become the Cube's primary position source without any
additional serial cable.
"""
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil

from ..state import BoatState


log = logging.getLogger(__name__)


# GPS_INPUT ignore-flags bitfield (from MAVLink spec)
GPS_INPUT_IGNORE_FLAG_ALT          = 0x01
GPS_INPUT_IGNORE_FLAG_HDOP         = 0x02
GPS_INPUT_IGNORE_FLAG_VDOP         = 0x04
GPS_INPUT_IGNORE_FLAG_VEL_HORIZ    = 0x08
GPS_INPUT_IGNORE_FLAG_VEL_VERT     = 0x10
GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY    = 0x20
GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 0x40
GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY   = 0x80


@dataclass
class MavlinkConfig:
    endpoint: str = "udpout:127.0.0.1:14550"
    source_system: int = 251
    source_component: int = 1
    target_system: int = 1
    target_component: int = 1
    emit_gps_input: bool = True
    emit_distance_sensor: bool = True
    emit_wind_cov: bool = True
    emit_ahrs2: bool = False
    max_gps_input_hz: float = 10.0
    stale_timeout_s: float = 3.0


class MavlinkOut:
    """Single-threaded emitter. Call .start() to run in background."""

    def __init__(self, cfg: MavlinkConfig, state: BoatState) -> None:
        self.cfg = cfg
        self.state = state
        self._stop = threading.Event()
        self._thr: Optional[threading.Thread] = None
        # pymavlink connection (serial / udp / tcp)
        self._mav = mavutil.mavlink_connection(
            cfg.endpoint,
            source_system=cfg.source_system,
            source_component=cfg.source_component,
            dialect="ardupilotmega",
        )
        self._last_gps_input_at = 0.0
        self._last_distance_at = 0.0
        self._last_wind_at = 0.0
        self._last_ahrs2_at = 0.0

    def start(self) -> None:
        self._thr = threading.Thread(target=self._run, daemon=True, name="mavlink-out")
        self._thr.start()

    def stop(self) -> None:
        self._stop.set()

    def close(self) -> None:
        self.stop()
        try: self._mav.close()
        except: pass

    # --- main loop ----------------------------------------------------------
    def _run(self) -> None:
        log.info("mavlink-out started; endpoint=%s target=sys%d/comp%d",
                 self.cfg.endpoint, self.cfg.target_system, self.cfg.target_component)
        gps_period = 1.0 / max(self.cfg.max_gps_input_hz, 1.0)
        while not self._stop.is_set():
            now = time.time()
            if self.cfg.emit_gps_input and (now - self._last_gps_input_at) >= gps_period:
                self._emit_gps_input(now)
                self._last_gps_input_at = now
            if self.cfg.emit_distance_sensor and (now - self._last_distance_at) >= 0.5:
                self._emit_distance_sensor(now)
                self._last_distance_at = now
            if self.cfg.emit_wind_cov and (now - self._last_wind_at) >= 1.0:
                self._emit_wind_cov(now)
                self._last_wind_at = now
            if self.cfg.emit_ahrs2 and (now - self._last_ahrs2_at) >= 0.1:
                self._emit_ahrs2(now)
                self._last_ahrs2_at = now
            time.sleep(0.02)

    # --- emitters -----------------------------------------------------------
    def _emit_gps_input(self, now: float) -> None:
        gps = self.state.snapshot_gps()
        # If stale, still send but with fix_type=0 so ArduPilot knows it's bad
        fix_type = gps.fix_type if not gps.stale(now, self.cfg.stale_timeout_s) else 0
        time_usec = int(now * 1e6)
        time_week = 0          # unknown / ignored
        time_week_ms = int((now % (7 * 24 * 3600)) * 1000)

        ignore = 0
        # Orca gives us vel/cog but separate; we compute NED from sog/cog
        crad = math.radians(gps.cog_deg)
        vn = gps.sog_mps * math.cos(crad)
        ve = gps.sog_mps * math.sin(crad)
        vd = 0.0

        horiz_acc = max(1.0, gps.hdop * 2.5)   # rough: HDOP × expected UERE
        vert_acc = max(1.0, gps.vdop * 2.5)
        speed_acc = 0.5
        try:
            self._mav.mav.gps_input_send(
                time_usec,                          # time since boot, µs
                0,                                  # gps_id
                ignore,                             # ignore_flags
                time_week_ms,                       # time_week_ms
                time_week,                          # time_week
                fix_type,                           # 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed
                int(gps.lat * 1e7),                 # lat_e7
                int(gps.lon * 1e7),                 # lon_e7
                float(gps.alt_m),                   # alt (m)
                float(gps.hdop),
                float(gps.vdop),
                float(vn), float(ve), float(vd),    # vel NED m/s
                float(speed_acc),
                float(horiz_acc),
                float(vert_acc),
                int(gps.satellites),
            )
        except Exception as e:
            log.debug("gps_input_send failed: %s", e)

    def _emit_distance_sensor(self, now: float) -> None:
        d = self.state.snapshot_depth()
        if d.updated_at == 0: return
        time_boot_ms = int((now % (2**31 / 1000)) * 1000)
        cm = int((d.meters_below_transducer + d.offset_m) * 100)
        try:
            self._mav.mav.distance_sensor_send(
                time_boot_ms,
                10,                          # min distance cm
                int(max(1000, d.range_m * 100)),  # max distance cm
                max(1, cm),                  # current distance cm
                2,                           # MAV_DISTANCE_SENSOR_INFRARED (no "sonar" enum exists)
                0,                           # id
                25,                          # MAV_SENSOR_ROTATION_PITCH_270 = downward
                0,                           # covariance (unknown)
            )
        except Exception as e:
            log.debug("distance_sensor_send failed: %s", e)

    def _emit_wind_cov(self, now: float) -> None:
        w = self.state.snapshot_wind()
        if w.updated_at == 0: return
        time_usec = int(now * 1e6)
        wx = w.speed_mps * math.cos(w.direction_rad)
        wy = w.speed_mps * math.sin(w.direction_rad)
        try:
            self._mav.mav.wind_cov_send(
                time_usec,
                wx, wy, 0.0,                 # wind_x,y,z m/s
                w.speed_mps,                 # var_horiz (placeholder)
                0.0,                         # var_vert
                0.0,                         # wind_alt
                0.0, 0.0,                    # horiz/vert accuracy
            )
        except Exception as e:
            log.debug("wind_cov_send failed: %s", e)

    def _emit_ahrs2(self, now: float) -> None:
        h = self.state.snapshot_heading()
        if h.updated_at == 0: return
        try:
            self._mav.mav.ahrs2_send(
                0.0, 0.0, float(h.true_rad),  # roll, pitch, yaw
                0.0,                           # altitude
                0, 0,                          # lat, lng (unknown here)
            )
        except Exception as e:
            log.debug("ahrs2_send failed: %s", e)
