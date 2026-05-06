#!/usr/bin/env python3
"""
mnp-sitl-server.py — Jet boat SITL with MNP over WebSocket.

Runs jet boat physics and sends MNP telemetry that the Meridian GCS
understands natively. No MAVLink. Pure MNP.

Usage:
    pip install websockets
    python tools/mnp-sitl-server.py
    # GCS: http://localhost:8080 → connect ws://localhost:5760 (protocol: mnp)
"""

import asyncio
import math
import struct
import sys
import os
import logging

try:
    import websockets
except ImportError:
    print("pip install websockets")
    exit(1)

# Add tools/ to path for terrain module
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from terrain import TerrainDB

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("sitl")

# ── MNP Message IDs (must match gcs/js/mnp.js MSG enum) ──────

MSG_HEARTBEAT   = 0x01
MSG_ATTITUDE    = 0x02
MSG_POSITION    = 0x03
MSG_BATTERY     = 0x04
MSG_GPS_RAW     = 0x05
MSG_VFR_HUD     = 0x06
MSG_EKF_STATUS  = 0x07
MSG_PARAM_VALUE = 0x09
MSG_TRAFFIC     = 0x0A   # Threat track (one per message)
MSG_STATION     = 0x0B   # Active station-keeping state
MSG_AIS         = 0x0C   # AIS contact (richer than TRAFFIC: MMSI, nav status, dimensions)
MSG_PERCEPTION  = 0x0D   # Onboard-detected object not on AIS (buoy, debris, swimmer)
MSG_CONTROL     = 0x0E   # Commanded vs actual control outputs (rudder/throttle)

# Commands from GCS
CMD_ARM           = 0x80
CMD_DISARM        = 0x81
CMD_SET_STATION   = 0x83  # set station target (lat, lon, radius)
CMD_SPAWN_THREAT  = 0x84  # demo: spawn a threat at lat/lon with bearing/speed
CMD_SET_MODE      = 0x85
CMD_DIRECT_CONTROL= 0x86  # bench-test: direct rudder/throttle override in MODE_BENCH
CMD_SET_PARAM     = 0x87
CMD_GET_PARAM     = 0x88
CMD_CLEAR_THREATS = 0x8B  # clear all threats
CMD_AIS_SIM       = 0x8C  # 0x8C 00=disable AIS sim, 01=enable
CMD_SPAWN_PERCEPT = 0x8D  # demo: spawn a perception contact (debris)

# Mode IDs (matching tablet MODES array)
MODE_STATION_KEEP = 7  # active station hold with avoidance
MODE_BENCH        = 8  # bench-test: direct pass-through of commanded rudder/throttle

# Modes (index must match mnp.js MODES array)
MODE_STABILIZE = 0  # Manual
MODE_ALT_HOLD  = 1
MODE_LOITER    = 2
MODE_RTL       = 3
MODE_AUTO      = 4
MODE_LAND      = 5
MODE_GUIDED    = 6

# ── COBS ──────────────────────────────────────────────────────

def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    out.append(0)
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    out.append(0)  # delimiter
    return bytes(out)

def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0:
            break
        i += 1
        for _ in range(code - 1):
            if i < len(data):
                out.append(data[i])
                i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    if out and out[-1] == 0:
        out = out[:-1]
    return bytes(out)

# ── MNP Message Builders ─────────────────────────────────────
# Each must match the parser in gcs/js/mnp.js exactly.

# Vehicle class in top 4 bits of status byte:
# 0=copter, 1=plane, 2=rover, 3=boat, 4=sub
VCLASS_BOAT = 3 << 4

def mnp_heartbeat(armed: bool, mode_idx: int, status: int = 4, vehicle_class: int = VCLASS_BOAT) -> bytes:
    payload = struct.pack("<BBB", MSG_HEARTBEAT, 1 if armed else 0, mode_idx)
    payload += struct.pack("<B", (vehicle_class & 0xF0) | (status & 0x0F))
    return cobs_encode(payload)

def mnp_attitude(roll: float, pitch: float, yaw: float,
                 roll_spd: float = 0, pitch_spd: float = 0, yaw_spd: float = 0) -> bytes:
    payload = struct.pack("<B ffffff", MSG_ATTITUDE, roll, pitch, yaw, roll_spd, pitch_spd, yaw_spd)
    return cobs_encode(payload)

def mnp_position(lat: float, lon: float, alt: float, rel_alt: float,
                 vx: float, vy: float, vz: float, hdg: float) -> bytes:
    # lat/lon in degE7, alt in mm, vel in cm/s, hdg in cdeg
    payload = struct.pack("<B iiii hhh H", MSG_POSITION,
        int(lat * 1e7), int(lon * 1e7), int(alt * 1000), int(rel_alt * 1000),
        int(vx * 100), int(vy * 100), int(vz * 100), int(hdg * 100))
    return cobs_encode(payload)

def mnp_battery(voltage: float, current: float, remaining: int) -> bytes:
    # voltage in mV, current in cA, remaining in %
    payload = struct.pack("<B hh B", MSG_BATTERY,
        int(voltage * 1000), int(current * 100), max(0, min(100, remaining)))
    return cobs_encode(payload)

def mnp_gps_raw(fix: int, lat: float, lon: float, alt: float,
                hdop: int, vdop: int, sats: int) -> bytes:
    payload = struct.pack("<B B iii HH B", MSG_GPS_RAW,
        fix, int(lat * 1e7), int(lon * 1e7), int(alt * 1000),
        hdop, vdop, sats)
    return cobs_encode(payload)

def mnp_vfr_hud(airspeed: float, groundspeed: float, heading: int,
                throttle: int, alt: float, climb: float) -> bytes:
    payload = struct.pack("<B ff h H ff", MSG_VFR_HUD,
        airspeed, groundspeed, heading, throttle, alt, climb)
    return cobs_encode(payload)

def mnp_ekf(vel_var: float, pos_var: float, hgt_var: float,
            mag_var: float, terr_var: float, flags: int) -> bytes:
    payload = struct.pack("<B fffff H", MSG_EKF_STATUS,
        vel_var, pos_var, hgt_var, mag_var, terr_var, flags)
    return cobs_encode(payload)

def mnp_traffic(track_id: int, lat: float, lon: float, course_deg: float,
                speed_mps: float, vessel_type: int, name: str = "") -> bytes:
    """Encode TRAFFIC: id + track_id(u16) + lat_e7(i32) + lon_e7(i32) + course(cdeg u16)
       + speed(cm/s u16) + type(u8) + name_len(u8) + name_bytes."""
    name_bytes = name.encode("ascii")[:24]
    payload = struct.pack("<B H i i H H B B",
        MSG_TRAFFIC, track_id & 0xFFFF,
        int(lat * 1e7), int(lon * 1e7),
        int(course_deg * 100) & 0xFFFF,
        int(speed_mps * 100) & 0xFFFF,
        vessel_type & 0xFF,
        len(name_bytes))
    payload += name_bytes
    return cobs_encode(payload)

def mnp_station(active: int, lat: float, lon: float, radius_m: float,
                hold_dist_m: float, threat_count: int) -> bytes:
    """Encode STATION: id + active(u8) + lat_e7(i32) + lon_e7(i32)
       + radius_m(f32) + hold_dist_m(f32) + threat_count(u8)."""
    payload = struct.pack("<B B i i f f B",
        MSG_STATION, active & 0xFF,
        int(lat * 1e7), int(lon * 1e7),
        radius_m, hold_dist_m, threat_count & 0xFF)
    return cobs_encode(payload)

def mnp_ais(mmsi: int, lat: float, lon: float, cog_deg: float, sog_mps: float,
            heading_deg: float, nav_status: int, vessel_class: int,
            length_m: int, beam_m: int, name: str = "", call_sign: str = "") -> bytes:
    """Encode AIS: id + mmsi(u32) + lat_e7(i32) + lon_e7(i32)
       + cog(cdeg u16) + sog(cm/s u16) + hdg(cdeg u16)
       + nav_status(u8) + class(u8) + length(u8) + beam(u8)
       + name_len(u8) + name + call_len(u8) + call."""
    nb = name.encode("ascii", errors="replace")[:20]
    cb = call_sign.encode("ascii", errors="replace")[:8]
    payload = struct.pack("<B I i i H H H B B B B",
        MSG_AIS, mmsi & 0xFFFFFFFF,
        int(lat * 1e7), int(lon * 1e7),
        int(cog_deg * 100) & 0xFFFF,
        int(sog_mps * 100) & 0xFFFF,
        int(heading_deg * 100) & 0xFFFF,
        nav_status & 0xFF, vessel_class & 0xFF,
        length_m & 0xFF, beam_m & 0xFF)
    payload += struct.pack("<B", len(nb)) + nb
    payload += struct.pack("<B", len(cb)) + cb
    return cobs_encode(payload)

def mnp_perception(obj_id: int, lat: float, lon: float, obj_class: int,
                   confidence: float, heading_deg: float, speed_mps: float,
                   length_m: float, width_m: float, source: int = 0) -> bytes:
    """Encode PERCEPTION: id + obj_id(u16) + lat_e7(i32) + lon_e7(i32)
       + class(u8) + confidence(u8, 0-100) + heading(cdeg u16) + speed(cm/s u16)
       + length(dm u16) + width(dm u16) + source(u8).
       class: 0=unknown, 1=buoy, 2=debris, 3=swimmer, 4=small-craft, 5=whale, 6=kelp
       source: 0=camera, 1=lidar, 2=radar, 3=fusion."""
    payload = struct.pack("<B H i i B B H H H H B",
        MSG_PERCEPTION, obj_id & 0xFFFF,
        int(lat * 1e7), int(lon * 1e7),
        obj_class & 0xFF,
        max(0, min(100, int(confidence * 100))),
        int(heading_deg * 100) & 0xFFFF,
        int(speed_mps * 100) & 0xFFFF,
        int(length_m * 10) & 0xFFFF,
        int(width_m * 10) & 0xFFFF,
        source & 0xFF)
    return cobs_encode(payload)

def mnp_control(cmd_thr: float, act_thr: float, cmd_steer: float, act_steer: float,
                rudder_deg: float) -> bytes:
    """Encode CONTROL: id + cmd_thr(f32, 0-1) + act_thr(f32, 0-1)
       + cmd_steer(f32, -1..+1) + act_steer(f32, -1..+1) + rudder_deg(f32)."""
    payload = struct.pack("<B fffff",
        MSG_CONTROL, cmd_thr, act_thr, cmd_steer, act_steer, rudder_deg)
    return cobs_encode(payload)

def mnp_param_value(name: str, value: float, count: int = 0, index: int = 0) -> bytes:
    """Encode PARAM_VALUE: msg_id + name_len_varint + name_bytes + f32 value + u8 type + u16 count + u16 index."""
    name_bytes = name.encode("ascii")
    # postcard varint for name length (single byte for len < 0x80)
    payload = struct.pack("<B", MSG_PARAM_VALUE)
    payload += struct.pack("<B", len(name_bytes))  # length prefix (varint, fits in 1 byte)
    payload += name_bytes
    payload += struct.pack("<f", value)
    payload += struct.pack("<B", 0)  # param type (0 = f32)
    payload += struct.pack("<HH", count, index)
    return cobs_encode(payload)

# ── USV Parameter Store ──────────────────────────────────────
# Default PID values for the Vanguard-class jet boat USV.
# Values mirror the MeridianTuned param file.
DEFAULT_PARAMS = {
    # Steering rate PID
    "ATC_STR_RAT_P": 0.20,
    "ATC_STR_RAT_I": 0.10,
    "ATC_STR_RAT_D": 0.005,
    "ATC_STR_RAT_FF": 0.30,
    "ATC_STR_RAT_MAX": 120.0,
    "ATC_STR_ANG_P": 2.5,
    # Speed PID
    "ATC_SPEED_P": 0.20,
    "ATC_SPEED_I": 0.10,
    "ATC_SPEED_D": 0.0,
    "ATC_SPEED_FF": 0.40,
    # Cruise / Nav
    "CRUISE_SPEED": 3.0,
    "CRUISE_THROTTLE": 50.0,
    "WP_SPEED": 3.0,
    "WP_RADIUS": 2.0,
    "WP_PIVOT_ANGLE": 60.0,
    "WP_PIVOT_RATE": 90.0,
    "LOIT_RADIUS": 5.0,
    "NAVL1_PERIOD": 17.0,
    "NAVL1_DAMPING": 0.75,
    "ATC_TURN_MAX_G": 0.6,
    # Station-keeping defaults
    "STATION_RADIUS": 5.0,
    "AVOID_RADIUS": 30.0,
    "AVOID_TCPA": 30.0,
    "AVOID_GAIN": 1.5,
    "TERRAIN_AVOID": 1.0,
    "COLREG_LEVEL": 2.0,       # 0=off, 1=simple (head-on + cross-starboard), 2=full rules 13-17
    # Bench-test / rudder diagnostics
    "RUDDER_TRIM_DEG": 0.0,    # static trim offset (deg) — added to commanded nozzle
    "RUDDER_MAX_DEG": 20.0,    # mechanical limit (±deg)
    "RUDDER_SIGN": 1.0,        # -1 to invert steering direction
    # AIS simulator
    "AIS_SIM_ENABLED": 1.0,    # auto-spawn realistic vessel traffic
    "AIS_SIM_MAX": 8.0,        # max simultaneous AIS contacts
}

# ── Threat / AIS Track ───────────────────────────────────────

class Threat:
    """A dynamic obstacle with constant-velocity prediction."""
    def __init__(self, track_id, lat, lon, course_deg, speed_mps,
                 vessel_type=0, name=""):
        self.id = track_id
        self.lat = lat
        self.lon = lon
        self.course = math.radians(course_deg)  # heading in radians
        self.speed = speed_mps
        self.type = vessel_type   # 0=unknown, 1=cargo, 2=fishing, 3=pleasure, 4=power, 5=sailing
        self.name = name
        self.last_update = 0.0

    def step(self, dt):
        """Constant-velocity propagation."""
        if self.speed < 1e-3:
            return
        d_north = self.speed * math.cos(self.course) * dt
        d_east = self.speed * math.sin(self.course) * dt
        self.lat += d_north / 6371000.0 * (180 / math.pi)
        self.lon += d_east / (6371000.0 * math.cos(math.radians(self.lat))) * (180 / math.pi)

    def vel_ned(self):
        return (self.speed * math.cos(self.course),
                self.speed * math.sin(self.course))


# ── AIS Contact (realistic vessel metadata) ──────────────────

# Nav status codes (AIS standard)
AIS_UNDERWAY       = 0
AIS_AT_ANCHOR      = 1
AIS_NOT_UNDER_CMD  = 2
AIS_RESTRICTED     = 3
AIS_CONSTRAINED    = 4
AIS_MOORED         = 5
AIS_AGROUND        = 6
AIS_FISHING        = 7
AIS_SAILING        = 8

# Vessel class (AIS type-of-ship codes, collapsed)
VC_UNKNOWN   = 0
VC_CARGO     = 70   # cargo ship
VC_TANKER    = 80
VC_PASSENGER = 60
VC_FISHING   = 30
VC_TUG       = 52
VC_PILOT     = 50
VC_HSC       = 40   # high speed craft (ferry)
VC_SAILING   = 36
VC_PLEASURE  = 37
VC_MILITARY  = 35

class AISContact:
    """A simulated AIS-broadcasting vessel with waypoint-based path."""
    def __init__(self, mmsi, name, call_sign, lat, lon, cog_deg, sog_mps,
                 vessel_class, length_m, beam_m, waypoints=None, nav_status=AIS_UNDERWAY):
        self.mmsi = mmsi
        self.name = name
        self.call_sign = call_sign
        self.lat = lat
        self.lon = lon
        self.cog = math.radians(cog_deg)
        self.sog = sog_mps
        self.heading = self.cog   # for most vessels heading ≈ COG
        self.nav_status = nav_status
        self.vessel_class = vessel_class
        self.length = length_m
        self.beam = beam_m
        self.waypoints = waypoints or []
        self.wp_idx = 0

    def step(self, dt):
        """Advance the vessel, following its waypoints when set, else CV."""
        if self.nav_status in (AIS_AT_ANCHOR, AIS_MOORED, AIS_AGROUND):
            self.sog = 0.0
            return
        # Follow waypoints if we have them
        if self.waypoints and self.wp_idx < len(self.waypoints):
            wp = self.waypoints[self.wp_idx]
            dlat = wp[0] - self.lat
            dlon = wp[1] - self.lon
            dist = math.sqrt((dlat * 111320) ** 2
                             + (dlon * 111320 * math.cos(math.radians(self.lat))) ** 2)
            if dist < 30.0:
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    # Reverse: loop back
                    self.waypoints = list(reversed(self.waypoints))
                    self.wp_idx = 0
                return
            bearing = math.atan2(
                dlon * 111320 * math.cos(math.radians(self.lat)),
                dlat * 111320)
            # Smoothly turn toward bearing (max 2°/s for realism)
            err = bearing - self.cog
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            max_turn = math.radians(2.0) * dt
            self.cog += max(-max_turn, min(max_turn, err))
            # Keep cog in [0, 2π) to avoid overflow in wire format (u16 cdeg)
            self.cog %= 2 * math.pi
            self.heading = self.cog
        # Advance position along COG
        d_north = self.sog * math.cos(self.cog) * dt
        d_east = self.sog * math.sin(self.cog) * dt
        self.lat += d_north / 6371000.0 * (180 / math.pi)
        self.lon += d_east / (6371000.0 * math.cos(math.radians(self.lat))) * (180 / math.pi)


# ── AIS Simulator (auto-generates realistic Botany Bay traffic) ──

# Sydney Botany Bay shipping lane waypoints (approx)
BOTANY_BAY_LANES = {
    "channel_in":  [(-34.0050, 151.2300), (-33.9850, 151.2150), (-33.9650, 151.2050), (-33.9400, 151.1900)],
    "channel_out": [(-33.9400, 151.1900), (-33.9650, 151.2050), (-33.9850, 151.2150), (-34.0050, 151.2300)],
    "port_approach": [(-33.9950, 151.2250), (-33.9800, 151.2200), (-33.9700, 151.2150)],
    "fishing_grounds": [(-34.0100, 151.2400), (-34.0200, 151.2500), (-34.0000, 151.2600), (-33.9900, 151.2500)],
    "ferry_route": [(-33.9550, 151.2000), (-33.9750, 151.2100), (-33.9950, 151.2200)],
}

# Plausible vessel fleet around Sydney
AIS_FLEET = [
    # (name, call, vessel_class, length, beam, nav_status, lane, sog_range)
    ("MV PACIFIC DAWN",    "VHTL", VC_CARGO,     180, 28, AIS_UNDERWAY, "channel_in",  (4.0, 6.0)),
    ("MSC EMMA",           "3ELM", VC_CARGO,     250, 32, AIS_UNDERWAY, "channel_out", (5.0, 7.0)),
    ("FERRY COLLAROY",     "VKCY", VC_PASSENGER,  35,  9, AIS_UNDERWAY, "ferry_route", (6.0, 9.0)),
    ("F/V SEA HAWK",       "VKSH", VC_FISHING,    18,  5, AIS_FISHING,  "fishing_grounds", (2.0, 3.5)),
    ("F/V SOUTHERN STAR",  "VKSS", VC_FISHING,    22,  6, AIS_FISHING,  "fishing_grounds", (2.5, 3.8)),
    ("TUG RELIABLE",       "VKRL", VC_TUG,        28,  9, AIS_UNDERWAY, "port_approach", (3.0, 5.0)),
    ("PILOT BOAT 12",      "VKP1", VC_PILOT,      14,  4, AIS_UNDERWAY, "port_approach", (5.0, 8.0)),
    ("MY SERENITY",        "VJMS", VC_PLEASURE,   24,  6, AIS_UNDERWAY, "ferry_route", (3.0, 5.0)),
    ("SY WANDERER",        "VJSW", VC_SAILING,    12,  4, AIS_SAILING,  "ferry_route", (1.5, 3.0)),
    ("HMAS ARUNTA",        "VMA3", VC_MILITARY,  118, 15, AIS_UNDERWAY, "channel_out", (6.0, 10.0)),
]

class AISSimulator:
    """Spawns + maintains realistic AIS traffic around the operating area."""
    def __init__(self):
        self.contacts = {}        # mmsi -> AISContact
        self._next_mmsi = 503000000  # Australian MMSI range (503xxxxxx)
        self.enabled = True
        self.max_contacts = 8
        self._fleet_iter = 0
        import random
        self.rnd = random.Random(0x42)

    def spawn_one(self):
        """Spawn one vessel from the fleet pool, avoiding duplicates."""
        if len(self.contacts) >= self.max_contacts:
            return
        # Pick a vessel type we don't already have (or any if all are out)
        available = [v for v in AIS_FLEET
                     if not any(c.name == v[0] for c in self.contacts.values())]
        pool = available if available else AIS_FLEET
        spec = self.rnd.choice(pool)
        name, call, vclass, length, beam, status, lane, sog_range = spec
        waypoints = list(BOTANY_BAY_LANES[lane])
        # Randomize direction (half go reverse)
        if self.rnd.random() < 0.5:
            waypoints = list(reversed(waypoints))
        # Start somewhere along the route
        start_idx = self.rnd.randint(0, len(waypoints) - 1)
        start = waypoints[start_idx]
        # Perturb start slightly
        start = (start[0] + self.rnd.uniform(-0.0015, 0.0015),
                 start[1] + self.rnd.uniform(-0.0015, 0.0015))
        sog = self.rnd.uniform(*sog_range)
        next_wp = waypoints[(start_idx + 1) % len(waypoints)]
        dlat = next_wp[0] - start[0]
        dlon = next_wp[1] - start[1]
        cog = math.degrees(math.atan2(dlon * math.cos(math.radians(start[0])), dlat))
        if cog < 0: cog += 360
        mmsi = self._next_mmsi
        self._next_mmsi += 1
        contact = AISContact(mmsi, name, call, start[0], start[1], cog, sog,
                             vclass, length, beam,
                             waypoints=waypoints[start_idx:] + waypoints[:start_idx],
                             nav_status=status)
        self.contacts[mmsi] = contact

    def step(self, dt, boat_lat, boat_lon):
        """Advance all contacts + auto-spawn/despawn as needed."""
        if not self.enabled:
            return
        for c in self.contacts.values():
            c.step(dt)
        # Despawn anything more than 10km from own boat
        far = [m for m, c in self.contacts.items()
               if abs(c.lat - boat_lat) > 0.09 or abs(c.lon - boat_lon) > 0.09]
        for m in far:
            del self.contacts[m]
        # Spawn up to max
        while len(self.contacts) < self.max_contacts:
            self.spawn_one()

    def clear(self):
        self.contacts.clear()


# ── Perception (onboard-detected objects not on AIS) ─────────

class PerceptionContact:
    """An onboard-detected object (buoy, debris, swimmer, small craft not broadcasting AIS)."""
    OBJ_UNKNOWN = 0
    OBJ_BUOY = 1
    OBJ_DEBRIS = 2
    OBJ_SWIMMER = 3
    OBJ_SMALL_CRAFT = 4
    OBJ_WHALE = 5
    OBJ_KELP = 6

    def __init__(self, obj_id, lat, lon, obj_class, confidence,
                 heading_deg=0.0, speed_mps=0.0, length_m=1.0, width_m=1.0, source=0):
        self.id = obj_id
        self.lat = lat
        self.lon = lon
        self.obj_class = obj_class
        self.confidence = confidence
        self.heading = math.radians(heading_deg)
        self.speed = speed_mps
        self.length = length_m
        self.width = width_m
        self.source = source     # 0=camera, 1=lidar, 2=radar, 3=fusion
        self.first_seen = 0.0
        self.last_seen = 0.0
        self.stale_after = 8.0   # seconds

    def step(self, dt):
        if self.speed > 1e-3:
            d_north = self.speed * math.cos(self.heading) * dt
            d_east = self.speed * math.sin(self.heading) * dt
            self.lat += d_north / 6371000.0 * (180 / math.pi)
            self.lon += d_east / (6371000.0 * math.cos(math.radians(self.lat))) * (180 / math.pi)


class PerceptionSim:
    """Generates synthetic perception events near the boat for testing the overlay.
    In real life this is replaced by onboard camera/lidar detections."""
    def __init__(self):
        self.contacts = {}
        self._next_id = 1
        import random
        self.rnd = random.Random(0xBEEF)

    def step(self, dt, sim_time, boat_lat, boat_lon):
        # Decay / remove stale
        stale = [k for k, c in self.contacts.items() if sim_time - c.last_seen > c.stale_after]
        for k in stale:
            del self.contacts[k]
        # Step existing
        for c in self.contacts.values():
            c.step(dt)
        # Occasionally spawn new (roughly 1 per 15s)
        if self.rnd.random() < dt / 15.0 and len(self.contacts) < 4:
            # Pick random bearing + distance (30-200m)
            bearing = self.rnd.uniform(0, 2 * math.pi)
            dist = self.rnd.uniform(30, 200)
            lat = boat_lat + dist * math.cos(bearing) / 111320.0
            lon = boat_lon + dist * math.sin(bearing) / (111320.0 * math.cos(math.radians(boat_lat)))
            # Weighted class pick — mostly buoys/debris, rarely a swimmer
            r = self.rnd.random()
            if r < 0.45:
                oc, conf, length, width, spd = PerceptionContact.OBJ_BUOY, 0.95, 0.6, 0.6, 0.0
            elif r < 0.80:
                oc, conf, length, width, spd = PerceptionContact.OBJ_DEBRIS, 0.72, 1.5, 0.8, 0.3
            elif r < 0.88:
                oc, conf, length, width, spd = PerceptionContact.OBJ_SMALL_CRAFT, 0.84, 4.0, 1.6, self.rnd.uniform(1.0, 4.0)
            elif r < 0.94:
                oc, conf, length, width, spd = PerceptionContact.OBJ_KELP, 0.60, 3.0, 2.0, 0.0
            elif r < 0.98:
                oc, conf, length, width, spd = PerceptionContact.OBJ_SWIMMER, 0.88, 0.6, 0.4, 0.8
            else:
                oc, conf, length, width, spd = PerceptionContact.OBJ_WHALE, 0.65, 12.0, 3.0, 2.0
            hdg = math.degrees(self.rnd.uniform(0, 2 * math.pi))
            c = PerceptionContact(self._next_id, lat, lon, oc, conf, hdg, spd, length, width,
                                  source=self.rnd.choice([0, 1, 3]))
            c.first_seen = sim_time
            c.last_seen = sim_time
            self.contacts[self._next_id] = c
            self._next_id += 1
        # Re-confirm existing contacts (bump last_seen) with some chance
        for c in self.contacts.values():
            if self.rnd.random() < dt * 2.0:
                c.last_seen = sim_time

    def spawn_debris(self, lat, lon, obj_class=None):
        """Manually inject a contact from demo button."""
        if obj_class is None:
            obj_class = PerceptionContact.OBJ_DEBRIS
        c = PerceptionContact(self._next_id, lat, lon, obj_class, 0.85,
                              heading_deg=self.rnd.uniform(0, 360),
                              speed_mps=self.rnd.uniform(0, 1.0),
                              length_m=1.5, width_m=1.0, source=0)
        c.first_seen = 0.0
        c.last_seen = 1e9  # never stale for demo
        self.contacts[self._next_id] = c
        self._next_id += 1
        return c


# ── Jet Boat Physics ─────────────────────────────────────────

class JetBoat:
    def __init__(self, lat, lon, heading_deg, terrain=None):
        self.lat = lat
        self.lon = lon
        self.speed = 0.0
        self.heading = math.radians(heading_deg)
        self.yaw_rate = 0.0
        self.thrust = 0.0
        self.nozzle = 0.0
        self.armed = False
        self.mode = MODE_STABILIZE
        self.battery = 92.0
        self.voltage = 25.1
        self.current_n = 0.2  # mild current
        self.current_e = 0.15
        self.terrain = terrain
        self.depth = None       # current depth (negative = water)
        self.grounded = False   # true if run aground
        # Tunable parameters (overridden by SitlServer at startup)
        self.steer_p = 0.6
        self.steer_d = 0.35
        self.steer_ff = 0.0
        self.speed_p = 0.20
        self.speed_ff = 0.40
        self.wp_radius = 2.0
        self.wp_speed = 3.0
        self.cruise_throttle = 0.55
        self.loiter_radius = 5.0
        # Station keeping
        self.station = None              # (lat, lon) or None
        self.station_radius = 5.0
        self.avoid_radius = 30.0
        self.avoid_tcpa = 30.0
        self.avoid_gain = 1.5
        self.terrain_avoid_on = True
        self.threats = []                # list of Threat objects
        self.evading = False             # true if currently dodging a threat
        self.waypoints = []
        self.wp_idx = 0
        self.loiter_center = None
        self.home = None
        # Rudder / throttle diagnostics
        self.rudder_trim_deg = 0.0
        self.rudder_max_deg = 20.0
        self.rudder_sign = 1.0
        # Last commanded vs actual control outputs (broadcast every loop)
        self.cmd_throttle = 0.0
        self.cmd_steer = 0.0
        self.actual_throttle = 0.0       # actual throttle (after dynamics)
        self.actual_steer = 0.0          # commanded steer (for now; servo limits not enforced yet)
        # Bench-test direct control overrides
        self.bench_throttle = 0.0
        self.bench_steer = 0.0
        # COLREG level (0=off, 1=simple, 2=full rules 13-17)
        self.colreg_level = 2.0

    def step(self, throttle, steering, dt):
        # Record commanded outputs pre-armament gate for diagnostics/bench
        self.cmd_throttle = max(0.0, min(1.0, throttle))
        self.cmd_steer = max(-1.0, min(1.0, steering))
        if not self.armed:
            self.speed *= 0.95
            self.yaw_rate *= 0.9
            self.actual_throttle = 0.0
            self.actual_steer = 0.0
            # Rudder still responds visually when disarmed in bench mode; otherwise hold
            if self.mode == MODE_BENCH:
                target_n = (self.cmd_steer * self.rudder_sign) * math.radians(
                    max(0.0, min(90.0, self.rudder_max_deg)))
                target_n += math.radians(self.rudder_trim_deg)
                rate = 1.5 * dt
                d = target_n - self.nozzle
                self.nozzle += max(-rate, min(rate, d))
                self.actual_steer = self.cmd_steer
            return

        # Spool — 5m Vanguard USV jet pump
        target = self.cmd_throttle * 450.0  # ~450N jet thrust on 5m hull
        self.thrust += (dt / 1.0) * (target - self.thrust)  # bigger engine, slower spool
        self.actual_throttle = self.thrust / 450.0  # normalized 0..1 actual
        # Nozzle — hydraulic actuator on larger vessel, with trim + sign + configurable limit
        steer_cmd = self.cmd_steer * self.rudder_sign
        target_n = steer_cmd * math.radians(max(0.0, min(90.0, self.rudder_max_deg)))
        target_n += math.radians(self.rudder_trim_deg)
        rate = 1.5 * dt  # slower actuator
        d = target_n - self.nozzle
        self.nozzle += max(-rate, min(rate, d))
        self.actual_steer = self.cmd_steer
        # Forces — 5m hull, ~180kg, CRUISE_SPEED=3 m/s at 50% throttle
        # Drag 50: at 50% throttle (225N), speed = sqrt(225/50) ≈ 2.1, spool → ~3 m/s
        # Full throttle: sqrt(450/50) ≈ 3.0, spool → ~4.5 m/s
        fwd = self.thrust * math.cos(self.nozzle) - 50.0 * self.speed * abs(self.speed)
        # Jet nozzle steering authority scales with water flow — no speed, no steering
        steer_authority = min(1.0, abs(self.speed) / 1.5)
        torque = self.thrust * math.sin(self.nozzle) * 1.5 * steer_authority - 25.0 * self.yaw_rate * abs(self.yaw_rate)
        self.speed += fwd / 180.0 * dt  # 180kg vessel
        self.yaw_rate += torque / 40.0 * dt  # larger moment of inertia
        self.heading += self.yaw_rate * dt
        self.heading %= 2 * math.pi
        # Position
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        new_lat = self.lat + vn * dt / 6371000.0 * (180 / math.pi)
        new_lon = self.lon + ve * dt / (6371000.0 * math.cos(math.radians(self.lat))) * (180 / math.pi)
        # Terrain check — grounding
        if self.terrain:
            self.depth = self.terrain.get_depth(new_lat, new_lon)
            if self.depth is not None and self.depth > -0.5:
                # Too shallow or on land — grounded
                if not self.grounded:
                    log.warning(f"GROUNDED at ({new_lat:.6f},{new_lon:.6f}), depth={self.depth:.1f}m")
                    self.grounded = True
                self.speed *= 0.5  # rapid deceleration
                self.yaw_rate *= 0.5
                # Don't update position — stuck
            else:
                self.grounded = False
                self.lat = new_lat
                self.lon = new_lon
        else:
            self.lat = new_lat
            self.lon = new_lon
        # Battery
        self.battery = max(0, self.battery - 0.0003 * dt * (throttle + 0.1))
        self.voltage = 22.0 + self.battery / 100.0 * 3.2

    def _shore_avoidance(self, steer, throttle):
        """Look ahead along current heading; steer away from shallow water."""
        if not self.terrain or self.speed < 0.3:
            return steer, throttle
        # Probe 50m, 100m, 200m ahead
        for probe_dist in [50, 100, 200]:
            probe_lat = self.lat + probe_dist * math.cos(self.heading) / 111320
            probe_lon = self.lon + probe_dist * math.sin(self.heading) / (111320 * math.cos(math.radians(self.lat)))
            d = self.terrain.get_depth(probe_lat, probe_lon)
            if d is not None and d > -2.0:
                # Shallow or land ahead — check which side is deeper
                left_hdg = self.heading + math.radians(45)
                right_hdg = self.heading - math.radians(45)
                left_lat = self.lat + 80 * math.cos(left_hdg) / 111320
                left_lon = self.lon + 80 * math.sin(left_hdg) / (111320 * math.cos(math.radians(self.lat)))
                right_lat = self.lat + 80 * math.cos(right_hdg) / 111320
                right_lon = self.lon + 80 * math.sin(right_hdg) / (111320 * math.cos(math.radians(self.lat)))
                d_left = self.terrain.get_depth(left_lat, left_lon) or 0
                d_right = self.terrain.get_depth(right_lat, right_lon) or 0
                # Steer toward deeper water, strength inversely proportional to probe distance
                urgency = 1.0 - (probe_dist - 50) / 200.0  # 1.0 at 50m, 0.25 at 200m
                if d_left < d_right:
                    steer = max(-1, min(1, steer + 0.8 * urgency))
                else:
                    steer = max(-1, min(1, steer - 0.8 * urgency))
                # Slow down if close
                if probe_dist <= 100:
                    throttle *= 0.5
                log.debug(f"Shore avoid: depth {d:.1f}m at {probe_dist}m ahead, "
                          f"left={d_left:.1f} right={d_right:.1f}")
                break
        return steer, throttle

    # ── Station Keeping with Dynamic Obstacle Avoidance ──────
    def _ll_to_ne(self, lat, lon, ref_lat, ref_lon):
        """Convert lat/lon to local NE meters relative to reference."""
        n = (lat - ref_lat) * 111320.0
        e = (lon - ref_lon) * 111320.0 * math.cos(math.radians(ref_lat))
        return n, e

    def _ne_to_ll(self, n, e, ref_lat):
        """Convert local NE meters back to delta lat/lon."""
        return n / 111320.0, e / (111320.0 * math.cos(math.radians(ref_lat)))

    def _threat_metrics(self, threat):
        """Compute distance, CPA, TCPA between own boat and a threat.
        Returns (distance_now_m, cpa_m, tcpa_s, bearing_rel_rad).
        bearing_rel_rad: angle from own bow to threat (-pi to +pi)."""
        # Own state
        own_n = 0.0
        own_e = 0.0
        own_vn, own_ve = self.vel_ned()
        # Threat state in local frame
        tn, te = self._ll_to_ne(threat.lat, threat.lon, self.lat, self.lon)
        tvn, tve = threat.vel_ned()
        # Relative kinematics
        dn, de = tn - own_n, te - own_e
        dvn, dve = tvn - own_vn, tve - own_ve
        dist = math.sqrt(dn*dn + de*de)
        v2 = dvn*dvn + dve*dve
        if v2 < 1e-6:
            tcpa = 0.0
            cpa = dist
        else:
            tcpa = -(dn*dvn + de*dve) / v2
            cpa_n = dn + dvn * max(0.0, tcpa)
            cpa_e = de + dve * max(0.0, tcpa)
            cpa = math.sqrt(cpa_n*cpa_n + cpa_e*cpa_e)
        # Bearing from own bow to threat (positive = starboard)
        threat_bearing = math.atan2(de, dn)
        bearing_rel = threat_bearing - self.heading
        while bearing_rel > math.pi: bearing_rel -= 2 * math.pi
        while bearing_rel < -math.pi: bearing_rel += 2 * math.pi
        return dist, cpa, tcpa, bearing_rel

    def _station_keep(self, dt):
        """Active station-keeping with multi-threat APF avoidance + COLREG bias."""
        sx, sy = self.station
        # Distance to station
        sn, se = self._ll_to_ne(sx, sy, self.lat, self.lon)
        station_dist = math.sqrt(sn*sn + se*se)

        # ── 1. Attraction vector toward station ──
        if station_dist > 0.1:
            attract_n = sn / station_dist
            attract_e = se / station_dist
            # Magnitude scales with distance: gentle near station, full far away
            attract_mag = min(1.0, station_dist / max(self.station_radius, 1.0))
        else:
            attract_n = 0.0
            attract_e = 0.0
            attract_mag = 0.0

        # ── 2. Repulsion vectors from threats ──
        repel_n = 0.0
        repel_e = 0.0
        urgent_threats = 0
        colreg_bias = 0.0  # positive = bias to starboard (right turn)

        for t in self.threats:
            dist, cpa, tcpa, bearing_rel = self._threat_metrics(t)
            if dist < 1e-3:
                continue
            # Threat is concerning if: close OR converging fast
            close = dist < self.avoid_radius
            converging = tcpa > 0 and tcpa < self.avoid_tcpa and cpa < self.avoid_radius
            if not (close or converging):
                continue
            urgent_threats += 1
            # Repulsion vector points from threat to own
            tn, te = self._ll_to_ne(t.lat, t.lon, self.lat, self.lon)
            # Direction away from threat (own - threat); in local frame own is at origin
            away_n = -tn / dist
            away_e = -te / dist
            # Strength: inverse-square distance, plus inverse TCPA boost when converging
            strength = (self.avoid_radius / max(dist, 1.0)) ** 2
            if converging and tcpa > 0:
                strength += (self.avoid_tcpa / max(tcpa, 1.0)) ** 1.5
            repel_n += away_n * strength
            repel_e += away_e * strength
            # ── COLREG rules (International Regulations for Preventing Collisions at Sea) ──
            # bearing_rel: angle from own bow to threat (positive = starboard)
            # Classify encounter geometry using bearing_rel + threat's COG relative to our heading
            threat_rel_cog = t.course - self.heading
            while threat_rel_cog > math.pi: threat_rel_cog -= 2 * math.pi
            while threat_rel_cog < -math.pi: threat_rel_cog += 2 * math.pi
            # Reciprocal COG = heading at us (head-on signature)
            reciprocal = abs(abs(threat_rel_cog) - math.pi) < math.radians(20)
            same_dir = abs(threat_rel_cog) < math.radians(45)

            if self.colreg_level >= 2.0:
                # Rule 14: Head-on — both turn to starboard to pass port-to-port
                if abs(bearing_rel) < math.radians(15) and reciprocal:
                    colreg_bias += 0.9 * strength   # strong right
                # Rule 13: Overtaking — we're approaching from astern (threat ahead, same direction)
                elif abs(bearing_rel) < math.radians(112.5) and same_dir and self.speed > t.speed:
                    # Pass on either side; default to starboard for consistency
                    colreg_bias += 0.5 * strength
                # Rule 15: Crossing from starboard → WE are give-way, turn right to pass astern
                elif math.radians(10) < bearing_rel < math.radians(112.5) and not reciprocal and not same_dir:
                    colreg_bias += 0.75 * strength  # strong give-way turn right
                # Rule 15/17: Crossing from port → WE are stand-on (hold course unless near)
                elif math.radians(-112.5) < bearing_rel < math.radians(-10) and not reciprocal and not same_dir:
                    if dist < self.avoid_radius * 0.6:
                        # Very close → Rule 17(b) action to avoid; turn starboard (away from crossing vessel)
                        colreg_bias += 0.4 * strength
                    # else: stand on, no bias added
                # Rule 13: Being overtaken (threat astern, same direction, faster than us)
                elif abs(bearing_rel) > math.radians(112.5) and same_dir and t.speed > self.speed:
                    # We are stand-on: hold course
                    pass
            elif self.colreg_level >= 1.0:
                # Simple bias (previous behavior)
                if abs(bearing_rel) < math.radians(15):
                    colreg_bias += 0.6 * strength
                elif math.radians(15) < bearing_rel < math.radians(112.5):
                    colreg_bias += 0.3 * strength

        # ── 3. Terrain repulsion (shoreline awareness) ──
        terrain_n = 0.0
        terrain_e = 0.0
        if self.terrain_avoid_on and self.terrain is not None:
            # Probe 8 directions at 30m
            for k in range(8):
                ang = 2 * math.pi * k / 8
                pn, pe = 30.0 * math.cos(ang), 30.0 * math.sin(ang)
                dlat, dlon = self._ne_to_ll(pn, pe, self.lat)
                d = self.terrain.get_depth(self.lat + dlat, self.lon + dlon)
                if d is not None and d > -2.0:
                    # Land/shallow detected — push away from this direction
                    factor = (d + 2.0) * 2.0  # 0 at -2m, 4 at 0m
                    terrain_n -= math.cos(ang) * factor
                    terrain_e -= math.sin(ang) * factor

        # ── 4. Combine vectors ──
        # When urgent threats exist, repulsion dominates. Otherwise, attraction.
        if urgent_threats > 0:
            self.evading = True
            tot_n = (repel_n * self.avoid_gain
                     + attract_n * attract_mag * 0.5
                     + terrain_n * 1.5)
            tot_e = (repel_e * self.avoid_gain
                     + attract_e * attract_mag * 0.5
                     + terrain_e * 1.5)
        else:
            self.evading = False
            tot_n = attract_n * attract_mag + terrain_n * 1.5
            tot_e = attract_e * attract_mag + terrain_e * 1.5

        mag = math.sqrt(tot_n*tot_n + tot_e*tot_e)
        if mag < 0.05 and not self.evading:
            # We're at station with no threats — gentle rudder corrections to hold heading
            # Use minimal throttle just to keep steerage way against current
            # Counter current by aiming bow into estimated current direction
            cur_dir = math.atan2(-self.current_e, -self.current_n)  # face into current
            err = cur_dir - self.heading
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            steer = err * 0.4 - self.yaw_rate * 0.3
            return 0.12, max(-1, min(1, steer))  # idle throttle for steerage

        # Desired course = direction of combined vector
        desired_course = math.atan2(tot_e, tot_n)
        # Apply COLREG bias (starboard rotation when threats from bow/starboard)
        if colreg_bias > 0:
            desired_course -= min(math.radians(45), colreg_bias * 0.3)

        err = desired_course - self.heading
        while err > math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi

        # Throttle: more throttle when farther from station or actively evading
        if self.evading:
            throttle = self.cruise_throttle * 0.85  # firm but controlled
        elif station_dist > self.station_radius * 3:
            throttle = self.cruise_throttle
        elif station_dist > self.station_radius:
            throttle = self.cruise_throttle * 0.7
        else:
            throttle = max(0.15, station_dist * 0.04)

        # Steering with PD + tunable gains
        steer = err * self.steer_p - self.yaw_rate * self.steer_d + self.steer_ff * (err / math.pi)
        # Always overlay shore probe avoidance as a final safety net
        steer, throttle = self._shore_avoidance(max(-1, min(1, steer)), throttle)
        return throttle, max(-1, min(1, steer))

    def autopilot(self, dt):
        # Bench-test mode: pass through direct commands verbatim.
        # This runs regardless of armed state so you can verify rudder actuation.
        if self.mode == MODE_BENCH:
            return self.bench_throttle, self.bench_steer

        if self.grounded:
            # Try to back off — reverse slowly
            return -0.15, 0.0

        if self.mode == MODE_AUTO and self.wp_idx < len(self.waypoints):
            wp = self.waypoints[self.wp_idx]
            dlat = wp[0] - self.lat
            dlon = wp[1] - self.lon
            dist = math.sqrt((dlat * 111320)**2 + (dlon * 111320 * math.cos(math.radians(self.lat)))**2)
            bearing = math.atan2(dlon * math.cos(math.radians(self.lat)), dlat)
            if dist < self.wp_radius:  # tunable WP_RADIUS
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.loiter_center = (self.lat, self.lon)
                    self.mode = MODE_LOITER
                    log.info("Mission complete — loitering")
                    return 0.15, 0.0
                log.info(f"WP {self.wp_idx} reached → WP {self.wp_idx + 1}")
            err = bearing - self.heading
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            # Throttle profile uses tunable cruise_throttle
            if dist > 20:
                throttle = self.cruise_throttle
            elif dist > 8:
                throttle = self.cruise_throttle * 0.82
            else:
                throttle = max(0.25, dist * 0.04)
            # PD steering with tunable gains (ATC_STR_RAT_P/D + FF)
            steer = err * self.steer_p - self.yaw_rate * self.steer_d + self.steer_ff * (err / math.pi)
            steer, throttle = self._shore_avoidance(max(-1, min(1, steer)), throttle)
            return throttle, max(-1, min(1, steer))

        elif self.mode == MODE_STATION_KEEP and self.station is not None:
            return self._station_keep(dt)

        elif self.mode == MODE_RTL and self.home:
            dlat = self.home[0] - self.lat
            dlon = self.home[1] - self.lon
            dist = math.sqrt((dlat * 111320)**2 + (dlon * 111320 * math.cos(math.radians(self.lat)))**2)
            if dist < 3.0:
                self.loiter_center = self.home
                self.mode = MODE_LOITER
                log.info("RTL complete — loitering at home")
                return 0.05, 0.0
            bearing = math.atan2(dlon * math.cos(math.radians(self.lat)), dlat)
            err = bearing - self.heading
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            if dist > 20:
                throttle = 0.55
            elif dist > 8:
                throttle = 0.45
            else:
                throttle = max(0.25, dist * 0.04)
            steer = err * 0.6 - self.yaw_rate * 0.35
            steer, throttle = self._shore_avoidance(max(-1, min(1, steer)), throttle)
            return throttle, max(-1, min(1, steer))

        elif self.mode == MODE_LOITER and self.loiter_center:
            cx, cy = self.loiter_center
            dlat = cx - self.lat
            dlon = cy - self.lon
            dist = math.sqrt((dlat * 111320)**2 + (dlon * 111320 * math.cos(math.radians(self.lat)))**2)
            if dist < 3.0:
                return 0.05, 0.0
            bearing = math.atan2(dlon * math.cos(math.radians(self.lat)), dlat)
            err = bearing - self.heading
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            steer = err * 0.8 - self.yaw_rate * 0.3
            return min(0.35, dist * 0.06), max(-1, min(1, steer))

        return 0.0, 0.0

    def hdg_deg(self):
        return (math.degrees(self.heading) % 360 + 360) % 360

    def gnd_speed(self):
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        return math.sqrt(vn * vn + ve * ve)

    def vel_ned(self):
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        return vn, ve

# ── Server ────────────────────────────────────────────────────

class SitlServer:
    def __init__(self):
        # Load terrain/bathymetry
        cache_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "..", "data", "terrain")
        self.terrain = TerrainDB(cache_dir)
        # Ensure we have data for the operating area
        self.terrain.ensure_region(-34.02, 151.15, -33.93, 151.25)

        # Middle of Botany Bay, Sydney — actually in the water
        self.boat = JetBoat(-33.9700, 151.2000, 45.0, terrain=self.terrain)
        self.boat.waypoints = []
        self.clients = set()
        self.tick = 0

        # AIS traffic simulator and perception sim (created BEFORE params applied
        # so that _apply_param_live can reach them safely)
        self.ais_sim = AISSimulator()
        self.perception_sim = PerceptionSim()
        self.sim_time = 0.0

        # USV parameter store (for PID tuning UI)
        self.params = dict(DEFAULT_PARAMS)
        self._param_names = list(self.params.keys())  # stable ordering for index
        # Apply initial values to boat
        for name, value in self.params.items():
            self._apply_param_live(name, value)

        # Threat tracking
        self._next_threat_id = 1
        self._threat_names = ["Cargo Vessel ACME", "F/V Sea Hawk", "M/V Lighthouse",
                              "P/V Patrol 12", "Sailing Vessel Wanderer",
                              "Tug Reliable", "Survey Vessel Atlas"]

    def _apply_param_live(self, name, value):
        """Apply a parameter change to live boat physics where applicable."""
        b = self.boat
        if name == "ATC_STR_RAT_P":
            b.steer_p = value
        elif name == "ATC_STR_RAT_D":
            b.steer_d = value
        elif name == "ATC_STR_RAT_FF":
            b.steer_ff = value
        elif name == "ATC_SPEED_P":
            b.speed_p = value
        elif name == "ATC_SPEED_FF":
            b.speed_ff = value
        elif name == "WP_RADIUS":
            b.wp_radius = value
        elif name == "WP_SPEED":
            b.wp_speed = value
        elif name == "CRUISE_THROTTLE":
            b.cruise_throttle = value / 100.0
        elif name == "LOIT_RADIUS":
            b.loiter_radius = value
        elif name == "STATION_RADIUS":
            b.station_radius = value
        elif name == "AVOID_RADIUS":
            b.avoid_radius = value
        elif name == "AVOID_TCPA":
            b.avoid_tcpa = value
        elif name == "AVOID_GAIN":
            b.avoid_gain = value
        elif name == "TERRAIN_AVOID":
            b.terrain_avoid_on = value > 0.5
        elif name == "COLREG_LEVEL":
            b.colreg_level = value
        elif name == "RUDDER_TRIM_DEG":
            b.rudder_trim_deg = value
        elif name == "RUDDER_MAX_DEG":
            b.rudder_max_deg = value
        elif name == "RUDDER_SIGN":
            b.rudder_sign = 1.0 if value >= 0 else -1.0
        elif name == "AIS_SIM_ENABLED":
            self.ais_sim.enabled = value > 0.5
        elif name == "AIS_SIM_MAX":
            self.ais_sim.max_contacts = int(max(0, min(32, value)))

    async def handle_client(self, ws):
        self.clients.add(ws)
        log.info(f"GCS connected ({len(self.clients)} clients)")
        try:
            async for msg in ws:
                if isinstance(msg, bytes):
                    await self.handle_cmd(msg, ws)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)
            log.info(f"GCS disconnected ({len(self.clients)} clients)")

    async def handle_cmd(self, data, ws=None):
        try:
            payload = cobs_decode(data)
            if len(payload) < 1:
                return
            cmd = payload[0]
            if cmd == CMD_ARM:
                self.boat.armed = True
                if self.boat.home is None:
                    self.boat.home = (self.boat.lat, self.boat.lon)
                    log.info(f"Home set: ({self.boat.lat:.6f}, {self.boat.lon:.6f})")
                log.info("ARMED")
            elif cmd == CMD_DISARM:
                self.boat.armed = False
                log.info("DISARMED")
            elif cmd == CMD_SET_MODE:
                if len(payload) > 1:
                    self.boat.mode = payload[1]
                    names = {0: 'MANUAL', 2: 'LOITER', 3: 'RTL', 4: 'AUTO',
                             7: 'STATION_KEEP', 8: 'BENCH'}
                    log.info(f"Mode → {names.get(self.boat.mode, self.boat.mode)}")
                    if self.boat.mode == MODE_AUTO:
                        self.boat.wp_idx = 0
                        self.boat.loiter_center = None
                        log.info(f"Mission: {len(self.boat.waypoints)} waypoints")
            elif cmd == 0x8A:  # CMD_MISSION_COUNT
                if len(payload) >= 3:
                    count = struct.unpack("<H", payload[1:3])[0]
                    self.boat.waypoints = [None] * count
                    log.info(f"Mission upload: expecting {count} waypoints")
            elif cmd == 0x89:  # CMD_MISSION_ITEM
                if len(payload) >= 11:
                    seq = struct.unpack("<H", payload[1:3])[0]
                    lat = struct.unpack("<i", payload[3:7])[0] / 1e7
                    lon = struct.unpack("<i", payload[7:11])[0] / 1e7
                    # Validate waypoint against terrain
                    depth = self.terrain.get_depth(lat, lon)
                    if depth is not None and depth > -1.0:
                        log.warning(f"  WP {seq+1}: REJECTED — {'LAND' if depth >= 0 else 'TOO SHALLOW'} "
                                    f"({lat:.6f}, {lon:.6f}), depth={depth:.1f}m")
                    else:
                        if seq < len(self.boat.waypoints):
                            self.boat.waypoints[seq] = (lat, lon)
                            depth_str = f", depth={depth:.1f}m" if depth is not None else ""
                            log.info(f"  WP {seq+1}: ({lat:.6f}, {lon:.6f}){depth_str}")
                    # Check if mission is complete
                    if all(wp is not None for wp in self.boat.waypoints):
                        log.info(f"Mission upload complete: {len(self.boat.waypoints)} waypoints")
                        # Validate path between waypoints
                        wps = self.boat.waypoints
                        prev = (self.boat.lat, self.boat.lon)
                        for i, wp in enumerate(wps):
                            ok, blat, blon, bdepth = self.terrain.check_path(
                                prev[0], prev[1], wp[0], wp[1], step_m=100)
                            if not ok:
                                log.warning(f"  Path {i}→{i+1}: crosses land/shallow at "
                                            f"({blat:.6f},{blon:.6f}), depth={bdepth:.1f}m")
                            prev = wp
            elif cmd == CMD_GET_PARAM:
                # payload: [0x88, name_len(1B), name_bytes...]
                if len(payload) >= 2:
                    name_len = payload[1]
                    if len(payload) >= 2 + name_len:
                        name = payload[2:2 + name_len].decode("ascii", errors="ignore")
                        if name == "":
                            # Empty name = list-all request → broadcast all params
                            count = len(self._param_names)
                            for idx, pname in enumerate(self._param_names):
                                msg = mnp_param_value(pname, self.params[pname], count, idx)
                                if ws is not None:
                                    await ws.send(msg)
                            log.info(f"PARAM_LIST: sent {count} params")
                        elif name in self.params:
                            count = len(self._param_names)
                            idx = self._param_names.index(name)
                            msg = mnp_param_value(name, self.params[name], count, idx)
                            if ws is not None:
                                await ws.send(msg)
                        else:
                            log.warning(f"PARAM_GET: unknown '{name}'")
            elif cmd == CMD_SET_PARAM:
                # payload: [0x87, name_len(1B), name_bytes..., f32 value]
                if len(payload) >= 2:
                    name_len = payload[1]
                    if len(payload) >= 2 + name_len + 4:
                        name = payload[2:2 + name_len].decode("ascii", errors="ignore")
                        value = struct.unpack("<f", payload[2 + name_len:2 + name_len + 4])[0]
                        if name in self.params:
                            self.params[name] = value
                            log.info(f"PARAM_SET: {name} = {value}")
                            # Apply select params live to the boat physics
                            self._apply_param_live(name, value)
                            # Echo back so the GCS confirms
                            count = len(self._param_names)
                            idx = self._param_names.index(name)
                            msg = mnp_param_value(name, value, count, idx)
                            if ws is not None:
                                await ws.send(msg)
                        else:
                            log.warning(f"PARAM_SET: unknown '{name}'")
            elif cmd == CMD_SET_STATION:
                # payload: [0x83, lat_e7(i32), lon_e7(i32), radius(f32)]
                if len(payload) >= 13:
                    lat = struct.unpack("<i", payload[1:5])[0] / 1e7
                    lon = struct.unpack("<i", payload[5:9])[0] / 1e7
                    radius = struct.unpack("<f", payload[9:13])[0]
                    # Validate: must be water
                    depth = self.terrain.get_depth(lat, lon)
                    if depth is not None and depth > -1.0:
                        log.warning(f"STATION REJECTED — {'LAND' if depth >= 0 else 'TOO SHALLOW'} "
                                    f"({lat:.6f}, {lon:.6f}), depth={depth:.1f}m")
                    else:
                        self.boat.station = (lat, lon)
                        self.boat.station_radius = max(2.0, radius)
                        self.boat.mode = MODE_STATION_KEEP
                        log.info(f"STATION SET: ({lat:.6f}, {lon:.6f}), radius={radius:.1f}m")
            elif cmd == CMD_SPAWN_THREAT:
                # payload: [0x84, lat_e7(i32), lon_e7(i32), course_cdeg(u16), speed_cm_s(u16), type(u8)]
                if len(payload) >= 14:
                    lat = struct.unpack("<i", payload[1:5])[0] / 1e7
                    lon = struct.unpack("<i", payload[5:9])[0] / 1e7
                    course = struct.unpack("<H", payload[9:11])[0] / 100.0
                    speed = struct.unpack("<H", payload[11:13])[0] / 100.0
                    vtype = payload[13]
                    name = self._threat_names[(self._next_threat_id - 1) % len(self._threat_names)]
                    threat = Threat(self._next_threat_id, lat, lon, course, speed, vtype, name)
                    self.boat.threats.append(threat)
                    self._next_threat_id += 1
                    log.info(f"THREAT SPAWNED #{threat.id}: {name} "
                             f"@ ({lat:.6f},{lon:.6f}) course={course:.0f}° speed={speed:.1f}m/s")
            elif cmd == CMD_CLEAR_THREATS:
                n = len(self.boat.threats)
                self.boat.threats = []
                log.info(f"THREATS CLEARED ({n})")
            elif cmd == CMD_DIRECT_CONTROL:
                # payload: [0x86, throttle_pct(i16 signed, -100..+100), steer_pct(i16 signed, -100..+100)]
                if len(payload) >= 5:
                    thr_pct = struct.unpack("<h", payload[1:3])[0]
                    str_pct = struct.unpack("<h", payload[3:5])[0]
                    self.boat.bench_throttle = max(-1.0, min(1.0, thr_pct / 100.0))
                    self.boat.bench_steer = max(-1.0, min(1.0, str_pct / 100.0))
                    # Auto-enter bench mode on first direct-control command
                    if self.boat.mode != MODE_BENCH:
                        self.boat.mode = MODE_BENCH
                        log.info("Mode → BENCH (direct control)")
            elif cmd == CMD_AIS_SIM:
                # payload: [0x8C, enabled(u8)]
                if len(payload) >= 2:
                    self.ais_sim.enabled = payload[1] != 0
                    if not self.ais_sim.enabled:
                        self.ais_sim.clear()
                    self.params["AIS_SIM_ENABLED"] = 1.0 if self.ais_sim.enabled else 0.0
                    log.info(f"AIS sim → {'ON' if self.ais_sim.enabled else 'OFF'}")
            elif cmd == CMD_SPAWN_PERCEPT:
                # payload: [0x8D, lat_e7(i32), lon_e7(i32), class(u8)]
                if len(payload) >= 10:
                    lat = struct.unpack("<i", payload[1:5])[0] / 1e7
                    lon = struct.unpack("<i", payload[5:9])[0] / 1e7
                    oc = payload[9]
                    self.perception_sim.spawn_debris(lat, lon, oc)
                    log.info(f"PERCEPTION spawned class={oc} at ({lat:.6f},{lon:.6f})")
        except Exception as e:
            log.debug(f"cmd parse error: {e}")

    async def broadcast(self, data):
        if self.clients:
            await asyncio.gather(
                *[ws.send(data) for ws in self.clients],
                return_exceptions=True
            )

    async def sim_loop(self):
        dt = 0.02  # 50Hz
        while True:
            b = self.boat
            self.sim_time += dt
            # Step threats forward (constant velocity)
            for t in b.threats:
                t.step(dt)
            # Auto-prune threats that have left the operating area
            b.threats = [t for t in b.threats
                         if abs(t.lat - b.lat) < 0.05 and abs(t.lon - b.lon) < 0.05]
            # Step AIS + perception sims
            self.ais_sim.step(dt, b.lat, b.lon)
            self.perception_sim.step(dt, self.sim_time, b.lat, b.lon)

            thr, steer = b.autopilot(dt)
            b.step(thr, steer, dt)
            self.tick += 1

            # 10Hz telemetry
            if self.tick % 5 == 0 and self.clients:
                vn, ve = b.vel_ned()

                await self.broadcast(mnp_heartbeat(b.armed, b.mode))
                await self.broadcast(mnp_attitude(0.0, 0.0, b.heading, 0.0, 0.0, b.yaw_rate))
                await self.broadcast(mnp_position(b.lat, b.lon, 0.0, 0.0, vn, ve, 0.0, b.hdg_deg()))
                await self.broadcast(mnp_battery(b.voltage, 18.0 if b.armed else 0.2, int(b.battery)))
                await self.broadcast(mnp_gps_raw(3, b.lat, b.lon, 0.0, 90, 120, 14))
                await self.broadcast(mnp_vfr_hud(b.gnd_speed(), b.gnd_speed(), int(b.hdg_deg()), int(thr * 100), 0.0, 0.0))
                await self.broadcast(mnp_ekf(0.05, 0.08, 0.03, 0.02, 0.01, 0x1FF))

                # Control output (commanded vs actual rudder/throttle) — diagnostic for bench test
                await self.broadcast(mnp_control(
                    b.cmd_throttle, b.actual_throttle,
                    b.cmd_steer, b.actual_steer,
                    math.degrees(b.nozzle)))

                # Broadcast threat tracks (one message per threat)
                for t in b.threats:
                    await self.broadcast(mnp_traffic(
                        t.id, t.lat, t.lon, math.degrees(t.course),
                        t.speed, t.type, t.name))

                # Broadcast AIS contacts (one message per contact)
                for c in self.ais_sim.contacts.values():
                    await self.broadcast(mnp_ais(
                        c.mmsi, c.lat, c.lon,
                        math.degrees(c.cog), c.sog,
                        math.degrees(c.heading),
                        c.nav_status, c.vessel_class,
                        c.length, c.beam,
                        c.name, c.call_sign))

                # Broadcast perception contacts
                for pc in self.perception_sim.contacts.values():
                    await self.broadcast(mnp_perception(
                        pc.id, pc.lat, pc.lon, pc.obj_class,
                        pc.confidence, math.degrees(pc.heading),
                        pc.speed, pc.length, pc.width, pc.source))

                # Broadcast station state
                if b.station is not None:
                    sn = (b.station[0] - b.lat) * 111320.0
                    se = (b.station[1] - b.lon) * 111320.0 * math.cos(math.radians(b.lat))
                    hold_dist = math.sqrt(sn*sn + se*se)
                    await self.broadcast(mnp_station(
                        1 if b.mode == MODE_STATION_KEEP else 0,
                        b.station[0], b.station[1],
                        b.station_radius, hold_dist, len(b.threats)))
                else:
                    await self.broadcast(mnp_station(0, 0.0, 0.0, 0.0, 0.0, len(b.threats)))

            # Status every 5s
            if self.tick % 250 == 0:
                wp_str = (f"WP {b.wp_idx+1}/{len(b.waypoints)}" if b.mode == MODE_AUTO
                          else "STATION-KEEP" if b.mode == MODE_STATION_KEEP
                          else "BENCH" if b.mode == MODE_BENCH
                          else "LOITER" if b.mode == MODE_LOITER else "idle")
                depth_str = f" depth={b.depth:.1f}m" if b.depth is not None else ""
                ground_str = " GROUNDED" if b.grounded else ""
                evade_str = " EVADING" if b.evading else ""
                threat_str = f" threats={len(b.threats)}" if b.threats else ""
                ais_str = f" ais={len(self.ais_sim.contacts)}" if self.ais_sim.contacts else ""
                perc_str = f" perc={len(self.perception_sim.contacts)}" if self.perception_sim.contacts else ""
                log.info(f"{'ARM' if b.armed else 'DSRM'} hdg={b.hdg_deg():.0f}° spd={b.gnd_speed():.1f}m/s "
                         f"batt={b.battery:.0f}%{depth_str}{ground_str}{evade_str}{threat_str}{ais_str}{perc_str} {wp_str} "
                         f"({b.lat:.6f},{b.lon:.6f})")

            await asyncio.sleep(dt)

    async def run(self, port=5760):
        log.info("═══════════════════════════════════════════")
        log.info("  Meridian SITL — Jet Boat (MNP)")
        log.info(f"  Position: Botany Bay, Sydney (in the water)")
        log.info(f"  Mission: A → B then loiter")
        log.info(f"  WebSocket: ws://0.0.0.0:{port}")
        log.info("═══════════════════════════════════════════")
        log.info("Connect GCS → Settings → Connection → ws://localhost:5760")
        log.info("Then: arm → set mode AUTO → watch it go")
        log.info("")

        async with websockets.serve(self.handle_client, "0.0.0.0", port):
            await self.sim_loop()

if __name__ == "__main__":
    try:
        asyncio.run(SitlServer().run())
    except KeyboardInterrupt:
        log.info("Stopped.")
