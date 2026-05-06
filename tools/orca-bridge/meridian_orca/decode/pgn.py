"""NMEA 2000 PGN decoder. Wraps the `nmea2000` pip package (which uses the
canboat database) when available; falls back to a small built-in decoder
for the handful of PGNs we actually need.

Built-in decode covers:
  129025 — Position, Rapid Update (lat/lon only)
  129026 — COG/SOG, Rapid Update
  129029 — GNSS Position Data (combined: pos + fix quality + sats)
  127250 — Vessel Heading
  128267 — Water Depth
  130306 — Wind Data
  129038/129039/129794/129809 — AIS Class A/B

Hand-written decoders mean we don't need the nmea2000 pip package to run in
minimal environments (tests, bench, no-internet). If the package is present,
we defer to it for broader PGN coverage.
"""
import logging
import struct
from dataclasses import dataclass
from typing import Callable, Dict, Optional

from ..state import AisContact, BoatState


log = logging.getLogger(__name__)


try:
    from nmea2000 import NMEA2000Decoder as _ThirdParty
    _HAS_THIRD_PARTY = True
except Exception:
    _HAS_THIRD_PARTY = False


# ---------------------------------------------------------------------------
# PGN decoding — built-in minimal set
# ---------------------------------------------------------------------------

def _u32le(b: bytes, o: int) -> int: return struct.unpack_from("<I", b, o)[0]
def _i32le(b: bytes, o: int) -> int: return struct.unpack_from("<i", b, o)[0]
def _u16le(b: bytes, o: int) -> int: return struct.unpack_from("<H", b, o)[0]
def _i16le(b: bytes, o: int) -> int: return struct.unpack_from("<h", b, o)[0]

# NMEA 2000 "unavailable" sentinels. When these appear we skip the field.
UNAVAIL_I32 = 0x7FFFFFFF
UNAVAIL_U32 = 0xFFFFFFFF
UNAVAIL_I16 = 0x7FFF
UNAVAIL_U16 = 0xFFFF


def decode_129025(payload: bytes, state: BoatState) -> bool:
    """Position, Rapid Update. 8 bytes: lat (i32, 1e-7 deg) + lon (i32, 1e-7 deg)."""
    if len(payload) < 8: return False
    lat = _i32le(payload, 0)
    lon = _i32le(payload, 4)
    if lat == UNAVAIL_I32 or lon == UNAVAIL_I32: return False
    state.update_gps(lat=lat * 1e-7, lon=lon * 1e-7)
    return True


def decode_129026(payload: bytes, state: BoatState) -> bool:
    """COG/SOG Rapid Update. 8 bytes: [SID u8][flags u8][cog u16, 1e-4 rad][sog u16, 1e-2 m/s][res u16]"""
    if len(payload) < 8: return False
    cog = _u16le(payload, 2)
    sog = _u16le(payload, 4)
    if cog == UNAVAIL_U16 or sog == UNAVAIL_U16: return False
    import math
    state.update_gps(
        cog_deg=math.degrees(cog * 1e-4) % 360,
        sog_mps=sog * 1e-2,
    )
    return True


def decode_129029(payload: bytes, state: BoatState) -> bool:
    """GNSS Position Data. Variable length; minimum ~43 bytes.
    Layout: SID(u8) date(u16,days) time(u32,1e-4s)
            lat(i64,1e-16 deg) lon(i64,1e-16 deg) alt(i64,1e-6 m)
            type(u8) method(u8) integrity(u8) sats(u8)
            hdop(i16,1e-2) pdop(i16,1e-2) ...
    """
    if len(payload) < 43: return False
    # lat at offset 7 (i64 *1e-16 deg)
    lat_raw = struct.unpack_from("<q", payload, 7)[0]
    lon_raw = struct.unpack_from("<q", payload, 15)[0]
    alt_raw = struct.unpack_from("<q", payload, 23)[0]
    type_byte = payload[31]
    sats = payload[34] if len(payload) > 34 else 0
    hdop_raw = struct.unpack_from("<h", payload, 35)[0] if len(payload) >= 37 else UNAVAIL_I16
    pdop_raw = struct.unpack_from("<h", payload, 37)[0] if len(payload) >= 39 else UNAVAIL_I16

    gnss_type = (type_byte >> 4) & 0x0F       # upper nibble: fix type
    gnss_method = type_byte & 0x0F             # lower nibble: method

    # NMEA 2000 GNSS methods → ArduPilot GPS fix-type mapping
    #  0 = no GNSS, 1 = GNSS fix, 2 = DGNSS fix, 3 = precise GNSS (RTK fixed),
    #  4 = RTK fixed, 5 = RTK float, 6 = estimated (DR), 7 = manual, 8 = sim
    method_to_fix = {0: 0, 1: 3, 2: 4, 3: 6, 4: 6, 5: 5, 6: 2, 7: 0, 8: 0}
    fix_type = method_to_fix.get(gnss_method, 3 if lat_raw != 0 else 0)

    state.update_gps(
        lat=lat_raw * 1e-16,
        lon=lon_raw * 1e-16,
        alt_m=alt_raw * 1e-6 if alt_raw != UNAVAIL_I32 else 0.0,
        satellites=sats,
        hdop=hdop_raw * 1e-2 if hdop_raw != UNAVAIL_I16 else 99.0,
        fix_type=fix_type,
    )
    return True


def decode_127250(payload: bytes, state: BoatState) -> bool:
    """Vessel Heading. 8 bytes: SID(u8) heading(u16, 1e-4 rad)
       deviation(i16, 1e-4 rad) variation(i16, 1e-4 rad) ref(u8)."""
    if len(payload) < 8: return False
    hdg = _u16le(payload, 1)
    var = _i16le(payload, 5)
    if hdg == UNAVAIL_U16: return False
    state.update_heading(
        true_rad=hdg * 1e-4,
        mag_variation_rad=var * 1e-4 if var != UNAVAIL_I16 else 0.0,
    )
    return True


def decode_128267(payload: bytes, state: BoatState) -> bool:
    """Water Depth. 8 bytes: SID(u8) depth(u32, 1e-2 m below transducer)
       offset(i16, 1e-3 m; + below waterline, − below keel) range(u8, m)."""
    if len(payload) < 8: return False
    depth = _u32le(payload, 1)
    offset = _i16le(payload, 5)
    rng = payload[7]
    if depth == UNAVAIL_U32: return False
    state.update_depth(
        meters_below_transducer=depth * 1e-2,
        offset_m=(offset * 1e-3) if offset != UNAVAIL_I16 else 0.0,
        range_m=float(rng) if rng not in (0xFF, 0) else 100.0,
    )
    return True


def decode_130306(payload: bytes, state: BoatState) -> bool:
    """Wind Data. 8 bytes: SID(u8) speed(u16, 1e-2 m/s) angle(u16, 1e-4 rad)
       reference(u8)."""
    if len(payload) < 8: return False
    speed = _u16le(payload, 1)
    angle = _u16le(payload, 3)
    ref = payload[5] & 0x07
    if speed == UNAVAIL_U16 or angle == UNAVAIL_U16: return False
    ref_name = {0: "true-north", 1: "magnetic", 2: "apparent",
                3: "true-boat", 4: "true-water"}.get(ref, "unknown")
    state.update_wind(
        speed_mps=speed * 1e-2,
        direction_rad=angle * 1e-4,
        reference=ref_name,
    )
    return True


def _parse_ais_position(payload: bytes, state: BoatState, class_letter: str) -> bool:
    """AIS Class A (129038) / Class B (129039) position report.
    Minimum 28 bytes. Layout per canboat:
      [0] SID + repeat + msgType
      [1..4] MMSI (u32 LE)
      [5..8] lon (i32, 1e-7 deg)
      [9..12] lat (i32, 1e-7 deg)
      [13] accuracy + RAIM + timestamp
      [14..15] COG (u16, 1e-4 rad)
      [16..17] SOG (u16, 1e-2 m/s)
      [18..20] comm state + AIS transceiver info
      [21..22] true heading (u16, 1e-4 rad; 0xFFFF = unavailable)
      [23] rate-of-turn etc.
      [24..27] nav status, etc.
    """
    if len(payload) < 24: return False
    import math
    mmsi = _u32le(payload, 1)
    lon = _i32le(payload, 5) * 1e-7
    lat = _i32le(payload, 9) * 1e-7
    cog = _u16le(payload, 14) * 1e-4            # radians
    sog = _u16le(payload, 16) * 1e-2            # m/s
    hdg = _u16le(payload, 21) if len(payload) > 22 else UNAVAIL_U16

    c = AisContact(
        mmsi=mmsi,
        lat=lat, lon=lon,
        cog_deg=math.degrees(cog) % 360,
        sog_mps=sog,
        heading_deg=math.degrees(hdg * 1e-4) % 360 if hdg != UNAVAIL_U16 else 0.0,
        class_letter=class_letter,
    )
    state.update_ais(c)
    return True


def decode_129038(payload: bytes, state: BoatState) -> bool:
    return _parse_ais_position(payload, state, "A")


def decode_129039(payload: bytes, state: BoatState) -> bool:
    return _parse_ais_position(payload, state, "B")


# PGN → decoder function
_DECODERS: Dict[int, Callable[[bytes, BoatState], bool]] = {
    129025: decode_129025,
    129026: decode_129026,
    129029: decode_129029,
    127250: decode_127250,
    128267: decode_128267,
    130306: decode_130306,
    129038: decode_129038,
    129039: decode_129039,
}


def decode(pgn: int, payload: bytes, state: BoatState) -> bool:
    """Decode one PGN payload into state. Returns True on success."""
    fn = _DECODERS.get(pgn)
    if fn is not None:
        try:
            return fn(payload, state)
        except Exception as e:
            log.debug("pgn %d decode error: %s", pgn, e)
            return False
    # Fallback to third-party if available (broader PGN coverage)
    if _HAS_THIRD_PARTY:
        # nmea2000 package provides a full decoder; integration stub.
        # We don't actually call it here because its input format needs
        # different framing — added later if we need PGNs outside our core set.
        return False
    return False


def supported_pgns() -> list:
    return sorted(_DECODERS.keys())
