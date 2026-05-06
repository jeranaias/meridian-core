"""Unit tests for PGN decoders. No external hardware needed."""
import math
import struct
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from meridian_orca.state import BoatState
from meridian_orca.decode.pgn import (
    decode, decode_129025, decode_129026, decode_129029,
    decode_127250, decode_128267, decode_130306,
)


def test_129025_position_rapid():
    # Sydney Harbour: lat -33.8688, lon 151.2093
    lat_e7 = int(-33.8688 * 1e7)
    lon_e7 = int(151.2093 * 1e7)
    payload = struct.pack("<ii", lat_e7, lon_e7)
    s = BoatState()
    assert decode_129025(payload, s) is True
    gps = s.snapshot_gps()
    assert abs(gps.lat - -33.8688) < 1e-6
    assert abs(gps.lon - 151.2093) < 1e-6


def test_129026_cog_sog():
    # SID=0, flags=0, COG = π/2 rad = 90° east, SOG = 5.00 m/s
    cog_raw = int(math.pi / 2 / 1e-4)
    sog_raw = 500
    payload = struct.pack("<BBHHH", 0, 0, cog_raw, sog_raw, 0xFFFF)
    s = BoatState()
    assert decode_129026(payload, s) is True
    gps = s.snapshot_gps()
    assert abs(gps.cog_deg - 90.0) < 0.1
    assert abs(gps.sog_mps - 5.00) < 0.01


def test_127250_heading():
    # Heading = π/4 rad = 45°, variation = 0.2 rad east
    hdg_raw = int(math.pi / 4 / 1e-4)
    var_raw = int(0.2 / 1e-4)
    dev_raw = 0
    payload = struct.pack("<BHhhB", 0, hdg_raw, dev_raw, var_raw, 0)
    s = BoatState()
    assert decode_127250(payload, s) is True
    hdg = s.snapshot_heading()
    assert abs(hdg.true_rad - math.pi / 4) < 1e-3
    assert abs(hdg.mag_variation_rad - 0.2) < 1e-3


def test_128267_depth():
    # Depth = 5.25 m, offset = 0.30 m, range = 100 m
    depth_raw = 525          # 5.25 m in 0.01 m units
    offset_raw = 300         # 0.30 m in 0.001 m units
    payload = struct.pack("<BIhB", 0, depth_raw, offset_raw, 100)
    s = BoatState()
    assert decode_128267(payload, s) is True
    d = s.snapshot_depth()
    assert abs(d.meters_below_transducer - 5.25) < 0.01
    assert abs(d.offset_m - 0.30) < 0.01


def test_130306_wind():
    # Wind speed = 8.00 m/s, direction = π rad (south), apparent
    speed_raw = 800
    angle_raw = int(math.pi / 1e-4)
    payload = struct.pack("<BHHBBB", 0, speed_raw, angle_raw, 2, 0xFF, 0xFF)
    s = BoatState()
    assert decode_130306(payload, s) is True
    w = s.snapshot_wind()
    assert abs(w.speed_mps - 8.00) < 0.01
    assert abs(w.direction_rad - math.pi) < 1e-3
    assert w.reference == "apparent"


def test_129025_unavailable_ignored():
    payload = struct.pack("<ii", 0x7FFFFFFF, 0x7FFFFFFF)
    s = BoatState()
    assert decode_129025(payload, s) is False
    assert s.snapshot_gps().lat == 0.0


def test_dispatch_via_decode():
    # Sanity check: the registry works
    payload = struct.pack("<ii", int(-33 * 1e7), int(151 * 1e7))
    s = BoatState()
    assert decode(129025, payload, s) is True
    assert decode(999999, b"", s) is False  # unknown PGN returns False


if __name__ == "__main__":
    import traceback
    tests = [v for k, v in globals().items() if k.startswith("test_")]
    passed = failed = 0
    for t in tests:
        try:
            t()
            passed += 1
            print(f"  PASS  {t.__name__}")
        except Exception as e:
            failed += 1
            print(f"  FAIL  {t.__name__}: {e}")
            traceback.print_exc()
    print(f"\n{passed}/{passed+failed} pass")
    sys.exit(0 if failed == 0 else 1)
