#!/usr/bin/env python3
"""
preflight-check.py — Pre-flight verification for the Vanguard USV lake test.

Runs a series of checks against the ground station setup and reports
green/yellow/red status for each.  Run this BEFORE putting the boat in
the water.

Usage:
    python3 preflight-check.py --serial /dev/ttyACM0 --baud 115200 --ws-port 5761
    python3 preflight-check.py --tcp 127.0.0.1:5760 --ws-port 5761
    python3 preflight-check.py --lat 36.6 --lon -121.9  # just check terrain

The tool performs these checks:

  1. Python dependencies (websockets, pyserial)
  2. MAVLink source reachable (serial/TCP/UDP)
  3. MAVLink heartbeats arriving at expected rate
  4. GPS fix type (>= 3 required for mission mode)
  5. Satellite count (>= 6 recommended)
  6. Battery voltage (within configured range)
  7. Terrain data cached for the operating area
  8. WebSocket bridge reachable from browser
  9. HTTP server serving the tablet app

Green = ready. Yellow = warning. Red = do NOT fly.
"""

import argparse
import asyncio
import os
import socket
import struct
import sys
import time
from pathlib import Path

# ── ANSI colors (degrade gracefully on Windows cmd without colorama) ──

RESET = "\033[0m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
BOLD = "\033[1m"
DIM = "\033[2m"

def _enable_windows_colors():
    if sys.platform == "win32":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
        except Exception:
            pass

_enable_windows_colors()


class Check:
    def __init__(self, name):
        self.name = name
        self.status = "pending"  # pending, pass, warn, fail
        self.message = ""

    def passed(self, msg=""):
        self.status = "pass"
        self.message = msg
        return self

    def warn(self, msg):
        self.status = "warn"
        self.message = msg
        return self

    def failed(self, msg):
        self.status = "fail"
        self.message = msg
        return self

    def render(self):
        icon = {
            "pending": f"{DIM}...{RESET}",
            "pass":    f"{GREEN}[OK]{RESET}",
            "warn":    f"{YELLOW}[!]{RESET} ",
            "fail":    f"{RED}[X]{RESET} ",
        }[self.status]
        color = {"pending": DIM, "pass": GREEN, "warn": YELLOW, "fail": RED}[self.status]
        line = f"  {icon} {self.name}"
        if self.message:
            line += f" {DIM}—{RESET} {color}{self.message}{RESET}"
        return line


class PreflightRunner:
    def __init__(self, args):
        self.args = args
        self.checks = []
        self.mavlink_source = None
        self.heartbeats = 0
        self.gps_fix = 0
        self.satellites = 0
        self.battery_voltage = 0.0
        self.battery_pct = 0
        self.armed = False
        self.mode = None

    def section(self, title):
        print(f"\n{BOLD}{title}{RESET}")

    def add(self, check):
        self.checks.append(check)
        print(check.render())
        return check

    # ── Check 1: Python deps ──
    def check_deps(self):
        self.section("1. Python dependencies")
        ws = Check("websockets")
        try:
            import websockets as _w
            ws.passed(f"v{_w.__version__}")
        except ImportError:
            ws.failed("missing (pip install websockets)")
        self.add(ws)

        ser = Check("pyserial")
        try:
            import serial as _s
            ser.passed(f"v{_s.__version__}")
        except ImportError:
            if self.args.serial:
                ser.failed("missing (pip install pyserial)")
            else:
                ser.warn("not installed (needed only for --serial mode)")
        self.add(ser)

        np = Check("numpy + scipy (for terrain)")
        try:
            import numpy as _n
            import scipy as _sc
            np.passed(f"numpy v{_n.__version__}, scipy v{_sc.__version__}")
        except ImportError:
            np.failed("missing (pip install numpy scipy)")
        self.add(np)

    # ── Check 2-6: MAVLink source and telemetry ──
    def check_mavlink_source(self):
        self.section("2. MAVLink source connection")
        if self.args.serial:
            check = Check(f"Serial {self.args.serial} @ {self.args.baud}")
            try:
                import serial
                s = serial.Serial(self.args.serial, self.args.baud, timeout=1)
                s.reset_input_buffer()
                # Read for 2 seconds
                data = bytearray()
                start = time.time()
                while time.time() - start < 2:
                    n = s.in_waiting
                    if n > 0:
                        data.extend(s.read(n))
                    time.sleep(0.01)
                s.close()
                if len(data) > 0:
                    check.passed(f"received {len(data)} bytes in 2s")
                    self._parse_mavlink(bytes(data))
                else:
                    check.failed("no data received in 2s (check radio/baud)")
            except Exception as e:
                check.failed(f"{type(e).__name__}: {e}")
            self.add(check)

        elif self.args.tcp:
            host, port = self.args.tcp.split(":")
            port = int(port)
            check = Check(f"TCP {host}:{port}")
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3)
                s.connect((host, port))
                s.settimeout(0.1)
                data = bytearray()
                start = time.time()
                while time.time() - start < 2:
                    try:
                        chunk = s.recv(4096)
                        if chunk:
                            data.extend(chunk)
                    except socket.timeout:
                        pass
                s.close()
                if len(data) > 0:
                    check.passed(f"received {len(data)} bytes in 2s")
                    self._parse_mavlink(bytes(data))
                else:
                    check.failed("no data received (is the SITL/bridge running?)")
            except Exception as e:
                check.failed(f"{type(e).__name__}: {e}")
            self.add(check)

        elif self.args.udp:
            host, port = self.args.udp.split(":")
            port = int(port)
            check = Check(f"UDP {host}:{port}")
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind((host, port))
                s.settimeout(0.1)
                data = bytearray()
                start = time.time()
                while time.time() - start < 3:
                    try:
                        chunk, _ = s.recvfrom(4096)
                        data.extend(chunk)
                    except socket.timeout:
                        pass
                s.close()
                if len(data) > 0:
                    check.passed(f"received {len(data)} bytes in 3s")
                    self._parse_mavlink(bytes(data))
                else:
                    check.failed("no data received (is MAVProxy forwarding?)")
            except Exception as e:
                check.failed(f"{type(e).__name__}: {e}")
            self.add(check)

        else:
            self.add(Check("MAVLink source").warn("no --serial/--tcp/--udp specified, skipping telemetry checks"))

    def _parse_mavlink(self, data):
        """Parse MAVLink v2 frames for telemetry extraction."""
        i = 0
        while i < len(data) - 12:
            if data[i] != 0xFD:
                i += 1
                continue
            plen = data[i + 1]
            if i + 10 + plen + 2 > len(data):
                break
            msg_id = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
            payload = data[i + 10:i + 10 + plen]

            if msg_id == 0:  # HEARTBEAT
                self.heartbeats += 1
                if len(payload) >= 8:
                    custom_mode = struct.unpack("<I", payload[0:4])[0]
                    base_mode = payload[6]
                    self.armed = (base_mode & 0x80) != 0
                    self.mode = custom_mode
            elif msg_id == 24 and len(payload) >= 30:  # GPS_RAW_INT
                self.gps_fix = payload[28]
                self.satellites = payload[29]
            elif msg_id == 1 and len(payload) >= 31:  # SYS_STATUS
                self.battery_voltage = struct.unpack("<H", payload[14:16])[0] / 1000.0
                self.battery_pct = struct.unpack("<b", payload[30:31])[0]

            i += 10 + plen + 2

    def check_telemetry(self):
        if not (self.args.serial or self.args.tcp or self.args.udp):
            return

        self.section("3. Telemetry health")

        hb = Check("Heartbeats")
        if self.heartbeats == 0:
            hb.failed("no HEARTBEAT messages received")
        elif self.heartbeats < 2:
            hb.warn(f"only {self.heartbeats} heartbeats in 2s (expect 2+ at 1 Hz)")
        else:
            hb.passed(f"{self.heartbeats} received")
        self.add(hb)

        gps = Check("GPS fix type")
        if self.gps_fix == 0:
            gps.warn("no GPS_RAW_INT received yet (may need more time)")
        elif self.gps_fix < 3:
            gps.failed(f"fix type {self.gps_fix} (need >= 3 for 3D fix)")
        elif self.gps_fix >= 4:
            gps.passed(f"fix type {self.gps_fix} (DGPS or better)")
        else:
            gps.passed(f"fix type {self.gps_fix} (3D fix)")
        self.add(gps)

        sats = Check("Satellites")
        if self.satellites == 0:
            sats.warn("GPS data not yet received")
        elif self.satellites < 6:
            sats.failed(f"{self.satellites} satellites (need >= 6 for reliable position)")
        elif self.satellites < 10:
            sats.warn(f"{self.satellites} satellites (more is better)")
        else:
            sats.passed(f"{self.satellites} satellites")
        self.add(sats)

        bat = Check("Battery voltage")
        if self.battery_voltage == 0:
            bat.warn("SYS_STATUS not yet received")
        elif self.battery_voltage < 22.0:
            bat.failed(f"{self.battery_voltage:.1f}V (LOW — charge before flight)")
        elif self.battery_voltage < 24.0:
            bat.warn(f"{self.battery_voltage:.1f}V (consider charging)")
        else:
            bat.passed(f"{self.battery_voltage:.1f}V")
        self.add(bat)

    # ── Check 7: Terrain ──
    def check_terrain(self):
        self.section("4. Terrain / bathymetry")
        cache_dir = Path(__file__).parent.parent / "data" / "terrain"
        check = Check("Terrain cache directory")
        if not cache_dir.exists():
            check.failed(f"{cache_dir} does not exist")
            self.add(check)
            return
        tiles = list(cache_dir.glob("*.npz"))
        if not tiles:
            check.failed("no cached terrain tiles")
            self.add(check)
            return
        check.passed(f"{len(tiles)} tile(s) cached ({sum(t.stat().st_size for t in tiles) // 1024} KB total)")
        self.add(check)

        # Query a specific point if provided
        if self.args.lat is not None and self.args.lon is not None:
            sys.path.insert(0, str(Path(__file__).parent))
            try:
                from terrain import TerrainDB
                db = TerrainDB(str(cache_dir))
                depth = db.get_depth(self.args.lat, self.args.lon)
                depth_check = Check(f"Depth at ({self.args.lat}, {self.args.lon})")
                if depth is None:
                    depth_check.failed("no terrain data covers this point — fetch with: "
                                       f"python tools/terrain.py --lat {self.args.lat} --lon {self.args.lon} --radius 50")
                elif depth > 0:
                    depth_check.failed(f"{depth:.1f}m (LAND — not water!)")
                elif depth > -1.0:
                    depth_check.warn(f"{depth:.1f}m (SHALLOW — verify before launch)")
                else:
                    depth_check.passed(f"{depth:.1f}m (water)")
                self.add(depth_check)
            except Exception as e:
                self.add(Check("Terrain query").failed(f"{type(e).__name__}: {e}"))

    # ── Check 8: Bridge WebSocket + telemetry ──
    def check_bridge_ws(self):
        if not self.args.ws_port:
            return
        self.section("5. WebSocket bridge")
        check = Check(f"WebSocket on port {self.args.ws_port}")
        try:
            import websockets
            async def verify():
                uri = f"ws://127.0.0.1:{self.args.ws_port}"
                async with websockets.connect(uri, open_timeout=2) as ws:
                    total_bytes = 0
                    heartbeats = 0
                    start = time.time()
                    while time.time() - start < 3:
                        try:
                            data = await asyncio.wait_for(ws.recv(), timeout=1)
                            total_bytes += len(data)
                            for i in range(len(data) - 12):
                                if data[i] == 0xFD:
                                    msg_id = data[i+7] | (data[i+8] << 8) | (data[i+9] << 16)
                                    if msg_id == 0:
                                        heartbeats += 1
                        except asyncio.TimeoutError:
                            pass
                    return total_bytes, heartbeats
            total, hb = asyncio.run(verify())
            if total == 0:
                check.warn("bridge reachable but no MAVLink data flowing — is the upstream source alive?")
            elif hb == 0:
                check.warn(f"{total} bytes flowing but no heartbeats yet (may need more time)")
            else:
                check.passed(f"{hb} heartbeats / {total} bytes in 3s — bridge is streaming")
        except ImportError:
            check.warn("websockets not installed — can't verify data flow")
        except Exception as e:
            check.warn(f"not reachable ({type(e).__name__}) — start bridge with: "
                       f"python tools/mavlink-ws-bridge.py --serial ... --ws-port {self.args.ws_port}")
        self.add(check)

    # ── Check 9: HTTP server ──
    def check_http(self):
        if not self.args.http_port:
            return
        self.section("6. Tablet app HTTP server")
        check = Check(f"HTTP on port {self.args.http_port}")
        try:
            import urllib.request
            with urllib.request.urlopen(f"http://127.0.0.1:{self.args.http_port}/mission.html", timeout=2) as r:
                content = r.read(4096).lower()
                if b"mission" in content or b"meridian" in content:
                    check.passed("mission.html reachable")
                else:
                    check.warn("server responds but mission.html content unexpected")
        except Exception as e:
            check.warn(f"not reachable ({type(e).__name__}) — start with: "
                       f"cd gcs && python -m http.server {self.args.http_port}")
        self.add(check)

    # ── Summary ──
    def print_summary(self):
        print()
        passed = sum(1 for c in self.checks if c.status == "pass")
        warned = sum(1 for c in self.checks if c.status == "warn")
        failed = sum(1 for c in self.checks if c.status == "fail")
        total = len(self.checks)

        bar = f"{GREEN}{passed}{RESET} passed"
        if warned:
            bar += f", {YELLOW}{warned}{RESET} warnings"
        if failed:
            bar += f", {RED}{failed}{RESET} failed"
        print(f"{BOLD}Summary:{RESET} {bar} / {total} total")

        if failed:
            print(f"\n{RED}{BOLD}DO NOT FLY{RESET} — resolve the red items above before launching.")
            return 1
        elif warned:
            print(f"\n{YELLOW}{BOLD}CAUTION{RESET} — review warnings before launching.")
            return 0
        else:
            print(f"\n{GREEN}{BOLD}READY TO FLY{RESET}")
            return 0

    def run(self):
        print(f"{BOLD}Meridian Pre-flight Check{RESET}")
        print(f"{DIM}{'=' * 50}{RESET}")
        self.check_deps()
        self.check_mavlink_source()
        self.check_telemetry()
        self.check_terrain()
        self.check_bridge_ws()
        self.check_http()
        return self.print_summary()


def main():
    p = argparse.ArgumentParser(
        description="Pre-flight verification for Meridian + Vanguard USV lake test",
    )
    src = p.add_mutually_exclusive_group()
    src.add_argument("--serial", metavar="PORT", help="Serial port (MAVLink source)")
    src.add_argument("--tcp", metavar="HOST:PORT", help="TCP MAVLink source")
    src.add_argument("--udp", metavar="HOST:PORT", help="UDP MAVLink source")
    p.add_argument("--baud", type=int, default=115200, help="Serial baud (default 115200)")
    p.add_argument("--lat", type=float, help="Operating area latitude (to verify terrain cache)")
    p.add_argument("--lon", type=float, help="Operating area longitude")
    p.add_argument("--ws-port", type=int, default=5761, help="Bridge WebSocket port to verify (default 5761)")
    p.add_argument("--http-port", type=int, default=8080, help="Tablet HTTP port to verify (default 8080)")
    p.add_argument("--no-http", action="store_true", help="Skip HTTP check")
    p.add_argument("--no-ws", action="store_true", help="Skip WebSocket check")
    args = p.parse_args()

    if args.no_http:
        args.http_port = 0
    if args.no_ws:
        args.ws_port = 0

    runner = PreflightRunner(args)
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())
