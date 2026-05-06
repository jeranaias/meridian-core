#!/usr/bin/env python3
"""
tristan-gcs-bridge.py — One-command bootstrap for the Vanguard test console.

Runs on the USV laptop. Does three things:

  1. Stops the auto-flash watcher so it doesn't catch the running ArduRover
     in the middle of normal MAVLink work and try to flash it.
  2. Finds the active Cube COM port (matches VID 0x2DAE) and starts
     mavlink-ws-bridge.py against it on ws://0.0.0.0:5760.
  3. Prints the URLs to open in a browser.

Usage:
    python tools/tristan-gcs-bridge.py

When you want to flash a new firmware build instead of run the GCS, stop
this script (Ctrl-C) and run one of the launchers in the same dir, e.g.
launch-meridian-v1.2.ps1 or launch-ardurover-restore.ps1.

Requires: pip install websockets pyserial pymavlink
"""
import os
import subprocess
import sys
import time
from pathlib import Path
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parent.parent
BRIDGE = ROOT / "tools" / "mavlink-ws-bridge.py"
WATCHER_TASK = "MeridianAggressiveWatcher"
WS_PORT = 5760


def stop_watcher():
    """Stop the firmware auto-flash watcher so we don't fight it."""
    try:
        subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             f"Stop-ScheduledTask -TaskName {WATCHER_TASK} -EA SilentlyContinue; "
             "Get-CimInstance Win32_Process | Where-Object { "
             "$_.CommandLine -like '*usv-watch-aggressive.py*' -or "
             "$_.CommandLine -like '*uploader.py*' "
             "} | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -EA SilentlyContinue }"],
            check=False, capture_output=True, timeout=15,
        )
        print("[bridge] firmware watcher stopped (if it was running)")
    except Exception as e:
        print(f"[bridge] stop_watcher: {e} (non-fatal)", file=sys.stderr)


def find_cube_port():
    """Return the first VID 0x2DAE COM port (ArduRover MAVLink primary)."""
    candidates = sorted(
        (p for p in list_ports.comports() if (p.vid or 0) == 0x2DAE),
        key=lambda p: p.device,
    )
    if not candidates:
        return None
    # ArduPilot enumerates COM4 as MI_00 (primary MAVLink) and COM5 as MI_02.
    # MI_00 is what Mission Planner / QGC connects to. Pick whichever appears
    # first by name (sorted) — usually that's MI_00 = primary.
    return candidates[0]


def get_tailscale_ip():
    """Best-effort Tailscale IP for printing browser URLs."""
    try:
        r = subprocess.run(["tailscale", "ip", "-4"],
                           capture_output=True, text=True, timeout=5)
        if r.returncode == 0:
            ip = r.stdout.strip().splitlines()[0]
            return ip if ip else None
    except Exception:
        pass
    return None


def main():
    if not BRIDGE.exists():
        print(f"FATAL: bridge not found at {BRIDGE}", file=sys.stderr)
        sys.exit(2)

    stop_watcher()

    port = find_cube_port()
    if port is None:
        print("FATAL: no Cube on USB. ArduRover should be running and "
              "enumerated as VID 0x2DAE. Plug it in / power-cycle and retry.",
              file=sys.stderr)
        sys.exit(3)
    print(f"[bridge] Cube on {port.device}  (VID=0x{port.vid:04x} "
          f"PID=0x{port.pid:04x}  desc={port.description!r})")

    ts_ip = get_tailscale_ip()
    print()
    print("=" * 60)
    print(" MAVLink WebSocket bridge — listening on ws://0.0.0.0:%d" % WS_PORT)
    print("=" * 60)
    print(" Open the test console in a browser:")
    if ts_ip:
        print(f"   GCS dev server (Jesse's machine): http://<jesse-tailscale-ip>:8765/test-console.html")
        print(f"   In the WS URL field, enter:        ws://{ts_ip}:{WS_PORT}")
    else:
        print(f"   On this machine: http://localhost:8765/test-console.html")
        print(f"   In the WS URL field, enter:        ws://localhost:{WS_PORT}")
    print(" Stop with Ctrl-C.")
    print("=" * 60)
    print()

    # Hand off to the bridge.
    cmd = [
        sys.executable, str(BRIDGE),
        "--serial", port.device,
        "--baud", "115200",
        "--ws-host", "0.0.0.0",
        "--ws-port", str(WS_PORT),
    ]
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n[bridge] stopped")


if __name__ == "__main__":
    main()
