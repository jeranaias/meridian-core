#!/usr/bin/env python3
"""Query specific named DroneCAN params on a node."""
from __future__ import annotations
import argparse, sys, time
import dronecan
from dronecan import uavcan

NAMES = [
    "GPS_TYPE", "GPS_RATE_MS", "GPS_GNSS_MODE", "GPS_RAW_DATA",
    "GPS_DRIVER_OPTIONS", "GPS_MB_ROLE", "GPS_MB_PORT", "GPS_MB_DATA",
    "BCN_MB_ROLE", "RTK_ROLE", "MB_ROLE",
    "GNSS_TYPE", "GNSS_MODE", "GPS_DRV_OPT", "GPS_INST",
    "BARO_PROBE", "COMPASS_ENABLE", "COMPASS_PRIO1_ID",
    "BRD_HW_REV", "BRD_VER", "VERSION",
    "AHRS_ORIENTATION",
]

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--node", type=int, default=125)
    p.add_argument("--port", default="mavcan:udpout:127.0.0.1:14550")
    args = p.parse_args()

    print(f"[+] Opening node 127 on {args.port}")
    node = dronecan.make_node(args.port, node_id=127, bitrate=1000000,
                               mavlink_target_system=1, mavlink_target_component=1)

    def safe_spin(t=0.1):
        try: node.spin(t)
        except Exception: pass

    end = time.time() + 3
    while time.time() < end: safe_spin()

    print(f"[+] Querying {len(NAMES)} named params on node {args.node}")
    for name in NAMES:
        result = {}
        def cb(event):
            result["event"] = event
        req = uavcan.protocol.param.GetSet.Request(index=0, name=name)
        node.request(req, args.node, cb, priority=16, timeout=2.0)
        end = time.time() + 2.5
        while time.time() < end and "event" not in result: safe_spin()
        if "event" not in result or result["event"] is None:
            print(f"  {name:24s}  TIMEOUT")
            continue
        ev = result["event"]
        if ev.response is None:
            print(f"  {name:24s}  no response")
            continue
        got_name = bytes(ev.response.name).decode("ascii", errors="replace")
        v = ev.response.value
        if got_name == "":
            print(f"  {name:24s}  NOT FOUND")
            continue
        if v.integer_value is not None: val = f"int={v.integer_value}"
        elif v.real_value is not None: val = f"real={v.real_value}"
        elif v.boolean_value is not None: val = f"bool={v.boolean_value}"
        else: val = "empty"
        print(f"  {got_name:24s}  {val}")

if __name__ == "__main__":
    main()
