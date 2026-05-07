#!/usr/bin/env python3
"""Reset HERE4 #2's internal GPS_TYPE param back to 1 (auto) over
DroneCAN-via-MAVLink. This unsticks the Moving Baseline Base mode that
got flashed into the periph's NVM and is causing the
"Incorrect Role 125 should be Base" loop + GPS2 NoFix.

Usage:
    # Start the relay first (in another terminal or via run_in_background):
    python ws-udp-relay.py --ws ws://100.72.16.72:5760 --udp 127.0.0.1:14550

    # Then run this:
    python here4-fix.py --node 125 --port mavcan:udpout:127.0.0.1:14550

The script:
  1. Allocates a local DroneCAN node on the MAVLink CAN tunnel
  2. Sends param.GetSet  to target node, name="GPS_TYPE", value=1
  3. Verifies the response
  4. Sends param.ExecuteOpcode SAVE so it persists across reboot
  5. Sends RestartNode to make it pick up the new role
"""
from __future__ import annotations
import argparse, sys, time
import dronecan
from dronecan import uavcan

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--node", type=int, default=125,
                   help="Target DroneCAN node ID (HERE4 #2 = 125)")
    p.add_argument("--port", default="mavcan:udpout:127.0.0.1:14550",
                   help="dronecan-py transport URL")
    p.add_argument("--name", default="GPS_TYPE",
                   help="Periph param name to write")
    p.add_argument("--value", type=int, default=1,
                   help="Periph param new value (1 = auto)")
    p.add_argument("--no-restart", action="store_true",
                   help="Skip RestartNode at the end")
    args = p.parse_args()

    print(f"[+] Opening DroneCAN node on {args.port} (local node 127)")
    node = dronecan.make_node(args.port, node_id=127, bitrate=1000000,
                               mavlink_target_system=1,
                               mavlink_target_component=1)

    # Tell ArduPilot to start forwarding CAN bus 2 traffic over MAVLink
    # (the mavcan driver does this internally on a timer; we don't have
    # to do anything, but a hint here makes early-bus traffic visible.)
    print(f"[+] Letting CAN forwarding settle (3 s)...")
    end = time.time() + 3
    def safe_spin(t=0.1):
        try:
            node.spin(t)
        except Exception:
            pass
    while time.time() < end:
        safe_spin()

    # Helper to wait for a service response synchronously.
    result_holder = {}
    def on_resp(event):
        result_holder["event"] = event

    # ---- 1. param.GetSet to write GPS_TYPE = args.value -------------
    print(f"[+] Writing {args.name} = {args.value} on node {args.node} ...")
    req = uavcan.protocol.param.GetSet.Request(
        index=0,
        name=args.name,
        value=uavcan.protocol.param.Value(integer_value=args.value),
    )
    result_holder.clear()
    node.request(req, args.node, on_resp, priority=16, timeout=4.0)
    end = time.time() + 5.0
    while time.time() < end and "event" not in result_holder:
        safe_spin()
    if "event" not in result_holder:
        print(f"[!] No response to GetSet within 5 s.")
        sys.exit(2)
    ev = result_holder["event"]
    if ev.transfer is None or not ev.response:
        print(f"[!] GetSet response empty: {ev}")
        sys.exit(2)
    resp = ev.response
    print(f"    response: name={bytes(resp.name).decode('ascii', errors='replace')!r} "
          f"value={resp.value} default={resp.default_value}")

    # ---- 2. param.ExecuteOpcode SAVE (persist to NVM) ---------------
    print(f"[+] Saving params to NVM on node {args.node} ...")
    save_req = uavcan.protocol.param.ExecuteOpcode.Request(opcode=0)  # 0 = SAVE
    result_holder.clear()
    node.request(save_req, args.node, on_resp, priority=16, timeout=4.0)
    end = time.time() + 5.0
    while time.time() < end and "event" not in result_holder:
        safe_spin()
    if "event" not in result_holder:
        print(f"[!] No response to ExecuteOpcode within 5 s. Did param save anyway? Continuing.")
    else:
        ev = result_holder["event"]
        if ev.response and getattr(ev.response, 'ok', None):
            print(f"    save ok={ev.response.ok}")
        else:
            print(f"    save response: {ev.response}")

    # ---- 3. RestartNode (so the new role takes effect) --------------
    if not args.no_restart:
        print(f"[+] Restarting node {args.node} ...")
        restart_req = uavcan.protocol.RestartNode.Request(
            magic_number=0xACCE551B1E,
        )
        result_holder.clear()
        node.request(restart_req, args.node, on_resp, priority=16, timeout=2.0)
        end = time.time() + 3.0
        while time.time() < end and "event" not in result_holder:
            safe_spin()
        if "event" in result_holder:
            print(f"    restart ack: ok={getattr(result_holder['event'].response, 'ok', '?')}")
        else:
            print(f"    no ack -- node likely already restarting (this is normal)")

    print(f"[+] Done. Wait ~30 s for HERE4 #{args.node} to come back, "
          f"then check both GPSes.")

if __name__ == "__main__":
    main()
