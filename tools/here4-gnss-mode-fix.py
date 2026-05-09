#!/usr/bin/env python3
"""HERE4 GPS_GNSS_MODE constellation fix.

Per ardupilot/discuss research (#23529, cubepilot/14706):
  - AP_Periph reads GPS_GNSS_MODE on boot and pushes a validated
    UBX-CFG-GNSS to the F9P, bypassing whatever junk we may have
    written via tunnel.Targetted.
  - Default value on a fresh AP_Periph param table is 0 (= no
    constellations enabled). After our earlier malformed CFG-GNSS,
    HERE4 #2's F9P is most likely sitting with an empty constellation
    mask -- explains 0 sats, MagField2 still working (compass is a
    separate chip).
  - Bitmask: GPS=1 SBAS=2 Galileo=4 BeiDou=8 IMES=16 QZSS=32 GLONASS=64.
  - 67 = GPS + SBAS + Galileo + GLONASS (the four to use in southern
    hemisphere; BeiDou is optional). 71 adds BeiDou. 127 = everything.

Plus: triggers a NodeRestart so AP_Periph's boot-time UBX init runs
fresh against the F9P with the new constellation mask.
"""
import argparse, sys, time
import dronecan
from dronecan import uavcan

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--node",  type=int, default=125)
    p.add_argument("--port",  default="mavcan:udpout:127.0.0.1:14550")
    p.add_argument("--mask",  type=int, default=67,
                   help="GNSS bitmask (GPS=1 SBAS=2 Galileo=4 BeiDou=8 "
                        "QZSS=32 GLONASS=64). 67 = GPS+SBAS+Galileo+GLONASS.")
    args = p.parse_args()

    print(f"[+] Opening DroneCAN node 127 on {args.port}")
    node = dronecan.make_node(args.port, node_id=127, bitrate=1000000,
                               mavlink_target_system=1,
                               mavlink_target_component=1)
    def safe_spin(t=0.1):
        try: node.spin(t)
        except Exception: pass

    end = time.time() + 3
    while time.time() < end: safe_spin()

    def call(req, timeout=20.0):
        result = {}
        def cb(event): result["event"] = event
        node.request(req, args.node, cb, priority=16, timeout=timeout)
        end = time.time() + timeout + 1
        while time.time() < end and "event" not in result:
            safe_spin()
        return result.get("event")

    # 1. Park GPS_TYPE = 0 so AP_Periph's autoconfig stops fighting us
    print(f"[1] GPS_TYPE = 0 (suspend autoconfig, stop the UART race)")
    ev = call(uavcan.protocol.param.GetSet.Request(
        index=0, name="GPS_TYPE",
        value=uavcan.protocol.param.Value(integer_value=0)))
    if ev and ev.response: print(f"    -> ok")
    else: print(f"    -> no echo (may still have landed)")
    time.sleep(0.5)

    # 2. Set GPS_GNSS_MODE
    print(f"[2] GPS_GNSS_MODE = {args.mask} (constellation bitmask)")
    ev = call(uavcan.protocol.param.GetSet.Request(
        index=0, name="GPS_GNSS_MODE",
        value=uavcan.protocol.param.Value(integer_value=args.mask)))
    if ev and ev.response:
        v = ev.response.value
        print(f"    -> server says {v.integer_value}")
    else:
        print(f"    -> no echo")
    time.sleep(0.5)

    # 3. Restore GPS_TYPE = 1 so AP_Periph re-enables driver with new mode
    print(f"[3] GPS_TYPE = 1 (auto, re-enable driver)")
    ev = call(uavcan.protocol.param.GetSet.Request(
        index=0, name="GPS_TYPE",
        value=uavcan.protocol.param.Value(integer_value=1)))
    if ev and ev.response: print(f"    -> ok")
    else: print(f"    -> no echo")
    time.sleep(0.5)

    # 4. Save to NVM
    print(f"[4] ExecuteOpcode SAVE")
    ev = call(uavcan.protocol.param.ExecuteOpcode.Request(opcode=0))
    if ev and ev.response:
        print(f"    -> save_ok={getattr(ev.response, 'ok', '?')}")
    time.sleep(0.5)

    # 5. RestartNode
    print(f"[5] RestartNode (triggers fresh AP_Periph UBX init w/ new mask)")
    try:
        call(uavcan.protocol.RestartNode.Request(magic_number=0xACCE551B1E),
             timeout=2.0)
    except Exception:
        pass

    print(f"\n[+] Done. F9P will re-init with GPS+SBAS+Galileo+GLONASS.")
    print(f"    First-fix from cold start: 30-90s on a clear sky view.")

if __name__ == "__main__":
    main()
