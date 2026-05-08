#!/usr/bin/env python3
"""Send UBX-CFG-RST (cold start) directly to the F9P inside a HERE4.

The HERE4's F9P chip is downstream of AP_Periph (the firmware running
on the HERE4's STM32). Normal DroneCAN param fixes only touch
AP_Periph's view of the F9P -- they trigger AP_Periph to re-send its
canned UBX init sequence to the F9P, but they don't reset the F9P's
own internal state (BBR, almanac, persistent CFG bits).

This script tunnels raw UBX bytes through DroneCAN using
`uavcan.tunnel.Targetted`. AP_Periph receives those messages, peels
out the buffer, and writes the bytes straight to the F9P's UART.
The F9P then processes them as if they came from the host.

Specifically we send UBX-CFG-RST with navBbrMask=0xFFFF and
resetMode=0x01 (controlled software reset). That's a hot-restart-with-
BBR-clear -- equivalent to "factory reset GNSS state". The F9P will
re-acquire from cold and any stuck BBR config goes away.

Usage:
    python ws-udp-relay.py --ws ws://100.72.16.72:5760 &
    python here4-f9p-coldstart.py --node 125
"""
from __future__ import annotations
import argparse, sys, time
import dronecan
from dronecan import uavcan

# --- UBX framing helpers --------------------------------------------
UBX_HDR = b"\xB5\x62"

def ubx_checksum(payload: bytes) -> bytes:
    """Fletcher-style 8-bit checksum over class+id+len+payload."""
    a = b = 0
    for byte in payload:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b])

def ubx_msg(cls: int, msg_id: int, payload: bytes) -> bytes:
    """Build a complete UBX frame: B5 62 <cls> <id> <len_lo> <len_hi> <payload> <ck_a> <ck_b>"""
    body = bytes([cls, msg_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    return UBX_HDR + body + ubx_checksum(body)

# UBX-CFG-RST (class 0x06, id 0x04). Payload:
#   uint16 navBbrMask  -- 0xFFFF = clear all BBR (cold start)
#                          0x0001 = ephemeris only (warm)
#                          0x0000 = no clear (hot start)
#   uint8  resetMode   -- 0x00 hardware reset (immediate)
#                          0x01 controlled software reset
#                          0x02 controlled SW reset, GNSS only
#                          0x04 hardware reset after shutdown
#                          0x08 controlled GNSS stop
#                          0x09 controlled GNSS start
#   uint8  reserved    -- 0
def ubx_cfg_rst(bbr_mask: int = 0xFFFF, reset_mode: int = 0x01) -> bytes:
    payload = bytes([
        bbr_mask & 0xFF, (bbr_mask >> 8) & 0xFF,
        reset_mode & 0xFF,
        0x00,
    ])
    return ubx_msg(0x06, 0x04, payload)

# UBX-CFG-CFG (class 0x06, id 0x09). Payload:
#   uint32 clearMask   -- bitmask of config sections to clear from BBR/Flash
#   uint32 saveMask    -- "    save to BBR/Flash
#   uint32 loadMask    -- "    load from BBR/Flash
#   uint8  deviceMask  -- 0x01 BBR, 0x02 Flash, 0x04 EEPROM (extension)
# Use 0xFFFF clear+save = full factory reset persisting to flash.
def ubx_cfg_cfg_factory_reset() -> bytes:
    payload = bytes([
        0xFF, 0xFF, 0x00, 0x00,  # clearMask: I/O port + msg + INF + nav + RXM + RINV + ANT + LOG
        0xFF, 0xFF, 0x00, 0x00,  # saveMask: same -- save defaults
        0x00, 0x00, 0x00, 0x00,  # loadMask: 0 (don't reload)
        0x07,                     # deviceMask: BBR + Flash + EEPROM
    ])
    return ubx_msg(0x06, 0x09, payload)

# --- DroneCAN tunnel send -------------------------------------------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--node", type=int, default=125,
                   help="Target HERE4 node ID (HERE4 #2 = 125)")
    p.add_argument("--port", default="mavcan:udpout:127.0.0.1:14550")
    p.add_argument("--mode", choices=["cold", "factory"], default="cold",
                   help="cold = UBX-CFG-RST coldstart (BBR clear). "
                        "factory = UBX-CFG-CFG full clear+save to flash, then RST.")
    p.add_argument("--repeat", type=int, default=3,
                   help="How many times to send each UBX message (DroneCAN MTU "
                        "fragmentation + radio path can drop frames; redundancy helps).")
    args = p.parse_args()

    print(f"[+] Opening DroneCAN node 127 on {args.port}")
    node = dronecan.make_node(args.port, node_id=127, bitrate=1000000,
                               mavlink_target_system=1,
                               mavlink_target_component=1)

    def safe_spin(t=0.1):
        try: node.spin(t)
        except Exception: pass

    # Settle the bus / get CAN forwarding established
    print("[+] Letting CAN forwarding settle (3 s)...")
    end = time.time() + 3
    while time.time() < end: safe_spin()

    def send_ubx(name, ubx_bytes):
        print(f"[+] {name}: {len(ubx_bytes)} bytes UBX -> tunnel.Targetted node {args.node}")
        for attempt in range(args.repeat):
            try:
                msg = uavcan.tunnel.Targetted()
                msg.target_node = args.node
                # protocol.protocol = the wire format. 4 = "GPS UBX" by AP_Periph
                # convention. Some AP_Periph versions just take any protocol value
                # and forward bytes regardless; we set it to GPS_UBX (4).
                try:
                    msg.protocol.protocol = 4  # GPS_UBX
                except Exception:
                    pass
                msg.serial_id = 0       # primary serial port (UART to F9P)
                msg.baudrate = 0        # 0 = leave unchanged
                msg.options = 0
                msg.buffer = list(ubx_bytes)
                node.broadcast(msg)
                print(f"    sent (attempt {attempt+1}/{args.repeat})")
            except Exception as e:
                print(f"    send error: {e}")
            # Let the message actually flush
            t = time.time() + 0.4
            while time.time() < t: safe_spin()

    if args.mode == "factory":
        send_ubx("UBX-CFG-CFG factory clear+save", ubx_cfg_cfg_factory_reset())
        # Allow the F9P to commit to flash before the next reset
        end = time.time() + 2.0
        while time.time() < end: safe_spin()

    send_ubx("UBX-CFG-RST cold start (BBR clear)", ubx_cfg_rst(0xFFFF, 0x01))

    print("\n[+] Done. Waiting 5 s for F9P to come back from cold start...")
    end = time.time() + 5.0
    while time.time() < end: safe_spin()
    print("[+] Now check GPS2 with full-check.py -- F9P will need 30-60s "
          "for first fix from a cold start.")

if __name__ == "__main__":
    main()
