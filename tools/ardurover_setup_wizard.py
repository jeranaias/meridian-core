#!/usr/bin/env python3
"""
ArduRover propulsion setup wizard for the Vanguard USV.

Designed so a Claude Code agent on the USV laptop can drive the whole
configuration from a few natural-language requests, with the operator
confirming physical safety (prop out of water) when the wizard wiggles
hardware. No firmware flashing required — every change is a live
PARAM_SET over MAVLink.

Subcommands:

    status              Print current SERVO_n_FUNCTION + frame + arming
                        config as JSON. Read-only, safe.

    probe CH            Send a small, brief PWM pulse to channel CH.
                        Operator watches and reports which output
                        physically responded.

    configure           Set SERVO_n_FUNCTION values for the propulsion
                        system. --throttle-ch N --steering-ch M
                        --esc-type {pwm,dshot}

    verify              Quick functional check post-configure: ARM
                        with safety bypassed, drive throttle/steering
                        for ~2 seconds, DISARM. Prop MUST be out of
                        water; the wizard confirms this before doing
                        anything.

    arming-bypass       Set ARMING_CHECK / BRD_SAFETYENABLE so bench
                        testing doesn't refuse-arm on no-GPS / no-RC.
                        Reversible — keep a record of original values.

Connection: ws://localhost:5760 by default (the mavlink-ws-bridge.py
that's running on the USV as a Scheduled Task). Override with --ws.

The script never makes a destructive change without an explicit
operator-confirmed flag passed in. Defaults are safe.
"""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import json
import struct
import sys
import time
from typing import Optional

try:
    import websockets
except ImportError:
    print("ERROR: pip install websockets", file=sys.stderr)
    sys.exit(2)


# ──────────────────────────────────────────────────────────────────
# MAVLink v2 framing — minimal, just what we need
# ──────────────────────────────────────────────────────────────────

# CRC_EXTRA values for messages we send/receive. Keep this list
# explicit so a typo here doesn't silently fail with a CRC mismatch.
CRC_EXTRA = {
    0:   50,   # HEARTBEAT
    11: 89,   # SET_MODE (legacy)
    20:  214,  # PARAM_REQUEST_READ
    21:  159,  # PARAM_REQUEST_LIST
    22:  220,  # PARAM_VALUE
    23:  168,  # PARAM_SET
    24:  24,   # GPS_RAW_INT
    30:  39,   # ATTITUDE
    33:  104,  # GLOBAL_POSITION_INT
    66:  148,  # REQUEST_DATA_STREAM
    74:  20,   # VFR_HUD
    76:  152,  # COMMAND_LONG
    77:  143,  # COMMAND_ACK
    148: 178,  # AUTOPILOT_VERSION
    253: 83,   # STATUSTEXT
}


def _crc16_mcrf4xx(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for byte in list(data) + [extra]:
        tmp = (byte ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ ((tmp << 4) & 0xFF)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


@dataclasses.dataclass
class _SeqState:
    seq: int = 0


def _encode(msgid: int, payload: bytes, st: _SeqState, sysid: int = 255, compid: int = 190) -> bytes:
    plen = len(payload)
    hdr = bytes([0xFD, plen, 0, 0, st.seq, sysid, compid,
                 msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    st.seq = (st.seq + 1) & 0xFF
    body = hdr[1:] + payload
    crc = _crc16_mcrf4xx(body, CRC_EXTRA.get(msgid, 0))
    return hdr + payload + struct.pack("<H", crc)


def _command_long(cmd: int, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0,
                  target_sys: int = 1, target_comp: int = 1) -> bytes:
    # 7 floats (x4) + uint16 cmd + 3 uint8 = 33 bytes
    return struct.pack("<7fHBBB", p1, p2, p3, p4, p5, p6, p7,
                       cmd, target_sys, target_comp, 0)


def _param_set(name: str, value: float, ptype: int = 9, target_sys: int = 1, target_comp: int = 1) -> bytes:
    # PARAM_SET: float value, 16-byte name, uint8 ptype, uint8 sys, uint8 comp = 23 bytes
    name_b = name.encode("ascii", errors="ignore")[:16].ljust(16, b"\x00")
    return struct.pack("<f", value) + name_b + struct.pack("<BBB", ptype, target_sys, target_comp)


def _param_request_read(name: str, target_sys: int = 1, target_comp: int = 1) -> bytes:
    # PARAM_REQUEST_READ wire order (size descending):
    #   int16 param_index (-1 = lookup by name)
    #   char[16] param_id
    #   uint8 target_system
    #   uint8 target_component
    # Total = 20 bytes.
    name_b = name.encode("ascii", errors="ignore")[:16].ljust(16, b"\x00")
    return struct.pack("<h", -1) + name_b + struct.pack("<BB", target_sys, target_comp)


# ──────────────────────────────────────────────────────────────────
# Connection helpers
# ──────────────────────────────────────────────────────────────────

class Bridge:
    def __init__(self, ws_url: str):
        self.ws_url = ws_url
        self.ws = None
        self.seq = _SeqState()

    async def __aenter__(self):
        self.ws = await websockets.connect(self.ws_url, open_timeout=10)
        return self

    async def __aexit__(self, *exc):
        if self.ws:
            await self.ws.close()

    async def send(self, msgid: int, payload: bytes) -> None:
        assert self.ws is not None
        await self.ws.send(_encode(msgid, payload, self.seq))

    async def recv_until(self, msgid_filter, timeout: float):
        """Receive frames for `timeout` sec, yielding decoded ones whose
        msgid is in `msgid_filter`. msgid_filter can be int or set."""
        if isinstance(msgid_filter, int):
            msgid_filter = {msgid_filter}
        end = asyncio.get_event_loop().time() + timeout
        while asyncio.get_event_loop().time() < end:
            try:
                msg = await asyncio.wait_for(self.ws.recv(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            if not isinstance(msg, bytes) or len(msg) < 12 or msg[0] != 0xFD:
                continue
            mid = msg[7] | (msg[8] << 8) | (msg[9] << 16)
            if mid not in msgid_filter:
                continue
            payload_len = msg[1]
            yield mid, msg[10:10 + payload_len]


# ──────────────────────────────────────────────────────────────────
# Domain helpers — params we care about
# ──────────────────────────────────────────────────────────────────

# ArduPilot/ArduRover SERVO_FUNCTION values (subset) — see
# ArduPilot/libraries/SRV_Channel/SRV_Channel.h enum Function.
SERVO_FN = {
    0:   "Disabled",
    1:   "RCPassThru",
    26:  "GroundSteering",       # rover steering / nozzle
    51:  "RCIN1",
    52:  "RCIN2",
    53:  "RCIN3",
    70:  "Throttle",             # rover single throttle (ESC)
    73:  "ThrottleLeft",         # skid-steer left
    74:  "ThrottleRight",        # skid-steer right
    94:  "Mainsail",
    102: "ESC",
}

THROTTLE_FN = 70           # what we set for single-thrust ESC
STEERING_FN = 26           # what we set for nozzle / rudder

# Frame-class (rover): 1 = rover, but actually FRAME_CLASS not used for
# ArduRover boats — they use FRAME_CLASS=2 for boat. Will read & report.

ARDU_ROVER_MODES = {
    0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER",
    6: "FOLLOW", 7: "SIMPLE", 10: "AUTO", 11: "RTL", 12: "SMART_RTL",
    15: "GUIDED", 16: "INITIALISING",
}


def _decode_param_value(payload: bytes):
    """PARAM_VALUE wire order (size desc):
        float param_value          @ 0..3
        uint16 param_count         @ 4..5
        uint16 param_index         @ 6..7
        char[16] param_id          @ 8..23
        uint8 param_type           @ 24
    """
    if len(payload) < 25:
        return None, None
    val = struct.unpack_from("<f", payload, 0)[0]
    name = payload[8:24].rstrip(b"\x00").decode("ascii", errors="replace")
    return name, val


async def _read_param(bridge: Bridge, name: str, timeout: float = 3.0) -> Optional[float]:
    """Send PARAM_REQUEST_READ and wait for matching PARAM_VALUE."""
    await bridge.send(20, _param_request_read(name))
    async for mid, payload in bridge.recv_until(22, timeout):
        rname, val = _decode_param_value(payload)
        if rname == name:
            return val
    return None


async def _set_param(bridge: Bridge, name: str, value: float, timeout: float = 3.0) -> bool:
    """Send PARAM_SET, wait for echo PARAM_VALUE confirming."""
    await bridge.send(23, _param_set(name, value))
    async for mid, payload in bridge.recv_until(22, timeout):
        rname, new_val = _decode_param_value(payload)
        if rname == name:
            return abs(new_val - value) < 0.001 or int(new_val) == int(value)
    return False


async def _send_command(bridge: Bridge, cmd: int, *params, timeout: float = 3.0) -> Optional[int]:
    """Send COMMAND_LONG, wait for COMMAND_ACK. Returns result or None."""
    payload = _command_long(cmd, *params)
    await bridge.send(76, payload)
    async for mid, ack_payload in bridge.recv_until(77, timeout):
        ack_cmd, result = struct.unpack_from("<HB", ack_payload)
        if ack_cmd == cmd:
            return result
    return None


# ──────────────────────────────────────────────────────────────────
# Subcommands
# ──────────────────────────────────────────────────────────────────

async def cmd_status(args) -> int:
    out = {"ws": args.ws, "ardurover_setup": {}}
    async with Bridge(args.ws) as bridge:
        # Issue PARAM_REQUEST_LIST and collect for up to ~25 sec (or until idle).
        # Targeted PARAM_REQUEST_READ by name is unreliable on ArduPilot — the
        # full bulk dump is the supported path.
        await bridge.send(21, struct.pack("<BB", 1, 1))
        params = {}
        idle = 0
        end = asyncio.get_event_loop().time() + (args.list_timeout if hasattr(args, 'list_timeout') and args.list_timeout else 25.0)
        last_count = 0
        while asyncio.get_event_loop().time() < end:
            try:
                msg = await asyncio.wait_for(bridge.ws.recv(), timeout=0.5)
                idle = 0
            except asyncio.TimeoutError:
                idle += 1
                if idle > 6 and len(params) > 50:
                    break  # ~3 sec idle after substantial data → stream done
                continue
            if not isinstance(msg, bytes) or len(msg) < 12 or msg[0] != 0xFD:
                continue
            mid = msg[7] | (msg[8] << 8) | (msg[9] << 16)
            if mid == 22:  # PARAM_VALUE
                pl = msg[10:10 + msg[1]]
                rname, val = _decode_param_value(pl)
                if rname:
                    params[rname] = val
                    if len(params) - last_count >= 50:
                        last_count = len(params)
            elif mid == 0 and "live" not in out["ardurover_setup"]:
                pl = msg[10:10 + msg[1]]
                custom_mode, _t, _ap, base_mode, sysstat, _ver = struct.unpack_from("<IBBBBB", pl)
                out["ardurover_setup"]["live"] = {
                    "armed": bool(base_mode & 0x80),
                    "mode_num": int(custom_mode),
                    "mode_name": ARDU_ROVER_MODES.get(int(custom_mode), f"mode{custom_mode}"),
                    "system_status": sysstat,
                }

        # Pull SERVOn_FUNCTION for n=1..16 from the bulk dump
        servos = {}
        for n in range(1, 17):
            v = params.get(f"SERVO{n}_FUNCTION")
            if v is not None and int(v) != 0:
                ival = int(v)
                servos[f"SERVO{n}_FUNCTION"] = {
                    "value": ival,
                    "name": SERVO_FN.get(ival, f"unknown({ival})"),
                }
        out["ardurover_setup"]["servos"] = servos

        # Frame / arming / motor params we care about
        for pname in ("FRAME_CLASS", "FRAME_TYPE", "ARMING_CHECK", "BRD_SAFETYENABLE",
                      "MOT_PWM_TYPE", "BATT_MONITOR", "RCMAP_THROTTLE", "RCMAP_ROLL",
                      "RCMAP_PITCH", "RCMAP_YAW"):
            if pname in params:
                out["ardurover_setup"][pname] = params[pname]

        out["ardurover_setup"]["_total_params_received"] = len(params)

    # Best-guess analysis
    analysis = []
    servos = out["ardurover_setup"].get("servos", {})
    throttle_chs = [k for k, v in servos.items() if v["value"] == THROTTLE_FN]
    steering_chs = [k for k, v in servos.items() if v["value"] == STEERING_FN]
    if throttle_chs:
        analysis.append(f"Throttle (FN={THROTTLE_FN}) is on: {', '.join(throttle_chs)}")
    else:
        analysis.append(f"No SERVO is set to Throttle (FN={THROTTLE_FN}). ESC is unmapped.")
    if steering_chs:
        analysis.append(f"GroundSteering (FN={STEERING_FN}) is on: {', '.join(steering_chs)}")
    else:
        analysis.append(f"No SERVO is set to GroundSteering (FN={STEERING_FN}). Steering is unmapped.")
    out["analysis"] = analysis

    print(json.dumps(out, indent=2))
    return 0


async def cmd_probe(args) -> int:
    """Send a small, brief PWM pulse on a single channel. Operator
    watches the boat and reports physically what moved."""
    if not args.confirm_prop_out:
        print("REFUSED: pass --confirm-prop-out to acknowledge the prop is out of water.")
        return 3

    pwm = max(args.min_pwm, min(args.max_pwm, args.pwm))
    duration = max(0.2, min(2.0, args.duration))

    async with Bridge(args.ws) as bridge:
        # MAV_CMD_DO_SET_SERVO (cmd 183): p1=channel, p2=pwm
        print(f"  -> pulsing CH{args.channel} @ {pwm}us for {duration}s")
        result = await _send_command(bridge, 183, args.channel, pwm)
        if result != 0:
            print(f"  ! DO_SET_SERVO ACK result={result} (non-zero = denied/temp-rejected)")
            return 4
        await asyncio.sleep(duration)
        # Return to neutral / disabled (1500us neutral; if ESC, stops).
        print(f"  -> returning CH{args.channel} to 1500us (neutral)")
        await _send_command(bridge, 183, args.channel, 1500)
    return 0


async def cmd_configure(args) -> int:
    """Set SERVO_<throttle_ch>_FUNCTION = 73 and SERVO_<steering_ch>_FUNCTION = 26.
    Also set sane MIN/TRIM/MAX defaults if requested."""
    if args.throttle_ch < 1 or args.throttle_ch > 16:
        print("ERROR: --throttle-ch must be 1..16")
        return 5
    if args.steering_ch < 1 or args.steering_ch > 16:
        print("ERROR: --steering-ch must be 1..16")
        return 5
    if args.throttle_ch == args.steering_ch:
        print("ERROR: throttle and steering must be on different channels")
        return 5

    async with Bridge(args.ws) as bridge:
        applied = {}

        # Snapshot before (for diff)
        for n in (args.throttle_ch, args.steering_ch):
            v = await _read_param(bridge, f"SERVO{n}_FUNCTION")
            applied[f"SERVO{n}_FUNCTION_before"] = int(v) if v is not None else None

        # Set the propulsion functions
        if not await _set_param(bridge, f"SERVO{args.throttle_ch}_FUNCTION", THROTTLE_FN):
            print(f"ERROR: failed to confirm SERVO{args.throttle_ch}_FUNCTION = {THROTTLE_FN}")
            return 6
        applied[f"SERVO{args.throttle_ch}_FUNCTION"] = THROTTLE_FN
        if not await _set_param(bridge, f"SERVO{args.steering_ch}_FUNCTION", STEERING_FN):
            print(f"ERROR: failed to confirm SERVO{args.steering_ch}_FUNCTION = {STEERING_FN}")
            return 6
        applied[f"SERVO{args.steering_ch}_FUNCTION"] = STEERING_FN

        # PWM bounds for both — sane defaults for hobby ESCs and servos.
        # Tristan can override later via Mission Planner.
        if args.set_pwm_bounds:
            for n, fn_label in [(args.throttle_ch, "throttle"), (args.steering_ch, "steering")]:
                for k, v in [(f"SERVO{n}_MIN", 1100), (f"SERVO{n}_TRIM", 1500), (f"SERVO{n}_MAX", 1900)]:
                    if await _set_param(bridge, k, v):
                        applied[k] = v

        # ESC type — for standard PWM, MOT_PWM_TYPE = 0. For DShot variants, different values.
        if args.esc_type == "pwm":
            if await _set_param(bridge, "MOT_PWM_TYPE", 0):
                applied["MOT_PWM_TYPE"] = 0
        elif args.esc_type == "dshot":
            if await _set_param(bridge, "MOT_PWM_TYPE", 6):  # DShot300
                applied["MOT_PWM_TYPE"] = 6

        print(json.dumps({"applied": applied}, indent=2))
    return 0


async def cmd_arming_bypass(args) -> int:
    """Loosen ArduRover's pre-arm checks so bench testing without GPS /
    RC works. Reversible: print the BEFORE values so they can be put back."""
    async with Bridge(args.ws) as bridge:
        before = {}
        for p in ("ARMING_CHECK", "BRD_SAFETYENABLE"):
            v = await _read_param(bridge, p)
            if v is not None:
                before[p] = v
        print(json.dumps({"before": before}, indent=2))
        if not args.confirm_bench_only:
            print("REFUSED: pass --confirm-bench-only to acknowledge this is a bench test.")
            return 3
        # ARMING_CHECK = 0 disables ALL pre-arm checks. For bench only.
        await _set_param(bridge, "ARMING_CHECK", 0)
        # BRD_SAFETYENABLE = 0 disables the physical safety switch requirement.
        await _set_param(bridge, "BRD_SAFETYENABLE", 0)
        print(json.dumps({"after": {"ARMING_CHECK": 0, "BRD_SAFETYENABLE": 0}}, indent=2))
        print("\nTo restore stock arming behavior:")
        for k, v in before.items():
            print(f"  python {sys.argv[0]} restore-param --name {k} --value {int(v)}")
    return 0


async def cmd_verify(args) -> int:
    """Quick post-configure sanity sweep: ARM, brief throttle/steering pulse, DISARM."""
    if not args.confirm_prop_out:
        print("REFUSED: pass --confirm-prop-out to acknowledge prop is out of water.")
        return 3

    async with Bridge(args.ws) as bridge:
        # Find currently-mapped throttle + steering channels
        throttle_ch = steering_ch = None
        for n in range(1, 17):
            v = await _read_param(bridge, f"SERVO{n}_FUNCTION")
            if v is None: continue
            iv = int(v)
            if iv == THROTTLE_FN: throttle_ch = n
            elif iv == STEERING_FN: steering_ch = n
        if throttle_ch is None or steering_ch is None:
            print(f"ERROR: throttle (FN={THROTTLE_FN}) or steering (FN={STEERING_FN}) not configured. "
                  f"Found throttle_ch={throttle_ch}, steering_ch={steering_ch}")
            return 7

        print(f"  throttle on CH{throttle_ch}, steering on CH{steering_ch}")

        # Set MANUAL mode, ARM (force)
        print("  -> setting MANUAL mode")
        r = await _send_command(bridge, 176, 1, 0, 0, 0, 0, 0, 0)  # base_mode=1 (CUSTOM), custom_mode=0 (MANUAL)
        print(f"     ACK: {r}")

        print("  -> ARM (force-magic 21196)")
        r = await _send_command(bridge, 400, 1, 21196, 0, 0, 0, 0, 0)
        print(f"     ACK: {r} (0=accepted)")
        if r != 0:
            print("     arming refused — likely pre-arm check failed. Run arming-bypass first.")
            return 8

        # Throttle pulse
        print(f"  -> throttle pulse CH{throttle_ch} = 1550us for 1.5s")
        await _send_command(bridge, 183, throttle_ch, 1550)
        await asyncio.sleep(1.5)
        await _send_command(bridge, 183, throttle_ch, 1500)

        await asyncio.sleep(0.5)

        # Steering sweep
        print(f"  -> steering sweep CH{steering_ch}: 1300 -> 1700 -> 1500")
        for pwm in (1300, 1700, 1500):
            await _send_command(bridge, 183, steering_ch, pwm)
            await asyncio.sleep(0.7)

        # DISARM
        print("  -> DISARM (force)")
        r = await _send_command(bridge, 400, 0, 21196, 0, 0, 0, 0, 0)
        print(f"     ACK: {r}")

        print("\nIf the ESC spun on throttle and the nozzle moved on steering — config is good.")
    return 0


async def cmd_fix_streams(args) -> int:
    """Set ArduPilot's SRn_* stream rate params so we get the messages
    we actually use in the GCS — derived from analysis of real recorded
    sessions where BATTERY_STATUS / VFR_HUD / VIBRATION / RC_CHANNELS
    / AHRS / ESC_TELEMETRY were all missing.

    SR0_* governs streams sent over Serial0 (USB on Cube Plus).
    """
    # (param_name, hz, what's in this stream)
    PLAN = [
        ("SR0_RAW_SENS",  4,  "RAW_IMU, SCALED_PRESSURE, SCALED_IMU2"),
        ("SR0_EXT_STAT",  2,  "SYS_STATUS, GPS_RAW_INT, BATTERY_STATUS"),
        ("SR0_RC_CHAN",   2,  "RC_CHANNELS, SERVO_OUTPUT_RAW"),
        ("SR0_RAW_CTRL",  1,  "RC controller raw"),
        ("SR0_POSITION",  5,  "GLOBAL_POSITION_INT, LOCAL_POS, HOME_POSITION"),
        ("SR0_EXTRA1",   10,  "ATTITUDE, ATTITUDE_TARGET"),
        ("SR0_EXTRA2",    4,  "VFR_HUD"),
        ("SR0_EXTRA3",    2,  "AHRS, AHRS2, VIBRATION, BATTERY_STATUS, ESC_TELEMETRY"),
        ("SR0_PARAMS",   10,  "PARAM_VALUE during dump"),
    ]
    async with Bridge(args.ws) as bridge:
        applied = {}
        for name, rate, what in PLAN:
            ok = await _set_param(bridge, name, float(rate))
            applied[name] = {"rate_hz": rate, "applied": ok, "messages": what}
        print(json.dumps({"streams": applied}, indent=2))
    return 0


async def cmd_restore_param(args) -> int:
    async with Bridge(args.ws) as bridge:
        ok = await _set_param(bridge, args.name, args.value)
        print(json.dumps({"set": {args.name: args.value, "confirmed": ok}}, indent=2))
    return 0 if ok else 6


# ──────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="ArduRover propulsion setup wizard")
    p.add_argument("--ws", default="ws://localhost:5760", help="MAVLink WebSocket URL")
    sp = p.add_subparsers(dest="cmd", required=True)

    sst = sp.add_parser("status", help="Read & print current SERVO + frame setup as JSON")
    sst.add_argument("--list-timeout", type=float, default=25.0,
                     help="Max seconds to wait for the param dump (default 25)")

    pp = sp.add_parser("probe", help="Pulse one channel briefly so operator can ID it")
    pp.add_argument("--channel", type=int, required=True)
    pp.add_argument("--pwm", type=int, default=1550, help="PWM in us; 1500=neutral")
    pp.add_argument("--duration", type=float, default=1.0, help="Pulse seconds (0.2–2.0)")
    pp.add_argument("--min-pwm", type=int, default=1450)
    pp.add_argument("--max-pwm", type=int, default=1600)
    pp.add_argument("--confirm-prop-out", action="store_true",
                    help="REQUIRED: confirms prop is out of water before pulsing")

    cp = sp.add_parser("configure", help="Set SERVO functions for throttle + steering")
    cp.add_argument("--throttle-ch", type=int, required=True)
    cp.add_argument("--steering-ch", type=int, required=True)
    cp.add_argument("--esc-type", choices=("pwm", "dshot"), default="pwm")
    cp.add_argument("--set-pwm-bounds", action="store_true",
                    help="Also set MIN=1100/TRIM=1500/MAX=1900 for both channels")

    ab = sp.add_parser("arming-bypass", help="Loosen pre-arm checks for bench testing")
    ab.add_argument("--confirm-bench-only", action="store_true", required=False,
                    help="REQUIRED: acknowledges this is for bench, not the field")

    vp = sp.add_parser("verify", help="ARM + brief throttle/steering pulse + DISARM")
    vp.add_argument("--confirm-prop-out", action="store_true",
                    help="REQUIRED: confirms prop is out of water")

    sp.add_parser("fix-streams",
        help="Set SR0_* stream rate params so the GCS gets BATTERY_STATUS / "
             "VIBRATION / VFR_HUD / RC_CHANNELS / etc. (by default ArduRover "
             "leaves several streams disabled; this enables them persistently)")

    rp = sp.add_parser("restore-param", help="Set one param back to a value (rollback helper)")
    rp.add_argument("--name", required=True)
    rp.add_argument("--value", type=float, required=True)

    args = p.parse_args()

    fn_map = {
        "status": cmd_status,
        "probe": cmd_probe,
        "configure": cmd_configure,
        "arming-bypass": cmd_arming_bypass,
        "verify": cmd_verify,
        "restore-param": cmd_restore_param,
        "fix-streams": cmd_fix_streams,
    }
    rc = asyncio.run(fn_map[args.cmd](args))
    sys.exit(rc)


if __name__ == "__main__":
    main()
