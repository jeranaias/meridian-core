#!/usr/bin/env python3
"""End-to-end pre-flight audit. Connects to the bridge, scrapes ~12s of
telemetry, requests pre-arm checks, queries critical params, and reports
everything in one shot."""
from __future__ import annotations
import asyncio, struct, sys, time, math
import websockets

WS = sys.argv[1] if len(sys.argv) > 1 else "ws://100.72.16.72:5760"
WATCH_S = 12.0

WANT_PARAMS = [
    "ARMING_CHECK", "BRD_SAFETY_DEFLT", "FS_THR_ENABLE",
    "AHRS_EKF_TYPE", "EK3_SRC1_YAW",
    "GPS1_TYPE", "GPS2_TYPE", "GPS_PRIMARY", "GPS_AUTO_CONFIG",
    "COMPASS_ENABLE", "COMPASS_ORIENT",
    "SERIAL0_PROTOCOL", "SERIAL0_BAUD",
    "SERIAL1_PROTOCOL", "SERIAL1_BAUD",
    "SERVO1_FUNCTION", "SERVO2_FUNCTION", "SERVO3_FUNCTION", "SERVO4_FUNCTION",
    "BATT_MONITOR", "BATT_CAPACITY",
    "FRAME_TYPE", "FRAME_CLASS",
]

ROVER_MODES = {0:"MANUAL",1:"ACRO",3:"STEERING",4:"HOLD",5:"LOITER",
               6:"FOLLOW",7:"SIMPLE",10:"AUTO",11:"RTL",12:"SMART_RTL",
               15:"GUIDED",16:"INITIALISING"}
FIX_NAMES = {0:"NoGPS",1:"NoFix",2:"2D",3:"3D",4:"DGPS",5:"RTKf",6:"RTK"}

def _crc(b, c):
    t = (b ^ (c & 0xFF)) & 0xFF
    t = (t ^ ((t << 4) & 0xFF)) & 0xFF
    return ((c >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
def crc(d, x):
    c = 0xFFFF
    for b in d: c = _crc(b, c)
    return _crc(x, c)
def enc(mid, payload, crc_extra, seq):
    p = bytes(payload)
    while p and p[-1] == 0:
        p = p[:-1]
    plen = len(p)
    f = struct.pack("<BBBBBBBBBB", 0xFD, plen, 0, 0, seq, 255, 1,
                    mid & 0xFF, (mid >> 8) & 0xFF, (mid >> 16) & 0xFF) + p
    return f + struct.pack("<H", crc(f[1:], crc_extra))

def req_param(name, sq):
    nb = name.encode("ascii")[:16].ljust(16, b"\x00")
    return enc(20, struct.pack("<hBB", -1, 1, 0) + nb, 214, sq)
def cmd_long(cmd, p1=0, sq=1):
    pl = struct.pack("<fffffffHBBB", p1, 0, 0, 0, 0, 0, 0, cmd, 1, 0, 0)
    return enc(76, pl, 152, sq)

async def main():
    state = {
        "armed": None, "mode": None, "lat": None, "lon": None,
        "alt": None, "hdg": None, "spd": None, "yaw": None,
        "voltage": None, "current": None, "batt_pct": None,
        "ekf": None,
    }
    gps1 = {"fix": None, "sats": None, "len": 0}
    gps2 = {"fix": None, "sats": None, "len": 0}
    statustexts = []
    msg_counts = {}
    params = {}
    servo_pwm = [None] * 8

    print(f"connecting to {WS} for {WATCH_S}s ...", flush=True)
    try:
        async with websockets.connect(WS, open_timeout=8,
                                       ping_interval=60, ping_timeout=60) as ws:
            sq = 1
            for n in WANT_PARAMS:
                await ws.send(req_param(n, sq)); sq = (sq + 1) & 0xFF
            await ws.send(cmd_long(401, sq=sq)); sq = (sq + 1) & 0xFF  # PreArm checks
            t_end = time.time() + WATCH_S
            while time.time() < t_end:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=0.5)
                except asyncio.TimeoutError:
                    continue
                if not isinstance(msg, bytes): continue
                i = 0
                while i < len(msg):
                    if msg[i] != 0xFD: i += 1; continue
                    if i + 12 > len(msg): break
                    plen = msg[i+1]; end = i + 10 + plen + 2
                    if end > len(msg): break
                    f = msg[i:end]; i = end
                    mid = f[7] | (f[8]<<8) | (f[9]<<16)
                    p = f[10:10+plen]
                    msg_counts[mid] = msg_counts.get(mid, 0) + 1

                    if mid == 0 and len(p) >= 6:  # HEARTBEAT
                        cm = struct.unpack_from("<I", p, 0)[0]
                        base = p[5]
                        state["mode"] = ROVER_MODES.get(int(cm), f"mode{cm}")
                        state["armed"] = bool(base & 0x80)
                    elif mid == 22 and len(p) >= 17:  # PARAM_VALUE
                        v = struct.unpack_from("<f", p, 0)[0]
                        pid = bytes(p[8:24]).split(b"\x00",1)[0].decode("ascii", errors="replace")
                        if pid in WANT_PARAMS:
                            params[pid] = v
                    elif mid == 24 and len(p) >= 16:  # GPS_RAW_INT
                        gps1["len"] = max(gps1["len"], plen)
                        if len(p) >= 30:
                            gps1["fix"] = p[28]; gps1["sats"] = p[29]
                    elif mid == 124:  # GPS2_RAW
                        gps2["len"] = max(gps2["len"], plen)
                        if len(p) >= 34:
                            gps2["fix"] = p[32]; gps2["sats"] = p[33]
                    elif mid == 30 and len(p) >= 16:  # ATTITUDE
                        state["yaw"] = struct.unpack_from("<f", p, 12)[0]
                    elif mid == 33 and len(p) >= 28:  # GLOBAL_POSITION_INT
                        # time_ms(u32) lat(i32) lon(i32) alt_mm(i32) rel_alt(i32)
                        # vx vy vz (i16) hdg (u16)
                        state["lat"] = struct.unpack_from("<i", p, 4)[0] / 1e7
                        state["lon"] = struct.unpack_from("<i", p, 8)[0] / 1e7
                        if len(p) >= 28:
                            hdg = struct.unpack_from("<H", p, 26)[0]
                            if hdg < 36000:
                                state["hdg"] = hdg / 100.0
                    elif mid == 74 and len(p) >= 16:  # VFR_HUD
                        # f32 airspd, groundspd, alt, climb, i16 hdg, u16 throttle
                        state["spd"] = struct.unpack_from("<f", p, 4)[0]
                    elif mid == 1 and len(p) >= 30:  # SYS_STATUS
                        # voltage at offset 14 (u16, mV)
                        state["voltage"] = struct.unpack_from("<H", p, 14)[0] / 1000.0
                        state["current"] = struct.unpack_from("<h", p, 16)[0] / 100.0
                        if len(p) >= 31:
                            state["batt_pct"] = p[30] if p[30] <= 100 else None
                    elif mid == 36 and len(p) >= 21:  # SERVO_OUTPUT_RAW
                        for ch in range(8):
                            servo_pwm[ch] = struct.unpack_from("<H", p, 5 + ch*2)[0]
                    elif mid == 193 and len(p) >= 22:  # EKF_STATUS_REPORT
                        state["ekf"] = {
                            "vel_var": struct.unpack_from("<f", p, 0)[0],
                            "pos_h":   struct.unpack_from("<f", p, 4)[0],
                            "compass": struct.unpack_from("<f", p, 12)[0],
                            "flags":   struct.unpack_from("<H", p, 20)[0],
                        }
                    elif mid == 253 and len(p) >= 2:  # STATUSTEXT
                        sev = p[0]
                        txt = bytes(p[1:51]).split(b"\x00",1)[0].decode("ascii", errors="replace")
                        if sev <= 5:
                            statustexts.append((sev, txt))
    except Exception as e:
        print(f"connection error: {e}")
        return

    fmt_int = lambda v: int(v) if isinstance(v, float) and v == int(v) else v
    fmt_v = lambda v: "—" if v is None else (round(v, 4) if isinstance(v, float) else v)

    print()
    print("============== BRIDGE / LINK ==============")
    print(f"  msgid types in 12s: {len(msg_counts)}")
    print(f"  HEARTBEATs:    {msg_counts.get(0, 0)} ({msg_counts.get(0, 0)/WATCH_S:.1f}/s)")
    print(f"  GPS_RAW_INT:   {msg_counts.get(24, 0)}")
    print(f"  GPS2_RAW:      {msg_counts.get(124, 0)}")
    print(f"  ATTITUDE:      {msg_counts.get(30, 0)}")
    print(f"  PARAM_VALUE:   {msg_counts.get(22, 0)}")
    print()
    print("============== VEHICLE STATE ==============")
    print(f"  mode:     {state['mode']}")
    print(f"  armed:    {state['armed']}")
    if state["lat"] is not None:
        print(f"  position: {state['lat']:.6f}, {state['lon']:.6f}")
    print(f"  hdg:      {fmt_v(state['hdg'])}°  (yaw rad: {fmt_v(state['yaw'])})")
    print(f"  speed:    {fmt_v(state['spd'])} m/s")
    print(f"  voltage:  {fmt_v(state['voltage'])} V    current: {fmt_v(state['current'])} A    batt: {fmt_v(state['batt_pct'])}%")
    print()
    print("============== GPS ==============")
    print(f"  GPS1: fix={FIX_NAMES.get(gps1['fix'], gps1['fix'])}  sats={gps1['sats']}  payload_max={gps1['len']}")
    print(f"  GPS2: fix={FIX_NAMES.get(gps2['fix'], gps2['fix'])}  sats={gps2['sats']}  payload_max={gps2['len']}")
    print()
    print("============== SERVO OUTPUTS (us) ==============")
    print(f"  ch1-8: {servo_pwm}")
    print()
    print("============== EKF ==============")
    if state["ekf"]:
        e = state["ekf"]
        print(f"  vel_var={e['vel_var']:.3f}  pos_h={e['pos_h']:.3f}  compass={e['compass']:.3f}  flags=0x{e['flags']:04x}")
    else:
        print("  no EKF_STATUS_REPORT received")
    print()
    print("============== KEY PARAMS ==============")
    for n in WANT_PARAMS:
        v = params.get(n, "<not received>")
        if isinstance(v, float): v = fmt_int(v)
        print(f"  {n:22s} = {v}")
    print()
    print("============== STATUSTEXTs (sev <= 5) ==============")
    seen = set()
    for sev, txt in statustexts:
        if (sev, txt) in seen: continue
        seen.add((sev, txt))
        print(f"  [sev{sev}] {txt}")
    if not statustexts:
        print("  (none — clean)")

if __name__ == "__main__":
    asyncio.run(main())
