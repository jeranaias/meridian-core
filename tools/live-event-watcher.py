#!/usr/bin/env python3
"""Live event watcher — connects to the MAVLink WebSocket bridge and
emits one stdout line per "interesting" telemetry event. Designed to
feed a Monitor harness so jesse gets notified of mode changes / errors
without seeing the firehose.

Filtered events:
  - ARM / DISARM transitions
  - Mode changes
  - GPS fix-quality transitions (NoFix <-> 3D)
  - STATUSTEXT severity <= 4 (Warning and worse)
  - COMMAND_ACK with result != 0 (not Accepted)
  - Disconnect / reconnect of the WS
"""
from __future__ import annotations
import asyncio, struct, sys, time
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
import websockets

WS_URL = "ws://100.72.16.72:5760"
ROVER_MODES = {0:"MANUAL",1:"ACRO",3:"STEERING",4:"HOLD",5:"LOITER",6:"FOLLOW",7:"SIMPLE",
               10:"AUTO",11:"RTL",12:"SMART_RTL",15:"GUIDED",16:"INITIALISING"}
FIX_NAMES = ["NoGPS","NoFix","2D","3D","DGPS","RTKf","RTK"]
SEV = ["EMERG","ALERT","CRIT","ERROR","WARN","NOTICE","INFO","DEBUG"]

def emit(tag, msg):
    ts = time.strftime("%H:%M:%S")
    print(f"BOAT {ts} {tag} {msg}", flush=True)

async def watch():
    state = {"armed": None, "mode": None, "fix": None, "connected": False}
    backoff = 1.0
    while True:
        try:
            async with websockets.connect(WS_URL, open_timeout=8, ping_interval=20) as ws:
                if not state["connected"]:
                    emit("LINK", f"connected -> {WS_URL}")
                    state["connected"] = True
                backoff = 1.0
                while True:
                    msg = await ws.recv()
                    if not isinstance(msg, bytes): continue
                    # Frames may be batched — split on 0xFD
                    i = 0
                    while i < len(msg):
                        if msg[i] != 0xFD:
                            i += 1; continue
                        if i + 12 > len(msg): break
                        plen = msg[i + 1]
                        end = i + 10 + plen + 2
                        if end > len(msg): break
                        frame = msg[i:end]; i = end
                        mid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
                        p = frame[10:10+plen]
                        if mid == 0 and len(p) >= 8:  # HEARTBEAT
                            cm, _, _, base, _, _ = struct.unpack_from("<IBBBBB", p)
                            armed = bool(base & 0x80)
                            mode = ROVER_MODES.get(int(cm), f"mode{cm}")
                            if state["armed"] is not None and state["armed"] != armed:
                                emit("ARM" if armed else "DISARM", f"now {mode}")
                            elif state["mode"] is not None and state["mode"] != mode:
                                emit("MODE", f"{state['mode']} -> {mode}")
                            state["armed"] = armed
                            state["mode"] = mode
                        elif mid == 24 and len(p) >= 30:  # GPS_RAW_INT
                            fix = p[28]
                            if state["fix"] is not None and state["fix"] != fix:
                                fn_old = FIX_NAMES[state["fix"]] if state["fix"] < 7 else "?"
                                fn_new = FIX_NAMES[fix] if fix < 7 else "?"
                                emit("GPS", f"{fn_old} -> {fn_new}")
                            state["fix"] = fix
                        elif mid == 253 and len(p) >= 2:  # STATUSTEXT
                            sev = p[0]
                            if sev <= 4:
                                txt = bytes(p[1:51]).split(b"\x00",1)[0].decode("ascii",errors="replace")
                                lvl = SEV[sev] if sev < 8 else f"sev{sev}"
                                emit(lvl, txt)
                        elif mid == 77 and len(p) >= 3:  # COMMAND_ACK
                            cmd, result = struct.unpack_from("<HB", p, 0)
                            if result != 0:
                                emit("ACK_FAIL", f"cmd={cmd} result={result}")
        except Exception as e:
            if state["connected"]:
                emit("LINK", f"disconnected ({type(e).__name__}: {str(e)[:80]})")
                state["connected"] = False
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30)

if __name__ == "__main__":
    asyncio.run(watch())
