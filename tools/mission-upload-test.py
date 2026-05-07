#!/usr/bin/env python3
"""Verify the mission upload protocol works against the live FC -- same
flow the tablet GCS now uses after commit 48424d1.

Sends MISSION_COUNT for 3 dummy waypoints near GPS1's last position,
listens for MISSION_REQUEST_INT, replies with MISSION_ITEM_INT, expects
MISSION_ACK with type=0 (ACCEPTED). Either we get the ACK -- protocol
works on both the tablet and the boat -- or we don't, and we have a
concrete error to chase.
"""
from __future__ import annotations
import asyncio, struct, sys, time
import websockets

WS = sys.argv[1] if len(sys.argv) > 1 else "ws://100.72.16.72:5760"

def _crc(b, c):
    t = (b ^ (c & 0xFF)) & 0xFF
    t = (t ^ ((t << 4) & 0xFF)) & 0xFF
    return ((c >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
def crc(d, x):
    c = 0xFFFF
    for b in d: c = _crc(b, c)
    return _crc(x, c)
def encode(mid, payload, crc_extra, seq):
    p = bytes(payload)
    while p and p[-1] == 0:
        p = p[:-1]
    plen = len(p)
    f = struct.pack("<BBBBBBBBBB", 0xFD, plen, 0, 0, seq, 255, 1,
                    mid & 0xFF, (mid >> 8) & 0xFF, (mid >> 16) & 0xFF) + p
    return f + struct.pack("<H", crc(f[1:], crc_extra))

# CRC_EXTRAs from MAVLink common.xml
CE = {
    39: 254,   # MISSION_ITEM (legacy, float coords)
    40: 230,   # MISSION_REQUEST (legacy)
    44: 221,   # MISSION_COUNT
    51: 196,   # MISSION_REQUEST_INT
    73:  38,   # MISSION_ITEM_INT
    47: 153,   # MISSION_ACK
}

def mission_item(seq, lat, lon, alt, command=16, frame=3, current=0,
                 autocontinue=1, mission_type=0,
                 p1=0.0, p2=2.0, p3=0.0, p4=0.0):
    # MISSION_ITEM (legacy) wire order v2 size-desc:
    #   7 floats (param1..4, x, y, z) = 28
    #   2 uint16 (seq, command)        = 4
    #   5 uint8 (tsys, tcomp, frame, current, autocontinue) = 5
    #   1 uint8 mission_type (extension)
    pl = struct.pack(
        "<fffffffHHBBBBBB",
        p1, p2, p3, p4,
        float(lat), float(lon), float(alt),  # x, y, z as floats (legacy)
        seq, command,
        1, 1, frame, current, autocontinue, mission_type,
    )
    return encode(39, pl, CE[39], (seq + 2) & 0xFF)

def mission_count(count, mission_type=0):
    pl = struct.pack("<HBBB", count, 1, 1, mission_type)
    return encode(44, pl, CE[44], 1)

def mission_item_int(seq, lat, lon, alt, command=16, frame=3, current=0,
                     autocontinue=1, mission_type=0,
                     p1=0.0, p2=2.0, p3=0.0, p4=0.0):
    # Wire order (size-desc, 38 bytes): 4 floats, 2 int32, 1 float,
    # 2 uint16, 6 uint8 (target_sys, target_comp, frame, current,
    # autocontinue, mission_type)
    pl = struct.pack(
        "<ffffiifHHBBBBBB",
        p1, p2, p3, p4,
        int(lat * 1e7), int(lon * 1e7),
        alt,
        seq, command,
        1, 1, frame, current, autocontinue, mission_type,
    )
    return encode(73, pl, CE[73], (seq + 2) & 0xFF)

async def main():
    print(f"connecting to {WS} ...", flush=True)
    async with websockets.connect(WS, open_timeout=8,
                                   ping_interval=60, ping_timeout=60) as ws:
        # Pull GPS1 position so we put waypoints somewhere plausible
        gps_pos = {"lat": -38.1729, "lon": 144.5798}  # Geelong fallback
        t_end = time.time() + 3
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
                plen = msg[i + 1]; end = i + 10 + plen + 2
                if end > len(msg): break
                f = msg[i:end]; i = end
                mid = f[7] | (f[8] << 8) | (f[9] << 16)
                p = f[10:10 + plen]
                if mid == 24 and len(p) >= 16:
                    lat = struct.unpack_from("<i", p, 8)[0] / 1e7
                    lon = struct.unpack_from("<i", p, 12)[0] / 1e7
                    if abs(lat) > 0.01:
                        gps_pos["lat"] = lat
                        gps_pos["lon"] = lon
        print(f"  base position: {gps_pos['lat']:.5f}, {gps_pos['lon']:.5f}", flush=True)

        # Build 3 waypoints in a tight triangle ~10m off the GPS1 position
        # (small enough that the FC won't refuse on distance).
        d = 0.0001  # ~11 m
        wps = [
            (gps_pos["lat"] + d, gps_pos["lon"] + d),
            (gps_pos["lat"] + d, gps_pos["lon"] - d),
            (gps_pos["lat"] - d, gps_pos["lon"]),
        ]

        print(f"sending MISSION_COUNT count=3 ...", flush=True)
        await ws.send(mission_count(3))

        # Listen for MISSION_REQUEST_INT (51), reply, until MISSION_ACK (47).
        sent = set()
        ack_type = None
        attempts = 0
        deadline = time.time() + 25
        last_action = time.time()
        msg_counts = {}
        while time.time() < deadline:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=0.4)
            except asyncio.TimeoutError:
                # Retry the count if nothing's happened in 1.5s
                if time.time() - last_action > 1.5 and attempts < 4:
                    attempts += 1
                    print(f"  no requests in 1.5s -- resending COUNT (attempt {attempts})", flush=True)
                    await ws.send(mission_count(3))
                    last_action = time.time()
                continue
            if not isinstance(msg, bytes): continue
            i = 0
            while i < len(msg):
                if msg[i] != 0xFD: i += 1; continue
                if i + 12 > len(msg): break
                plen = msg[i + 1]; end = i + 10 + plen + 2
                if end > len(msg): break
                f = msg[i:end]; i = end
                mid = f[7] | (f[8] << 8) | (f[9] << 16)
                p = f[10:10 + plen]

                if mid == 51 and len(p) >= 2:  # MISSION_REQUEST_INT
                    seq = struct.unpack_from("<H", p, 0)[0]
                    print(f"  -> MISSION_REQUEST_INT seq={seq}", flush=True)
                    if 0 <= seq < len(wps):
                        wp = wps[seq]
                        await ws.send(mission_item_int(seq, wp[0], wp[1], 0))
                        sent.add(seq)
                        last_action = time.time()
                elif mid == 40 and len(p) >= 2:  # MISSION_REQUEST (legacy)
                    seq = struct.unpack_from("<H", p, 0)[0]
                    print(f"  -> MISSION_REQUEST (legacy) seq={seq}", flush=True)
                    if 0 <= seq < len(wps):
                        wp = wps[seq]
                        # ArduPilot wants MISSION_ITEM_INT in response even
                        # when it sent the legacy request. The STATUSTEXT
                        # "got MISSION_ITEM; GCS should send MISSION_ITEM_INT"
                        # confirms the preference.
                        await ws.send(mission_item_int(seq, wp[0], wp[1], 0))
                        sent.add(seq)
                        last_action = time.time()
                elif mid == 47 and len(p) >= 2:  # MISSION_ACK
                    # wire order v2: target_sys(1) target_comp(1) type(1) [mission_type(1)]
                    # type=0 strips, so payload may be only 2 bytes
                    ack_type = p[2] if len(p) > 2 else 0
                    mt = p[3] if len(p) > 3 else 0
                    print(f"  -> MISSION_ACK type={ack_type} mission_type={mt}", flush=True)
                    deadline = time.time()  # break outer loop
                    break
                elif mid == 253 and len(p) >= 2:  # STATUSTEXT
                    sev = p[0]
                    txt = bytes(p[1:51]).split(b"\x00",1)[0].decode("ascii", errors="replace")
                    print(f"  FC: [sev{sev}] {txt}", flush=True)
                else:
                    # Print every msgid we see while testing -- helps spot
                    # if FC is replying with something unexpected.
                    msg_counts.setdefault(mid, 0)
                    msg_counts[mid] += 1

        print()
        print(f"msgid traffic seen: {dict(sorted(msg_counts.items()))}")
        print()
        if ack_type == 0:
            print(f"PASS: FC accepted {len(sent)}/{len(wps)} items, ACK=ACCEPTED")
            sys.exit(0)
        elif ack_type is None:
            print(f"FAIL: timeout, no MISSION_ACK received. items sent: {sorted(sent)}")
            sys.exit(2)
        else:
            err_names = {0:"ACCEPTED",1:"ERROR",2:"UNSUPPORTED_FRAME",
                         3:"UNSUPPORTED",4:"NO_SPACE",5:"INVALID",
                         6:"INVALID_PARAM1",7:"INVALID_PARAM2",
                         8:"INVALID_PARAM3",9:"INVALID_PARAM4",
                         10:"INVALID_PARAM5_X",11:"INVALID_PARAM6_Y",
                         12:"INVALID_PARAM7",13:"INVALID_SEQUENCE",
                         14:"DENIED",15:"OPERATION_CANCELLED"}
            print(f"FAIL: FC rejected with type={ack_type} ({err_names.get(ack_type,'?')})")
            sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())
