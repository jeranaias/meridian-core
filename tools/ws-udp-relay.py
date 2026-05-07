#!/usr/bin/env python3
"""WebSocket <-> UDP MAVLink relay so dronecan-py's mavcan driver (which
speaks UDP) can use our existing WS bridge as transport.

Layout:
    [tablet GCS UI] ---WS---\
                            \
                          [bridge ws:5760] <--USB--> [Cube]
                            /
    [dronecan-py] <--UDP-->/  (this relay sits in the middle)

Run:
    python ws-udp-relay.py --ws ws://100.72.16.72:5760 --udp 127.0.0.1:14550

Then point dronecan-py at  mavcan:udpin:0.0.0.0:14551  with peer 127.0.0.1:14550
(it'll send heartbeats to us; we forward upstream).
"""
from __future__ import annotations
import argparse, asyncio, socket, sys
import websockets

async def main(ws_url, udp_host, udp_port):
    print(f"connecting to {ws_url} ...", flush=True)
    ws = await websockets.connect(ws_url, open_timeout=8,
                                   ping_interval=60, ping_timeout=60,
                                   max_size=2**20)
    print(f"WS up; binding UDP listen on {udp_host}:{udp_port}", flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((udp_host, udp_port))
    sock.setblocking(False)
    loop = asyncio.get_event_loop()

    udp_peer = {"addr": None}

    async def ws_to_udp():
        while True:
            try:
                msg = await ws.recv()
            except websockets.exceptions.ConnectionClosed:
                print("WS closed", flush=True); return
            if not isinstance(msg, bytes):
                continue
            if udp_peer["addr"]:
                try:
                    sock.sendto(msg, udp_peer["addr"])
                except Exception as e:
                    print(f"udp send err: {e}", flush=True)

    async def udp_to_ws():
        while True:
            data, addr = await loop.sock_recvfrom(sock, 65535)
            udp_peer["addr"] = addr
            try:
                await ws.send(data)
            except Exception as e:
                print(f"ws send err: {e}", flush=True); return

    await asyncio.gather(ws_to_udp(), udp_to_ws())

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ws",  default="ws://100.72.16.72:5760")
    p.add_argument("--udp", default="127.0.0.1:14550")
    a = p.parse_args()
    host, port = a.udp.split(":")
    try:
        asyncio.run(main(a.ws, host, int(port)))
    except KeyboardInterrupt:
        print("bye", flush=True)
