#!/usr/bin/env python3
"""
mavlink-ws-bridge.py — Universal MAVLink to WebSocket bridge.

Takes MAVLink from a serial port, UDP endpoint, or TCP connection and exposes
it to browser-based GCS clients over WebSocket.  Bidirectional: telemetry
flows vehicle → browser, commands flow browser → vehicle.

Designed to run on any deployment topology:

  1. Companion computer on the boat (Pi/Jetson connected to FC via USB):
     python3 mavlink-ws-bridge.py --serial /dev/ttyACM0 --baud 115200

  2. Ground station laptop with an RFD900x USB radio:
     python3 mavlink-ws-bridge.py --serial COM4 --baud 57600

  3. Alongside MAVProxy (MAVProxy already forwarding to UDP):
     mavproxy.py --master /dev/ttyACM0 --out udp:127.0.0.1:14550
     python3 mavlink-ws-bridge.py --udp 127.0.0.1:14550

  4. TCP source (e.g., ArduPilot SITL on localhost):
     python3 mavlink-ws-bridge.py --tcp 127.0.0.1:5760

The bridge always listens for WebSocket clients on --ws-host:--ws-port
(default 0.0.0.0:5760).  Connect the tablet at:

    http://<server>:8080/mission.html?proto=mavlink&ws=ws://<tailscale-ip>:5760

Requires: pip install websockets pyserial
"""

import argparse
import asyncio
import logging
import signal
import sys
from typing import Optional, Set

try:
    import websockets
except ImportError:
    print("ERROR: pip install websockets", file=sys.stderr)
    sys.exit(1)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("mav-ws")


# ── Transport abstractions ────────────────────────────────────

class Transport:
    """Bidirectional byte stream to the vehicle."""

    async def read(self) -> bytes:
        raise NotImplementedError

    async def write(self, data: bytes) -> None:
        raise NotImplementedError

    async def close(self) -> None:
        raise NotImplementedError


class SerialTransport(Transport):
    """Serial port (USB CDC or FTDI RFD900x).

    Self-healing: if `port` is "auto" or read/write start failing, the
    transport rescans for a Cube COM port and reopens — so an FC reboot
    that re-enumerates as a different COM port no longer requires a
    manual bridge restart.
    """

    # USB VIDs we treat as a Cube/Pixhawk family flight controller.
    # 2DAE = Hex/CubePilot, 1209 = pid.codes (some variants), 26AC = 3DR.
    FC_VIDS = {0x2DAE, 0x1209, 0x26AC}

    def __init__(self, port: str, baud: int):
        try:
            import serial  # noqa: F401
            from serial.tools import list_ports  # noqa: F401
        except ImportError:
            log.error("pip install pyserial")
            sys.exit(1)
        self.requested_port = port
        self.baud = baud
        self.ser = None
        self._open_blocking()  # sync open at startup so we fail fast if no FC present

    @classmethod
    def _autodetect(cls):
        """Return the lowest-numbered COM port whose USB VID matches a
        flight controller. None if no candidates."""
        from serial.tools import list_ports
        cands = []
        for p in list_ports.comports():
            vid = getattr(p, "vid", None)
            if vid in cls.FC_VIDS:
                cands.append(p.device)
        if not cands:
            return None
        # Sort by numeric COM index when possible (COM3 < COM10).
        def keyfn(name):
            digits = "".join(c for c in name if c.isdigit())
            return (int(digits) if digits else 0, name)
        cands.sort(key=keyfn)
        return cands[0]

    def _open_blocking(self):
        import serial
        port = self.requested_port
        if port.lower() == "auto":
            port = self._autodetect()
            if port is None:
                raise RuntimeError("No flight controller COM port found (VID 2DAE/1209/26AC)")
        log.info(f"Opening serial port {port} @ {self.baud} baud")
        ser = serial.Serial(port, self.baud, timeout=0)
        ser.reset_input_buffer()
        self.ser = ser
        self.current_port = port
        log.info(f"Serial port open: {port}")

    def _try_reconnect(self) -> bool:
        """Close the dead handle and try to find/reopen a Cube COM port.
        Returns True on success."""
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

        # Always autodetect on reconnect — even if the user passed an
        # explicit port, the FC may have re-enumerated as a different
        # COM number after a reboot. Auto-detect picks up the new one.
        new_port = self._autodetect()
        if new_port is None:
            return False
        try:
            import serial
            ser = serial.Serial(new_port, self.baud, timeout=0)
            ser.reset_input_buffer()
            self.ser = ser
            self.current_port = new_port
            log.info(f"Serial reconnected on {new_port}")
            return True
        except Exception as e:
            log.warning(f"Reconnect attempt failed on {new_port}: {e}")
            return False

    async def read(self) -> bytes:
        loop = asyncio.get_event_loop()
        data = await loop.run_in_executor(None, self._read_blocking)
        return data

    def _read_blocking(self):
        import time
        if self.ser is None:
            # No port — try to recover, then yield
            if self._try_reconnect():
                return b""
            time.sleep(1.0)  # back off when no FC is present
            return b""
        try:
            n = self.ser.in_waiting
            if n > 0:
                return self.ser.read(n)
        except Exception as e:
            log.error(f"Serial read error: {e}; attempting reconnect")
            self._try_reconnect()
            return b""
        time.sleep(0.005)
        return b""

    async def write(self, data: bytes) -> None:
        if self.ser is None:
            return
        try:
            self.ser.write(data)
        except Exception as e:
            log.error(f"Serial write error: {e}; attempting reconnect")
            self._try_reconnect()

    async def close(self) -> None:
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


class UdpTransport(Transport):
    """UDP endpoint (MAVProxy --out udp:host:port)."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.peer = None
        self.transport = None
        self.queue: asyncio.Queue = asyncio.Queue()
        log.info(f"UDP bridge mode: listening on {host}:{port}")

    class _Protocol(asyncio.DatagramProtocol):
        def __init__(self, outer):
            self.outer = outer

        def connection_made(self, transport):
            self.outer.transport = transport

        def datagram_received(self, data, addr):
            self.outer.peer = addr
            self.outer.queue.put_nowait(data)

        def error_received(self, exc):
            log.error(f"UDP error: {exc}")

    async def start(self):
        loop = asyncio.get_event_loop()
        await loop.create_datagram_endpoint(
            lambda: self._Protocol(self),
            local_addr=(self.host, self.port),
        )

    async def read(self) -> bytes:
        return await self.queue.get()

    async def write(self, data: bytes) -> None:
        if self.transport and self.peer:
            self.transport.sendto(data, self.peer)

    async def close(self) -> None:
        if self.transport:
            self.transport.close()


class TcpTransport(Transport):
    """TCP client (connect to SITL or another TCP MAVLink source)."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None

    async def start(self):
        log.info(f"Connecting to TCP {self.host}:{self.port}")
        self.reader, self.writer = await asyncio.open_connection(self.host, self.port)
        log.info("TCP connected")

    async def read(self) -> bytes:
        if not self.reader:
            return b""
        try:
            data = await self.reader.read(4096)
            if not data:
                log.warning("TCP connection closed by peer")
            return data
        except Exception as e:
            log.error(f"TCP read error: {e}")
            return b""

    async def write(self, data: bytes) -> None:
        if self.writer:
            try:
                self.writer.write(data)
                await self.writer.drain()
            except Exception as e:
                log.error(f"TCP write error: {e}")

    async def close(self) -> None:
        if self.writer:
            self.writer.close()
            try:
                await self.writer.wait_closed()
            except Exception:
                pass


# ── Bridge core ───────────────────────────────────────────────

class Bridge:
    def __init__(self, transport: Transport):
        self.transport = transport
        self.ws_clients: Set = set()
        self.bytes_from_vehicle = 0
        self.bytes_to_vehicle = 0
        self.heartbeats_seen = 0
        self.last_stats = 0

    async def handle_ws_client(self, ws):
        addr = ws.remote_address
        self.ws_clients.add(ws)
        log.info(f"WS client connected: {addr} (total={len(self.ws_clients)})")
        try:
            async for msg in ws:
                if isinstance(msg, bytes):
                    self.bytes_to_vehicle += len(msg)
                    await self.transport.write(msg)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            log.error(f"WS client error: {e}")
        finally:
            self.ws_clients.discard(ws)
            log.info(f"WS client disconnected: {addr} (total={len(self.ws_clients)})")

    async def vehicle_to_ws_task(self):
        """Read from transport, broadcast to all WebSocket clients."""
        buf = bytearray()
        while True:
            try:
                data = await self.transport.read()
                if not data:
                    await asyncio.sleep(0.005)
                    continue
                self.bytes_from_vehicle += len(data)
                buf.extend(data)
                # Count MAVLink v2 heartbeats (0xFD start, msg id 0)
                i = 0
                while i < len(buf) - 12:
                    if buf[i] == 0xFD:
                        plen = buf[i + 1]
                        if i + 10 + plen + 2 > len(buf):
                            break  # incomplete
                        msg_id = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                        if msg_id == 0:
                            self.heartbeats_seen += 1
                        i += 10 + plen + 2
                    else:
                        i += 1
                # Drop processed bytes to prevent unbounded growth
                if len(buf) > 8192:
                    buf = buf[-4096:]

                # Broadcast to all WebSocket clients
                if self.ws_clients:
                    dead = []
                    for ws in list(self.ws_clients):
                        try:
                            await ws.send(bytes(data))
                        except Exception as e:
                            log.error(f"WS send error: {type(e).__name__}: {e}")
                            dead.append(ws)
                    for ws in dead:
                        self.ws_clients.discard(ws)
            except Exception as e:
                log.error(f"Vehicle→WS error: {e}")
                await asyncio.sleep(0.1)

    async def stats_task(self):
        """Print periodic stats."""
        while True:
            await asyncio.sleep(5)
            log.info(
                f"Stats: {self.heartbeats_seen} heartbeats, "
                f"↓{self.bytes_from_vehicle}B ↑{self.bytes_to_vehicle}B, "
                f"{len(self.ws_clients)} clients"
            )


# ── Main ──────────────────────────────────────────────────────

async def main_async(args):
    # Create transport
    transport: Transport
    if args.serial:
        transport = SerialTransport(args.serial, args.baud)
    elif args.udp:
        host, port = args.udp.split(":")
        transport = UdpTransport(host, int(port))
        await transport.start()
    elif args.tcp:
        host, port = args.tcp.split(":")
        transport = TcpTransport(host, int(port))
        await transport.start()
    else:
        log.error("Must specify one of: --serial, --udp, --tcp")
        sys.exit(1)

    bridge = Bridge(transport)

    # Start WebSocket server
    log.info(f"WebSocket server: ws://{args.ws_host}:{args.ws_port}")
    log.info(f"Tablet URL: mission.html?proto=mavlink&ws=ws://<host>:{args.ws_port}")

    async with websockets.serve(bridge.handle_ws_client, args.ws_host, args.ws_port):
        # Run background tasks
        tasks = [
            asyncio.create_task(bridge.vehicle_to_ws_task()),
            asyncio.create_task(bridge.stats_task()),
        ]

        # Wait for SIGINT
        stop = asyncio.Event()

        def handle_sig():
            log.info("Shutdown requested")
            stop.set()

        try:
            asyncio.get_event_loop().add_signal_handler(signal.SIGINT, handle_sig)
            asyncio.get_event_loop().add_signal_handler(signal.SIGTERM, handle_sig)
        except NotImplementedError:
            # Windows: add_signal_handler not supported
            pass

        try:
            await stop.wait()
        except KeyboardInterrupt:
            pass
        finally:
            for task in tasks:
                task.cancel()
            await transport.close()
            log.info("Shutdown complete")


def main():
    p = argparse.ArgumentParser(
        description="MAVLink <-> WebSocket bridge for the Meridian tablet app",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:

  Companion computer (USB to CubeOrange):
    %(prog)s --serial /dev/ttyACM0 --baud 115200

  Ground station (RFD900x USB):
    %(prog)s --serial COM4 --baud 57600

  MAVProxy forwarding to UDP:
    mavproxy.py --master /dev/ttyACM0 --out udp:127.0.0.1:14550
    %(prog)s --udp 127.0.0.1:14550

  ArduPilot SITL over TCP:
    %(prog)s --tcp 127.0.0.1:5760

Then open the tablet:
    http://localhost:8080/mission.html?proto=mavlink&ws=ws://<TAILSCALE_IP>:5760
        """,
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--serial", metavar="PORT",
                     help="Serial port (e.g., COM4, /dev/ttyACM0). "
                          "Use 'auto' to autodetect Cube COM port and self-heal "
                          "across FC reboots.")
    src.add_argument("--udp", metavar="HOST:PORT",
                     help="UDP endpoint to receive MAVLink from")
    src.add_argument("--tcp", metavar="HOST:PORT",
                     help="TCP endpoint to connect to")
    p.add_argument("--baud", type=int, default=115200,
                   help="Serial baud rate (default: 115200)")
    p.add_argument("--ws-host", default="0.0.0.0",
                   help="WebSocket listen host (default: 0.0.0.0)")
    p.add_argument("--ws-port", type=int, default=5760,
                   help="WebSocket listen port (default: 5760)")
    args = p.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
