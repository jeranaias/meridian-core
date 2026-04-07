#!/usr/bin/env python3
"""
mnp-radio-bridge.py — Bridge between MNP WebSocket (GCS) and serial port (RFD900x radio).

The RFD900x is a transparent serial bridge. MNP frames are COBS-encoded,
so they pass through unchanged. This script bridges the WebSocket from
the GCS to the serial port connected to the radio.

Usage:
    python tools/mnp-radio-bridge.py --port COM3 --baud 57600 --ws-port 5760

    GCS connects to ws://localhost:5760
    Radio is on COM3 at 57600 baud (RFD900x default)

Architecture:
    [Meridian GCS] ←WebSocket→ [This Bridge] ←Serial→ [RFD900x] ~~~radio~~~ [RFD900x] ←Serial→ [CubeOrange+]
"""

import argparse
import asyncio
import logging
import sys

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial required. Install with: pip install pyserial")
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("ERROR: websockets required. Install with: pip install websockets")
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("mnp-bridge")

# COBS framing: 0x00 is the frame delimiter
COBS_DELIMITER = b'\x00'


class RadioBridge:
    def __init__(self, serial_port: str, baud: int):
        self.serial_port = serial_port
        self.baud = baud
        self.ser = None
        self.ws_clients = set()
        self.stats = {
            "serial_rx_bytes": 0,
            "serial_tx_bytes": 0,
            "ws_rx_bytes": 0,
            "ws_tx_bytes": 0,
            "serial_rx_frames": 0,
            "serial_tx_frames": 0,
            "ws_clients": 0,
        }

    def open_serial(self):
        """Open the serial port to the RFD900x radio."""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud,
                timeout=0.01,  # 10ms read timeout (non-blocking-ish)
                write_timeout=1.0,
            )
            log.info(f"Serial opened: {self.serial_port} @ {self.baud} baud")
            return True
        except serial.SerialException as e:
            log.error(f"Failed to open {self.serial_port}: {e}")
            return False

    async def handle_ws_client(self, websocket):
        """Handle a single WebSocket client connection (the GCS)."""
        self.ws_clients.add(websocket)
        self.stats["ws_clients"] = len(self.ws_clients)
        remote = websocket.remote_address
        log.info(f"GCS connected: {remote[0]}:{remote[1]}")

        try:
            async for message in websocket:
                if isinstance(message, bytes):
                    # Forward WebSocket → Serial (GCS → Radio → Vehicle)
                    self.stats["ws_rx_bytes"] += len(message)
                    if self.ser and self.ser.is_open:
                        try:
                            self.ser.write(message)
                            self.stats["serial_tx_bytes"] += len(message)
                            self.stats["serial_tx_frames"] += message.count(b'\x00')
                        except serial.SerialException as e:
                            log.warning(f"Serial write error: {e}")
                elif isinstance(message, str):
                    # Text message — might be a command from GCS
                    log.debug(f"WS text: {message[:80]}")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.stats["ws_clients"] = len(self.ws_clients)
            log.info(f"GCS disconnected: {remote[0]}:{remote[1]}")

    async def serial_reader(self):
        """Read from serial (Radio → Bridge) and forward to all WebSocket clients."""
        buf = bytearray()
        while True:
            if self.ser and self.ser.is_open:
                try:
                    # Read available bytes
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        data = self.ser.read(waiting)
                        self.stats["serial_rx_bytes"] += len(data)
                        buf.extend(data)

                        # Extract COBS frames (delimited by 0x00)
                        while b'\x00' in buf:
                            idx = buf.index(b'\x00')
                            frame = bytes(buf[:idx + 1])  # include delimiter
                            buf = buf[idx + 1:]
                            self.stats["serial_rx_frames"] += 1

                            # Forward to all connected GCS clients
                            if self.ws_clients:
                                await asyncio.gather(
                                    *[ws.send(frame) for ws in self.ws_clients],
                                    return_exceptions=True
                                )
                                self.stats["ws_tx_bytes"] += len(frame) * len(self.ws_clients)
                except serial.SerialException as e:
                    log.warning(f"Serial read error: {e}")
                    await asyncio.sleep(1.0)
            else:
                await asyncio.sleep(0.5)

            # Yield to event loop
            await asyncio.sleep(0.005)  # 5ms — fast enough for 57600 baud

    async def status_printer(self):
        """Print stats every 10 seconds."""
        while True:
            await asyncio.sleep(10)
            s = self.stats
            log.info(
                f"Stats: clients={s['ws_clients']} "
                f"serial_rx={s['serial_rx_bytes']}B/{s['serial_rx_frames']}frames "
                f"serial_tx={s['serial_tx_bytes']}B/{s['serial_tx_frames']}frames "
                f"ws_rx={s['ws_rx_bytes']}B ws_tx={s['ws_tx_bytes']}B"
            )

    async def run(self, ws_host: str, ws_port: int):
        """Start the bridge."""
        if not self.open_serial():
            return

        log.info(f"WebSocket server starting on ws://{ws_host}:{ws_port}")
        log.info(f"Bridge: [GCS] <-ws-> [Bridge] <-serial-> [{self.serial_port}] ~~~radio~~~ [Vehicle]")

        async with websockets.serve(self.handle_ws_client, ws_host, ws_port):
            await asyncio.gather(
                self.serial_reader(),
                self.status_printer(),
            )


def list_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for p in ports:
        print(f"  {p.device:12s}  {p.description}")


def main():
    parser = argparse.ArgumentParser(
        description="MNP radio bridge — WebSocket (GCS) to serial (RFD900x)"
    )
    parser.add_argument("--port", "-p", default="COM3",
                        help="Serial port for RFD900x radio (default: COM3)")
    parser.add_argument("--baud", "-b", type=int, default=57600,
                        help="Baud rate (default: 57600, RFD900x default)")
    parser.add_argument("--ws-host", default="0.0.0.0",
                        help="WebSocket bind address (default: 0.0.0.0)")
    parser.add_argument("--ws-port", type=int, default=5760,
                        help="WebSocket port (default: 5760)")
    parser.add_argument("--list-ports", "-l", action="store_true",
                        help="List available serial ports and exit")

    args = parser.parse_args()

    if args.list_ports:
        list_ports()
        return

    bridge = RadioBridge(args.port, args.baud)
    try:
        asyncio.run(bridge.run(args.ws_host, args.ws_port))
    except KeyboardInterrupt:
        log.info("Bridge stopped.")


if __name__ == "__main__":
    main()
