"""Yacht Devices RAW protocol over UDP/TCP.

YDWG-02 / YDEN-02 / YDNU-02 all support this. Orca's proprietary format is
similar enough that this may work out of the box after a wire-capture test.

Wire format (line-oriented ASCII, one CAN frame per line, terminated \\r\\n):

  <time>,R,<can_id_hex>,<data_byte_0>,...,<data_byte_7>\\r\\n

Example:
  17:33:21.107,R,09F8027F,05,C7,88,00,FF,7F,FF,7F

CAN id contains priority + PGN + source address (standard J1939 format).
This module parses the line and converts to (pgn, source_addr, payload).
"""
import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from ..decode.pgn import decode as pgn_decode
from ..state import BoatState


log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# J1939 / NMEA 2000 CAN ID parsing
# ---------------------------------------------------------------------------

def parse_can_id(can_id: int) -> "tuple[int, int, int]":
    """Return (priority, pgn, source_addr) from an extended 29-bit J1939 CAN ID.

    PGN construction:
      pgn = (PF << 8) | PS   if PF >= 240 (PDU2 — broadcast/global, PS is group ext)
      pgn = (PF << 8)        if PF < 240  (PDU1 — PS is destination, not part of PGN)
    """
    priority = (can_id >> 26) & 0x7
    pf = (can_id >> 16) & 0xFF
    ps = (can_id >> 8) & 0xFF
    sa = can_id & 0xFF
    if pf < 240:
        pgn = pf << 8              # PDU1: destination-addressed, PGN ignores PS
    else:
        pgn = (pf << 8) | ps       # PDU2: broadcast, PS is group ext
    return priority, pgn, sa


# ---------------------------------------------------------------------------
# Fast-packet reassembly (for PGNs > 8 bytes)
# ---------------------------------------------------------------------------

class FastPacketAssembler:
    """NMEA 2000 fast-packet protocol reassembles multi-frame PGNs using a
    sequence counter in byte 0. Per-source-per-PGN state is kept briefly."""

    def __init__(self) -> None:
        # key = (source_addr, pgn) -> {counter, expected, buf}
        self._pending: dict = {}

    def feed(self, source_addr: int, pgn: int, frame: bytes) -> Optional[bytes]:
        """Feed one 8-byte CAN frame. Returns full payload if assembled."""
        if len(frame) < 2:
            return None
        key = (source_addr, pgn)
        seq_frame = frame[0]
        frame_counter = seq_frame & 0x1F     # low 5 bits = sequence
        seq_id = (seq_frame >> 5) & 0x07     # high 3 bits = sequence group id

        if frame_counter == 0:
            # First frame: byte 1 = total byte count, bytes 2..7 = first 6 data
            total = frame[1]
            self._pending[key] = {
                "seq_id": seq_id, "expected": total,
                "buf": bytearray(frame[2:]),
            }
            if total <= 6:
                return bytes(self._pending.pop(key)["buf"][:total])
            return None

        entry = self._pending.get(key)
        if entry is None or entry["seq_id"] != seq_id:
            return None                       # lost the start; drop
        entry["buf"].extend(frame[1:])
        if len(entry["buf"]) >= entry["expected"]:
            result = bytes(entry["buf"][:entry["expected"]])
            del self._pending[key]
            return result
        return None


# ---------------------------------------------------------------------------
# Single-packet handling
# ---------------------------------------------------------------------------

# PGNs that are always single-frame (payload ≤ 8 bytes)
_SINGLE_FRAME_PGNS = {
    129025, 129026, 127250, 128267, 130306,
}


def dispatch_frame(can_id: int, data: bytes, state: BoatState,
                   assembler: FastPacketAssembler) -> bool:
    _prio, pgn, sa = parse_can_id(can_id)
    if pgn in _SINGLE_FRAME_PGNS:
        return pgn_decode(pgn, data, state)
    # Fast-packet (multi-frame)
    payload = assembler.feed(sa, pgn, data)
    if payload is None:
        return False
    return pgn_decode(pgn, payload, state)


# ---------------------------------------------------------------------------
# UDP reader
# ---------------------------------------------------------------------------

@dataclass
class RawUdpConfig:
    listen_host: str = "0.0.0.0"
    listen_port: int = 1457


class YachtDevicesRawUdp:
    """Listens for Yacht Devices RAW ASCII lines on UDP and feeds state."""

    def __init__(self, cfg: RawUdpConfig, state: BoatState) -> None:
        self.cfg = cfg
        self.state = state
        self._stop = threading.Event()
        self._thr: Optional[threading.Thread] = None
        self._assembler = FastPacketAssembler()
        self._sock: Optional[socket.socket] = None

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.cfg.listen_host, self.cfg.listen_port))
        self._sock.settimeout(1.0)
        self._thr = threading.Thread(target=self._run, daemon=True, name="yd-raw-udp")
        self._thr.start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            try: self._sock.close()
            except: pass

    def _run(self) -> None:
        log.info("Listening Yacht Devices RAW UDP on %s:%d",
                 self.cfg.listen_host, self.cfg.listen_port)
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except Exception as e:
                log.debug("recv err: %s", e)
                continue
            self._handle_datagram(data)

    def _handle_datagram(self, data: bytes) -> None:
        text = data.decode("ascii", errors="ignore")
        for line in text.splitlines():
            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        # Expected: "HH:MM:SS.mmm,R,HHHHHHHH,DD,DD,DD,DD,DD,DD,DD,DD"
        parts = line.strip().split(",")
        if len(parts) < 4:
            return
        if parts[1] not in ("R", "T"):
            return
        try:
            can_id = int(parts[2], 16)
            payload = bytes(int(p, 16) for p in parts[3:] if p)
        except ValueError:
            return
        self.state.packets_seen += 1
        if dispatch_frame(can_id, payload, self.state, self._assembler):
            self.state.packets_decoded += 1
