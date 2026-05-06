"""Orca Core 2 native protocol.

STATUS: NOT YET REVERSE-ENGINEERED.

Orca's Ethernet output carries a "direct NMEA 2000 data stream" that is
richer than standard NMEA 0183 (supports radar, etc.). The wire format is
proprietary; once we have a packet capture (tcpdump) from a live Orca we
fill this in.

Plausible formats to try first based on comparable products:
- Yacht Devices RAW-like ASCII (see yacht_devices_raw.py — same wire, different port)
- Actisense NGT-1 binary framing
- Signal K JSON (see signalk.py — if Orca exposes a Signal K endpoint)

Once we have a sample, this module will:
  1. Detect the framing (length-prefix, delimiter, or stream)
  2. Extract each N2K frame
  3. Hand to dispatch_frame() in yacht_devices_raw.py (same decoder path)
"""
import logging

log = logging.getLogger(__name__)


class OrcaNativeNotImplemented(Exception):
    pass


def detect_and_parse(data: bytes) -> None:
    raise OrcaNativeNotImplemented(
        "Orca native protocol parser not yet implemented. "
        "Capture a sample via `tcpdump -i <iface> -w orca.pcap host <orca-ip>` "
        "and file an issue with the pcap."
    )
