# meridian-orca-bridge

Python companion-computer daemon that ingests marine data from an **Orca Core 2**
(NMEA2000-over-Ethernet gateway) and forwards it to an ArduPilot flight
controller over MAVLink.

## Why

The Vanguard USV strategy is **one GPS — the Orca's** — no dedicated pucks on
deck. This daemon is the software piece that makes that work:

```
Orca Core 2 ─── Ethernet ───► this daemon ─── MAVLink GPS_INPUT ───► Cube Plus
(10 Hz GNSS,                                     + SCALED_PRESSURE
 9-axis IMU,                                     + DISTANCE_SENSOR
 N2K bridge)                                     + GLOBAL_POSITION
                                                 + WIND_COV, etc.
```

The Cube sees the Orca's GPS as `GPS_TYPE=14` (MAVLink GPS) and blends/fails-over
to any existing serial GPS via `GPS_TYPE2`.

## Architecture

- **protocol/** — wire-level reader. Autodetects Signal K WebSocket, Yacht
  Devices RAW over UDP/TCP, or Orca's native format once reverse-engineered.
- **decode/** — NMEA 2000 PGN decoder. Wraps the `nmea2000` pip package
  (which uses the canboat database).
- **state.py** — running EKF-input state: GPS fix, heading, depth, wind, AIS.
- **mavlink/** — outbound MAVLink sender: GPS_INPUT, DISTANCE_SENSOR,
  WIND_COV, GLOBAL_POSITION_INT.
- **mock.py** — synthetic N2K stream generator for testing without real
  hardware.

## Modes

| Mode | Source | Target | Use |
|---|---|---|---|
| `--mock` | Internal generator | Internal sink | Development / CI |
| `--replay FILE.pcap` | Captured pcap | Configured MAVLink | Offline testing |
| `--live` | Real Orca IP | Real Cube via MAVLink | Production |

## Quick start (dev)

    pip install -r requirements.txt
    python -m meridian_orca --mock --mavlink-out udpout:127.0.0.1:14550 -v

Then `mavproxy.py --master udp:127.0.0.1:14550` or any MAVLink GCS to see the
synthetic GPS_INPUT arrive.

## Stand-on-shoulders credits

- **[canboat/canboat](https://github.com/canboat/canboat)** — NMEA2000 PGN database, 1000+ messages, MIT license.
- **[tomer-w/nmea2000](https://github.com/tomer-w/nmea2000)** — Python decoder built on canboat DB.
- **[mavlink/pymavlink](https://github.com/mavlink/pymavlink)** — MAVLink wire format.
