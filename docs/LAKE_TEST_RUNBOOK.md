# Vanguard USV Lake Test Runbook

Everything needed to run the first hardware test with the Meridian tablet
interface controlling an ArduPilot-based CubeOrange+ over Tailscale.

**Intended audience:** Jesse (Monterey, CA) + Vanguard team (boat location).

---

## 1. Architecture

```
    ┌──────────────────────────────────────────────────────────┐
    │  BOAT SIDE                                               │
    │                                                          │
    │   CubeOrange+ (ArduPilot, MAVLink v2)                    │
    │        │                                                 │
    │        ├── USB ──┬── Companion computer (Pi/Jetson)      │
    │        │         │   + Tailscale                         │
    │        │         │   + mavlink-ws-bridge.py              │
    │        │         │                                       │
    │        └── RFD900x ──── (RF link) ──── RFD900x           │
    │                                          │               │
    └──────────────────────────────────────────┼───────────────┘
                                               │
    ┌──────────────────────────────────────────┼───────────────┐
    │  GROUND STATION                          │               │
    │                                          │               │
    │   Ground laptop                          ├── USB ────────┘
    │   + Tailscale                            │
    │   + mavlink-ws-bridge.py  (EITHER here OR on companion)  │
    │   + python -m http.server 8080                           │
    │                                                          │
    └──────────────────────────────────────────────────────────┘
                              │
                              │  Tailscale network
                              │
    ┌─────────────────────────┼────────────────────────────────┐
    │  JESSE (Monterey)       │                                │
    │                                                          │
    │   Browser: mission.html?proto=mavlink&ws=ws://TSIP:5761  │
    │                                                          │
    └──────────────────────────────────────────────────────────┘
```

**Where does `mavlink-ws-bridge.py` run?**

Pick the machine that has direct access to the MAVLink source:

- **Companion computer on the boat** (preferred) if there's a Pi/Jetson
  connected via USB to the CubeOrange+. Zero radio hops.
- **Ground station laptop** if only the RFD900x base radio is available.
  Add one RF hop vs. direct, otherwise identical.

Whichever machine runs the bridge must also run Tailscale and be reachable
from Jesse's browser.

---

## 2. One-time setup

### 2.1 On the bridge host (boat companion OR ground laptop)

Install Python 3.9+ and clone the repo:

```bash
git clone https://github.com/jeranaias/meridian.git
cd meridian
pip install websockets pyserial numpy scipy
```

Install and start Tailscale. Verify your Tailscale IP (you'll need it later):

```bash
tailscale status
tailscale ip -4
```

### 2.2 On Jesse's laptop (Monterey)

Install Tailscale. Verify you can reach the bridge host:

```bash
tailscale ping <bridge-host>
```

---

## 3. Pre-flight sequence

Run these steps in order. The `preflight-check.py` tool will verify each
step — do not skip it.

### 3.1 Start the MAVLink bridge

**On the companion computer (USB to FC):**

```bash
python tools/mavlink-ws-bridge.py --serial /dev/ttyACM0 --baud 115200
```

**On the ground station with RFD900x USB radio:**

```bash
python tools/mavlink-ws-bridge.py --serial COM4 --baud 57600
```

(Windows: the port is usually `COM4` or `COM5`.  Check Device Manager.
Linux: `ls /dev/tty{ACM,USB}*`)

**If you're already running MAVProxy and don't want to stop it:**

```bash
# In MAVProxy, add an output (if not already):
# output add udp:127.0.0.1:14550

python tools/mavlink-ws-bridge.py --udp 127.0.0.1:14550
```

Expected output:

```
19:54:12 [INFO] Opening serial port /dev/ttyACM0 @ 115200 baud
19:54:12 [INFO] Serial port open
19:54:12 [INFO] WebSocket server: ws://0.0.0.0:5760
19:54:12 [INFO] Tablet URL: mission.html?proto=mavlink&ws=ws://<host>:5760
19:54:17 [INFO] Stats: 5 heartbeats, ↓427B ↑0B, 0 clients
```

If you see `0 heartbeats` after 10 seconds, the FC is not sending MAVLink —
check the serial port, baud rate, and that the CubeOrange+ is powered on.

### 3.2 Start the tablet HTTP server

```bash
cd meridian/gcs
python -m http.server 8080
```

This serves `mission.html` on port 8080. Both the bridge and the HTTP
server can run on the same machine.

### 3.3 Pre-cache terrain for the operating area

Before going to the lake, while you still have internet:

```bash
python tools/terrain.py --lat <LAKE_LAT> --lon <LAKE_LON> --radius 5
```

Example for Lake Del Valle, CA:

```bash
python tools/terrain.py --lat 37.59 --lon -121.72 --radius 5
```

The central California region (~300km across Monterey Bay + Sierra
foothills) is already cached in `data/terrain/` — most lake locations in
the area should be covered without extra downloads.

### 3.4 Run the smoke test

From the bridge host:

```bash
python tools/preflight-check.py \
  --serial /dev/ttyACM0 --baud 115200 \
  --lat <LAKE_LAT> --lon <LAKE_LON> \
  --ws-port 5760 --http-port 8080
```

Expected output (all green = ready to fly):

```
1. Python dependencies
  [OK] websockets — v16.0
  [OK] pyserial — v3.5
  [OK] numpy + scipy — numpy v1.26.4, scipy v1.17.0

2. MAVLink source connection
  [OK] Serial /dev/ttyACM0 @ 115200 — received 4200 bytes in 2s

3. Telemetry health
  [OK] Heartbeats — 4 received
  [OK] GPS fix type — fix type 3 (3D fix)
  [OK] Satellites — 14 satellites
  [OK] Battery voltage — 25.1V

4. Terrain / bathymetry
  [OK] Terrain cache directory — 2 tile(s) cached (102 KB total)
  [OK] Depth at (LAT, LON) — -4.2m (water)

5. WebSocket bridge
  [OK] WebSocket on port 5760 — 5 heartbeats / 2436 bytes in 3s

6. Tablet app HTTP server
  [OK] HTTP on port 8080 — mission.html reachable

Summary: 11 passed / 11 total

READY TO FLY
```

**Red items must be fixed before launching.** The tool will tell you what
to do for each failure.

### 3.5 Open the tablet

From Jesse's laptop, open:

```
http://<BRIDGE_TAILSCALE_IP>:8080/mission.html?proto=mavlink&ws=ws://<BRIDGE_TAILSCALE_IP>:5760
```

Or, open `http://<BRIDGE_TAILSCALE_IP>:8080/mission.html`, click the
gear icon (top right), select **MAVLink**, enter
`ws://<BRIDGE_TAILSCALE_IP>:5760` in the WebSocket URL field, and click
Connect. Settings persist to browser localStorage.

The status bar should show:
- Green connection dot (top-left)
- Mode name (probably `STABILIZE` or `MANUAL`)
- GPS sat count, battery percentage, speed, heading
- The boat marker on the map at its real lat/lon

If any of these are wrong or missing, **stop** and re-run preflight-check.

---

## 4. First mission — dry run (no props)

Before putting the boat in water, verify the command path with the props
removed or the kill switch engaged.

1. **Arm the boat** — tap the ARM button.  Expect:
   - Button turns orange, text changes to `DISARM`
   - Mode indicator changes to `STABILIZE` or whatever mode is active
   - CubeOrange+ should beep or light up per ArduPilot convention
2. **Switch to LOITER** — tap the mode button (not implemented in mission.html
   directly; use QGC or the command is auto-switched by GO)
3. **Tap the map** to set a destination.  Expect:
   - A red pin on the map
   - Distance and ETA displayed in the mission info bar
   - GO button enabled
4. **Tap GO** — expect:
   - Mode changes to `AUTO`
   - Vehicle starts moving toward the waypoint (or motors would spin if
     props were on)
5. **Tap HOME** — expect:
   - Mode changes to `RTL`
6. **Tap STOP** — expect:
   - Immediate disarm

If any of the above doesn't work, check the MAVLink command ACK in the
bridge log (`stats` line shows `↑XXXB` bytes flowing to vehicle).

---

## 5. Wet test — on the water

1. **Physical checks:**
   - Props secure
   - Kill switch wired and tested
   - Battery fully charged (>24V for 6S LiPo)
   - GPS antenna has sky view
   - Radio antenna clear of obstructions

2. **Rerun `preflight-check.py`** with the boat on the water.

3. **Arm → GO to a nearby waypoint (50m)** — short hop to verify
   navigation control loop works on real water.

4. **HOME** — return to launch, verify RTL brings it back.

5. **Progressively longer missions** as confidence builds.

---

## 6. Troubleshooting

### "Bridge connected but 0 heartbeats"

- Check baud rate matches the FC setting (ArduPilot default is 115200 for
  USB, 57600 for RFD900x telemetry ports)
- Verify the FC is powered and in a normal operating state (not in DFU
  mode or bootloader)
- Try swapping the USB cable — some only carry power, not data

### "Tablet says DISCONNECTED even though bridge shows a client"

- Check the WebSocket URL in settings — must be `ws://HOST:PORT`, not
  `http://`
- Check browser console (F12) for CORS or network errors
- Tailscale ACLs — make sure the port is allowed between the two nodes

### "GPS fix never reaches 3D"

- Antenna doesn't have sky view
- GPS module hasn't completed cold start (up to 60s after first power)
- Electromagnetic interference from nearby radios

### "Mission uploads but boat doesn't move"

- Boat not in AUTO mode — the mission.html GO button should set it
  automatically
- Check vehicle type in ArduPilot — must be ArduRover configured for boat
  (FRAME_CLASS=2 for Boat)
- Pre-arm checks failing — check QGC or Mission Planner for the specific
  failure reason

### "Terrain check fails: no data covers this point"

```bash
python tools/terrain.py --lat LAT --lon LON --radius 10
```

This downloads NOAA ETOPO bathymetry for a 10km radius around the point.
Only works with internet access.

### "WebSocket bridge reachable but no MAVLink data flowing"

- The upstream source (serial port, TCP, UDP) is not producing data
- Run with verbose logging: the bridge prints a `Stats` line every 5s
  showing bytes received
- If `↓0B` persistently, the serial/TCP/UDP source is the problem, not
  the bridge

### "Unicode errors on Windows"

If preflight-check.py crashes on Windows with `UnicodeEncodeError`, your
terminal doesn't support ANSI escape sequences. Use Windows Terminal or
PowerShell 7 instead of the legacy cmd.exe.

---

## 7. What this setup gives you

- **Tablet mission control** — touch-optimized interface with ARM/GO/HOME
- **Terrain-aware navigation** — waypoint validation against real NOAA
  bathymetry, prevents dropping pins on land or in shallows
- **Dual protocol support** — works with ArduPilot (MAVLink) today,
  ready for Meridian firmware (MNP) when that's bench-tested
- **Tailscale-native** — no port forwarding, no public IPs, works across
  NAT / firewalls
- **Offline map tiles** — mission.html caches Leaflet tiles via the
  browser Cache API; works without cell/wifi once cached
- **Pre-flight verification** — `preflight-check.py` catches missing deps,
  bad coordinates, no-fix GPS, dead telemetry, broken WebSocket, etc.

---

## 8. Files touched during this setup

| File | Purpose |
|------|---------|
| `tools/mavlink-ws-bridge.py` | MAVLink ↔ WebSocket bridge |
| `tools/preflight-check.py` | Pre-flight verification tool |
| `tools/terrain.py` | NOAA bathymetry downloader |
| `gcs/mission.html` | Tablet mission interface (dual protocol) |
| `data/terrain/*.npz` | Cached bathymetry tiles |

---

## 9. Emergency procedures

**Boat goes into unexpected mode or trajectory:**
- Tap STOP in the tablet (immediate disarm)
- If STOP doesn't respond: switch radio to manual/RC override
- If that fails: wait for failsafe RTL to trigger (typically 3-5s after
  comms loss) — hence why Tailscale reliability matters

**Lost Tailscale connection:**
- Boat should continue current mission (ArduPilot failsafe config
  dependent)
- Switch to RC transmitter manual control
- Disarm via RC once you regain visual contact

**Battery warning:**
- Return immediately via HOME button
- ArduPilot's own low-battery failsafe should also trigger RTL at the
  configured threshold
