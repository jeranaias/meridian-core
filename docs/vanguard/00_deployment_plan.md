# Vanguard + Meridian — Deployment Plan

**Two phases. Same codebase. Same product identity.** The first deploy gets
Tristan's Vanguard autonomous in a week. The full vision turns Meridian into
the platform every autonomous boat should run.

---

## FIRST DEPLOY — MVP on Tristan's Vanguard

**Goal:** A fully autonomous Vanguard running Meridian end-to-end, with the
Orca Core 2 as its only GPS, by the next field test.

### What ships
| Component | Where it runs |
|---|---|
| **Meridian firmware** | Directly on the Cube Plus (H7) — replaces whatever was on it before |
| **Tablet GCS** (`mission.html`) | Any browser, over Tailscale |
| **Orca bridge daemon** | Small Linux box on the boat OR Tristan's companion laptop |
| **Configuration bundle** | `vanguard_defaults.meridian-params` imported once |

### Success criteria
1. Tablet shows a live HEARTBEAT from the Meridian-powered Cube within 10 s of power-on
2. Orca's GPS fix appears on the tablet within 60 s (10 Hz, < 3 m accuracy)
3. Bench test verifies rudder actuation in both directions via webcam
4. Autotune button produces workable steering PID gains
5. Station-keeping test in water holds position within 2 m for 5 minutes
6. Autonomous waypoint mission completes, boat returns to home

### Work to finish (prioritized)
1. **Flash Meridian to the Cube Plus** — board config (`boards/CubeOrangePlus.toml`) is drafted; need a flashable `.apj`/`.bin` build and a first boot validation
2. **Meridian-native parameter bundle** — one file the user loads via the tablet's PARAM_SET protocol
3. **Orca protocol capture** — tcpdump a real Orca talking on Ethernet, confirm framing (Yacht Devices RAW vs. Orca native vs. Signal K), wire into our ingest
4. **Orca bridge → Meridian GPS input** — Meridian's native GPS ingest (not the MAVLink stopgap), fed directly by the bridge
5. **Field validation** — compass cal on-site, bench test, dock test, waypoint test

### Timeline estimate
Working from current state: ~1-2 weeks of focused work to hit success criteria.
Bottleneck is the Meridian-firmware-on-Cube flash cycle + the Orca protocol
capture; everything else is code we've written.

---

## FULL VISION — Meridian as the autonomous marine platform

**Goal:** Meridian becomes the default autopilot for any autonomous or
semi-autonomous boat, replacing legacy autopilots and proprietary stacks.

### Product surfaces

1. **Autopilot firmware** — Meridian native on STM32H7-class FCs. Today supports
   Matek H743 and Cube Plus; extend to Pixhawk 6X, Holybro H7 line, CUAV
   V6X, Kakute H7 as demand pulls. One codebase, many boards.
2. **Tablet GCS** — browser-based, responsive, offline-capable tile cache,
   demo tour built in. Runs on iPad, Surface, Android tablet, any laptop.
3. **Orca / marine gateway ingest** — sensor fusion from any N2K-over-IP
   source. Signal K, Yacht Devices RAW, Orca native, Actisense — all
   autodetected and decoded.
4. **Meridian Native Protocol (MNP)** — the wire format between boat and
   GCS. COBS-framed, fits 57600 baud RFD 900x links, 10 Hz rich telemetry.
5. **Companion daemon (`meridian-orca-bridge`)** — when the boat has a
   marine data gateway, the bridge runs on the onboard computer and
   forwards sensor streams into Meridian's EKF.
6. **Cloud console** *(future)* — fleet view, mission planning, log
   archival, over-the-air config sync across tailnet.

### Differentiation
Laid out in `04_competitive_analysis.md`:
- **Zero dedicated GPS hardware** (Orca-only architecture)
- **10 Hz GPS passthrough**, faster than most dual-GPS setups
- **4-layer safety** (primary GPS + mag+IMU+COG fusion + VESC RPM dead-reckoning + bathymetric map-matching)
- **Open autopilot** — customers aren't locked into our stack the way
  SeaRobotics / Saildrone / MR customers are
- **Single tablet, real-time COLREG-aware threat avoidance** proven in SITL

### Target markets (order of priority)
1. **Defense / government C-UAS maritime nodes** — Vanguard-class USVs
   doing autonomous patrol, port security, interdiction support
2. **Hydrographic survey** — small USV operators needing clean bathy data
   + autonomous lawn-mower patterns
3. **Research / academia** — replacing legacy open-source autopilot
   stacks on BlueBoat / WAM-V / Heron style platforms with something
   modern and marine-native
4. **Commercial fisheries / aquaculture** — sensor-tending, cage inspection
5. **Maritime autonomy startups** — OEM/reference platform

### Roadmap buckets

| Bucket | Examples | Depends on |
|---|---|---|
| **Core autopilot polish** | VTOL-class surface transitions, high-speed planing hull support, AUV dive profiles | First deploy working |
| **Sensor ecosystem** | Sidescan sonar integration, thermal camera perception, radar overlay, LiDAR | Tablet layers already built |
| **Fleet / multi-vessel** | Formation keeping, leader-follower, distributed task allocation | MNP multi-vehicle already in |
| **Autonomy features** | Waypoint missions with adaptive replanning, auto-dock, anchor-drag detection, lost-link auto-return | Station-keep + COLREG already in |
| **Cloud & ops** | Mission library, log archival, tailnet-based fleet oversight | Onboard telemetry already streams |
| **Safety & certification** | IEC 61508 functional safety evidence, ISO 17894 maritime software path | Deterministic core, logging already built |
| **Commercial ops** | Licensing, support contracts, integrator certification | Product maturity from first deploys |

### North-star metrics
- **Time from box-opening to first autonomous waypoint mission**: target < 2 hours
- **Cost of nav hardware per vessel**: target $0 incremental over existing marine electronics
- **Safety incidents per 1,000 autonomous mission hours**: target < 1
- **Active deployments at 12 months post-launch**: target 50+

---

## The transition

**First deploy proves the architecture.** We land Meridian on Tristan's
Vanguard with the Orca-only GPS story working in water, and the tablet
GCS driving it all. That one win opens the credibility needed to talk to
defense, survey, and academic buyers.

**Full vision builds on that substrate.** Every feature in the roadmap
buckets composes onto the same codebase. We don't throw anything away
between phases — we layer on.

---

## Dependencies

What we need to keep validating as we move from first to full:

1. **Meridian firmware stability on the Cube Plus** across the vehicle's full
   operating envelope (docked, at speed, in chop, in rain, in cold)
2. **Orca bridge robustness** — handle packet loss, reconnection, mixed
   protocol hints, partial decoding errors
3. **Tablet GCS performance** across devices and network conditions
   (cellular, mesh, high-latency links)
4. **Parameter evolution** — when we change defaults, existing deployments
   need a migration path

Each first-deploy customer's field log feeds back into the platform.
Full vision is what the 10th customer gets.
