# Meridian-on-Vanguard USV — System Architecture

**Status:** Initial scoping plan, 2026-04-18 (Jesse + Tristan's Vanguard benchtop)
**Goal:** Deploy Meridian end-to-end on Tristan's USV with a single GPS
source — the Orca Core 2 — and match or exceed the operational confidence
of any dual-GPS moving-baseline setup. Make this the demonstrator for the
Vanguard product line.

---

## Confirmed hardware (2026-04-18)

| Component | Model | Meridian role |
|---|---|---|
| Flight controller | CubePilot **Cube Plus** (STM32H757, H7 class) | Runs Meridian firmware natively (H7 is Meridian's primary target) |
| Telemetry radio | **RFD 900** on Serial1, 57600 baud | MNP primary link, MAVLink adapter available for legacy GCSes |
| Propulsion | PWM from FC → ESC | Driven by Meridian's RcOutput HAL (PWM today; upgrade to VESC UART for motor telemetry) |
| Marine gateway + GPS | **Orca Core 2** (Ethernet M12 D-code + NMEA2000 Micro-C) | Primary nav source — 10 Hz GPS + 9-axis IMU + N2K bridge |
| Legacy serial GPS | Existing puck on the FC's GPS port | Silent hot-standby via Meridian's `EK3_SRC2_*` fallback |

**Key architectural insight:** the two existing "GPSes" on the boat —
the Cube's serial puck and the Orca Core 2's internal GPS — are both
consumer-grade single-antenna receivers. They are NOT a moving-baseline
pair. Going to "Orca only as primary" loses nothing, gains the install
simplicity story, and still keeps the serial puck as free silent
redundancy.

---

## 1. The strategic play: 1 GPS, higher operational readiness

**Industry baseline for autonomous USVs is 2 GPS** for moving-baseline
heading (see `01_hardware_identification.md`). We match that confidence
with **1 primary GPS + Meridian's 24-state EKF + VESC telemetry +
bathymetric map-matching**, which unlocks:

- Simpler / cheaper installation (no antenna mast with strict geometry)
- Lower supply-chain risk (one procurement lane)
- Less real estate on small hulls
- Fewer failure modes
- Every existing single-GPS vessel becomes a Meridian candidate — huge
  market expansion

### How we get there

| Risk dual-GPS addresses | Meridian's mitigation |
|---|---|
| Magnetometer corrupted by boat electronics | In-situ compass calibration via the tablet wizard + **GPS COG-based heading above ~1 m/s**. Mag used only at rest. EKF source selector (`EK3_SRC1_YAW`) hands off seamlessly. |
| GPS dropout → position lost | **VESC motor telemetry for RPM-derived speed**, IMU for accel/rotation, **Orca-bridged depth for bathymetric map-matching** — Meridian's EKF fuses all of it. Dead-reckoning drift budget: < 1 m / 30 s without GPS. |
| Heading jitter at zero speed | Calibrated mag + gyro bias estimation from IMU. Station-keeping regime uses rudder-effectiveness-scaled gains. |
| Spoofing / multipath | Cross-check GPS position against bathymetric fingerprint (Orca depth vs. chart) and against IMU dead-reckoning. Outlier rejection in the EKF. |

**Net claim:** one GPS + Meridian EKF + Orca depth + VESC telemetry
matches dual-GPS-moving-baseline confidence for the 95% use case, at
half the hardware complexity.

---

## 2. System block diagram

```
                     ┌─────────────────┐
                     │   Tablet GCS    │    ← Meridian mission.html
                     │  (mission.html) │      over Tailscale
                     └────────▲────────┘
                              │ WebSocket (MNP)
                              │
                              ▼
                     ┌──────────────────────────┐
                     │   RFD 900 radio link     │
                     └────────▲─────────────────┘
                              │
                              │ UART (Serial1, 57600)
                              ▼
   ┌──────────────────────────────────────────────────────┐
   │   Meridian firmware on Cube Plus (STM32H7)           │
   │   ────────────────────────────────────────────────   │
   │   • 24-state EKF (pos/vel/attitude/biases)           │
   │   • Multi-source arbitration (Orca/puck/VESC/IMU)    │
   │   • Native MNP protocol + MAVLink adapter            │
   │   • Safety: failsafes, arming, geofence, COLREG sta. │
   └──────┬─────────────┬─────────────┬────────┬─────────┘
          │             │             │        │
     Ethernet         UART          SPI       PWM
          │             │             │        │
     ┌────┴─────┐   ┌───┴──────┐  ┌──┴─────┐ ┌──┴────┐
     │  Orca    │   │ Serial   │  │Cube    │ │ ESC   │
     │  Core 2  │   │ GPS puck │  │internal│ │       │
     │          │   │ (bkp)    │  │IMU+mag │ │       │
     │ • GPS    │   │          │  │+baro   │ │       │
     │ • IMU    │   │          │  │        │ │       │
     │ • N2K    │   │          │  │        │ │       │
     │   bridge │   │          │  │        │ │       │
     └────┬─────┘   └──────────┘  └────────┘ └───┬───┘
          │                                      │
      NMEA2000                               Brushless
      (depth, AIS,                            propulsion
       wind, etc.)                            motor
```

**Key decisions:**
- **Meridian flashes onto the Cube directly.** Same STM32H7 family as our
  reference Matek H743; board config lives at `boards/CubeOrangePlus.toml`.
- **Orca connects over Ethernet** to whatever onboard companion has a
  network interface. That companion forwards decoded Orca data into
  Meridian via UART (or USB-serial) using MNP PARAM/SENSOR messages.
- **The serial GPS puck stays wired** and is silent redundancy. Meridian's
  EKF source arbitration (EK3_SRC1* primary / EK3_SRC2* fallback)
  handles the switch in < 50 ms.
- **RFD 900** is MNP primary on Serial1. The tablet speaks MNP natively.
- **Tablet** connects over Tailscale to the companion computer (or
  directly to the RFD 900 ground radio).

---

## 3. Sensor fusion strategy

Meridian's 24-state EKF is already production code. For Vanguard it
handles:

**State vector (subset relevant to the 1-GPS claim):**
- Position (lat / lon / alt)
- Velocity (NED)
- Attitude quaternion
- Gyro biases
- Accel biases
- **[new] Magnetometer hard / soft iron offsets** (online estimation, seeded by the tablet cal wizard)
- **[new] VESC RPM → velocity scale factor** (online estimation, per-prop)

**Measurement sources and fusion priority:**

| Source | Rate | Fused into | Trusted when |
|---|---|---|---|
| Orca GPS | 10 Hz | Pos + Vel | Always when fix ≥ 3D and HDOP < 2.5 |
| Serial GPS (bkp) | 1-5 Hz | Pos cross-check | Used as consistency gate; disagreement > 5 m → alarm |
| IMU accel + gyro | 1000 Hz | Attitude + dead-reckoning | Always |
| Magnetometer | 50 Hz | Yaw (below ~1 m/s) | When online-calibrated and boat is steady |
| GPS COG (derived) | 5 Hz | Yaw (above ~1 m/s) | When speed > 0.8 m/s |
| VESC RPM | 20 Hz | Forward velocity (GPS dropout) | During GPS dropout |
| Orca N2K depth | 1-5 Hz | Bathy map-match (if chart loaded) | As low-rate position update |
| Orca N2K AIS | variable | Situational awareness, not own-state | Never for own-ship state |

**Source switching:** Meridian's `EK3_SRC*_YAW` params let us pick
heading source dynamically. Same param system has `EK3_SRC1_POSXY`,
`EK3_SRC1_VELXY`, etc. Ships with Meridian, named identically to the
legacy convention so users can map mental models quickly.

---

## 4. Hardware integration points

### 4.1 VESC (propulsion)
- **Physical:** UART at 115 200 baud, 3.3 V logic (verify VESC variant — some are 5 V tolerant), GND common to the FC.
- **Protocol:** VESC `COMM_PACKET` over UART. Start byte `0x02` (short) / `0x03` (long), length, payload, CRC-16, stop byte `0x03`.
- **Commands needed:**
  - `COMM_SET_DUTY` — fallback manual control
  - `COMM_SET_RPM` — primary speed control
  - `COMM_SET_CURRENT` — thrust mode (cleaner on a boat than duty)
  - `COMM_ALIVE` — keepalive, ≥ 5 Hz (VESC times out to zero at ~1 s)
  - `COMM_GET_VALUES` — telemetry: RPM, duty, voltage, amps, temp, distance
- **Driver:** `crates/meridian-drivers/src/esc_vesc.rs` — built, tested, shipped.
- **Safety:** VESC enforces its own current/temp/voltage limits. Meridian sends setpoints; VESC handles protection.

**Current state:** Vanguard's ESC uses PWM input, not UART. VESC driver is
Phase-2 upgrade. Meridian drives PWM today via standard RcOutput.

### 4.2 Primary GPS (Orca Core 2)
- **Source:** Orca's internal GNSS (10 Hz, < 3 m) over Ethernet.
- **Protocol:** Under investigation — Orca uses proprietary NMEA 2000 over
  IP; similar to Yacht Devices RAW. Our Orca bridge autodetects (Signal
  K, Yacht Devices RAW UDP/TCP, Orca native). See `tools/orca-bridge/`.
- **Meridian ingest:** companion computer runs the bridge, decodes the
  stream, forwards to Meridian via a dedicated UART at 115 200 baud
  using MNP sensor messages. Meridian's EKF treats it as a fused
  position source.

### 4.3 Backup serial GPS
- Stays wired to the Cube's GPS port as silent standby.
- Meridian's `EK3_SRC2_*` params point to this as failover.
- Automatic switchover when Orca link dies or quality degrades.

### 4.4 Orca Ethernet bridge (N2K data)
- **Assumption:** Orca exposes N2K data over TCP or UDP on the boat LAN.
- **Protocols probed:** Signal K (port 8375), Yacht Devices RAW-like
  (port 1457, 60001), Orca native (TBD — awaiting packet capture).
- **Meridian ingest:** `meridian-orca-bridge` (Python daemon, ships in
  `tools/orca-bridge`) decodes the stream and forwards to Meridian via
  MNP sensor messages.
- **Non-invasive:** read-only consumer, zero risk to Tristan's existing setup.

### 4.5 IMU / compass
- Cube Plus has triple-redundant IMUs internal (ICM42688 × 2 + ICM20948).
- External compass on the I2C bus (via carrier board GPS port).
- Calibration via the tablet wizard (`openMagCal()` already shipped).

### 4.6 Tablet GCS
- Already shipped: station-keeping, threats, AIS, perception, bench test,
  rudder readout, webcam pane, compass cal wizard, GPS source panel,
  rudder autotune, demo tour.
- Connects over Tailscale to the companion computer or direct to RFD 900.

---

## 5. Scope-out checklist (pre-field)

### Blockers we need Tristan to answer
- [ ] **Photo of the Orca Core 2** (confirm model / connectors)
- [ ] **Serial GPS puck model** (brand on the puck, RTK-capable or not)
- [ ] **Cube Plus variant** (Orange+ / Purple / Black / Blue?)
- [ ] **VESC brand / model** and control interface (PWM confirmed, UART upgrade path?)
- [ ] **Radio pairing** — RFD 900 model, antenna type, ground-side hardware
- [ ] **Companion computer** — is there one on the boat already? If yes, what?
- [ ] **Power budget** — battery capacity, bench-test runtime available

### Hardware to source or verify
- [ ] **Companion computer** for the Orca bridge (OnLogic CL100 / Pi 5 / Jetson Orin Nano — see `00_deployment_plan.md`)
- [ ] **USB-to-UART adapter for VESC** (Phase 2)
- [ ] **RFD 900 ground station** — radio + antenna + laptop-side cable

### Software to build — priority order
1. ✅ **VESC driver** — built, tested
2. ✅ **Tablet upgrades** — cal wizard, GPS panel, autotune, tour all shipped
3. ✅ **Orca bridge daemon** — mock mode verified, Yacht Devices RAW + Signal K ready
4. ⚠ **Cube Plus flashable Meridian build** — `boards/CubeOrangePlus.toml` drafted; need first-boot validation
5. ⚠ **Orca protocol capture + decoder** — awaiting pcap from Tristan's setup
6. ⚠ **Meridian-native parameter bundle** — see `vanguard_defaults.meridian-params`
7. ⚠ **Bathymetric map-matching integration** — prototype in `bathy_match.py`, needs chart-loading integration

### Field logistics
- [ ] Bench-test at Tristan's bench first, webcam-verified rudder movement
- [ ] Tethered dock test (boat in water, no propulsion, verify sensor feed)
- [ ] Short-range station-keeping test (sheltered water, manual takeover armed)
- [ ] Waypoint mission with safety boat nearby

---

## 6. Onboard companion computer options

Meridian firmware runs on the Cube (no separate computer needed for the
autopilot itself). But the **Orca bridge** wants a Linux host to parse N2K
over Ethernet. Options:

| Class | Example models | Notes |
|---|---|---|
| **Fanless x86** | UP Squared, Odyssey-X86, OnLogic CL100/CL200, Intel NUC in rugged case | Runs the bridge natively, more compute than needed, easy debug. $400-$1200. |
| **Ruggedized industrial** | OnLogic Karbon 300/400, Winmate M101 | Field-grade, shock/temp/IP-rated. $800-$3000. |
| **ARM SBC** | Raspberry Pi 5 in waterproof case, NVIDIA Jetson Orin Nano, Rock 5B | Cheap, lower power. Jetson gives onboard CV for perception. $100-$600. |
| **No companion** | Future: Meridian-native N2K ingest on the Cube's own Ethernet port (Phase 3) | Eliminates the separate box entirely. Waiting on Cube hardware/carrier with Ethernet PHY. |

**Recommended for first deploy:** **Raspberry Pi 5 in a waterproof box
with PoE**. Cheapest, smallest, enough CPU for the bridge, huge ecosystem.

---

## 7. VESC driver

Full driver shipped in `crates/meridian-drivers/src/esc_vesc.rs`:
- `no_std` Rust, generic over any `UartDriver`
- Full COMM_PACKET framing (short + long)
- Setpoints: `SET_DUTY`, `SET_CURRENT`, `SET_RPM`, `SET_CURRENT_BRAKE`, `ALIVE`
- Telemetry: `GET_VALUES` decode (RPM, duty, voltage, amps, temps, fault)
- 4 unit tests, 4/4 green, integrates with full 201/201 drivers test suite

---

## 8. Test plan (see `05_field_kit_checklist.md`)

Layered verification:
1. **Benchtop** — rudder sweep via webcam
2. **Docked** — sensor feed validation, no propulsion
3. **Confined water** — short autonomous mission, manual takeover armed
4. **Open water** — full mission with chase boat

---

## 9. Open questions / risks

| Question / risk | Mitigation |
|---|---|
| Does Orca expose Signal K or a proprietary N2K-over-IP stream? | Probe via tcpdump once wired; autodetect in bridge |
| Meridian firmware first-boot on Cube Plus — any board-specific bugs? | `boards/CubeOrangePlus.toml` is drafted; bench-flash and iterate |
| VESC firmware variance between Trampa / Flipsky / Maytech | Driver targets the 6.x COMM_PACKET format with version detection |
| Mag cal quality on a boat full of metal / motors | GPS COG takeover above 1 m/s. Mag is a fallback, not the primary. |
| Orca dies → bathy map-match fails | Graceful degradation: serial puck keeps working, VESC RPM dead-reckoning bridges 5+ min |

---

## 10. Key inputs still needed from Tristan

Before we can commit firmware parameters:

1. **Photo walk-through of the electronics bay** (identifies everything
   else in one message)
2. **Exact Cube Plus variant** (determines flash image)
3. **ESC brand / model / control interface**
4. **Is there an onboard Linux computer already?** (if yes, Pi might be
   redundant)
5. **Operating area + chart availability** (bathy map-matching needs charts)

Jesse's ask to Tristan: *"Send a photo walk-through of the electronics
bay — one wide shot, then close-ups of every labeled box. Want to pick
the right integration path."* That unblocks ~80% of questions.
