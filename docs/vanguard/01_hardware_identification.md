# Vanguard USV — Hardware Identification Checklist

This doc lists candidate models for every piece of hardware we think is on the boat. Jesse confirms each line with Tristan (photo / label / cable type).

## 1. ORCA box — CONFIRMED: Orca Core 2

**Confirmed by Jesse 2026-04-18: it IS an Orca Core 2.** (May have a dark housing / sleeve — doesn't matter, the function is what we needed to nail down.)

What Orca Core 2 actually provides:
- **Built-in single-antenna GPS** (consumer-grade, ~2.5 m accuracy)
- **9-axis IMU** (accel + gyro + mag)
- **NMEA2000 ↔ Ethernet bridge** (its primary role in our architecture)
- Powered from the N2K bus
- Exposes data to LAN via proprietary Orca protocol + optional Signal K

Link: <https://getorca.com/products/core-2>

### Implications for our plan
- **The 2nd "GPS" on the boat is the Orca's internal GPS** — not a moving-baseline peer. So nothing lost going to one primary GPS.
- We read Orca over Ethernet (N2K data bridged to IP). Protocol to confirm — Signal K (port 8375) or proprietary.
- We do **NOT** write to Orca (read-only consumer).

---

## 1a. (Historical) Black box candidates — SUPERSEDED BY ABOVE
(Kept for reference in case other black boxes appear in the system later.)

Narrowed candidates for palm-sized, black, marine-capable, Ethernet-in / NMEA-out:

| Candidate | Form factor | Fit | Link |
|---|---|---|---|
| **Orca AI SeaPod / compute module** | Rugged black box, AI watchkeeping (camera-based collision avoidance). Not traditionally bathymetric — if he said "bathymetry" he may have meant "knowing what's around us." | Likely | <https://www.orca-ai.io/seapod/> |
| **Cerulean Sounder S500 companion computer** | Black aluminum, 80×55 mm. Paired with S500 echosounder for USV bathy | Possible | <https://ceruleansonar.com/product/sounder-s500/> |
| **Signal K Node server on Raspberry Pi 4 in black case** | Custom build. Palm-sized. Ethernet + USB-CAN hat. Software: Signal K server + CANboat. | **Very possible** — common DIY marine platform | <https://signalk.org/> |
| **Actisense W2K-1** | Dark gray/black industrial, N2K + 2×Ethernet | Possible | <https://www.actisense.com/product/w2k-1/> |
| **Yacht Devices YDEN-02** | Black plastic, N2K + RJ45 | Possible | <https://www.yachtd.com/products/ethernet_gateway.html> |
| **Custom ARM/Intel SBC in a black enclosure** | Lots of USV builders roll their own | Possible | — |

### What to ask Tristan (to pin it down in one message)
1. "Can you send me a photo of the front and back of the ORCA box?" — most efficient
2. If no photo: "Any brand name or logo on it? Any labels like 'Cerulean', 'Actisense', 'Yacht Devices', 'Orca AI', 'Signal K'?"
3. "What does it actually output — is it doing depth soundings, or AI camera/watchkeeping, or just N2K data bridging?"
4. "Does it have any sensors attached (transducer on a cable, camera lens visible)?"

### What Jesse said it does
"**Ethernet in, NMEA out** — they are using ORCA for bathymetry."

That phrasing most cleanly fits a **bathymetry-focused embedded computer** (Signal K box + echosounder, or Cerulean companion computer). **If it's Orca AI SeaPod, it's doing watchkeeping, NOT bathymetry** — in which case the bathymetry is coming from a separate N2K depth sounder.

---

## 2. GPS receivers

**Per Jesse: both GPSes were already on the boat when Tristan got it.** That matters — it means they're whatever the *original builder* of the USV installed, not Tristan's deliberate moving-baseline design choice. If the builder picked 2 GPSes for "redundancy" without planning moving-baseline heading, **the 2nd one is mostly dead weight** and Jesse's 1-GPS thesis is strong.

Candidates, ranked by likelihood for an already-built USV:

| Model | Looks like | RTK? | Moving-baseline? | Notes |
|---|---|---|---|---|
| **u-blox ZED-F9P** on an ArduSimple **simpleRTK2B** | Small red PCB ~50×70mm, USB-C + UART + antenna SMA | Yes | Yes (with 2 of them) | Hobbyist/research gold standard. $200/unit. |
| **u-blox ZED-F9P** on a **Holybro H-RTK F9P** | Common puck + PCB from the open-autopilot ecosystem | Yes | Yes | Sold as matched pair for moving-baseline yaw |
| **CUAV C-RTK 9Ps** | White puck, marked CUAV | Yes | Yes | Designed for dual-antenna yaw, DroneCAN |
| **Here4 / Here3+** | White pucks from ProfiCNC/Hex | Yes | Yes | DroneCAN native |
| **Emlid Reach M+ / M2** | Blue PCB ~50×70mm with dual SMA | M+: L1 only. M2: L1+L2 | With two | Common in survey applications |
| **u-blox NEO-M8N** (legacy) | Small PCB with ceramic patch antenna | No | No | Consumer, 2.5m accuracy. If this is what he has, **single one is plenty** |
| **Garmin / Navico / consumer** GPS | Marine-looking puck with N2K Micro-C | No | No | Just a position source on the N2K bus |

### What to ask Tristan
- "Can you read the label on the GPS modules? Are they the **same model**?"
- "Where are they mounted relative to each other? (distance in cm, axis)"
- "Does each GPS have its own antenna, or are they single-antenna units?"
- "Are they wired to the autopilot (Pixhawk/etc.) or to the NMEA2000 bus?"

If he has two **ZED-F9P based** units mounted 30+ cm apart front-to-back — that's a moving-baseline setup and worth keeping both.
If he has two **identical consumer NEO-M8N pucks** sitting next to each other — the 2nd one is redundant, drop it.

Links:
- ArduSimple simpleRTK2B: <https://www.ardusimple.com/product/simplertk2b/>
- u-blox ZED-F9P: <https://www.u-blox.com/en/product/zed-f9p-module>
- Holybro H-RTK F9P: <https://holybro.com/products/h-rtk-f9p>
- CUAV C-RTK 9Ps: <https://cuav.net/c-rtk-9ps/>

---

## 3. NMEA2000 backbone / gateway

Tristan said "Ethernet in, NMEA out." Possibilities:
- **The Orca Core 2 itself is the gateway** — most likely given it has Ethernet + N2K ports, and he's not mentioned a separate box
- **Yacht Devices YDEN-02** Ethernet gateway (blue PCB in plastic housing, N2K Micro-C + RJ45) — <https://www.yachtd.com/products/ethernet_gateway.html>
- **Actisense W2K-1** (robust industrial gateway, N2K + 2× Ethernet) — <https://www.actisense.com/product/w2k-1-nmea-2000-to-wi-fi-or-ethernet-gateway/>
- **Digital Yacht iKonvert** (N2K to USB, less likely given "Ethernet" mention)

### To confirm
- "Is the NMEA2000 network just the Orca Core, or is there a separate gateway box too?"
- If separate: brand/model on the label

---

## 4. Propulsion / ESC — CONFIRMED: PWM to ESC

Confirmed by Jesse 2026-04-18. The Cube drives the ESC with a standard **PWM signal** (no UART/CAN control).

### Implication
- **No motor telemetry**: we lose RPM, bus voltage, current, and thermal data that a VESC in UART/CAN mode would give us. That's a real limitation for dead-reckoning during GPS dropout.
- Our VESC UART driver (`esc_vesc.rs`) is built and ready, but becomes a **future upgrade path**, not current-phase work.
- For now the Cube's PWM output goes to whatever ESC is installed and Meridian commands throttle via the Cube's servo outputs.

### Still useful to know
- **Which ESC**? (even if PWM-driven, a VESC-based ESC could later be switched to UART mode with a firmware setting. Worth identifying.)
- **VESC Tool was recently run on Tristan's laptop** — suggests the ESC *may* be VESC-based. If so, future UART upgrade is a day's work.
- Is it a single ESC or dual (twin jet pumps)?
- Is there a kill/arm switch in the propulsion loop?

### Future VESC UART upgrade (when we want motor telemetry)
- Requires swapping from PWM input to UART on the VESC's input config
- Single extra UART wire pair from Cube/companion computer to the VESC
- Meridian's `esc_vesc` driver handles the rest — telemetry at 20 Hz, safe keepalive, full fault reporting
- **Payoff**: RPM-based forward-speed estimation during GPS dropout (key to the 1-GPS claim)

---

## 5. Autopilot flight controller — CONFIRMED: CubePilot Cube Plus

Confirmed by Jesse 2026-04-18. CubePilot Cube+ (almost certainly **Cube Orange+**, H7 family — the current flagship).

- MCU: STM32H757 (H7 family, dual-core M7+M4) — same family as Meridian's reference board (Matek H743)
- Carrier board: CubePilot standard carrier
- Current firmware: whatever Tristan has flashed (to be replaced with Meridian firmware)
- Serial ports: Serial1 = RFD900 telemetry, others TBD
- Standard Cube GPS port: serial puck (Here3 or Here4) plugged in

### Why this matters
- **Meridian firmware targets H7 natively.** Board config drafted at `boards/CubeOrangePlus.toml`. Flash Meridian to the Cube and it takes over — same carrier, same actuator wiring, nothing mechanical changes.
- One flash moves the boat from whatever legacy firmware was on it to a modern, marine-native autopilot.

Link: <https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview>

### To confirm with Tristan
- Exact variant: Cube Orange+, Cube Purple, Cube Black, Cube Blue?
- Current firmware (so we know what we're replacing)
- What GPS is wired to the Cube's GPS port? (Here3 / Here4 / other)
- Any other peripherals plugged into the carrier (airspeed / I2C magnetometer / etc.)

---

## 6. Radio / telemetry — CONFIRMED: RFD 900 on Cube Serial1

Confirmed by Jesse 2026-04-18.

- **RFD 900** (probably x or x2 variant, 915 MHz, transparent serial mesh)
- Wired to **Serial1** on the Cube carrier
- Transparent serial at 57 600 baud (radio-side is protocol-agnostic)
- 40+ km range with good antennas

### Implication for Meridian
- **Meridian Native Protocol (MNP)** is explicitly designed to fit the 57 600 baud RFD 900 link budget (COBS-framed postcard, ~11% utilization at 57 600).
- After Meridian is flashed to the Cube, RFD 900 stays on Serial1 unchanged — it's just carrying MNP instead of a legacy wire format.
- MAVLink adapter is available for legacy GCSes that need it, but the primary protocol to the tablet is MNP.

### Still useful to know
- Exact model: RFD 900, RFD 900x, RFD 900+, or RFD 900ux?
- Antenna type on boat side and ground-station side
- Max range they've validated in the field
- Is there also an LTE modem (explains the Telstra CGNAT IP we saw for Tailscale's direct endpoint)?

---

## 7. IMU / magnetometer

If the flight controller is Pixhawk-class, the IMU + mag are **inside the FC**. Still worth asking:
- "Any external compass (mast-mounted)?"
- "Any vibration-isolated IMU mount?"

---

## Photo request list for Tristan

If he can send a few photos:
1. The whole electronics tray / bay (one wide shot)
2. Close-up of each labeled box: GPS modules, Orca, VESC, FC, gateways
3. The two GPS antennas as-mounted on the boat (shows baseline)
4. Any manufacturer nameplate / serial tag / barcode

That'll let me identify everything without asking a ton of specific questions.
