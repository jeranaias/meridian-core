# Vanguard + Meridian — Competitive Analysis

## Summary
Every commercial autonomous USV platform ships with dedicated GPS
hardware — either by design (dual-GPS moving-baseline) or because their
autopilot assumes a direct-to-FC sensor chain. Meridian eliminates that
layer by treating the marine-grade nav gateway already on the boat as
the authoritative sensor.

---

## Competitor map

| Platform | Price (approx.) | GPS config | N2K integration | Autopilot openness | Key differentiator | Our edge |
|---|---|---|---|---|---|---|
| **BlueRobotics BlueBoat** | $8,500 (kit) | 1× u-blox F9P RTK (optional) | None (not marketed) | Open (runs generic USV stacks) | Open & affordable DIY | Meridian integrates directly with any N2K source they add; our EKF is tuned for marine single-GPS |
| **SeaRobotics USV-2600** | $100K+ | Dual RTK GPS | Proprietary | Closed SeaHive stack | Turnkey commercial | Faster integration, 1/10th the install cost, modern tablet UX, no vendor lock |
| **Saildrone Explorer** | Service-only (not sold) | Dual RTK + celestial | Proprietary | Closed | Ocean-scale autonomy | Not comparable — different market (multi-month ocean missions) |
| **Ocean Infinity Armada** | Service-only | Dual GNSS INS (mil-grade) | Proprietary | Closed | Ex-military grade | Different market (offshore survey fleets) |
| **Clearpath Heron** | $55K (base) | 1× u-blox M8N / F9P | None | ROS-based, research-focused | Research robotics ecosystem | Productionized stack without ROS dependency; ships to customers, not labs |
| **Maritime Robotics Mariner** | $300K+ | Dual RTK + fiber-optic gyro | Proprietary | Closed | Norwegian survey market | Different price bracket / market |
| **WAM-V (Marine Advanced Robotics)** | $150K+ | Dual F9P (typical) | None built-in | Customer-supplied | Twin-hull survey platform | Meridian runs their hull too; we don't force dual GPS |
| **Generic "Pixhawk on a boat" DIY** | $500-2,000 | 1 or 2× Here3/Here4 | None | Open legacy firmware | Cheap & customizable | Orca-only path cuts hardware cost further, adds marine sensor integration, upgrades them to modern tablet UX |

---

## Axes of differentiation

### 1. Install cost
Most competitors require 4-8 hours of antenna mounting, cable routing,
moving-baseline calibration, and magnetometer setup. Vanguard +
Meridian requires **30 minutes** — one Ethernet cable and a
parameter-bundle import.

### 2. Hardware bill of materials
Every competitor ships GPS antennas and receivers as part of their
required kit. Our path **uses the Orca Core 2 the boat often already
has** for chartplotter duty. If the customer doesn't have one, a single
$500 Orca unit replaces $800-$1,500 of dual-GPS hardware *plus* adds
chartplotter functionality — it pays for itself.

### 3. GPS data rate
Most competitors: 5 Hz (consumer) or 1-10 Hz (RTK). Vanguard's
Orca-only path: **10 Hz passthrough**. Tighter station-keeping, tighter
waypoint tracking, better UX feel.

### 4. Compass-failure tolerance
Magnetometers fail or drift on boats (steel hulls, motors, VHF, cables).
Dual-GPS moving-baseline competitors mitigate this with GPS yaw;
Meridian mitigates it with **four independent heading sources blended
by the EKF**: Cube mag + Orca 9-axis IMU + GPS COG (> 0.8 m/s) + gyro
drift estimate.

### 5. Autopilot openness
- Closed-stack competitors (SeaRobotics, Saildrone, MR Mariner, Ocean
  Infinity) lock customers into service contracts.
- Open competitors (BlueBoat, WAM-V, Clearpath) run older autopilot
  firmware often years behind feature-wise.
- **Meridian is a modern, open, marine-native autopilot** — Rust,
  no-std firmware, auditable telemetry, and a tablet UI designed for
  actual on-water operations.

### 6. Bathymetric positioning backup
No competitor ships bathymetric map-matching as a tertiary position
source. This matters for GPS-denied operations (under piers, near tall
structures, deliberate GNSS jamming).

### 7. Price point
Not the cheapest hull (BlueBoat wins on raw kit price), but the
cheapest **field-deployed integrated autonomous system**. BlueBoat is a
hull kit; Meridian is a hull-agnostic full-stack autopilot + tablet GCS
+ pre-configured marine sensor integration.

### 8. Marine-native UX
The tablet ships with AIS traffic overlay, COLREG-aware threat
avoidance, perception contact rendering, bench-test mode with live
rudder chart, and a compass calibration wizard. **Built for boats**,
not ported from a drone codebase.

---

## Where we don't win (and don't try to)

- **Ocean-scale autonomy** (Saildrone's market): need satellite C2,
  wing propulsion, solar. Not our game.
- **Survey-grade RTK precision** (Maritime Robotics, EMILY): we can
  support external RTK, but our Orca-only story is about install
  simplicity, not cm-accuracy.
- **Military-grade INS** (Ocean Infinity): fiber-optic gyros cost more
  than entire Vanguard builds.

---

## Positioning statements

### For technical buyers (defense, research, commercial survey ops)
> "Meridian-on-Vanguard is the autopilot stack that respects the marine
> sensors you already own. One Ethernet cable into your existing Orca,
> Signal K, or Actisense gateway, and your USV is autonomous — with
> four-layer GPS redundancy and a tablet GCS that actually works on
> water."

### For procurement / decision-makers
> "Every hour saved on installation is an hour your vessel is working.
> Vanguard cuts nav-system install from half a day to half an hour —
> and the boat is safer for it."

### For research / integrator community
> "Modern Rust autopilot, MIT-licensed tablet GCS, auditable sensor
> fusion. Your sensors, our fusion, full telemetry. Runs on hardware
> you already own."

---

## Head-to-head sales sheet (Saildrone-class prospect)

| Feature | Saildrone Explorer | Vanguard + Meridian |
|---|---|---|
| Autonomy | Mission-level (weeks) | Mission-level (hours-days) |
| GPS hardware | Dual RTK + INS | **Orca Core 2 only** |
| Install time | Factory only | **30 min customer install** |
| Open source | No | **Yes (AGPL / MIT)** |
| Field-repairable | No | **Yes** |
| Tablet GCS | iPad-based | **Browser-based, any device** |
| Commercial model | Service lease | **Purchase** |

---

## What we need to prove in the field

To earn the "proven" label next to these differentiators:

1. **10 Hz GPS passthrough → Meridian EKF round-trip** measured on live hardware (bench test)
2. **Failover timing** when Orca drops: target < 100 ms switchover to serial puck
3. **5-minute GPS-blackout dead-reckoning drift** with VESC telemetry (target < 30 m error)
4. **Station-keeping accuracy** with only the Orca GPS (target < 2 m hold radius)
5. **Install time to first autonomous mission** on a customer boat (target 30 min)

Once we have these numbers, this doc becomes the sales sheet.
