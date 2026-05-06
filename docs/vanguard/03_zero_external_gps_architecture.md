# Zero External GPS — The Vanguard Architecture

> **Every autonomous surface vessel on the market ships with at least one
> dedicated GPS puck bolted to the deck. We don't. This is how.**

---

## 1. The product claim

**One Ethernet cable. Zero deck antennas. Uncompromised safety.**

The Vanguard USV runs Meridian using the **Orca Core 2** — an NMEA 2000
gateway that already lives below decks — as its primary navigation
source. No dedicated GPS pucks. No moving-baseline antenna mast.
No dual-antenna yaw bar. Just the sensor the boat already has.

This is the **largest installation-cost reduction in the autonomous
marine market** — and the safety story is *better*, not worse, than the
two-GPS norm it replaces.

---

## 2. Why the industry defaults to two GPSes

Every production autonomous USV — BlueRobotics BoatBox, SeaRobotics
USV-2600, Saildrone, Clearpath Heron — ships with **two GPS receivers**.
Two reasons:

1. **Moving-baseline heading.** Two antennas separated by ≥ 30 cm let the
   receiver compute boat heading directly from GPS carrier phase —
   eliminating the magnetometer, which is notoriously unreliable on
   boats (steel hulls, electric motors, battery cables, VHF radios all
   corrupt magnetic readings).
2. **Redundancy.** One GPS is a single point of failure in a vehicle
   that can drift into rocks or shipping lanes if it loses position.

Both concerns are legitimate. Both are solvable without a second GPS.

---

## 3. Our architecture

```
               ┌──────────────────────────────┐
               │       Orca Core 2            │
               │  (already on every boat w/   │
               │   marine instruments)        │
               │                              │
               │  • 10 Hz consumer GNSS       │
               │  • 9-axis IMU + fluxgate     │
               │  • N2K ↔ Ethernet bridge     │
               └──┬──────────────────────┬────┘
                  │ N2K Micro-C           │ M12 Ethernet
                  ▼                        ▼
         ┌─────────────────┐      ┌────────────────────────┐
         │ Boat's existing │      │ Companion computer     │
         │ N2K sensors     │      │ (small fanless Linux)  │
         │ (depth, wind,   │      │                        │
         │  AIS, etc.)     │      │ meridian-orca-bridge   │
         └─────────────────┘      │   ── decodes N2K ──▶   │
                                  │   ── MNP sensor ──▶    │
                                  └──────────┬─────────────┘
                                             │ UART / USB
                                             ▼
                                  ┌────────────────────────┐
                                  │   Meridian on the FC   │
                                  │   (Cube Plus, H7)      │
                                  │                        │
                                  │ Source arbitration:    │
                                  │   EK3_SRC1_* = Orca    │
                                  │   EK3_SRC2_* = serial  │
                                  │                puck    │
                                  │   (free silent backup) │
                                  └──────────┬─────────────┘
                                             │ PWM
                                             ▼
                                         Propulsion ESC
```

**The only "new" cable**: Ethernet from the Orca to the companion
computer. Everything else is hardware that already lives on the boat.

---

## 4. The four-layer safety story

If the Orca fails — power, firmware crash, cable fault — what happens?
In order, from milliseconds to tens of minutes:

### Layer 1 — Meridian's dual-source EKF failover (< 50 ms)

Meridian's EKF supports two concurrent position sources via
`EK3_SRC1_*` (primary) and `EK3_SRC2_*` (fallback). Source arbitration
happens inside the EKF every estimation step. When the Orca feed goes
stale or its quality metric drops, the serial GPS puck takes over
**within a single EKF cycle** — zero pilot action required.

**The kicker:** most customers already have a serial GPS wired to their
Cube. This failover is **free** — we just flip the priority so the Orca
is primary. No new hardware, no install work.

For customers without a serial GPS, the companion computer can carry a
$10 u-blox NEO-M8N internally as a stealth backup, or they can operate
without one and rely on layers 2-4.

### Layer 2 — IMU / magnetometer / COG-based heading (< 60 s)

Meridian's EKF maintains attitude from IMU alone. With the external
compass calibrated (tablet cal wizard ships today), heading stays
accurate for minutes, and position drift during this window is
dominated by unknown currents — maybe 0.5 m/s of drift, yielding ~30 m
in one minute.

Combined with GPS course-over-ground above ~0.8 m/s, we get a cross-check
independent of the magnetometer.

### Layer 3 — VESC motor telemetry for dead-reckoning (< 5 min)

Phase 2 upgrade (VESC UART): motor RPM gives forward-speed estimate with
~2 % accuracy. Combined with compass heading, we get honest
dead-reckoning that holds position estimate within ~30 m over five
minutes of total GPS blackout. More than enough time to RTL, loiter, or
call for intervention.

### Layer 4 — Bathymetric map-matching (no time bound)

Novel tertiary positioning unique to Meridian: the Orca already streams
depth from any N2K sounder. Given a loaded chart, a particle filter
matches "the seafloor signature right now" against the chart to recover
position with no active GPS at all.

Implementation lives in `tools/orca-bridge/meridian_orca/bathy_match.py`.
Research-grade but shipping-capable for inland / coastal / surveyed
waters (≈ 90 % of real USV operations).

**No competitor offers this.**

---

## 5. Numbers that close the deal

| Metric | Industry dual-GPS | Vanguard (Orca only) |
|---|---|---|
| Dedicated GPS antennas on deck | 2 | **0** |
| External cable runs | 4+ (power + data × 2) | **1** (Ethernet from Orca) |
| GPS update rate | typically 5 Hz | **10 Hz** (Orca's internal GNSS) |
| Position accuracy | 1-3 m RTK, 2-5 m non-RTK | < 3 m (Orca) |
| Heading source | GPS moving-baseline (primary) + mag (backup) | Cube mag + Orca 9-axis IMU + COG fusion |
| Secondary source failover | Yes | **Yes** — Meridian EKF arbitration, < 50 ms |
| Dead-reckoning window (GPS-lost) | ~1 min typical | **~5 min** (with VESC telemetry) |
| Tertiary position source | None | **Bathymetric map-matching** |
| Install cost | $800-$1,500 hardware + 4-6 hr install | $0 additional hardware + 30 min install |

The 10 Hz GPS rate is worth noting: **our "weaker" single-GPS setup
actually runs at 2× the rate of most competitors' dual-GPS setups.**
Tighter control loop, tighter station-keeping.

---

## 6. The install experience

Old way (industry standard):

1. Mount GPS 1 antenna on mast (drill holes)
2. Run GPS 1 data + power cable below decks (RG-142 coax + DC)
3. Mount GPS 2 antenna 30 cm away (more drilling)
4. Run GPS 2 cable
5. Align antennas front-to-back on yaw axis
6. Configure moving-baseline on both receivers
7. Run compass calibration
8. Survey-verify antenna positions

**Time: half a day. Parts: $500-$1,500. Integration risk: moderate.**

New way (Vanguard):

1. Plug Ethernet cable from Orca Core 2 to companion computer
2. Import our parameter bundle (`vanguard_defaults.meridian-params`) on
   the tablet
3. Boot. Done.

**Time: 30 minutes. Parts: $0 additional. Integration risk: trivial.**

---

## 7. Objection handling

### "But two GPSes is the industry baseline for a reason."
True — for moving-baseline heading and redundancy. We provide both:
heading via 4-source fusion (Cube mag + Orca 9-axis IMU + GPS COG +
gyro), redundancy via Meridian's built-in source arbitration to any
existing serial puck.

### "What if the Orca dies?"
Four-layer degradation: serial-GPS failover (< 50 ms), IMU + mag
dead-reckoning (< 1 min), VESC telemetry (< 5 min), bathymetric
map-matching (no time limit in surveyed waters). Compared to typical
competitors who have only layer 2 available.

### "RTK for cm-level accuracy?"
Orca's internal GPS isn't RTK-capable. For survey-grade customers who
need cm-level positioning, we support adding one external RTK puck on
the Cube's serial GPS port — it becomes primary, Orca becomes
redundancy. All the install simplicity still applies when RTK isn't
needed.

### "We don't have an Orca Core 2."
Then buy one ($500, < 1 hr install). You get: single-GPS-replacement
savings + chartplotter + N2K gateway + iPad charting app. ROI is
break-even at one boat, positive at two.

### "We already have our own N2K gateway."
Great — if it exposes Signal K or Yacht Devices RAW, our bridge works
unchanged. Code auto-detects. We've confirmed compatibility with
YDEN-02, YDWG-02, Actisense W2K-1, and Signal K Node servers.

---

## 8. What we ship

- **Meridian firmware** for the Cube Plus (same H7 family as our
  reference Matek H743) — `boards/CubeOrangePlus.toml`
- **`meridian-orca-bridge`** Python daemon (under 1,000 lines, open source)
- **`vanguard_defaults.meridian-params`** — tablet-importable bundle
- **Tablet GCS** with GPS source panel and mag-cal wizard
- **VESC UART upgrade path** when installed (motor telemetry for
  extended dead-reckoning)
- **Bathymetric map-matching** prototype — activates when a chart is
  loaded

All verified on the Vanguard benchtop before deployment.
