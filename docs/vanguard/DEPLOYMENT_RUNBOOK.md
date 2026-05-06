# Vanguard + Meridian — Deployment Runbook

**Purpose:** hand-off document for first-flash through first-field-deploy.
Written for someone who knows the boat but hasn't been in the Meridian
codebase. If anything surprises you, stop and ask before proceeding.

---

## 0. Before you touch the boat

- [ ] Confirm Cube Plus firmware backup of the *current* ArduPilot install is on a USB stick. We are flashing over a working autopilot — if Meridian first-boot has a surprise, we need a known-good restore.
- [ ] Tablet pre-paired to RFD 900 radio on laptop side. Verify link by reading heartbeat with ArduPilot still on (sanity on the radio layer).
- [ ] Orca Core 2 already flowing data on the N2K bus. Verify with the Orca web UI (depth, GPS, wind all visible).
- [ ] Chart tile for the deploy area loaded onto the companion PC at `/opt/meridian/charts/<area>.npz`. See §7 for fetch.
- [ ] Companion PC has `meridian-orca-bridge` daemon installed but **not started** yet.

## 1. Flash Meridian to the Cube Plus — estimate: 2-4 days

The board config (`boards/CubeOrangePlus.toml`) is drafted but has
never been compiled. Expect to discover missing HAL mappings.

1. Build the firmware image from the board config:
   ```
   cd <meridian repo>
   ./tools/build-board CubeOrangePlus --release
   ```
2. Backup the current autopilot firmware — **do not skip.**
3. Flash via DFU mode: hold BOOT on the Cube while plugging USB, then:
   ```
   ./tools/flash.sh CubeOrangePlus build/out/meridian.bin
   ```
4. Boot with the Cube **disconnected from props**. Watch the console.
   Expected boot sequence:
   - HAL init messages
   - Sensor probe: IMU, baro, magnetometer, SD card
   - EKF init
   - MNP heartbeat starts
5. If boot fails, diff against the known-working ArduPilot log. Common issues:
   - Missing I2C bus mapping for the baro
   - Wrong SPI pin for the magnetometer
   - SD card controller mismatch (fatal — must fix)

Do **not** proceed to §2 until you see a stable heartbeat on serial.

## 2. Load the parameter bundle — estimate: 30 min

```
# On the laptop, tablet open in browser:
# Tools → Parameters → Load from file
# Pick: docs/vanguard/vanguard_defaults.meridian-params
```

The tablet will PARAM_SET each row, one at a time. Any parameter that
does not exist on the firmware will log a red "UNKNOWN PARAM" line.
We expect zero. If you see any: **stop**, that means the params file
is out of sync with the firmware version.

Expected parameters: ~180. Confirm the `verify complete` message before
moving on.

## 3. Bench test — estimate: 1 hour

Boat on jack stands, props can spin freely but have clearance. Webcam
pointed at the rudder.

1. Arm the boat from the tablet (ARM → confirm dialog).
2. Open Bench Test overlay (top-right hamburger → Bench).
3. Send commanded rudder: step ±50% (slider or test pattern "STEP").
4. Verify actuator follows commanded with <0.3s lag.
5. Run Rudder Autotune: hit AUTOTUNE STEER button. The filter will
   auto-suggest PID gains. Record them — you'll load these into Meridian
   in §4.
6. Throttle test: step 0→30% for 2 seconds, then back to 0. Confirm prop
   spins and stops cleanly.
7. **DISARM** before leaving the boat unattended.

## 4. Compass calibration — estimate: 15 min

Boat still on stands, or on a level dock surface with no big iron
within 3 m.

1. Tablet → Tools → Compass Cal Wizard.
2. Follow the on-screen rotation sequence (nose up, nose down, roll
   left, roll right, yaw full circle).
3. Wizard writes the offsets to Meridian. Accept the summary dialog.

## 5. Orca wire-protocol capture — estimate: 1 day

We have YD RAW and Signal K decoders ready. Orca-native is stubbed.

1. Enable packet capture on the companion PC:
   ```
   sudo tcpdump -i eth0 -w /tmp/orca_capture.pcap
   ```
2. Wait 60 s with the Orca flowing normal N2K traffic.
3. Stop capture (`Ctrl-C`).
4. Run the autodetect:
   ```
   python -m meridian_orca.protocol.detect /tmp/orca_capture.pcap
   ```
5. If autodetect says "YD RAW" or "Signal K", we're done — start the
   bridge daemon:
   ```
   systemctl --user start meridian-orca-bridge
   ```
6. If autodetect says "UNKNOWN — reverse-engineering needed", send
   the pcap to me. Expect 4-8 hours to add the decoder.

## 6. Dock test — estimate: 2 hours

Boat in the water, tied to the slip, **props can engage but not
meaningfully propel** (tied securely on at least two lines).

1. Start the bridge daemon. Tablet should show AIS traffic, depth,
   wind, GPS, compass all flowing.
2. Verify the bathy filter is running: status bar shows NAV: GPS badge.
   Bathy marker (dashed amber "B") should appear on the map within
   30 s of the filter getting its first depth samples.
3. **GPS-jam simulation test:** hit the GPS Jam button. Within 2 s:
   - NAV badge flips to amber "BATHY (GPS DENIED)"
   - If ≥2 AIS targets visible within 1.5 km: badge upgrades to cyan
     "BATHY+AIS×2" with dashed range lines to the targets
   - Bathy marker takes over as the primary position indicator
4. Restore GPS (click button again). NAV badge returns to green "GPS"
   within a few seconds.
5. Confirm all data freshness indicators (GPS, depth, AIS) show green
   dots in the status bar.

If anything in §6 shows red or stale, **do not** go to §7.

## 7. Sheltered-water autonomous mission — estimate: 2 hours

Open water (sheltered bay, channel), chase boat armed, manual takeover
ready.

1. Untie the boat.
2. Tablet → Mission → long-press map to set 3-4 waypoints in a loop.
3. Engage AUTO mode from the tablet.
4. Watch the boat follow waypoints. Expected: <5 m cross-track error
   with current Rover PID gains; tune during bench if this is off.
5. Simulate GPS jam mid-leg. Watch the boat continue autonomously
   using bathy nav for 60 s. Cross-track may widen to 20-30 m but boat
   should still follow the line.
6. Restore GPS. Boat snaps back to tight tracking.
7. Trigger RTL (return to launch) from the tablet. Confirm safe return.
8. Disarm on approach to the dock.

## 8. Full field deploy — estimate: 1 day on-site

Repeat §7 at full demo scale. Record video of:
- Autonomous waypoint loop (green nav)
- GPS-denied segment with AIS fusion visible (cyan badge + range lines)
- Threat-avoid scenario (spawn a simulated AIS target on a collision
  course and watch COLREG evasion)
- Station-keep mode (drop a virtual station, boat holds position)

Post-flight: pull telemetry logs off the companion PC via the tablet.

---

## Abort conditions — stop immediately if:

- IMU vibration warning triples (possible prop imbalance or mount fail)
- Any voltage dip below 11.0 V on a 3S setup or 14.8 V on 4S
- Compass heading drift >30° from GPS COG sustained for 5+ seconds
- Tablet loses link for more than 20 s (boat should RTL automatically,
  but be ready to recover manually)
- Water in the hull (duh)
- Props visibly spinning with ARMED = false (never should happen —
  indicates a safety interlock bug; kill main power)

## Chart loader quick reference (§0 / §7)

```bash
# Fetch NOAA ETOPO tile for your area (set LAT/LON bounds):
python -m meridian_orca.chart_loader fetch-etopo \
    --bounds -34.00 -33.90 151.15 151.25 \
    --output /opt/meridian/charts/sydney.npz

# Fetch GEBCO if you need 400m global coverage:
python -m meridian_orca.chart_loader fetch-gebco \
    --bounds ... \
    --output /opt/meridian/charts/sydney_gebco.npz

# Verify a chart:
python -m meridian_orca.chart_loader info /opt/meridian/charts/sydney.npz
```

## Known rough edges (as of 19 April 2026)

- **CNN bathy filter**: 3-5% divergence rate on fine charts under harsh
  sonar noise. Mitigation: supervisor runs bootstrap in parallel and
  hot-swaps on divergence. End-to-end divergence rate is < 0.5%.
- **Bootstrap+AIS fusion**: was collapsing on sharp likelihoods in v1;
  fixed with adaptive Cauchy half-width (v2, 19 April). Validate on
  your own data before trusting.
- **VESC UART**: wire it if you want motor telemetry; boat will work
  without.
- **Cold start (no GPS at boot)**: grid-seed cold-start works in sim;
  has never been tested on live hardware. Budget extra time if the
  boat has to boot without GPS.
- **Zero in-water hours** on this stack. Everything above is Monte
  Carlo / SITL validated. The dock test (§6) is the first real hardware
  contact.

## Support

For anything weird, ping Jesse. Logs from `journalctl -u
meridian-orca-bridge` + tablet screenshots + a one-sentence description
of what you expected vs. what happened is usually enough to debug
remotely.
