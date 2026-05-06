# Vanguard + Meridian — Field Kit Checklist

Everything Tristan (or any operator) needs for the first real field deployment.

Grouped by priority: **must-have** for the day of ops, **strongly recommended**
for field safety, and **nice-to-have** for efficient debugging.

---

## MUST-HAVE (no ops without these)

### Hardware on the boat
- [ ] Vanguard USV hull, fully fueled / battery-charged
- [ ] **CubePilot Cube Plus** running **Meridian firmware** (flash and boot-verify before launch day)
- [ ] **Orca Core 2** installed, wired to N2K + Ethernet
- [ ] NMEA 2000 backbone with depth/wind/AIS sensors connected
- [ ] Companion computer (embedded Linux box) running `meridian-orca-bridge`
- [ ] Cube ↔ companion computer UART cable (115 200 baud, MNP sensor link)
- [ ] **Kill switch** (mechanical E-stop) between battery and motor — **TEST before EVERY deploy**

### Ground station
- [ ] Laptop or tablet with:
  - [ ] Meridian tablet GCS (`mission.html`) — loaded in browser
  - [ ] Tailscale installed + connected to the tailnet
- [ ] **RFD 900 ground radio** + USB cable to laptop + antenna
- [ ] Backup internet (phone tether works — our test used iPhone hotspot)

### Safety gear
- [ ] PFDs for all hands on the water
- [ ] Chase boat or shore-recovery plan
- [ ] VHF radio on the chase boat
- [ ] First-aid kit
- [ ] Cell phone + fully charged spare power bank

### Operational paperwork
- [ ] Float plan filed with shoreside contact (who to call if we don't return)
- [ ] Mission geofence drawn on the tablet (via long-press → set station and enable fence)
- [ ] RTL point confirmed
- [ ] Weather check completed (wind, tide, forecast)
- [ ] Local notices to mariners reviewed
- [ ] Launch-site permissions confirmed

---

## STRONGLY RECOMMENDED

### Spare electronics (assume at least one thing dies today)
- [ ] Spare USB cables (A-to-B and USB-C, 1 m and 3 m)
- [ ] Spare Ethernet cable (M12 D-code → RJ45 if Orca needs it)
- [ ] Spare 12 V battery for the companion computer (if separate from main)
- [ ] Backup GPS puck (any cheap uBlox module) — Meridian source-arbitrates to it
- [ ] Spare RFD 900 antenna (cables snap, connectors corrode)
- [ ] USB hub (powered — the unpowered ones die under load)

### Diagnostic tools
- [ ] Multimeter (measure battery, verify kill-switch, trace shorts)
- [ ] USB-serial dongle (direct Cube console debugging if companion link fails)
- [ ] Wire strippers + electrical tape + small zip-ties
- [ ] Hand tools (screwdrivers, pliers) for quick repairs
- [ ] Silicone spray / marine lube for stiff connectors

### Software / data
- [ ] Exported Meridian parameter dump (from the tablet's PARAM READ)
- [ ] `vanguard_defaults.meridian-params` bundle on the GCS laptop
- [ ] Offline tablet tile cache for the operating area (in case cellular drops)
- [ ] Paper charts for the area (hard backup)
- [ ] Bathymetric chart (GEBCO or local hydrographic) — for bathy map-match sanity check

---

## NICE-TO-HAVE

- [ ] Thermal / FLIR camera for post-mission anomaly scan
- [ ] Waterproof action camera on the boat for mission replay
- [ ] Tow rope in case recovery is needed
- [ ] Sunshade for the GCS operator
- [ ] Snacks, water, insulated drink bottle
- [ ] Notebook + sharpie for logging observations that telemetry misses

---

## Pre-launch checklist (run through on-site)

### Sensors (verify on the tablet)
- [ ] GPS: status bar shows "GPS: Orca" badge within 60 s of power-on, fix ≥ 3D, ≥ 10 sats
- [ ] Compass: heading matches actual compass / manual sighting
- [ ] Depth: reads within 1 m of known depth at launch ramp
- [ ] AIS: at least 1 contact visible on the tablet if there's any local traffic
- [ ] Wind: rough magnitude matches feel
- [ ] Battery voltage: > 12.0 V on the status bar

### Comms
- [ ] Tablet GCS → Meridian on the Cube: heartbeat visible at 1 Hz, conn dot green
- [ ] Tablet GCS → SITL (fallback): verify bench functionality if the boat is disconnected
- [ ] RFD 900 link: at 10 m stand-off, verify full telemetry, then 50 m, then launch distance
- [ ] Tailscale: `ping 100.72.16.72` < 500 ms from GCS laptop

### Safety
- [ ] Kill switch: physically pull, verify motor disarms instantly
- [ ] EMERGENCY STOP button on tablet: verify disarms in < 1 s
- [ ] RTL test: trigger in bench mode, verify the boat would try to go home
- [ ] Geofence: drive toward the boundary in simulation — verify auto-RTL

### Mission
- [ ] Waypoints loaded
- [ ] Loiter point set
- [ ] Home position GPS-locked before arm
- [ ] Weather still within go/no-go
- [ ] Chase boat ready
- [ ] All hands briefed on abort procedure

---

## Post-ops wrap-up

- [ ] Save Meridian's onboard log from the Cube's SD card
- [ ] Save tablet GCS session (if a scenario was recorded)
- [ ] Photograph any damage for reporting
- [ ] Rinse boat + dry electronics
- [ ] Recharge batteries
- [ ] Note in the logbook: observations, anomalies, parameter tweaks made in the field
- [ ] Debrief: what worked, what broke, what we fix before next deploy

---

## When things go wrong in the field

| Symptom | First action |
|---|---|
| Tablet shows "Not connected" | Check RFD 900 antenna + companion computer power |
| "GPS: Orca" badge never appears | Verify companion computer running (`systemctl status meridian-orca-bridge`), Orca Ethernet cable seated, Orca has fix |
| Boat drifts but doesn't steer | Check ESC arm, kill switch, rudder readout on tablet (cmd ≠ 0 but act = 0 = mechanical fault) |
| Boat in circles | Bad compass cal — RTL, recover, rerun compass cal wizard away from dock |
| Compass says north when facing south | Magnetic interference — motor/battery/VHF cable too close to mag; move the mag module |
| Depth sounder reading 0 | Transducer clogged or cable wet at connector |
| RFD 900 link drops at distance | Check antennas, orient vertical, move ground station higher |
| Everything works then dies | Battery sag — check voltage trend on the tablet status bar |
| Don't know what's wrong | RTL (home button on tablet), recover, open the Meridian log from SD card |

---

## First-deploy acceptance tests

Pass all five to call the first field deploy a success:

1. **Boot-to-connect < 10 s** — from Cube power-on to tablet heartbeat
2. **GPS fix < 60 s** — Orca-sourced fix shown on tablet, 3D lock, HDOP < 2
3. **Bench-test actuation** — rudder sweep visible on the webcam, tablet autotune prints sane gains
4. **Station-keep hold** — long-press set station, boat holds within 2 m for 5 minutes
5. **Waypoint mission** — 3-waypoint loop completes, boat returns to home, no operator intervention
