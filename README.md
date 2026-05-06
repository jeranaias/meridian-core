# Vanguard MVP — Hand-Off Repo

> **🟡 STATUS: ALMOST READY — see [`STATUS.md`](STATUS.md).**
> Telemetry pipe + outbound commands proven end-to-end. Tablet GCS PWA-wrapped
> and connected to the live boat. **Blocking the tank test:** ESC and steering
> servo aren't mapped to ArduRover output channels yet. Tristan, see § 4 in
> STATUS.md — three quick answers from you and we're green.

---

Private working repo for getting a complete tablet GCS shipped on the
**Vanguard USV running ArduRover 4.6.3**, with Meridian firmware
iteration paused until hardware lets us iterate safely.

This is a living working repo, not a polished product release. Code,
half-finished experiments, audit notes, and brittle scripts coexist.
Tour the directories from this README and don't trust the parts of
the tree we haven't explicitly listed below — they're either shipped
features or speculative branches that aren't currently in play.

> **Public OSS Meridian remains at `github.com/jeranaias/meridian`** —
> this private repo is for Vanguard-specific deployment work. Don't
> push from here to `origin` (the public remote). Push to `vanguard` only.

---

## 1. Current state of the world

| | |
|---|---|
| **Boat firmware** | ArduRover 4.6.3 (`backups/usv-vanguard/firmware/ardurover-4.6.3-CubeOrangePlus.apj`) — live, MAVLink alive on COM4 |
| **Meridian firmware** | v1.2 baseline proven safe (safety-net fires, USB CDC silent) — **iteration frozen** until USB relay arrives (~2 days) |
| **Auto-recovery** | `usv-watch-aggressive.py` armed with 4-hour window; any DFU appearance triggers auto-reflash |
| **GCS** | Built, polished today; production page (`gcs/index.html`) + dense test console (`gcs/test-console.html`) |
| **Boat hardware** | Cube Orange Plus (STM32H757), Vanguard hull, jet drive, BEC-powered, USB-C to USV laptop |

### Why we picked ArduRover for the MVP

- ArduRover is mature, well-documented, MAVLink-complete
- Meridian's USB CDC isn't enumerating yet (deferred — see § 6)
- Meridian's safety net was empirically verified today via `--features nettest` canary
- The strategy: **ship working product on ArduRover now; iterate Meridian
  freely once the USB relay enables remote power-cycling.** No more locking
  ourselves out.

---

## 2. Quick start for Tristan / your Claude Code

### 2.1 Clone and orient

```bash
git clone git@github.com:jeranaias/vanguard-mvp.git
cd vanguard-mvp
```

Open `VANGUARD-MVP-README.md` (this file). Then:
- `docs/vanguard/00_deployment_plan.md` for the field plan
- `docs/flash-sessions/meridian-firmware-version-history.md` for what
  happened with Meridian firmware (do **not** flash any v1.x without
  reading this end-to-end)
- `docs/LAKE_TEST_RUNBOOK.md` for water-test procedure

### 2.2 Connect a browser GCS to the live boat

Boat side (USV laptop):

```bash
# Stops the auto-flash watcher and starts MAVLink WebSocket bridge.
# Prints the URL to open + the ws:// to plug in.
python tools/tristan-gcs-bridge.py
```

Tablet/laptop side (anywhere on Tailscale):

```
http://<jesse-tailscale-ip>:8765/test-console.html
   ↳ enter ws://<usv-tailscale-ip>:5760 in the WS URL field
   ↳ click Connect
```

You should see live heartbeat / mode / GPS / battery within 1 sec. Click
buttons. Each one logs to the right pane with success/timeout/denied
status from the vehicle's COMMAND_ACK.

### 2.3 If the boat is wedged (no USB enumeration)

```bash
# from the USV laptop:
python C:\Users\vangu\check-safety-net.ps1   # see what's on USB
```

Three recovery modes, in order of preference:

1. **Wait** — if any Meridian build is loaded, its safety net should
   fire within 120s and put the boat in DFU; auto-flash watcher catches.
2. **MAVLink reboot** — if it's enumerating but stuck, run
   `python C:\Users\vangu\usv-mav-reboot-to-bl.py` to drop to bootloader
   on demand.
3. **Power cycle** — when nothing else works. Until the USB relay arrives,
   this needs Tristan's hand on the BEC. After relay: scriptable.

---

## 3. Project layout (subsystem map)

```
vanguard-mvp/
├── bin/                     ← executable Rust crates (firmware, SITL)
│   ├── meridian-stm32/      ← STM32H757 firmware (no_std, RTIC)
│   └── meridian-sitl/       ← Software-in-the-loop physics + MAVLink server
├── crates/                  ← 47 Rust library crates (HAL, EKF, control, drivers…)
├── gcs/                     ← Browser GCS (vanilla JS, no build)
│   ├── index.html           ← production fly view
│   ├── test-console.html    ← dense command tester (BUILT TODAY)
│   ├── mission.html         ← legacy mission view (large)
│   ├── js/, css/, locales/  ← modules + theming + i18n
│   └── tests/               ← MAVLink/MNP codec tests
├── tools/                   ← Python utilities
│   ├── tristan-gcs-bridge.py   ← one-command bring-up (BUILT TODAY)
│   ├── mavlink-ws-bridge.py    ← MAVLink ↔ WebSocket adapter
│   ├── mnp-sitl-server.py      ← MNP SITL for dev
│   ├── usv-sim-suite.py        ← 12 boat-sim scenarios + replay
│   ├── meridian-pack.py        ← firmware packager (.bin → .apj)
│   ├── preflight-check.py      ← pre-flight system check
│   ├── terrain.py              ← ETOPO bathymetry downloader
│   ├── hwdef-to-toml.py        ← ArduPilot hwdef → board TOML
│   └── orca-bridge/            ← bathymetric terrain matching daemon
├── boards/                  ← board TOMLs (CubeOrangePlus.toml + others)
├── vehicles/                ← vehicle config TOMLs (USV, copter, plane, …)
├── docs/
│   ├── flash-sessions/      ← v0.1–v1.4 firmware iteration log
│   ├── vanguard/            ← deployment plan, hardware ID, runbook, brief
│   ├── panel_01..20_*.md    ← 20 expert panel reviews (read for design context)
│   ├── audit_*.md           ← 14 ArduPilot parity audits
│   ├── final_review_*.md    ← deep dives (control, EKF, drivers, modes)
│   ├── LAKE_TEST_RUNBOOK.md
│   ├── usv-deploy-log.md
│   └── …
├── backups/usv-vanguard/    ← every firmware candidate + parm snapshots
├── vendor/stm32h7xx-hal/    ← vendored HAL with `USB2.HIGH_SPEED = false` patch
└── results/                 ← cached USV sim scenario JSON logs
```

---

## 4. Subsystem details

### 4.1 Firmware (Rust no_std, STM32H757)

`bin/meridian-stm32/` and `crates/meridian-*` — **47 crates, ~85,000 LOC.**

Highest-leverage crates for the MVP path (ArduRover-side, not used live
yet but built and tested):

| Crate | LOC | Purpose |
|---|---|---|
| `meridian-types` | 631 | Shared data types, units, enums |
| `meridian-hal` | 1,182 | HAL traits (GPIO/PWM/UART/SPI/I2C/DMA), platform-pluggable |
| `meridian-platform-stm32` | 4,858 | Bare-metal STM32H7, RTIC tasks, **rescue.rs**, **watchdog.rs** |
| `meridian-drivers` | 14,524 | 44 driver files: IMU, baro, mag, GPS, rangefinder, DroneCAN, optical flow… |
| `meridian-comms` | 679 | MNP — COBS-framed postcard wire format |
| `meridian-mavlink` | 3,531 | MAVLink v2 bridge, CRC-X.25 framing, GPS_INPUT/DISTANCE_SENSOR/WIND_COV |
| `meridian-ekf` | 6,821 | 24-state Extended Kalman Filter (attitude + position) |
| `meridian-control` | 3,704 | Attitude stabilization PID |
| `meridian-mixing` | 2,037 | Motor/servo mixer (multirotor + USV + plane + heli) |
| `meridian-modes` | 4,700 | 44 flight modes across all vehicle classes |
| `meridian-mission` | 1,466 | 55 MAVLink commands implemented |
| `meridian-rc` | 2,501 | PPM, SBUS, DroneCAN RC decoders |
| `meridian-failsafe` | 2,594 | Safety state machine |
| `meridian-drl-tune` | 1,812 | DRL adaptive PID (SAC-inspired, no_std, 21 tests) |
| `meridian-autotune` | 1,284 | Relay feedback bootstrap (multirotor twitch + USV) |

**Status:** All compile and pass tests. Real-hardware bring-up is
blocked on USB CDC (still silent on every Meridian variant). The
**safety net** (`arm_rescue_flag` → IWDG → bootloader hold) was verified
end-to-end today on v1.2 and on a `--features nettest` canary build.
Iteration is frozen until the USB relay arrives so we can flash freely.

#### NETTEST canary

Build `--features nettest` swaps the post-IWDG path for an infinite
`nop` loop. Flash it, watch DFU appear within 5 sec, and you've proven
the safety chain on that exact code state — *before* risking a real flash.

```bash
RUST_MIN_STACK=16777216 rustup run 1.86.0 cargo build \
  --target thumbv7em-none-eabihf -p meridian-stm32-bin \
  --release --features nettest
rust-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/meridian-stm32-bin \
  target/thumbv7em-none-eabihf/release/canary.bin
python tools/meridian-pack.py pack \
  target/thumbv7em-none-eabihf/release/canary.bin 1063 CubeOrangePlus \
  > target/thumbv7em-none-eabihf/release/canary.apj
```

### 4.2 GCS (browser, vanilla JS, no build step)

`gcs/` — **~21,000 LOC JS across 31 modules.** Loads as a static
website via any HTTP server (`python -m http.server 8765`).

| View | Files | Notable features |
|---|---|---|
| `js/fly/` | 15 | ADI w/ pitch ladder, alt/airspeed tapes, compass strip, 8-field quick widget, battery widget, wind overlay, video PiP, vessel tracker, split view, spray/thermal widgets |
| `js/plan/` | 12 | Waypoint editor, 4 survey types (polygon grid, corridor, orbit, quickshots), geofence, photogrammetry, inspection, terrain profile, validator |
| `js/setup/` | 12 | Regulatory checklist (FAA/EU), accel/compass/radio cal, frame, flight modes, failsafe, motor test, firmware installer, battery |
| `js/params/` | 4 | 17 grouped categories, 50+ descriptions, search, import/export, PID tuning + step-response chart, Betaflight CLI import |
| `js/logs/` | 6 | tlog recording (IndexedDB chunks), variable-speed replay, graph viewer, MAVLink inspector, battery lifecycle, auto-analysis (6 anomaly checks), scripting console |
| `js/status/` | 1 | 166+ telemetry fields, 4Hz refresh with change animation |

Core utilities: `state.js` (multi-vehicle state + event bus),
`connection.js` (dual MNP/MAVLink WS), `router.js`, `theme.js` (dark/light),
`i18n.js`, `commands.js`, `fleet.js`, `ros-bridge.js`, `mavlink.js`,
`mnp.js`, `demo.js` (synthetic telemetry).

#### `test-console.html` — built today for dense vehicle testing

Single-page button grid with: ARM/DISARM, all 12 ArduRover modes,
DO_SET_SERVO sliders for ch1–ch10 with optional 1Hz keep-alive, mission
ops (request/clear/upload-test-3WP/start/pause/resume/set-current),
system commands (request params, set home, reboot, reboot-to-bootloader,
calibration), browser-side banner/audio tests. Live telemetry strip
across the top. ACK + STATUSTEXT logs on the right. Drop in any WS URL.

#### Polish landed today

- ADI vignette removed (Tufte-style declutter)
- Failsafe banner: full-width, 20px, color-modulating pulse, HUD edge
  red glow, distinct labels for `battery_critical` / `fence_breach` /
  `disarm_in_flight`, audio alert wired
- Quick-widget telemetry: 22px / 700-weight / 44px touch targets /
  brighter labels — sunlight readable on a tablet

#### Polish still pending (pick from these)

- Connection affordance (`CONNECT` button instead of `DISCONNECTED` text)
- HUD consolidation (alt tape + ADI + airspeed tape into one flush unit)
- Map follow-mode throttling (Meier round-2 critique #2 — perf)
- Message log expand affordance (Oborne round-2 #3)
- Multi-vehicle selector chip (Meier round-2 #3)

### 4.3 Bathymetric terrain-aided navigation

`tools/orca-bridge/` — **~6,800 LOC Python, 9 test files.**

Companion-computer daemon. Ingests NMEA 2000 (Orca Core 2: 10Hz GNSS,
9-axis IMU, N2K bridge) → forwards `MAVLink GPS_INPUT` +
`SCALED_PRESSURE` + `DISTANCE_SENSOR` to the FC.

Core algorithms:
- `bathy_rbpf.py` — Rao-Blackwellized particle filter, 1000 particles,
  per-particle Kalman for velocity + tidal current
- `bathy_match.py` — gradient-aware particle weighting via chart slope
- `bathy_cnn*.py` — learned feature matcher (CNN) + filter
- `chart_loader.py` — GEBCO ETOPO 2022 from NOAA OPeNDAP
- `current_estimator.py` — tide + current state estimation
- `ais_fusion.py` — multi-modal AIS target overlay
- `cold_start.py` — GPS-denied init heuristics
- `bathy_supervisor.py` — orchestration

Replaces naive particle matching with RBPF for ~2–3× tighter error at
the same particle count, plus MCC outlier-robust weighting.

### 4.4 DRL adaptive PID

`crates/meridian-drl-tune/` (1,812 LOC) + `crates/meridian-autotune/`
(1,284 LOC), **21 unit tests.**

Two-stage system:
1. **Relay feedback bootstrap** (60 sec) — multirotor twitch method
   (AC_AutoTune-style) or USV speed/heading axes
2. **SAC-inspired DRL agent** — observation window with normalization,
   integral + derivative computation, sea-state estimate; outputs gain
   adjustments; safety: hard gain limits, NaN/inf revert, action clipping

State: pre-trained in SITL. Not yet on real vehicle. No DRL UI in GCS
yet (auto-tune setup panel exists for the relay-feedback half only).

### 4.5 USV simulation

`tools/usv-sim-suite.py` (846 LOC) — **12 cached scenarios** in
`results/scenario_*.json`. Jet boat physics (thrust + nozzle steering +
drag + water current), realistic battery drain, autopilot to waypoint
with cross-track-error correction, JSON logs replayable into the GCS
over WebSocket.

`bin/meridian-sitl/` (4,558 LOC) — full Rust SITL boot of the actual
flight stack at 1200 Hz physics + 400 Hz control, MAVLink TCP on
127.0.0.1:5760 for QGroundControl.

### 4.6 Vanguard deployment kit

`docs/vanguard/`:
- `00_deployment_plan.md` — overall plan
- `01_hardware_identification.md` — Cube Orange Plus specs (STM32H757,
  HSE 24 MHz, OTG2 USB-C, 3× ICM42688 IMU, 2× MS5611 baro, IOMCU)
- `02_architecture_plan.md`
- `03_zero_external_gps_architecture.md` — single GPS via Orca
- `04_competitive_analysis.md`
- `05_field_kit_checklist.md`
- `DEPLOYMENT_RUNBOOK.md` — § 0–10 (pre-flight → recovery → post-flight)
- `brief/` — LaTeX briefing decks (v1–v6)
- `vanguard_defaults.meridian-params` — ~180 params, GCS-loadable

`boards/CubeOrangePlus.toml` — pin map, USB ID, UART order, SPI buses.
`vehicles/jetboat-single-nozzle.toml` — Vanguard-specific physics.

### 4.7 Tools and host-side scripts

`tools/`:
- `preflight-check.py` (468 LOC) — Python deps, MAVLink reachability,
  heartbeat rate, GPS fix, battery, terrain cache, WS bridge, HTTP server
- `mavlink-ws-bridge.py` (384 LOC) — MAVLink-to-WebSocket adapter
- `mnp-sitl-server.py` (1,422 LOC) — MNP SITL for dev
- `mnp-radio-bridge.py` — MNP serial-to-radio repeater
- `terrain.py` (408 LOC) — ETOPO 2022 downloader from NOAA OPeNDAP
- `hwdef-to-toml.py` (1,129 LOC) — ArduPilot hwdef.dat → board TOML
- `meridian-pack.py` — `.bin` → `.apj` packager
- `tristan-gcs-bridge.py` — **built today**: kills watcher + finds Cube +
  starts MAVLink WS bridge in one command

USV-laptop scripts (lives in `C:\Users\jesse\bin\`, mirrored on USV):
- `usv-watch-aggressive.py` — VID 0x2DAE bootloader catcher, 4-hour
  poll window, fires uploader against any DFU appearance
- `usv-mav-reboot-to-bl.py` — sends `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`
  param1=3 (stay in bootloader)
- `launch-meridian-v1.2.ps1`, `launch-ardurover-restore.ps1`,
  `launch-canary-nettest.ps1` — each kills any running watcher/uploader,
  re-registers a fresh ScheduledTask, then verifies the registered Action
  arguments match the intended APJ before exiting

### 4.8 Audit and review docs

`docs/`:
- 20 expert panel reviews (`panel_01_tridgell.md` …
  `panel_20_doll.md`) + 4 GCS-specific reviews + 4 round-2 wave reviews
- 14 ArduPilot parity audits (`audit_*.md`)
- 4 final reviews (`final_review_*.md`) — control/motors, drivers/protocols,
  EKF, modes/nav/safety
- `MASTER_GCS_REVIEW.md` (62KB) — comprehensive GCS critique
- `FULL_PARITY_AUDIT.md` + `PARITY_GAP_MASTER.md` + 6 parity_*.md
- `LAKE_TEST_RUNBOOK.md`, `usv-deploy-log.md`, identification_*.md (5)

**~17,700 lines of audit notes total.**

---

## 5. Branching and workflow

```
main ← stable, what you want to clone first
  ↑
feat/<thing>          ← short-lived branches per feature
fix/<thing>
chore/<thing>
```

Push directly to `main` for fast moves; cut a branch + PR if you want
review. PRs are not blocking — there's no CI gate yet (we'll add one
once we slow down).

**Don't push to public origin from this checkout.** Remotes here are:
- `origin` → public Meridian (read-only for our purposes — never push)
- `gcs-origin` → public GCS export (also don't push)
- `vanguard` → **this private repo** (push here)

If you `git push` without a remote name, **be sure git's tracking branch
config doesn't accidentally route to `origin`.** Use:
```bash
git push vanguard <branch>
```

---

## 6. What's deferred (and why)

**Meridian USB CDC.** Every variant v0.1 → v1.4 enumerates silent on
USB. v1.2 has a working safety net so the boat can self-recover; v1.3
and v1.4 broke the safety net by removing apparently-no-op register
writes that turned out to matter. We don't have a debug probe and we
don't have a USB relay yet. Iteration without one of those is a coin
flip — even with the canary, today proved we can lose a boat to a
single bad iteration. **Wait for the relay (~2 days), then iterate
freely with the canary as a hard gate before each real flash.**

**DRL adaptive PID GCS panel.** Algorithm is in `meridian-drl-tune/`,
SITL-trained, but no UI to drive it from the tablet. Add when the
auto-tune panel needs it.

**On-water shakedown.** Bench testing first, water test only after the
mission upload + RTL behavior + geofence + kill switch are all
exercised at the bench with Tristan as safety pilot.

---

## 7. References

- Cube Orange Plus pinout: `docs/vanguard/01_hardware_identification.md`
- Firmware version history (read this before flashing!):
  `docs/flash-sessions/meridian-firmware-version-history.md`
- ArduPilot bootloader source for the RTC magic check:
  `libraries/AP_HAL_ChibiOS/hwdef/common/stm32_util.h`
  (`enum rtc_boot_magic { RTC_BOOT_HOLD = 0xb0070001, ... }`)
- MAVLink message reference: https://mavlink.io/en/messages/common.html
- ArduRover parameters: https://ardupilot.org/rover/docs/parameters.html

---

## 8. Today's milestones (2026-05-06)

- v1.2 safety-net empirically observed firing end-to-end (TIM6 120s →
  bootloader DFU)
- NETTEST canary built and verified the IWDG path one cycle (then went
  silent — see version history doc for the post-mortem)
- Strategic pivot: ArduRover for production, Meridian iteration deferred
  to relay arrival
- GCS dev server running, test console built and live
- ArduRover restored cleanly via watcher after Tristan's main-power cycle
- This repo (`vanguard-mvp`) created, populated, and ready for collaboration
