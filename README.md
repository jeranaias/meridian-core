> **Note:** This is the open-source predecessor of Meridian. Meridian is now developed commercially by **Thornveil LLC** ([thornveil.ai](https://thornveil.ai)). This repository remains as a community resource and historical reference. The productized Thornveil version contains substantial additional engineering not present here and is not publicly available.

---

<p align="center">
  <img src="assets/meridian-logo.svg" width="80" alt="Meridian">
</p>

<h1 align="center">Meridian</h1>

<p align="center">
  <strong>A modern autopilot, written from scratch in Rust.</strong><br>
  Full ArduPilot-class capability. No legacy code. No technical debt.<br>
  78,000 lines of Rust. 47 crates. 28,000 lines of browser-based GCS.<br>
  Open a URL and fly.
</p>

<p align="center">
  <a href="#why-meridian">Why</a> &middot;
  <a href="#quick-start">Quick Start</a> &middot;
  <a href="#architecture">Architecture</a> &middot;
  <a href="#the-47-crates">Crates</a> &middot;
  <a href="#ground-control-station">GCS</a> &middot;
  <a href="#supported-hardware">Hardware</a> &middot;
  <a href="#supported-vehicles">Vehicles</a> &middot;
  <a href="#flight-features">Features</a> &middot;
  <a href="#sensor-drivers">Sensors</a> &middot;
  <a href="#communication-protocols">Protocols</a> &middot;
  <a href="#building">Building</a> &middot;
  <a href="#project-status">Status</a> &middot;
  <a href="COMMUNITY.md"><strong>Get Involved</strong></a> &middot;
  <a href="#license">License</a>
</p>

<p align="center">
  <a href="COMMUNITY.md">Community Guide</a> &middot;
  <a href="gcs/README.md">GCS Docs</a> &middot;
  <a href="gcs/CONTRIBUTING.md">Contributing</a> &middot;
  <a href="docs/FULL_PARITY_AUDIT.md">Audit Notes</a> &middot;
  <a href="https://github.com/jeranaias/meridian/issues">Issues</a> &middot;
  <a href="https://github.com/jeranaias/meridian/pulls">Pull Requests</a>
</p>

---

## Why Meridian

### Standing on the Shoulders of Giants

Meridian exists because of [ArduPilot](https://ardupilot.org). Every flight algorithm in this project was studied, understood, and carefully ported from ArduPilot's remarkable codebase — the result of 15 years of development by thousands of contributors who built the most capable open-source autopilot in the world. ArduPilot flies millions of vehicles across every continent, from hobby quadcopters to industrial inspection platforms to humanitarian aid drones. It is battle-tested software that has saved lives and enabled an entire industry.

[Mission Planner](https://ardupilot.org/planner/) and [QGroundControl](http://qgroundcontrol.com/) are the ground control stations that made all of that flying possible. Mission Planner's information density and feature completeness set the standard for what a GCS should be able to do. QGroundControl's cross-platform approach and touch-first design showed that a GCS doesn't have to be Windows-only.

We owe these projects an enormous debt. Meridian is not a replacement — it's a next chapter, built on the foundation they laid.

### A Fresh Start

As systems mature, they accumulate complexity. ArduPilot's 1.5 million lines of C++ carry the weight of supporting every board, every vehicle, every edge case discovered over 15 years. That breadth is its strength — and also the reason it's increasingly difficult for new contributors to get started, for new ideas to be integrated, and for the codebase to adopt modern tooling.

Meridian asks: **what would you build if you could start fresh today, with the benefit of everything ArduPilot taught us?**

- **Rust** — Memory safety without runtime cost. The compiler catches the classes of bugs that crash vehicles: use-after-free, data races, buffer overflows. No more debugging segfaults in flight.
- **RTIC** — Deterministic real-time scheduling with compile-time priority analysis. No RTOS overhead, no priority inversion.
- **Cargo** — Build any of 47 crates with `cargo build`. Run any test with `cargo test`. The toolchain just works.
- **Board TOML** — Human-readable hardware definitions. Add a board by adding a file.
- **Native Rust extensions** instead of Lua — Type-safe, sandboxed, compiled to the same binary. No interpreter overhead. No runtime string parsing.
- **Browser-based GCS** instead of desktop apps — Open a URL. You're flying. Any device, any OS, no install.

Every flight algorithm in Meridian traces directly to ArduPilot source code, verified line-by-line against 17,697 lines of surgical audit notes covering all 154 ArduPilot libraries. This is not a simplified reimplementation. This is a complete, parity-verified autopilot.

---

## Quick Start

### Ground Control Station

The fastest way to see Meridian in action:

```bash
git clone https://github.com/jeranaias/meridian.git
cd meridian/gcs
python -m http.server 8080
```

Open [http://localhost:8080](http://localhost:8080) in any modern browser. Go to **Settings > Connection > Start Demo Mode** to explore the interface with simulated flight telemetry.

### Building the Autopilot

```bash
# Build the entire workspace
cargo build --workspace

# Build and run the SITL (Software-In-The-Loop) simulator
cargo build --bin meridian-sitl
cargo run --bin meridian-sitl

# Run the full test suite
cargo test --workspace

# Build firmware for STM32H743
cargo build --bin meridian-stm32 --target thumbv7em-none-eabihf --release
```

### Connecting GCS to SITL

1. Start the SITL: `cargo run --bin meridian-sitl`
2. Open the GCS: `http://localhost:8080`
3. Click the **DISCONNECTED** indicator in the toolbar
4. Enter `ws://localhost:5760` and connect

The GCS defaults to Meridian Native Protocol (MNP) over WebSocket. MAVLink v2 is supported for compatibility with legacy autopilots, QGroundControl, and Mission Planner.

---

## Architecture

```
meridian/
├── Cargo.toml                  # Workspace root — 47 crates
├── crates/                     # All Rust crates
│   ├── meridian-hal/           # Hardware abstraction traits
│   ├── meridian-types/         # Shared types and units
│   ├── meridian-math/          # Vectors, matrices, quaternions
│   ├── meridian-sync/          # RTIC-aware synchronization
│   ├── meridian-bus/           # Lock-free pub/sub message bus
│   ├── meridian-ekf/           # 24-state Extended Kalman Filter
│   ├── meridian-ahrs/          # Attitude + heading reference
│   ├── meridian-control/       # PID controllers, sqrt_controller
│   ├── meridian-nav/           # Waypoint navigation, spline paths
│   ├── meridian-mission/       # Mission execution (55 commands)
│   ├── meridian-modes/         # Flight modes (22 copter, 24 plane)
│   ├── meridian-mixing/        # Motor output mixing
│   ├── meridian-drivers/       # Sensor drivers
│   ├── meridian-rc/            # RC input protocols
│   ├── meridian-mavlink/       # MAVLink v2 bridge
│   ├── meridian-comms/         # Meridian Native Protocol
│   ├── meridian-params/        # Parameter system
│   ├── meridian-log/           # Binary logging
│   ├── meridian-fence/         # Geofencing
│   ├── meridian-failsafe/      # Failsafe state machine
│   ├── meridian-arming/        # Pre-arm checks
│   └── [26 more crates]        # See full list below
├── boards/                     # Board definitions (TOML)
│   ├── CubeOrange.toml
│   ├── Pixhawk6X.toml
│   ├── MatekH743.toml
│   ├── MatekL431.toml
│   └── SpeedyBeeF405Wing.toml
├── vehicles/                   # Vehicle type configurations
│   ├── quad-x.toml
│   ├── hex-x.toml
│   ├── fixed-wing.toml
│   ├── vtol-quadplane.toml
│   └── [6 more]
├── bin/                        # Binary targets
│   ├── meridian-sitl/          # SITL simulator
│   ├── meridian-linux/         # Linux companion computer
│   └── meridian-stm32/        # STM32H7 firmware
├── gcs/                        # Browser-based Ground Control Station
│   ├── index.html              # Single-file entry point
│   ├── css/                    # 18 CSS files, 6,000 lines
│   ├── js/                     # 80 JS files, 20,600 lines
│   ├── locales/                # i18n translations
│   └── tests/                  # Unit tests
├── docs/                       # Audit notes and parity analysis
└── tools/                      # Build helpers, hwdef converter
```

### Design Principles

| Principle | ArduPilot Approach | Meridian Approach |
|-----------|-------------------|-------------------|
| **Memory safety** | Manual C++ memory management, runtime checks | Rust ownership — compile-time memory safety, zero-cost abstractions |
| **Concurrency** | Recursive mutexes, RTOS threads | RTIC interrupt-driven scheduling, priority inversion free |
| **Hardware abstraction** | AP_HAL class hierarchy with virtual dispatch | Trait-based HAL — static dispatch, no vtable overhead |
| **Build system** | WAF (Python), hwdef.dat preprocessor | Cargo workspace, board TOML, `build.rs` code generation |
| **Modularity** | Monolithic `libraries/` directory | 47 independent crates — test any subsystem in isolation |
| **Configuration** | hwdef.dat with C preprocessor macros | Human-readable TOML with schema validation |
| **Scripting** | Lua interpreter (runtime overhead) | WASM sandbox (compiled, type-safe) |
| **GCS protocol** | MAVLink v2 only | Meridian Native Protocol (MNP) primary, MAVLink v2 for compatibility |
| **GCS application** | Desktop apps (WinForms, Qt) | Browser-based — zero install, any device |
| **Testing** | Python test harness, SITL only | Rust `#[test]` on every crate + SITL integration |

### The Control Loop

```
400Hz Main Loop:
  ┌─────────────┐
  │  Sensor Read │  IMU DMA → FIFO drain → calibration → rotation → filter
  └──────┬──────┘
         │
  ┌──────▼──────┐
  │  EKF Update  │  24-state prediction → innovation → Kalman gain → state update
  └──────┬──────┘
         │
  ┌──────▼──────┐
  │  Mode Logic  │  Position/velocity/attitude targets from active flight mode
  └──────┬──────┘
         │
  ┌──────▼──────┐
  │  Controller  │  Position → Velocity → Attitude → Rate → Motor output
  └──────┬──────┘
         │
  ┌──────▼──────┐
  │  Motor Mix   │  Thrust + torque → per-motor PWM/DShot commands
  └──────┬──────┘
         │
  ┌──────▼──────┐
  │  Output      │  DMA transfer to ESCs, logging, telemetry
  └─────────────┘
```

---

## The 47 Crates

Meridian is organized into 47 independent crates across 7 architectural layers. Every crate compiles independently, has its own test suite, and declares explicit dependencies.

### Foundation Layer

| Crate | Description |
|-------|-------------|
| `meridian-types` | Shared types: `LatLon`, `Attitude`, `VehicleState`, SI units with type-level safety, timestamps, enums for modes/commands/status |
| `meridian-math` | Quaternions, 3D vectors, rotation matrices, coordinate frame conversions (NED/ENU/body/earth), geodetic math (haversine, vincenty), type-safe angle units |
| `meridian-sync` | `no_std` synchronization primitives: `RecursiveMutex` (required for ArduPilot algorithm parity), priority-aware locking for RTIC, 9 tests passing |

### Core Infrastructure

| Crate | Description |
|-------|-------------|
| `meridian-bus` | Typed publish/subscribe message bus with lock-free ring buffers (`no_std` mode) or crossbeam channels (`std` mode). Decouples producers and consumers across the flight stack |
| `meridian-hal` | Hardware abstraction traits: `UartDriver`, `SpiDevice`, `I2cDevice`, `Gpio`, `RcOutput`, `RcInput`, `Storage`, `Scheduler`, `AnalogIn`, `Timer`. No implementations — just interfaces |
| `meridian-boardcfg` | Board configuration system: parses TOML board definitions, generates Rust code via `build.rs`. Replaces ArduPilot's `hwdef.dat` + Python preprocessor |

### State Estimation

| Crate | Description |
|-------|-------------|
| `meridian-ekf` | 24-state Extended Kalman Filter (EKF3 port): position, velocity, attitude (quaternion), gyro bias, accel bias, earth magnetic field, body magnetic field, wind velocity. Covariance prediction with 24x24 matrix, sequential measurement fusion, health monitoring |
| `meridian-ahrs` | AHRS manager: multi-core EKF with lane switching (up to 3 parallel EKF instances), DCM fallback for degraded GPS, compass consistency checking, vibration-based IMU weighting, automatic source selection |

### Sensor Drivers

| Crate | Description |
|-------|-------------|
| `meridian-drivers` | Complete sensor driver suite — see [Sensor Drivers](#sensor-drivers) section for full detail. Covers IMU (ICM-42688, BMI270, BMI088, MPU6000), barometer (BMP280, BMP388, DPS310, MS5611), compass (IST8310, QMC5883L, RM3100, LIS3MDL), GPS (uBlox, NMEA), rangefinder, optical flow, airspeed |

### Control Systems

| Crate | Description |
|-------|-------------|
| `meridian-control` | Cascaded PID controllers: attitude (angle) → rate → motor output. Includes `sqrt_controller` kinematic shaper (core of all position/velocity tracking in ArduPilot), feed-forward, I-term limiting, derivative filtering. Per-axis tuning with autotune integration |
| `meridian-mixing` | Motor/actuator mixing: 38 frame presets (Quad X/+/H/V, Hex X/+, Octa X/+, Y6, Tri, DodecaHex, Deca). Thrust linearization with voltage compensation, battery sag resistance estimation. 190 servo function slots |
| `meridian-modes` | Flight mode state machines �� 22 copter modes (Stabilize, AltHold, Loiter, Auto, Guided, RTL, Land, Circle, Drift, Sport, Flip, AutoTune, PosHold, Brake, Throw, SmartRTL, FlowHold, Follow, ZigZag, SystemID, Heli_Autorotate, Turtle) and 24 plane modes |

### Navigation & Guidance

| Crate | Description |
|-------|-------------|
| `meridian-nav` | L1 guidance controller, waypoint navigation with acceptance radius, Hermite/Catmull-Rom spline corners, 7-phase jerk-limited S-curve trajectories, terrain following with lookahead, orbit/loiter control |
| `meridian-mission` | Behavior tree mission engine: 55 NAV/DO commands (WAYPOINT, TAKEOFF, LAND, RTL, LOITER_UNLIM/TIME/TURNS, SPLINE_WAYPOINT, GUIDED_ENABLE, DO_SET_MODE, DO_SET_SERVO, DO_SET_RELAY, DO_REPEAT_SERVO, DO_SET_ROI, DO_DIGICAM_CONTROL, DO_MOUNT_CONTROL, DO_GRIPPER, DO_PARACHUTE, DO_WINCH, and more). Arena-allocated, `no_std` compatible |
| `meridian-fence` | Geofencing: polygon inclusion/exclusion zones, circular zones, altitude ceiling/floor. Breach detection with configurable actions (report, RTL, land, brake). Polygon point-in-polygon test with winding number algorithm |
| `meridian-terrain` | Terrain database: 32x28 SRTM grid blocks, 12-block LRU cache (~22KB RAM), bilinear interpolation, MAVLink TERRAIN_REQUEST/DATA protocol for GCS-assisted terrain data |

### Mission Safety

| Crate | Description |
|-------|-------------|
| `meridian-failsafe` | Failsafe monitor: RC loss (configurable timeout), battery voltage/capacity, GCS heartbeat timeout, EKF variance, geofence breach. Priority-based action selection (land > RTL > continue > report). Concrete types, no dynamic dispatch |
| `meridian-arming` | Pre-arm check framework: GPS quality (fix type, HDOP, satellite count), compass calibration and consistency, IMU health (vibration, temperature, multi-IMU agreement at 0.75 m/s^2 accel / 5 deg/s gyro sustained 10s), barometer calibration, RC calibration, battery health, fence configuration |

### Communication

| Crate | Description |
|-------|-------------|
| `meridian-comms` | Meridian Native Protocol (MNP): COBS framing over any byte stream, postcard binary serialization, typed message envelopes. Designed for low-overhead, low-latency vehicle-to-GCS communication over WebSocket |
| `meridian-mavlink` | MAVLink v2 bridge: CRC-X.25 with per-message CRC_EXTRA, 90+ message encode/decode handlers, stream rate system (10 groups), HMAC-SHA256 signing, 8s mission upload timeout, 30-entry statustext queue. Full compatibility with QGroundControl and Mission Planner |
| `meridian-rc` | RC input protocol decoders: SBUS (with dual failsafe — explicit flag AND implicit channel check), CRSF/ELRS (with CRSFv3 baud negotiation), SRXL2, DSM/DSMX, SUMD, ST24, FPort, PPM. Telemetry: FrSky Sport passthrough with 10+ app IDs |
| `meridian-can` | DroneCAN (UAVCAN v0): CAN frame encoding/decoding, dynamic node allocation (DNA) server, ESC RawCommand output, sensor message dispatch. Built on `libcanard` FFI for protocol compliance |
| `meridian-adsb` | ADS-B traffic awareness: ICAO address tracking, position/velocity/heading decode, threat assessment with distance and closure rate, configurable avoidance actions |

### Logging & Parameters

| Crate | Description |
|-------|-------------|
| `meridian-log` | Structured binary logging: AP-compatible format with 0xA3/0x95 sync bytes, FMT format messages, 150+ message types, per-message rate limiting via 256-element timestamp array. SD card or file output |
| `meridian-params` | Runtime parameter system: 18-bit group tree encoding (EEPROM-stable format compatible with ArduPilot), flash wear-leveling for embedded, file-backed for Linux/SITL. Supports PARAM_REQUEST_LIST, PARAM_SET, PARAM_VALUE MAVLink protocol |

### Advanced Features

| Crate | Description |
|-------|-------------|
| `meridian-autotune` | PID auto-tuning: multirotor "twitch" method (180 deg/s steps, 5-step sequence, 4 consecutive passes, 25% backoff on oscillation), helicopter frequency-sweep chirp with gain/phase extraction |
| `meridian-fft` | Real-time FFT on gyro data: tracks 3 simultaneous noise peaks using distance-matrix matching, harmonic detection (frequency within 10% of N * fundamental), dynamic notch filter center frequency update every control loop iteration |
| `meridian-precland` | Precision landing: separate 2-state Kalman filter per axis (X/Y), inertial history ring buffer for lag compensation, IR-Lock/companion computer sensor fusion |
| `meridian-proximity` | Proximity sensor integration: 8 sectors x 5 layers = 40 3D boundary faces, 3-sample median filter + IIR smoothing, AC_Avoid velocity bending for obstacle avoidance |
| `meridian-mount` | Gimbal/mount control: 14 backend types — Servo (direct PWM), MAVLink (GIMBAL_MANAGER protocol), Siyi (binary serial with P=1.5 angle-to-rate controller), Gremsy, Alexmos, SToRM32 |
| `meridian-camera` | Camera trigger: servo/relay pulse, MAVLink Camera v2 protocol, distance/time-based triggering, GPS geotagging with shutter lag compensation |

### Vehicle-Specific

| Crate | Description |
|-------|-------------|
| `meridian-heli` | Helicopter support: swashplate mixing (H1/H3/H4), collective-to-yaw feedforward, rotor speed governor, autorotation entry/glide/flare state machine, tail rotor modes |
| `meridian-vehicle` | Vehicle definition loader: parses TOML vehicle configs (mass, motor layout, PID defaults, failsafe thresholds, sensor assignments), dynamics model validation |

### Payload & Utilities

| Crate | Description |
|-------|-------------|
| `meridian-notify` | LED + buzzer notification: priority-based pattern scheduler (boot, arm, GPS lock, failsafe, battery warning, EKF error), NeoPixel via SPI DMA, piezo tune sequences |
| `meridian-osd` | On-screen display: MAX7456 analog (SPI, character-cell 30x16 PAL) + MSP DisplayPort digital (DJI/Avatar/HDZero). 56+ display items, 4 switchable screen layouts |
| `meridian-opendroneid` | FAA/EU Remote ID: 5 message types (Basic ID, Location, Authentication, Self-ID, System), WiFi NaN / BLE 4/5 broadcast, MAVLink relay. Pre-arm requires arm_status handshake |
| `meridian-parachute` | Parachute deployment: motor shutdown sequence, configurable delay, servo/relay pulse. Trigger on altitude, speed, or manual command |
| `meridian-gripper` | Gripper control: servo + EPM (electromagnet) backends, grab/release state machine with feedback |
| `meridian-winch` | Winch control: position/rate/RC control modes, line length tracking |
| `meridian-landing-gear` | Landing gear: deploy/retract servo control, altitude-based auto triggers, weight-on-wheels sensor input |
| `meridian-sprayer` | Agricultural sprayer: pump + spinner PWM control, ground-speed-proportional flow rate |
| `meridian-wasm` | WASM extension runtime: sandboxed execution environment for user scripts, 350-400 host bindings to vehicle state. Replaces ArduPilot's Lua scripting with compiled, type-safe extensions |

### Platform Implementations

| Crate | Description |
|-------|-------------|
| `meridian-platform-stm32` | STM32H7 platform: clock init (8MHz HSE → 400MHz PLL), DMA with per-stream recursive mutex, SPI with DMA_NOSHARE for IMU buses, I2C with bus clear recovery, 9 UARTs, USB CDC, Timer PWM + DShot via DMA, ADC, Flash storage (pages 14-15), SD card (SDMMC + FatFs), hardware watchdog. RTIC task mapping with interrupt priorities |
| `meridian-platform-linux` | Linux companion computer: Tokio async runtime, serial UART, SPI/I2C via spidev/i2cdev, UDP networking, file-backed storage |
| `meridian-platform-sitl` | Software-in-the-loop: UDP physics bridge (port 5501), TCP MAVLink server (port 5760), simulated SPI/I2C returning sensor data, file-backed storage |
| `meridian-sitl` | SITL physics engine: rigid body dynamics with configurable vehicle models, sensor noise simulation, wind model, deterministic replay mode for regression testing |
| `meridian-gcs` | GCS crate: WASM bindings for browser-side Rust code (telemetry parsing, protocol handling). The main GCS is in `gcs/` as vanilla JS/HTML/CSS |

---

## Ground Control Station

The `gcs/` directory contains a complete browser-based Ground Control Station — **28,000 lines** of hand-written JavaScript, CSS, and HTML with zero framework dependencies.

### Why Browser-Based

| Question | Mission Planner | QGroundControl | Meridian GCS |
|----------|----------------|----------------|--------------|
| Install required? | Yes (Windows MSI) | Yes (platform binary) | No — open a URL |
| Works on tablet? | No | Partial (Qt scaling) | Yes — touch-first design |
| Works on phone? | No | "Painful" (QGC docs) | Yes — responsive breakpoints |
| Dark theme? | HUD only | 2 presets | Full dark/light with canvas adaptation |
| Offline capable? | Desktop app | Desktop app | Yes — Cache API tile storage |
| Custom instruments? | Limited | Grid widget | 8-field color-coded quick readout |
| Build step? | Visual Studio | Qt/CMake | None — edit and refresh |

### GCS Features

**Fly View:**
- Canvas flight instruments: ADI with pitch ladder and bank marks, horizontal compass strip, speed and altitude tapes — all theme-aware and GPU-accelerated
- Leaflet map: vehicle icon with heading rotation, velocity trail, trajectory projection, home guide line, mission path overlay, ADSB traffic markers, drag-to-fly, geofence display, uncertainty ellipse
- 8-field quick readout: altitude, speed, home distance, WP distance, climb rate, heading, flight time, throttle — color-coded per field, right-click customizable
- Always-visible telemetry strip: GPS fix + satellites, battery % + voltage, RC RSSI, EKF variance, flight time — visible across all views
- Wind estimation: real-time speed and direction from ground/airspeed vector difference with arrow overlay on map
- Battery intelligence: time remaining estimate from current draw + capacity, consumption rate (mAh/min), color-coded warnings
- Video feed: MJPEG/RTSP PiP overlay, draggable, fullscreen swap
- Slide-to-arm with pre-flight checklist gate, long-press emergency KILL with 1.5s hold

**Plan View:**
- Click-to-add waypoints with drag reorder and inline parameter editing
- Survey tools: polygon grid scan, corridor scan, orbit missions, cinematic quickshots
- Terrain altitude profile chart with ground clearance warnings
- Mission validator: altitude/distance limits, battery endurance vs flight time, duplicate waypoint detection, first-WP-not-takeoff warning
- Statistics: distance, estimated time, max altitude, max distance from home, battery endurance margin
- Geofence polygon drawing with FENCE_POINT upload/download
- Import/Export: QGC WPL 110 waypoint file format

**Setup:**
- Pre-flight regulatory checklist: 7-item FAA/EU compliance check with auto + manual items
- Calibration wizards: accelerometer (6-position), compass (3-axis visualization), radio (live channel bars)
- Frame selection: visual grid with motor layout diagrams
- Flight modes: 6-slot configuration with PWM range visualization
- Failsafe: RC loss, battery, GCS timeout with action selection
- Motor test: individual motor spin with throttle and duration control
- Firmware: OTA update placeholder

**Parameters:**
- Grouped by prefix: 17 categories (Attitude Control, Battery, Failsafe, Geofence, Navigation, etc.)
- 50+ parameter descriptions with human-readable explanations
- Search, load from file, save to file, Betaflight CLI dump import
- PID tuning panel with per-axis sliders and step response chart
- Modified parameter highlighting with default comparison

**Logs:**
- Tlog recording to IndexedDB with 64KB chunks, auto-start on connect
- Flight replay: play back tlog through HUD instruments and map at 0.5x-10x speed
- Time-series graph viewer for any telemetry field
- MAVLink inspector: live message stream with field-level decode and XSS escaping
- Battery lifecycle tracking per pack with cycle count and health scoring
- Auto-analysis: 6 anomaly checks on recorded flight data
- Scripting console: sandboxed JavaScript with vehicle state access helpers

**Status:**
- 166+ telemetry fields organized by category (System, Attitude, Position, GPS, EKF, RC, Battery, Mission)
- Units on every value (m, m/s, V, A, %, mAh, degrees)
- 4Hz live update with change-flash animation
- Collapsible category sections with search/filter
- Alternating row colors for readability

**Settings:**
- 22 configuration sections: theme, units, map provider, connection, ADSB, operator identity, EU compliance, offline maps, recording, ROS2 bridge, STANAG 4586, audio alerts
- Offline tile caching with area selection and zoom range
- Multi-vehicle connection pool with fleet registry
- i18n framework with locale-based string translation
- Demo mode toggle

**Accessibility:**
- `:focus-visible` rings on all interactive elements
- Skip-to-content link
- ARIA roles on all custom widgets (instruments, arm slider, health grid)
- `prefers-reduced-motion` support — disables all animations
- `prefers-color-scheme` detection for automatic dark/light
- Print stylesheet
- Thin scrollbar styling
- Keyboard shortcuts with `?` overlay (F/P/S/R/L/T/Esc/Ctrl+,/Ctrl+Shift+A/K)
- Touch targets: 44px minimum, 48px on coarse pointer devices

---

## Supported Hardware

### Flight Controllers

| Board | MCU | IMU | Baro | Compass | Features |
|-------|-----|-----|------|---------|----------|
| **CubeOrange** | STM32H743 @ 400MHz | ICM20602 + ICM20948 (dual) | MS5611 (dual) | AK09916 | IOMCU, dual CAN, 6 FMU PWM, FRAM, SD card |
| **Pixhawk6X** | STM32H743 @ 400MHz | ICM42688 + BMI088 + ICM42670 | BMP388 + ICP201XX (dual) | BMM150 / RM3100 | IOMCU, dual CAN, Ethernet, FRAM, 8 FMU PWM |
| **MatekH743** | STM32H743 @ 480MHz | ICM42688 | DPS310 | — | 13 PWM outputs, SD card, DShot |
| **MatekL431** | STM32L431 @ 80MHz | — | — | — | DroneCAN peripheral node |
| **SpeedyBeeF405Wing** | STM32F405 @ 168MHz | ICM42688 | DPS310 | — | Wing-specific, 12 PWM, SD card |

These are the 5 Tier 1 boards with full sensor/pin configurations. Beyond these:

| Tier | Boards | Status |
|------|--------|--------|
| **Tier 1** | 5 | CI-tested, recommended for first flights |
| **Tier 2** | 19 | Popular boards (KakuteH7, Pixhawk4, CubeBlack, Durandal, etc.), validated TOML |
| **Tier 3** | 359 | Auto-converted from ArduPilot hwdef.dat — community validation needed |
| **Total** | **383** | Every board ArduPilot supports, converted to TOML |

Board definitions are TOML files in `boards/`. Adding a new board requires only a TOML file — no code changes. See [COMMUNITY.md](COMMUNITY.md) for how to help validate Tier 3 boards.

### Target Platforms

| Platform | Crate | Runtime | Use Case |
|----------|-------|---------|----------|
| STM32H7 | `meridian-platform-stm32` | RTIC | Flight controller firmware |
| Linux | `meridian-platform-linux` | Tokio | Companion computer, Raspberry Pi |
| SITL | `meridian-platform-sitl` | std | Development, testing, CI |

---

## Supported Vehicles

All vehicle types are defined as TOML configurations in `vehicles/`:

| Vehicle | Type | Motors | Key Parameters |
|---------|------|--------|---------------|
| **Quad-X** | Multirotor | 4 | 1.5kg, DShot600, 30N thrust |
| **Quad-Plus** | Multirotor | 4 | Plus motor layout |
| **Hex-X** | Multirotor | 6 | Redundant motors |
| **Octo-X** | Multirotor | 8 | Heavy lift |
| **Fixed-Wing** | Airplane | 1 | 2.0kg, 0.5m^2 wing, TECS |
| **VTOL QuadPlane** | Hybrid | 5 | Quad + pusher, transition logic |
| **Rover (Skid)** | Ground | 2 | Skid steering |
| **Rover (Ackermann)** | Ground | 1+servo | Car-style steering |
| **Boat** | Marine | 2 | Differential thrust |
| **Sub (6DOF)** | Underwater | 6 | Full 6-axis control |

### Motor Mixing

38 frame geometry presets:

- **Quad:** X, Plus, H, V, A-Tail, Y4
- **Hex:** X, Plus, CoaxCopter
- **Octa:** X, Plus, H, DJI, CoaxQuad
- **Y6:** Standard, Inverted
- **Tri:** Standard (with yaw servo)
- **DodecaHex:** X, Plus
- **Deca:** X, Plus
- **OctaQuad:** X, H, Plus, V

---

## Flight Features

### Flight Modes

**Copter (22 modes):**

| Mode | Description |
|------|-------------|
| Stabilize | Manual throttle, self-leveling attitude |
| AltHold | Barometric altitude hold, manual position |
| Loiter | GPS position + altitude hold |
| Auto | Autonomous mission execution |
| Guided | GCS-commanded position targets |
| RTL | Return to launch with configurable altitude |
| Land | Autonomous landing with ground detection |
| Circle | Orbit around a point |
| Drift | Rate-controlled with GPS speed limiting |
| Sport | High-rate manual with GPS assist |
| Flip | Automated flip maneuver |
| AutoTune | In-flight PID optimization |
| PosHold | Simplified loiter with direct stick response |
| Brake | Immediate deceleration to stop |
| Throw | Launch detection + automatic stabilization |
| SmartRTL | Return via recorded path with loop pruning |
| FlowHold | Optical flow position hold (no GPS) |
| Follow | Follow another vehicle or GCS |
| ZigZag | Precision agriculture back-and-forth |
| SystemID | System identification for tuning |
| Heli_Autorotate | Helicopter emergency autorotation |
| Turtle | Flip-over recovery (Betaflight-style) |

**Plane (24 modes):** MANUAL, CIRCLE, STABILIZE, TRAINING, ACRO, FLY_BY_WIRE_A, FLY_BY_WIRE_B, CRUISE, AUTOTUNE, AUTO, RTL, LOITER, TAKEOFF, GUIDED, QSTABILIZE, QHOVER, QLOITER, QLAND, QRTL, QAUTOTUNE, QACRO, THERMAL, SOARING, AUTOLAND

### Navigation

- **L1 guidance controller:** Period-based lateral guidance with damping ratio
- **Waypoint navigation:** Configurable acceptance radius, flythrough vs stop-at-waypoint
- **Spline paths:** Hermite/Catmull-Rom interpolation for smooth corners
- **S-curve trajectories:** 7-phase jerk-limited profiles for precise position control
- **Terrain following:** Continuous altitude adjustment using terrain database with lookahead
- **SmartRTL:** Records breadcrumb trail, prunes loops (Douglas-Peucker + intersection detection), replays in reverse
- **Orbit/Circle:** Configurable radius, speed, direction, and center tracking

### Safety Systems

- **Pre-arm checks:** 15+ categories verified before arming (GPS, compass, IMU, baro, RC, battery, fence, EKF)
- **Failsafe cascade:** RC loss → battery low → battery critical → GCS loss → EKF failure → geofence breach, each with configurable action (continue, RTL, land, brake, SmartRTL)
- **Geofencing:** Polygon + circle + altitude, inclusion and exclusion zones, breach actions
- **Emergency kill:** MAV_CMD_DO_FLIGHTTERMINATION — immediate motor shutdown
- **Parachute:** Altitude/speed triggered or manual, motor shutdown + servo pulse sequence
- **Watchdog:** Hardware watchdog on STM32, software watchdog on all platforms

### Signal Processing

- **Biquad low-pass filters** on all sensor data
- **Notch filters** with 5% slew limiter (critical — without slew limiting, filter rings on step input)
- **Harmonic notch filter:** Up to 16 harmonics, 5 tracking modes (throttle, RPM, FFT, ESC telemetry, fixed)
- **GyroFFT:** Real-time 3-peak tracking with distance-matrix matching, harmonic validation (within 10% of N * fundamental)

---

## Sensor Drivers

### Inertial Measurement Units

| Sensor | Interface | Features |
|--------|-----------|----------|
| ICM-42688 | SPI + DMA | 12-register init, FIFO with 16-byte packets, header 0x68 validation, AFSR bug mitigation (critical: disable anti-alias filter or gyro stalls at 100 deg/s) |
| BMI270 | SPI | 4KB firmware upload before operational, separate accel/gyro FIFO frames |
| BMI088 | SPI | Separate accel/gyro dies, individual FIFO config |
| MPU6000/9250 | SPI | Legacy support, 512-byte FIFO |

**IMU Pipeline:** DMA read → FIFO drain → temperature calibration (3rd-order polynomial) → board rotation → low-pass filter → notch filter → EKF input

**Multi-IMU:** Up to 3 simultaneous IMUs with health monitoring. Consistency check: angle difference for gyro (< 5 deg/s sustained 10s), vector distance for accel (< 0.75 m/s^2 sustained 10s). Vibration detection via 5Hz + 2Hz LP envelope, clipping at 152 m/s^2.

### Barometers

| Sensor | Interface | Compensation |
|--------|-----------|-------------|
| BMP280 | I2C/SPI | Integer Bosch algorithm |
| BMP388/390 | I2C/SPI | Float 11-coefficient compensation |
| DPS310 | I2C/SPI | Temperature-compensated |
| MS5611 | I2C/SPI | PROM calibration coefficients |
| ICP-201XX | I2C | High-accuracy industrial |

**Baro Pipeline:** Raw ADC → compensation algorithm → ground calibration (10s settle + 5-sample average) → low-pass filter → altitude conversion

### Compass/Magnetometer

| Sensor | Interface | Features |
|--------|-----------|----------|
| IST8310 | I2C | 200Hz, 16-bit |
| QMC5883L | I2C | Temperature compensated |
| RM3100 | SPI | High dynamic range |
| LIS3MDL | I2C/SPI | 80Hz, low noise |
| AK09916 | I2C | On-chip in ICM-20948 |
| BMM150 | I2C | Integrated in BMI088 |

**Compass Calibration:** Levenberg-Marquardt optimization fitting sphere + ellipsoid model (9 parameters: 3 offsets, 3 diagonal scale, 3 off-diagonal). Motor compensation: per-throttle or per-current 3D vector subtraction. Consistency check: 3D angle < 90 degrees, XY angle < 60 degrees.

### GPS/GNSS

| Receiver | Interface | Features |
|----------|-----------|----------|
| uBlox M8/M9/F9 | UART | 22-step configuration, 8-baud auto-detect (9600→460800), NAV-PVT/NAV-STATUS/NAV-DOP/NAV-SAT parsing, RTK via RTCM3, heading via RELPOSNED |
| NMEA | UART | GGA + RMC within 150ms window, standard position/fix/satellite data |

**GPS Pipeline:** Auto-detect → configure → parse → health check (delta_time EMA, delayed_count) → blending (inverse-variance weighting for dual GPS) → EKF fusion

---

## Communication Protocols

### Meridian Native Protocol (MNP)

The primary vehicle-to-GCS protocol, designed for modern transport layers.

- **Framing:** COBS (Consistent Overhead Byte Stuffing) — zero-byte-free encoding for reliable framing over any byte stream
- **Serialization:** postcard — compact binary format based on serde, `no_std` compatible
- **Transport:** WebSocket (browser GCS), UART (companion computer), USB CDC (direct)
- **Message types:** 12 defined message types for telemetry, commands, parameters, missions
- **Overhead:** 1-2 bytes per frame (COBS) + 1-4 bytes per field (varint encoding)

### MAVLink v2

Full compatibility layer for legacy ground stations and autopilots.

- **90+ message handlers** covering all standard MAVLink message types
- **10 telemetry stream groups** with configurable rates
- **Mission protocol:** MISSION_COUNT → REQUEST_INT → ITEM_INT → ACK state machine with retry logic and 8s timeout
- **Parameter protocol:** PARAM_REQUEST_LIST, PARAM_SET, PARAM_VALUE with acknowledgment
- **Command protocol:** COMMAND_LONG with COMMAND_ACK tracking
- **Signing:** HMAC-SHA256 link authentication
- **STATUSTEXT:** 30-entry queue with 50-character chunking and severity levels

### RC Protocols

| Protocol | Baud | Channels | Failsafe | Notes |
|----------|------|----------|----------|-------|
| SBUS | 100000 (inverted) | 16 | Dual: flag + channel check | Channels 1-4 <= 875us indicates implicit failsafe |
| CRSF | 416666 | 16 | 150ms RX timeout | CRSFv3 adds baud negotiation (0x70/0x71) |
| ELRS | 420000 | 16 | 150ms RX timeout | Bootstrap baud differs from CRSF |
| SRXL2 | 115200 | 32 | Handshake-based | Spektrum bidirectional |
| DSM/DSMX | 115200 | 12 | Frame loss counting | Spektrum legacy |
| PPM | — | 8 | Pulse width monitoring | Legacy analog |

### DroneCAN

| Feature | Implementation |
|---------|---------------|
| Frame format | CAN 2.0B, 29-bit ID with priority/source/destination encoding |
| Node management | Dynamic Node Allocation (DNA) server in pure Rust |
| ESC output | `uavcan.equipment.esc.RawCommand` at loop rate |
| Sensors | GPS, compass, baro, airspeed, rangefinder message dispatch |
| Firmware update | Binary upload to peripheral nodes |

---

## Building

### Requirements

| Target | Requirements |
|--------|-------------|
| All targets | Rust 1.75+ (stable), Cargo |
| STM32H7 | `rustup target add thumbv7em-none-eabihf`, `probe-rs` or `openocd` for flashing |
| Linux | Standard Linux toolchain |
| SITL | Linux or WSL with UDP networking |
| GCS | Any modern browser (Chrome 60+, Firefox 55+, Safari 11+) |

### Build Commands

```bash
# Build everything
cargo build --workspace

# Build in release mode (optimized)
cargo build --workspace --release

# Run all tests
cargo test --workspace

# Build SITL binary
cargo build --bin meridian-sitl

# Run SITL
cargo run --bin meridian-sitl

# Build STM32H743 firmware
cargo build --bin meridian-stm32 --target thumbv7em-none-eabihf --release

# Flash to board (requires probe-rs)
probe-rs run --chip STM32H743ZI target/thumbv7em-none-eabihf/release/meridian-stm32

# Build Linux companion
cargo build --bin meridian-linux --release

# Run a specific crate's tests
cargo test -p meridian-ekf
cargo test -p meridian-control
cargo test -p meridian-mavlink

# Check all crates compile for no_std
cargo check -p meridian-types --no-default-features
cargo check -p meridian-math --no-default-features
cargo check -p meridian-sync --no-default-features
```

### Project Statistics

| Metric | Count |
|--------|-------|
| Rust source lines | 78,000 |
| Crates | 47 |
| GCS source lines | 28,000 |
| GCS files | 105 |
| Vehicle profiles | 10 |
| Board targets | 5 (Tier 1) |
| MAVLink messages | 90+ |
| Flight modes | 46 (22 copter + 24 plane) |
| Sensor drivers | 20+ |
| Frame geometries | 38 |
| Total lines of code | 106,000 |

---

## Project Status

Meridian is in active development. The core flight stack, sensor drivers, and ground control station are implemented and functionally complete. Hardware flight testing is the next milestone.

| Component | Lines | Status |
|-----------|-------|--------|
| EKF (24-state) | 4,800 | Complete |
| AHRS (multi-lane) | 3,200 | Complete |
| PID controllers | 2,100 | Complete |
| Navigation (L1, spline, S-curve) | 2,800 | Complete |
| Mission engine (55 commands) | 1,900 | Complete |
| Flight modes (46 total) | 3,400 | Complete |
| Motor mixing (38 geometries) | 1,600 | Complete |
| Sensor drivers (20+) | 5,200 | Complete |
| RC protocols (8 types) | 1,800 | Complete |
| MAVLink v2 (90+ messages) | 2,200 | Complete |
| MNP protocol | 1,400 | Complete |
| Parameter system | 1,500 | Complete |
| Binary logging | 1,100 | Complete |
| Geofencing | 800 | Complete |
| Failsafe system | 700 | Complete |
| STM32H7 platform | 3,600 | Complete |
| SITL platform | 1,400 | Complete |
| Browser GCS | 28,000 | Complete |
| Hardware flight test | — | Next milestone |
| Board support (430 targets) | — | In progress |

### Roadmap

1. **SITL integration testing** — Full flight simulation with GCS connected
2. **First hardware hover** — MatekH743, Stabilize mode, 60 seconds
3. **Mission flight** — 10-waypoint AUTO mission via GCS
4. **ArduPilot Python test compatibility** — Pass `tests1a` batch
5. **Board scaling** — 50 Tier 2 boards via TOML auto-generation
6. **WASM extensions** — User scripting runtime
7. **Community beta** — Open testing with selected operators

---

## ArduPilot Parity

Every algorithm in Meridian was ported from ArduPilot with surgical precision. The project maintains 17,697 lines of audit notes across 19 files (`docs/audit_*.md`), covering all 154 ArduPilot libraries.

The audit was conducted by analyzing the ArduPilot source code (1.5M lines across 154 libraries and 430 board definitions), identifying every algorithm, data structure, and edge case, then verifying that Meridian's Rust implementation handles each one correctly.

### Key Parity Points

- **EKF:** 24-state model matches AP_NavEKF3 state vector, covariance prediction, and measurement fusion
- **PID controllers:** Rate and angle loops match AC_AttitudeControl with identical gain scheduling
- **sqrt_controller:** Kinematic shaper matches `control.h` — this is the core of all ArduPilot position/velocity control
- **Motor mixing:** All 38 frame geometries produce identical thrust vectors
- **Failsafe:** RC loss, battery, GCS, EKF cascade matches ArduPilot priority ordering
- **SBUS parsing:** Both explicit failsafe flag AND implicit channel check (channels 1-4 <= 875us)
- **Compass calibration:** Levenberg-Marquardt sphere+ellipsoid fit with motor compensation
- **Mission protocol:** MISSION_COUNT/REQUEST_INT/ITEM_INT/ACK state machine with 8s timeout and retry

---

## Contributing

Meridian is a community project. We need your help to get to first flight and beyond.

**Read the full guide: [COMMUNITY.md](COMMUNITY.md)** — everything you need to know about forking, building, testing, and submitting PRs.

**Quick links:**
- [Open an issue](https://github.com/jeranaias/meridian/issues/new/choose) — bugs, features, board requests
- [Submit a PR](https://github.com/jeranaias/meridian/pulls) — code, docs, board configs, translations
- [GCS contributing guide](gcs/CONTRIBUTING.md) — zero-build philosophy and code style
- [Community guide](COMMUNITY.md) — full task list of what needs doing right now

**What we need most right now:**

| Area | What | Difficulty |
|------|------|-----------|
| **Testing** | Connect GCS to SITL, fly a mission, report bugs | Easy |
| **Board validation** | Test Tier 3 board TOMLs on real hardware | Medium |
| **Sensor drivers** | Test ICM-42688, BMP388, uBlox on dev boards | Medium |
| **GCS widgets** | New instruments, better mobile layout, video | Medium |
| **Translations** | i18n for the GCS (`locales/en.json` as template) | Easy |
| **Documentation** | Getting started guides, tutorials, architecture docs | Easy |
| **EKF tuning** | Real flight data for filter validation | Hard |
| **RC protocols** | Test CRSF/ELRS, SBUS, DSM with real receivers | Medium |

If you love the idea of a Rust-native autopilot, if you want to `cargo test` a single flight algorithm in isolation, if you've wanted a GCS that runs on your tablet without installing anything — **fork this repo and start building.**

---

## License

[MIT](gcs/LICENSE)

Use it. Fork it. Fly with it.
