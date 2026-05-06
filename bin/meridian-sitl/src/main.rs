//! Meridian SITL binary — software-in-the-loop simulator.
//!
//! Boots the full Meridian flight stack, runs physics at 1200 Hz,
//! the control loop at 400 Hz, and serves MAVLink over TCP to QGroundControl.
//!
//! Usage: `cargo run -p meridian-sitl-bin`
//! Connect QGC to TCP 127.0.0.1:5760.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

#[cfg(feature = "dds")]
mod dds;

#[cfg(not(feature = "dds"))]
mod dds {
    //! Stub when DDS feature is disabled.
    use meridian_math::Quaternion;
    use meridian_math::frames::{Body, NED};
    use meridian_math::Vec3;
    use meridian_math::geodetic::LatLonAlt;
    use meridian_types::messages::GnssFixType;
    use meridian_types::vehicle::FlightModeId;

    pub struct DdsHandle;
    impl DdsHandle {
        pub fn send(&self, _telem: DdsTelemetry) {}
    }

    pub fn start() -> DdsHandle { DdsHandle }

    #[allow(dead_code)]
    pub struct DdsTelemetry {
        pub boot_secs: f64,
        pub ekf: EkfSnapshot,
        pub imu: ImuSnapshot,
        pub gps: Option<GpsSnapshot>,
        pub mag: Option<MagSnapshot>,
        pub baro: Option<BaroSnapshot>,
        pub armed: bool,
        pub mode: FlightModeId,
        pub battery_voltage: f32,
        pub battery_pct: f32,
        pub throttle: f32,
        pub num_sats: u8,
        pub gps_fix: GnssFixType,
    }

    #[allow(dead_code)]
    #[derive(Clone, Copy)]
    pub struct EkfSnapshot {
        pub attitude: Quaternion,
        pub velocity_ned: Vec3<NED>,
        pub position_ned: Vec3<NED>,
        pub gyro_bias: Vec3<Body>,
        pub accel_bias: Vec3<Body>,
        pub origin: LatLonAlt,
        pub healthy: bool,
    }

    #[allow(dead_code)]
    #[derive(Clone, Copy)]
    pub struct ImuSnapshot {
        pub accel: Vec3<Body>,
        pub gyro: Vec3<Body>,
        pub temperature: f32,
    }

    #[allow(dead_code)]
    #[derive(Clone, Copy)]
    pub struct GpsSnapshot {
        pub fix_type: GnssFixType,
        pub position: LatLonAlt,
        pub velocity_ned: Vec3<NED>,
        pub horizontal_accuracy: f32,
        pub vertical_accuracy: f32,
        pub num_sats: u8,
    }

    #[allow(dead_code)]
    #[derive(Clone, Copy)]
    pub struct MagSnapshot { pub field: Vec3<Body> }

    #[allow(dead_code)]
    #[derive(Clone, Copy)]
    pub struct BaroSnapshot {
        pub pressure_pa: f32,
        pub temperature: f32,
        pub altitude_m: f32,
    }
}

use meridian_control::attitude_controller::AttitudeController;
use meridian_control::position_controller::PositionController;
use meridian_control::rate_controller::RateController;
use meridian_ekf::core::EkfCore;
use meridian_math::geodetic::LatLonAlt;
use meridian_math::frames::NED;
use meridian_math::{Quaternion, Vec3};
use meridian_mavlink::adapter::{
    MAV_STATE_ACTIVE, MAV_STATE_STANDBY, MAV_TYPE_QUADROTOR,
};
use meridian_mavlink::server::{MavlinkServer, ServerAction, VehicleState};
use meridian_mixing::{Mixer, MixingMatrix, MAX_MOTORS};
use meridian_modes::mode_trait::{FlightMode, ModeInput, ModeOutput};
use meridian_modes::multirotor::MultirotorModes;
use meridian_params::ParamStore;
use meridian_sitl::physics::{PhysicsState, VehicleParams, PHYSICS_HZ};
use meridian_sitl::sensors::SensorSim;
use meridian_types::vehicle::FlightModeId;
use meridian_vehicle::VehiclePhysics;

// ─── Constants ───

/// Main loop rate (Hz). Controllers and EKF run at this rate.
const MAIN_HZ: u32 = 400;
/// Main loop timestep (seconds).
const MAIN_DT: f32 = 1.0 / MAIN_HZ as f32;
/// Physics substeps per main tick (1200 Hz / 400 Hz = 3).
const PHYSICS_SUBSTEPS: u32 = PHYSICS_HZ / MAIN_HZ;
/// Physics timestep (seconds).
const PHYSICS_DT: f32 = 1.0 / PHYSICS_HZ as f32;

/// TCP listen address.
const LISTEN_ADDR: &str = "0.0.0.0:5760";

/// Home position: 35.0N, 120.0W, ground level.
const HOME_LAT_DEG: f64 = 35.0;
const HOME_LON_DEG: f64 = -120.0;
const HOME_ALT_M: f64 = 0.0;

/// GPS update interval (ticks). 10 Hz = every 40th tick at 400 Hz.
const GPS_INTERVAL: u32 = 40;
/// Barometer update interval (ticks). 50 Hz = every 8th tick.
const BARO_INTERVAL: u32 = 8;
/// Magnetometer update interval (ticks). 100 Hz = every 4th tick.
const MAG_INTERVAL: u32 = 4;

/// MAVLink TX buffer size (bytes).
const TX_BUF_SIZE: usize = 8192;
/// MAVLink RX buffer size (bytes).
const RX_BUF_SIZE: usize = 4096;

fn main() {
    println!("Meridian SITL v0.1.0");

    // ─── 1. Init params ───
    let mut params = ParamStore::new();
    params.register_defaults();

    // ─── 2. Init vehicle & physics ───
    let vehicle = VehiclePhysics::default_quad();
    let vehicle_params = VehicleParams::from_vehicle(&vehicle);
    let mut physics = PhysicsState::new();

    // ─── 3. Init sensors ───
    let origin = LatLonAlt::from_degrees(HOME_LAT_DEG, HOME_LON_DEG, HOME_ALT_M);
    let mut sensors = SensorSim::new(origin);

    // ─── 4. Init EKF ───
    let mut ekf = EkfCore::new(origin);

    // ─── 5. Init controllers ───
    let mut attitude_ctl = AttitudeController::new();
    let mut rate_ctl = RateController::new();
    let mut pos_ctl = PositionController::new();

    // ─── 6. Init mixer (quad-X) ───
    let mut mixer = Mixer::new(MixingMatrix::quad_x());

    // ─── 7. Init modes ───
    let mut modes = MultirotorModes::new();
    // Start in Stabilize (safest default)
    let initial_input = make_mode_input(&physics, &ekf, 0.0);
    modes.set_mode(FlightModeId::Stabilize, &initial_input);

    // ─── 8. Init MAVLink server ───
    let mut mav_server = MavlinkServer::new(1, 1);

    // ─── 8b. Start DDS publisher thread ───
    let dds_handle = dds::start();
    println!("DDS publisher thread started (ROS2 topics)");

    // ─── 9. Start TCP listener ───
    let listener = TcpListener::bind(LISTEN_ADDR).expect("Failed to bind TCP listener");
    listener
        .set_nonblocking(true)
        .expect("Failed to set listener non-blocking");
    println!("Listening on {} (MAVLink)", LISTEN_ADDR);
    println!("Waiting for GCS connection...");

    // ─── State ───
    let mut client: Option<TcpStream> = None;
    let mut armed = false;
    let mut motor_outputs = [0.0f32; MAX_MOTORS];
    let mut tick: u64 = 0;
    let mut throttle_out: f32 = 0.0;
    let boot_time = Instant::now();
    let mut rx_buf = [0u8; RX_BUF_SIZE];
    let mut last_gps: Option<dds::GpsSnapshot> = None;
    let mut last_mag: Option<dds::MagSnapshot> = None;
    let mut last_baro: Option<dds::BaroSnapshot> = None;

    // ─── 10. Main loop at 400 Hz ───
    loop {
        let tick_start = Instant::now();
        let boot_ms = boot_time.elapsed().as_millis() as u64;

        // ── a. Check for new TCP connection (non-blocking accept) ──
        if client.is_none() {
            match listener.accept() {
                Ok((stream, addr)) => {
                    stream
                        .set_nonblocking(true)
                        .expect("Failed to set stream non-blocking");
                    stream
                        .set_nodelay(true)
                        .ok();
                    println!("[Connected] GCS from {}", addr);
                    client = Some(stream);
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => eprintln!("Accept error: {}", e),
            }
        }

        // ── b. Read incoming MAVLink from TCP (non-blocking) ──
        let mut inbound_bytes: usize = 0;
        if let Some(ref mut stream) = client {
            match stream.read(&mut rx_buf) {
                Ok(0) => {
                    // Client disconnected
                    println!("[Disconnected] GCS");
                    client = None;
                }
                Ok(n) => {
                    inbound_bytes = n;
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(_) => {
                    println!("[Disconnected] GCS (error)");
                    client = None;
                }
            }
        }

        // ── c. Parse and dispatch inbound commands ──
        if inbound_bytes > 0 {
            let cmds = mav_server
                .adapter_mut()
                .feed_bytes(&rx_buf[..inbound_bytes]);

            let mut tx_reply = [0u8; TX_BUF_SIZE];
            let mut reply_len = 0;
            let vs = build_vehicle_state(
                &physics, &ekf, armed, &modes, throttle_out, boot_ms as u32, origin,
            );

            for cmd in cmds.iter() {
                let (n, action) = mav_server.handle_message(
                    cmd,
                    boot_ms,
                    &vs,
                    &mut tx_reply[reply_len..],
                );
                reply_len += n;

                if let Some(action) = action {
                    dispatch_action(
                        &action,
                        &mut armed,
                        &mut modes,
                        &mut attitude_ctl,
                        &mut rate_ctl,
                        &mut mav_server,
                        &physics,
                        &ekf,
                        tick,
                    );
                }
            }

            // Send reply bytes
            if reply_len > 0 {
                if let Some(ref mut stream) = client {
                    let _ = stream.write_all(&tx_reply[..reply_len]);
                }
            }
        }

        // ── d. Generate simulated sensor data from physics state ──
        let imu = sensors.sample_imu(&physics, &vehicle_params, &motor_outputs, MAIN_DT);

        // ── e. Feed sensors to EKF ──
        ekf.predict(&imu);

        if tick % GPS_INTERVAL as u64 == 0 {
            let gps = sensors.sample_gps(&physics);
            last_gps = Some(dds::GpsSnapshot {
                fix_type: gps.fix_type,
                position: gps.position,
                velocity_ned: gps.velocity_ned,
                horizontal_accuracy: gps.horizontal_accuracy,
                vertical_accuracy: gps.vertical_accuracy,
                num_sats: gps.num_sats,
            });
            ekf.fuse_gps(&gps);
        }
        if tick % BARO_INTERVAL as u64 == 0 {
            let baro = sensors.sample_baro(&physics);
            last_baro = Some(dds::BaroSnapshot {
                pressure_pa: baro.pressure_pa,
                temperature: baro.temperature,
                altitude_m: baro.altitude_m,
            });
            ekf.fuse_baro(&baro);
        }
        if tick % MAG_INTERVAL as u64 == 0 {
            let mag = sensors.sample_mag(&physics);
            last_mag = Some(dds::MagSnapshot { field: mag.field });
            ekf.fuse_mag(&mag);
        }

        // ── f. Get EKF state output ──
        let ekf_state = ekf.output_state();

        // ── g-k. Run flight mode → position → attitude → rate → mixer ──
        if armed {
            let time_since_arm = tick as f32 * MAIN_DT;
            let mode_input = make_mode_input(&physics, &ekf, time_since_arm);
            let mode_output = modes.update(&mode_input, MAIN_DT);

            let (target_quat, thr) = match mode_output {
                ModeOutput::PositionTarget {
                    position,
                    velocity,
                    yaw: _,
                } => {
                    // h. Run position controller
                    let current_pos = ekf_state.position_ned;
                    let current_vel = ekf_state.velocity_ned;
                    let (_, _, current_yaw) = ekf_state.attitude.to_euler();
                    let pos_out = pos_ctl.update(
                        &position,
                        &velocity,
                        &current_pos,
                        &current_vel,
                        current_yaw,
                        MAIN_DT,
                    );
                    (pos_out.target_quat, pos_out.throttle)
                }
                ModeOutput::AttitudeTarget { quaternion, throttle } => {
                    (quaternion, throttle)
                }
                ModeOutput::Idle | _ => {
                    (Quaternion::identity(), 0.0)
                }
            };

            // i. Run attitude controller → rate targets
            let rate_target = attitude_ctl.update(
                &target_quat,
                &ekf_state.attitude,
                MAIN_DT,
            );

            // j. Run rate controller → axis commands
            // T3 fix: Use EKF-corrected gyro through sensor chain, not raw physics state.
            // Raw physics bypasses the sensor→EKF pipeline and would mask sensor bugs in SITL.
            let corrected_gyro = meridian_math::Vec3::<meridian_math::frames::Body>::new(
                imu.gyro.x - ekf_state.gyro_bias.x,
                imu.gyro.y - ekf_state.gyro_bias.y,
                imu.gyro.z - ekf_state.gyro_bias.z,
            );
            let (roll_cmd, pitch_cmd, yaw_cmd) =
                rate_ctl.update_simple(&rate_target, &corrected_gyro, MAIN_DT);

            // k. Run mixer → motor outputs
            throttle_out = thr;
            motor_outputs = mixer.mix(roll_cmd, pitch_cmd, yaw_cmd, thr);
        } else {
            // Disarmed: zero motors
            motor_outputs = [0.0f32; MAX_MOTORS];
            throttle_out = 0.0;
            // Keep controllers reset
            attitude_ctl.reset();
            rate_ctl.reset();
        }

        // ── l. Step physics with motor outputs (3 substeps) ──
        for _ in 0..PHYSICS_SUBSTEPS {
            meridian_sitl::physics::step(
                &mut physics,
                &motor_outputs,
                &vehicle_params,
                PHYSICS_DT,
            );
        }

        // ── m-n. Update MAVLink server and send telemetry ──
        if client.is_some() {
            let vs = build_vehicle_state(
                &physics, &ekf, armed, &modes, throttle_out, boot_ms as u32, origin,
            );
            let mut tx_buf = [0u8; TX_BUF_SIZE];
            let n = mav_server.update(boot_ms, &vs, &mut tx_buf);

            if n > 0 {
                if let Some(ref mut stream) = client {
                    if stream.write_all(&tx_buf[..n]).is_err() {
                        println!("[Disconnected] GCS (write error)");
                        client = None;
                    }
                }
            }
        }

        // ── n2. Send telemetry to DDS thread ──
        {
            let ekf_out = ekf.output_state();
            dds_handle.send(dds::DdsTelemetry {
                boot_secs: boot_time.elapsed().as_secs_f64(),
                ekf: dds::EkfSnapshot {
                    attitude: ekf_out.attitude,
                    velocity_ned: ekf_out.velocity_ned,
                    position_ned: ekf_out.position_ned,
                    gyro_bias: ekf_out.gyro_bias,
                    accel_bias: ekf_out.accel_bias,
                    origin,
                    healthy: ekf_out.healthy,
                },
                imu: dds::ImuSnapshot {
                    accel: imu.accel,
                    gyro: imu.gyro,
                    temperature: imu.temperature,
                },
                gps: last_gps,
                mag: last_mag,
                baro: last_baro,
                armed,
                mode: modes.active_mode(),
                battery_voltage: 16.8,
                battery_pct: 100.0,
                throttle: throttle_out,
                num_sats: 14,
                gps_fix: meridian_types::messages::GnssFixType::Fix3D,
            });
        }

        // ── o. Log flight data (periodic console print) ──
        if tick % (MAIN_HZ as u64 * 2) == 0 {
            let (roll, pitch, yaw) = ekf.output_state().attitude.to_euler();
            let alt = physics.altitude();
            let mode_name = modes.name();
            let armed_str = if armed { "ARMED" } else { "DISARMED" };
            eprintln!(
                "[{:>8.1}s] {} {} | alt={:.1}m rpy=({:.1},{:.1},{:.1})° vz={:.2} m/s",
                tick as f64 / MAIN_HZ as f64,
                armed_str,
                mode_name,
                alt,
                roll.to_degrees(),
                pitch.to_degrees(),
                yaw.to_degrees(),
                physics.velocity.z,
            );
        }

        tick += 1;

        // ── p. Sleep remainder of 2.5ms tick ──
        let elapsed = tick_start.elapsed();
        let tick_duration = Duration::from_micros(2500);
        if elapsed < tick_duration {
            std::thread::sleep(tick_duration - elapsed);
        }
    }
}

// ─── Helper: build VehicleState for MAVLink telemetry ───

fn build_vehicle_state(
    physics: &PhysicsState,
    ekf: &EkfCore,
    armed: bool,
    modes: &MultirotorModes,
    throttle: f32,
    boot_ms: u32,
    origin: LatLonAlt,
) -> VehicleState {
    let ekf_out = ekf.output_state();
    let (roll, pitch, yaw) = ekf_out.attitude.to_euler();
    let lla = ekf.position_lla();

    // Convert lat/lon to 1e7 integer format
    let lat_e7 = (lla.lat.to_degrees() * 1e7) as i32;
    let lon_e7 = (lla.lon.to_degrees() * 1e7) as i32;
    let alt_mm = (lla.alt * 1000.0) as i32;
    let relative_alt_mm = (physics.altitude() * 1000.0) as i32;

    // Velocity in cm/s (MAVLink uses cm/s for GLOBAL_POSITION_INT)
    let vx = (ekf_out.velocity_ned.x * 100.0) as i16;
    let vy = (ekf_out.velocity_ned.y * 100.0) as i16;
    let vz = (ekf_out.velocity_ned.z * 100.0) as i16;

    // Heading in centidegrees [0, 35999]
    let heading_deg = yaw.to_degrees();
    let heading_cdeg = if heading_deg < 0.0 {
        ((heading_deg + 360.0) * 100.0) as u16
    } else {
        (heading_deg * 100.0) as u16
    };

    // Groundspeed
    let gs = f32::sqrt(
        ekf_out.velocity_ned.x * ekf_out.velocity_ned.x
            + ekf_out.velocity_ned.y * ekf_out.velocity_ned.y,
    );

    // Custom mode number for current flight mode
    let custom_mode = modes.active_mode().to_number() as u32;

    // Home position
    let home_lat_e7 = (origin.lat.to_degrees() * 1e7) as i32;
    let home_lon_e7 = (origin.lon.to_degrees() * 1e7) as i32;
    let home_alt_mm = (origin.alt * 1000.0) as i32;

    VehicleState {
        armed,
        custom_mode,
        mav_type: MAV_TYPE_QUADROTOR,
        system_status: if armed { MAV_STATE_ACTIVE } else { MAV_STATE_STANDBY },
        roll,
        pitch,
        yaw,
        rollspeed: physics.gyro.x,
        pitchspeed: physics.gyro.y,
        yawspeed: physics.gyro.z,
        lat_e7,
        lon_e7,
        alt_mm,
        relative_alt_mm,
        vx,
        vy,
        vz,
        heading: heading_cdeg,
        airspeed: gs,
        groundspeed: gs,
        alt: physics.altitude(),
        climb: -ekf_out.velocity_ned.z,
        throttle: (throttle * 100.0) as u16,
        voltage_mv: 16800, // 4S LiPo at 4.2V/cell
        current_ca: 0,
        remaining_pct: 100,
        sensor_health: meridian_mavlink::adapter::SensorHealth::all_healthy(),
        boot_time_ms: boot_ms,
        home_lat_e7,
        home_lon_e7,
        home_alt_mm,
        mission_current_seq: 0,
    }
}

// ─── Helper: build ModeInput from physics and EKF state ───

fn make_mode_input(physics: &PhysicsState, ekf: &EkfCore, time_since_arm: f32) -> ModeInput {
    let ekf_out = ekf.output_state();
    let (_, _, yaw) = ekf_out.attitude.to_euler();
    let alt = physics.altitude();
    let gs = f32::sqrt(
        ekf_out.velocity_ned.x * ekf_out.velocity_ned.x
            + ekf_out.velocity_ned.y * ekf_out.velocity_ned.y,
    );
    let gc = f32::atan2(ekf_out.velocity_ned.y, ekf_out.velocity_ned.x);

    ModeInput {
        attitude: ekf_out.attitude,
        position: ekf_out.position_ned,
        velocity: ekf_out.velocity_ned,
        gyro: meridian_math::Vec3::new(physics.gyro.x, physics.gyro.y, physics.gyro.z),
        altitude: alt,
        yaw,
        ground_course: gc,
        ground_speed: gs,
        airspeed: physics.velocity.length(),
        home: Vec3::zero(),
        time_since_arm,
        // No RC input in SITL — sticks centered, throttle at 0
        rc_roll: 0.0,
        rc_pitch: 0.0,
        rc_yaw: 0.0,
        rc_throttle: 0.0,
        // Land detector: default to not-landed conditions in SITL
        throttle_at_min: false,
        throttle_mix_at_min: false,
        small_angle_error: true,
        accel_near_1g: true,
        rangefinder_near_ground: true,
    }
}

// ─── Helper: dispatch MAVLink ServerAction ───

fn dispatch_action(
    action: &ServerAction,
    armed: &mut bool,
    modes: &mut MultirotorModes,
    attitude_ctl: &mut AttitudeController,
    rate_ctl: &mut RateController,
    mav_server: &mut MavlinkServer,
    physics: &PhysicsState,
    ekf: &EkfCore,
    tick: u64,
) {
    match action {
        ServerAction::Arm => {
            *armed = true;
            attitude_ctl.reset();
            rate_ctl.reset();
            mav_server.queue_statustext(6, "ARMED");
            eprintln!("[Action] Armed");
        }
        ServerAction::Disarm => {
            *armed = false;
            mav_server.queue_statustext(6, "DISARMED");
            eprintln!("[Action] Disarmed");
        }
        ServerAction::SetMode(mode_num) => {
            let mode_id = FlightModeId::from_number(*mode_num as u8);
            let input = make_mode_input(physics, ekf, tick as f32 * MAIN_DT);
            modes.set_mode(mode_id, &input);
            mav_server.queue_statustext(6, mode_id.name());
            eprintln!("[Action] Mode → {}", mode_id.name());
        }
        ServerAction::Takeoff(alt) => {
            // Switch to Guided mode and set position target above home
            let input = make_mode_input(physics, ekf, tick as f32 * MAIN_DT);
            modes.set_mode(FlightModeId::Guided, &input);
            let target = Vec3::<NED>::new(0.0, 0.0, -*alt); // NED: negative Z = up
            modes.set_guided_target(target, 0.0);
            mav_server.queue_statustext(6, "Taking off");
            eprintln!("[Action] Takeoff to {}m", alt);
        }
        ServerAction::Land => {
            let input = make_mode_input(physics, ekf, tick as f32 * MAIN_DT);
            modes.set_mode(FlightModeId::Land, &input);
            mav_server.queue_statustext(6, "Landing");
            eprintln!("[Action] Land");
        }
        ServerAction::Rtl => {
            let input = make_mode_input(physics, ekf, tick as f32 * MAIN_DT);
            modes.set_mode(FlightModeId::RTL, &input);
            mav_server.queue_statustext(6, "RTL");
            eprintln!("[Action] RTL");
        }
        ServerAction::SetParam { name, value } => {
            if let Some(_) = params_find_mut(&name.as_str()) {
                eprintln!("[Action] Param {} = {}", name, value);
            } else {
                eprintln!("[Action] Param {} not found", name);
            }
        }
        ServerAction::RequestAutopilotVersion => {
            // Handled by the server — nothing extra needed
        }
        ServerAction::RequestMessage(_msg_id) => {
            // Handled by the server
        }
        ServerAction::MissionUploaded => {
            mav_server.queue_statustext(6, "Mission received");
            eprintln!("[Action] Mission uploaded ({} items)", mav_server.mission_items().len());
        }
        ServerAction::MissionCleared => {
            mav_server.queue_statustext(6, "Mission cleared");
            eprintln!("[Action] Mission cleared");
        }
        ServerAction::MotorTest { .. } => {
            eprintln!("[Action] Motor test (SITL stub)");
        }
        ServerAction::SetMessageInterval { msg_id, interval_us } => {
            eprintln!("[Action] SetMessageInterval msg_id={} interval={}us", msg_id, interval_us);
        }
    }
}

/// Stub for param lookup — the ParamStore is not easily accessible from this
/// dispatch function, so we just log it. Full param integration would pass
/// the store through or use a global.
fn params_find_mut(_name: &str) -> Option<()> {
    // In a full implementation, this would look up the param in the ParamStore.
    // For now, the MAVLink server handles param streaming from its internal table.
    Some(())
}
