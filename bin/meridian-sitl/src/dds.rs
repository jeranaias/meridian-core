//! DDS/ROS2 telemetry publisher for SITL.
//!
//! Runs in a dedicated thread, receives telemetry snapshots from the main
//! 400 Hz loop, and publishes standard ROS2 topics via ros2-client (pure
//! Rust DDS — no ROS2 installation needed).

use std::sync::mpsc;
use std::thread;
use std::time::Instant as StdInstant;

use meridian_dds::cdr::{CdrSerialize, CdrWriter, CDR_LE_HEADER};
use meridian_dds::ros2_msgs::*;
use meridian_dds::convert;
use meridian_dds::topics;
use meridian_math::Quaternion;
use meridian_math::frames::{Body, NED};
use meridian_math::Vec3;
use meridian_math::geodetic::LatLonAlt;
use meridian_types::messages::{
    ImuSample, GnssPosition, GnssFixType, MagField, BaroPressure,
};
use meridian_types::vehicle::FlightModeId;
use meridian_types::time::Instant as MInstant;

/// Telemetry snapshot sent from the main loop to the DDS thread.
#[derive(Clone)]
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

#[derive(Clone, Copy)]
pub struct ImuSnapshot {
    pub accel: Vec3<Body>,
    pub gyro: Vec3<Body>,
    pub temperature: f32,
}

#[derive(Clone, Copy)]
pub struct GpsSnapshot {
    pub fix_type: GnssFixType,
    pub position: LatLonAlt,
    pub velocity_ned: Vec3<NED>,
    pub horizontal_accuracy: f32,
    pub vertical_accuracy: f32,
    pub num_sats: u8,
}

#[derive(Clone, Copy)]
pub struct MagSnapshot {
    pub field: Vec3<Body>,
}

#[derive(Clone, Copy)]
pub struct BaroSnapshot {
    pub pressure_pa: f32,
    pub temperature: f32,
    pub altitude_m: f32,
}

/// Handle to send telemetry to the DDS thread.
pub struct DdsHandle {
    tx: mpsc::SyncSender<DdsTelemetry>,
}

impl DdsHandle {
    pub fn send(&self, telem: DdsTelemetry) {
        let _ = self.tx.try_send(telem);
    }
}

struct RateLimiter {
    interval_secs: f64,
    last: f64,
}

impl RateLimiter {
    fn new(hz: u32) -> Self {
        Self { interval_secs: 1.0 / hz as f64, last: 0.0 }
    }
    fn check(&mut self, now: f64) -> bool {
        if now - self.last >= self.interval_secs {
            self.last = now;
            true
        } else {
            false
        }
    }
}

fn to_ros2_time(boot_secs: f64, epoch_offset: f64) -> Time {
    let wall = epoch_offset + boot_secs;
    let sec = wall as i32;
    let nanosec = ((wall - sec as f64) * 1e9) as u32;
    Time { sec, nanosec }
}

/// Serialize a CdrSerialize type into a CDR-encoded Vec<u8> with encapsulation header.
fn to_cdr_bytes<T: CdrSerialize>(msg: &T) -> Vec<u8> {
    let mut buf = vec![0u8; 4096];
    buf[..4].copy_from_slice(&CDR_LE_HEADER);
    let n = {
        let mut w = CdrWriter::new(&mut buf[4..]);
        if msg.cdr_serialize(&mut w).is_err() { return vec![]; }
        w.position()
    };
    buf.truncate(4 + n);
    buf
}

/// Start the DDS publisher thread.
pub fn start() -> DdsHandle {
    let (tx, rx) = mpsc::sync_channel::<DdsTelemetry>(16);

    thread::Builder::new()
        .name("dds-publisher".into())
        .spawn(move || {
            if let Err(e) = dds_thread(rx) {
                eprintln!("[DDS] Thread error: {}", e);
            }
        })
        .expect("Failed to spawn DDS thread");

    DdsHandle { tx }
}

fn dds_thread(rx: mpsc::Receiver<DdsTelemetry>) -> Result<(), Box<dyn std::error::Error>> {
    use ros2_client::*;
    use rustdds::policy::*;
    use rustdds::QosPolicies;

    eprintln!("[DDS] Creating ROS2 context (domain 0)...");

    let ctx = Context::new()?;
    let mut node = ctx.new_node(
        NodeName::new("/meridian", "sitl").unwrap(),
        NodeOptions::default(),
    )?;

    // QoS profiles
    let sensor_qos = QosPolicies::builder()
        .reliability(Reliability::BestEffort)
        .durability(Durability::Volatile)
        .history(History::KeepLast { depth: 5 })
        .build();

    let state_qos = QosPolicies::builder()
        .reliability(Reliability::Reliable {
            max_blocking_time: rustdds::Duration::from_millis(100),
        })
        .durability(Durability::TransientLocal)
        .history(History::KeepLast { depth: 1 })
        .build();

    // Create publishers for each topic — using raw CDR bytes
    // ros2-client MessageTypeName must match ROS2 DDS type naming
    let imu_topic = node.create_topic(
        &Name::new("/meridian/imu", "data").unwrap(),
        MessageTypeName::new("sensor_msgs::msg::dds_", "Imu_"),
        &sensor_qos,
    )?;
    let imu_pub = node.create_publisher::<Vec<u8>>(&imu_topic, None)?;

    let fix_topic = node.create_topic(
        &Name::new("/meridian/global_position", "global").unwrap(),
        MessageTypeName::new("sensor_msgs::msg::dds_", "NavSatFix_"),
        &sensor_qos,
    )?;
    let fix_pub = node.create_publisher::<Vec<u8>>(&fix_topic, None)?;

    let pose_topic = node.create_topic(
        &Name::new("/meridian/local_position", "pose").unwrap(),
        MessageTypeName::new("geometry_msgs::msg::dds_", "PoseStamped_"),
        &sensor_qos,
    )?;
    let pose_pub = node.create_publisher::<Vec<u8>>(&pose_topic, None)?;

    let twist_topic = node.create_topic(
        &Name::new("/meridian/local_position", "velocity").unwrap(),
        MessageTypeName::new("geometry_msgs::msg::dds_", "TwistStamped_"),
        &sensor_qos,
    )?;
    let twist_pub = node.create_publisher::<Vec<u8>>(&twist_topic, None)?;

    let odom_topic = node.create_topic(
        &Name::new("/meridian", "odom").unwrap(),
        MessageTypeName::new("nav_msgs::msg::dds_", "Odometry_"),
        &sensor_qos,
    )?;
    let odom_pub = node.create_publisher::<Vec<u8>>(&odom_topic, None)?;

    let bat_topic = node.create_topic(
        &Name::new("/meridian", "battery").unwrap(),
        MessageTypeName::new("sensor_msgs::msg::dds_", "BatteryState_"),
        &state_qos,
    )?;
    let bat_pub = node.create_publisher::<Vec<u8>>(&bat_topic, None)?;

    let state_topic = node.create_topic(
        &Name::new("/meridian", "state").unwrap(),
        MessageTypeName::new("meridian_msgs::msg::dds_", "VehicleState_"),
        &state_qos,
    )?;
    let state_pub = node.create_publisher::<Vec<u8>>(&state_topic, None)?;

    eprintln!("[DDS] ROS2 node ready — 7 publishers active");
    eprintln!("[DDS] Topics: /meridian/imu/data, /meridian/global_position/global,");
    eprintln!("[DDS]         /meridian/local_position/pose, /meridian/local_position/velocity,");
    eprintln!("[DDS]         /meridian/odom, /meridian/battery, /meridian/state");

    let epoch_offset = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64();

    let mut rate_imu = RateLimiter::new(topics::rates::IMU);
    let mut rate_pose = RateLimiter::new(topics::rates::POSE);
    let mut rate_twist = RateLimiter::new(topics::rates::TWIST);
    let mut rate_odom = RateLimiter::new(topics::rates::ODOMETRY);
    let mut rate_gps = RateLimiter::new(topics::rates::NAV_SAT_FIX);
    let mut rate_bat = RateLimiter::new(topics::rates::BATTERY);
    let mut rate_state = RateLimiter::new(topics::rates::VEHICLE_STATE);

    let mut msg_count: u64 = 0;
    let start = StdInstant::now();

    while let Ok(telem) = rx.recv() {
        let stamp = to_ros2_time(telem.boot_secs, epoch_offset);
        let now = telem.boot_secs;

        // ── IMU at 50 Hz ──
        if rate_imu.check(now) {
            let imu_sample = ImuSample {
                timestamp: MInstant::from_micros(0),
                imu_index: 0,
                accel: telem.imu.accel,
                gyro: telem.imu.gyro,
                temperature: telem.imu.temperature,
            };
            let ros_imu = convert::imu_to_ros2(&imu_sample, stamp);
            let bytes = to_cdr_bytes(&ros_imu);
            let _ = imu_pub.publish(bytes);
            msg_count += 1;
        }

        // ── PoseStamped at 30 Hz ──
        if rate_pose.check(now) {
            let ekf_state = ekf_snapshot_to_state(&telem.ekf);
            let ros_pose = convert::ekf_to_pose_stamped(&ekf_state, stamp);
            let _ = pose_pub.publish(to_cdr_bytes(&ros_pose));
            msg_count += 1;
        }

        // ── TwistStamped at 30 Hz ──
        if rate_twist.check(now) {
            let ekf_state = ekf_snapshot_to_state(&telem.ekf);
            let ros_twist = convert::ekf_to_twist_stamped(&ekf_state, stamp);
            let _ = twist_pub.publish(to_cdr_bytes(&ros_twist));
            msg_count += 1;
        }

        // ── Odometry at 30 Hz ──
        if rate_odom.check(now) {
            let ekf_state = ekf_snapshot_to_state(&telem.ekf);
            let ros_odom = convert::ekf_to_odometry(&ekf_state, stamp);
            let _ = odom_pub.publish(to_cdr_bytes(&ros_odom));
            msg_count += 1;
        }

        // ── NavSatFix at 5 Hz ──
        if rate_gps.check(now) {
            if let Some(gps) = &telem.gps {
                let gnss = GnssPosition {
                    timestamp: MInstant::from_micros(0),
                    fix_type: gps.fix_type,
                    position: gps.position,
                    velocity_ned: gps.velocity_ned,
                    horizontal_accuracy: gps.horizontal_accuracy,
                    vertical_accuracy: gps.vertical_accuracy,
                    speed_accuracy: 0.5,
                    num_sats: gps.num_sats,
                };
                let ros_fix = convert::gnss_to_nav_sat_fix(&gnss, stamp);
                let _ = fix_pub.publish(to_cdr_bytes(&ros_fix));
                msg_count += 1;
            }
        }

        // ── BatteryState at 1 Hz ──
        if rate_bat.check(now) {
            let bat = BatteryState {
                header: Header { stamp, frame_id: FrameId::from_str("battery") },
                voltage: telem.battery_voltage,
                temperature: f32::NAN,
                current: f32::NAN,
                charge: f32::NAN,
                capacity: f32::NAN,
                design_capacity: f32::NAN,
                percentage: telem.battery_pct / 100.0,
                power_supply_status: 2,
                power_supply_health: 1,
                power_supply_technology: 3,
                present: true,
            };
            let _ = bat_pub.publish(to_cdr_bytes(&bat));
            msg_count += 1;
        }

        // ── VehicleState at 2 Hz ──
        if rate_state.check(now) {
            let vs = MeridianVehicleState {
                header: Header { stamp, frame_id: FrameId::from_str("") },
                armed: telem.armed,
                mode: telem.mode.to_number(),
                ekf_healthy: telem.ekf.healthy,
                battery_voltage: telem.battery_voltage,
                battery_percentage: telem.battery_pct / 100.0,
                gps_fix_type: match telem.gps_fix {
                    GnssFixType::NoFix => 0,
                    GnssFixType::Fix2D => 2,
                    GnssFixType::Fix3D => 3,
                    GnssFixType::DGps => 4,
                    GnssFixType::RtkFloat => 5,
                    GnssFixType::RtkFixed => 6,
                },
                num_satellites: telem.num_sats,
            };
            let _ = state_pub.publish(to_cdr_bytes(&vs));
            msg_count += 1;
        }

        // Periodic stats
        let elapsed = start.elapsed().as_secs_f64();
        if msg_count > 0 && msg_count % 1000 == 0 {
            eprintln!("[DDS] {} msgs published ({:.0} msg/s)", msg_count, msg_count as f64 / elapsed);
        }
    }

    Ok(())
}

fn ekf_snapshot_to_state(snap: &EkfSnapshot) -> meridian_types::messages::EkfState {
    meridian_types::messages::EkfState {
        timestamp: MInstant::from_micros(0),
        attitude: snap.attitude,
        velocity_ned: snap.velocity_ned,
        position_ned: snap.position_ned,
        gyro_bias: snap.gyro_bias,
        accel_bias: snap.accel_bias,
        origin: snap.origin,
        healthy: snap.healthy,
    }
}
