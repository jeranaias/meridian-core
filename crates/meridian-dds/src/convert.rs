//! Bidirectional conversion between Meridian internal types and ROS2 messages.
//!
//! **NED ↔ ENU convention:**
//! Meridian uses NED (North-East-Down) internally, matching ArduPilot.
//! ROS2 convention is ENU (East-North-Up).
//!
//! Conversion: `x_enu = y_ned`, `y_enu = x_ned`, `z_enu = -z_ned`

use crate::ros2_msgs::*;
use meridian_types::messages::*;

// ─── Coordinate Frame Conversion ──────────────────────────────

/// Convert NED velocity/position to ENU.
fn ned_to_enu(n: f32, e: f32, d: f32) -> (f64, f64, f64) {
    (e as f64, n as f64, -(d as f64))
}

/// Convert ENU velocity/position to NED.
fn enu_to_ned(x: f64, y: f64, z: f64) -> (f32, f32, f32) {
    (y as f32, x as f32, -(z as f32))
}

/// Convert Meridian quaternion (NED frame) to ROS2 quaternion (ENU frame).
///
/// The NED→ENU rotation is a 180° rotation about the (1,1,0)/√2 axis
/// followed by renaming. For quaternions representing attitude:
///   q_enu = R_ned2enu * q_ned * R_ned2enu_inv
///
/// Simplified for the NED→ENU case (swap x↔y, negate z, keep w):
fn quat_ned_to_enu(q: &meridian_math::Quaternion) -> Quaternion {
    Quaternion {
        x: q.y as f64,
        y: q.x as f64,
        z: -(q.z as f64),
        w: q.w as f64,
    }
}

// ─── Meridian → ROS2 (Publish Direction) ──────────────────────

/// Convert `ImuSample` to `sensor_msgs/Imu`.
///
/// Note: orientation is left as identity (raw IMU has no attitude estimate).
/// The EKF state provides attitude via PoseStamped/Odometry.
pub fn imu_to_ros2(imu: &ImuSample, stamp: Time) -> Imu {
    // IMU body frame: accel and gyro stay in body frame (FRD → FLU for ROS)
    // ROS body frame convention: x=Forward, y=Left, z=Up
    // Meridian body frame: x=Forward, y=Right, z=Down
    // Conversion: x_flu = x_frd, y_flu = -y_frd, z_flu = -z_frd
    Imu {
        header: Header { stamp, frame_id: FrameId::from_str("imu_link") },
        orientation: Quaternion::default(), // identity — no attitude from raw IMU
        orientation_covariance: {
            let mut c = [0.0f64; 9];
            c[0] = -1.0; // -1 in [0] means orientation not provided
            c
        },
        angular_velocity: Vector3 {
            x: imu.gyro.x as f64,
            y: -(imu.gyro.y as f64),
            z: -(imu.gyro.z as f64),
        },
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: Vector3 {
            x: imu.accel.x as f64,
            y: -(imu.accel.y as f64),
            z: -(imu.accel.z as f64),
        },
        linear_acceleration_covariance: [0.0; 9],
    }
}

/// Convert `GnssPosition` to `sensor_msgs/NavSatFix`.
pub fn gnss_to_nav_sat_fix(gnss: &GnssPosition, stamp: Time) -> NavSatFix {
    let status = match gnss.fix_type {
        GnssFixType::NoFix => -1i8,
        GnssFixType::Fix2D | GnssFixType::Fix3D => 0,
        GnssFixType::DGps => 1,
        GnssFixType::RtkFloat | GnssFixType::RtkFixed => 2,
    };

    let h_var = (gnss.horizontal_accuracy * gnss.horizontal_accuracy) as f64;
    let v_var = (gnss.vertical_accuracy * gnss.vertical_accuracy) as f64;

    NavSatFix {
        header: Header { stamp, frame_id: FrameId::from_str("gps") },
        status: NavSatStatus { status, service: 1 }, // GPS
        latitude: gnss.position.lat * (180.0 / core::f64::consts::PI),
        longitude: gnss.position.lon * (180.0 / core::f64::consts::PI),
        altitude: gnss.position.alt,
        position_covariance: [
            h_var, 0.0, 0.0,
            0.0, h_var, 0.0,
            0.0, 0.0, v_var,
        ],
        position_covariance_type: 2, // DIAGONAL_KNOWN
    }
}

/// Convert `EkfState` to `geometry_msgs/PoseStamped` in ENU frame.
pub fn ekf_to_pose_stamped(ekf: &EkfState, stamp: Time) -> PoseStamped {
    let (x, y, z) = ned_to_enu(
        ekf.position_ned.x,
        ekf.position_ned.y,
        ekf.position_ned.z,
    );
    PoseStamped {
        header: Header { stamp, frame_id: FrameId::from_str("map") },
        pose: Pose {
            position: Point { x, y, z },
            orientation: quat_ned_to_enu(&ekf.attitude),
        },
    }
}

/// Convert `EkfState` to `geometry_msgs/TwistStamped` in ENU frame.
pub fn ekf_to_twist_stamped(ekf: &EkfState, stamp: Time) -> TwistStamped {
    let (vx, vy, vz) = ned_to_enu(
        ekf.velocity_ned.x,
        ekf.velocity_ned.y,
        ekf.velocity_ned.z,
    );
    TwistStamped {
        header: Header { stamp, frame_id: FrameId::from_str("map") },
        twist: Twist {
            linear: Vector3 { x: vx, y: vy, z: vz },
            angular: Vector3::default(), // angular velocity not in EkfState
        },
    }
}

/// Convert `EkfState` to `nav_msgs/Odometry` in ENU frame.
pub fn ekf_to_odometry(ekf: &EkfState, stamp: Time) -> Odometry {
    let (px, py, pz) = ned_to_enu(
        ekf.position_ned.x,
        ekf.position_ned.y,
        ekf.position_ned.z,
    );
    let (vx, vy, vz) = ned_to_enu(
        ekf.velocity_ned.x,
        ekf.velocity_ned.y,
        ekf.velocity_ned.z,
    );
    Odometry {
        header: Header { stamp, frame_id: FrameId::from_str("odom") },
        child_frame_id: FrameId::from_str("base_link"),
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point { x: px, y: py, z: pz },
                orientation: quat_ned_to_enu(&ekf.attitude),
            },
            covariance: [0.0; 36],
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: Vector3 { x: vx, y: vy, z: vz },
                angular: Vector3::default(),
            },
            covariance: [0.0; 36],
        },
    }
}

/// Convert `MagField` to `sensor_msgs/MagneticField`.
/// Meridian stores in gauss, ROS2 uses Tesla (1 gauss = 1e-4 Tesla).
pub fn mag_to_ros2(mag: &MagField, stamp: Time) -> MagneticField {
    const GAUSS_TO_TESLA: f64 = 1e-4;
    MagneticField {
        header: Header { stamp, frame_id: FrameId::from_str("imu_link") },
        magnetic_field: Vector3 {
            x: mag.field.x as f64 * GAUSS_TO_TESLA,
            y: -(mag.field.y as f64) * GAUSS_TO_TESLA,
            z: -(mag.field.z as f64) * GAUSS_TO_TESLA,
        },
        magnetic_field_covariance: [0.0; 9],
    }
}

/// Convert `BaroPressure` to `sensor_msgs/FluidPressure`.
pub fn baro_to_ros2(baro: &BaroPressure, stamp: Time) -> FluidPressure {
    FluidPressure {
        header: Header { stamp, frame_id: FrameId::from_str("baro") },
        fluid_pressure: baro.pressure_pa as f64,
        variance: 0.0,
    }
}

/// Convert `RangefinderReading` to `sensor_msgs/Range`.
pub fn rangefinder_to_ros2(rf: &RangefinderReading, stamp: Time) -> Range {
    Range {
        header: Header { stamp, frame_id: FrameId::from_str("rangefinder") },
        radiation_type: 1, // INFRARED (typical for flight controllers)
        field_of_view: 0.05,
        min_range: 0.05,
        max_range: 40.0,
        range: if rf.valid { rf.distance_m } else { f32::INFINITY },
    }
}

/// Convert `VehicleState` to `meridian_msgs/VehicleState`.
pub fn vehicle_state_to_ros2(vs: &VehicleState, stamp: Time, num_sats: u8) -> MeridianVehicleState {
    let gps_fix = match vs.gps_fix {
        GnssFixType::NoFix => 0,
        GnssFixType::Fix2D => 2,
        GnssFixType::Fix3D => 3,
        GnssFixType::DGps => 4,
        GnssFixType::RtkFloat => 5,
        GnssFixType::RtkFixed => 6,
    };
    MeridianVehicleState {
        header: Header { stamp, frame_id: FrameId::from_str("") },
        armed: vs.armed,
        mode: vs.mode.to_number(),
        ekf_healthy: vs.ekf_healthy,
        battery_voltage: vs.battery_voltage,
        battery_percentage: vs.battery_remaining_pct / 100.0,
        gps_fix_type: gps_fix,
        num_satellites: num_sats,
    }
}

/// Convert `VehicleState` battery info to `sensor_msgs/BatteryState`.
pub fn vehicle_to_battery_state(vs: &VehicleState, stamp: Time) -> BatteryState {
    BatteryState {
        header: Header { stamp, frame_id: FrameId::from_str("battery") },
        voltage: vs.battery_voltage,
        temperature: f32::NAN, // unknown from VehicleState
        current: f32::NAN,     // not in VehicleState
        charge: f32::NAN,
        capacity: f32::NAN,
        design_capacity: f32::NAN,
        percentage: vs.battery_remaining_pct / 100.0,
        power_supply_status: 2, // DISCHARGING
        power_supply_health: 1, // GOOD
        power_supply_technology: 3, // LIPO
        present: true,
    }
}

/// Convert `MissionStatus` to `meridian_msgs/MissionStatus`.
pub fn mission_status_to_ros2(ms: &MissionStatus, stamp: Time) -> MeridianMissionStatus {
    MeridianMissionStatus {
        header: Header { stamp, frame_id: FrameId::from_str("") },
        active: ms.active,
        current_item: ms.current_item,
        total_items: ms.total_items,
    }
}
