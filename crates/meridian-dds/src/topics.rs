//! ROS2 topic names and QoS profile definitions.

/// Published topics (Meridian → ROS2 network).
pub mod publish {
    pub const IMU: &str = "meridian/imu/data";
    pub const NAV_SAT_FIX: &str = "meridian/global_position/global";
    pub const BATTERY: &str = "meridian/battery";
    pub const MAG: &str = "meridian/mag";
    pub const BARO: &str = "meridian/pressure";
    pub const POSE: &str = "meridian/local_position/pose";
    pub const TWIST: &str = "meridian/local_position/velocity";
    pub const ODOMETRY: &str = "meridian/local_position/odom";
    pub const RANGE: &str = "meridian/rangefinder";
    pub const VEHICLE_STATE: &str = "meridian/state";
    pub const MISSION_STATUS: &str = "meridian/mission/status";
}

/// Subscribed topics (ROS2 network → Meridian).
pub mod subscribe {
    pub const CMD_VEL: &str = "meridian/cmd_vel";
    pub const GOAL_POSE: &str = "meridian/goal_pose";
}

/// Service names.
pub mod service {
    pub const ARM: &str = "meridian/arm";
    pub const SET_MODE: &str = "meridian/set_mode";
    pub const TAKEOFF: &str = "meridian/takeoff";
}

/// DDS type names following ROS2 mangling convention.
/// Format: `<package>::msg::dds_::<Type>_`
pub mod type_names {
    pub const IMU: &str = "sensor_msgs::msg::dds_::Imu_";
    pub const NAV_SAT_FIX: &str = "sensor_msgs::msg::dds_::NavSatFix_";
    pub const BATTERY_STATE: &str = "sensor_msgs::msg::dds_::BatteryState_";
    pub const MAGNETIC_FIELD: &str = "sensor_msgs::msg::dds_::MagneticField_";
    pub const FLUID_PRESSURE: &str = "sensor_msgs::msg::dds_::FluidPressure_";
    pub const RANGE: &str = "sensor_msgs::msg::dds_::Range_";
    pub const POSE_STAMPED: &str = "geometry_msgs::msg::dds_::PoseStamped_";
    pub const TWIST_STAMPED: &str = "geometry_msgs::msg::dds_::TwistStamped_";
    pub const ODOMETRY: &str = "nav_msgs::msg::dds_::Odometry_";
    pub const VEHICLE_STATE: &str = "meridian_msgs::msg::dds_::VehicleState_";
    pub const MISSION_STATUS: &str = "meridian_msgs::msg::dds_::MissionStatus_";
}

/// Default publish rates (Hz).
pub mod rates {
    pub const IMU: u32 = 50;
    pub const NAV_SAT_FIX: u32 = 5;
    pub const BATTERY: u32 = 1;
    pub const MAG: u32 = 10;
    pub const BARO: u32 = 10;
    pub const POSE: u32 = 30;
    pub const TWIST: u32 = 30;
    pub const ODOMETRY: u32 = 30;
    pub const RANGE: u32 = 10;
    pub const VEHICLE_STATE: u32 = 2;
    pub const MISSION_STATUS: u32 = 1;
}
