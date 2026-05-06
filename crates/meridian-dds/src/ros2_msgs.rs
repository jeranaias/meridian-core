//! ROS2 standard message types with CDR serialization.
//!
//! Each struct matches the exact field layout of its ROS2 `.msg` definition so that
//! CDR serialization produces wire-compatible bytes.  DDS type names follow the
//! ROS2 mangling convention: `sensor_msgs::msg::dds_::Imu_`.

use crate::cdr::{CdrSerialize, CdrDeserialize, CdrWriter, CdrReader, CdrError};

// ─── builtin_interfaces ───────────────────────────────────────

/// `builtin_interfaces/msg/Time`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

impl CdrSerialize for Time {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        w.write_i32(self.sec)?;
        w.write_u32(self.nanosec)
    }
}

impl CdrDeserialize for Time {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self {
            sec: r.read_i32()?,
            nanosec: r.read_u32()?,
        })
    }
}

// ─── std_msgs ─────────────────────────────────────────────────

/// `std_msgs/msg/Header`
///
/// The `frame_id` is stored as a fixed-capacity byte array to avoid heap
/// allocation.  Max 63 bytes + NUL.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: FrameId,
}

impl Default for Header {
    fn default() -> Self {
        Self { stamp: Time::default(), frame_id: FrameId::new() }
    }
}

/// Fixed-capacity string for `frame_id` fields (max 63 chars).
#[derive(Clone, Copy, PartialEq)]
pub struct FrameId {
    bytes: [u8; 64],
    len: u8,
}

impl FrameId {
    pub fn new() -> Self {
        Self { bytes: [0; 64], len: 0 }
    }

    pub fn from_str(s: &str) -> Self {
        let mut f = Self::new();
        let n = s.len().min(63);
        f.bytes[..n].copy_from_slice(&s.as_bytes()[..n]);
        f.len = n as u8;
        f
    }

    pub fn as_str(&self) -> &str {
        // Safety: we only store valid UTF-8 from from_str
        unsafe { core::str::from_utf8_unchecked(&self.bytes[..self.len as usize]) }
    }
}

impl core::fmt::Debug for FrameId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "\"{}\"", self.as_str())
    }
}

impl CdrSerialize for Header {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.stamp.cdr_serialize(w)?;
        w.write_string(self.frame_id.as_str())
    }
}

impl CdrDeserialize for Header {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let stamp = Time::cdr_deserialize(r)?;
        let bytes = r.read_string_bytes()?;
        let s = core::str::from_utf8(bytes).map_err(|_| CdrError::InvalidUtf8)?;
        Ok(Self { stamp, frame_id: FrameId::from_str(s) })
    }
}

// ─── geometry_msgs primitives ─────────────────────────────────

/// `geometry_msgs/msg/Vector3`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl CdrSerialize for Vector3 {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        w.write_f64(self.x)?;
        w.write_f64(self.y)?;
        w.write_f64(self.z)
    }
}

impl CdrDeserialize for Vector3 {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self { x: r.read_f64()?, y: r.read_f64()?, z: r.read_f64()? })
    }
}

/// `geometry_msgs/msg/Point`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl CdrSerialize for Point {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        w.write_f64(self.x)?;
        w.write_f64(self.y)?;
        w.write_f64(self.z)
    }
}

impl CdrDeserialize for Point {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self { x: r.read_f64()?, y: r.read_f64()?, z: r.read_f64()? })
    }
}

/// `geometry_msgs/msg/Quaternion`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
}

impl CdrSerialize for Quaternion {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        w.write_f64(self.x)?;
        w.write_f64(self.y)?;
        w.write_f64(self.z)?;
        w.write_f64(self.w)
    }
}

impl CdrDeserialize for Quaternion {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self { x: r.read_f64()?, y: r.read_f64()?, z: r.read_f64()?, w: r.read_f64()? })
    }
}

/// `geometry_msgs/msg/Pose`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

impl CdrSerialize for Pose {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.position.cdr_serialize(w)?;
        self.orientation.cdr_serialize(w)
    }
}

impl CdrDeserialize for Pose {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self { position: Point::cdr_deserialize(r)?, orientation: Quaternion::cdr_deserialize(r)? })
    }
}

/// `geometry_msgs/msg/Twist`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

impl CdrSerialize for Twist {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.linear.cdr_serialize(w)?;
        self.angular.cdr_serialize(w)
    }
}

impl CdrDeserialize for Twist {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self { linear: Vector3::cdr_deserialize(r)?, angular: Vector3::cdr_deserialize(r)? })
    }
}

// ─── geometry_msgs/msg/PoseStamped ────────────────────────────

/// `geometry_msgs/msg/PoseStamped`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

impl CdrSerialize for PoseStamped {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        self.pose.cdr_serialize(w)
    }
}

impl CdrDeserialize for PoseStamped {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self { header: Header::cdr_deserialize(r)?, pose: Pose::cdr_deserialize(r)? })
    }
}

// ─── geometry_msgs/msg/TwistStamped ───────────────────────────

/// `geometry_msgs/msg/TwistStamped`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TwistStamped {
    pub header: Header,
    pub twist: Twist,
}

impl CdrSerialize for TwistStamped {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        self.twist.cdr_serialize(w)
    }
}

impl CdrDeserialize for TwistStamped {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self { header: Header::cdr_deserialize(r)?, twist: Twist::cdr_deserialize(r)? })
    }
}

// ─── sensor_msgs/msg/Imu ──────────────────────────────────────

/// `sensor_msgs/msg/Imu`
///
/// 9-element covariance arrays are row-major 3x3 matrices.
/// Set all to 0 if covariance is unknown, or first element to -1 to indicate
/// the covariance is not applicable.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}

impl Default for Imu {
    fn default() -> Self {
        Self {
            header: Header::default(),
            orientation: Quaternion::default(),
            orientation_covariance: [0.0; 9],
            angular_velocity: Vector3::default(),
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: Vector3::default(),
            linear_acceleration_covariance: [0.0; 9],
        }
    }
}

impl CdrSerialize for Imu {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        self.orientation.cdr_serialize(w)?;
        w.write_f64_array(&self.orientation_covariance)?;
        self.angular_velocity.cdr_serialize(w)?;
        w.write_f64_array(&self.angular_velocity_covariance)?;
        self.linear_acceleration.cdr_serialize(w)?;
        w.write_f64_array(&self.linear_acceleration_covariance)
    }
}

impl CdrDeserialize for Imu {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let header = Header::cdr_deserialize(r)?;
        let orientation = Quaternion::cdr_deserialize(r)?;
        let mut orientation_covariance = [0.0f64; 9];
        r.read_f64_into(&mut orientation_covariance)?;
        let angular_velocity = Vector3::cdr_deserialize(r)?;
        let mut angular_velocity_covariance = [0.0f64; 9];
        r.read_f64_into(&mut angular_velocity_covariance)?;
        let linear_acceleration = Vector3::cdr_deserialize(r)?;
        let mut linear_acceleration_covariance = [0.0f64; 9];
        r.read_f64_into(&mut linear_acceleration_covariance)?;
        Ok(Self {
            header, orientation, orientation_covariance,
            angular_velocity, angular_velocity_covariance,
            linear_acceleration, linear_acceleration_covariance,
        })
    }
}

// ─── sensor_msgs/msg/NavSatFix ────────────────────────────────

/// `sensor_msgs/msg/NavSatStatus`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct NavSatStatus {
    pub status: i8,   // STATUS_NO_FIX=-1, FIX=0, SBAS=1, GBAS=2
    pub service: u16, // SERVICE_GPS=1, GLONASS=2, COMPASS=4, GALILEO=8
}

impl CdrSerialize for NavSatStatus {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        w.write_i8(self.status)?;
        w.write_u16(self.service)
    }
}

impl CdrDeserialize for NavSatStatus {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self { status: r.read_i8()?, service: r.read_u16()? })
    }
}

/// `sensor_msgs/msg/NavSatFix`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct NavSatFix {
    pub header: Header,
    pub status: NavSatStatus,
    pub latitude: f64,          // degrees
    pub longitude: f64,         // degrees
    pub altitude: f64,          // metres above WGS84 ellipsoid
    pub position_covariance: [f64; 9],
    pub position_covariance_type: u8, // 0=unknown, 1=approximated, 2=diagonal, 3=known
}

impl CdrSerialize for NavSatFix {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        self.status.cdr_serialize(w)?;
        w.write_f64(self.latitude)?;
        w.write_f64(self.longitude)?;
        w.write_f64(self.altitude)?;
        w.write_f64_array(&self.position_covariance)?;
        w.write_u8(self.position_covariance_type)
    }
}

impl CdrDeserialize for NavSatFix {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let header = Header::cdr_deserialize(r)?;
        let status = NavSatStatus::cdr_deserialize(r)?;
        let latitude = r.read_f64()?;
        let longitude = r.read_f64()?;
        let altitude = r.read_f64()?;
        let mut position_covariance = [0.0f64; 9];
        r.read_f64_into(&mut position_covariance)?;
        let position_covariance_type = r.read_u8()?;
        Ok(Self { header, status, latitude, longitude, altitude, position_covariance, position_covariance_type })
    }
}

// ─── sensor_msgs/msg/BatteryState ─────────────────────────────

/// `sensor_msgs/msg/BatteryState` (simplified — core fields only)
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BatteryState {
    pub header: Header,
    pub voltage: f32,          // Volts
    pub temperature: f32,      // Celsius (NaN if unknown)
    pub current: f32,          // Amps (positive discharging)
    pub charge: f32,           // Ah
    pub capacity: f32,         // Ah
    pub design_capacity: f32,  // Ah
    pub percentage: f32,       // 0.0–1.0
    pub power_supply_status: u8,
    pub power_supply_health: u8,
    pub power_supply_technology: u8,
    pub present: bool,
    // cell_voltage and cell_temperature are dynamic arrays — serialize as empty
}

impl CdrSerialize for BatteryState {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        w.write_f32(self.voltage)?;
        w.write_f32(self.temperature)?;
        w.write_f32(self.current)?;
        w.write_f32(self.charge)?;
        w.write_f32(self.capacity)?;
        w.write_f32(self.design_capacity)?;
        w.write_f32(self.percentage)?;
        w.write_u8(self.power_supply_status)?;
        w.write_u8(self.power_supply_health)?;
        w.write_u8(self.power_supply_technology)?;
        w.write_bool(self.present)?;
        // cell_voltage: empty f32 sequence
        w.write_sequence_len(0)?;
        // cell_temperature: empty f32 sequence
        w.write_sequence_len(0)?;
        // location string
        w.write_string("")?;
        // serial_number string
        w.write_string("")
    }
}

impl CdrDeserialize for BatteryState {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let header = Header::cdr_deserialize(r)?;
        let voltage = r.read_f32()?;
        let temperature = r.read_f32()?;
        let current = r.read_f32()?;
        let charge = r.read_f32()?;
        let capacity = r.read_f32()?;
        let design_capacity = r.read_f32()?;
        let percentage = r.read_f32()?;
        let power_supply_status = r.read_u8()?;
        let power_supply_health = r.read_u8()?;
        let power_supply_technology = r.read_u8()?;
        let present = r.read_bool()?;
        // Skip cell_voltage sequence
        let n = r.read_sequence_len()?;
        for _ in 0..n { r.read_f32()?; }
        // Skip cell_temperature sequence
        let n = r.read_sequence_len()?;
        for _ in 0..n { r.read_f32()?; }
        // Skip location and serial_number strings
        let _ = r.read_string_bytes()?;
        let _ = r.read_string_bytes()?;
        Ok(Self {
            header, voltage, temperature, current, charge, capacity,
            design_capacity, percentage, power_supply_status,
            power_supply_health, power_supply_technology, present,
        })
    }
}

// ─── sensor_msgs/msg/MagneticField ────────────────────────────

/// `sensor_msgs/msg/MagneticField`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MagneticField {
    pub header: Header,
    pub magnetic_field: Vector3,             // Tesla
    pub magnetic_field_covariance: [f64; 9], // row-major 3x3
}

impl CdrSerialize for MagneticField {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        self.magnetic_field.cdr_serialize(w)?;
        w.write_f64_array(&self.magnetic_field_covariance)
    }
}

impl CdrDeserialize for MagneticField {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let header = Header::cdr_deserialize(r)?;
        let magnetic_field = Vector3::cdr_deserialize(r)?;
        let mut magnetic_field_covariance = [0.0f64; 9];
        r.read_f64_into(&mut magnetic_field_covariance)?;
        Ok(Self { header, magnetic_field, magnetic_field_covariance })
    }
}

// ─── sensor_msgs/msg/FluidPressure ────────────────────────────

/// `sensor_msgs/msg/FluidPressure`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FluidPressure {
    pub header: Header,
    pub fluid_pressure: f64, // Pascals
    pub variance: f64,
}

impl CdrSerialize for FluidPressure {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        w.write_f64(self.fluid_pressure)?;
        w.write_f64(self.variance)
    }
}

impl CdrDeserialize for FluidPressure {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let header = Header::cdr_deserialize(r)?;
        let fluid_pressure = r.read_f64()?;
        let variance = r.read_f64()?;
        Ok(Self { header, fluid_pressure, variance })
    }
}

// ─── sensor_msgs/msg/Range ────────────────────────────────────

/// `sensor_msgs/msg/Range`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Range {
    pub header: Header,
    pub radiation_type: u8,  // ULTRASOUND=0, INFRARED=1
    pub field_of_view: f32,  // radians
    pub min_range: f32,      // metres
    pub max_range: f32,      // metres
    pub range: f32,          // metres (current reading)
}

impl CdrSerialize for Range {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        w.write_u8(self.radiation_type)?;
        w.write_f32(self.field_of_view)?;
        w.write_f32(self.min_range)?;
        w.write_f32(self.max_range)?;
        w.write_f32(self.range)
    }
}

impl CdrDeserialize for Range {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let header = Header::cdr_deserialize(r)?;
        let radiation_type = r.read_u8()?;
        let field_of_view = r.read_f32()?;
        let min_range = r.read_f32()?;
        let max_range = r.read_f32()?;
        let range = r.read_f32()?;
        Ok(Self { header, radiation_type, field_of_view, min_range, max_range, range })
    }
}

// ─── nav_msgs/msg/Odometry ────────────────────────────────────

/// `geometry_msgs/msg/PoseWithCovariance`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    pub covariance: [f64; 36], // 6x6 row-major (x,y,z,roll,pitch,yaw)
}

impl Default for PoseWithCovariance {
    fn default() -> Self {
        Self { pose: Pose::default(), covariance: [0.0; 36] }
    }
}

impl CdrSerialize for PoseWithCovariance {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.pose.cdr_serialize(w)?;
        w.write_f64_array(&self.covariance)
    }
}

impl CdrDeserialize for PoseWithCovariance {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let pose = Pose::cdr_deserialize(r)?;
        let mut covariance = [0.0f64; 36];
        r.read_f64_into(&mut covariance)?;
        Ok(Self { pose, covariance })
    }
}

/// `geometry_msgs/msg/TwistWithCovariance`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    pub covariance: [f64; 36],
}

impl Default for TwistWithCovariance {
    fn default() -> Self {
        Self { twist: Twist::default(), covariance: [0.0; 36] }
    }
}

impl CdrSerialize for TwistWithCovariance {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.twist.cdr_serialize(w)?;
        w.write_f64_array(&self.covariance)
    }
}

impl CdrDeserialize for TwistWithCovariance {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let twist = Twist::cdr_deserialize(r)?;
        let mut covariance = [0.0f64; 36];
        r.read_f64_into(&mut covariance)?;
        Ok(Self { twist, covariance })
    }
}

/// `nav_msgs/msg/Odometry`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: FrameId,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}

impl Default for Odometry {
    fn default() -> Self {
        Self {
            header: Header::default(),
            child_frame_id: FrameId::from_str("base_link"),
            pose: PoseWithCovariance::default(),
            twist: TwistWithCovariance::default(),
        }
    }
}

impl CdrSerialize for Odometry {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        w.write_string(self.child_frame_id.as_str())?;
        self.pose.cdr_serialize(w)?;
        self.twist.cdr_serialize(w)
    }
}

impl CdrDeserialize for Odometry {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        let header = Header::cdr_deserialize(r)?;
        let bytes = r.read_string_bytes()?;
        let s = core::str::from_utf8(bytes).map_err(|_| CdrError::InvalidUtf8)?;
        let child_frame_id = FrameId::from_str(s);
        let pose = PoseWithCovariance::cdr_deserialize(r)?;
        let twist = TwistWithCovariance::cdr_deserialize(r)?;
        Ok(Self { header, child_frame_id, pose, twist })
    }
}

// ─── Custom Meridian messages ─────────────────────────────────

/// `meridian_msgs/msg/VehicleState`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MeridianVehicleState {
    pub header: Header,
    pub armed: bool,
    pub mode: u8,           // FlightModeId as u8
    pub ekf_healthy: bool,
    pub battery_voltage: f32,
    pub battery_percentage: f32, // 0.0–1.0
    pub gps_fix_type: u8,
    pub num_satellites: u8,
}

impl CdrSerialize for MeridianVehicleState {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        w.write_bool(self.armed)?;
        w.write_u8(self.mode)?;
        w.write_bool(self.ekf_healthy)?;
        w.write_f32(self.battery_voltage)?;
        w.write_f32(self.battery_percentage)?;
        w.write_u8(self.gps_fix_type)?;
        w.write_u8(self.num_satellites)
    }
}

impl CdrDeserialize for MeridianVehicleState {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self {
            header: Header::cdr_deserialize(r)?,
            armed: r.read_bool()?,
            mode: r.read_u8()?,
            ekf_healthy: r.read_bool()?,
            battery_voltage: r.read_f32()?,
            battery_percentage: r.read_f32()?,
            gps_fix_type: r.read_u8()?,
            num_satellites: r.read_u8()?,
        })
    }
}

/// `meridian_msgs/msg/MissionStatus`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MeridianMissionStatus {
    pub header: Header,
    pub active: bool,
    pub current_item: u16,
    pub total_items: u16,
}

impl CdrSerialize for MeridianMissionStatus {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError> {
        self.header.cdr_serialize(w)?;
        w.write_bool(self.active)?;
        w.write_u16(self.current_item)?;
        w.write_u16(self.total_items)
    }
}

impl CdrDeserialize for MeridianMissionStatus {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError> {
        Ok(Self {
            header: Header::cdr_deserialize(r)?,
            active: r.read_bool()?,
            current_item: r.read_u16()?,
            total_items: r.read_u16()?,
        })
    }
}

// ─── Tests ────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdr::CDR_LE_HEADER;

    fn roundtrip<T: CdrSerialize + CdrDeserialize + PartialEq + core::fmt::Debug>(msg: &T) {
        let mut buf = [0u8; 2048];
        let end = {
            let mut w = CdrWriter::new(&mut buf);
            msg.cdr_serialize(&mut w).expect("serialize");
            w.position()
        };
        let mut r = CdrReader::new(&buf[..end]);
        let decoded = T::cdr_deserialize(&mut r).expect("deserialize");
        assert_eq!(msg, &decoded);
    }

    #[test]
    fn test_time_roundtrip() {
        roundtrip(&Time { sec: 1234567890, nanosec: 500_000_000 });
    }

    #[test]
    fn test_header_roundtrip() {
        roundtrip(&Header {
            stamp: Time { sec: 100, nanosec: 200 },
            frame_id: FrameId::from_str("base_link"),
        });
    }

    #[test]
    fn test_imu_roundtrip() {
        let mut imu = Imu::default();
        imu.header.stamp = Time { sec: 42, nanosec: 0 };
        imu.header.frame_id = FrameId::from_str("imu_link");
        imu.orientation = Quaternion { x: 0.0, y: 0.0, z: 0.707, w: 0.707 };
        imu.angular_velocity = Vector3 { x: 0.01, y: -0.02, z: 0.03 };
        imu.linear_acceleration = Vector3 { x: 0.1, y: 0.2, z: 9.81 };
        roundtrip(&imu);
    }

    #[test]
    fn test_nav_sat_fix_roundtrip() {
        let fix = NavSatFix {
            header: Header { stamp: Time { sec: 1, nanosec: 0 }, frame_id: FrameId::from_str("gps") },
            status: NavSatStatus { status: 0, service: 1 },
            latitude: 36.6,
            longitude: -121.9,
            altitude: 15.0,
            position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0],
            position_covariance_type: 2,
        };
        roundtrip(&fix);
    }

    #[test]
    fn test_battery_state_roundtrip() {
        let bat = BatteryState {
            header: Header::default(),
            voltage: 25.2,
            temperature: f32::NAN, // NaN tests that we handle IEEE 754 correctly
            current: 18.5,
            charge: 5.0,
            capacity: 10.0,
            design_capacity: 10.0,
            percentage: 0.50,
            power_supply_status: 2, // DISCHARGING
            power_supply_health: 1, // GOOD
            power_supply_technology: 3, // LIPO
            present: true,
        };
        // NaN != NaN, so we test fields individually
        let mut buf = [0u8; 512];
        let end = {
            let mut w = CdrWriter::new(&mut buf);
            bat.cdr_serialize(&mut w).unwrap();
            w.position()
        };
        let mut r = CdrReader::new(&buf[..end]);
        let decoded = BatteryState::cdr_deserialize(&mut r).unwrap();
        assert_eq!(decoded.voltage, bat.voltage);
        assert!(decoded.temperature.is_nan());
        assert_eq!(decoded.current, bat.current);
        assert_eq!(decoded.percentage, bat.percentage);
        assert_eq!(decoded.present, true);
    }

    #[test]
    fn test_magnetic_field_roundtrip() {
        roundtrip(&MagneticField {
            header: Header::default(),
            magnetic_field: Vector3 { x: 0.00002, y: 0.00001, z: -0.00004 },
            magnetic_field_covariance: [0.0; 9],
        });
    }

    #[test]
    fn test_fluid_pressure_roundtrip() {
        roundtrip(&FluidPressure {
            header: Header::default(),
            fluid_pressure: 101325.0,
            variance: 10.0,
        });
    }

    #[test]
    fn test_range_roundtrip() {
        roundtrip(&Range {
            header: Header::default(),
            radiation_type: 1,
            field_of_view: 0.05,
            min_range: 0.1,
            max_range: 40.0,
            range: 3.5,
        });
    }

    #[test]
    fn test_pose_stamped_roundtrip() {
        roundtrip(&PoseStamped {
            header: Header { stamp: Time { sec: 10, nanosec: 0 }, frame_id: FrameId::from_str("map") },
            pose: Pose {
                position: Point { x: 1.0, y: 2.0, z: 3.0 },
                orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            },
        });
    }

    #[test]
    fn test_twist_stamped_roundtrip() {
        roundtrip(&TwistStamped {
            header: Header::default(),
            twist: Twist {
                linear: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
                angular: Vector3 { x: 0.0, y: 0.0, z: 0.1 },
            },
        });
    }

    #[test]
    fn test_odometry_roundtrip() {
        roundtrip(&Odometry::default());
    }

    #[test]
    fn test_vehicle_state_roundtrip() {
        roundtrip(&MeridianVehicleState {
            header: Header::default(),
            armed: true,
            mode: 5, // Loiter
            ekf_healthy: true,
            battery_voltage: 25.2,
            battery_percentage: 0.85,
            gps_fix_type: 3,
            num_satellites: 14,
        });
    }

    #[test]
    fn test_mission_status_roundtrip() {
        roundtrip(&MeridianMissionStatus {
            header: Header::default(),
            active: true,
            current_item: 3,
            total_items: 8,
        });
    }
}
