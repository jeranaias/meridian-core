#![no_std]

pub mod compass_cal;
pub mod baro_dps310;
pub mod imu_icm426xx;
pub mod imu_bmi270;
pub mod imu_bmi088;
pub mod imu_invensense;
pub mod compass_ist8310;
pub mod compass_qmc5883l;
pub mod compass_rm3100;
pub mod gps_ublox;
pub mod baro_bmp280;
pub mod baro_ms5611;
pub mod airspeed;
pub mod rangefinder;
pub mod optical_flow;
pub mod gps_nmea;

// --- New modules (parity gap fixes) ---

/// ICM-20948 / ICM-20649 / ICM-20648 IMU driver (Invensensev2 family).
pub mod imu_invensensev2;

/// LSM6DSV IMU driver (new ST 6-axis MEMS).
pub mod imu_lsm6dsv;

/// External AHRS IMU passthrough (VectorNav, SBG, etc.).
pub mod imu_external_ahrs;

/// IMU vibration and clipping detection (per-sample, two-stage LP filter).
pub mod imu_vibration;

/// IMU temperature calibration (3rd-order polynomial).
pub mod imu_tempcal;

/// SPL06-001 barometric pressure sensor (common budget baro).
pub mod baro_spl06;

/// Per-motor compass compensation (power^0.65 mode).
pub mod compass_motor_comp;

/// GPS blending (weighted average by inverse hacc^2).
pub mod gps_blend;

/// UBX configuration sequence, RTK moving baseline stub, GPS-for-yaw stub.
pub mod gps_ubx_config;

/// I2C rangefinders: Garmin Lite, MaxBotix, VL53L0X, TeraRanger, analog, PWM, etc.
pub mod rangefinder_i2c;

/// Benewake TFMini / TF02 / TF03 serial lidar rangefinders.
pub mod rangefinder_benewake;

/// LightWare I2C + serial lidar rangefinders (SF10/SF11/SF20/LW20/SF40C/SF45B).
pub mod rangefinder_lightware;

/// STM VL53L0X / VL53L1X time-of-flight I2C rangefinders.
pub mod rangefinder_vl53l;

/// PX4Flow optical flow I2C sensor.
pub mod optical_flow_px4flow;

/// VectorNav VN-100/200/300 external AHRS binary protocol.
pub mod external_ahrs_vectornav;

/// DLVR digital differential pressure sensor (airspeed).
pub mod airspeed_dlvr;

/// MS4525DO differential pressure sensor (most common airspeed sensor).
pub mod airspeed_ms4525;

/// Sensirion SDP3x differential pressure sensor (airspeed).
pub mod airspeed_sdp3x;

/// Airspeed health monitoring (probability LPF, auto-disable/re-enable).
/// Re-exported from ms4525 module — used by all airspeed backends.
pub use airspeed_ms4525::AirspeedHealth;

/// 3-state Kalman in-flight airspeed calibration.
pub mod airspeed_cal;

/// 6-point accelerometer calibration (AP_AccelCal equivalent).
pub mod accel_cal;

/// Analog battery monitor (ADC voltage/current with scale factors).
pub mod battery_analog;

/// DroneCAN GPS receiver + ExternalAHRS GPS passthrough.
pub mod gps_dronecan;

/// MAVLink DISTANCE_SENSOR, DroneCAN rangefinder, Maxbotix serial.
pub mod rangefinder_mavlink;

/// PMW3901 SPI, MAVLink OPTICAL_FLOW, CXOF serial optical flow.
pub mod optical_flow_extras;

/// Indoor positioning beacons (Marvelmind, Pozyx, Nooploop).
pub mod beacon;

/// InertialLabs, MicroStrain GX5/GQ7, SBG Ellipse external AHRS + shared infra.
pub mod external_ahrs_extras;

/// VESC brushless ESC driver over UART — Vanguard USV jet-pump propulsion.
/// COMM_PACKET protocol: setpoint control (duty/RPM/current) + telemetry.
pub mod esc_vesc;
