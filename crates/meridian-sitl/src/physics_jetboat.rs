//! Jet boat physics simulation — single jet engine with steering nozzle.
//!
//! Control axes:
//!   - Throttle: 0.0 to 1.0 (jet pump speed)
//!   - Steering: -1.0 to +1.0 (nozzle deflection, negative = port/left, positive = starboard/right)
//!
//! Physics model:
//!   - Thrust = throttle * max_thrust, always directed through the nozzle
//!   - Nozzle deflects thrust vector left/right by up to max_nozzle_angle
//!   - Forward force = thrust * cos(nozzle_angle)
//!   - Yaw torque = thrust * sin(nozzle_angle) * moment_arm
//!   - Water drag: quadratic in speed (F_drag = -Cd * v * |v|)
//!   - Yaw drag: quadratic in yaw rate (T_drag = -Cd_yaw * r * |r|)
//!   - No roll, no pitch, no heave — the boat stays flat on the water.

use meridian_math::geodetic;

// ─── Constants ─────────────────────────────────────────────────

/// Earth radius for position integration (meters).
const R_EARTH: f64 = 6_371_000.0;

// ─── Configuration ─────────────────────────────────────────────

/// Jet boat physical parameters.
#[derive(Debug, Clone)]
pub struct JetBoatParams {
    /// Dry mass including fuel and payload (kg).
    pub mass_kg: f32,
    /// Hull waterline length (m). Affects hull speed limit.
    pub hull_length_m: f32,
    /// Maximum jet thrust at full throttle (Newtons).
    pub max_thrust_n: f32,
    /// Maximum nozzle deflection angle (radians). Typically 20-30°.
    pub max_nozzle_angle_rad: f32,
    /// Distance from center of mass to nozzle (m). Affects yaw torque.
    pub nozzle_moment_arm_m: f32,
    /// Longitudinal drag coefficient (N / (m/s)²).
    /// Water drag is much higher than air: typical 10-30 for a small jet boat.
    pub drag_longitudinal: f32,
    /// Lateral drag coefficient (N / (m/s)²).
    /// Hulls resist sideways motion strongly.
    pub drag_lateral: f32,
    /// Yaw drag coefficient (N·m / (rad/s)²).
    pub drag_yaw: f32,
    /// Moment of inertia about vertical axis (kg·m²).
    pub yaw_inertia: f32,
    /// Thrust spool-up time constant (seconds). Jets don't respond instantly.
    pub thrust_time_constant: f32,
    /// Nozzle deflection rate limit (rad/s). Mechanical actuator speed.
    pub nozzle_rate_limit: f32,
    /// Hull speed limit factor. Hull speed = factor * sqrt(waterline_length).
    /// Typical: 1.34 for displacement hulls, 2.5+ for planing hulls.
    pub hull_speed_factor: f32,
}

impl Default for JetBoatParams {
    fn default() -> Self {
        Self {
            mass_kg: 30.0,
            hull_length_m: 1.5,
            max_thrust_n: 50.0,
            max_nozzle_angle_rad: 25.0 * core::f32::consts::PI / 180.0, // 25°
            nozzle_moment_arm_m: 0.6, // nozzle is 0.6m behind CoM
            drag_longitudinal: 15.0,
            drag_lateral: 40.0,   // hulls really resist sideways motion
            drag_yaw: 8.0,
            yaw_inertia: 3.0,     // kg·m² — depends on mass distribution
            thrust_time_constant: 0.8, // jets spool slowly
            nozzle_rate_limit: 2.0,    // ~115°/s nozzle actuator
            hull_speed_factor: 1.8,    // semi-planing hull
        }
    }
}

impl JetBoatParams {
    /// Theoretical hull speed (m/s). Above this, drag increases dramatically.
    pub fn hull_speed(&self) -> f32 {
        self.hull_speed_factor * libm::sqrtf(self.hull_length_m)
    }
}

// ─── State ────────────────────────────────────────────────────

/// Jet boat simulation state.
#[derive(Debug, Clone)]
pub struct JetBoatState {
    // Position (WGS84)
    pub lat: f64,    // degrees
    pub lon: f64,    // degrees

    // Velocity in body frame (m/s)
    pub speed_forward: f32,  // surge (positive = ahead)
    pub speed_lateral: f32,  // sway (positive = starboard)

    // Heading
    pub heading_rad: f32,    // radians, 0 = north, pi/2 = east
    pub yaw_rate: f32,       // rad/s, positive = clockwise (turning right)

    // Engine state
    pub actual_thrust: f32,  // current thrust after spool-up (Newtons)
    pub actual_nozzle: f32,  // current nozzle angle (radians)

    // Telemetry
    pub ground_speed: f32,   // m/s (magnitude of velocity vector)
    pub ground_course: f32,  // radians (direction of travel over ground)

    // Water current (for simulation)
    pub current_n: f32,      // m/s northward current
    pub current_e: f32,      // m/s eastward current
}

impl JetBoatState {
    pub fn new(lat: f64, lon: f64, heading_deg: f32) -> Self {
        Self {
            lat,
            lon,
            speed_forward: 0.0,
            speed_lateral: 0.0,
            heading_rad: heading_deg * core::f32::consts::PI / 180.0,
            yaw_rate: 0.0,
            actual_thrust: 0.0,
            actual_nozzle: 0.0,
            ground_speed: 0.0,
            ground_course: 0.0,
            current_n: 0.0,
            current_e: 0.0,
        }
    }

    /// Set a simulated water current.
    pub fn set_current(&mut self, speed_ms: f32, from_deg: f32) {
        let to_rad = (from_deg + 180.0) * core::f32::consts::PI / 180.0;
        self.current_n = speed_ms * libm::cosf(to_rad);
        self.current_e = speed_ms * libm::sinf(to_rad);
    }

    /// Heading in degrees (0-360).
    pub fn heading_deg(&self) -> f32 {
        let deg = self.heading_rad * 180.0 / core::f32::consts::PI;
        ((deg % 360.0) + 360.0) % 360.0
    }

    /// Velocity in NED frame (m/s), including current.
    pub fn velocity_ned(&self) -> (f32, f32) {
        let cos_h = libm::cosf(self.heading_rad);
        let sin_h = libm::sinf(self.heading_rad);

        // Body to NED rotation
        let vn = self.speed_forward * cos_h - self.speed_lateral * sin_h + self.current_n;
        let ve = self.speed_forward * sin_h + self.speed_lateral * cos_h + self.current_e;
        (vn, ve)
    }
}

// ─── Simulation Step ──────────────────────────────────────────

/// Advance the jet boat simulation by one time step.
///
/// `throttle_cmd`: 0.0 to 1.0 (jet pump speed command)
/// `steering_cmd`: -1.0 to +1.0 (nozzle deflection command, negative = turn left)
/// `dt`: time step in seconds
pub fn step(
    state: &mut JetBoatState,
    params: &JetBoatParams,
    throttle_cmd: f32,
    steering_cmd: f32,
    dt: f32,
) {
    // ── 1. Engine spool-up (first-order lag) ──
    let target_thrust = throttle_cmd.clamp(0.0, 1.0) * params.max_thrust_n;
    let thrust_alpha = dt / (params.thrust_time_constant + dt);
    state.actual_thrust += thrust_alpha * (target_thrust - state.actual_thrust);

    // ── 2. Nozzle deflection (rate-limited) ──
    let target_nozzle = steering_cmd.clamp(-1.0, 1.0) * params.max_nozzle_angle_rad;
    let nozzle_delta = target_nozzle - state.actual_nozzle;
    let max_delta = params.nozzle_rate_limit * dt;
    if libm::fabsf(nozzle_delta) > max_delta {
        state.actual_nozzle += if nozzle_delta > 0.0 { max_delta } else { -max_delta };
    } else {
        state.actual_nozzle = target_nozzle;
    }

    // ── 3. Forces in body frame ──

    // Thrust decomposed by nozzle angle
    let thrust_forward = state.actual_thrust * libm::cosf(state.actual_nozzle);
    let thrust_lateral = state.actual_thrust * libm::sinf(state.actual_nozzle);

    // Water drag (quadratic, opposes motion)
    let drag_forward = -params.drag_longitudinal * state.speed_forward * libm::fabsf(state.speed_forward);
    let drag_lateral = -params.drag_lateral * state.speed_lateral * libm::fabsf(state.speed_lateral);

    // Hull speed resistance: drag increases sharply above hull speed
    let hull_speed = params.hull_speed();
    let speed_ratio = libm::fabsf(state.speed_forward) / hull_speed.max(0.1);
    let hull_drag_multiplier = if speed_ratio > 1.0 {
        1.0 + 3.0 * (speed_ratio - 1.0) * (speed_ratio - 1.0) // quadratic penalty above hull speed
    } else {
        1.0
    };

    // Net forces
    let force_forward = thrust_forward + drag_forward * hull_drag_multiplier;
    let force_lateral = thrust_lateral + drag_lateral;

    // ── 4. Yaw torque ──

    // Nozzle yaw torque: lateral thrust component * moment arm
    let yaw_torque = thrust_lateral * params.nozzle_moment_arm_m;

    // Yaw drag (opposes rotation)
    let yaw_drag = -params.drag_yaw * state.yaw_rate * libm::fabsf(state.yaw_rate);

    let net_yaw_torque = yaw_torque + yaw_drag;

    // ── 5. Integration (semi-implicit Euler) ──

    // Accelerations
    let accel_forward = force_forward / params.mass_kg;
    let accel_lateral = force_lateral / params.mass_kg;
    let yaw_accel = net_yaw_torque / params.yaw_inertia;

    // Update velocities
    state.speed_forward += accel_forward * dt;
    state.speed_lateral += accel_lateral * dt;
    state.yaw_rate += yaw_accel * dt;

    // Update heading
    state.heading_rad += state.yaw_rate * dt;
    // Normalize to [0, 2π)
    state.heading_rad = ((state.heading_rad % (2.0 * core::f32::consts::PI))
        + 2.0 * core::f32::consts::PI) % (2.0 * core::f32::consts::PI);

    // ── 6. Position update (NED to lat/lon) ──

    let (vn, ve) = state.velocity_ned();
    state.ground_speed = libm::sqrtf(vn * vn + ve * ve);
    state.ground_course = libm::atan2f(ve, vn);

    // Integrate position (small angle approximation for GPS-scale motion)
    let dlat = (vn as f64 * dt as f64) / R_EARTH * (180.0 / core::f64::consts::PI);
    let dlon = (ve as f64 * dt as f64) / (R_EARTH * libm::cos(state.lat * core::f64::consts::PI / 180.0))
        * (180.0 / core::f64::consts::PI);

    state.lat += dlat;
    state.lon += dlon;
}

// ─── Sensor Simulation ───────────────────────────────────────

/// Simulated sensor readings from the jet boat state.
pub struct JetBoatSensors {
    /// GPS latitude (degrees).
    pub gps_lat: f64,
    /// GPS longitude (degrees).
    pub gps_lon: f64,
    /// GPS ground speed (m/s).
    pub gps_speed: f32,
    /// GPS ground course (degrees, 0=north).
    pub gps_course_deg: f32,
    /// Compass heading (degrees, with noise).
    pub compass_heading_deg: f32,
    /// Yaw rate from gyro (rad/s, with noise).
    pub gyro_yaw_rate: f32,
    /// Forward acceleration from accelerometer (m/s², with noise).
    pub accel_forward: f32,
    /// Lateral acceleration from accelerometer (m/s², with noise).
    pub accel_lateral: f32,
}

/// Generate simulated sensor readings from physics state.
///
/// `noise_seed`: incrementing counter for deterministic noise.
pub fn sample_sensors(state: &JetBoatState, params: &JetBoatParams, noise_seed: u32) -> JetBoatSensors {
    // Simple deterministic noise using seed
    let noise = |seed: u32, amplitude: f32| -> f32 {
        let n = ((seed.wrapping_mul(1103515245).wrapping_add(12345)) >> 16) as f32 / 32768.0;
        (n - 0.5) * 2.0 * amplitude
    };

    let gps_noise_m = 1.5; // ±1.5m GPS noise
    let gps_lat_noise = noise(noise_seed, gps_noise_m) as f64 / R_EARTH * (180.0 / core::f64::consts::PI);
    let gps_lon_noise = noise(noise_seed + 1, gps_noise_m) as f64
        / (R_EARTH * libm::cos(state.lat * core::f64::consts::PI / 180.0))
        * (180.0 / core::f64::consts::PI);

    JetBoatSensors {
        gps_lat: state.lat + gps_lat_noise,
        gps_lon: state.lon + gps_lon_noise,
        gps_speed: state.ground_speed + noise(noise_seed + 2, 0.1),
        gps_course_deg: state.heading_deg() + noise(noise_seed + 3, 2.0),
        compass_heading_deg: state.heading_deg() + noise(noise_seed + 4, 1.5),
        gyro_yaw_rate: state.yaw_rate + noise(noise_seed + 5, 0.005),
        accel_forward: (state.actual_thrust * libm::cosf(state.actual_nozzle)) / params.mass_kg
            + noise(noise_seed + 6, 0.1),
        accel_lateral: (state.actual_thrust * libm::sinf(state.actual_nozzle)) / params.mass_kg
            + noise(noise_seed + 7, 0.1),
    }
}

// ─── Tests ───────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_state() -> JetBoatState {
        JetBoatState::new(35.75, -120.77, 0.0)
    }

    #[test]
    fn test_stationary() {
        let mut state = default_state();
        let params = JetBoatParams::default();

        // No throttle, no steering — should stay put
        for _ in 0..100 {
            step(&mut state, &params, 0.0, 0.0, 0.1);
        }

        assert!(state.speed_forward.abs() < 0.01);
        assert!(state.yaw_rate.abs() < 0.01);
    }

    #[test]
    fn test_straight_ahead() {
        let mut state = default_state();
        let params = JetBoatParams::default();

        // Full throttle, no steering — should go straight north
        for _ in 0..200 {
            step(&mut state, &params, 0.8, 0.0, 0.1);
        }

        assert!(state.speed_forward > 1.0, "should be moving forward: {}", state.speed_forward);
        assert!(state.speed_lateral.abs() < 0.1, "should not be drifting sideways");
        assert!(state.lat > 35.75, "should have moved north");
    }

    #[test]
    fn test_turn_right() {
        let mut state = default_state();
        let params = JetBoatParams::default();

        // Throttle + right steering
        for _ in 0..200 {
            step(&mut state, &params, 0.6, 0.5, 0.1);
        }

        assert!(state.yaw_rate > 0.0, "should be turning right (positive yaw rate)");
        assert!(state.heading_deg() > 10.0, "heading should have changed from 0");
    }

    #[test]
    fn test_turn_left() {
        let mut state = default_state();
        let params = JetBoatParams::default();

        for _ in 0..200 {
            step(&mut state, &params, 0.6, -0.5, 0.1);
        }

        assert!(state.yaw_rate < 0.0, "should be turning left (negative yaw rate)");
    }

    #[test]
    fn test_no_steering_without_thrust() {
        let mut state = default_state();
        let params = JetBoatParams::default();

        // Steering with no throttle — nozzle deflects but no thrust = no yaw torque
        for _ in 0..100 {
            step(&mut state, &params, 0.0, 1.0, 0.1);
        }

        assert!(state.yaw_rate.abs() < 0.01, "no thrust = no turning force");
    }

    #[test]
    fn test_hull_speed_limit() {
        let mut state = default_state();
        let params = JetBoatParams::default();
        let hull_speed = params.hull_speed();

        // Full throttle for a long time — should approach but not greatly exceed hull speed
        for _ in 0..1000 {
            step(&mut state, &params, 1.0, 0.0, 0.1);
        }

        assert!(
            state.speed_forward < hull_speed * 1.5,
            "speed {} should not greatly exceed hull speed {}",
            state.speed_forward, hull_speed
        );
    }

    #[test]
    fn test_current_drift() {
        let mut state = default_state();
        let params = JetBoatParams::default();

        // Set a 1 m/s current from the west (pushing east)
        state.set_current(1.0, 270.0);

        // No throttle — should drift east
        let start_lon = state.lon;
        for _ in 0..200 {
            step(&mut state, &params, 0.0, 0.0, 0.1);
        }

        assert!(state.lon > start_lon, "should have drifted east with current");
    }

    #[test]
    fn test_thrust_spool_up() {
        let mut state = default_state();
        let params = JetBoatParams::default();

        // Step to full throttle — thrust should lag
        step(&mut state, &params, 1.0, 0.0, 0.01);
        assert!(
            state.actual_thrust < params.max_thrust_n * 0.5,
            "thrust should not jump instantly: {}",
            state.actual_thrust
        );

        // After many steps, should approach max
        for _ in 0..500 {
            step(&mut state, &params, 1.0, 0.0, 0.01);
        }
        assert!(
            state.actual_thrust > params.max_thrust_n * 0.95,
            "thrust should have spooled up: {}",
            state.actual_thrust
        );
    }

    #[test]
    fn test_sensor_noise() {
        let state = default_state();
        let params = JetBoatParams::default();

        let s1 = sample_sensors(&state, &params, 100);
        let s2 = sample_sensors(&state, &params, 200);

        // Different seeds should give different noise
        assert!((s1.compass_heading_deg - s2.compass_heading_deg).abs() > 0.01);
    }
}
