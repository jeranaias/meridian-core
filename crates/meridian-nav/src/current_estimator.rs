//! Water current estimation for USVs.
//!
//! Estimates water current speed and direction by comparing GPS groundspeed
//! vector with the commanded thrust vector. Same principle as wind estimation
//! for aircraft (groundspeed = watertrack + current, analogous to
//! groundspeed = airspeed + wind).
//!
//! The estimator uses a simple first-order low-pass filter on the difference
//! between GPS velocity and estimated vehicle-through-water velocity.
//!
//! For current-compensated navigation, the estimated current vector is fed
//! into the L1 guidance controller so waypoint tracking accounts for drift.

use meridian_math::Vec3;
use meridian_math::frames::NED;

/// Water current estimation configuration.
#[derive(Debug, Clone, Copy)]
pub struct CurrentEstimatorConfig {
    /// Filter time constant (seconds). Higher = smoother, slower to adapt.
    /// Default 10.0 — water currents change slowly.
    pub filter_tc_s: f32,
    /// Minimum groundspeed for valid estimation (m/s).
    /// Below this, thrust-to-speed mapping is unreliable.
    pub min_speed_ms: f32,
    /// Maximum plausible current speed (m/s). Estimates above this are clamped.
    pub max_current_ms: f32,
    /// Whether to feed the estimate into navigation for compensation.
    pub use_for_navigation: bool,
}

impl Default for CurrentEstimatorConfig {
    fn default() -> Self {
        Self {
            filter_tc_s: 10.0,
            min_speed_ms: 0.5,
            max_current_ms: 2.0,
            use_for_navigation: true,
        }
    }
}

/// Water current estimate.
#[derive(Debug, Clone, Copy, Default)]
pub struct CurrentEstimate {
    /// Current velocity vector NED (m/s). Positive N = current flowing north.
    pub velocity_ned: Vec3<NED>,
    /// Current speed (m/s).
    pub speed_ms: f32,
    /// Current direction FROM (degrees, meteorological convention: 0=from north, 90=from east).
    pub from_deg: f32,
    /// Whether the estimate is valid (enough speed, enough time).
    pub valid: bool,
    /// Age of the estimate (seconds since last update).
    pub age_s: f32,
}

/// Water current estimator.
///
/// Algorithm:
/// 1. Observe GPS groundspeed vector: V_ground = [vn, ve] (NED)
/// 2. Estimate vehicle-through-water speed from commanded throttle + heading:
///    V_water = throttle * max_speed * [cos(heading), sin(heading)]
/// 3. Current = V_ground - V_water
/// 4. Low-pass filter the result
///
/// This is a simplified model. A more sophisticated approach would use
/// a Kalman filter or the EKF wind state. But for USVs where the thrust
/// model is simple (throttle → speed is nearly linear), this works well.
pub struct CurrentEstimator {
    config: CurrentEstimatorConfig,

    // Filtered estimate
    current_n: f32, // north component (m/s)
    current_e: f32, // east component (m/s)

    // State
    initialized: bool,
    last_update_s: f32,
    samples: u32,
}

impl CurrentEstimator {
    pub fn new(config: CurrentEstimatorConfig) -> Self {
        Self {
            config,
            current_n: 0.0,
            current_e: 0.0,
            initialized: false,
            last_update_s: 0.0,
            samples: 0,
        }
    }

    /// Update the current estimate with new data.
    ///
    /// `gps_vel_n`: GPS northward velocity (m/s)
    /// `gps_vel_e`: GPS eastward velocity (m/s)
    /// `gps_speed`: GPS ground speed (m/s)
    /// `heading_rad`: Vehicle heading (radians, 0 = north, pi/2 = east)
    /// `throttle`: Commanded throttle (0.0 - 1.0)
    /// `max_speed`: Maximum vehicle speed at full throttle (m/s)
    /// `dt`: Time step (seconds)
    pub fn update(
        &mut self,
        gps_vel_n: f32,
        gps_vel_e: f32,
        gps_speed: f32,
        heading_rad: f32,
        throttle: f32,
        max_speed: f32,
        dt: f32,
    ) {
        // Need minimum speed for valid estimation
        if gps_speed < self.config.min_speed_ms || throttle < 0.05 {
            self.last_update_s += dt;
            return;
        }

        // Estimate vehicle-through-water velocity from thrust model
        // Simple linear model: water_speed = throttle * max_speed
        let water_speed = throttle * max_speed;
        let water_n = water_speed * libm::cosf(heading_rad);
        let water_e = water_speed * libm::sinf(heading_rad);

        // Raw current estimate: ground_velocity - water_velocity
        let raw_n = gps_vel_n - water_n;
        let raw_e = gps_vel_e - water_e;

        // Clamp to maximum plausible current
        let raw_speed = libm::sqrtf(raw_n * raw_n + raw_e * raw_e);
        let (clamped_n, clamped_e) = if raw_speed > self.config.max_current_ms {
            let scale = self.config.max_current_ms / raw_speed;
            (raw_n * scale, raw_e * scale)
        } else {
            (raw_n, raw_e)
        };

        // First-order low-pass filter
        let alpha = if self.config.filter_tc_s > 0.0 {
            let a = dt / (self.config.filter_tc_s + dt);
            a.min(1.0)
        } else {
            1.0
        };

        if !self.initialized {
            self.current_n = clamped_n;
            self.current_e = clamped_e;
            self.initialized = true;
        } else {
            self.current_n += alpha * (clamped_n - self.current_n);
            self.current_e += alpha * (clamped_e - self.current_e);
        }

        self.last_update_s = 0.0;
        self.samples += 1;
    }

    /// Get the current estimate.
    pub fn estimate(&self) -> CurrentEstimate {
        let speed = libm::sqrtf(self.current_n * self.current_n + self.current_e * self.current_e);

        // Direction the current is flowing FROM (meteorological convention)
        let to_deg = libm::atan2f(self.current_e, self.current_n) * 180.0 / core::f32::consts::PI;
        let from_deg = ((to_deg + 180.0) % 360.0 + 360.0) % 360.0;

        CurrentEstimate {
            velocity_ned: Vec3::new(self.current_n, self.current_e, 0.0),
            speed_ms: speed,
            from_deg,
            valid: self.initialized && self.samples > 10 && speed < self.config.max_current_ms,
            age_s: self.last_update_s,
        }
    }

    /// Get the current vector for navigation compensation.
    /// Returns (north_ms, east_ms) or (0, 0) if disabled or invalid.
    pub fn for_navigation(&self) -> (f32, f32) {
        if !self.config.use_for_navigation || !self.initialized || self.samples < 10 {
            return (0.0, 0.0);
        }
        (self.current_n, self.current_e)
    }

    /// Reset the estimator (e.g., on mode change or arm).
    pub fn reset(&mut self) {
        self.current_n = 0.0;
        self.current_e = 0.0;
        self.initialized = false;
        self.last_update_s = 0.0;
        self.samples = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_current() {
        let mut est = CurrentEstimator::new(CurrentEstimatorConfig {
            filter_tc_s: 1.0,
            min_speed_ms: 0.1,
            ..Default::default()
        });

        // Vehicle heading north at 2 m/s, GPS shows 2 m/s north → no current
        for _ in 0..50 {
            est.update(2.0, 0.0, 2.0, 0.0, 0.5, 4.0, 0.1);
        }

        let e = est.estimate();
        assert!(e.speed_ms < 0.1, "expected near-zero current, got {}", e.speed_ms);
    }

    #[test]
    fn test_eastward_current() {
        let mut est = CurrentEstimator::new(CurrentEstimatorConfig {
            filter_tc_s: 1.0,
            min_speed_ms: 0.1,
            ..Default::default()
        });

        // Vehicle heading north at 2 m/s, but GPS shows 2m/s north + 1m/s east
        // → 1 m/s current from west (pushing east)
        for _ in 0..100 {
            est.update(2.0, 1.0, 2.24, 0.0, 0.5, 4.0, 0.1);
        }

        let e = est.estimate();
        assert!(e.valid);
        assert!((e.speed_ms - 1.0).abs() < 0.2, "expected ~1 m/s current, got {}", e.speed_ms);
        // Current flowing east → from_deg should be near 270 (from west)
        assert!((e.from_deg - 270.0).abs() < 30.0, "expected from ~270°, got {}", e.from_deg);
    }

    #[test]
    fn test_clamp_max_current() {
        let mut est = CurrentEstimator::new(CurrentEstimatorConfig {
            filter_tc_s: 0.1,
            min_speed_ms: 0.1,
            max_current_ms: 1.5,
            ..Default::default()
        });

        // Huge discrepancy → should clamp
        for _ in 0..50 {
            est.update(10.0, 0.0, 10.0, 0.0, 0.1, 4.0, 0.1);
        }

        let e = est.estimate();
        assert!(e.speed_ms <= 1.6, "should clamp to max, got {}", e.speed_ms);
    }

    #[test]
    fn test_navigation_output() {
        let mut est = CurrentEstimator::new(CurrentEstimatorConfig {
            filter_tc_s: 0.5,
            min_speed_ms: 0.1,
            use_for_navigation: true,
            ..Default::default()
        });

        // Not enough samples yet
        assert_eq!(est.for_navigation(), (0.0, 0.0));

        for _ in 0..20 {
            est.update(2.0, 0.5, 2.06, 0.0, 0.5, 4.0, 0.1);
        }

        let (n, e) = est.for_navigation();
        assert!(n.abs() < 0.5); // small north current component
        assert!(e > 0.1); // eastward current detected
    }

    #[test]
    fn test_reset() {
        let mut est = CurrentEstimator::new(Default::default());
        for _ in 0..20 {
            est.update(2.0, 1.0, 2.24, 0.0, 0.5, 4.0, 0.1);
        }
        assert!(est.estimate().valid);

        est.reset();
        assert!(!est.estimate().valid);
        assert_eq!(est.samples, 0);
    }
}
