//! State representation for the DRL PID tuning agent.
//!
//! The state vector captures everything the agent needs to decide
//! whether to adjust PID gains: current gains, tracking errors,
//! error dynamics, vehicle state, and sea state estimate.

/// Number of elements in the state vector.
pub const STATE_DIM: usize = 20;

/// Number of PID axes (steering P/I/D + speed P/I/D).
pub const NUM_GAINS: usize = 6;

/// State vector for the DRL agent.
///
/// All values are normalized to roughly [-1, 1] range for neural network input.
#[derive(Clone, Copy)]
pub struct AgentState {
    pub data: [f32; STATE_DIM],
}

impl AgentState {
    pub fn zero() -> Self {
        Self { data: [0.0; STATE_DIM] }
    }

    /// Build state from raw vehicle telemetry.
    ///
    /// `gains`: current [steer_p, steer_i, steer_d, speed_p, speed_i, speed_d]
    /// `errors`: [crosstrack_m, heading_err_rad, speed_err_ms]
    /// `error_dots`: [d_crosstrack, d_heading_err, d_speed_err] per second
    /// `error_integrals`: [int_crosstrack, int_heading, int_speed] over last 10s
    /// `vehicle`: [speed_ms, yaw_rate_rads, throttle_frac]
    /// `sea_state`: [heading_oscillation_amplitude, speed_oscillation_amplitude]
    pub fn from_telemetry(
        gains: &[f32; NUM_GAINS],
        errors: &[f32; 3],
        error_dots: &[f32; 3],
        error_integrals: &[f32; 3],
        vehicle: &[f32; 3],
        sea_state: &[f32; 2],
    ) -> Self {
        let mut s = Self::zero();

        // Gains (normalized by typical range)
        s.data[0] = gains[0] / 2.0;  // steer P / 2.0
        s.data[1] = gains[1] / 1.0;  // steer I / 1.0
        s.data[2] = gains[2] / 0.1;  // steer D / 0.1
        s.data[3] = gains[3] / 2.0;  // speed P
        s.data[4] = gains[4] / 1.0;  // speed I
        s.data[5] = gains[5] / 0.1;  // speed D

        // Tracking errors (normalized)
        s.data[6] = (errors[0] / 5.0).clamp(-1.0, 1.0);  // crosstrack / 5m
        s.data[7] = (errors[1] / 0.5).clamp(-1.0, 1.0);  // heading / 0.5 rad (~30°)
        s.data[8] = (errors[2] / 2.0).clamp(-1.0, 1.0);  // speed / 2 m/s

        // Error derivatives
        s.data[9]  = (error_dots[0] / 2.0).clamp(-1.0, 1.0);
        s.data[10] = (error_dots[1] / 1.0).clamp(-1.0, 1.0);
        s.data[11] = (error_dots[2] / 1.0).clamp(-1.0, 1.0);

        // Error integrals (over 10s window)
        s.data[12] = (error_integrals[0] / 20.0).clamp(-1.0, 1.0);
        s.data[13] = (error_integrals[1] / 5.0).clamp(-1.0, 1.0);
        s.data[14] = (error_integrals[2] / 10.0).clamp(-1.0, 1.0);

        // Vehicle dynamics
        s.data[15] = (vehicle[0] / 5.0).clamp(0.0, 1.0);  // speed / 5 m/s
        s.data[16] = (vehicle[1] / 1.0).clamp(-1.0, 1.0);  // yaw rate / 1 rad/s
        s.data[17] = vehicle[2].clamp(0.0, 1.0);            // throttle fraction

        // Sea state estimate
        s.data[18] = (sea_state[0] / 10.0).clamp(0.0, 1.0); // heading oscillation / 10°
        s.data[19] = (sea_state[1] / 0.5).clamp(0.0, 1.0);  // speed oscillation / 0.5 m/s

        s
    }
}

/// Observation window for computing error derivatives and integrals.
pub struct ObservationWindow {
    /// Ring buffer of recent errors [crosstrack, heading, speed].
    history: [[f32; 3]; 100], // 10s at 10Hz
    head: usize,
    count: usize,
    dt: f32,
}

impl ObservationWindow {
    pub fn new(dt: f32) -> Self {
        Self {
            history: [[0.0; 3]; 100],
            head: 0,
            count: 0,
            dt,
        }
    }

    /// Push a new error sample.
    pub fn push(&mut self, errors: [f32; 3]) {
        self.history[self.head] = errors;
        self.head = (self.head + 1) % 100;
        if self.count < 100 { self.count += 1; }
    }

    /// Compute error derivatives (finite difference of last 2 samples).
    pub fn derivatives(&self) -> [f32; 3] {
        if self.count < 2 { return [0.0; 3]; }
        let curr = self.history[(self.head + 99) % 100];
        let prev = self.history[(self.head + 98) % 100];
        [
            (curr[0] - prev[0]) / self.dt,
            (curr[1] - prev[1]) / self.dt,
            (curr[2] - prev[2]) / self.dt,
        ]
    }

    /// Compute error integrals (sum over window * dt).
    pub fn integrals(&self) -> [f32; 3] {
        let mut sum = [0.0f32; 3];
        let n = self.count.min(100);
        for i in 0..n {
            let idx = (self.head + 100 - n + i) % 100;
            sum[0] += libm::fabsf(self.history[idx][0]);
            sum[1] += libm::fabsf(self.history[idx][1]);
            sum[2] += libm::fabsf(self.history[idx][2]);
        }
        [sum[0] * self.dt, sum[1] * self.dt, sum[2] * self.dt]
    }

    /// Estimate sea state: amplitude of oscillation in heading and speed.
    pub fn sea_state_estimate(&self) -> [f32; 2] {
        if self.count < 20 { return [0.0; 2]; }
        let n = self.count.min(50);
        let mut heading_min = f32::MAX;
        let mut heading_max = f32::MIN;
        let mut speed_min = f32::MAX;
        let mut speed_max = f32::MIN;
        for i in 0..n {
            let idx = (self.head + 100 - n + i) % 100;
            let h = self.history[idx][1]; // heading error
            let s = self.history[idx][2]; // speed error
            if h < heading_min { heading_min = h; }
            if h > heading_max { heading_max = h; }
            if s < speed_min { speed_min = s; }
            if s > speed_max { speed_max = s; }
        }
        [(heading_max - heading_min) * 0.5, (speed_max - speed_min) * 0.5]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_dims() {
        let s = AgentState::zero();
        assert_eq!(s.data.len(), STATE_DIM);
    }

    #[test]
    fn test_observation_window() {
        let mut w = ObservationWindow::new(0.1);
        for i in 0..50 {
            w.push([i as f32 * 0.1, 0.0, 0.0]);
        }
        let d = w.derivatives();
        assert!(d[0] > 0.0, "crosstrack derivative should be positive");
        let int = w.integrals();
        assert!(int[0] > 0.0, "crosstrack integral should be positive");
    }

    #[test]
    fn test_sea_state() {
        let mut w = ObservationWindow::new(0.1);
        // Simulate heading oscillation ±5°
        for i in 0..50 {
            let osc = 5.0 * libm::sinf(i as f32 * 0.5);
            w.push([0.0, osc, 0.0]);
        }
        let ss = w.sea_state_estimate();
        assert!(ss[0] > 2.0, "should detect heading oscillation: {}", ss[0]);
    }
}
