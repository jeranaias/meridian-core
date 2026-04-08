//! Reward function for the DRL PID tuning agent.
//!
//! The reward captures what "good control" means for a USV:
//! - Low tracking error (crosstrack + heading + speed)
//! - Smooth behavior (low oscillation)
//! - Efficiency (reaching target speed with minimum throttle)
//! - Stability penalty for large gain changes

/// Reward computation weights.
#[derive(Clone, Copy)]
pub struct RewardWeights {
    /// Weight for crosstrack error penalty. Default 1.0.
    pub crosstrack: f32,
    /// Weight for heading error penalty. Default 0.8.
    pub heading: f32,
    /// Weight for speed error penalty. Default 0.5.
    pub speed: f32,
    /// Weight for heading oscillation penalty. Default 0.3.
    pub heading_oscillation: f32,
    /// Weight for speed oscillation penalty. Default 0.2.
    pub speed_oscillation: f32,
    /// Weight for gain change penalty (stability). Default 0.1.
    pub gain_change: f32,
    /// Weight for efficiency bonus. Default 0.1.
    pub efficiency: f32,
}

impl Default for RewardWeights {
    fn default() -> Self {
        Self {
            crosstrack: 1.0,
            heading: 0.8,
            speed: 0.5,
            heading_oscillation: 0.3,
            speed_oscillation: 0.2,
            gain_change: 0.1,
            efficiency: 0.1,
        }
    }
}

/// Compute the reward for the current timestep.
///
/// `crosstrack_err`: absolute crosstrack error in meters
/// `heading_err`: absolute heading error in radians
/// `speed_err`: absolute speed error in m/s
/// `heading_osc`: heading oscillation amplitude (degrees)
/// `speed_osc`: speed oscillation amplitude (m/s)
/// `gain_change_mag`: sum of absolute fractional gain changes this step
/// `throttle`: current throttle fraction (0-1)
/// `target_speed`: desired speed (m/s)
/// `actual_speed`: measured speed (m/s)
pub fn compute_reward(
    w: &RewardWeights,
    crosstrack_err: f32,
    heading_err: f32,
    speed_err: f32,
    heading_osc: f32,
    speed_osc: f32,
    gain_change_mag: f32,
    throttle: f32,
    target_speed: f32,
    actual_speed: f32,
) -> f32 {
    // Tracking error penalties (quadratic — penalizes large errors heavily)
    let r_crosstrack = -w.crosstrack * crosstrack_err * crosstrack_err;
    let r_heading = -w.heading * heading_err * heading_err;
    let r_speed = -w.speed * speed_err * speed_err;

    // Oscillation penalties (linear — constant tax on jitter)
    let r_heading_osc = -w.heading_oscillation * heading_osc;
    let r_speed_osc = -w.speed_oscillation * speed_osc;

    // Gain change penalty (discourage rapid PID adjustments)
    let r_gain_change = -w.gain_change * gain_change_mag;

    // Efficiency bonus: reward maintaining speed with low throttle
    let speed_ratio = if target_speed > 0.1 {
        (actual_speed / target_speed).clamp(0.0, 1.5)
    } else {
        1.0
    };
    let r_efficiency = w.efficiency * speed_ratio * (1.0 - throttle * 0.5);

    // Alive bonus: small positive reward for each timestep the vehicle is tracking
    // (prevents the agent from learning to "do nothing")
    let r_alive = 0.05;

    r_crosstrack + r_heading + r_speed
        + r_heading_osc + r_speed_osc
        + r_gain_change + r_efficiency + r_alive
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perfect_tracking() {
        let w = RewardWeights::default();
        let r = compute_reward(&w, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 3.0, 3.0);
        assert!(r > 0.0, "perfect tracking should give positive reward: {}", r);
    }

    #[test]
    fn test_large_crosstrack_error() {
        let w = RewardWeights::default();
        let r_good = compute_reward(&w, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 3.0, 3.0);
        let r_bad = compute_reward(&w, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 3.0, 3.0);
        assert!(r_good > r_bad, "large crosstrack should give worse reward");
    }

    #[test]
    fn test_gain_change_penalty() {
        let w = RewardWeights::default();
        let r_stable = compute_reward(&w, 0.5, 0.1, 0.1, 0.0, 0.0, 0.0, 0.5, 3.0, 3.0);
        let r_changing = compute_reward(&w, 0.5, 0.1, 0.1, 0.0, 0.0, 1.0, 0.5, 3.0, 3.0);
        assert!(r_stable > r_changing, "gain changes should be penalized");
    }

    #[test]
    fn test_oscillation_penalty() {
        let w = RewardWeights::default();
        let r_smooth = compute_reward(&w, 0.5, 0.1, 0.1, 0.0, 0.0, 0.0, 0.5, 3.0, 3.0);
        let r_oscillating = compute_reward(&w, 0.5, 0.1, 0.1, 10.0, 0.5, 0.0, 0.5, 3.0, 3.0);
        assert!(r_smooth > r_oscillating, "oscillation should be penalized");
    }
}
