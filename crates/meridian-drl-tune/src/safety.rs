//! Safety constraints for the DRL adaptive PID agent.
//!
//! These constraints prevent the agent from destabilizing the vehicle.
//! They are hard limits that cannot be overridden by the policy.

use crate::state::NUM_GAINS;

/// Safety configuration for the adaptive agent.
#[derive(Clone, Copy)]
pub struct SafetyConfig {
    /// Minimum allowed gain values (absolute floor).
    pub gain_min: [f32; NUM_GAINS],
    /// Maximum allowed gain values (absolute ceiling).
    pub gain_max: [f32; NUM_GAINS],
    /// Maximum fractional adjustment per second (e.g., 0.05 = 5%/s).
    pub max_adjust_rate: f32,
    /// Crosstrack error threshold (m) — revert to baseline if exceeded.
    pub crosstrack_safety_threshold: f32,
    /// Heading error threshold (rad) — revert if exceeded.
    pub heading_safety_threshold: f32,
    /// Minimum time between adjustments (seconds).
    pub min_adjust_interval: f32,
    /// Whether the agent is currently frozen (operator override).
    pub frozen: bool,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            // [steer_P, steer_I, steer_D, speed_P, speed_I, speed_D]
            gain_min: [0.05, 0.0, 0.0, 0.05, 0.0, 0.0],
            gain_max: [5.0,  2.0, 0.5, 5.0,  2.0, 0.5],
            max_adjust_rate: 0.05,  // 5% per second
            crosstrack_safety_threshold: 10.0, // 10m — something is very wrong
            heading_safety_threshold: 1.0,     // ~57° — way off course
            min_adjust_interval: 1.0,          // no faster than 1Hz adjustments
            frozen: false,
        }
    }
}

/// Safety enforcement result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SafetyAction {
    /// Adjustments are allowed.
    Allow,
    /// Adjustments are clamped to safe range.
    Clamped,
    /// Agent is frozen (operator override).
    Frozen,
    /// Emergency: revert to baseline gains immediately.
    RevertToBaseline,
}

/// Safety monitor that enforces constraints on gain adjustments.
pub struct SafetyMonitor {
    pub config: SafetyConfig,
    /// Baseline gains from relay auto-tune (safe fallback).
    pub baseline_gains: [f32; NUM_GAINS],
    /// Last adjustment time (seconds since boot).
    last_adjust_time: f32,
    /// Whether we're currently in emergency revert mode.
    pub emergency_active: bool,
    /// Number of consecutive safety violations.
    violation_count: u32,
}

impl SafetyMonitor {
    pub fn new(config: SafetyConfig, baseline: [f32; NUM_GAINS]) -> Self {
        Self {
            config,
            baseline_gains: baseline,
            last_adjust_time: 0.0,
            emergency_active: false,
            violation_count: 0,
        }
    }

    /// Check whether a proposed gain adjustment is safe.
    ///
    /// `current_gains`: current PID gains
    /// `proposed_gains`: what the agent wants to set
    /// `crosstrack_err`: current crosstrack error (m)
    /// `heading_err`: current heading error (rad)
    /// `now`: current time (seconds)
    ///
    /// Returns the action to take and the (possibly clamped) gains.
    pub fn check(
        &mut self,
        current_gains: &[f32; NUM_GAINS],
        proposed_gains: &mut [f32; NUM_GAINS],
        crosstrack_err: f32,
        heading_err: f32,
        now: f32,
    ) -> SafetyAction {
        // Frozen by operator
        if self.config.frozen {
            *proposed_gains = *current_gains;
            return SafetyAction::Frozen;
        }

        // Emergency: crosstrack or heading way off
        if libm::fabsf(crosstrack_err) > self.config.crosstrack_safety_threshold
            || libm::fabsf(heading_err) > self.config.heading_safety_threshold
        {
            self.violation_count += 1;
            if self.violation_count >= 3 {
                // 3 consecutive violations — revert to baseline
                *proposed_gains = self.baseline_gains;
                self.emergency_active = true;
                return SafetyAction::RevertToBaseline;
            }
        } else {
            self.violation_count = 0;
            self.emergency_active = false;
        }

        // Rate limiting: don't adjust faster than min_adjust_interval
        if now - self.last_adjust_time < self.config.min_adjust_interval {
            *proposed_gains = *current_gains;
            return SafetyAction::Allow; // no change, just holding
        }

        // Clamp each gain to safe range and limit adjustment rate
        let mut was_clamped = false;
        let dt = (now - self.last_adjust_time).max(0.01);

        for i in 0..NUM_GAINS {
            // Clamp to absolute bounds
            proposed_gains[i] = proposed_gains[i].clamp(
                self.config.gain_min[i],
                self.config.gain_max[i],
            );

            // Rate limit: max_adjust_rate fraction per second
            let max_delta = libm::fabsf(current_gains[i]) * self.config.max_adjust_rate * dt;
            let delta = proposed_gains[i] - current_gains[i];
            if libm::fabsf(delta) > max_delta {
                proposed_gains[i] = current_gains[i] + max_delta * if delta > 0.0 { 1.0 } else { -1.0 };
                was_clamped = true;
            }
        }

        self.last_adjust_time = now;

        if was_clamped { SafetyAction::Clamped } else { SafetyAction::Allow }
    }

    /// Freeze the agent (operator override).
    pub fn freeze(&mut self) { self.config.frozen = true; }

    /// Unfreeze the agent.
    pub fn unfreeze(&mut self) { self.config.frozen = false; }

    /// Set new baseline gains (e.g., after relay auto-tune).
    pub fn set_baseline(&mut self, gains: [f32; NUM_GAINS]) {
        self.baseline_gains = gains;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_gains() -> [f32; NUM_GAINS] {
        [0.2, 0.2, 0.0, 0.2, 0.2, 0.0]
    }

    #[test]
    fn test_normal_operation() {
        let mut monitor = SafetyMonitor::new(SafetyConfig::default(), default_gains());
        let current = default_gains();
        let mut proposed = [0.21, 0.2, 0.0, 0.2, 0.2, 0.0]; // small change
        let action = monitor.check(&current, &mut proposed, 1.0, 0.1, 1.0);
        assert_ne!(action, SafetyAction::RevertToBaseline);
    }

    #[test]
    fn test_emergency_revert() {
        let mut monitor = SafetyMonitor::new(SafetyConfig::default(), default_gains());
        let current = [1.0, 0.5, 0.1, 0.5, 0.3, 0.05];
        let mut proposed = current;

        // 3 consecutive violations
        for t in 0..3 {
            monitor.check(&current, &mut proposed, 15.0, 0.5, t as f32 * 2.0);
        }
        let action = monitor.check(&current, &mut proposed, 15.0, 0.5, 8.0);
        assert_eq!(action, SafetyAction::RevertToBaseline);
        assert_eq!(proposed, default_gains()); // reverted to baseline
    }

    #[test]
    fn test_frozen() {
        let mut monitor = SafetyMonitor::new(SafetyConfig::default(), default_gains());
        monitor.freeze();
        let current = default_gains();
        let mut proposed = [0.5, 0.5, 0.1, 0.5, 0.5, 0.1];
        let action = monitor.check(&current, &mut proposed, 0.5, 0.1, 1.0);
        assert_eq!(action, SafetyAction::Frozen);
        assert_eq!(proposed, current); // no change
    }

    #[test]
    fn test_rate_limiting() {
        let mut monitor = SafetyMonitor::new(SafetyConfig::default(), default_gains());
        let current = [0.2, 0.2, 0.0, 0.2, 0.2, 0.0];
        let mut proposed = [2.0, 2.0, 0.5, 2.0, 2.0, 0.5]; // huge jump
        let action = monitor.check(&current, &mut proposed, 0.5, 0.1, 2.0);
        assert_eq!(action, SafetyAction::Clamped);
        // Should be close to current, not jumped to 2.0
        assert!(proposed[0] < 0.25, "should be rate-limited: {}", proposed[0]);
    }
}
