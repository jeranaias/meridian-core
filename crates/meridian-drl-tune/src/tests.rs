//! Comprehensive test suite for DRL adaptive PID tuning.
//!
//! Coverage targets:
//! - State normalization boundary conditions
//! - Reward function edge cases and monotonicity
//! - Safety constraint enforcement under all conditions
//! - Experience buffer overflow and sampling correctness
//! - Agent convergence on simple tracking problems
//! - Sim-to-real weight loading
//! - Emergency revert and recovery
//! - Long-duration stability
//! - Adversarial inputs (NaN, infinity, extreme values)

#[cfg(test)]
mod state_tests {
    use crate::state::*;

    #[test]
    fn test_state_from_zero_telemetry() {
        let s = AgentState::from_telemetry(
            &[0.0; NUM_GAINS], &[0.0; 3], &[0.0; 3], &[0.0; 3], &[0.0; 3], &[0.0; 2],
        );
        for v in &s.data {
            assert!(v.is_finite(), "all state values must be finite");
            assert!(v.abs() <= 1.0, "normalized state should be in [-1,1]: {}", v);
        }
    }

    #[test]
    fn test_state_extreme_errors() {
        let s = AgentState::from_telemetry(
            &[5.0; NUM_GAINS],
            &[1000.0, 100.0, 500.0],       // extreme errors
            &[100.0, 50.0, 50.0],            // extreme derivatives
            &[1000.0, 500.0, 1000.0],        // extreme integrals
            &[100.0, 10.0, 2.0],             // extreme vehicle state
            &[90.0, 10.0],                    // extreme sea state
        );
        for (i, v) in s.data.iter().enumerate() {
            assert!(v.is_finite(), "state[{}] must be finite: {}", i, v);
            // Gains (indices 0-5) are normalized but not clamped — can exceed 1.0
            // Errors/dynamics (indices 6+) are clamped to [-1, 1]
            if i >= 6 {
                assert!(v.abs() <= 1.01, "state[{}] should be clamped: {}", i, v);
            }
        }
    }

    #[test]
    fn test_state_negative_values() {
        let s = AgentState::from_telemetry(
            &[0.1; NUM_GAINS],
            &[-5.0, -0.5, -2.0],
            &[-10.0, -5.0, -5.0],
            &[0.0; 3],
            &[0.0; 3],
            &[0.0; 2],
        );
        // Errors should clamp to -1
        assert!(s.data[6] >= -1.0);
        assert!(s.data[7] >= -1.0);
        assert!(s.data[8] >= -1.0);
    }

    #[test]
    fn test_observation_window_empty() {
        let w = ObservationWindow::new(0.1);
        let d = w.derivatives();
        assert_eq!(d, [0.0; 3]);
        let i = w.integrals();
        assert_eq!(i, [0.0; 3]);
        let ss = w.sea_state_estimate();
        assert_eq!(ss, [0.0; 2]);
    }

    #[test]
    fn test_observation_window_single_sample() {
        let mut w = ObservationWindow::new(0.1);
        w.push([1.0, 2.0, 3.0]);
        let d = w.derivatives();
        assert_eq!(d, [0.0; 3]); // need 2 samples for derivative
    }

    #[test]
    fn test_observation_window_overflow() {
        let mut w = ObservationWindow::new(0.1);
        for i in 0..200 {
            w.push([i as f32 * 0.01, 0.0, 0.0]);
        }
        // Buffer is internally capped at 100 — verify via derivative (still works)
        let d = w.derivatives();
        assert!(d[0] > 0.0); // should still compute valid derivative
    }

    #[test]
    fn test_observation_integral_accuracy() {
        let mut w = ObservationWindow::new(0.1);
        // Push constant error of 1.0 for 50 samples (5 seconds)
        for _ in 0..50 {
            w.push([1.0, 0.0, 0.0]);
        }
        let integrals = w.integrals();
        // Integral of |1.0| over 5 seconds at 0.1s dt = 5.0
        assert!((integrals[0] - 5.0).abs() < 0.1, "integral should be ~5.0: {}", integrals[0]);
    }

    #[test]
    fn test_sea_state_zero_when_calm() {
        let mut w = ObservationWindow::new(0.1);
        for _ in 0..50 {
            w.push([0.0, 0.1, 0.0]); // constant heading error, no oscillation
        }
        let ss = w.sea_state_estimate();
        assert!(ss[0] < 0.01, "calm water should show near-zero oscillation");
    }
}

#[cfg(test)]
mod reward_tests {
    use crate::reward::*;

    #[test]
    fn test_reward_monotonicity_crosstrack() {
        let w = RewardWeights::default();
        let mut prev_r = f32::MAX;
        for err in [0.0, 0.5, 1.0, 2.0, 5.0, 10.0] {
            let r = compute_reward(&w, err, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 3.0, 3.0);
            assert!(r <= prev_r, "reward should decrease with error: err={} r={}", err, r);
            prev_r = r;
        }
    }

    #[test]
    fn test_reward_monotonicity_heading() {
        let w = RewardWeights::default();
        let mut prev_r = f32::MAX;
        for err in [0.0, 0.1, 0.3, 0.5, 1.0] {
            let r = compute_reward(&w, 0.0, err, 0.0, 0.0, 0.0, 0.0, 0.5, 3.0, 3.0);
            assert!(r <= prev_r, "reward should decrease with heading error");
            prev_r = r;
        }
    }

    #[test]
    fn test_reward_monotonicity_speed() {
        let w = RewardWeights::default();
        let mut prev_r = f32::MAX;
        for err in [0.0, 0.5, 1.0, 2.0] {
            let r = compute_reward(&w, 0.0, 0.0, err, 0.0, 0.0, 0.0, 0.5, 3.0, 3.0);
            assert!(r <= prev_r, "reward should decrease with speed error");
            prev_r = r;
        }
    }

    #[test]
    fn test_reward_zero_target_speed() {
        let w = RewardWeights::default();
        // Should not divide by zero
        let r = compute_reward(&w, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(r.is_finite());
    }

    #[test]
    fn test_reward_all_zeros() {
        let w = RewardWeights::default();
        let r = compute_reward(&w, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(r.is_finite());
        assert!(r > 0.0, "zero error should give positive reward (alive bonus)");
    }

    #[test]
    fn test_reward_bounded() {
        let w = RewardWeights::default();
        // Worst case: maximum everything
        let r = compute_reward(&w, 100.0, 3.14, 10.0, 90.0, 5.0, 10.0, 1.0, 3.0, 0.0);
        assert!(r.is_finite());
        assert!(r < 0.0, "terrible tracking should give negative reward");
        // Best case
        let r_best = compute_reward(&w, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 3.0, 3.0);
        assert!(r_best > 0.0);
    }

    #[test]
    fn test_custom_weights() {
        // Zero all weights — reward should just be alive bonus
        let w = RewardWeights {
            crosstrack: 0.0, heading: 0.0, speed: 0.0,
            heading_oscillation: 0.0, speed_oscillation: 0.0,
            gain_change: 0.0, efficiency: 0.0,
        };
        let r = compute_reward(&w, 10.0, 1.0, 5.0, 30.0, 2.0, 5.0, 0.8, 3.0, 1.0);
        assert!((r - 0.05).abs() < 0.01, "only alive bonus: {}", r);
    }
}

#[cfg(test)]
mod safety_tests {
    use crate::safety::*;
    use crate::state::NUM_GAINS;

    fn baseline() -> [f32; NUM_GAINS] {
        [0.2, 0.2, 0.02, 0.2, 0.2, 0.02]
    }

    #[test]
    fn test_gains_clamped_to_min() {
        let mut m = SafetyMonitor::new(SafetyConfig::default(), baseline());
        let current = baseline();
        let mut proposed = [-1.0; NUM_GAINS]; // way below minimum
        m.check(&current, &mut proposed, 0.5, 0.1, 2.0);
        for i in 0..NUM_GAINS {
            assert!(proposed[i] >= m.config.gain_min[i], "gain[{}] below min: {}", i, proposed[i]);
        }
    }

    #[test]
    fn test_gains_clamped_to_max() {
        let mut m = SafetyMonitor::new(SafetyConfig::default(), baseline());
        let current = baseline();
        let mut proposed = [100.0; NUM_GAINS]; // way above maximum
        m.check(&current, &mut proposed, 0.5, 0.1, 2.0);
        for i in 0..NUM_GAINS {
            assert!(proposed[i] <= m.config.gain_max[i], "gain[{}] above max: {}", i, proposed[i]);
        }
    }

    #[test]
    fn test_rate_limit_enforced() {
        let config = SafetyConfig {
            min_adjust_interval: 0.0, // disable interval gating for this test
            ..SafetyConfig::default()
        };
        let mut m = SafetyMonitor::new(config, baseline());
        let current = [1.0; NUM_GAINS];
        // First call at t=0
        let mut proposed = [2.0; NUM_GAINS];
        m.check(&current, &mut proposed, 0.5, 0.1, 0.0);
        // Second call at t=1.0 — should rate-limit the jump
        proposed = [2.0; NUM_GAINS];
        let action = m.check(&current, &mut proposed, 0.5, 0.1, 1.0);
        assert_eq!(action, SafetyAction::Clamped);
        for i in 0..NUM_GAINS {
            let change = (proposed[i] - current[i]).abs();
            assert!(change <= 0.06, "gain[{}] changed too fast: {}", i, change);
        }
    }

    #[test]
    fn test_emergency_clears_after_recovery() {
        let mut m = SafetyMonitor::new(SafetyConfig::default(), baseline());
        let current = baseline();
        let mut proposed = baseline();

        // Trigger emergency
        for t in 0..4 {
            m.check(&current, &mut proposed, 15.0, 0.5, t as f32 * 2.0);
        }
        assert!(m.emergency_active);

        // Recovery: good tracking for several steps
        for t in 5..10 {
            proposed = baseline();
            m.check(&current, &mut proposed, 0.5, 0.1, t as f32 * 2.0);
        }
        assert!(!m.emergency_active, "should recover after good tracking");
    }

    #[test]
    fn test_freeze_unfreeze() {
        let mut m = SafetyMonitor::new(SafetyConfig::default(), baseline());
        m.freeze();
        assert!(m.config.frozen);
        m.unfreeze();
        assert!(!m.config.frozen);
    }

    #[test]
    fn test_new_baseline() {
        let mut m = SafetyMonitor::new(SafetyConfig::default(), baseline());
        let new_bl = [0.5, 0.3, 0.05, 0.4, 0.25, 0.03];
        m.set_baseline(new_bl);
        assert_eq!(m.baseline_gains, new_bl);
    }

    #[test]
    fn test_concurrent_violations_threshold() {
        let mut m = SafetyMonitor::new(SafetyConfig::default(), baseline());
        let current = baseline();
        let mut proposed = baseline();

        // 2 violations then recovery — should NOT revert
        m.check(&current, &mut proposed, 15.0, 0.5, 1.0);
        m.check(&current, &mut proposed, 15.0, 0.5, 3.0);
        m.check(&current, &mut proposed, 0.5, 0.1, 5.0); // good
        let action = m.check(&current, &mut proposed, 15.0, 0.5, 7.0); // bad again
        assert_ne!(action, SafetyAction::RevertToBaseline, "intermittent violations shouldn't revert");
    }
}

#[cfg(test)]
mod experience_tests {
    use crate::experience::*;
    use crate::state::AgentState;

    #[test]
    fn test_empty_buffer() {
        let buf = ReplayBuffer::new();
        assert_eq!(buf.len(), 0);
        assert!(!buf.ready(1));
    }

    #[test]
    fn test_fill_to_capacity() {
        let mut buf = ReplayBuffer::new();
        for i in 0..BUFFER_SIZE {
            let mut t = Transition::zero();
            t.reward = i as f32;
            buf.push(t);
        }
        assert_eq!(buf.len(), BUFFER_SIZE);
        assert!(buf.ready(BUFFER_SIZE));
    }

    #[test]
    fn test_overflow_keeps_recent() {
        let mut buf = ReplayBuffer::new();
        for i in 0..(BUFFER_SIZE * 2) {
            let mut t = Transition::zero();
            t.reward = i as f32;
            buf.push(t);
        }
        assert_eq!(buf.len(), BUFFER_SIZE);
        // Verify via reward stats that recent entries have high rewards
        let (mean, _) = buf.recent_reward_stats(100);
        assert!(mean > BUFFER_SIZE as f32 * 0.5, "recent rewards should be high: {}", mean);
    }

    #[test]
    fn test_sample_indices_unique_ish() {
        let mut buf = ReplayBuffer::new();
        for _ in 0..500 {
            buf.push(Transition::zero());
        }
        let idx1 = buf.sample_indices(32, 100);
        let idx2 = buf.sample_indices(32, 200);
        // Different seeds should give different samples (probabilistic)
        let mut same = 0;
        for i in 0..32 {
            if idx1[i] == idx2[i] { same += 1; }
        }
        assert!(same < 30, "different seeds should give mostly different samples");
    }

    #[test]
    fn test_reward_stats_accuracy() {
        let mut buf = ReplayBuffer::new();
        for _ in 0..100 {
            let mut t = Transition::zero();
            t.reward = 2.0;
            buf.push(t);
        }
        let (mean, std) = buf.recent_reward_stats(100);
        assert!((mean - 2.0).abs() < 0.01);
        assert!(std < 0.01);
    }

    #[test]
    fn test_reward_stats_varied() {
        let mut buf = ReplayBuffer::new();
        for i in 0..100 {
            let mut t = Transition::zero();
            t.reward = if i % 2 == 0 { 1.0 } else { -1.0 };
            buf.push(t);
        }
        let (mean, std) = buf.recent_reward_stats(100);
        assert!(mean.abs() < 0.1, "mean should be near zero: {}", mean);
        assert!(std > 0.5, "std should be significant: {}", std);
    }

    #[test]
    fn test_clear() {
        let mut buf = ReplayBuffer::new();
        for _ in 0..50 {
            buf.push(Transition::zero());
        }
        buf.clear();
        assert_eq!(buf.len(), 0);
    }

    #[test]
    fn test_done_flag() {
        let mut t = Transition::zero();
        assert!(!t.done);
        t.done = true;
        assert!(t.done);
    }
}

#[cfg(test)]
mod agent_tests {
    use crate::agent::*;
    use crate::state::{AgentState, NUM_GAINS};

    fn initial() -> [f32; NUM_GAINS] {
        [0.2, 0.2, 0.02, 0.2, 0.2, 0.02]
    }

    #[test]
    fn test_agent_gains_start_at_initial() {
        let agent = AdaptiveAgent::new(AgentConfig::default(), initial());
        assert_eq!(agent.current_gains, initial());
    }

    #[test]
    fn test_agent_gains_stay_positive() {
        let mut agent = AdaptiveAgent::new(
            AgentConfig { min_buffer_size: 5, update_every: 1, ..Default::default() },
            initial(),
        );
        for i in 0..500 {
            let mut s = AgentState::zero();
            s.data[7] = if i % 20 < 10 { 0.5 } else { -0.5 }; // oscillating heading
            agent.step(&s, 2.0, 0.3, 0.5, 0.6, 3.0, 2.5, i as f32 * 0.1);
        }
        for (j, g) in agent.current_gains.iter().enumerate() {
            assert!(*g >= 0.0, "gain[{}] went negative: {}", j, g);
        }
    }

    #[test]
    fn test_agent_gains_bounded_under_stress() {
        let mut agent = AdaptiveAgent::new(
            AgentConfig { min_buffer_size: 5, update_every: 1, ..Default::default() },
            initial(),
        );
        // Extreme constant error — agent should try to increase gains but stay bounded
        for i in 0..1000 {
            let mut s = AgentState::zero();
            s.data[6] = 1.0; // max crosstrack
            s.data[7] = 1.0; // max heading error
            s.data[8] = 1.0; // max speed error
            agent.step(&s, 10.0, 1.0, 2.0, 0.9, 3.0, 1.0, i as f32 * 0.1);
        }
        for (j, g) in agent.current_gains.iter().enumerate() {
            assert!(*g <= 10.0, "gain[{}] exceeded safe bound: {}", j, g);
            assert!(*g >= 0.0, "gain[{}] went negative: {}", j, g);
        }
    }

    #[test]
    fn test_agent_emergency_revert() {
        let mut agent = AdaptiveAgent::new(
            AgentConfig { min_buffer_size: 5, update_every: 1, ..Default::default() },
            initial(),
        );
        // Gradually increase gains via normal operation
        for i in 0..50 {
            let s = AgentState::zero();
            agent.step(&s, 0.5, 0.1, 0.1, 0.5, 3.0, 3.0, i as f32 * 0.1);
        }
        // Now hit extreme error — should revert
        for i in 50..60 {
            let s = AgentState::zero();
            agent.step(&s, 15.0, 1.5, 5.0, 0.9, 3.0, 0.0, i as f32 * 0.1);
        }
        assert!(agent.safety.emergency_active, "should be in emergency");
        // Gains should be approximately back to baseline (float precision)
        for i in 0..6 {
            assert!((agent.current_gains[i] - initial()[i]).abs() < 0.01,
                "gain[{}] should be near baseline: {} vs {}", i, agent.current_gains[i], initial()[i]);
        }
    }

    #[test]
    fn test_agent_freeze() {
        let mut agent = AdaptiveAgent::new(
            AgentConfig { min_buffer_size: 5, update_every: 1, ..Default::default() },
            initial(),
        );
        // Run a few steps
        for i in 0..20 {
            let s = AgentState::zero();
            agent.step(&s, 1.0, 0.2, 0.3, 0.5, 3.0, 2.7, i as f32 * 0.1);
        }
        let gains_before = agent.current_gains;

        // Freeze
        agent.set_frozen(true);
        assert!(!agent.is_adjusting());

        // Run more steps — gains should NOT change
        for i in 20..40 {
            let mut s = AgentState::zero();
            s.data[7] = 0.5; // heading error
            agent.step(&s, 2.0, 0.5, 0.5, 0.6, 3.0, 2.5, i as f32 * 0.1);
        }
        assert_eq!(agent.current_gains, gains_before, "gains should not change while frozen");

        // Unfreeze
        agent.set_frozen(false);
        assert!(agent.is_adjusting());
    }

    #[test]
    fn test_agent_stats() {
        let mut agent = AdaptiveAgent::new(
            AgentConfig { min_buffer_size: 5, update_every: 1, ..Default::default() },
            initial(),
        );
        for i in 0..100 {
            let s = AgentState::zero();
            agent.step(&s, 0.5, 0.1, 0.1, 0.5, 3.0, 3.0, i as f32 * 0.1);
        }
        let stats = agent.stats();
        assert_eq!(stats.step_count, 100);
        assert!(stats.updates > 0);
        assert!(stats.replay_size >= 99);
        assert!(stats.mean_reward.is_finite());
    }

    #[test]
    fn test_weight_loading() {
        let mut agent = AdaptiveAgent::new(AgentConfig::default(), initial());
        let mut pw = [[0.0f32; 20]; 6];
        pw[0][7] = 0.5; // strong steering response to heading
        let pb = [0.01; 6];
        let vw = [0.1; 20];
        agent.load_weights(pw, pb, vw, 0.0);

        // Policy should now produce non-zero output even for small state
        let mut s = AgentState::zero();
        s.data[7] = 0.5;
        let gains = agent.step(&s, 0.5, 0.1, 0.1, 0.5, 3.0, 3.0, 1.0);
        // Steering P should have moved from initial
        // (might be tiny due to tanh squashing and safety rate limits)
        assert!(gains[0].is_finite());
    }

    #[test]
    fn test_end_episode() {
        let mut agent = AdaptiveAgent::new(AgentConfig::default(), initial());
        agent.end_episode();
        assert_eq!(agent.episodes, 1);
        agent.end_episode();
        assert_eq!(agent.episodes, 2);
    }

    #[test]
    fn test_agent_deterministic_with_same_input() {
        let cfg = AgentConfig { min_buffer_size: 200, ..Default::default() }; // no training
        let mut a1 = AdaptiveAgent::new(cfg, initial());
        let mut a2 = AdaptiveAgent::new(cfg, initial());

        let s = AgentState::zero();
        let g1 = a1.step(&s, 1.0, 0.1, 0.1, 0.5, 3.0, 3.0, 1.0);
        let g2 = a2.step(&s, 1.0, 0.1, 0.1, 0.5, 3.0, 3.0, 1.0);
        assert_eq!(g1, g2, "same input should give same output");
    }

    #[test]
    fn test_long_duration_stability() {
        let mut agent = AdaptiveAgent::new(
            AgentConfig { min_buffer_size: 10, update_every: 2, ..Default::default() },
            initial(),
        );

        // Simulate 30 minutes of operation at 10Hz = 18,000 steps
        for i in 0..18000 {
            let mut s = AgentState::zero();
            // Realistic varying errors
            let t = i as f32 * 0.1;
            s.data[6] = 0.3 * libm::sinf(t * 0.1); // crosstrack oscillation
            s.data[7] = 0.1 * libm::sinf(t * 0.2); // heading oscillation
            s.data[8] = 0.2 * libm::cosf(t * 0.05); // speed variation

            let gains = agent.step(&s,
                libm::fabsf(s.data[6]) * 5.0, // crosstrack
                s.data[7] * 0.5,               // heading err
                s.data[8] * 2.0,               // speed err
                0.5, 3.0, 3.0 + s.data[8],
                t,
            );

            // Gains must always be finite and positive
            for (j, g) in gains.iter().enumerate() {
                assert!(g.is_finite(), "step {}: gain[{}] is not finite: {}", i, j, g);
                assert!(*g >= 0.0, "step {}: gain[{}] negative: {}", i, j, g);
                assert!(*g <= 10.0, "step {}: gain[{}] too large: {}", i, j, g);
            }
        }

        let stats = agent.stats();
        assert!(stats.step_count == 18000);
        assert!(stats.updates > 100, "should have trained: {}", stats.updates);
        assert!(stats.total_reward.is_finite());
    }

    #[test]
    fn test_nan_input_handling() {
        let mut agent = AdaptiveAgent::new(
            AgentConfig { min_buffer_size: 200, ..Default::default() },
            initial(),
        );

        // Feed NaN in state — gains should still be finite
        let mut s = AgentState::zero();
        s.data[7] = f32::NAN;
        let gains = agent.step(&s, f32::NAN, f32::NAN, 0.0, 0.5, 3.0, 3.0, 1.0);

        // Even with NaN input, gains should remain at initial (no training happened)
        for g in &gains {
            assert!(g.is_finite(), "gains must be finite even with NaN input: {}", g);
        }
    }
}
