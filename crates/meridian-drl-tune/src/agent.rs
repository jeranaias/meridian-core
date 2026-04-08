//! SAC-inspired adaptive PID agent.
//!
//! This is a simplified Soft Actor-Critic that runs on embedded hardware.
//! Instead of full neural networks, it uses a compact linear policy
//! (state → action) that's small enough for a flight controller.
//!
//! Architecture:
//!   - Policy: linear map from 20-dim state to 6-dim action (gain adjustments)
//!     with tanh squashing to bound output to [-max_adjust, +max_adjust]
//!   - Value function: linear map from 20-dim state to scalar (estimated return)
//!   - Both are updated via gradient descent on mini-batches from replay buffer
//!
//! Why linear instead of neural network:
//!   - Runs on STM32H7 at 10Hz without GPU
//!   - 20×6 = 120 policy weights + 20 value weights = 140 parameters total
//!   - Interpretable: you can see which state features drive which gain adjustments
//!   - Stable: linear policies don't have the catastrophic forgetting problem
//!   - Can be pre-loaded with weights from simulation training
//!
//! For more complex environments, this can be upgraded to a 2-layer MLP
//! (20→32→6) at ~1200 parameters — still fits on embedded.

use crate::state::{AgentState, STATE_DIM, NUM_GAINS};
use crate::experience::{ReplayBuffer, Transition};
use crate::reward::{self, RewardWeights};
use crate::safety::{SafetyMonitor, SafetyConfig, SafetyAction};

/// Agent configuration.
#[derive(Clone, Copy)]
pub struct AgentConfig {
    /// Learning rate for policy updates. Default 0.001.
    pub policy_lr: f32,
    /// Learning rate for value function. Default 0.005.
    pub value_lr: f32,
    /// Discount factor (gamma). Default 0.99.
    pub gamma: f32,
    /// Entropy coefficient (alpha) — controls exploration vs exploitation.
    /// Higher = more exploration. Default 0.1.
    pub alpha: f32,
    /// Maximum fractional gain adjustment per step. Default 0.02 (2%).
    pub max_adjust: f32,
    /// Mini-batch size for training updates. Default 32.
    pub batch_size: usize,
    /// Minimum replay buffer size before training starts. Default 100.
    pub min_buffer_size: usize,
    /// How often to update (every N agent steps). Default 5.
    pub update_every: usize,
    /// Reward weights.
    pub reward_weights: RewardWeights,
}

impl Default for AgentConfig {
    fn default() -> Self {
        Self {
            policy_lr: 0.001,
            value_lr: 0.005,
            gamma: 0.99,
            alpha: 0.1,
            max_adjust: 0.02,
            batch_size: 32,
            min_buffer_size: 100,
            update_every: 5,
            reward_weights: RewardWeights::default(),
        }
    }
}

/// The adaptive PID agent.
///
/// Call `step()` at 10Hz with current telemetry. It returns gain adjustments
/// (or zeros if not yet ready to adjust). The adjustments are fractional:
/// new_gain = current_gain * (1 + adjustment).
pub struct AdaptiveAgent {
    config: AgentConfig,

    // Linear policy: W_policy[NUM_GAINS][STATE_DIM] + bias[NUM_GAINS]
    policy_w: [[f32; STATE_DIM]; NUM_GAINS],
    policy_b: [f32; NUM_GAINS],

    // Linear value function: W_value[STATE_DIM] + bias
    value_w: [f32; STATE_DIM],
    value_b: f32,

    // Experience replay
    pub replay: ReplayBuffer,

    // Safety
    pub safety: SafetyMonitor,

    // Running state
    step_count: u32,
    last_state: AgentState,
    last_action: [f32; NUM_GAINS],
    pub current_gains: [f32; NUM_GAINS],
    pub training_active: bool,

    // Statistics
    pub total_reward: f32,
    pub episodes: u32,
    pub updates: u32,
}

impl AdaptiveAgent {
    /// Create a new agent with initial gains from relay auto-tune.
    pub fn new(config: AgentConfig, initial_gains: [f32; NUM_GAINS]) -> Self {
        // Initialize policy weights to small random values
        // In practice, these would be loaded from simulation pre-training
        let mut policy_w = [[0.0f32; STATE_DIM]; NUM_GAINS];
        let policy_b = [0.0f32; NUM_GAINS];

        // Seed the policy with a sensible prior: respond to own error
        // Steering gains (indices 0-2) should respond to heading error (index 7)
        // Speed gains (indices 3-5) should respond to speed error (index 8)
        policy_w[0][7] = 0.01;  // steer_P responds to heading error
        policy_w[1][13] = 0.005; // steer_I responds to heading integral
        policy_w[2][10] = 0.005; // steer_D responds to heading derivative
        policy_w[3][8] = 0.01;  // speed_P responds to speed error
        policy_w[4][14] = 0.005; // speed_I responds to speed integral
        policy_w[5][11] = 0.005; // speed_D responds to speed derivative

        let safety_config = SafetyConfig::default();
        let safety = SafetyMonitor::new(safety_config, initial_gains);

        Self {
            config,
            policy_w,
            policy_b,
            value_w: [0.0; STATE_DIM],
            value_b: 0.0,
            replay: ReplayBuffer::new(),
            safety,
            step_count: 0,
            last_state: AgentState::zero(),
            last_action: [0.0; NUM_GAINS],
            current_gains: initial_gains,
            training_active: true,
            total_reward: 0.0,
            episodes: 0,
            updates: 0,
        }
    }

    /// Load pre-trained weights from simulation.
    pub fn load_weights(
        &mut self,
        policy_w: [[f32; STATE_DIM]; NUM_GAINS],
        policy_b: [f32; NUM_GAINS],
        value_w: [f32; STATE_DIM],
        value_b: f32,
    ) {
        self.policy_w = policy_w;
        self.policy_b = policy_b;
        self.value_w = value_w;
        self.value_b = value_b;
    }

    /// Run one agent step. Call at 10Hz.
    ///
    /// Returns the new gains to apply (after safety checks).
    pub fn step(
        &mut self,
        state: &AgentState,
        crosstrack_err: f32,
        heading_err: f32,
        speed_err: f32,
        throttle: f32,
        target_speed: f32,
        actual_speed: f32,
        now_s: f32,
    ) -> [f32; NUM_GAINS] {
        self.step_count += 1;

        // Compute reward for previous step
        let sea = [0.0f32; 2]; // TODO: get from observation window
        let gain_change = self.last_action.iter().map(|a| libm::fabsf(*a)).sum::<f32>();
        let r = reward::compute_reward(
            &self.config.reward_weights,
            libm::fabsf(crosstrack_err),
            libm::fabsf(heading_err),
            libm::fabsf(speed_err),
            sea[0], sea[1],
            gain_change,
            throttle,
            target_speed,
            actual_speed,
        );
        self.total_reward += r;

        // Store transition
        if self.step_count > 1 {
            self.replay.push(Transition {
                state: self.last_state,
                action: self.last_action,
                reward: r,
                next_state: *state,
                done: false,
            });
        }

        // Compute action (policy forward pass)
        let raw_action = self.policy_forward(state);

        // Scale to fractional adjustments — guard against NaN
        let mut adjustments = [0.0f32; NUM_GAINS];
        for i in 0..NUM_GAINS {
            let a = tanh(raw_action[i]) * self.config.max_adjust;
            adjustments[i] = if a.is_finite() { a } else { 0.0 };
        }

        // Apply adjustments to current gains
        let mut proposed = self.current_gains;
        for i in 0..NUM_GAINS {
            proposed[i] = self.current_gains[i] * (1.0 + adjustments[i]);
        }

        // Safety check
        let action = self.safety.check(
            &self.current_gains,
            &mut proposed,
            crosstrack_err,
            heading_err,
            now_s,
        );

        match action {
            SafetyAction::RevertToBaseline => {
                // Emergency — gains already set to baseline by safety monitor
            }
            SafetyAction::Frozen => {
                // No changes allowed
            }
            _ => {
                // Apply (possibly clamped) gains
                self.current_gains = proposed;
            }
        }

        // Train periodically
        if self.training_active
            && self.step_count % self.config.update_every as u32 == 0
            && self.replay.ready(self.config.min_buffer_size)
        {
            self.train_step();
        }

        // Remember for next step
        self.last_state = *state;
        self.last_action = adjustments;

        self.current_gains
    }

    /// Policy forward pass: state → raw action logits.
    fn policy_forward(&self, state: &AgentState) -> [f32; NUM_GAINS] {
        let mut out = [0.0f32; NUM_GAINS];
        for i in 0..NUM_GAINS {
            let mut sum = self.policy_b[i];
            for j in 0..STATE_DIM {
                sum += self.policy_w[i][j] * state.data[j];
            }
            out[i] = sum;
        }
        out
    }

    /// Value function forward pass: state → scalar value estimate.
    fn value_forward(&self, state: &AgentState) -> f32 {
        let mut sum = self.value_b;
        for j in 0..STATE_DIM {
            sum += self.value_w[j] * state.data[j];
        }
        sum
    }

    /// One training step: sample batch, compute gradients, update weights.
    fn train_step(&mut self) {
        let batch_size = self.config.batch_size.min(64);
        let indices = self.replay.sample_indices(batch_size, self.step_count);
        let n = batch_size.min(self.replay.len());

        // Compute value targets and policy gradients
        let mut policy_grad = [[0.0f32; STATE_DIM]; NUM_GAINS];
        let mut policy_b_grad = [0.0f32; NUM_GAINS];
        let mut value_grad = [0.0f32; STATE_DIM];
        let mut value_b_grad = 0.0f32;

        for k in 0..n {
            let t = self.replay.get(indices[k]);

            // TD target: r + gamma * V(s')
            let v_next = if t.done { 0.0 } else { self.value_forward(&t.next_state) };
            let td_target = t.reward + self.config.gamma * v_next;
            let v_current = self.value_forward(&t.state);
            let td_error = td_target - v_current;

            // Value gradient: dL/dw = -td_error * state
            for j in 0..STATE_DIM {
                value_grad[j] += -td_error * t.state.data[j];
            }
            value_b_grad += -td_error;

            // Policy gradient: advantage-weighted
            // Simplified: if TD error positive (action was better than expected), reinforce
            let advantage = td_error;
            let action = self.policy_forward(&t.state);
            for i in 0..NUM_GAINS {
                let act_grad = 1.0 - tanh(action[i]) * tanh(action[i]); // dtanh/dx
                for j in 0..STATE_DIM {
                    policy_grad[i][j] += -advantage * act_grad * t.state.data[j];
                }
                policy_b_grad[i] += -advantage * act_grad;

                // Entropy bonus: encourage exploration
                policy_b_grad[i] += self.config.alpha * tanh(action[i]);
            }
        }

        let inv_n = 1.0 / n as f32;

        // Update value weights
        for j in 0..STATE_DIM {
            self.value_w[j] -= self.config.value_lr * value_grad[j] * inv_n;
        }
        self.value_b -= self.config.value_lr * value_b_grad * inv_n;

        // Update policy weights
        for i in 0..NUM_GAINS {
            for j in 0..STATE_DIM {
                self.policy_w[i][j] -= self.config.policy_lr * policy_grad[i][j] * inv_n;
            }
            self.policy_b[i] -= self.config.policy_lr * policy_b_grad[i] * inv_n;
        }

        self.updates += 1;
    }

    /// Signal end of episode (e.g., vehicle disarmed).
    pub fn end_episode(&mut self) {
        // Mark last transition as done
        if self.replay.len() > 0 {
            // Can't modify the last pushed item easily in a ring buffer,
            // but the reward for the last step won't have a next_state anyway.
        }
        self.episodes += 1;
    }

    /// Get agent statistics for display.
    pub fn stats(&self) -> AgentStats {
        let (mean_r, std_r) = self.replay.recent_reward_stats(100);
        AgentStats {
            step_count: self.step_count,
            episodes: self.episodes,
            updates: self.updates,
            replay_size: self.replay.len(),
            mean_reward: mean_r,
            std_reward: std_r,
            total_reward: self.total_reward,
            training_active: self.training_active,
            emergency_active: self.safety.emergency_active,
            current_gains: self.current_gains,
        }
    }

    /// Freeze/unfreeze gain adjustments.
    pub fn set_frozen(&mut self, frozen: bool) {
        if frozen { self.safety.freeze(); } else { self.safety.unfreeze(); }
    }

    /// Whether the agent is actively adjusting gains.
    pub fn is_adjusting(&self) -> bool {
        self.training_active && !self.safety.config.frozen && !self.safety.emergency_active
    }
}

/// Agent statistics for monitoring/display.
#[derive(Clone, Copy)]
pub struct AgentStats {
    pub step_count: u32,
    pub episodes: u32,
    pub updates: u32,
    pub replay_size: usize,
    pub mean_reward: f32,
    pub std_reward: f32,
    pub total_reward: f32,
    pub training_active: bool,
    pub emergency_active: bool,
    pub current_gains: [f32; NUM_GAINS],
}

/// Fast tanh approximation for no_std.
fn tanh(x: f32) -> f32 {
    // Padé approximation: accurate to ~0.001 for |x| < 3
    if x > 3.0 { return 1.0; }
    if x < -3.0 { return -1.0; }
    let x2 = x * x;
    x * (27.0 + x2) / (27.0 + 9.0 * x2)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn initial_gains() -> [f32; NUM_GAINS] {
        [0.2, 0.2, 0.0, 0.2, 0.2, 0.0]
    }

    #[test]
    fn test_agent_creation() {
        let agent = AdaptiveAgent::new(AgentConfig::default(), initial_gains());
        assert_eq!(agent.current_gains, initial_gains());
        assert_eq!(agent.step_count, 0);
    }

    #[test]
    fn test_agent_step() {
        let mut agent = AdaptiveAgent::new(AgentConfig::default(), initial_gains());
        let state = AgentState::zero();
        let gains = agent.step(&state, 1.0, 0.1, 0.2, 0.5, 3.0, 2.8, 1.0);
        assert_eq!(gains.len(), NUM_GAINS);
        assert!(agent.step_count == 1);
    }

    #[test]
    fn test_agent_training() {
        let mut agent = AdaptiveAgent::new(
            AgentConfig { min_buffer_size: 10, update_every: 1, ..Default::default() },
            initial_gains(),
        );

        // Fill replay buffer
        for i in 0..50 {
            let mut state = AgentState::zero();
            state.data[7] = 0.1 * (i as f32).sin(); // varying heading error
            agent.step(&state, 1.0, 0.1, 0.2, 0.5, 3.0, 2.8, i as f32 * 0.1);
        }

        assert!(agent.updates > 0, "should have trained: updates={}", agent.updates);
        assert!(agent.replay.len() >= 49);
    }

    #[test]
    fn test_tanh_approx() {
        assert!((tanh(0.0)).abs() < 0.001);
        assert!((tanh(1.0) - 0.762).abs() < 0.02);
        assert!((tanh(-1.0) + 0.762).abs() < 0.02);
        assert!((tanh(5.0) - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_gains_bounded() {
        let mut agent = AdaptiveAgent::new(AgentConfig::default(), initial_gains());
        // Run many steps — gains should stay bounded
        for i in 0..200 {
            let mut state = AgentState::zero();
            state.data[7] = 5.0; // large heading error
            agent.step(&state, 10.0, 1.0, 2.0, 0.8, 3.0, 1.0, i as f32 * 0.1);
        }
        for g in &agent.current_gains {
            assert!(*g >= 0.0 && *g <= 10.0, "gain out of bounds: {}", g);
        }
    }

    #[test]
    fn test_stats() {
        let agent = AdaptiveAgent::new(AgentConfig::default(), initial_gains());
        let stats = agent.stats();
        assert_eq!(stats.step_count, 0);
        assert!(stats.training_active);
        assert!(!stats.emergency_active);
    }
}
