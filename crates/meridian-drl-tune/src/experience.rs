//! Experience replay buffer for off-policy DRL training.
//!
//! Stores (state, action, reward, next_state) transitions.
//! Used by the SAC agent for batch training updates.
//! Fixed-size ring buffer — no heap allocation.

use crate::state::{AgentState, STATE_DIM, NUM_GAINS};

/// A single experience transition.
#[derive(Clone, Copy)]
pub struct Transition {
    pub state: AgentState,
    pub action: [f32; NUM_GAINS],   // gain adjustments applied
    pub reward: f32,
    pub next_state: AgentState,
    pub done: bool,                  // episode terminated (e.g., disarmed)
}

impl Transition {
    pub fn zero() -> Self {
        Self {
            state: AgentState::zero(),
            action: [0.0; NUM_GAINS],
            reward: 0.0,
            next_state: AgentState::zero(),
            done: false,
        }
    }
}

/// Fixed-size replay buffer.
///
/// Size 2048 transitions ≈ ~200KB at 100 bytes/transition.
/// At 10Hz agent update rate, this holds ~3.4 minutes of experience.
/// For longer retention, transitions should be flushed to storage periodically.
pub const BUFFER_SIZE: usize = 2048;

pub struct ReplayBuffer {
    buffer: [Transition; BUFFER_SIZE],
    head: usize,
    count: usize,
}

impl ReplayBuffer {
    pub fn new() -> Self {
        Self {
            buffer: [Transition::zero(); BUFFER_SIZE],
            head: 0,
            count: 0,
        }
    }

    /// Add a transition to the buffer.
    pub fn push(&mut self, t: Transition) {
        self.buffer[self.head] = t;
        self.head = (self.head + 1) % BUFFER_SIZE;
        if self.count < BUFFER_SIZE { self.count += 1; }
    }

    /// Number of transitions stored.
    pub fn len(&self) -> usize {
        self.count
    }

    /// Whether the buffer has enough data for a training batch.
    pub fn ready(&self, min_size: usize) -> bool {
        self.count >= min_size
    }

    /// Sample a batch of transitions using a deterministic seed.
    ///
    /// Returns indices into the buffer. The caller reads transitions at those indices.
    /// Uses a simple LCG PRNG for no_std compatibility.
    pub fn sample_indices(&self, batch_size: usize, seed: u32) -> [usize; 64] {
        let mut indices = [0usize; 64];
        let n = batch_size.min(64).min(self.count);
        let mut rng = seed;

        for i in 0..n {
            // LCG: next = (a * current + c) mod m
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
            let idx = ((rng >> 16) as usize) % self.count;
            // Map to actual buffer position
            let buf_idx = if self.count < BUFFER_SIZE {
                idx
            } else {
                (self.head + idx) % BUFFER_SIZE
            };
            indices[i] = buf_idx;
        }

        indices
    }

    /// Get a transition by buffer index.
    pub fn get(&self, idx: usize) -> &Transition {
        &self.buffer[idx % BUFFER_SIZE]
    }

    /// Clear all stored transitions.
    pub fn clear(&mut self) {
        self.head = 0;
        self.count = 0;
    }

    /// Get statistics about recent rewards (for monitoring).
    pub fn recent_reward_stats(&self, n: usize) -> (f32, f32) {
        let n = n.min(self.count);
        if n == 0 { return (0.0, 0.0); }
        let mut sum = 0.0f32;
        let mut sum_sq = 0.0f32;
        for i in 0..n {
            let idx = (self.head + BUFFER_SIZE - 1 - i) % BUFFER_SIZE;
            let r = self.buffer[idx].reward;
            sum += r;
            sum_sq += r * r;
        }
        let mean = sum / n as f32;
        let variance = sum_sq / n as f32 - mean * mean;
        (mean, libm::sqrtf(libm::fabsf(variance)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_push_and_len() {
        let mut buf = ReplayBuffer::new();
        assert_eq!(buf.len(), 0);
        buf.push(Transition::zero());
        assert_eq!(buf.len(), 1);
    }

    #[test]
    fn test_ring_buffer_wrap() {
        let mut buf = ReplayBuffer::new();
        for i in 0..(BUFFER_SIZE + 100) {
            let mut t = Transition::zero();
            t.reward = i as f32;
            buf.push(t);
        }
        assert_eq!(buf.len(), BUFFER_SIZE);
        // Most recent should have high reward
        let last = buf.get((buf.head + BUFFER_SIZE - 1) % BUFFER_SIZE);
        assert!(last.reward > BUFFER_SIZE as f32);
    }

    #[test]
    fn test_sample_indices() {
        let mut buf = ReplayBuffer::new();
        for _ in 0..100 {
            buf.push(Transition::zero());
        }
        let indices = buf.sample_indices(32, 42);
        // All indices should be valid
        for i in 0..32 {
            assert!(indices[i] < BUFFER_SIZE);
        }
    }

    #[test]
    fn test_reward_stats() {
        let mut buf = ReplayBuffer::new();
        for i in 0..50 {
            let mut t = Transition::zero();
            t.reward = 1.0; // constant reward
            buf.push(t);
        }
        let (mean, std) = buf.recent_reward_stats(50);
        assert!((mean - 1.0).abs() < 0.01);
        assert!(std < 0.01);
    }
}
