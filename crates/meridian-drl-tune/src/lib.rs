#![no_std]

//! Deep Reinforcement Learning adaptive PID tuning for USVs.
//!
//! Two-stage system:
//! 1. Relay feedback for initial 60-second bootstrap (in meridian-autotune)
//! 2. SAC-inspired continuous adaptive agent (this crate)
//!
//! The agent observes tracking performance and adjusts PID gains in real-time.
//! Pre-trained in simulation, fine-tuned on real vehicle, continuously learning.

pub mod state;
pub mod agent;
pub mod reward;
pub mod safety;
pub mod experience;
