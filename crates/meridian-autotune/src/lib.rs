#![no_std]

//! PID auto-tuning for all vehicle types.
//!
//! - Multirotor: twitch method (AC_AutoTune_Multi)
//! - Helicopter: frequency sweep (AC_AutoTune_Heli)
//! - USV/Boat: relay feedback for speed and heading axes

pub mod tuner;
pub mod usv_tune;

pub use tuner::{AutoTuner, TuneAxis, TuneStep, TuneState, TuneResults, MultiAxisAutoTuner, HeliAutoTuner};
pub use usv_tune::{UsvAutoTune, UsvTuneConfig, TunePhase, PidGains, RelayResult};

#[cfg(test)]
mod tests;
