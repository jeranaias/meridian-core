//! USV (Unmanned Surface Vehicle) auto-tuning.
//!
//! Two-axis tuning for boats:
//!   1. Speed axis: throttle step → measure speed response → compute speed PID
//!   2. Heading axis: rudder/differential step → measure yaw response → compute turn PID
//!
//! Method: Relay feedback (Åström-Hägglund) with step response characterization.
//!
//! The relay method oscillates the system around the setpoint using bang-bang control,
//! measures the ultimate gain (Ku) and ultimate period (Tu) from the resulting
//! oscillation, then computes PID gains using Ziegler-Nichols or Tyreus-Luyben rules.
//!
//! For USVs this is much simpler than copter autotune because:
//! - Only 2 axes (speed + heading) vs 3 rate axes
//! - Lower bandwidth (boats respond in seconds, not milliseconds)
//! - Step response is practical (boats can do speed steps on water)

/// Tuning state machine phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TunePhase {
    /// Waiting for operator to start tuning.
    Idle,
    /// Measuring speed axis response.
    SpeedStep,
    /// Relay feedback on speed axis to find Ku and Tu.
    SpeedRelay,
    /// Measuring heading axis response.
    HeadingStep,
    /// Relay feedback on heading axis to find Ku and Tu.
    HeadingRelay,
    /// Computing final gains.
    Computing,
    /// Tuning complete — gains stored.
    Complete,
    /// Tuning failed.
    Failed,
}

/// Results from a relay feedback test on one axis.
#[derive(Debug, Clone, Copy, Default)]
pub struct RelayResult {
    /// Ultimate gain — the relay amplitude that produces sustained oscillation.
    pub ultimate_gain: f32,
    /// Ultimate period — the period of the sustained oscillation (seconds).
    pub ultimate_period: f32,
    /// Number of complete oscillation cycles measured.
    pub cycles: u16,
    /// Whether the result is valid (enough cycles, consistent period).
    pub valid: bool,
}

/// PID gains computed from relay feedback.
#[derive(Debug, Clone, Copy, Default)]
pub struct PidGains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

impl PidGains {
    /// Compute PID gains from ultimate gain and period using Ziegler-Nichols classic.
    pub fn from_ziegler_nichols(ku: f32, tu: f32) -> Self {
        Self {
            p: 0.6 * ku,
            i: 0.6 * ku / (0.5 * tu), // Ki = Kp / Ti, Ti = Tu/2
            d: 0.6 * ku * 0.125 * tu,  // Kd = Kp * Td, Td = Tu/8
        }
    }

    /// Compute PID gains using Tyreus-Luyben (less aggressive, better for marine).
    /// Preferred for USVs — less overshoot in water where recovery is slow.
    pub fn from_tyreus_luyben(ku: f32, tu: f32) -> Self {
        Self {
            p: 0.45 * ku,
            i: 0.45 * ku / (2.2 * tu), // Ti = 2.2 * Tu
            d: 0.45 * ku * tu / 6.3,    // Td = Tu / 6.3
        }
    }

    /// Conservative gains for initial testing (50% of Tyreus-Luyben).
    pub fn conservative(ku: f32, tu: f32) -> Self {
        let tl = Self::from_tyreus_luyben(ku, tu);
        Self {
            p: tl.p * 0.5,
            i: tl.i * 0.5,
            d: tl.d * 0.5,
        }
    }
}

/// Step response characterization.
#[derive(Debug, Clone, Copy, Default)]
pub struct StepResponse {
    /// Time from step input to 10% of final value (seconds).
    pub delay_time: f32,
    /// Time from step input to 63% of final value (seconds).
    pub time_constant: f32,
    /// Time from step input to first crossing of final value (seconds).
    pub rise_time: f32,
    /// Maximum overshoot as fraction of final value (0.0 = no overshoot).
    pub overshoot: f32,
    /// Time from step input to settling within ±5% of final value (seconds).
    pub settling_time: f32,
    /// Steady-state value after settling.
    pub steady_state: f32,
    /// Whether the response is valid.
    pub valid: bool,
}

// ---------------------------------------------------------------------------
// Auto-Tuner
// ---------------------------------------------------------------------------

/// Configuration for USV auto-tuning.
#[derive(Debug, Clone, Copy)]
pub struct UsvTuneConfig {
    /// Speed step amplitude (fraction of max throttle, 0.0-1.0). Default 0.3.
    pub speed_step_amplitude: f32,
    /// Heading step amplitude (degrees). Default 30.0.
    pub heading_step_deg: f32,
    /// Relay amplitude for speed (fraction of throttle). Default 0.2.
    pub speed_relay_amplitude: f32,
    /// Relay amplitude for heading (fraction of rudder). Default 0.3.
    pub heading_relay_amplitude: f32,
    /// Minimum oscillation cycles for valid relay result. Default 4.
    pub min_relay_cycles: u16,
    /// Maximum time for each test phase (seconds). Default 60.
    pub phase_timeout_s: f32,
    /// Which tuning method to use for gain computation.
    pub method: TuneMethod,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TuneMethod {
    /// Ziegler-Nichols classic — more aggressive.
    ZieglerNichols,
    /// Tyreus-Luyben — less aggressive, preferred for marine.
    TyreusLuyben,
    /// Conservative — 50% of Tyreus-Luyben for first test.
    Conservative,
}

impl Default for UsvTuneConfig {
    fn default() -> Self {
        Self {
            speed_step_amplitude: 0.3,
            heading_step_deg: 30.0,
            speed_relay_amplitude: 0.2,
            heading_relay_amplitude: 0.3,
            min_relay_cycles: 4,
            phase_timeout_s: 60.0,
            method: TuneMethod::TyreusLuyben,
        }
    }
}

/// USV auto-tuner state machine.
pub struct UsvAutoTune {
    pub phase: TunePhase,
    config: UsvTuneConfig,

    // Step response tracking
    step_start_time: f32,
    step_input_value: f32,
    step_initial_output: f32,
    step_peak_output: f32,
    step_peak_time: f32,
    step_settled: bool,
    step_settle_start: f32,
    step_response: StepResponse,

    // Relay feedback tracking
    relay_output: f32, // current relay output (+amp or -amp)
    relay_last_crossing_time: f32,
    relay_periods: [f32; 16],
    relay_amplitudes: [f32; 16],
    relay_cycle_count: u16,
    relay_setpoint: f32,

    // Results
    pub speed_result: RelayResult,
    pub heading_result: RelayResult,
    pub speed_gains: PidGains,
    pub heading_gains: PidGains,

    // Timer
    phase_elapsed: f32,
}

impl UsvAutoTune {
    pub fn new(config: UsvTuneConfig) -> Self {
        Self {
            phase: TunePhase::Idle,
            config,
            step_start_time: 0.0,
            step_input_value: 0.0,
            step_initial_output: 0.0,
            step_peak_output: 0.0,
            step_peak_time: 0.0,
            step_settled: false,
            step_settle_start: 0.0,
            step_response: StepResponse::default(),
            relay_output: 0.0,
            relay_last_crossing_time: 0.0,
            relay_periods: [0.0; 16],
            relay_amplitudes: [0.0; 16],
            relay_cycle_count: 0,
            relay_setpoint: 0.0,
            speed_result: RelayResult::default(),
            heading_result: RelayResult::default(),
            speed_gains: PidGains::default(),
            heading_gains: PidGains::default(),
            phase_elapsed: 0.0,
        }
    }

    /// Start the auto-tuning sequence. Vehicle must be in water, armed, GPS lock.
    pub fn start(&mut self) {
        self.phase = TunePhase::SpeedRelay;
        self.reset_relay();
        self.phase_elapsed = 0.0;
    }

    /// Cancel tuning and return to idle.
    pub fn cancel(&mut self) {
        self.phase = TunePhase::Idle;
    }

    /// Update the auto-tuner. Call at control loop rate (10-50Hz for boats).
    ///
    /// `dt`: time step in seconds.
    /// `speed_ms`: current ground speed in m/s.
    /// `heading_deg`: current heading in degrees (0-360).
    /// `heading_target_deg`: desired heading during heading tuning.
    ///
    /// Returns: (throttle_command, steering_command) to send to motors.
    /// Both are -1.0 to +1.0. Returns None if tuning is idle/complete.
    pub fn update(
        &mut self,
        dt: f32,
        speed_ms: f32,
        heading_deg: f32,
    ) -> Option<(f32, f32)> {
        self.phase_elapsed += dt;

        // Timeout check
        if self.phase_elapsed > self.config.phase_timeout_s
            && self.phase != TunePhase::Idle
            && self.phase != TunePhase::Complete
            && self.phase != TunePhase::Computing
        {
            self.phase = TunePhase::Failed;
            return None;
        }

        match self.phase {
            TunePhase::Idle | TunePhase::Complete | TunePhase::Failed => None,

            TunePhase::SpeedRelay => {
                let (throttle, done) = self.run_relay(
                    speed_ms,
                    self.config.speed_relay_amplitude,
                    dt,
                );
                if done {
                    self.speed_result = self.compute_relay_result();
                    if self.speed_result.valid {
                        self.phase = TunePhase::HeadingRelay;
                        self.reset_relay();
                        self.phase_elapsed = 0.0;
                        self.relay_setpoint = heading_deg;
                    } else {
                        self.phase = TunePhase::Failed;
                    }
                }
                Some((throttle, 0.0))
            }

            TunePhase::HeadingRelay => {
                // Normalize heading error
                let heading_error = normalize_angle(heading_deg - self.relay_setpoint);
                let (steering, done) = self.run_relay(
                    heading_error,
                    self.config.heading_relay_amplitude,
                    dt,
                );
                if done {
                    self.heading_result = self.compute_relay_result();
                    if self.heading_result.valid {
                        self.phase = TunePhase::Computing;
                    } else {
                        self.phase = TunePhase::Failed;
                    }
                }
                // Hold speed steady during heading tuning
                Some((self.config.speed_step_amplitude, steering))
            }

            TunePhase::Computing => {
                self.compute_gains();
                self.phase = TunePhase::Complete;
                None
            }

            // Step response phases (optional — not used in relay-first flow)
            TunePhase::SpeedStep | TunePhase::HeadingStep => None,
        }
    }

    // --- Relay feedback ---

    fn reset_relay(&mut self) {
        self.relay_output = 0.0;
        self.relay_last_crossing_time = 0.0;
        self.relay_periods = [0.0; 16];
        self.relay_amplitudes = [0.0; 16];
        self.relay_cycle_count = 0;
        self.relay_setpoint = 0.0;
    }

    /// Run relay feedback on one axis. Returns (control_output, is_done).
    fn run_relay(
        &mut self,
        measured: f32,
        amplitude: f32,
        dt: f32,
    ) -> (f32, bool) {
        let error = measured - self.relay_setpoint;

        // Relay: switch output sign when error crosses zero
        let new_sign = if error > 0.0 { -1.0 } else { 1.0 };
        let old_sign = if self.relay_output >= 0.0 { 1.0 } else { -1.0 };

        if new_sign != old_sign && self.phase_elapsed > 1.0 {
            // Zero crossing detected
            let period = self.phase_elapsed - self.relay_last_crossing_time;

            if self.relay_last_crossing_time > 0.0 && period > 0.5 {
                let idx = (self.relay_cycle_count as usize).min(15);
                self.relay_periods[idx] = period * 2.0; // full cycle = 2 half-cycles
                self.relay_amplitudes[idx] = libm::fabsf(error);
                self.relay_cycle_count += 1;
            }

            self.relay_last_crossing_time = self.phase_elapsed;
        }

        self.relay_output = new_sign * amplitude;

        let done = self.relay_cycle_count >= self.config.min_relay_cycles;
        (self.relay_output, done)
    }

    fn compute_relay_result(&self) -> RelayResult {
        if self.relay_cycle_count < self.config.min_relay_cycles {
            return RelayResult { valid: false, ..Default::default() };
        }

        let n = self.relay_cycle_count.min(16) as usize;

        // Average period (skip first cycle — often anomalous)
        let start = if n > 2 { 1 } else { 0 };
        let mut period_sum = 0.0f32;
        let mut amp_sum = 0.0f32;
        let count = (n - start) as f32;

        for i in start..n {
            period_sum += self.relay_periods[i];
            amp_sum += self.relay_amplitudes[i];
        }

        let tu = period_sum / count;
        let avg_amplitude = amp_sum / count;

        // Ultimate gain: Ku = 4 * relay_amplitude / (π * oscillation_amplitude)
        let relay_amp = if self.phase == TunePhase::SpeedRelay {
            self.config.speed_relay_amplitude
        } else {
            self.config.heading_relay_amplitude
        };
        let ku = 4.0 * relay_amp / (core::f32::consts::PI * avg_amplitude.max(0.001));

        // Validate: period should be consistent (std dev < 30% of mean)
        let mut var_sum = 0.0f32;
        for i in start..n {
            let diff = self.relay_periods[i] - tu;
            var_sum += diff * diff;
        }
        let std_dev = libm::sqrtf(var_sum / count);
        let consistent = std_dev < 0.3 * tu;

        RelayResult {
            ultimate_gain: ku,
            ultimate_period: tu,
            cycles: self.relay_cycle_count,
            valid: consistent && ku > 0.0 && tu > 0.1,
        }
    }

    fn compute_gains(&mut self) {
        let compute = match self.config.method {
            TuneMethod::ZieglerNichols => PidGains::from_ziegler_nichols,
            TuneMethod::TyreusLuyben => PidGains::from_tyreus_luyben,
            TuneMethod::Conservative => PidGains::conservative,
        };

        if self.speed_result.valid {
            self.speed_gains = compute(
                self.speed_result.ultimate_gain,
                self.speed_result.ultimate_period,
            );
        }

        if self.heading_result.valid {
            self.heading_gains = compute(
                self.heading_result.ultimate_gain,
                self.heading_result.ultimate_period,
            );
        }
    }

    pub fn is_complete(&self) -> bool {
        self.phase == TunePhase::Complete
    }

    pub fn is_failed(&self) -> bool {
        self.phase == TunePhase::Failed
    }

    pub fn progress_percent(&self) -> u8 {
        match self.phase {
            TunePhase::Idle => 0,
            TunePhase::SpeedStep | TunePhase::SpeedRelay => {
                let relay_pct = (self.relay_cycle_count as f32 / self.config.min_relay_cycles as f32 * 45.0) as u8;
                relay_pct.min(45)
            }
            TunePhase::HeadingStep | TunePhase::HeadingRelay => {
                let relay_pct = (self.relay_cycle_count as f32 / self.config.min_relay_cycles as f32 * 45.0) as u8;
                50 + relay_pct.min(45)
            }
            TunePhase::Computing => 95,
            TunePhase::Complete => 100,
            TunePhase::Failed => 0,
        }
    }
}

/// Normalize angle to -180..+180 degrees.
fn normalize_angle(mut deg: f32) -> f32 {
    while deg > 180.0 { deg -= 360.0; }
    while deg < -180.0 { deg += 360.0; }
    deg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ziegler_nichols() {
        let gains = PidGains::from_ziegler_nichols(2.0, 4.0);
        assert!((gains.p - 1.2).abs() < 0.01);     // 0.6 * 2.0
        assert!((gains.i - 0.6).abs() < 0.01);      // 1.2 / 2.0
        assert!((gains.d - 0.6).abs() < 0.01);       // 1.2 * 0.5
    }

    #[test]
    fn test_tyreus_luyben() {
        let gains = PidGains::from_tyreus_luyben(2.0, 4.0);
        assert!((gains.p - 0.9).abs() < 0.01);      // 0.45 * 2.0
        assert!(gains.i > 0.0);
        assert!(gains.d > 0.0);
        // TL should be less aggressive than ZN
        let zn = PidGains::from_ziegler_nichols(2.0, 4.0);
        assert!(gains.p < zn.p);
    }

    #[test]
    fn test_conservative() {
        let tl = PidGains::from_tyreus_luyben(2.0, 4.0);
        let cons = PidGains::conservative(2.0, 4.0);
        assert!((cons.p - tl.p * 0.5).abs() < 0.001);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(190.0) - (-170.0)).abs() < 0.01);
        assert!((normalize_angle(-190.0) - 170.0).abs() < 0.01);
        assert!((normalize_angle(45.0) - 45.0).abs() < 0.01);
    }

    #[test]
    fn test_relay_result_invalid_few_cycles() {
        let tuner = UsvAutoTune::new(UsvTuneConfig::default());
        let result = tuner.compute_relay_result();
        assert!(!result.valid);
    }

    #[test]
    fn test_tuner_lifecycle() {
        let mut tuner = UsvAutoTune::new(UsvTuneConfig {
            min_relay_cycles: 2, // low for test
            phase_timeout_s: 10.0,
            ..Default::default()
        });
        assert_eq!(tuner.phase, TunePhase::Idle);
        tuner.start();
        assert_eq!(tuner.phase, TunePhase::SpeedRelay);
        assert!(!tuner.is_complete());
    }
}
