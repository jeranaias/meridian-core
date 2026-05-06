//! VESC (Vedder Electronic Speed Controller) UART driver.
//!
//! Implements the `COMM_PACKET` wire protocol used by VESC firmware 3.x-6.x
//! over a serial UART. Provides setpoint control (duty, RPM, current) plus
//! telemetry (RPM, duty, bus voltage, motor current, MOSFET/motor temperature,
//! accumulated distance).
//!
//! Protocol reference: <http://vedder.se/2015/10/communicating-with-the-vesc-using-uart/>
//!
//! Wire format:
//!   short packet (payload ≤ 255 B):
//!       [0x02] [len: u8] [payload...] [crc_hi] [crc_lo] [0x03]
//!   long packet (payload > 255 B):
//!       [0x03] [len_hi: u8] [len_lo: u8] [payload...] [crc_hi] [crc_lo] [0x03]
//!
//!   CRC-16-CCITT (poly 0x1021, init 0x0000) over the payload only.
//!
//! Usage pattern:
//!   let mut vesc = Vesc::new();
//!   vesc.send_alive(&mut uart);            // ~5 Hz keepalive
//!   vesc.send_set_rpm(&mut uart, 3000.0);  // rpm setpoint
//!   vesc.request_values(&mut uart);        // poll telemetry
//!   while let Some(b) = uart.read_byte() { vesc.feed(b); }
//!   if let Some(t) = vesc.latest() { /* use t */ }
//!
//! Safety: VESC automatically times out to zero throttle if no command is
//! received for ~1 s. Call `send_alive` at ≥5 Hz or send any setpoint command
//! at that rate to maintain control.

#![allow(dead_code)]

use meridian_hal::uart::UartDriver;

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

const SOF_SHORT: u8 = 0x02;
const SOF_LONG:  u8 = 0x03;
const EOF_BYTE:  u8 = 0x03;

/// Maximum payload we encode (short packets only — 255 B is plenty for all
/// commands we send). Incoming decode buffer sized 2 KB to cover GET_VALUES
/// responses plus safety margin.
const MAX_TX_PAYLOAD: usize = 255;
const RX_BUF_SIZE: usize = 2048;

// ---------------------------------------------------------------------------
// COMM_PACKET command IDs (subset we use)
// Source: VESC firmware `datatypes.h::COMM_PACKET_ID`
// ---------------------------------------------------------------------------

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum CommId {
    GetFwVersion        = 0,
    GetValues           = 4,
    SetDuty             = 5,
    SetCurrent          = 6,
    SetCurrentBrake     = 7,
    SetRpm              = 8,
    SetPos              = 9,
    SetHandbrake        = 10,
    ForwardCan          = 34,
    Alive               = 30,
    RebootToBootloader  = 53,
    GetValuesSelective  = 50,
}

// ---------------------------------------------------------------------------
// CRC16-CCITT (XMODEM variant used by VESC)
// ---------------------------------------------------------------------------

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ---------------------------------------------------------------------------
// Telemetry data (decoded from GET_VALUES response)
// ---------------------------------------------------------------------------

/// Decoded telemetry from a `COMM_GET_VALUES` response.
///
/// All fields use SI base units where practical. Fault code is raw.
#[derive(Debug, Default, Clone, Copy)]
pub struct VescValues {
    pub temp_fet_c:        f32,   // MOSFET temperature, °C
    pub temp_motor_c:      f32,   // motor temperature, °C
    pub current_motor_a:   f32,   // motor current, A
    pub current_in_a:      f32,   // bus input current, A
    pub duty_cycle:        f32,   // -1.0 .. +1.0
    pub rpm:               f32,   // electrical RPM (divide by pole pairs for mech RPM)
    pub voltage_in_v:      f32,   // bus voltage, V
    pub amp_hours:         f32,   // accumulated Ah consumed
    pub amp_hours_charged: f32,   // accumulated Ah regenerated
    pub watt_hours:        f32,   // accumulated Wh consumed
    pub watt_hours_charged:f32,   // accumulated Wh regenerated
    pub tachometer:        i32,   // raw tach count (electrical)
    pub tachometer_abs:    i32,   // absolute tach distance
    pub fault_code:        u8,    // 0 = no fault
}

/// Fault codes (subset — see VESC `datatypes.h::mc_fault_code`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VescFault {
    None                     = 0,
    OverVoltage              = 1,
    UnderVoltage             = 2,
    Drv                      = 3,
    AbsOvercurrent           = 4,
    OverTempFet              = 5,
    OverTempMotor            = 6,
    GateDriverOverVoltage    = 7,
    GateDriverUnderVoltage   = 8,
    McuUnderVoltage          = 9,
    BootingFromWatchdogReset = 10,
    EncoderSpiFail           = 11,
    EncoderSinCosBelowMinAmp = 12,
    EncoderSinCosAboveMaxAmp = 13,
}

impl VescFault {
    pub fn from_byte(b: u8) -> Option<Self> {
        use VescFault::*;
        Some(match b {
            0  => None,
            1  => OverVoltage,
            2  => UnderVoltage,
            3  => Drv,
            4  => AbsOvercurrent,
            5  => OverTempFet,
            6  => OverTempMotor,
            7  => GateDriverOverVoltage,
            8  => GateDriverUnderVoltage,
            9  => McuUnderVoltage,
            10 => BootingFromWatchdogReset,
            11 => EncoderSpiFail,
            12 => EncoderSinCosBelowMinAmp,
            13 => EncoderSinCosAboveMaxAmp,
            _ => return Option::None,
        })
    }
}

// ---------------------------------------------------------------------------
// Driver state machine
// ---------------------------------------------------------------------------

/// Incremental RX parser state. VESC packets can be interrupted mid-stream,
/// so we feed one byte at a time and emit decoded packets as complete.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RxState {
    WaitSof,
    ReadLen1,
    ReadLen2,
    ReadPayload,
    ReadCrcHi,
    ReadCrcLo,
    ReadEof,
}

pub struct Vesc {
    state:      RxState,
    is_long:    bool,
    expected:   u16,     // declared payload length
    received:   u16,     // payload bytes received so far
    crc_hi:     u8,
    crc_lo:     u8,
    buf:        [u8; RX_BUF_SIZE],
    /// Latest decoded telemetry. Updated on each successful GET_VALUES.
    latest:     Option<VescValues>,
    /// Monotonic counter of valid decoded packets (for health checks).
    packet_count: u32,
    /// Count of dropped/corrupt packets (CRC or frame errors).
    error_count:  u32,
}

impl Default for Vesc {
    fn default() -> Self {
        Self::new()
    }
}

impl Vesc {
    pub const fn new() -> Self {
        Self {
            state: RxState::WaitSof,
            is_long: false,
            expected: 0,
            received: 0,
            crc_hi: 0,
            crc_lo: 0,
            buf: [0u8; RX_BUF_SIZE],
            latest: None,
            packet_count: 0,
            error_count: 0,
        }
    }

    pub fn latest(&self) -> Option<VescValues> { self.latest }
    pub fn packet_count(&self) -> u32 { self.packet_count }
    pub fn error_count(&self) -> u32 { self.error_count }

    /// Reset the parser (e.g., after a disconnect). Does not clear telemetry.
    pub fn reset_parser(&mut self) {
        self.state = RxState::WaitSof;
        self.expected = 0;
        self.received = 0;
    }

    // --- Command senders ----------------------------------------------------

    /// Send `COMM_SET_DUTY` with duty in range -1.0 .. +1.0.
    pub fn send_set_duty<U: UartDriver>(&self, uart: &mut U, duty: f32) {
        let d = (duty.clamp(-1.0, 1.0) * 100_000.0) as i32;
        let mut payload = [0u8; 5];
        payload[0] = CommId::SetDuty as u8;
        payload[1..].copy_from_slice(&d.to_be_bytes());
        self.write_short(uart, &payload);
    }

    /// Send `COMM_SET_CURRENT` with motor current in amps.
    pub fn send_set_current<U: UartDriver>(&self, uart: &mut U, current_a: f32) {
        let c = (current_a * 1000.0) as i32;   // mA
        let mut payload = [0u8; 5];
        payload[0] = CommId::SetCurrent as u8;
        payload[1..].copy_from_slice(&c.to_be_bytes());
        self.write_short(uart, &payload);
    }

    /// Send `COMM_SET_RPM` with target electrical RPM.
    /// Mechanical RPM = electrical RPM / pole-pairs.
    pub fn send_set_rpm<U: UartDriver>(&self, uart: &mut U, erpm: f32) {
        let r = erpm as i32;
        let mut payload = [0u8; 5];
        payload[0] = CommId::SetRpm as u8;
        payload[1..].copy_from_slice(&r.to_be_bytes());
        self.write_short(uart, &payload);
    }

    /// Send `COMM_SET_CURRENT_BRAKE` (regenerative brake current, A).
    pub fn send_brake<U: UartDriver>(&self, uart: &mut U, brake_a: f32) {
        let c = (brake_a * 1000.0) as i32;
        let mut payload = [0u8; 5];
        payload[0] = CommId::SetCurrentBrake as u8;
        payload[1..].copy_from_slice(&c.to_be_bytes());
        self.write_short(uart, &payload);
    }

    /// Send `COMM_ALIVE` — keepalive. Call ≥5 Hz; VESC disables output if
    /// no command arrives for ~1 s.
    pub fn send_alive<U: UartDriver>(&self, uart: &mut U) {
        let payload = [CommId::Alive as u8];
        self.write_short(uart, &payload);
    }

    /// Send `COMM_GET_VALUES` — VESC responds with a full telemetry packet.
    pub fn request_values<U: UartDriver>(&self, uart: &mut U) {
        let payload = [CommId::GetValues as u8];
        self.write_short(uart, &payload);
    }

    /// Send `COMM_GET_FW_VERSION` — useful to detect protocol version.
    pub fn request_fw_version<U: UartDriver>(&self, uart: &mut U) {
        let payload = [CommId::GetFwVersion as u8];
        self.write_short(uart, &payload);
    }

    // --- Low-level framing --------------------------------------------------

    fn write_short<U: UartDriver>(&self, uart: &mut U, payload: &[u8]) {
        debug_assert!(payload.len() <= MAX_TX_PAYLOAD);
        let mut frame = [0u8; MAX_TX_PAYLOAD + 5];
        frame[0] = SOF_SHORT;
        frame[1] = payload.len() as u8;
        frame[2..2 + payload.len()].copy_from_slice(payload);
        let crc = crc16_ccitt(payload);
        frame[2 + payload.len()]     = (crc >> 8) as u8;
        frame[2 + payload.len() + 1] = (crc & 0xFF) as u8;
        frame[2 + payload.len() + 2] = EOF_BYTE;
        uart.write(&frame[..payload.len() + 5]);
    }

    // --- RX feed ------------------------------------------------------------

    /// Feed one incoming byte. Returns `Some(command_id)` when a complete
    /// valid packet has been decoded (caller can then read `latest()` if it
    /// was a GET_VALUES response).
    pub fn feed(&mut self, b: u8) -> Option<u8> {
        match self.state {
            RxState::WaitSof => {
                if b == SOF_SHORT {
                    self.is_long = false;
                    self.state = RxState::ReadLen1;
                } else if b == SOF_LONG {
                    self.is_long = true;
                    self.state = RxState::ReadLen1;
                }
                None
            }
            RxState::ReadLen1 => {
                if self.is_long {
                    self.expected = (b as u16) << 8;
                    self.state = RxState::ReadLen2;
                } else {
                    self.expected = b as u16;
                    if self.expected == 0 || self.expected as usize > RX_BUF_SIZE {
                        self.error_count += 1;
                        self.reset_parser();
                        return None;
                    }
                    self.received = 0;
                    self.state = RxState::ReadPayload;
                }
                None
            }
            RxState::ReadLen2 => {
                self.expected |= b as u16;
                if self.expected == 0 || self.expected as usize > RX_BUF_SIZE {
                    self.error_count += 1;
                    self.reset_parser();
                    return None;
                }
                self.received = 0;
                self.state = RxState::ReadPayload;
                None
            }
            RxState::ReadPayload => {
                self.buf[self.received as usize] = b;
                self.received += 1;
                if self.received == self.expected {
                    self.state = RxState::ReadCrcHi;
                }
                None
            }
            RxState::ReadCrcHi => {
                self.crc_hi = b;
                self.state = RxState::ReadCrcLo;
                None
            }
            RxState::ReadCrcLo => {
                self.crc_lo = b;
                self.state = RxState::ReadEof;
                None
            }
            RxState::ReadEof => {
                let result = if b != EOF_BYTE {
                    self.error_count += 1;
                    None
                } else {
                    let computed = crc16_ccitt(&self.buf[..self.expected as usize]);
                    let received = ((self.crc_hi as u16) << 8) | self.crc_lo as u16;
                    if computed != received {
                        self.error_count += 1;
                        None
                    } else {
                        self.packet_count += 1;
                        let cmd = self.buf[0];
                        self.on_packet(cmd, &[]);
                        Some(cmd)
                    }
                };
                self.reset_parser();
                result
            }
        }
    }

    fn on_packet(&mut self, cmd: u8, _unused: &[u8]) {
        if cmd == CommId::GetValues as u8 {
            if let Some(v) = self.decode_get_values() {
                self.latest = Some(v);
            }
        }
    }

    // --- GET_VALUES payload decode -----------------------------------------
    //
    // Layout for VESC firmware ~6.00 (big-endian integers scaled by a
    // per-field factor). Offsets below are into `self.buf` starting after the
    // command byte at index 0.
    //
    //   [1..3]   temp_fet     (i16 / 10)      →  °C
    //   [3..5]   temp_motor   (i16 / 10)      →  °C
    //   [5..9]   avg_mot_cur  (i32 / 100)     →  A
    //   [9..13]  avg_in_cur   (i32 / 100)     →  A
    //   [13..17] d-current    (i32 / 100)     (unused here)
    //   [17..21] q-current    (i32 / 100)     (unused here)
    //   [21..23] duty_now     (i16 / 1000)    →  duty (-1..+1)
    //   [23..27] erpm         (i32)           →  rpm
    //   [27..29] voltage_in   (i16 / 10)      →  V
    //   [29..33] amp_hours    (i32 / 10000)   →  Ah
    //   [33..37] amp_hours_ch (i32 / 10000)   →  Ah (charged)
    //   [37..41] watt_hours   (i32 / 10000)   →  Wh
    //   [41..45] watt_hrs_ch  (i32 / 10000)   →  Wh (charged)
    //   [45..49] tachometer   (i32)
    //   [49..53] tach_abs     (i32)
    //   [53]     fault_code   (u8)
    fn decode_get_values(&self) -> Option<VescValues> {
        const MIN_LEN: usize = 54;
        if (self.expected as usize) < MIN_LEN { return None; }
        let b = &self.buf;
        let i16_at = |o: usize| i16::from_be_bytes([b[o], b[o+1]]);
        let i32_at = |o: usize| i32::from_be_bytes([b[o], b[o+1], b[o+2], b[o+3]]);

        Some(VescValues {
            temp_fet_c:         i16_at(1) as f32 * 0.1,
            temp_motor_c:       i16_at(3) as f32 * 0.1,
            current_motor_a:    i32_at(5) as f32 * 0.01,
            current_in_a:       i32_at(9) as f32 * 0.01,
            duty_cycle:         i16_at(21) as f32 * 0.001,
            rpm:                i32_at(23) as f32,
            voltage_in_v:       i16_at(27) as f32 * 0.1,
            amp_hours:          i32_at(29) as f32 * 1e-4,
            amp_hours_charged:  i32_at(33) as f32 * 1e-4,
            watt_hours:         i32_at(37) as f32 * 1e-4,
            watt_hours_charged: i32_at(41) as f32 * 1e-4,
            tachometer:         i32_at(45),
            tachometer_abs:     i32_at(49),
            fault_code:         b[53],
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_known_vectors() {
        // Standard XMODEM CRC-16 reference: "123456789" = 0x31C3
        assert_eq!(crc16_ccitt(b"123456789"), 0x31C3);
        // Manually computed XMODEM CRC for single byte 0x04 = 0x4084
        assert_eq!(crc16_ccitt(&[0x04]), 0x4084);
        // Empty input
        assert_eq!(crc16_ccitt(&[]), 0x0000);
    }

    #[test]
    fn decode_minimal_get_values() {
        // Build a GET_VALUES payload: cmd byte + 53 bytes of test data.
        let mut v = Vesc::new();
        v.expected = 54;
        v.buf[0] = CommId::GetValues as u8;
        // temp_fet = 423 (0.1°C units) → 42.3°C
        v.buf[1] = 0x01;
        v.buf[2] = 0xA7;
        // temp_motor = 500 → 50.0°C
        v.buf[3] = 0x01;
        v.buf[4] = 0xF4;
        // avg_mot_current = 1234 (0.01A) → 12.34 A
        v.buf[5..9].copy_from_slice(&1234i32.to_be_bytes());
        // duty_now = 500 (×0.001) → 0.500
        v.buf[21..23].copy_from_slice(&500i16.to_be_bytes());
        // erpm = 3000
        v.buf[23..27].copy_from_slice(&3000i32.to_be_bytes());
        // voltage_in = 250 (×0.1V) → 25.0 V
        v.buf[27..29].copy_from_slice(&250i16.to_be_bytes());
        // fault = 5 (OverTempFet)
        v.buf[53] = 5;

        let d = v.decode_get_values().expect("decode");
        assert!((d.temp_fet_c - 42.3).abs() < 0.01);
        assert!((d.temp_motor_c - 50.0).abs() < 0.01);
        assert!((d.current_motor_a - 12.34).abs() < 0.01);
        assert!((d.duty_cycle - 0.500).abs() < 0.001);
        assert!((d.rpm - 3000.0).abs() < 0.01);
        assert!((d.voltage_in_v - 25.0).abs() < 0.01);
        assert_eq!(d.fault_code, 5);
        assert_eq!(VescFault::from_byte(d.fault_code), Some(VescFault::OverTempFet));
    }

    #[test]
    fn framing_round_trip() {
        // Build a valid short packet for cmd = GetFwVersion (payload = [0x00]),
        // feed it into parser, expect packet_count to increment.
        let payload = [CommId::GetFwVersion as u8];
        let crc = crc16_ccitt(&payload);
        let frame = [
            SOF_SHORT,
            payload.len() as u8,
            payload[0],
            (crc >> 8) as u8,
            (crc & 0xFF) as u8,
            EOF_BYTE,
        ];
        let mut v = Vesc::new();
        let mut got = None;
        for b in frame.iter() {
            if let Some(cmd) = v.feed(*b) {
                got = Some(cmd);
            }
        }
        assert_eq!(got, Some(CommId::GetFwVersion as u8));
        assert_eq!(v.packet_count(), 1);
        assert_eq!(v.error_count(), 0);
    }

    #[test]
    fn bad_crc_counted_as_error() {
        let frame = [
            SOF_SHORT, 1, 0x00, 0xDE, 0xAD, EOF_BYTE, // wrong CRC
        ];
        let mut v = Vesc::new();
        for b in frame.iter() { v.feed(*b); }
        assert_eq!(v.packet_count(), 0);
        assert_eq!(v.error_count(), 1);
    }
}
