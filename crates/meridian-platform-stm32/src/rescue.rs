//! Recovery / rescue paths for remote firmware iteration.
//!
//! Three independent mechanisms ensure the chip can always be returned
//! to the ChibiOS bootloader for re-flashing without physical access to
//! the BOOT button:
//!
//! 1. **MAVLink reboot-to-bootloader** — the running firmware listens
//!    on its primary command channel for the standard ArduPilot
//!    `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` command with `param1=3`. On
//!    receipt it stamps the bootloader magic in DTCM and triggers an
//!    NVIC system reset. Driven from the host via uploader.py or any
//!    MAVLink GCS.
//!
//! 2. **Safe-mode GPIO** — at boot, before any peripheral init that
//!    could brick the firmware, a configurable GPIO is sampled. If
//!    held in the rescue state, the firmware bypasses normal boot and
//!    jumps straight to the bootloader. Last-resort recovery if the
//!    firmware is so broken it never reaches the MAVLink loop.
//!
//! 3. **Watchdog backstop** — the IWDG is armed early in boot. If the
//!    firmware hangs or panics before petting it, the watchdog resets
//!    the chip, the bootloader runs its 5-second listen window, and a
//!    waiting host catches it.
//!
//! The bootloader-magic mechanism matches ArduPilot's convention so
//! the existing ChibiOS bootloader on Cube Orange Plus (and similar
//! ArduPilot-baseline boards) recognizes the request without changes.
//!
//! See `docs/flash-sessions/2026-04-26_first-meridian-flash.md` for the
//! incident that motivated all three.

use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};

use cortex_m::peripheral::SCB;
use stm32h7xx_hal::pac;

/// Sentinel meaning "no pending servo command".
const PENDING_SERVO_NONE: u32 = 0xFFFF_FFFF;

/// MAVLink-driven servo command queue. Set by the USB / UART command
/// parsers; drained by the main loop, which then mutates the live PWM
/// duty cycle. Encoded high16 = MAVLink servo channel (1..=14),
/// low16 = pulse width in microseconds (typically 1100..=1900).
static PENDING_SERVO: AtomicU32 = AtomicU32::new(PENDING_SERVO_NONE);

/// Drain one pending servo command if available. The main loop calls
/// this every iteration and applies the result to its PWM peripheral.
pub fn take_pending_servo() -> Option<(u8, u16)> {
    let v = PENDING_SERVO.swap(PENDING_SERVO_NONE, Ordering::Relaxed);
    if v == PENDING_SERVO_NONE {
        return None;
    }
    let channel = ((v >> 16) & 0xFFFF) as u8;
    let pwm = (v & 0xFFFF) as u16;
    Some((channel, pwm))
}

/// Internal: stash a decoded DO_SET_SERVO command for the main loop.
fn store_pending_servo(channel: u8, pwm: u16) {
    let encoded = ((channel as u32) << 16) | (pwm as u32);
    PENDING_SERVO.store(encoded, Ordering::Relaxed);
}

/// ArduPilot bootloader "stay in DFU" magic. Stored in `RTC->BKP0R`.
/// On reset, the ChibiOS-based ArduPilot bootloader reads BKP0R; if
/// it equals this value, the bootloader stays in DFU mode forever
/// instead of running its 5-second window and jumping to the app.
///
/// Verified against ArduPilot source:
///   `libraries/AP_HAL_ChibiOS/hwdef/common/stm32_util.h`
///   `enum rtc_boot_magic { RTC_BOOT_HOLD = 0xb0070001, ... }`
///   `Tools/AP_Bootloader/AP_Bootloader.cpp` — the `m == RTC_BOOT_HOLD`
///   branch is the "stay in bootloader" path.
///
/// **CRITICAL HISTORY**: v0.1-v0.8 of this firmware wrote a different
/// magic (`0x5A5A5A5A`) to a DTCM word that the bootloader does NOT
/// check. Every "self-recovery" path that depended on the magic was
/// silently broken. v0.9+ uses the correct RTC backup register.
pub const RTC_BOOT_HOLD: u32 = 0xb007_0001;
pub const RTC_BOOT_OFF: u32 = 0x0000_0000;

/// Enable backup-domain write access. Required before writing RTC
/// backup registers. Idempotent.
///
/// CRITICAL ORDER for H7 backup-domain access:
///   1. **RCC.APB4ENR.RTCAPBEN must be 1** — this is the APB interface
///      clock to the RTC peripheral. Without it, every read/write to
///      `RTC->BKP*R` is a silent no-op. This is the #1 H7 gotcha (PWR
///      itself is always-clocked on H7 — it's the RTC APB clock that
///      gates BKP register access).
///   2. **PWR.CR1.DBP must be 1** to remove backup-domain write
///      protection.
///   3. **RCC.BDCR.RTCEN must be 1** (with a valid RTCSEL) for
///      `RTC->BKP*R` writes to actually land.
///
/// In v0.9, this function lacked the RTCAPBEN enable, so on a cold
/// boot before the HAL had configured peripherals, the BKP0R writes
/// were silent no-ops and our self-recovery safety net never fired.
fn enable_backup_domain_write() {
    unsafe {
        let rcc = &*pac::RCC::ptr();

        // 1. Enable RTC APB clock so the RTC peripheral's registers
        // (including BKP*R) are reachable from the CPU bus.
        rcc.apb4enr.modify(|_, w| w.rtcapben().set_bit());
        let _ = rcc.apb4enr.read().rtcapben().bit();

        // 2. Disable backup-domain write protection.
        let pwr = &*pac::PWR::ptr();
        pwr.cr1.modify(|_, w| w.dbp().set_bit());
        // Wait until DBP is observed set (the H7 power controller is on
        // a slow APB; modify can return before the bit is latched).
        while pwr.cr1.read().dbp().bit_is_clear() {}

        // 3. Make sure RTC clock is on. The bootloader normally leaves
        // it running, but on a cold boot via VBAT loss we need to start
        // it ourselves. RTCSEL is write-once after backup-domain reset
        // — if the bootloader already chose a source we leave it alone.
        if rcc.bdcr.read().rtcen().bit_is_clear() {
            if rcc.csr.read().lsion().bit_is_clear() {
                rcc.csr.modify(|_, w| w.lsion().set_bit());
                while rcc.csr.read().lsirdy().bit_is_clear() {}
            }
            rcc.bdcr.modify(|_, w| w.rtcsel().lsi().rtcen().set_bit());
        }
    }
}

/// Write the bootloader-stay magic into RTC backup register 0.
/// Caller must have enabled backup-domain write access.
fn write_rtc_boot_magic(value: u32) {
    unsafe {
        let rtc = &*pac::RTC::ptr();
        rtc.bkpr[0].write(|w| w.bits(value));
    }
}

/// Read RTC backup register 0.
fn read_rtc_boot_magic() -> u32 {
    unsafe {
        let rtc = &*pac::RTC::ptr();
        rtc.bkpr[0].read().bits()
    }
}

/// Trigger a clean reset into the ChibiOS bootloader, requesting that
/// it stay in DFU mode forever.
///
/// Writes `RTC_BOOT_HOLD` into the RTC backup register the ArduPilot
/// bootloader actually checks, then performs a system reset.
pub fn reboot_to_bootloader() -> ! {
    enable_backup_domain_write();
    write_rtc_boot_magic(RTC_BOOT_HOLD);

    compiler_fence(Ordering::SeqCst);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    SCB::sys_reset();
}

/// Set the bootloader-stay magic without rebooting. Used at boot start
/// as a safety net: if anything below crashes/hangs and triggers any
/// reset, the bootloader sees the magic and stays in DFU forever
/// instead of jumping to broken firmware.
///
/// Cleared (via `clear_rescue_flag`) only after the firmware has proven
/// itself healthy.
///
/// Returns whether the write was readable back as the magic value. If
/// this is false, backup-domain access didn't take and the safety net
/// is non-functional this boot.
pub fn arm_rescue_flag() -> bool {
    enable_backup_domain_write();
    write_rtc_boot_magic(RTC_BOOT_HOLD);

    // Force the store to drain to the backup-domain APB before any
    // subsequent code (especially anything that could trigger a reset)
    // runs. Per H7 RM the BKP write goes via APB4 + a bridge to the
    // backup domain; without DSB+ISB it can sit in the write buffer
    // when sys_reset hits and never reach BKP0R.
    compiler_fence(Ordering::SeqCst);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Verify the write landed. If backup-domain access is broken
    // (PWR clock off, DBP didn't stick, RTC clock off, etc.) the write
    // is a silent no-op and read-back returns whatever was previously
    // there. Caller can log this for diagnostic.
    read_rtc_boot_magic() == RTC_BOOT_HOLD
}

/// Returns true if the rescue magic is currently set in the RTC backup
/// register. The bootloader clears it after observing, so this only
/// returns true between an `arm_rescue_flag` (or `reboot_to_bootloader`)
/// call and the next time the bootloader runs.
///
/// Enables backup-domain access first; without this the BKP0R read can
/// return zero even when the magic was previously written, because
/// RCC.APB4ENR.RTCAPBEN may not be set on a cold boot before the HAL
/// has touched it.
pub fn rescue_flag_was_set() -> bool {
    enable_backup_domain_write();
    read_rtc_boot_magic() == RTC_BOOT_HOLD
}

/// Clear the rescue flag. Call this once the firmware has reached a
/// known-good state (e.g. after USB has enumerated successfully). After
/// this, subsequent resets will go through the bootloader's normal
/// 5-second DFU window then jump to the app — same as a clean cold boot.
pub fn clear_rescue_flag() {
    enable_backup_domain_write();
    write_rtc_boot_magic(RTC_BOOT_OFF);
    compiler_fence(Ordering::SeqCst);
}

// ============================================================================
// Safe-mode GPIO check
// ============================================================================

/// Configuration for the boot-time safe-mode GPIO check.
///
/// At firmware entry, before any non-trivial init, this pin is sampled.
/// If `assertion_state == Low` and the pin reads low (or High and the
/// pin reads high), the firmware skips all normal boot and reboots
/// straight into the bootloader.
///
/// Pin choice on Cube Orange Plus: TBD — should be a pin that's
/// physically accessible on a service connector and not used by any
/// shipped peripheral. PE3 (the boot LED) is a candidate during dev.
/// Leaving the chosen pin floating reads as the un-asserted state via
/// internal pull-up/down configured here.
#[derive(Clone, Copy, Debug)]
pub struct SafeModeConfig {
    /// GPIO port and pin (e.g. ('E', 3) for PE3)
    pub port: char,
    pub pin: u8,
    /// State at which we treat the pin as "rescue requested"
    pub assertion_state: PinState,
    /// Whether to enable an internal pull resistor opposite to the
    /// assertion direction. If assertion is Low, this enables pull-up.
    pub enable_pull: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinState {
    Low,
    High,
}

/// Sample the safe-mode pin and, if asserted, reboot to bootloader.
///
/// Must be called as early as possible in `main()` — before any
/// peripheral init that could fault, before the watchdog is armed,
/// before USB enumeration. The whole point is to provide an escape
/// hatch when other init paths are broken.
///
/// # Safety
/// Touches the GPIO peripheral registers directly to avoid pulling in
/// the full HAL stack before we know the firmware is healthy enough
/// to use it.
pub fn check_safe_mode(config: &SafeModeConfig) {
    use stm32h7xx_hal::pac;

    // Map port letter to base address. On H7, GPIOA..GPIOK are at
    // contiguous AHB4 base offsets.
    let port_base: u32 = match config.port {
        'A' => 0x5802_0000,
        'B' => 0x5802_0400,
        'C' => 0x5802_0800,
        'D' => 0x5802_0C00,
        'E' => 0x5802_1000,
        'F' => 0x5802_1400,
        'G' => 0x5802_1800,
        'H' => 0x5802_1C00,
        'I' => 0x5802_2000,
        'J' => 0x5802_2400,
        'K' => 0x5802_2800,
        _ => return, // bad config, don't risk a wrong GPIO
    };

    // Enable the GPIO clock for the requested port. RCC AHB4ENR is at
    // 0x5802_44E0; bit `port - 'A'` enables GPIOx clock.
    unsafe {
        let rcc_ahb4enr = 0x5802_44E0 as *mut u32;
        let bit = (config.port as u32).wrapping_sub('A' as u32);
        core::ptr::write_volatile(
            rcc_ahb4enr,
            core::ptr::read_volatile(rcc_ahb4enr) | (1 << bit),
        );
        cortex_m::asm::dsb();

        // Configure pin as input. MODER is at port_base + 0x00; 2 bits per pin.
        let moder = port_base as *mut u32;
        let pin_shift = (config.pin as u32) * 2;
        let mut moder_val = core::ptr::read_volatile(moder);
        moder_val &= !(0b11 << pin_shift); // clear -> input mode (00)
        core::ptr::write_volatile(moder, moder_val);

        // Configure pull-up/down. PUPDR is at port_base + 0x0C.
        if config.enable_pull {
            let pupdr = (port_base + 0x0C) as *mut u32;
            let mut pupdr_val = core::ptr::read_volatile(pupdr);
            pupdr_val &= !(0b11 << pin_shift);
            // Pull opposite to the assertion direction so a floating
            // pin reads as un-asserted.
            let pull_bits = match config.assertion_state {
                PinState::Low => 0b01,  // pull-up
                PinState::High => 0b10, // pull-down
            };
            pupdr_val |= pull_bits << pin_shift;
            core::ptr::write_volatile(pupdr, pupdr_val);
        }

        // Allow the input + pull to settle
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }

        // Read IDR (port_base + 0x10) and check the pin
        let idr = (port_base + 0x10) as *const u32;
        let idr_val = core::ptr::read_volatile(idr);
        let pin_high = (idr_val >> config.pin) & 1 == 1;

        let asserted = match config.assertion_state {
            PinState::Low => !pin_high,
            PinState::High => pin_high,
        };

        if asserted {
            // Rescue requested — reboot into bootloader.
            // Note: we leave the GPIO clock enabled so the bootloader
            // can re-read the pin if it cares. Cost is negligible.
            reboot_to_bootloader();
        }
    }

    // Dummy reference to keep the pac import valid even if the H7 pac
    // is moved later.
    let _ = core::mem::size_of::<pac::Peripherals>();
}

// ============================================================================
// MAVLink reboot-to-bootloader command parser
// ============================================================================

/// Stateful incremental parser for the MAVLink v2 frame stream.
///
/// We don't link in a full MAVLink library — for rescue purposes we
/// only need to recognize one specific command. Parses bytes one at a
/// time and returns `Some(())` when the magic command is detected.
pub struct RebootCommandParser {
    state: ParserState,
    payload_len: usize,
    payload_buf: [u8; 64],
    payload_idx: usize,
    msg_id_low: u8,
    msg_id_mid: u8,
    msg_id_high: u8,
}

enum ParserState {
    WaitMagic,
    Length,
    IncompatFlags,
    CompatFlags,
    Sequence,
    SystemId,
    ComponentId,
    MessageId0,
    MessageId1,
    MessageId2,
    Payload,
    Crc1,
    Crc2,
}

impl Default for RebootCommandParser {
    fn default() -> Self {
        Self::new()
    }
}

impl RebootCommandParser {
    pub const fn new() -> Self {
        Self {
            state: ParserState::WaitMagic,
            payload_len: 0,
            payload_buf: [0; 64],
            payload_idx: 0,
            msg_id_low: 0,
            msg_id_mid: 0,
            msg_id_high: 0,
        }
    }

    /// Feed one byte of the MAVLink stream. Returns `true` exactly
    /// when a `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` (msg_id 76) with
    /// `param1 == 3` (remain in bootloader) is fully received.
    ///
    /// CRC validation is intentionally skipped — for a rescue command
    /// we accept slightly noisier framing in exchange for never
    /// missing a reboot request. If a stray bit-flip happens to spell
    /// out the exact reboot signature, the worst case is a reset, and
    /// the bootloader's 5-second window is not destructive.
    pub fn feed(&mut self, byte: u8) -> bool {
        match self.state {
            ParserState::WaitMagic => {
                if byte == 0xFD {
                    self.state = ParserState::Length;
                }
                false
            }
            ParserState::Length => {
                self.payload_len = byte as usize;
                if self.payload_len > self.payload_buf.len() {
                    self.reset();
                    return false;
                }
                self.payload_idx = 0;
                self.state = ParserState::IncompatFlags;
                false
            }
            ParserState::IncompatFlags => { self.state = ParserState::CompatFlags; false }
            ParserState::CompatFlags  => { self.state = ParserState::Sequence; false }
            ParserState::Sequence     => { self.state = ParserState::SystemId; false }
            ParserState::SystemId     => { self.state = ParserState::ComponentId; false }
            ParserState::ComponentId  => { self.state = ParserState::MessageId0; false }
            ParserState::MessageId0   => { self.msg_id_low = byte; self.state = ParserState::MessageId1; false }
            ParserState::MessageId1   => { self.msg_id_mid = byte; self.state = ParserState::MessageId2; false }
            ParserState::MessageId2   => {
                self.msg_id_high = byte;
                if self.payload_len == 0 {
                    self.state = ParserState::Crc1;
                } else {
                    self.state = ParserState::Payload;
                }
                false
            }
            ParserState::Payload => {
                if self.payload_idx < self.payload_buf.len() {
                    self.payload_buf[self.payload_idx] = byte;
                }
                self.payload_idx += 1;
                if self.payload_idx >= self.payload_len {
                    self.state = ParserState::Crc1;
                }
                false
            }
            ParserState::Crc1 => { self.state = ParserState::Crc2; false }
            ParserState::Crc2 => {
                // Frame complete. Inspect what we just received.
                let msg_id = (self.msg_id_low as u32)
                    | ((self.msg_id_mid as u32) << 8)
                    | ((self.msg_id_high as u32) << 16);
                let is_command_long = msg_id == 76;
                let is_reboot_cmd = is_command_long && self.is_reboot_to_bl();
                self.reset();
                is_reboot_cmd
            }
        }
    }

    /// Inspect the latest payload and decide what command was received.
    ///
    /// COMMAND_LONG payload layout (33 bytes):
    ///   param1..7: 7 × f32 little-endian (28 bytes)
    ///   command: u16 LE (2 bytes)
    ///   target_system: u8
    ///   target_component: u8
    ///   confirmation: u8
    ///
    /// Returns `true` only for the reboot request (so the legacy caller
    /// keeps working). For other recognized commands (DO_SET_SERVO),
    /// stashes the result into the `PENDING_SERVO` atomic and returns
    /// `false`. The main loop drains that atomic on its own cadence.
    fn is_reboot_to_bl(&self) -> bool {
        if self.payload_idx < 30 {
            return false;
        }
        // param1 — first 4 bytes, IEEE-754 float
        let p1 = f32::from_le_bytes([
            self.payload_buf[0],
            self.payload_buf[1],
            self.payload_buf[2],
            self.payload_buf[3],
        ]);
        // param2 — bytes 4..8
        let p2 = f32::from_le_bytes([
            self.payload_buf[4],
            self.payload_buf[5],
            self.payload_buf[6],
            self.payload_buf[7],
        ]);
        // command field at offset 28
        let cmd = (self.payload_buf[28] as u16) | ((self.payload_buf[29] as u16) << 8);

        match cmd {
            // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1 = 3 = stay in bootloader
            246 if (p1 - 3.0).abs() < 0.01 => true,
            // MAV_CMD_DO_SET_SERVO: param1 = servo number, param2 = PWM in us
            183 => {
                // Clamp to sensible ranges before stashing.
                let channel = if p1 > 0.5 && p1 < 254.5 { p1 as u8 } else { 0 };
                let pwm = if p2 > 500.0 && p2 < 2500.0 { p2 as u16 } else { 0 };
                if channel != 0 && pwm != 0 {
                    store_pending_servo(channel, pwm);
                }
                false
            }
            _ => false,
        }
    }

    fn reset(&mut self) {
        self.state = ParserState::WaitMagic;
        self.payload_len = 0;
        self.payload_idx = 0;
    }
}

// ============================================================================
// USB CDC ACM — full implementation
// ============================================================================

/// USB CDC ACM for the Cube Orange Plus.
///
/// The Cube's USB-C connector routes to OTG2_HS pins (PB14 = D-, PB15 = D+)
/// on the STM32H757. We drive the OTG2 controller in HS-with-internal-FS-PHY
/// mode so the host sees a 12 Mbps full-speed CDC ACM device — bit-identical
/// in shape to what the ChibiOS bootloader and ArduPilot enumerate.
///
/// USB ID: VID `0x1209` (pid.codes "Open Source") / PID `0x5741` (Meridian).
///
/// # Why this is the rescue path
/// USB CDC is the only host-reachable channel that survives independent of
/// the boat's UART/radio plumbing. As long as the USB-C cable is plugged in
/// and Meridian's USB enumerates, a host can:
///   - Read defmt-equivalent log strings emitted by the firmware
///   - Send a `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1=3` MAVLink frame
///     to drop into the bootloader
///   - Use `uploader.py` directly (it sends the magic reboot bytes itself)
///
/// # Lifetime model
/// The USB allocator owns the `UsbBus`. The `SerialPort` and `UsbDevice`
/// borrow from it. We park the allocator in a `static_cell::StaticCell`
/// to keep them all `'static` so a single global init is sound.
pub mod usb_cdc {
    use core::sync::atomic::{AtomicBool, Ordering};

    use static_cell::StaticCell;
    use stm32h7xx_hal::gpio::{Alternate, Pin};
    use stm32h7xx_hal::pac;
    use stm32h7xx_hal::rcc::{rec, CoreClocks};
    use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::prelude::*;
    use usbd_serial::SerialPort;

    /// USB OTG_FS data lines on the Cube Orange Plus.
    /// ArduPilot hwdef has these on OTG1 (the FS controller).
    /// stm32h7xx-hal's USB2 wants the pins in Analog mode (the chip
    /// internally muxes them as USB analog signals; no AF needed).
    pub type UsbDmPin = Pin<'A', 11, Alternate<10>>;
    pub type UsbDpPin = Pin<'A', 12, Alternate<10>>;

    use super::RebootCommandParser;

    /// USB IDs picked from the pid.codes "Open Source Hardware" pool.
    /// `0x1209` is the public-allocation VID; `0x5741` is reserved for
    /// the Meridian autopilot project.
    pub const USB_VID: u16 = 0x1209;
    pub const USB_PID: u16 = 0x5741;

    /// Endpoint memory pool. 1024 × u32 = 4 KB; the FS PHY needs ~512 B
    /// for two endpoints (CDC IN / OUT bulk + control). 4 KB leaves
    /// headroom for the IN-buffer doublebuffer the synopsys driver
    /// allocates internally.
    static EP_MEMORY: StaticCell<[u32; 1024]> = StaticCell::new();
    /// The USB allocator. Owned for the lifetime of the program.
    static USB_BUS: StaticCell<UsbBusAllocator<UsbBus<USB2>>> = StaticCell::new();
    /// CDC ACM serial class. Borrows from the allocator.
    static USB_SERIAL: StaticCell<SerialPort<'static, UsbBus<USB2>>> = StaticCell::new();
    /// USB device descriptor + state. Borrows from the allocator.
    static USB_DEVICE: StaticCell<UsbDevice<'static, UsbBus<USB2>>> = StaticCell::new();
    /// MAVLink rescue parser fed from the CDC RX stream.
    static USB_PARSER: StaticCell<RebootCommandParser> = StaticCell::new();
    /// Set to true once `init` has run successfully.
    static INITIALIZED: AtomicBool = AtomicBool::new(false);
    /// Set to true the first time `device.poll()` returns true (i.e. we've
    /// seen any USB event from a host). Main loop checks this for a
    /// silent-timeout guard: if 120s pass with no host activity, the
    /// firmware self-reboots to bootloader for the host watcher to catch.
    static HAS_ENUMERATED: AtomicBool = AtomicBool::new(false);

    /// Returns true once any USB event has been observed since boot.
    pub fn has_enumerated() -> bool {
        HAS_ENUMERATED.load(Ordering::Relaxed)
    }

    /// Mutable static pointers for the running USB stack. Filled by
    /// `init()` and used by `poll()` and `write_str()`. The
    /// `StaticCell` provides the safe one-time-init guarantee; we then
    /// take raw `&'static mut` references for the runtime.
    static mut USB_DEVICE_PTR: Option<&'static mut UsbDevice<'static, UsbBus<USB2>>> = None;
    static mut USB_SERIAL_PTR: Option<&'static mut SerialPort<'static, UsbBus<USB2>>> = None;
    static mut USB_PARSER_PTR: Option<&'static mut RebootCommandParser> = None;

    /// Errors that can occur during USB CDC bring-up.
    #[derive(Debug)]
    pub enum Error {
        /// `init()` was already called once.
        AlreadyInitialized,
        /// 48 MHz USB clock could not be configured. Caller must
        /// enable HSI48 (or PLL3_Q at 48 MHz) before calling.
        ClockUnsupported,
    }

    /// Initialize USB CDC ACM. Call exactly once after clocks are up
    /// and after the USB clock domain has been pointed at HSI48.
    ///
    /// # Arguments
    /// - `otg_hs_global` / `otg_hs_device` / `otg_hs_pwrclk`: PAC
    ///    register blocks for the OTG_HS controller.
    /// - `usb_rec`: stm32h7xx-hal's resource enabler for OTG_HS.
    /// - `clocks`: the configured `CoreClocks` (we read `hclk` from it).
    /// - `pin_dm`, `pin_dp`: PB14/PB15 already configured to AF12.
    ///
    /// # Pin requirements
    /// The caller must have set PB14 and PB15 to alternate function
    /// 12 (USB OTG_HS_FS) before calling. Configuring those pins
    /// requires the GPIOB clock and HAL ownership of the gpio module,
    /// which the caller has at the top of `main()`.
    pub fn init(
        otg_hs_global: pac::OTG2_HS_GLOBAL,
        otg_hs_device: pac::OTG2_HS_DEVICE,
        otg_hs_pwrclk: pac::OTG2_HS_PWRCLK,
        usb_dm: UsbDmPin,
        usb_dp: UsbDpPin,
        usb_rec: rec::Usb2Otg,
        clocks: &CoreClocks,
    ) -> Result<(), Error> {
        if INITIALIZED.swap(true, Ordering::SeqCst) {
            return Err(Error::AlreadyInitialized);
        }

        // Build the Synopsys USB OTG peripheral wrapper. The HAL's
        // `USB2::new` takes the raw PAC blocks, the data-line pins
        // (PA11/PA12 in OTG_FS-on-OTG2 mode), the resource enabler,
        // and the core clocks (so it can verify the 48 MHz USB clock
        // is configured) and produces a peripheral suitable for
        // `synopsys-usb-otg::UsbBus`.
        let usb = USB2::new(
            otg_hs_global,
            otg_hs_device,
            otg_hs_pwrclk,
            usb_dm,
            usb_dp,
            usb_rec,
            clocks,
        );

        // Allocate endpoint memory + the USB bus.
        let ep_memory: &'static mut [u32; 1024] = EP_MEMORY.init([0u32; 1024]);
        let allocator: &'static UsbBusAllocator<UsbBus<USB2>> =
            USB_BUS.init(UsbBus::new(usb, ep_memory));

        // v1.4: dropped the manual GOTGCTL/GCCFG override block.
        // It ran BEFORE `UsbDeviceBuilder::build()` which is when
        // synopsys-usb-otg's `enable()` actually configures the core.
        // synopsys's CSRST inside enable() resets GCCFG; our pre-set
        // values were wiped out. With `USB2.HIGH_SPEED = false` patched
        // in the vendored HAL, synopsys's FS init path itself sets:
        //   GCCFG.PWRDWN = 1
        //   GCCFG.VBDEN  = 0
        //   GOTGCTL.BVALOEN | BVALOVAL = 1
        // exactly what ArduPilot's ChibiOS does. So the manual block
        // was a no-op at best and confusing dead code at worst.

        // Build the serial class (CDC ACM).
        let serial: &'static mut SerialPort<'static, UsbBus<USB2>> =
            USB_SERIAL.init(SerialPort::new(allocator));

        // Build the device descriptor: VID/PID, manufacturer, product,
        // serial number, USB version, max packet size, etc. Match the
        // shape of an ArduPilot Cube enumeration so any host driver
        // already configured for ArduPilot Just Works.
        let device: &'static mut UsbDevice<'static, UsbBus<USB2>> = USB_DEVICE.init({
            UsbDeviceBuilder::new(allocator, UsbVidPid(USB_VID, USB_PID))
                .strings(&[StringDescriptors::new(usb_device::LangID::EN)
                    .manufacturer("Meridian")
                    .product("Meridian Autopilot")
                    .serial_number("MERIDIAN-001")])
                .unwrap()
                .device_class(usbd_serial::USB_CLASS_CDC)
                .self_powered(true)
                .max_packet_size_0(64)
                .unwrap()
                .build()
        });

        // Park a parser instance for incoming MAVLink-over-USB.
        let parser: &'static mut RebootCommandParser =
            USB_PARSER.init(RebootCommandParser::new());

        // Stash mutable pointers for poll()/write_str() to use.
        unsafe {
            USB_DEVICE_PTR = Some(device);
            USB_SERIAL_PTR = Some(serial);
            USB_PARSER_PTR = Some(parser);
        }

        let _ = clocks; // currently only used by USB2::new but kept for future asserts

        Ok(())
    }

    /// Drive the USB stack. Must be called frequently (every 1 ms or
    /// faster) from the main loop or, preferably, the OTG_HS interrupt
    /// handler. Reads any pending RX bytes, feeds them to the
    /// `RebootCommandParser`, and triggers a bootloader reboot if the
    /// rescue command is detected. Never returns under normal
    /// operation; on rescue, calls `reboot_to_bootloader()` which
    /// itself never returns.
    pub fn poll() {
        // Safety: USB_*_PTR are set once by init() and only accessed
        // here. `init` is one-shot guarded by an atomic; subsequent
        // calls to poll() always see the same pointers.
        let (device, serial, parser) = unsafe {
            match (
                USB_DEVICE_PTR.as_deref_mut(),
                USB_SERIAL_PTR.as_deref_mut(),
                USB_PARSER_PTR.as_deref_mut(),
            ) {
                (Some(d), Some(s), Some(p)) => (d, s, p),
                _ => return, // not initialized yet
            }
        };

        if !device.poll(&mut [serial]) {
            return;
        }
        // Mark that we've seen at least one USB event from a host.
        HAS_ENUMERATED.store(true, Ordering::Relaxed);

        // Drain any RX bytes into the rescue parser.
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Ok(n) if n > 0 => {
                for b in &buf[..n] {
                    if parser.feed(*b) {
                        // Rescue command received over USB CDC.
                        // Echo a confirmation, flush, and reboot.
                        let _ = serial.write(b"REBOOTING TO BOOTLOADER\r\n");
                        let _ = serial.flush();
                        // Tiny delay to let the bytes get out before reset.
                        for _ in 0..1_000_000 {
                            cortex_m::asm::nop();
                        }
                        super::reboot_to_bootloader();
                    }
                }
            }
            _ => {}
        }
    }

    /// Write a string to the CDC TX buffer. Best-effort; if the host
    /// isn't reading, bytes are silently dropped (we never block the
    /// main loop on USB).
    pub fn write_str(s: &str) {
        unsafe {
            if let Some(serial) = USB_SERIAL_PTR.as_deref_mut() {
                let _ = serial.write(s.as_bytes());
            }
        }
    }

    /// Write raw bytes (e.g. a MAVLink frame) to the CDC TX buffer.
    /// Same best-effort semantics as `write_str`; never blocks the main
    /// loop. Used to emit MAVLink heartbeats over USB so blackbox-style
    /// MAVLink GCS tools can connect over the USB-C port without needing
    /// a separate UART adapter.
    pub fn write_bytes(bytes: &[u8]) {
        unsafe {
            if let Some(serial) = USB_SERIAL_PTR.as_deref_mut() {
                let _ = serial.write(bytes);
            }
        }
    }

    /// True after `init()` succeeded. Other tasks can gate USB-only
    /// behavior on this without locking.
    pub fn is_ready() -> bool {
        INITIALIZED.load(Ordering::SeqCst)
    }
}
