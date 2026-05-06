// Meridian STM32H743 — Phase 9 Minimum Viable Firmware
//
// Build:  cargo build --target thumbv7em-none-eabihf -p meridian-stm32-bin --release
// Flash:  probe-rs run --chip STM32H743VIHx target/thumbv7em-none-eabihf/release/meridian-stm32-bin
//
// Hardware validation checklist:
//   [1] LED blinks at 2 Hz on PE3 (boot confirmation)
//   [2] ICM-42688 WHO_AM_I = 0x47 via SPI1 (PA5/PA6/PD7, CS=PC15)
//   [3] USART1 TX @ 57600 on PA9 — MNP heartbeat (COBS-framed postcard-serialized)
//       PRIMARY channel: FC -> RFD 900x radio -> ATAK plugin
//   [4] UART7 TX @ 57600 on PE8 — MAVLink v2 heartbeat (optional, for QGC/MP)
//       SECONDARY channel: only when a MAVLink GCS is connected
//   [5] TIM5 ch1-ch4 PWM at 50 Hz on PA0-PA3 (1500 us center pulse)
//   [6] TIM6 interrupt at 1 kHz toggling PA4 (oscilloscope verification)
//
// Protocol architecture:
//   Primary UART (USART1 / SERIAL2, PA9/PA10):
//     Meridian Native Protocol (MNP) — COBS-framed postcard-serialized messages.
//     See crates/meridian-comms/src/wire.rs for COBS frame format.
//     See crates/meridian-comms/src/messages.rs for MnpMessage::Heartbeat.
//     Frame: [0x00] [COBS(msg_id + seq_le + postcard_body)] [0x00]
//     FC speaks MNP directly over UART -> RFD 900x -> ATAK plugin. No bridge.
//
//   Secondary UART (UART7 / SERIAL1, PE8/PE7):
//     MAVLink v2 adapter — standard frames for QGC / Mission Planner compatibility.
//     Runs independently on a separate UART port. NOT the native protocol.
//
// Aparicio review deferred items addressed:
//   - TIM2 clash: using TIM6 for 1 kHz scheduler instead of TIM2
//   - Semaphore guard: deferred (known issue)
//   - D-cache: deferred (known issue, AXI SRAM buffers work without cache maint for MVP)

#![cfg_attr(target_arch = "arm", no_std)]
#![cfg_attr(target_arch = "arm", no_main)]

// On non-ARM hosts, compile as a regular binary with a message.
#[cfg(not(target_arch = "arm"))]
fn main() {
    println!("Meridian STM32H743 Phase 9 MVP — build for thumbv7em-none-eabihf to run on hardware");
    println!("  cargo build --target thumbv7em-none-eabihf -p meridian-stm32-bin --release");
    println!();
    println!("Primary UART (USART1/PA9):  MNP heartbeat — COBS-framed, postcard-serialized");
    println!("Secondary UART (UART7/PE8): MAVLink v2 heartbeat — optional GCS compat");
}

// ---------------------------------------------------------------------------
// Real firmware for ARM target
// ---------------------------------------------------------------------------
#[cfg(target_arch = "arm")]
mod firmware {
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

    use cortex_m::peripheral::NVIC;
    use defmt_rtt as _; // global logger
    use panic_probe as _; // panic handler

    use stm32h7xx_hal::pac;
    use stm32h7xx_hal::pac::interrupt;
    use stm32h7xx_hal::prelude::*;

    // -----------------------------------------------------------------------
    // ICM-42688 constants
    // -----------------------------------------------------------------------
    /// WHO_AM_I register address (read: set bit 7)
    const ICM42688_WHO_AM_I: u8 = 0x75;
    /// Expected WHO_AM_I response
    const ICM42688_WHO_AM_I_VALUE: u8 = 0x47;

    // -----------------------------------------------------------------------
    // MNP imports (primary protocol)
    // -----------------------------------------------------------------------
    // Meridian Native Protocol: COBS-framed postcard-serialized messages.
    // The FC speaks this directly over UART -> RFD 900x -> ATAK plugin.
    use meridian_comms::messages::{Heartbeat, MnpMessage};
    use meridian_comms::wire::MAX_FRAME_SIZE;

    // -----------------------------------------------------------------------
    // MAVLink v2 heartbeat (secondary protocol, for QGC/Mission Planner)
    // -----------------------------------------------------------------------
    /// Pre-built MAVLink v2 heartbeat template (system_id=1, component_id=1)
    ///
    /// MAVLink v2 header (10 bytes) + payload (9 bytes) + CRC (2 bytes) = 21 bytes
    ///
    /// Header:
    ///   FD         STX (MAVLink v2 magic)
    ///   09         Payload length (9 bytes)
    ///   00         Incompatibility flags
    ///   00         Compatibility flags
    ///   XX         Sequence (filled at runtime)
    ///   01         System ID = 1
    ///   01         Component ID = 1 (autopilot)
    ///   00 00 00   Message ID = 0 (HEARTBEAT), 3 bytes LE
    ///
    /// Payload (9 bytes):
    ///   00 00 00 00  custom_mode = 0
    ///   02           type = 2 (quadrotor)
    ///   08           autopilot = 8 (generic)
    ///   00           base_mode = 0
    ///   00           system_status = 0 (uninit)
    ///   03           mavlink_version = 3
    ///
    /// CRC: 2 bytes (computed at send time)
    const MAVLINK_HEARTBEAT_TEMPLATE: [u8; 18] = [
        0xFD, // STX
        0x09, // payload len
        0x00, // incompat flags
        0x00, // compat flags
        0x00, // sequence (overwritten)
        0x01, // sys id
        0x01, // comp id
        0x00, 0x00, 0x00, // msg id = 0 (heartbeat) LE 24-bit
        // payload:
        0x00, 0x00, 0x00, 0x00, // custom_mode
        0x02, // type = quadrotor
        0x08, // autopilot = generic
        0x00, // base_mode
        0x00, // system_status = uninit
        // mavlink_version is last payload byte
    ];

    /// Final payload byte (mavlink_version = 3) kept separate for CRC computation clarity
    const MAVLINK_VERSION_BYTE: u8 = 0x03;

    /// CRC seed for HEARTBEAT (msg_id 0, CRC_EXTRA = 50)
    const HEARTBEAT_CRC_EXTRA: u8 = 50;

    /// MAVLink CRC (X.25/CCITT) accumulate one byte
    #[inline]
    fn crc_accumulate(byte: u8, crc: &mut u16) {
        let tmp = (byte as u16) ^ (*crc & 0xFF);
        let tmp = tmp ^ ((tmp << 4) & 0xFF);
        *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }

    /// Compute MAVLink CRC over header[1..] + payload + CRC_EXTRA
    fn mavlink_crc(header_and_payload: &[u8], extra: u8) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for &b in header_and_payload {
            crc_accumulate(b, &mut crc);
        }
        crc_accumulate(extra, &mut crc);
        crc
    }

    // -----------------------------------------------------------------------
    // 1 kHz tick counter (written by TIM6 ISR, read by main)
    // -----------------------------------------------------------------------
    static TICK_COUNT: AtomicU32 = AtomicU32::new(0);
    static TIM6_TOGGLE: AtomicBool = AtomicBool::new(false);

    // -----------------------------------------------------------------------
    // Entry point
    // -----------------------------------------------------------------------
    #[cortex_m_rt::entry]
    fn main() -> ! {
        // ===================================================================
        // [STEP 0] Rescue paths
        // ===================================================================
        //
        // Three independent recovery mechanisms keep this firmware
        // remotely flashable without physical access:
        //   1. GPIO safe-mode flag (DISABLED by default — see below)
        //   2. Rescue-magic detection on warm boots (informational)
        //   3. MAVLink reboot-to-bootloader on UART7 + USB CDC, both
        //      wired up later in `main()`.
        //
        // GPIO safe-mode pin choice: defaults to PE5, but we leave the
        // check OFF in this build until we've verified on real hardware
        // that the chosen pin is electrically free on the Cube Orange
        // Plus carrier. Enabling it on a pin that's tied low elsewhere
        // would put us in a reboot-to-bootloader loop on every power-up.
        // Re-enable once a service-header pin is confirmed unused.
        use meridian_platform_stm32::rescue;
        const SAFE_MODE_GPIO_ENABLED: bool = false;
        if SAFE_MODE_GPIO_ENABLED {
            let rescue_cfg = rescue::SafeModeConfig {
                port: 'E',
                pin: 5,
                assertion_state: rescue::PinState::Low,
                enable_pull: true,
            };
            rescue::check_safe_mode(&rescue_cfg);
        }

        // Detect whether we just came out of a self-triggered rescue
        // cycle (e.g. previous boot crashed after writing the rescue
        // flag, then any reset cause ran the bootloader, which saw the
        // flag and stayed in DFU until something flashed us).
        let from_rescue = rescue::rescue_flag_was_set();

        // BOOT-MAGIC SAFETY NET (v1.0):
        //
        // ArduPilot's bootloader checks `RTC->BKP0R` for the magic
        // value `0xb0070001` (RTC_BOOT_HOLD). v0.9 had the right magic
        // but ran arm_rescue_flag BEFORE pwr.constrain().freeze(), so
        // the PWR clock was off and the DBP+BKP0R writes silent-no-op'd.
        //
        // arm_rescue_flag() now: enables RCC.APB4ENR.PWREN itself,
        // sets PWR.CR1.DBP, ensures RTCEN, writes BKP0R, DSB+ISB,
        // then reads back. Returns `true` only if the magic round-trips
        // (i.e. backup-domain access actually works).
        let armed = rescue::arm_rescue_flag();

        defmt::info!(
            "Meridian Phase 9 MVP boot (from_rescue={}, safety_net_armed={})",
            from_rescue, armed
        );

        // Take peripherals
        let cp = cortex_m::Peripherals::take().unwrap();
        let dp = pac::Peripherals::take().unwrap();

        // ===================================================================
        // [STEP 1] Clock init: 8 MHz HSE -> 400 MHz PLL1
        // ===================================================================
        // Explicit VOS1 (high-perf, max sysclk 400 MHz). HAL default is
        // Scale1 already per pwr.rs `Pwr::new`, but being explicit
        // future-proofs against HAL revs that might change the default.
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.vos1().freeze();

        let rcc = dp.RCC.constrain();
        let ccdr = rcc
            // CRITICAL (v1.1): Cube Orange Plus uses a 24 MHz crystal,
            // NOT 8 MHz. Verified from ArduPilot's CubeOrangePlus hwdef:
            //   OSCILLATOR_HZ 24000000
            // v0.1-v1.0 all declared HSE=8 MHz (left over from Matek H743
            // dev-board origins), which made the HAL configure PLL for
            // 8x50=400 MHz target. With actual 24 MHz HSE, the VCO would
            // try to run at 1200 MHz (way over H7's 836 MHz max) — PLL
            // lock fails or HAL falls back to HSI. Either way, sys_ck and
            // every derived clock (AHB, APB) is wildly wrong, which
            // breaks synopsys-usb-otg's FIFO timing and TIM6's 1 kHz
            // assumption simultaneously. This is the most likely root
            // cause of every USB-silent + safety-net-failure all day.
            .use_hse(24.MHz())
            .sys_ck(400.MHz())      // PLL1 -> 400 MHz system clock
                                    // (NOT 480 — per stm32h7xx-hal#503,
                                    // 480 MHz breaks USB; every working H7
                                    // USB ref runs at 400.)
            .hclk(200.MHz())        // AHB = 200 MHz
            .pclk1(100.MHz())       // APB1 = 100 MHz (timers get 2x = 200 MHz)
            .pclk2(100.MHz())       // APB2 = 100 MHz
            .pclk3(100.MHz())       // APB3 = 100 MHz
            .pclk4(100.MHz())       // APB4 = 100 MHz
            .pll2_p_ck(200.MHz())   // PLL2_P for SPI1/SPI4 clocks
                                    // USB clock is HSI48 (set up below).
                                    // Every known-working H7 USB CDC reference
                                    // uses HSI48 — olback's Portenta H7,
                                    // Embassy's stm32h7 example, ArduPilot/
                                    // ChibiOS. PLL3_Q on H7 has reported
                                    // instability that we hit in v0.6.
            .freeze(pwrcfg, &dp.SYSCFG);

        defmt::info!("Clock: sysclk={} MHz, hclk={} MHz",
            ccdr.clocks.sys_ck().raw() / 1_000_000,
            ccdr.clocks.hclk().raw() / 1_000_000);

        // Enable DWT cycle counter for timing
        let mut cp_dcb = cp.DCB;
        let mut cp_dwt = cp.DWT;
        cp_dcb.enable_trace();
        cortex_m::peripheral::DWT::unlock();
        cp_dwt.enable_cycle_counter();

        // ===================================================================
        // [STEP 2] GPIO init: LED on PE3, CS on PC15, scope pin on PA4
        // ===================================================================

        // Split GPIO ports
        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        // LED pin PE3 — push-pull output, start HIGH (LED on)
        let mut led = gpioe.pe3.into_push_pull_output();
        led.set_high();
        defmt::info!("LED on PE3: ON");

        // Scope verification pin PA4 — toggled by TIM6 ISR at 1 kHz
        let mut scope_pin = gpioa.pa4.into_push_pull_output();
        scope_pin.set_low();

        // SPI1 CS for ICM-42688 on PC15 — manual push-pull output, start HIGH (deselected)
        let mut imu_cs = gpioc.pc15.into_push_pull_output();
        imu_cs.set_high();

        // ===================================================================
        // [STEP 3] SPI1 init for ICM-42688: PA5(SCK), PA6(MISO), PD7(MOSI)
        // ===================================================================
        //
        // ICM-42688 talks SPI Mode 3 (CPOL=1, CPHA=1).
        // Low speed for initial probe: 1 MHz.
        // MatekH743 SPI1 pinout: SCK=PA5 AF5, MISO=PA6 AF5, MOSI=PD7 AF5.

        let sck  = gpioa.pa5.into_alternate::<5>();
        let miso = gpioa.pa6.into_alternate::<5>();
        let mosi = gpiod.pd7.into_alternate::<5>();

        // Configure SPI1 at 1 MHz for initial WHO_AM_I probe
        let mut spi1 = dp.SPI1.spi(
            (sck, miso, mosi),
            stm32h7xx_hal::spi::Config::new(stm32h7xx_hal::spi::MODE_3)
                .communication_mode(stm32h7xx_hal::spi::CommunicationMode::FullDuplex),
            1.MHz(),
            ccdr.peripheral.SPI1,
            &ccdr.clocks,
        );

        defmt::info!("SPI1 configured: Mode 3, 1 MHz");

        // Read ICM-42688 WHO_AM_I register
        imu_cs.set_low();
        // Small delay after CS assert (ICM-42688 needs ~100ns setup time)
        cortex_m::asm::delay(100);

        // embedded-hal 0.2 SPI transfer: tx and rx happen in-place on the same buffer.
        // Write [reg|0x80, 0x00], read back response in same buffer.
        let mut buf: [u8; 2] = [ICM42688_WHO_AM_I | 0x80, 0x00];

        use stm32h7xx_hal::hal::blocking::spi::Transfer;
        match spi1.transfer(&mut buf) {
            Ok(result) => {
                imu_cs.set_high();
                let who = result[1];
                if who == ICM42688_WHO_AM_I_VALUE {
                    defmt::info!("ICM-42688 WHO_AM_I = 0x{:02x} -- CORRECT", who);
                } else {
                    defmt::warn!("SPI1 WHO_AM_I = 0x{:02x} (expected 0x47 for ICM42688; 0x12=ICM20602, 0x68=MPU6000)", who);
                }
            }
            Err(_) => {
                imu_cs.set_high();
                defmt::error!("SPI1 transfer failed");
            }
        }

        // ===================================================================
        // [STEP 4a] PRIMARY UART: USART1 TX @ 57600 on PA9 — MNP
        // ===================================================================
        //
        // MatekH743 SERIAL2 = USART1 (PA9 TX, PA10 RX). AF7.
        // Primary communication: Meridian Native Protocol (MNP).
        // COBS-framed postcard-serialized messages.
        // FC -> USART1 -> RFD 900x radio -> ATAK plugin.
        // No MAVLink bridge needed on this port.

        let tx_pin = gpioa.pa9.into_alternate::<7>();
        let rx_pin = gpioa.pa10.into_alternate::<7>();

        let serial_config = stm32h7xx_hal::serial::config::Config::new(57_600.bps());
        let mut usart1 = dp
            .USART1
            .serial(
                (tx_pin, rx_pin),
                serial_config,
                ccdr.peripheral.USART1,
                &ccdr.clocks,
            )
            .unwrap();

        defmt::info!("USART1 (PRIMARY): 57600 baud on PA9/PA10 — MNP protocol");

        // ===================================================================
        // [STEP 4b] SECONDARY UART: UART7 TX @ 57600 on PE8 — MAVLink
        // ===================================================================
        //
        // MatekH743 SERIAL1 = UART7 (PE8 TX, PE7 RX). AF7.
        // Secondary communication: MAVLink v2 adapter.
        // Standard MAVLink frames for QGC / Mission Planner compatibility.
        // This is NOT the native protocol — MNP on USART1 is primary.
        // Only active when a MAVLink GCS is connected.

        let uart7_tx = gpioe.pe8.into_alternate::<7>();
        let uart7_rx = gpioe.pe7.into_alternate::<7>();

        let uart7_config = stm32h7xx_hal::serial::config::Config::new(57_600.bps());
        let uart7_serial = dp
            .UART7
            .serial(
                (uart7_tx, uart7_rx),
                uart7_config,
                ccdr.peripheral.UART7,
                &ccdr.clocks,
            )
            .unwrap();
        // Split into tx/rx so we can read RX bytes from the main loop
        // (used to feed the MAVLink reboot-to-bootloader rescue parser).
        let (mut uart7, mut uart7_rx_half) = uart7_serial.split();

        defmt::info!("UART7 (SECONDARY): 57600 baud on PE8/PE7 — MAVLink v2 adapter");

        // ===================================================================
        // [STEP 4b.4] EARLY TIM6 — must fire before USB init so the tick
        // counter is alive for the 120s USB-silence self-reboot guard.
        // Without this, a USB init hang means TICK_COUNT stays at 0
        // forever and the self-reboot never fires.
        // ===================================================================
        unsafe {
            let rcc_ptr = &*pac::RCC::ptr();
            rcc_ptr.apb1lenr.modify(|_, w| w.tim6en().set_bit());
            let _ = rcc_ptr.apb1lenr.read().tim6en().bit();
        }
        unsafe {
            let tim6 = &*pac::TIM6::ptr();
            tim6.cr1.modify(|_, w| w.cen().clear_bit());
            tim6.psc.write(|w| w.psc().bits(199));
            tim6.arr.write(|w| w.arr().bits(999));
            tim6.dier.modify(|_, w| w.uie().set_bit());
            tim6.egr.write(|w| w.ug().set_bit());
            tim6.sr.modify(|_, w| w.uif().clear_bit());
            tim6.cr1.modify(|_, w| w.cen().set_bit());
        }
        unsafe {
            NVIC::unmask(pac::Interrupt::TIM6_DAC);
        }
        defmt::info!("TIM6: 1 kHz tick armed BEFORE USB (safety net foundation)");

        // ===================================================================
        // [STEP 4b.5] EARLY watchdog arm — before any peripheral init that
        // could hang. If USB init or anything below blocks for >2s the IWDG
        // fires, the H7 resets, the bootloader runs its DFU window, and
        // the host watcher catches it for re-flash. Without this, a USB
        // hang locks us out (no remote power-cycle until USB relay arrives).
        // ===================================================================
        let mut watchdog = meridian_platform_stm32::watchdog::Watchdog::new();
        watchdog.start();
        defmt::info!("IWDG armed (2s timeout) — pre-USB safety net");

        // ===================================================================
        // [NETTEST CANARY] Park here forever without feeding IWDG.
        // ===================================================================
        // Build with `--features nettest` to produce a canary firmware that
        // proves the safety-net chain end-to-end on the exact code state of
        // the surrounding build. After ~2s without a pat the IWDG fires,
        // the H7 resets, the bootloader sees the RTC magic that
        // `arm_rescue_flag()` wrote at boot, and stays in DFU. The host
        // watcher sees the bootloader and is free to flash whatever it
        // wants next. If the boat never reappears on USB within ~5s of
        // flashing this canary, something in the chain (rescue magic write,
        // IWDG arming, bootloader magic check) is broken on this build —
        // abort the iteration before flashing the real one.
        #[cfg(feature = "nettest")]
        {
            defmt::info!("NETTEST canary parking — expect bootloader within ~3s");
            loop {
                cortex_m::asm::nop();
                // Intentionally NOT calling watchdog.pat() — let IWDG fire.
            }
        }

        // ===================================================================
        // [STEP 4c] USB CDC ACM — production rescue path
        // ===================================================================
        //
        // On the Cube Orange Plus the USB-C connector wires to the H757's
        // OTG_FS pins:
        //   PA11 -> USB_DM   PA12 -> USB_DP   (alternate function 10)
        //
        // The stm32h7xx-hal exposes this via its `USB2` peripheral type
        // (despite the name; "USB2" here refers to the second OTG core,
        // running its internal FS PHY off PA11/PA12 — same wiring
        // ArduPilot uses on this board).
        //
        // The CDC RX is fed to a MAVLink rescue parser; receipt of a
        // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN with param1=3 over USB
        // triggers an immediate jump back to the bootloader. That's
        // the channel `uploader.py` and Mission Planner use to flash
        // new firmware without any physical button press.

        // USB 3.3V regulator is enabled by stm32h7xx-hal's USB::enable()
        // (in usb_hs.rs `pwr.cr3.modify(|_, w| w.usb33den().set_bit())`),
        // which runs inside synopsys-usb-otg's UsbBus::new path.
        // v0.5 manually wrote USBREGEN+USB33DEN AFTER `pwr.freeze()` had
        // already latched PWR.CR3 — the manual write was a silent no-op
        // and the USB33RDY poll just timed out.
        // Removing the manual block entirely; let the HAL handle it at
        // the right point in the init sequence.

        // Bring up HSI48 and route the USB kernel clock to it.
        // USBSEL bits: 00=disabled, 01=PLL1_Q, 10=PLL3_Q, 11=HSI48.
        // Every known-working H7 USB CDC reference uses HSI48:
        //   - olback/h7 (Portenta H7, sibling silicon) — `kernel_usb_clk_mux(Hsi48)`
        //   - Embassy stm32h7 example — `mux::Usbsel::Hsi48`
        //   - ArduPilot/ChibiOS hwdef — HSI48 with CRS sync from USB SOF
        // We tried PLL3_Q in v0.6 thinking "more deterministic" — it's
        // actually the opposite per stm32h7xx-hal#503; the only safe
        // path on H7 is HSI48.
        let mut hsi48_ready = false;
        unsafe {
            let rcc_ptr = &*pac::RCC::ptr();
            rcc_ptr.cr.modify(|_, w| w.hsi48on().set_bit());
            for _ in 0..1_000_000 {
                if rcc_ptr.cr.read().hsi48rdy().bit_is_set() {
                    hsi48_ready = true;
                    break;
                }
                cortex_m::asm::nop();
            }
        }
        if !hsi48_ready {
            defmt::error!("HSI48 not ready — USB CDC will be skipped");
        } else {
            defmt::info!("HSI48 ready (USB clock source)");
            unsafe {
                let rcc = &*pac::RCC::ptr();
                rcc.d2ccip2r.modify(|_, w| w.usbsel().bits(0b11)); // HSI48
            }
        }
        if hsi48_ready {
            use stm32h7xx_hal::gpio::Speed;
            let usb_rec = ccdr.peripheral.USB2OTG;
            // Cube Plus (and ArduPilot/ChibiOS) configure the USB pins
            // at VeryHigh speed for clean 12 Mbps edge rates. HAL default
            // is Speed::Low which some hosts have flagged as marginal.
            let usb_dm = gpioa.pa11.into_alternate::<10>().speed(Speed::VeryHigh);
            let usb_dp = gpioa.pa12.into_alternate::<10>().speed(Speed::VeryHigh);

            match meridian_platform_stm32::rescue::usb_cdc::init(
                dp.OTG2_HS_GLOBAL,
                dp.OTG2_HS_DEVICE,
                dp.OTG2_HS_PWRCLK,
                usb_dm,
                usb_dp,
                usb_rec,
                &ccdr.clocks,
            ) {
                Ok(()) => defmt::info!(
                    "USB CDC: ready on OTG2 FS PHY (VID 0x1209 PID 0x5741) — remote rescue path live"
                ),
                Err(e) => defmt::error!("USB CDC init failed: {:?}", defmt::Debug2Format(&e)),
            }
        }

        // ===================================================================
        // [STEP 5] TIM5 PWM at 50 Hz on PA0-PA3 (motor outputs 9-12)
        // ===================================================================
        //
        // MatekH743 M9-M12 = TIM5 ch1-ch4 on PA0-PA3 AF2.
        // 50 Hz PWM, 1500 us center pulse (safe for servo test without props).

        let pa0 = gpioa.pa0.into_alternate::<2>();
        let pa1 = gpioa.pa1.into_alternate::<2>();
        let pa2 = gpioa.pa2.into_alternate::<2>();
        let pa3 = gpioa.pa3.into_alternate::<2>();

        let (mut pwm1, mut pwm2, mut pwm3, mut pwm4) = dp.TIM5.pwm(
            (pa0, pa1, pa2, pa3),
            50.Hz(),
            ccdr.peripheral.TIM5,
            &ccdr.clocks,
        );

        // Set all channels to 1500 us (center/neutral)
        // At 50 Hz, period = 20,000 us. max_duty corresponds to full period.
        let max1 = pwm1.get_max_duty();
        let max2 = pwm2.get_max_duty();
        let max3 = pwm3.get_max_duty();
        let max4 = pwm4.get_max_duty();

        // 1500 us / 20000 us = 7.5% duty
        pwm1.set_duty(max1 * 1500 / 20000);
        pwm2.set_duty(max2 * 1500 / 20000);
        pwm3.set_duty(max3 * 1500 / 20000);
        pwm4.set_duty(max4 * 1500 / 20000);

        pwm1.enable();
        pwm2.enable();
        pwm3.enable();
        pwm4.enable();

        defmt::info!("TIM5 PWM: 50 Hz, 1500 us on PA0-PA3 (max_duty={})", max1);

        // TIM6 + watchdog are armed earlier (steps 4b.4 / 4b.5) — keep
        // patting watchdog from the main loop below.

        // ===================================================================
        // Main loop: LED blink + MNP heartbeat + MAVLink heartbeat + status
        // ===================================================================
        defmt::info!("=== Meridian Phase 9 MVP boot complete ===");
        defmt::info!("  Primary:   MNP heartbeat @ 1 Hz on USART1 (PA9)");
        defmt::info!("  Secondary: MAVLink HB   @ 1 Hz on UART7  (PE8)");

        let mut last_led_toggle: u32 = 0;
        let mut last_heartbeat: u32 = 0;
        let mut last_status: u32 = 0;
        let mut mnp_seq: u16 = 0;
        let mut mavlink_seq: u8 = 0;
        let mut rescue_cleared: bool = false;

        // Rescue parser for the UART7 RX stream. Same code path the USB
        // CDC RX uses (each interface owns its own parser instance to
        // avoid frame-boundary confusion when both are receiving).
        let mut uart7_rescue = meridian_platform_stm32::rescue::RebootCommandParser::new();

        loop {
            // Pat the IWDG every iteration. If anything below blocks for
            // >2s the chip resets, bootloader runs, host watcher recovers.
            watchdog.pat();

            let tick = TICK_COUNT.load(Ordering::Relaxed);

            // Drive USB CDC every loop iteration — must be called faster
            // than 1 ms to avoid host complaints. The function reads any
            // pending RX, feeds the rescue parser, and reboots into the
            // bootloader if a MAVLink reboot-to-bootloader command is
            // received. The same parser also decodes MAV_CMD_DO_SET_SERVO
            // and stashes pending servo commands for the main loop below.
            meridian_platform_stm32::rescue::usb_cdc::poll();

            // Self-recovery safety net: if no USB host event has happened
            // by 120s after boot, the USB CDC bring-up failed silently.
            // Reboot to bootloader so the host watcher can re-flash a
            // working firmware. This guarantees we can never lock the
            // boat into an un-reachable state via a broken USB stack.
            const USB_SILENCE_TIMEOUT_TICKS: u32 = 120_000;
            if tick > USB_SILENCE_TIMEOUT_TICKS
                && !meridian_platform_stm32::rescue::usb_cdc::has_enumerated()
            {
                defmt::error!(
                    "USB never enumerated after {}s — rebooting to bootloader",
                    USB_SILENCE_TIMEOUT_TICKS / 1000
                );
                meridian_platform_stm32::rescue::reboot_to_bootloader();
            }

            // Once USB has enumerated AND we've been alive for at least
            // 5s, clear the boot-magic safety flag we set at boot — but
            // ONLY once (was running every loop iteration which churns
            // the slow backup-domain APB needlessly).
            if !rescue_cleared
                && tick > 5_000
                && meridian_platform_stm32::rescue::usb_cdc::has_enumerated()
            {
                meridian_platform_stm32::rescue::clear_rescue_flag();
                rescue_cleared = true;
            }

            // Drain any pending DO_SET_SERVO command and apply it live.
            // MAVLink servo channel mapping on the Cube Plus carrier:
            //   SERVO 9  -> AUX OUT 1 = PA0 = TIM5_CH1 = pwm1
            //   SERVO 10 -> AUX OUT 2 = PA1 = TIM5_CH2 = pwm2
            //   SERVO 11 -> AUX OUT 3 = PA2 = TIM5_CH3 = pwm3
            //   SERVO 12 -> AUX OUT 4 = PA3 = TIM5_CH4 = pwm4
            // Other servo IDs are silently dropped.
            if let Some((servo_ch, pwm_us)) = meridian_platform_stm32::rescue::take_pending_servo() {
                // PWM period at 50 Hz = 20_000 us. Duty for a given pulse
                // width: max_duty * pulse_us / 20_000.
                let target_duty = |max: u32| -> u32 {
                    max.saturating_mul(pwm_us as u32) / 20_000
                };
                match servo_ch {
                    9 => pwm1.set_duty(target_duty(max1)),
                    10 => pwm2.set_duty(target_duty(max2)),
                    11 => pwm3.set_duty(target_duty(max3)),
                    12 => pwm4.set_duty(target_duty(max4)),
                    _ => defmt::trace!("DO_SET_SERVO unmapped channel {}", servo_ch),
                }
                defmt::info!("DO_SET_SERVO ch{} = {} us", servo_ch, pwm_us);
            }

            // Poll UART7 RX. Each byte is fed to the rescue parser; on
            // detecting MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(param1=3) we
            // jump back to the bootloader. This makes the radio link
            // (RFD 900 -> UART7) a viable rescue path even when USB is
            // unplugged.
            {
                use stm32h7xx_hal::nb;
                use stm32h7xx_hal::hal::serial::Read as SerialRead;
                while let Ok(byte) = SerialRead::read(&mut uart7_rx_half) {
                    if uart7_rescue.feed(byte) {
                        defmt::warn!("UART7 rescue command received — rebooting to bootloader");
                        meridian_platform_stm32::rescue::reboot_to_bootloader();
                    }
                    let _: u8 = byte;
                    let _ = nb::Error::<()>::WouldBlock; // keep nb in scope for future use
                }
            }

            // Toggle PA4 from the main context based on TIM6_TOGGLE flag.
            // We handle the GPIO toggle here because we own the pin. The ISR
            // just sets the flag. This gives a clean 1 kHz square wave on PA4
            // minus the small latency from WFI wakeup.
            if TIM6_TOGGLE.swap(false, Ordering::Relaxed) {
                if scope_pin.is_set_high() {
                    scope_pin.set_low();
                } else {
                    scope_pin.set_high();
                }
            }

            // LED blink at 2 Hz (toggle every 250 ms = 250 ticks)
            if tick.wrapping_sub(last_led_toggle) >= 250 {
                last_led_toggle = tick;
                if led.is_set_high() {
                    led.set_low();
                } else {
                    led.set_high();
                }
            }

            // Heartbeat at 1 Hz (every 1000 ms = 1000 ticks)
            if tick.wrapping_sub(last_heartbeat) >= 1000 {
                last_heartbeat = tick;

                // ─── PRIMARY: MNP heartbeat on USART1 ───
                //
                // Build an MnpMessage::Heartbeat, serialize with postcard,
                // COBS-frame it, and send over USART1. This is what the
                // RFD 900x radio and ATAK plugin receive natively.
                //
                // Frame on wire: [0x00] [COBS(0x01 + seq_le + postcard(Heartbeat))] [0x00]
                let mnp_hb = MnpMessage::Heartbeat(Heartbeat {
                    vehicle_type: 1,     // quad
                    armed: false,
                    mode: 0,             // stabilize
                    system_status: 3,    // standby
                });

                let mut mnp_frame = [0u8; MAX_FRAME_SIZE];
                let mnp_len = mnp_hb.encode(mnp_seq, &mut mnp_frame);
                mnp_seq = mnp_seq.wrapping_add(1);

                if mnp_len > 0 {
                    use stm32h7xx_hal::hal::blocking::serial::Write as BlockingWrite;
                    let _ = usart1.bwrite_all(&mnp_frame[..mnp_len]);
                    defmt::trace!("MNP HB #{} ({} bytes)", mnp_seq, mnp_len);
                } else {
                    defmt::error!("MNP heartbeat encode failed");
                }

                // ─── SECONDARY: MAVLink v2 heartbeat on UART7 ───
                //
                // Standard MAVLink frame for QGC/Mission Planner compatibility.
                // Runs on a separate UART, independent of MNP.
                let mut mav_frame: [u8; 21] = [0; 21];
                mav_frame[..18].copy_from_slice(&MAVLINK_HEARTBEAT_TEMPLATE);
                mav_frame[4] = mavlink_seq;           // sequence number in header
                mav_frame[18] = MAVLINK_VERSION_BYTE; // 9th payload byte (mavlink_version=3)

                mavlink_seq = mavlink_seq.wrapping_add(1);

                // CRC covers bytes 1..19 (skip STX at [0], include all header+payload)
                let crc = mavlink_crc(&mav_frame[1..19], HEARTBEAT_CRC_EXTRA);
                mav_frame[19] = (crc & 0xFF) as u8;
                mav_frame[20] = (crc >> 8) as u8;

                // Transmit via UART7 (blocking for MVP). After the
                // `serial.split()` we have a `Tx` half that doesn't
                // auto-impl `BlockingWrite::Default`, so we drive the
                // `nb`-style write byte-by-byte.
                {
                    use stm32h7xx_hal::hal::serial::Write as SerialWrite;
                    for &b in &mav_frame {
                        while SerialWrite::write(&mut uart7, b).is_err() {}
                    }
                    while SerialWrite::flush(&mut uart7).is_err() {}
                }

                // Also emit the same MAVLink heartbeat over USB CDC so
                // a host connected to the Cube's USB-C port sees a live
                // autopilot. Best-effort: if no host is reading, the
                // serial layer drops silently. Critical for blackbox /
                // Mission Planner / QGC over USB without needing a
                // separate UART-to-USB adapter on TELEM1.
                meridian_platform_stm32::rescue::usb_cdc::write_bytes(&mav_frame);

                defmt::trace!("MAV HB #{}", mavlink_seq);
            }

            // Status log every 5 seconds
            if tick.wrapping_sub(last_status) >= 5000 {
                last_status = tick;
                let dwt_cycles = cortex_m::peripheral::DWT::cycle_count();
                defmt::info!("Alive: tick={}, DWT={}, MNP#{}, MAV#{}",
                    tick, dwt_cycles, mnp_seq, mavlink_seq);
            }

            // No WFI here — USB CDC needs to be polled faster than 1 ms.
            // The cortex-m on H7 @ 400 MHz spins fine; if we want lower
            // power later, move USB to interrupt-driven polling and
            // restore WFI. Same applies for UART7 RX once we move it
            // to interrupt + DMA.
        }
    }

    // ===================================================================
    // TIM6 ISR — 1 kHz tick
    // ===================================================================
    //
    // This is a bare interrupt handler (not RTIC). For the MVP we just
    // increment a counter and set a toggle flag. The full RTIC app will
    // replace this with proper task scheduling.
    #[interrupt]
    unsafe fn TIM6_DAC() {
        // Clear the update interrupt flag
        let tim6 = &*pac::TIM6::ptr();
        tim6.sr.modify(|_, w| w.uif().clear_bit());

        // Increment global tick counter
        TICK_COUNT.fetch_add(1, Ordering::Relaxed);

        // Signal main loop to toggle scope pin
        TIM6_TOGGLE.store(true, Ordering::Relaxed);
    }
}
