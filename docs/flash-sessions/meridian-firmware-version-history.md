# Meridian STM32 Firmware Version History

Each version below corresponds to a flashed `.apj` plus its outcome on the live Cube Orange Plus hardware (Vanguard USV bench, 2026-04-26 → 2026-05-04). The recurring failure mode is "flash succeeds, board boots, USB CDC silent."

| Version | Built | Flashed | Size | Result | What was wrong |
|---|---|---|---|---|---|
| v0.1 (`meridian-cubeorangeplus.apj`) | 2026-04-26 | 2026-04-26 | 9,992 B | ❌ silent on USB | Phase-9 MVP stub. No USB CDC code at all — just MNP/MAVLink heartbeat on UARTs. Recovery required a power cycle. |
| v0.2 (`-v0.2-rescue.apj`) | 2026-04-27 | 2026-04-27 | 25,196 B | ❌ silent on USB | Added USB CDC code path + reboot-to-bootloader handler scaffolding. USB CDC didn't enumerate — root cause not yet diagnosed. |
| v0.3 (`-v0.3-rescue.apj`) | 2026-04-27 | 2026-04-27 | 25,176 B | ❌ silent on USB | Refined v0.2 (HSI48 enable, USBSEL routing, PA11/PA12 AF10 wiring). All scaffolding in place — code-level audit later confirmed v0.3 already had heartbeat + servo + USB scaffolding. Still silent on USB. |
| v0.4 (`-v0.4-servo.apj`) | 2026-05-03 | 2026-05-03 | 25,804 B | ❌ silent on USB | First DO_SET_SERVO command handler + USB heartbeat plumbing added. Servo-channel mapping (ch9→PA0/TIM5_CH1) wired. **USB still silent — unrelated to anything we changed.** |
| v0.5 (`-v0.5-usb33.apj`) | 2026-05-03 | 2026-05-03 | 25,840 B | ❌ silent on USB | Added manual `PWR_CR3.USB33DEN + USBREGEN` enable with `USB33RDY` poll. Theory: pwr.freeze() doesn't enable USB regulator, transceiver pull-up has no power. Wrong: regulator wasn't the issue. |
| v0.6 (`-v0.6-pll3.apj`) | 2026-05-03 | 2026-05-03 | 25,820 B | ❌ silent on USB | Switched USB clock from HSI48 to PLL3_Q at 48 MHz. Theory: HSI48 not stable. Wrong: clock source wasn't the issue. |
| v0.7 (`-v0.7-vbus.apj`) | 2026-05-04 | 2026-05-04 | 26,128 B | ❌ silent on USB + locked-out | Theory from research agent: VBUS sensing on self-powered design; needed `GOTGCTL.BVALOEN+BVALOVAL=1` and `GCCFG.VBDEN=0`. **Theory was correct; my register write was wrong** — used VBDEN bit 24 (incorrect) instead of bit 21 (correct per ChibiOS source). Self-recovery 120s timeout broken because TIM6 init was AFTER USB init that hangs. Required Tristan power-cycle to recover. |
| v0.8 (`-v0.8-fs-mode.apj`) | 2026-05-04 | 2026-05-04 | 26,200 B | ❌ silent on USB + locked-out | Vendored `stm32h7xx-hal` with `USB2::HIGH_SPEED=false` (right idea, alone insufficient). Boot-magic safety net wrote `0x5A5A5A5A` to DTCM `0x2001_FFFC` — **but the ArduPilot bootloader doesn't check that location!** The actual bootloader-stay magic is `0xb0070001` (`RTC_BOOT_HOLD`) stored in `RTC->BKP0R`. Discovered after lock-out by reading ArduPilot's `enum rtc_boot_magic` in `libraries/AP_HAL_ChibiOS/hwdef/common/stm32_util.h`. Net effect: every "self-recovery" mechanism in v0.1–v0.8 wrote to a memory location the bootloader silently ignores. Required Tristan power-cycle to recover. |
| v0.9 (`-v0.9-rtc-magic.apj`) | 2026-05-04 | 2026-05-04 | 26,308 B | ❌ silent on USB + locked-out | RTC backup register safety net (right magic + address) BUT `RCC.APB4ENR.RTCAPBEN` was never enabled, so BKP0R writes were silent no-ops. synopsys-usb-otg barrier fork applied. HSI48 clock restored. Required Tristan recovery cycle. |
| v1.0 (`-v1.0-rtcapben.apj`) | 2026-05-04 | 2026-05-04 | 26,336 B | ❌ silent on USB + locked-out | Added `RCC.APB4ENR.RTCAPBEN` enable. Verified read-back of magic. Still failed because (a) HSE clock declared as 8 MHz when actually 24 MHz on Cube Plus → all derived clocks wrong; (b) manual `USBREGEN` write was AFTER `pwr.freeze()` (silent no-op on H7's write-once CR3). |
| v1.2 (`-v1.2-hse-vos-pwr.apj`) | 2026-05-04 | 2026-05-05 | 26,348 B | ❌ silent on USB ✅ **safety net WORKED for first time** | **The big systemic root causes.** (1) **HSE = 24 MHz, not 8 MHz.** v0.1–v1.0 all declared 8 MHz. With wrong HSE, HAL's PLL math targets 1200 MHz VCO (over H7 max) → fallback or silent miscalc → AHB and TIM6 wrong → USB FIFO timing wrong AND safety net timer wrong (which is why it never appeared to fire). (2) Removed manual `USBREGEN` write after `pwr.freeze()` (write-once-after-freeze silent no-op). (3) Explicit `pwr.vos1().freeze()`. (4) USB pin `Speed::VeryHigh`. (5) `clear_rescue_flag()` one-shot. (6) `rescue_flag_was_set()` enables backup domain access first. **OUTCOME**: USB still silent BUT the safety net fired correctly — bootloader visible at COM3 within ~8s (IWDG fired). First iteration where the boat doesn't lock out without Tristan. **HSE was a real fix, USB still has another bug.** |
| v1.3 (`-v1.3-h747cm7.apj`) | 2026-05-05 | 2026-05-05 | ~26,300 B | ❌ silent on USB + safety-net BROKE + locked out | Single change vs v1.2: `stm32h7xx-hal` feature flag `stm32h743v` → `stm32h747cm7` (proper RM0399 register layout for H757 chip). Theory: HAL feature mismatch was contributing. **Result: catastrophically wedged.** No USB enumeration AND no safety net firing for 150 seconds — meaning RM0399 PAC's register definitions broke clock setup or peripheral access early in boot, before TIM6/IWDG were armed. Required Tristan power cycle to recover. **Definitive evidence the binary should stay on `stm32h743v`** despite running on H757 silicon. |
| v1.4 (`-v1.4-no-override.apj`) | 2026-05-05 | 2026-05-05 | 26,312 B | ❌ silent on USB + safety-net BROKE + locked out | Single change vs v1.2: dropped the manual GOTGCTL/GCCFG override block. **Empirical result: same as v1.3** — full lockout, no DFU window for 170 sec. Pre-flash hypothesis: "manual block was a no-op since synopsys's CSRST resets GCCFG anyway, removing it is safe." **Hypothesis was wrong.** Removing the block somehow disabled the safety net. Doesn't physically make sense (those registers are USB peripheral, shouldn't affect TIM6/IWDG/main loop) but is empirically reproducible. Required Tristan recovery cycle. **Lesson: ANY change to USB init path can break the safety net path even when it shouldn't, theoretically. Don't iterate USB init without RTT visibility OR a USB relay for free recovery.** |

## Iteration freeze (post-v1.4)

Stopping firmware iteration until either:
- **USB relay arrived** (NYBG LCUS-1, ordered, ~2 days). Enables remote power cycle from SSH — every flash is free, even with broken safety net.
- **Debug probe attached at the bench** (ST-Link V3 / Pi Debug Probe). Enables `probe-rs attach` over SSH for live `defmt-rtt` logs — pinpoints exactly which init step is the last one before silence.

Both v1.3 and v1.4 broke the safety net via single small changes. The factor that makes v1.2's safety-net work is brittle and not understood without runtime visibility. Continuing blind iteration burns Tristan's time AND the Cube's flash cycles for no informational gain.

## v1.2 is the current "known-working safety-net baseline"

If a recovery cycle goes badly and we need to flash something fresh on the bench, **v1.2** (`meridian-cubeorangeplus-v1.2-hse-vos-pwr.apj`) is the binary to use. It was the only Meridian variant today where the safety net was verified to fire. USB still silent on it, but the boat is recoverable.

## 2026-05-06 milestones — never need Tristan again

After today's v1.4 lockout cycle, two pieces of infrastructure landed that close the gap from "I think the safety net works" to "I just observed it work on this code state":

### Empirical confirmation of v1.2 safety net
With ArduRover restored as a known-good fallback, v1.2 was reflashed and the host watched USB enumeration for 240 seconds. The TIM6 120-sec USB-silence reboot fired exactly as designed: v1.2 booted → no CDC → 120s elapsed → firmware self-reset → bootloader saw RTC magic → DFU enumerated → host poll caught it. **First time the chain was directly observed end-to-end.**

### NETTEST canary build (`--features nettest`)
A `nettest` Cargo feature on `meridian-stm32-bin` swaps the post-IWDG path for an infinite `nop` loop:

```rust
let mut watchdog = ::watchdog::Watchdog::new();
watchdog.start();
#[cfg(feature = "nettest")]
{
    loop { cortex_m::asm::nop(); }   // never feed IWDG
}
// (USB init only in non-nettest builds)
```

Build: `RUST_MIN_STACK=16777216 rustup run 1.86.0 cargo build --target thumbv7em-none-eabihf -p meridian-stm32-bin --release --features nettest`. Resulting `.bin` is ~9.7 KB (no USB code). Flashed via the existing watcher pipeline.

**Empirical canary result (2026-05-06)**: bootloader visible on USB within ~5 seconds of canary boot. Confirms — for *this* compiler output, *this* peripheral state, *this* board — that:
1. `arm_rescue_flag()` actually writes the RTC magic
2. IWDG actually arms
3. IWDG actually fires when not patted
4. ArduPilot bootloader actually checks RTC->BKP0R
5. ArduPilot bootloader actually holds in DFU

### Iteration protocol from now on
Every USB-CDC iteration produces TWO builds: the real firmware AND a `--features nettest` canary built from the same source. Flash the canary first. If bootloader appears within 5 sec → chain proven for this code state → flash the real one. If canary fails → abort, debug compiler output, investigate before risking a real flash. **Cost per canary cycle: ~5 sec of bootloader + one flash. Cheap insurance.**

Combined with the USB relay (~2 days out) for hardware-level backstop, the boat genuinely cannot get locked out without explicit canary failure.

## Hardware facts (Cube Orange Plus / STM32H757)

- **MCU**: STM32H757IIK (dual-core M7+M4, single-core M7 used). RM0399 register family.
- **HSE crystal**: **24 MHz** (per ArduPilot CubeOrange hwdef.inc `OSCILLATOR_HZ 24000000`).
- **USB-C connector** wires to OTG2 (FS-only controller) on PA11/PA12 with **AF10**.
  - OTG1 is HS-capable and ULPI-routed to PB14/PB15 (not used on this board).
- **Bootloader stay-in-DFU magic**: `0xb0070001` (`RTC_BOOT_HOLD`) stored in `RTC->BKP0R`. Per ArduPilot's `enum rtc_boot_magic` in `libraries/AP_HAL_ChibiOS/hwdef/common/stm32_util.h`.
- **Flash layout**: bootloader at `0x08000000` (128 KB), app at `0x08020000`.
- **VBUS not wired** to any OTG_VBUS alternate function — VBUS goes to PA9 as plain GPIO. Self-powered design; OTG core must override session-valid via GOTGCTL.

## v1.2 architecture (current)

### Clock setup
- `pwr.vos1().freeze()` (explicit Scale1 for 400 MHz)
- `use_hse(24.MHz())` — actual crystal
- `sys_ck = 400 MHz`, `hclk = 200 MHz`, `pclk1..4 = 100 MHz`
- `pll2_p_ck(200.MHz())` for SPI clocks
- HSI48 enabled manually after `freeze()`, `RCC.D2CCIP2R.USBSEL = 0b11` to route USB to HSI48

### USB CDC stack
- Vendored `stm32h7xx-hal` at `vendor/stm32h7xx-hal/` with `USB2.HIGH_SPEED = false` patch — makes synopsys-usb-otg run its FS init path (sets `GCCFG.PWRDWN=1`, clears `VBDEN`, sets `GOTGCTL.BVALOEN+BVALOVAL`)
- `synopsys-usb-otg` patched via `[patch.crates-io]` to AetherWareFoundation's `work-h7coreresethang` fork — adds `isb/dsb/dmb` barriers before `GRSTCTL.AHBIDL/CSRST` polling (cures stm32h7xx-hal#503 H7+M7 hang)
- USB pins PA11/PA12 with `Alternate<10>` and `Speed::VeryHigh`
- `USB2OTG` peripheral, `OTG2_HS_GLOBAL/DEVICE/PWRCLK` register blocks
- Manual VBUS override block (lines 705-721 of rescue.rs) is now redundant with HIGH_SPEED=false but kept as belt-and-suspenders documentation

### Boot sequence (top-of-`main`)
1. **`rescue_flag_was_set()`** — read RTC->BKP0R after enabling RTCAPBEN (logs `from_rescue` for debug)
2. **`arm_rescue_flag()`** — enables RCC.APB4ENR.RTCAPBEN + PWR.CR1.DBP, ensures RTCEN, writes `0xb0070001` to BKP0R, DSB+ISB, returns whether read-back equals magic
3. `pwr.vos1().freeze()`
4. `RCC.constrain().use_hse(24.MHz())...freeze()`
5. GPIO splits, SPI1 IMU probe, USART1 (MNP), UART7 (MAVLink)
6. **TIM6 1 kHz tick** (BEFORE USB so safety-net counter is real)
7. **IWDG 2-second watchdog** (BEFORE USB so any hang triggers reset)
8. HSI48 + USBSEL routing
9. **USB CDC init** — synopsys with HIGH_SPEED=false runs FS path
10. TIM5 PWM 50 Hz on PA0-PA3
11. Main loop: pat watchdog, drain pending DO_SET_SERVO, USB poll, MNP+MAVLink heartbeat, 120s USB-silence guard, one-shot rescue-flag clear

### Three safety nets (any one keeps boat recoverable without Tristan)
1. **IWDG (2s)** — armed before USB. If USB init hangs, watchdog resets within 2 seconds. Bootloader runs, sees `RTC_BOOT_HOLD` in BKP0R (set in step 2), stays in DFU forever, host watcher catches.
2. **120s USB-silence self-reboot** — main loop checks `tick > 120000 && !has_enumerated()`. If true, calls `reboot_to_bootloader()` which writes magic + sys_reset.
3. **`arm_rescue_flag` at boot start** — magic is set BEFORE any peripheral init that could hang. ANY reset cause from there forward results in the bootloader staying in DFU. Cleared only after USB enumerates AND firmware has been alive 5 seconds (one-shot via `rescue_cleared` flag).

## Open questions / remaining risk for v1.2

- IWDG reset path through the ChibiOS bootloader: the bootloader has a watchdog-override branch (`if (was_watchdog && m != RTC_BOOT_FWOK) { ... }`) that should keep us in DFU on watchdog reset. Untested on this exact board.
- HOLD branch is one-shot per write: the bootloader clears BKP0R after reading HOLD. If any second reset happens before the watcher catches, magic is gone. Mitigation: firmware re-arms the magic at every boot start.
- HAL feature flag is `stm32h743v` (RM0433) on a chip that's actually H757 (RM0399). Most peripherals register-compatible; USB code paths verified equivalent. Unrelated peripherals may have subtle differences.

## Reference for instrumentation (post-v1.2)

If v1.2 also fails:
- Attach debug probe (ST-Link V3 Mini / Pi Debug Probe). Run probe-rs attach, stream defmt-rtt logs, pinpoint last log line before silence.
- Run stm32h7xx-hal `examples/usb_serial.rs` on a Nucleo-H743ZI as a known-working baseline.
- Try Embassy stack (`embassy-stm32` + `embassy-usb`) instead of synopsys-usb-otg — Embassy ships extra H7-specific errata fixes (PRs #2677 and #2823) that synopsys-usb-otg never picked up.
