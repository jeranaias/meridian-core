#![no_std]

//! STM32H743 bare-metal platform for Meridian autopilot.
//!
//! Implements `meridian-hal` traits targeting the Matek H743 flight controller
//! (STM32H743VIT6) via direct PAC/HAL register access and RTIC task scheduling.
//!
//! # Memory Layout (H743)
//!
//! | Region   | Size   | Address        | DMA-safe | Usage                          |
//! |----------|--------|----------------|----------|--------------------------------|
//! | ITCM     | 64 KB  | 0x0000_0000    | No       | Fast instruction fetch         |
//! | DTCM     | 128 KB | 0x2000_0000    | **No**   | Stack, local vars (fastest)    |
//! | AXI SRAM | 512 KB | 0x2400_0000    | Yes      | Main heap, DMA buffers         |
//! | SRAM1    | 128 KB | 0x3000_0000    | Yes      | DMA buffers                    |
//! | SRAM2    | 128 KB | 0x3002_0000    | Yes      | DMA buffers                    |
//! | SRAM3    | 32 KB  | 0x3004_0000    | Yes      | DMA buffers                    |
//! | SRAM4    | 64 KB  | 0x3800_0000    | Yes      | Uncached, bidirectional DShot  |
//!
//! **CRITICAL:** DTCM is NOT DMA-accessible on H7. Using DTCM for DMA = silent
//! data corruption. All DMA buffers MUST be placed in AXI SRAM or SRAM1-4.
//!
//! # RTIC Priority Mapping (from ChibiOS threads)
//!
//! | RTIC Priority | ChibiOS Equivalent    | Rate    | Function                     |
//! |---------------|-----------------------|---------|------------------------------|
//! | 4             | Timer thread (181)    | 1 kHz   | IMU sampling, sensor accum   |
//! | 3             | Main thread (180)     | 400 Hz  | Flight control loop          |
//! | 2             | RC out/in (181/177)   | event   | Motor output, RC decode      |
//! | 1             | IO thread (58)        | 50 Hz   | GPS, baro, mag, telemetry    |

// All hardware-dependent modules are gated behind target_arch = "arm"
// so the crate compiles cleanly in workspace `cargo check` on the host.

#[cfg(target_arch = "arm")]
pub mod clock;
#[cfg(target_arch = "arm")]
pub mod memory;
#[cfg(target_arch = "arm")]
pub mod dma;
#[cfg(target_arch = "arm")]
pub mod spi;
#[cfg(target_arch = "arm")]
pub mod i2c;
#[cfg(target_arch = "arm")]
pub mod uart;
#[cfg(target_arch = "arm")]
pub mod gpio;
#[cfg(target_arch = "arm")]
pub mod pwm;
#[cfg(target_arch = "arm")]
pub mod adc;
#[cfg(target_arch = "arm")]
pub mod flash;
#[cfg(target_arch = "arm")]
pub mod watchdog;
#[cfg(target_arch = "arm")]
pub mod rtic_app;
#[cfg(target_arch = "arm")]
pub mod rescue;
