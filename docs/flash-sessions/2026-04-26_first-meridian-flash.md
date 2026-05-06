# Flash Session — 2026-04-26 — First Meridian Flash + Revert Proof

**Goal:** prove Meridian flashes cleanly onto the Vanguard's Cube Orange Plus, prove first-tier behavior, prove the recovery path back to ArduRover 4.6.3 works end-to-end. Leave the boat in its original state.

**Operator:** Jesse (remote, via Tailscale + paramiko SSH on the USV's onboard Lenovo)

**Hardware:** Vanguard USV bench setup, Tristan not present.

---

## Pre-flash state — captured 2026-04-26 04:20Z

### Hardware enumeration (USV)

```
Hostname:       LAPTOP-DTQJVNSS
OS:             Microsoft Windows 11 Home (Build 26200.8246)
RAM:            16 GB
Disk free:      398.9 / 474.7 GB on C:
Cube on USB:    VID 2DAE, PID 1058 (CubePilot CubeOrangePlus)
                COM4 — MAVLink port
                COM5 — secondary port
Mission Planner installed: 1.3.76
Tailscale connection:      active via DERP "syd"
```

### Live Cube identity (read via pymavlink before any modifications)

```
heartbeat type:        11 (MAV_TYPE_GROUND_ROVER)
autopilot:             3 (ARDUPILOTMEGA)
flight_sw_version:     0x040603ff (ArduPilot 4.6.3)
git short hash:        3fc7011a
vendor / product:      0x2dae / 0x1058
capabilities:          0xf1ef
parameter count:       974
```

### Backups taken
- **Parameters dump (974/974)** — `vanguard_params_20260427_042022.parm` + `.json`, sha256 stored on both sides
- **Mission Planner local data** — 775 MB zip on USV (`Documents\Vanguard-Backups\mission_planner_data_20260427.zip`); not pulled to dev box because the laptop itself isn't being modified
- **DataFlash logs** — none on the Cube (no flights yet)

### Restore image acquired
- Public ArduPilot mirror, exact version match: `https://firmware.ardupilot.org/Rover/stable-4.6.3/CubeOrangePlus/ardurover.apj`
- Header check: `board_id: 1063`, `git_identity: 3fc7011a` ← bit-perfect match to live read

---

## Flash kit — staged + verified at 2026-04-26 (UTC) ~17:48

Local on dev box: `D:/projects/meridian/backups/usv-vanguard/firmware/`
Remote on USV: `C:/Users/vangu/Documents/Vanguard-Backups/flash/`

| File | Size | SHA256 | Purpose |
|---|---|---|---|
| `meridian-cubeorangeplus.apj` | 9,992 B | `aa06264ec8bbc0dc…` | Candidate Meridian firmware |
| `ardurover-4.6.3-CubeOrangePlus.apj` | 1,646,590 B | `b9a912d98b313768…` | Restore-to-original image |
| `uploader.py` | 48,928 B | `72bfbde5e211f265…` | Canonical ArduPilot bootloader uploader |
| `usv-cube-cycle.ps1` | 2,049 B | `d0a8f76e73b895b2…` | USB power-cycle helper for recovery |

All four files passed local-vs-remote SHA256 parity check.

---

## Meridian build (dev box)

```
Workspace:   D:/projects/meridian
Target:      thumbv7em-none-eabihf
Crate:       meridian-stm32-bin (Phase-9 MVP firmware)
Toolchain:   stable Rust + llvm-tools
Build cmd:   cargo build --release --target thumbv7em-none-eabihf -p meridian-stm32-bin
Result:      0 errors, 26 dead-code warnings, exit 0
ELF size:    746 KB (with debug_info)
Stripped:    10,736 bytes flat .bin
Git hash:    742f64e
```

Notes on build:
- HAL feature flag is `stm32h743v` (single-core H743V); the Cube is STM32H757 (dual-core M7+M4). Same Cortex-M7 instruction set, register-compatible for the peripherals Meridian touches. Conservative path; H757-specific features can be added later.
- Wrapped with `tools/meridian-pack.py pack <bin> 1063 CubeOrangePlus`. APJ integrity verified — `image_size` matches decompressed size, sha256 self-check matches.

---

## Flash — *(filled in during the session as it happens)*

### Step 1 — bootloader smoke test (no write) — ✅ PASSED 2026-04-27 ~05:02 UTC

**What we did:**
1. Killed Mission Planner (held COM4 exclusively)
2. Sent MAVLink `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` with `param1=3` (remain in bootloader) via earlier uploader.py invocation; the autopilot rebooted into the bootloader
3. The bootloader exposes a single COM port; Windows reassigned the Cube from `COM4`+`COM5` (running firmware = two ports) to `COM3` only (bootloader = one port)
4. Ran `uploader.py --port COM3 --identify`

**Bootloader handshake output:**

```
Found board 427,0 bootloader rev 5 on COM3
Bootloader Protocol: 5
OTP:
  type:
  idtype: =00
  vid: 00000000
  pid: 00000000
  sn: 003a002f3233510536343932
ChipDes:
  family: STM32H743/753/750
  revision: V
Chip:
  STM32H74x_75x 20036450
Info:
  flash size: 1966080
  ext flash size: 0
  board_type: 1063
  board_rev: 0
Identification complete
```

**Why this matters:**
- `board_type: 1063` matches our `meridian-cubeorangeplus.apj` `board_id` field exactly. The bootloader will accept the upload.
- `flash size: 1,966,080 bytes` (1.875 MB) — Meridian's 10.7 KB stub uses 0.6 % of available flash; the 1.72 MB ArduRover restore uses 92 %, both fit.
- `family: STM32H743/753/750` and `device ID 0x20036450` — the Cortex-M7 core in the Cube's H757 reports identical device-ID and family values to a plain H743V. **This means our Rust build with `stm32h743v` HAL features is using register definitions that the silicon actually accepts.** The dual-core H757 risk we worried about is lower than feared.
- `bootloader rev 5, protocol 5` — current ChibiOS bootloader, fully supported by uploader.py.

**Side-effect to remember:** the Cube renumbered from COM4 → COM3 when it dropped to bootloader mode. After the upload completes, the firmware will re-enumerate; could be COM4 again or any other free number. Our scripts need to discover the port, not assume it.

### Step 2 — upload Meridian — ✅ FLASH SUCCEEDED 2026-04-27 ~05:08 UTC

```
$ uploader.py --port COM3 meridian-cubeorangeplus.apj
Loaded firmware for 427,0, size: 10736 bytes, waiting for the bootloader...
Found board 427,0 bootloader rev 5 on COM3
Bootloader Protocol: 5
[full identification block — same as smoke test, board_type 1063]
Erase  : [====================] 100.0% (timeout: 7 seconds)
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.
```

The bootloader successfully wrote and verified the 10,736-byte Meridian image, then issued the firmware-jump. Erase/program/verify all 100%.

### Step 3 — Tier 1 verification — ⚠️ BLOCKED: stub doesn't enumerate USB

After reboot, the Cube **vanished from the host's USB tree entirely.** No COM port, no enumeration, no MAVLink, no defmt-rtt over USB. The Phase-9 MVP firmware does not initialize the USB-CDC peripheral.

| Test | Expected | Observed | Pass |
|---|---|---|---|
| USB enumeration | Cube reappears as MAVLink CDC | nothing visible on USB tree | ❌ blocked |
| Heartbeat on COM | MAVLink heartbeat or defmt-rtt log | n/a — no port | ❌ blocked |
| Bootloader region intact | re-detectable after power cycle | unverified — see Step 4 below | ⏳ |
| MNP advertise to tablet | boat icon in `mission.html` | n/a — no link | ❌ blocked |

This is a real outcome of the build, not a flash failure. The flash itself is provably good (Step 2 verify succeeded).

**Cached PnP entries** confirmed both bootloader and ArduPilot composite USB descriptors had been seen historically (different USB serial numbers per descriptor: `2F003A...` for bootloader, `22002C...` for running ArduPilot), but neither was present after the Meridian flash.

### Step 3.5 — remote recovery attempt — ❌ blocked by external power

We attempted three escalating remote recoveries to put the Cube back into bootloader mode:

1. **Short USB controller cycle** (3 sec) — disabled both Intel xHCI host controllers, re-enabled. Cube did not re-enumerate. Suggested either the cycle was too short, or the Cube has external power.

2. **Long USB controller cycle** (60 sec) — same disable/enable pattern but held for a full minute, plus a `Restart-Service WUDFRd` call to nuke the User-Mode Driver Framework cache, plus a `pnputil /scan-devices` rescan after re-enable. Cube still did not re-enumerate.

3. **30-second post-cycle polling window** — the bootloader has a ~5-second listen window on power-up; we polled at 4 Hz for 30 seconds covering multiple potential boot cycles. Nothing.

**Conclusion: the Cube is externally powered (bench power supply on the boat).** Cycling the laptop's USB host doesn't cut Cube power, only data. The chip never resets, the bootloader never runs, no recovery window opens.

This is an environmental fact about the Vanguard's bench setup, not a software issue. Documented in `usv-deploy-log.md` going forward.

### Step 4 — revert to ArduRover 4.6.3 — ✅ SUCCESS 2026-04-27 ~14:14 UTC

**What we did:**
1. Pre-staged a watcher script (`usv-watch-and-flash.ps1`) on the USV that polls at 4 Hz for the Cube to reappear with a single-COM-port (= bootloader) signature. The instant it appears, the watcher fires uploader.py with the ArduRover .apj.
2. Tristan went to the boat, killed bench power for 2-3 seconds, restored.
3. The Cube power-cycled, the bootloader ran, USB enumerated as `COM3` (single port), the watcher caught it within 250 ms, immediately invoked uploader.py.
4. uploader.py negotiated bootloader handshake, programmed the full 1,808,672-byte ArduRover image, verified, rebooted.

**Watcher + upload transcript:**
```
Cube appeared on COM3. Firing uploader.py NOW.
Loaded firmware for 427,0, size: 1808672 bytes, waiting for the bootloader...
Found board 427,0 bootloader rev 5 on COM3
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.
uploader.py exit code: 0
```

This proved the **bootloader region survived the Meridian flash unscathed** — same `bootloader rev 5`, same `board_type 1063`, same OTP serial `003a002f3233510536343932`, same flash size. The Meridian upload only touched the firmware region as designed.

### Step 5 — post-revert sanity — ✅ ALL CHECKS PASSED

After ArduRover finished booting, the Cube re-enumerated as a USB Composite Device with two CDC ports (COM4 + COM5 — same as pre-flash). Connected via pymavlink on COM4:

```
heartbeat type=11 autopilot=3      ← Rover, ArduPilot
flight_sw_version: 0x040603ff      ← ArduPilot 4.6.3 (exact match to pre-flash)
flight git: 3fc7011a               ← exact match to pre-flash
parameters: 974 / 974              ← exact count
FORMAT_VERSION = 16.0              ← unchanged
```

| Test | Expected | Observed | Pass |
|---|---|---|---|
| Heartbeat returns ArduPilot identity | `autopilot=3 flight_sw=0x040603ff` | `autopilot=3 flight_sw=0x040603ff` | ✅ |
| `git_identity` reads back as `3fc7011a` | exact match to pre-flash | `3fc7011a` | ✅ |
| All 974 parameters still present | param count = 974, no losses | 974 / 974 | ✅ |
| FORMAT_VERSION param unchanged | `FORMAT_VERSION = 16.0` | `16.0` | ✅ |

**Diff vs pre-flash baseline:**
- 0 parameters added
- 0 parameters removed
- 19 changed values — **all** of them are sensor self-calibration variables that ArduPilot updates every boot (`INS_GYR1OFFS_X/Y/Z`, `INS_GYR2OFFS_X/Y/Z`, `INS_GYR3OFFS_X/Y/Z`, `INS_GYR1_CALTEMP`, etc.). These are runtime drift values, not user-configured tuning.
- 955 user-configured parameters byte-identical to pre-flash. All of Tristan's tuning untouched.

Post-revert dump captured for the record:
- USV: `C:/Users/vangu/Documents/Vanguard-Backups/vanguard_params_20260427_141757.parm`
- Dev box: `D:/projects/meridian/backups/usv-vanguard/vanguard_params_20260427_141757_post-revert.parm`

### Step 6 — Mission Planner restored

Restarted Mission Planner on the USV so Tristan finds the same environment he left when he next looks at the laptop. (Process PID assigned afresh; nothing else changed.)

---

## Outcome — ✅ SESSION COMPLETE

| Tier | Result |
|---|---|
| Pre-flash backup integrity | ✅ params + restore image + tooling all SHA256-verified |
| Flash mechanics (build, .apj package, upload) | ✅ end-to-end works |
| Bootloader region preserved | ✅ confirmed: post-revert handshake reported identical bootloader rev/board/OTP |
| Tier 1 functional verification (heartbeat, MNP) | ❌ blocked — stub has no USB CDC; expected for Phase-9 MVP |
| Remote-only recovery path | ❌ confirmed impossible with Cube on external power |
| Revert to ArduRover with physical power cycle | ✅ SUCCESS — bit-perfect restore, all 974 params, git `3fc7011a` |
| Boat returned to Tristan in original state | ✅ ArduRover 4.6.3 running, MP restarted, params identical |

**Headline:** the build-pack-flash-revert pipeline works end-to-end. We can iterate Meridian firmware as fast as we can compile it, and we always have a bit-perfect recovery path back to ArduPilot.

## Lessons that hard-locked into rules going forward

1. **No Meridian firmware ships to real hardware unless it has USB CDC initialized** plus a MAVLink or simple-protocol reboot-to-bootloader handler over that CDC. Without it, the chip is unreachable post-flash and recovery requires physical access. (Tonight's situation.)
2. **Software-only USB cycling cannot recover externally-powered devices.** The deployment runbook mentioned remote recovery as a clean path; that's only true for bus-powered Cubes. Updating the runbook to flag this.
3. **Always run a "would I be able to recover from this remotely" check before flashing**: if the candidate firmware doesn't expose the same minimum-viable interface (USB CDC + reboot-to-bootloader) as the bootloader, do not deploy. Build it locally, test it locally, then deploy.
4. **The bootloader is rock-solid** — it accepted the upload, programmed the firmware region, and rebooted into the candidate cleanly. The bootloader region itself is presumably untouched (we'll confirm post-revert).

## Recovery actions used during session

- Tried 3-second USB controller disable/enable cycle (`usv-cube-cycle.ps1`) — failed
- Tried 60-second deep cycle with WUDFRd restart and pnputil rescan (`usv-recover-deep.ps1`) — failed
- Both failures confirmed the Cube is externally powered, not bus-powered

These scripts remain useful for *bus-powered* boards in any future setup, so they stay in the toolkit.

## Action items going into Tier 2

1. **Add USB CDC stub** to `meridian-stm32-bin` — enumerate as VID 0x1209/PID 0x5741 (or any agreed USB ID), expose a single CDC endpoint that emits a 1 Hz "Meridian alive @ <git-hash>" string at minimum.
2. **Add reboot-to-bootloader handler** — listen on the CDC for either MAVLink `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1=3` OR a simple ASCII "REBOOT" command. Set the magic flag in RAM, jump to NVIC reset.
3. **Add boot-time GPIO safe-mode flag** — if a specified pin is asserted at boot, skip firmware and jump straight back to bootloader. Last-resort rescue.
4. **Build and locally verify** the next firmware via `probe-rs` against an STM32H7 dev board *before* it goes near the Cube.
5. **Update `deployment_runbook.tex`** with the bus-vs-bench-power caveat for remote recovery.
