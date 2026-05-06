# Vanguard USV — Deploy Log

Running state document for the on-water deployment of Meridian onto the Vanguard USV. Updated each working session.

## Status at a glance

| Milestone | State | Evidence |
|---|---|---|
| Remote SSH access to USV proven | ✅ working | paramiko via Tailscale, port 22, user `vangu` |
| Pre-flash backups captured | ✅ done | params + firmware identity + MP local data, see [2026-04-26 session](flash-sessions/2026-04-26_first-meridian-flash.md) |
| Build prep (.apj produced) | ✅ done | `backups/usv-vanguard/firmware/meridian-cubeorangeplus.apj`, board_id 1063 |
| Restore image staged | ✅ done | `ardurover-4.6.3-CubeOrangePlus.apj` (bit-exact match to live Cube) |
| Tier 1 flash mechanics (build → .apj → upload → verify) | ✅ proven | bootloader erase/program/verify all 100% on 2026-04-26 |
| Tier 1 functional verification | ❌ blocked by stub | Phase-9 firmware has no USB CDC; chip silent post-flash |
| Revert to ArduRover (physical power cycle required) | ✅ DONE 2026-04-27 | bit-perfect restore: 974 params, git `3fc7011a`, all user tuning intact |
| Round-trip flash/revert pipeline proven end-to-end | ✅ | iterate as fast as we compile, always recoverable |
| Tier 1.5 — minimum-viable Meridian (USB CDC + reboot handler + safe-mode GPIO) | ⏳ next firmware iteration | hard-locked rule, see "Ground rules" below |
| Tier 2 — sensor + actuator bring-up | ⏳ pending Tier 1.5 | requires meaningful firmware |
| Tier 3 — dock test, live Orca data | ⏳ pending Tristan + Tier 2 | |
| Tier 4 — sheltered-water mission | ⏳ pending Tier 3 + chase boat | |
| Tier 5 — SOCOM demo | ⏳ pending Tier 4 + scripted scenario | |

## Ground rules — locked in 2026-04-26 after the first flash

1. **No Meridian firmware ships to real hardware unless it has all three of:**
   - USB CDC enumeration (chip stays observable from host)
   - Reboot-to-bootloader handler over CDC (remote recovery path always available)
   - Boot-time GPIO safe-mode flag (last-resort rescue if firmware bricks)
2. **Software-only USB cycling cannot recover externally-powered Cubes.** The Vanguard's bench setup powers the Cube from boat power, not USB. Recovery in those conditions requires physical bench-power cycling. The deployment runbook has been updated to flag this distinction.
3. **Build → local-test → deploy.** Any new firmware target gets validated on a dev board with `probe-rs` before the .apj is staged on a real Cube.

## What's deployed on the USV right now

| Asset | Path | SHA256 |
|---|---|---|
| Meridian candidate firmware | `C:/Users/vangu/Documents/Vanguard-Backups/flash/meridian-cubeorangeplus.apj` | `aa06264ec8bbc0dc…` |
| ArduRover 4.6.3 restore image | `…/flash/ardurover-4.6.3-CubeOrangePlus.apj` | `b9a912d98b313768…` |
| ArduPilot uploader.py | `…/flash/uploader.py` | `72bfbde5e211f265…` |
| USB cycle helper | `…/flash/usv-cube-cycle.ps1` | `d0a8f76e73b895b2…` |
| Manifest of the above | `…/flash/MANIFEST.json` | n/a |

## What's deployed on the dev box (this machine)

| Asset | Path |
|---|---|
| Param backup (parm + json) | `D:/projects/meridian/backups/usv-vanguard/vanguard_params_20260427_042022.*` |
| Same Meridian + ArduRover .apj files | `D:/projects/meridian/backups/usv-vanguard/firmware/` |
| SSH helper for the USV | `C:/Users/jesse/bin/usv-ssh.py` |
| Stage-flash-kit script | `C:/Users/jesse/bin/usv-stage-flash-kit.py` |

## Hardware on the boat (for the curious reader)

- **Lenovo 83KX laptop** (Win 11 Home 25H2, 16 GB) — the onboard companion
- **CubePilot Cube Orange Plus** — STM32H757 dual-core, exposes COM4 (MAVLink) + COM5 over USB (VID 2DAE / PID 1058)
- **Orca Core 2** marine gateway — 10 Hz GPS, 9-axis IMU, NMEA2000-to-Ethernet bridge
- **RFD 900** telemetry radio
- **Webcam** pointed at the rudder for bench-test visual confirmation

## Sessions

- [2026-04-26 — first Meridian flash + revert proof](flash-sessions/2026-04-26_first-meridian-flash.md)

## Next sessions to schedule

1. **Tier 2 firmware iteration.** Advance the STM32 binary past Phase-9 stub — sensor probe, MAVLink emission, servo output. No new flash kit work needed; the upload pipeline is proven after Tier 1.
2. **Orca bridge live integration.** Once Meridian is producing real heartbeats, wire the Orca bridge daemon to feed GPS_INPUT.
3. **Dock test with Tristan present.** Boat in water, both filters live, supervisor swap demonstrated, GPS-jam cascade visible on tablet.
