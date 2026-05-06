# docs/ — what to read and what to skim

This directory has 71 markdown files. Here's the navigable map.

## Read first

These are the docs your future-self (or your Claude Code) will reach
for repeatedly:

| File | What it is |
|---|---|
| [`vanguard/00_deployment_plan.md`](vanguard/00_deployment_plan.md) | Top-level Vanguard field plan |
| [`vanguard/01_hardware_identification.md`](vanguard/01_hardware_identification.md) | Cube Orange Plus pinout + power rails + USB topology |
| [`vanguard/DEPLOYMENT_RUNBOOK.md`](vanguard/DEPLOYMENT_RUNBOOK.md) | § 0–10: pre-flight, flash, params, calibration, test, recovery |
| [`flash-sessions/meridian-firmware-version-history.md`](flash-sessions/meridian-firmware-version-history.md) | v0.1–v1.4 iteration log + safety-net forensics. **Read end-to-end before touching firmware.** |
| [`LAKE_TEST_RUNBOOK.md`](LAKE_TEST_RUNBOOK.md) | Step-by-step on-water shakedown |
| [`usv-deploy-log.md`](usv-deploy-log.md) | Running diary of bench/water sessions |

## Read when you need it (deployment / hardware)

| File | Topic |
|---|---|
| `vanguard/02_architecture_plan.md` | High-level Meridian + GCS + bathy architecture |
| `vanguard/03_zero_external_gps_architecture.md` | Single-GPS plan via Orca Core 2 |
| `vanguard/04_competitive_analysis.md` | Vanguard vs alternatives |
| `vanguard/05_field_kit_checklist.md` | What goes in the field kit |
| `vanguard/brief/` | LaTeX briefing decks (v1–v6 PDF + tex source) |
| `flash-sessions/2026-04-26_first-meridian-flash.md` | First Meridian flash on real hardware |
| `flash-sessions/v0.7-prep-and-safety-review.md` | v0.7 prep notes + early safety review |

## Read when you need it (design context / panel reviews)

20 expert panels reviewed Meridian's architecture. They live at
`panel_NN_<expert>.md`. **Don't read all 20 unless you have an afternoon
to spare.** When a specific topic comes up, hit the relevant one:

| Expert | Specialty | When to read |
|---|---|---|
| `panel_01_tridgell.md` | ArduPilot lead | Bus / link layer / overall arch |
| `panel_02_riseborough.md` | EKF | EKF design, fusion, variance handling |
| `panel_03_hall.md` | RC / radio | RC input architecture |
| `panel_04_mackay.md` | Multirotor stabilization | Attitude control, stabilization |
| `panel_05_barker.md` | Plane modes | Mode switching, plane logic |
| `panel_06_wurzburg.md` | Drivers | Driver architecture, IMU/baro |
| `panel_07_premerlani.md` | Estimation | Sensor fusion / DCM |
| `panel_08_aparicio.md` | Real-time | RTIC, TIM2 clash, scheduling |
| `panel_09_munns.md` | Mission | Mission planner, 55 commands |
| `panel_10_white.md` | Comms | MAVLink / serial / link reliability |
| `panel_11_koopman.md` | Safety | Failsafe / arming / safety architecture |
| `panel_12_leveson.md` | Software safety | STAMP, reasoning about failure modes |
| `panel_13_dandrea.md` | Control theory | PID / MPC / attitude |
| `panel_14_kumar.md` | Multirotors | Cascade controller architecture |
| `panel_15_crenshaw.md` | Aerospace heritage | Industrial design heritage |
| `panel_16_schlosser.md` | Boats | Surface vessel-specific |
| `panel_17_meier.md` | QGC | GCS architecture (round 1) |
| `panel_18_winer.md` | Embedded | Embedded constraints, no_std |
| `panel_19_jones.md` | Ops / field | Operations, deployment ergonomics |
| `panel_20_doll.md` | Hardware | Cube hardware architecture |

GCS-specific reviews:
- `panel_gcs_review.md` (round 1)
- `panel_gcs_review_v2.md`
- `panel_gcs_round2_review.md` (Oborne / Meier / Tufte / Krug / Victor / Ive / Zhuo — actionable polish list, our current source for what to clean up)
- `panel_gcs_flyview_review.md`
- `panel_round2_wave1.md` … `panel_round2_wave5.md` (multi-pass refinement)

`MASTER_GCS_REVIEW.md` (62KB) — comprehensive synthesis of GCS critiques.

## Read when you need it (parity audits)

These are line-by-line ArduPilot-vs-Meridian audits. They confirm we
match capability. **Skim only when you're worried something's missing
or the audit-of-record is what you need.**

- `FULL_PARITY_AUDIT.md`, `PARITY_GAP_MASTER.md`
- `parity_control.md`, `parity_drivers_protocols.md`, `parity_ekf.md`,
  `parity_motors.md`, `parity_nav_modes.md`, `parity_payload_osd.md`
- `audit_*.md` (14 audits, organized by subsystem)
- `final_review_control_motors.md`, `final_review_drivers_protocols.md`,
  `final_review_ekf.md`, `final_review_modes_nav_safety.md`

## Skim only / archived

- `identification_*.md` — early subsystem ID notes (5 files)
- `PANEL_WAVE1_SUMMARY.md`, `PANEL_MASTER_TODO.md` — meta-summaries
  (the panels themselves are more useful)
- `research_mission_planner.md`, `research_qgroundcontrol.md` — early
  competitor research notes
- `sensor_driver_audit.md`, `sensor_driver_status.md` — sensor parity
  status (subsumed by `audit_*` files)
- `recursive_semaphore_research.md` — niche RTOS research

## Conventions in this tree

- `panel_NN_<lastname>.md` = expert panel review, round 1
- `audit_<topic>.md` = ArduPilot parity audit
- `final_review_<topic>.md` = deep dive synthesis
- `parity_<topic>.md` = gap-only summary of an audit
- `vanguard/` = Vanguard-USV-specific docs
- `flash-sessions/` = firmware iteration logs
