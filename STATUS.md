# STATUS — Vanguard MVP live test readiness

> **🟡 STATUS: ALMOST READY — one configuration step remaining.**
>
> Telemetry pipe + outbound command path are both verified end-to-end on
> the live boat. Tablet GCS is PWA-wrapped, connected, and getting real
> MAVLink ACKs from ArduRover. **The one remaining gap is propulsion
> mapping** — ArduRover doesn't yet know which output channels drive the
> ESC and steering servo. Once that's set we're 🟢.
>
> **For Tristan's Claude Code:** the next section tells you exactly
> what to run. The wizard does the work; the operator only confirms
> physical safety (prop out of water) and watches what moves.

**Last update:** 2026-05-07 morning (USV/Sydney time)

---

## 1. Run this first — the setup wizard does the rest

The propulsion config is driven by `tools/ardurover_setup_wizard.py`.
It reads ArduRover's current parameter state, runs safe wiggle tests,
and writes the right `SERVO_n_FUNCTION` values via live `PARAM_SET` —
no firmware flash needed.

### 1.1 See what ArduRover currently thinks is connected where

```bash
python tools/ardurover_setup_wizard.py status --list-timeout 90
```

Output (JSON) shows current `SERVOn_FUNCTION` (non-zero only),
`FRAME_CLASS`, `ARMING_CHECK`, live armed/mode state, and an
**analysis** field that tells you whether throttle (FN=70 — single
ESC) and steering (FN=26 — GroundSteering) are mapped or unmapped.

**Note:** ArduPilot's `PARAM_REQUEST_LIST` dump is slow — give it the
full 90 sec. If after 90 sec the SERVO list still looks incomplete,
fall back to:
- Checking Mission Planner's `SERVO_OUTPUT` page on Tristan's side
  (fastest authoritative view), OR
- Running the wiggle test in § 1.2 to directly identify which physical
  output pin drives what.

If status already shows throttle (FN=70) and steering (FN=26) on
sensible channels, jump to § 1.4.

### 1.2 If you don't yet know which physical pin is the ESC / which is the steering servo

Run a wiggle test channel-by-channel. The wizard sends a brief PWM
pulse on one channel via `MAV_CMD_DO_SET_SERVO`, returns it to neutral,
and waits for the operator to report what they observed.

**Safety:** `--confirm-prop-out` is required — without it the wizard
refuses to pulse anything.

```bash
# For each candidate channel — sweep through 1..14, ~1 sec per check:
for ch in 1 2 3 5 6 9 10 11 12 13 14; do
    echo "==== CH${ch} (operator: watch the boat) ===="
    python tools/ardurover_setup_wizard.py probe \
        --channel $ch --pwm 1550 --duration 1 --confirm-prop-out
    sleep 2
done
```

Operator narrates: "CH3 made the ESC beep" → that's the throttle
channel. "CH5 nozzle deflected port" → that's the steering channel.

(Cube Orange Plus mapping: ArduRover output channels 1–8 are MAIN1–8
on the carrier's upper rail; channels 9–14 are AUX1–6 on the lower
rail. So `--channel 1` = MAIN1, `--channel 9` = AUX1.)

### 1.3 Once you know the channels — write the config

```bash
python tools/ardurover_setup_wizard.py configure \
    --throttle-ch <N> --steering-ch <M> \
    --esc-type pwm \
    --set-pwm-bounds
```

This sets:
- `SERVO<N>_FUNCTION = 73` (Throttle)
- `SERVO<M>_FUNCTION = 26` (GroundSteering)
- `SERVO<N>_MIN/TRIM/MAX = 1100/1500/1900`
- `SERVO<M>_MIN/TRIM/MAX = 1100/1500/1900`
- `MOT_PWM_TYPE = 0` for standard PWM (or `6` for DShot300 if `--esc-type dshot`)

`--set-pwm-bounds` is optional but recommended for first-time setup.
Skip if Tristan has already tuned PWM endpoints.

### 1.4 Loosen pre-arm checks for bench testing (optional but usually needed)

ArduRover refuses to arm without GPS fix + healthy compass + RC + lots
of other checks. For bench testing without the boat in water:

```bash
python tools/ardurover_setup_wizard.py arming-bypass --confirm-bench-only
```

Disables `ARMING_CHECK` and `BRD_SAFETYENABLE`. The wizard prints the
pre-change values so they can be restored. **Use this only at the
bench.** Restore stock arming for any actual deployment.

### 1.5 Verify the full propulsion path works

```bash
python tools/ardurover_setup_wizard.py verify --confirm-prop-out
```

Wizard:
1. Reads back the throttle + steering channels from current params
2. Sets MANUAL mode
3. ARMs (force-magic 21196)
4. Pulses throttle channel @ 1550 µs for 1.5 seconds
5. Sweeps steering channel: 1300 → 1700 → 1500
6. DISARMs

Operator watches: throttle pulse → ESC spins. Steering sweep → nozzle
moves through full travel. If both happen, the propulsion pipe is
verified end-to-end.

### 1.6 Flip the banner to 🟢

After § 1.5 passes, edit the banner at the top of this file and
`README.md` to GREEN, commit, push.

---

## 2. What's already verified working (no action needed)

| Layer | State | Evidence |
|---|---|---|
| Boat firmware | ArduRover 4.6.3 alive on Cube Orange Plus | COM4 (MI_00) + COM5 (MI_02) enumerated as VID_2DAE&PID_1058 |
| MAVLink WebSocket bridge | `MeridianMAVLinkBridge` Scheduled Task on USV laptop, survives detach/reboot | Bridge log: ~50 msg/s ingest from COM4; port 5760 LISTENING on 0.0.0.0 |
| Tailscale routing | `100.72.16.72:5760` reachable from anywhere on tailnet | Confirmed bidirectional |
| Live MAVLink decode | Decoded HEARTBEAT, RAW_IMU, AHRS, AHRS2, SERVO_OUTPUT_RAW frames | — |
| Outbound commands | Sent `MAV_CMD_REQUEST_AUTOPILOT_VERSION` (520), got `COMMAND_ACK result=0` | — |
| ARM / DISARM in `gcs/mission.html` | `MAV_CMD_COMPONENT_ARM_DISARM` (cmd 400), force-magic 21196 on disarm | — |
| Emergency Stop button | `btn-stop` → force-disarm → motors cut, boat coasts | — |
| ArduRover mode mapping in mission.html | Fixed in this session | RTL → AP 11, AUTO → AP 10, HOLD → AP 4 |
| Auto-stream-request on connect | mission.html `ws.onopen` sends `REQUEST_DATA_STREAM(ALL, 10Hz)` | — |
| Tablet GCS PWA wrapper + cyber aesthetic | manifest, sw.js, Orbitron, scanlines, brand hex | — |

---

## 3. One-line bring-up (any browser on Tailscale)

```
http://100.113.149.38:8765/mission.html
   ↳ tap ⚙ in status bar
   ↳ Protocol: MAVLink
   ↳ WebSocket URL: ws://100.72.16.72:5760
   ↳ Connect

Status bar fills within 1 second. GPS will read "NoFix" indoors —
expected, the telemetry pipe is fine.
```

---

## 4. Live-test checklist

**Bench (post-config-wizard, ~10 min):**

- [ ] § 1.1 status — wizard reports SERVO_FUNCTION values, no errors
- [ ] § 1.5 verify — ESC spins on throttle pulse, nozzle moves on steering sweep
- [ ] mission.html ARM button → mode badge switches to `ARMED`
- [ ] mission.html DISARM works
- [ ] mission.html EMERGENCY STOP button cuts motors instantly
- [ ] mission.html mode buttons land correct ArduRover mode (HEARTBEAT readback proves it)

**Tank, with Tristan as safety pilot:**

- [ ] Manual stick → boat moves. Steering response is correct direction.
- [ ] HOLD mode → boat sits. (Without GPS this may not station-keep, but motors should stop.)
- [ ] EMERGENCY STOP under power → motors cut immediately
- [ ] If GPS fix is available (open-air tank): brief LOITER test
- [ ] Single-waypoint AUTO transit (only if GPS works)

---

## 5. Recovery options if anything goes sideways

1. **mission.html EMERGENCY STOP** — kills thrust, boat coasts
2. **Bridge restart**:
   ```bash
   powershell -ExecutionPolicy Bypass -File C:\Users\vangu\bridge-as-task.ps1
   ```
3. **MAVLink reboot**:
   ```bash
   python C:\Users\vangu\usv-mav-reboot-to-bl.py
   ```
4. **Auto-flash watcher** can be re-armed with ArduRover restore APJ:
   ```bash
   powershell -ExecutionPolicy Bypass -File C:\Users\vangu\launch-ardurover-restore.ps1
   ```
5. **Power cycle** — last resort

---

## 6. Repo map (quick reference)

- `README.md` — project overview, branching, quick start
- `STATUS.md` — **this file.** Live-test readiness + the wizard.
- `tools/ardurover_setup_wizard.py` — **the wizard.** Subcommands:
  `status`, `probe`, `configure`, `arming-bypass`, `verify`, `restore-param`
- `gcs/mission.html` — the tablet GCS (single file, 2845+ lines, PWA-wrapped)
- `gcs/test-console.html` — engineer test console
- `gcs/index.html` — full multi-panel GCS (Fly/Plan/Setup/Params/Logs/Status)
- `apps/vanguard-pwa/` — Tristan's PWA (visual reference / demo theater)
- `tools/mavlink-ws-bridge.py` — running on the USV right now
- `tools/tristan-gcs-bridge.py` — one-command USV bring-up
- `docs/INDEX.md` — navigable map for the 71-file `docs/` tree

---

## 7. For Claude Code reading this repo

When the operator says "check repo for updates":

1. `git pull` from the `vanguard` remote (NOT `origin` — that's public OSS Meridian)
2. Read this file. The banner tells you state.
3. **🟡 banner** → there's a single clear unblock. Run § 1 in order. Each
   step has a concrete shell command and a clear human-confirmable
   safety prompt. Pass results back to the operator and ask the next
   question only when the wizard's output asks for it.
4. **🟢 banner** → run the § 4 checklist.
5. Never push to `origin`. Only push to `vanguard`.

When the operator says "we are ready to test" or similar before the
banner is 🟢, point them at § 1 and offer to run the wizard.
