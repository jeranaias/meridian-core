# STATUS — Vanguard MVP live test readiness

> **🟡 STATUS: ALMOST READY — TRISTAN, ACTION REQUIRED.**
>
> Telemetry pipe + outbound command path are both verified end-to-end.
> Tablet GCS is PWA-wrapped and connected to the live boat.
> **Blocking the tank test: the ESC + steering servo aren't mapped to
> ArduRover output channels yet.** Once that's done, we're green.
>
> If you (Tristan / Tristan's Claude Code) are reading this — see
> [§ 4. ACTION REQUIRED FROM TRISTAN](#4-action-required-from-tristan).

**Last update:** 2026-05-07 morning (USV/Sydney time)

---

## 1. What's verified working

| Layer | State | Evidence |
|---|---|---|
| Boat firmware | ArduRover 4.6.3 alive on Cube Orange Plus | COM4 (MI_00) + COM5 (MI_02) enumerated as VID_2DAE&PID_1058 |
| MAVLink WebSocket bridge | **Running as `MeridianMAVLinkBridge` Scheduled Task** on USV laptop, survives detach/reboot | Bridge log shows ~50 msg/s ingest from COM4; port 5760 LISTENING on 0.0.0.0 |
| Tailscale routing | Working both directions | `100.72.16.72:5760` reachable from anywhere on the tailnet |
| Live MAVLink decode | Telemetry pipe IN works | Decoded HEARTBEAT, RAW_IMU, AHRS, AHRS2, SERVO_OUTPUT_RAW frames |
| Outbound commands | Command pipe OUT works | Sent `MAV_CMD_REQUEST_AUTOPILOT_VERSION` (520), got `COMMAND_ACK result=0 (ACCEPTED)` |
| ARM / DISARM in `gcs/mission.html` | Wired correctly | `MAV_CMD_COMPONENT_ARM_DISARM` (cmd 400), force-magic 21196 on disarm for E-STOP behavior |
| Emergency Stop button | Wired correctly | `btn-stop` → force-disarm → motors cut, boat coasts |
| ArduRover mode mapping | **Fixed in this session** | Was copter-mapped (RTL → FOLLOW); now rover-mapped (RTL → AP mode 11) |
| Tablet GCS PWA wrapper | manifest.json + sw.js + cyber aesthetic | `gcs/manifest.json`, `gcs/sw.js`, brand hex + Orbitron + scanline overlay landed |
| Auto-stream-request on connect | Added | mission.html now sends `REQUEST_DATA_STREAM(ALL, 10Hz)` on `ws.onopen` so GPS/position/VFR_HUD start flowing |

## 2. What's NOT verified / is broken

- **ESC and steering servo are NOT mapped to ArduRover output channels.**
  This is the actual tank-test blocker — see § 4.
- **GPS fix:** boat is on bench/indoors → fix_type 0 (NoGPS), lat/lon=0.
  Telemetry pipe is fine; GPS hardware just can't lock without sky view.
- **Ground steering vs differential:** Vanguard hull is jet-drive single
  nozzle. Need to confirm `FRAME_CLASS=1` (rover) and pick differential
  vs Ackermann/skid in `FRAME_TYPE`.
- **Pre-arm checks:** ArduRover by default refuses to arm without GPS
  fix + healthy compass + sane params. For tank test we may want
  `ARMING_CHECK = 0` to bypass — but only after ESC mapping is verified
  in MANUAL mode at the bench.
- **Tablet PWA install** requires HTTPS — current `http://...:8765/`
  loads the GCS in a regular browser fine but Service Worker won't
  register, so no "Add to Home Screen" with offline cache. Acceptable
  for the tank test; revisit later for field use.

## 3. One-line bring-up (any browser on Tailscale)

```
http://100.113.149.38:8765/mission.html
   ↳ tap ⚙ in status bar
   ↳ Protocol: MAVLink
   ↳ WebSocket URL: ws://100.72.16.72:5760
   ↳ Connect

Status bar will fill within 1 second of connect. GPS will read
"NoFix" until the boat sees sky.
```

---

## 4. ACTION REQUIRED FROM TRISTAN

**To unblock the tank test, we need three pieces of info from you, plus
maybe one parameter snapshot.** Once we have these, we'll set the
right `SERVO<N>_FUNCTION` parameters via MAVLink, bench-test that ARM
+ small throttle stick spins the ESC (with prop OUT of water!), and
flip this STATUS.md to GREEN.

### 4.1 Tell us how the ESC and steering servo are wired

Reply with the answer to these three questions:

1. **Which physical output pin is the ESC signal on?**
   On the Cube Orange Plus carrier — is it MAIN1 / MAIN2 / MAIN3 / AUX1 /
   AUX2 / AUX3 / AUX4? If the carrier numbers them differently, just say
   "the leftmost AUX" or take a photo.

2. **Which physical output pin drives the steering servo (jet nozzle)?**
   Same as above.

3. **What ESC type?**
   - Standard PWM (1000–2000 µs analog servo signal) — typical hobby ESC
   - DShot / OneShot — newer digital protocols
   - BLHeli-32?

### 4.2 If you have them handy, also send

- A current ArduRover parameter dump from the boat — `Mission Planner →
  Config → Full Parameter List → Save to file`. That tells us what
  current SERVO functions/limits/PWM bounds are set to.
- Any photos of the wiring or your current Mission Planner SERVO_OUTPUT
  config.

### 4.3 What we'll do once we have those answers

1. Send `PARAM_SET` for the right `SERVO<N>_FUNCTION` (73 = Throttle,
   26 = GroundSteering) and `SERVO<N>_MIN/MAX/TRIM`.
2. Set `FRAME_CLASS = 1` (rover) and pick `FRAME_TYPE` for Vanguard
   single-nozzle jet boat.
3. Set `ARMING_CHECK` to a sane subset for bench testing.
4. Have you do a bench test: prop OUT, ARM, small forward throttle on
   stick → ESC spins. Reverse stick → ESC reverses (if your ESC has
   bidirectional). Steering stick → nozzle moves.
5. If that works at the bench, we go to the tank.

---

## 5. Live-test checklist (water tank, post-ESC-mapping)

**Pre-tank, at the bench (~10 min):**
- [ ] Tablet on Tailscale, mission.html loads, connects, telemetry strip lights up
- [ ] Mode badge shows `DISARMED · MANUAL`
- [ ] ARM works: red button → confirm → mode badge switches to `ARMED`
- [ ] DISARM works
- [ ] **ESC bench check (PROP OUT OF WATER):** in MANUAL, RC throttle stick or
      software throttle drives the ESC PWM up; ESC spins; stick to neutral → ESC stops
- [ ] **Steering bench check:** RC steering stick or software steering moves the nozzle servo through full travel both directions
- [ ] EMERGENCY STOP button cuts motors instantly
- [ ] Mode switch buttons work: MANUAL → HOLD → MANUAL (verified by reading back HEARTBEAT mode)
- [ ] Mission upload of one waypoint queues without ack-failure (RTL behavior in HOLD without GPS = "stay disarmed", that's expected on bench)

**In the tank, with Tristan as safety pilot:**
- [ ] Manual stick test — boat responds to throttle + steering inputs
- [ ] HOLD mode — boat sits in place
- [ ] DISARM under power → motors immediately cut
- [ ] If GPS is available (tank near a window?), brief LOITER test
- [ ] Single-waypoint AUTO transit (only if GPS works and tank is large enough)

---

## 6. Recovery options if anything goes sideways

1. **GCS DISARM** (red E-STOP button) — kills thrust, boat coasts
2. **Tablet/Bridge restart** — restart `mavlink-ws-bridge.py` Scheduled Task on USV; reconnect
3. **MAVLink reboot to bootloader** — `python C:\Users\vangu\usv-mav-reboot-to-bl.py` from USV
4. **MeridianAggressiveWatcher Scheduled Task** can be re-armed with ArduRover restore APJ for auto-recovery
5. **Power cycle** — last resort, requires hand on BEC

---

## 7. Repo map (quick reference)

- `README.md` — project overview, branching, quick start
- `STATUS.md` — **this file.** Live-test readiness + Tristan action items.
- `docs/INDEX.md` — navigable map for the 71-file `docs/` tree
- `gcs/mission.html` — **the tablet GCS** (single file, 2845+ lines, PWA-wrapped)
- `gcs/test-console.html` — engineer test console (dense button grid)
- `gcs/index.html` — full multi-panel GCS (Fly/Plan/Setup/Params/Logs/Status)
- `apps/vanguard-pwa/` — Tristan's PWA (visual reference / demo theater)
- `tools/mavlink-ws-bridge.py` — what's running on the USV right now
- `tools/tristan-gcs-bridge.py` — one-command USV bring-up (kills watcher + starts bridge)

---

## 8. For Claude Code reading this repo

When the user says "check repo for updates":

1. `git pull` from the `vanguard` remote (NOT `origin` — that's public OSS Meridian)
2. Read `STATUS.md` (this file). Top banner tells you readiness state.
3. If banner is 🟡 → there's an unblocker action. See §4.
4. If banner is 🟢 → run through the checklist in §5.
5. Never push to `origin`. Only push to `vanguard`.
