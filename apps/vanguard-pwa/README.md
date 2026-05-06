# VANGUARD · Mesh Relay Mission System

> Autonomous USV mesh radio relay — GPS-denied capable, ship-side deployable, tablet-controlled.

![Status](https://img.shields.io/badge/status-pre--deploy-orange) ![Platform](https://img.shields.io/badge/platform-PWA-teal) ![Autopilot](https://img.shields.io/badge/autopilot-Meridian%20v1.1-blue)

---

## What this is

A mission control UI and PWA for a fleet of small (1–2 m) Unmanned Surface Vehicles carrying **Silvus MANET radios**. Each USV launches from a ship's side, autonomously navigates to a GPS waypoint, station-keeps, and forms a mesh relay node — extending communications range by 5–10 NM.

When GPS is jammed, the system falls back to **depth-chart position matching** (Meridian autopilot) maintaining station to within 25 m — inside the 50 m threshold for continued mission effectiveness.

---

## Live demo

Open `index.html` in any modern browser, or install as a PWA:

```bash
cd vanguard_pwa
python3 -m http.server 8080
# open http://localhost:8080 on tablet → Add to Home Screen
```

Hit **▶ DEMO** to run the full scripted sequence automatically.

---

## Repo structure

```
vanguard_pwa/
├── index.html          # Full mission UI (single file, no dependencies)
├── manifest.json       # PWA manifest — fullscreen landscape, icons
├── sw.js               # Service worker — offline caching, push alerts
└── icons/              # App icons 72px → 512px
    ├── icon-72.png
    ├── icon-96.png
    ├── icon-128.png
    ├── icon-144.png
    ├── icon-152.png
    ├── icon-192.png
    ├── icon-384.png
    └── icon-512.png

docs/
├── Vanguard_USV_Scope_v2.docx    # Scope of work — Virtual Lab & Mission UI
└── deployment_runbook.pdf         # Meridian deployment runbook (Vanguard + Cube Plus)
```

---

## Features

| Feature | Status |
|---|---|
| Pre-launch checklist | ✅ Complete |
| GPS waypoint mission setup | ✅ Complete |
| Live tactical map — pan, zoom, drag waypoints | ✅ Complete |
| Depth chart underlay — channel, sandbank, contours | ✅ Complete |
| Multi-USV fleet management | ✅ Complete |
| GPS jam simulation + depth chart fallback | ✅ Complete |
| AIS landmark fusion display | ✅ Complete |
| Hold timer — live countdown | ✅ Complete |
| RTL — full return and recovery sequence | ✅ Complete |
| Emergency stop | ✅ Complete |
| Post-mission debrief screen | ✅ Complete |
| Demo mode — scripted auto-run | ✅ Complete |
| PWA — installs on tablet, works offline | ✅ Complete |
| Wake lock — screen stays on during mission | ✅ Complete |
| Meridian bridge WebSocket integration | 🔧 Next milestone |
| Live telemetry from USV | 🔧 Next milestone |
| Multi-node relay chain planner | 📋 Backlog |
| Drift projection after e-stop | 📋 Backlog |

---

## Autopilot — Meridian

The navigation backbone is **Meridian** — a modern autopilot built on the Cube Plus flight computer. Key capabilities relevant to this platform:

- **Depth-chart position filter** — two versions running in parallel (classical particle filter + neural net), supervised with automatic hot-swap on divergence. Maintains position to 20–25 m on public chart data.
- **AIS landmark fusion** — uses nearby vessels broadcasting position to triangulate own position to 4 m when two or more are visible.
- **GPS-denied cold start** — blind chart search returns top-5 candidate cells; operator selects or follow-up filter converges.
- **Parallel filter supervisor** — cuts system-level divergence from ~3–5% to under 0.5%.

See `docs/deployment_runbook.pdf` for full flash and field deploy procedure.

---

## Platform specification

| Parameter | Requirement |
|---|---|
| Hull length | 1.0–2.0 m |
| Launch | Thrown/dropped from ship side, up to 10 m |
| Self-righting | Required, passive |
| Sprint speed | 18–25 kt |
| Cruise speed | 8–12 kt |
| Sea state | SS3 operational, SS4 survival |
| Endurance | 5–10 NM at cruise |
| Payload | Silvus StreamCaster + retractable mast |
| Autopilot | Cube Plus + Meridian v1.1 |
| Nav fallback | Depth-chart filter — GPS-denied to 25 m |
| Recovery | Ship-side, single crew, retrieval line |

---

## Connecting to Meridian

The UI is structured for a WebSocket connection to Jesse's `meridian-orca-bridge`. To swap in live telemetry, replace the simulated state in `index.html`:

```javascript
// Replace simulation with live bridge connection
const bridge = new WebSocket('ws://companion-computer:5760/telemetry');

bridge.onmessage = (e) => {
  const msg = JSON.parse(e.data);
  // msg.type: 'telemetry' | 'status' | 'alert'
  updateUSVState(msg);
};

// Send commands
function issueRTL(usvId) {
  bridge.send(JSON.stringify({ command: 'RTL', target: usvId }));
}
```

Full bridge API: `journalctl -u meridian-orca-bridge` — last 200 lines covers most issues.

---

## Getting help

For anything related to Meridian autopilot, Cube Plus integration, or bridge API:

1. `journalctl -u meridian-orca-bridge` — last 200 lines
2. Screenshot of the nav-source badge and status bar at the moment of the issue
3. One sentence: what you expected vs what happened

Those three things resolve most issues inside 30 minutes.

---

## Roadmap

**Now (pre-deploy)**
- Flash Meridian onto Cube Plus — 2–4 days first boot
- Capture Orca wire format from boat
- Bench test + dock test
- Sheltered water autonomous mission

**Next 1–2 weeks**
- Wire UI to live Meridian bridge WebSocket
- Drift projection screen post e-stop
- Multi-node relay chain mission planner

**3-week horizon (SOCOM demo)**
- On-water validation complete
- Jamming demo rehearsed
- Pre-recorded backup footage in the can

---

## Licence

Restricted — Vanguard programme personnel only.  
Not for distribution outside the programme without authorisation.

---

*Built with Meridian v1.1 · Vanguard + Meridian · May 2026*
