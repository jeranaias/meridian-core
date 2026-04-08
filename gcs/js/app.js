/* ============================================================
   app.js — Boot / initialization script
   T1-28: Extracted from inline <script> in index.html.
   Also includes T1-11 (tooltips), T1-19 (ARIA), T1-3 (ACK wiring).
   ============================================================ */

(function () {
    'use strict';

    // Wire connection messages to state handler
    Connection.onMessage = function (msg) {
        meridian.handleMessage(msg);
    };

    // T1-3: Wire COMMAND_ACK handling to Connection tracker
    meridian.events.on('command_ack', function (msg) {
        Connection.handleCommandAck(msg);
    });

    // Build mode buttons from settings
    // Mode categories for organized tray
    const MODE_CATEGORIES = {
        'Manual': [0, 1, 7],           // STABILIZE, ACRO, SPORT -> wait, SPORT is 13
        'Assisted': [2, 5, 16, 22],    // ALT_HOLD, LOITER, POSHOLD, FLOWHOLD
        'Auto': [3, 4, 6, 21, 27],     // AUTO, GUIDED, RTL, SMART_RTL, AUTO_RTL
        'Special': [9, 17, 14, 15, 20],// LAND, BRAKE, FLIP, AUTOTUNE, GUIDED_NOGPS
        'Advanced': [11, 13, 23, 24, 25, 19, 26], // DRIFT, SPORT, FOLLOW, ZIGZAG, SYSTEMID, AVOID_ADSB, AUTOROTATE
    };

    // T1-11: Tooltip map — mode descriptions for all mode buttons
    const MODE_TOOLTIPS = {
        0:  'Manual flight with self-leveling',
        1:  'Manual flight — no stabilization',
        2:  'Altitude hold — throttle controls climb rate',
        3:  'Follow pre-loaded mission waypoints',
        4:  'Fly to GPS coordinates on command',
        5:  'Hold position and altitude using GPS',
        6:  'Return to launch point and land',
        7:  'Orbit around a point at fixed radius',
        9:  'Descend and land at current position',
        11: 'Drift flight — coordinated turns',
        13: 'Sport mode — higher rates, self-leveling',
        14: 'Perform a flip maneuver',
        15: 'Auto-tune PID controller gains',
        16: 'Hold position — simpler than Loiter',
        17: 'Stop immediately and hold position',
        18: 'Launch by throwing the vehicle',
        19: 'Avoid ADS-B traffic automatically',
        20: 'Guided mode without GPS',
        21: 'Smart RTL — retrace path home',
        22: 'Hold position using optical flow',
        23: 'Follow another vehicle or GCS',
        24: 'Fly zigzag pattern for spraying',
        25: 'System identification mode',
        26: 'Autorotation for helicopters',
        27: 'Auto RTL — mission then return',
    };

    // Modes requiring confirmation before sending (safety-critical)
    const CONFIRM_MODES = { 6: 'Return to Launch', 9: 'Land Now' };

    function makeModeBtn(num, name, onClick) {
        const btn = document.createElement('button');
        btn.type = 'button';
        btn.className = 'mode-btn';
        btn.textContent = name;
        btn.dataset.mode = num;
        btn.title = MODE_TOOLTIPS[num] || (name + ' flight mode');
        btn.addEventListener('click', async () => {
            // Confirm dangerous modes
            if (CONFIRM_MODES[num] && meridian.v && meridian.v.armed) {
                const ok = await Modal.confirm(name, CONFIRM_MODES[num] + '?', name);
                if (!ok) return;
            }
            const v = meridian.v;
            if (v) v._userModeChange = true;
            Connection.sendSetMode(num);
            meridian.log('Mode change: ' + name, 'info');
            if (onClick) onClick();
        });
        return btn;
    }

    // Boat modes (subset that makes sense for surface vehicles)
    const BOAT_MODES = { 0: 'MANUAL', 2: 'LOITER', 3: 'RTL', 4: 'AUTO', 6: 'GUIDED' };
    const BOAT_COMMON = [0, 2, 4, 3]; // MANUAL, LOITER, AUTO, RTL

    function buildModeButtons() {
        const row = document.getElementById('mode-row');
        if (!row) return;

        var v = meridian.v;
        var isBoat = v && (v.vehicleClass === 'boat' || v.vehicleClass === 'rover');
        const MODES = isBoat ? BOAT_MODES : meridian.COPTER_MODES;
        const common = isBoat ? BOAT_COMMON : meridian.settings.commonModes;

        row.innerHTML = '';
        common.forEach(num => {
            const name = MODES[num] || ('MODE_' + num);
            row.appendChild(makeModeBtn(num, name));
        });

        // "More" button
        const more = document.createElement('button');
        more.type = 'button';
        more.className = 'mode-btn more';
        more.id = 'btn-more-modes';
        more.textContent = 'MORE \u25B4';
        more.title = 'Show all flight modes';
        more.addEventListener('click', toggleModeTray);
        row.appendChild(more);
    }

    function toggleModeTray() {
        const existing = document.getElementById('mode-tray');
        if (existing) { existing.remove(); return; }

        const MODES = meridian.COPTER_MODES;
        const common = meridian.settings.commonModes;

        const tray = document.createElement('div');
        tray.id = 'mode-tray';

        for (const [catName, catModes] of Object.entries(MODE_CATEGORIES)) {
            // Filter to modes that exist and aren't in common bar
            const visible = catModes.filter(n => MODES[n] && !common.includes(n));
            if (visible.length === 0) continue;

            const group = document.createElement('div');
            group.className = 'mode-tray-group';

            const label = document.createElement('span');
            label.className = 'mode-tray-label';
            label.textContent = catName;
            group.appendChild(label);

            const btns = document.createElement('div');
            btns.className = 'mode-tray-btns';
            visible.forEach(num => {
                const name = MODES[num];
                const btn = makeModeBtn(num, name, () => tray.remove());
                const v = meridian.v;
                if (v && v.modeNum === num) btn.classList.add('active');
                btns.appendChild(btn);
            });
            group.appendChild(btns);
            tray.appendChild(group);
        }

        document.getElementById('action-bar').appendChild(tray);

        setTimeout(() => {
            document.addEventListener('click', function close(ev) {
                if (!tray.contains(ev.target) && !ev.target.classList.contains('more')) {
                    tray.remove();
                    document.removeEventListener('click', close);
                }
            });
        }, 0);
    }

    // Update active mode highlighting + takeoff state + more label
    meridian.events.on('heartbeat', function (v) {
        const common = meridian.settings.commonModes;
        let activeInCommon = false;

        document.querySelectorAll('.mode-btn[data-mode]').forEach(btn => {
            const isActive = parseInt(btn.dataset.mode) === v.modeNum;
            btn.classList.toggle('active', isActive);
            // T1-19: Update aria-pressed on mode buttons
            btn.setAttribute('aria-pressed', isActive ? 'true' : 'false');
            if (isActive && common.includes(v.modeNum)) activeInCommon = true;
        });

        // If active mode is NOT in the common bar, show its name on the More button
        const moreBtn = document.getElementById('btn-more-modes');
        if (moreBtn) {
            if (!activeInCommon && v.modeName && v.modeName !== 'UNKNOWN') {
                moreBtn.textContent = v.modeName + ' \u25B4';
                moreBtn.classList.add('active');
            } else {
                moreBtn.textContent = 'MORE \u25B4';
                moreBtn.classList.remove('active');
            }
        }

        // Disable takeoff when not armed
        const toBtn = document.getElementById('btn-takeoff');
        if (toBtn) toBtn.disabled = !v.armed;
    });

    // Action buttons
    document.getElementById('btn-takeoff').addEventListener('click', async function () {
        const v = meridian.v;
        const alt = parseFloat(document.getElementById('takeoff-alt').value) || 10;

        if (!v || !v.armed) {
            meridian.log('Cannot takeoff: vehicle not armed', 'warn');
            return;
        }
        if (v.fixType < 3) {
            meridian.log('Cannot takeoff: no GPS fix (type ' + v.fixType + ')', 'warn');
            return;
        }
        if (alt > 120) {
            meridian.log('Altitude capped at 120m', 'warn');
            document.getElementById('takeoff-alt').value = 120;
            return;
        }

        const ok = await Modal.confirm('Takeoff', 'Launch to ' + alt + 'm altitude?', 'Takeoff');
        if (!ok) return;

        Connection.sendTakeoff(alt);
        meridian.log('Takeoff to ' + alt + 'm', 'info');
    });

    // RTL is now handled via mode buttons (no separate btn-rtl)

    var pauseBtn = document.getElementById('btn-pause');
    if (pauseBtn) {
        pauseBtn.addEventListener('click', function () {
            const v = meridian.v;
            if (v) v._userModeChange = true;
            Connection.sendSetMode('BRAKE');
            meridian.log('Hold position (BRAKE)', 'info');
        });
    }

    // Initialize everything
    Modal.init();
    buildModeButtons();
    FlyMap.init();
    Theme.init();
    Tlog.init();
    FlyView.init();
    if (window.VideoFeed) VideoFeed.init();
    Toolbar.init();
    Confirm.init();
    Confirm.initKeyboardShortcuts();
    Messages.init();
    ContextMenu.init();
    PlanView.init();
    if (window.OrbitTool) OrbitTool.init();
    SetupView.init();
    ParamsView.init();
    LogsView.init();
    Router.init();
    if (window.MultiVehicle) MultiVehicle.init();
    // T1-24: Initialize audio alerts
    if (window.AudioAlerts) AudioAlerts.init();
    // T2-14: i18n framework
    if (window.i18n) i18n.init();
    // T2-13: Onboarding tutorial
    if (window.Onboarding) Onboarding.init();
    // Vehicle class adaptation — hide/show controls based on vehicle type
    meridian.events.on('vehicle_class', function (cls) {
        var takeoffBtn = document.getElementById('btn-takeoff');
        var altInput = document.getElementById('takeoff-alt');
        var pauseBtn = document.getElementById('btn-pause');

        // Adapt tab labels
        var flyTab = document.querySelector('[data-panel="fly"]');
        if (flyTab) {
            var kbd = flyTab.querySelector('.kbd');
            var kbdHtml = kbd ? ' <span class="kbd">' + kbd.textContent + '</span>' : '';
            if (cls === 'boat') {
                flyTab.innerHTML = 'Nav' + kbdHtml;
                flyTab.setAttribute('aria-label', 'Navigate view (F)');
            } else if (cls === 'rover') {
                flyTab.innerHTML = 'Drive' + kbdHtml;
                flyTab.setAttribute('aria-label', 'Drive view (F)');
            } else if (cls === 'sub') {
                flyTab.innerHTML = 'Dive' + kbdHtml;
                flyTab.setAttribute('aria-label', 'Dive view (F)');
            } else {
                flyTab.innerHTML = 'Fly' + kbdHtml;
                flyTab.setAttribute('aria-label', 'Fly view (F)');
            }
        }

        if (cls === 'boat' || cls === 'rover') {
            if (takeoffBtn) takeoffBtn.style.display = 'none';
            if (altInput) altInput.style.display = 'none';
            if (pauseBtn) pauseBtn.innerHTML = '<svg width="10" height="10" viewBox="0 0 10 10" fill="currentColor"><circle cx="5" cy="5" r="4"/></svg> HOLD';
        } else {
            if (takeoffBtn) takeoffBtn.style.display = '';
            if (altInput) altInput.style.display = '';
            if (pauseBtn) pauseBtn.innerHTML = '<svg width="10" height="10" viewBox="0 0 10 10" fill="currentColor"><rect x="1" y="1" width="3" height="8"/><rect x="6" y="1" width="3" height="8"/></svg> HOLD';
        }

        buildModeButtons();
        meridian.log('Vehicle type: ' + cls, 'info');
    });

    // T3-4: AIS vessel tracker
    if (window.VesselTracker) VesselTracker.init();
    // T3-5: Wind estimation overlay
    if (window.WindOverlay) WindOverlay.init();
    // T3-3: Thermal camera widget
    if (window.ThermalWidget) ThermalWidget.init();
    // T3-13: EU compliance enforcement
    if (window.EUCompliance) EUCompliance.init();
    // T3-7: Fleet registry
    if (window.Fleet) Fleet.init();
    // T3-8: Dispatch engine
    if (window.Dispatch) Dispatch.init();
    // T3-12: Split-screen multi-vehicle view
    if (window.SplitView) SplitView.init();
    // T3-1: Agricultural spray widget
    if (window.SprayWidget) SprayWidget.init();
    // T3-2: Per-battery cycle tracking
    if (window.BatteryLifecycle) BatteryLifecycle.init();
    // T3-17: Quickshot cinematic modes
    if (window.Quickshots) Quickshots.init();

    // Rebuild mode buttons when settings change (e.g. from Settings panel)
    meridian.events.on('settings_change', function (data) {
        if (data && data.key === 'commonModes') {
            buildModeButtons();
        }
    });

    // Failsafe alert system
    const alertBanner = document.getElementById('alert-banner');
    meridian.events.on('failsafe', function (data) {
        if (!alertBanner) return;
        alertBanner.className = 'alert-banner active emergency';
        if (data.type === 'disarm_in_flight') {
            alertBanner.textContent = '\u26A0 DISARMED IN FLIGHT \u2014 ' + data.alt.toFixed(0) + 'm';
        } else {
            alertBanner.textContent = '\u26A0 FAILSAFE: ' + data.mode;
        }
        // T0-6: Require explicit acknowledgment — NO auto-dismiss
        alertBanner.onclick = function () {
            alertBanner.classList.remove('active');
            meridian.log('Alert acknowledged', 'info');
        };
        // Banner stays until pilot clicks it
    });

    // T0-14: Battery critical fires failsafe event + banner
    meridian.events.on('battery', function (v) {
        if (v.batteryPct >= 0 && v.batteryPct <= 15 && v.armed) {
            if (!v._battCritLogged) {
                meridian.events.emit('failsafe', { type: 'battery_critical', pct: v.batteryPct });
                v._battCritLogged = true;
            }
        } else {
            v._battCritLogged = false;
        }
    });

    // T0-7: Heartbeat timeout watchdog — VEHICLE TIMEOUT banner after 5s
    setInterval(function () {
        const v = meridian.v;
        if (!v) return;
        const elapsed = Date.now() - v.lastHeartbeat;
        const badge = document.querySelector('.flight-state-badge');

        if (elapsed > 5000 && v.connected) {
            // Hard timeout — show persistent banner
            if (!v._timeoutBanner) {
                alertBanner.className = 'alert-banner active warning';
                alertBanner.textContent = '\u26A0 VEHICLE TIMEOUT \u2014 no heartbeat for ' + Math.round(elapsed / 1000) + 's';
                alertBanner.onclick = function () {
                    alertBanner.classList.remove('active');
                };
                v._timeoutBanner = true;
            }
            if (badge) badge.classList.add('stale');
            // T1-24: Audio alert for link stale
            if (!v._staleSoundPlayed) {
                meridian.events.emit('link_stale');
                v._staleSoundPlayed = true;
            }
        } else if (elapsed > 3000 && v.connected) {
            // Soft stale — CSS class toggle (no more opacity hack)
            if (badge) badge.classList.add('stale');
            if (!v._staleLogged) {
                meridian.log('Telemetry stale \u2014 no heartbeat for 3s', 'warn');
                v._staleLogged = true;
            }
        } else {
            if (badge) badge.classList.remove('stale');
            v._staleLogged = false;
            v._timeoutBanner = false;
            v._staleSoundPlayed = false;
        }
    }, 500);

    // Connection click: connect to vehicle or stop demo
    const connEl = document.querySelector('.conn-indicator');
    if (connEl) {
        connEl.addEventListener('click', async function (e) {
            e.stopImmediatePropagation();
            e.preventDefault();
            if (Demo.isActive()) {
                Demo.stop();
                const v = meridian.v;
                if (v) { v.connected = false; v.armed = false; v.trail = []; }
                meridian.events.emit('heartbeat', v);
                Toolbar.updateConnection(0);
                meridian.log('Demo stopped', 'info');
            } else if (meridian.connectionState === 2) {
                Connection.disconnect();
            } else {
                const url = await Modal.prompt('Connect', 'Enter WebSocket URL:', 'ws://localhost:5760');
                if (url) Connection.connect(url, 'mnp');
            }
        }, true);
    }

    // ================================================================
    // T1-11: Apply tooltips to HUD elements and action buttons
    // ================================================================
    const TOOLTIP_MAP = {
        // HUD instruments
        '.adi-container':      'Artificial Horizon \u2014 shows vehicle roll and pitch',
        '.compass-container':  'Heading Compass \u2014 shows vehicle bearing',
        '.tape-speed':         'Airspeed / Groundspeed tape',
        '.tape-alt':           'Altitude tape \u2014 meters above home',
        '#battery-widget':     'Battery status \u2014 hover for details',
        '#health-grid':        'System health \u2014 GPS, EKF, vibration, RC',
        '#quick-widget':       'Quick telemetry values',

        // Action buttons
        '#btn-takeoff':        'Command vehicle takeoff to specified altitude',
        '#btn-pause':          'Hold position \u2014 sends BRAKE mode',
        '.kill-btn':           'Emergency Kill \u2014 hold 1.5s to terminate flight',
        '.slide-to-arm':       'Slide right to ARM, slide left to DISARM',

        // Toolbar
        '#theme-toggle':       'Toggle light/dark theme',
        '#rec-indicator':      'Recording indicator \u2014 tlog recording status',
    };

    for (const [selector, tip] of Object.entries(TOOLTIP_MAP)) {
        const el = document.querySelector(selector);
        if (el) el.title = tip;
    }

    // ================================================================
    // T1-19: Apply ARIA attributes for accessibility
    // ================================================================

    // aria-live regions
    const stateBadge = document.querySelector('.flight-state-badge');
    if (stateBadge) stateBadge.setAttribute('aria-live', 'polite');

    const msgLog = document.getElementById('msg-log');
    if (msgLog) msgLog.setAttribute('aria-live', 'polite');

    if (alertBanner) alertBanner.setAttribute('aria-live', 'assertive');

    // role="status" on connection indicator
    const connIndicator = document.querySelector('.conn-indicator');
    if (connIndicator) connIndicator.setAttribute('role', 'status');

    // aria-label on icon-only buttons
    const ARIA_LABELS = {
        '#theme-toggle': 'Toggle light or dark theme',
        '.toolbar-settings[title*="Settings"]': 'Open settings panel',
    };

    for (const [sel, label] of Object.entries(ARIA_LABELS)) {
        const el = document.querySelector(sel);
        if (el) el.setAttribute('aria-label', label);
    }

    // Map toolbar icon-only buttons
    document.querySelectorAll('.map-toolbar button').forEach(function (btn) {
        if (btn.title && !btn.getAttribute('aria-label')) {
            btn.setAttribute('aria-label', btn.title);
        }
    });

    // Panel close buttons
    document.querySelectorAll('.panel-close').forEach(function (btn) {
        btn.setAttribute('aria-label', 'Close panel');
    });

    // aria-pressed on mode buttons (initial state — updated live on heartbeat)
    document.querySelectorAll('.mode-btn[data-mode]').forEach(function (btn) {
        btn.setAttribute('aria-pressed', btn.classList.contains('active') ? 'true' : 'false');
    });

    // ================================================================
    // T1-24: Add audio mute toggle to settings
    // ================================================================
    meridian.events.on('panel_change', function (panel) {
        if (panel === 'settings' && window.AudioAlerts) {
            // The settings panel re-renders each time; add mute toggle after render
            setTimeout(function () {
                var settingsWrapper = document.querySelector('.settings-panel');
                if (!settingsWrapper || document.getElementById('audio-mute-toggle')) return;

                var header = document.createElement('div');
                header.className = 'settings-section-header';
                header.textContent = 'Audio Alerts';
                settingsWrapper.appendChild(header);

                var row = document.createElement('div');
                row.className = 'settings-toggle-row';
                row.id = 'audio-mute-toggle';

                var label = document.createElement('span');
                label.className = 'settings-toggle-label';
                label.textContent = 'Mute audio alerts';

                var toggle = document.createElement('label');
                toggle.className = 'settings-toggle';

                var input = document.createElement('input');
                input.type = 'checkbox';
                input.checked = AudioAlerts.isMuted();
                input.addEventListener('change', function () {
                    AudioAlerts.setMuted(input.checked);
                });

                var slider = document.createElement('span');
                slider.className = 'settings-toggle-slider';

                toggle.appendChild(input);
                toggle.appendChild(slider);
                row.appendChild(label);
                row.appendChild(toggle);
                settingsWrapper.appendChild(row);
            }, 50);
        }
    });

    // ================================================================
    // T1-5: ADSB traffic update timer — refresh every 2s
    // ================================================================
    setInterval(function () {
        if (window.FlyMap && FlyMap.updateAdsbTraffic) {
            FlyMap.updateAdsbTraffic();
        }
    }, 2000);

    // T1-5: Connect to ADSB server if configured
    meridian.events.on('settings_change', function (data) {
        if (data && data.key === 'adsbServer') {
            connectAdsbServer(data.value);
        }
    });

    var adsbWs = null;
    function connectAdsbServer(url) {
        if (adsbWs) { adsbWs.close(); adsbWs = null; }
        if (!url) return;
        try {
            adsbWs = new WebSocket(url);
            adsbWs.binaryType = 'arraybuffer';
            adsbWs.onmessage = function (evt) {
                if (!(evt.data instanceof ArrayBuffer)) return;
                var parser = new MAVLink.FrameParser();
                parser.push(new Uint8Array(evt.data));
                var msgs = parser.extract();
                for (var i = 0; i < msgs.length; i++) {
                    if (msgs[i].type === 'adsb_vehicle') {
                        meridian.handleMessage(msgs[i]);
                    }
                }
            };
            adsbWs.onopen = function () {
                meridian.log('ADSB server connected: ' + url, 'info');
            };
            adsbWs.onerror = function () {
                meridian.log('ADSB server connection error', 'warn');
            };
        } catch (e) {
            meridian.log('ADSB server connect failed: ' + e.message, 'warn');
        }
    }

    // Auto-connect on startup if configured
    if (meridian.settings.adsbServer) {
        connectAdsbServer(meridian.settings.adsbServer);
    }

    // ================================================================
    // T1-2: Keep geofence visible across view switches
    // ================================================================
    meridian.events.on('geofence_change', function (points) {
        if (window.FlyMap && FlyMap.updateGeofence) {
            FlyMap.updateGeofence(points);
        }
    });

    meridian.log('Meridian GCS initialized', 'info');
})();
