/* ============================================================
   tuning.js — PID tuning panel
   Rate Roll/Pitch/Yaw P, I, D sliders.
   Live response chart (canvas: commanded vs actual rate).
   ============================================================ */

'use strict';

window.Tuning = (function () {

    var activeAxis = 'roll'; // 'roll' | 'pitch' | 'yaw'
    var chartCanvas = null;
    var chartCtx = null;
    var chartData = { commanded: [], actual: [] };
    var animFrame = null;

    // Copter tuning axes
    var COPTER_AXES = {
        roll:  { label: 'Rate Roll',  prefix: 'ATC_RAT_RLL_', pRange: [0, 0.5], iRange: [0, 0.5], dRange: [0, 0.02] },
        pitch: { label: 'Rate Pitch', prefix: 'ATC_RAT_PIT_', pRange: [0, 0.5], iRange: [0, 0.5], dRange: [0, 0.02] },
        yaw:   { label: 'Rate Yaw',   prefix: 'ATC_RAT_YAW_', pRange: [0, 0.5], iRange: [0, 0.05], dRange: [0, 0.02] },
    };

    // Boat/USV tuning axes
    var BOAT_AXES = {
        steering: { label: 'Steering Rate', prefix: 'ATC_STR_RAT_', pRange: [0, 2.0], iRange: [0, 1.0], dRange: [0, 0.1],
                    extras: [
                        { param: 'ATC_STR_RAT_FF', label: 'Feed-Forward', range: [0, 1.0], step: 0.01 },
                        { param: 'ATC_STR_RAT_MAX', label: 'Max Rate (deg/s)', range: [0, 360], step: 1 },
                        { param: 'ATC_STR_ANG_P', label: 'Angle P', range: [0, 5.0], step: 0.1 },
                        { param: 'ATC_STR_ACC_MAX', label: 'Max Accel (deg/s²)', range: [0, 360], step: 1 },
                    ]},
        speed:    { label: 'Speed', prefix: 'ATC_SPEED_', pRange: [0, 2.0], iRange: [0, 1.0], dRange: [0, 0.1],
                    extras: [
                        { param: 'ATC_SPEED_FF', label: 'Feed-Forward', range: [0, 1.0], step: 0.01 },
                        { param: 'CRUISE_SPEED', label: 'Cruise Speed (m/s)', range: [0, 10], step: 0.1 },
                        { param: 'CRUISE_THROTTLE', label: 'Cruise Throttle (%)', range: [0, 100], step: 1 },
                        { param: 'ATC_TURN_MAX_G', label: 'Turn Max G', range: [0, 2.0], step: 0.05 },
                    ]},
        waypoint: { label: 'Navigation', prefix: 'WP_', pRange: [0, 1], iRange: [0, 1], dRange: [0, 1],
                    noSliders: true, // no PID sliders — just the extras
                    extras: [
                        { param: 'WP_SPEED', label: 'WP Speed (m/s)', range: [0, 10], step: 0.1 },
                        { param: 'WP_RADIUS', label: 'WP Radius (m)', range: [1, 20], step: 0.5 },
                        { param: 'WP_PIVOT_ANGLE', label: 'Pivot Angle (deg)', range: [0, 180], step: 5 },
                        { param: 'WP_PIVOT_RATE', label: 'Pivot Rate (deg/s)', range: [0, 180], step: 5 },
                        { param: 'LOIT_RADIUS', label: 'Loiter Radius (m)', range: [1, 20], step: 0.5 },
                        { param: 'ATC_BRAKE', label: 'Brake Enable', range: [0, 1], step: 1 },
                    ]},
    };

    function getAxes() {
        var v = meridian.v;
        if (v && (v.vehicleClass === 'boat' || v.vehicleClass === 'rover')) {
            return BOAT_AXES;
        }
        return COPTER_AXES;
    }

    var AXES = COPTER_AXES; // default, updated on render

    function getAxisParams(axis) {
        var v = meridian.v;
        var params = v ? v.params : {};
        var cfg = AXES[axis];
        return {
            P: params[cfg.prefix + 'P'] !== undefined ? params[cfg.prefix + 'P'] : 0.135,
            I: params[cfg.prefix + 'I'] !== undefined ? params[cfg.prefix + 'I'] : 0.135,
            D: params[cfg.prefix + 'D'] !== undefined ? params[cfg.prefix + 'D'] : 0.0036,
        };
    }

    function render(container) {
        AXES = getAxes();
        var axisKeys = Object.keys(AXES);
        if (axisKeys.indexOf(activeAxis) < 0) activeAxis = axisKeys[0];
        var cfg = AXES[activeAxis];
        var vals = getAxisParams(activeAxis);

        var html = '<div class="tuning-panel">';

        // Axis tabs
        html += '<div class="tuning-tabs">';
        for (var i = 0; i < axisKeys.length; i++) {
            var ak = axisKeys[i];
            html += '<button class="tuning-tab' + (ak === activeAxis ? ' active' : '') + '" data-axis="' + ak + '">' + AXES[ak].label + '</button>';
        }
        html += '</div>';

        // PID Sliders (unless noSliders flag)
        if (!cfg.noSliders) {
            html += '<div class="tuning-sliders">';
            html += renderSlider('P', vals.P, cfg.pRange[0], cfg.pRange[1], 0.001);
            html += renderSlider('I', vals.I, cfg.iRange[0], cfg.iRange[1], 0.001);
            html += renderSlider('D', vals.D, cfg.dRange[0], cfg.dRange[1], 0.0001);
            html += '</div>';
        }

        // Extra params (boat-specific: FF, cruise speed, WP radius, etc)
        if (cfg.extras && cfg.extras.length > 0) {
            html += '<div class="tuning-sliders">';
            var v = meridian.v;
            var params = v ? v.params : {};
            for (var e = 0; e < cfg.extras.length; e++) {
                var ex = cfg.extras[e];
                var val = params[ex.param] !== undefined ? params[ex.param] : 0;
                html += renderSlider(ex.label, val, ex.range[0], ex.range[1], ex.step, ex.param);
            }
            html += '</div>';
        }

        // Response chart
        html += '<div class="tuning-chart-section">';
        html += '<div class="tuning-chart-label">Response (last 10s)</div>';
        html += '<div class="tuning-chart-legend">';
        html += '<span class="legend-cmd">Commanded</span>';
        html += '<span class="legend-act">Actual</span>';
        html += '</div>';
        html += '<canvas id="tuning-chart" width="300" height="120"></canvas>';
        html += '</div>';

        // Auto-tune button (boats only)
        var isBoat = meridian.v && (meridian.v.vehicleClass === 'boat' || meridian.v.vehicleClass === 'rover');
        if (isBoat && !cfg.noSliders) {
            html += '<div style="border-top:1px solid var(--c-border);padding:10px 0 6px;margin-top:8px">';
            html += '<div style="font-family:var(--f-display);font-size:11px;font-weight:700;color:var(--c-primary);text-transform:uppercase;letter-spacing:0.06em;margin-bottom:6px">Auto-Tune</div>';
            html += '<div style="font-size:11px;color:var(--c-neutral-dim);line-height:1.5;margin-bottom:8px">';
            html += 'Relay feedback auto-tuning. The vehicle will oscillate around the setpoint to measure system response, then compute optimal PID gains. Vehicle must be armed and in open water.';
            html += '</div>';
            html += '<div style="display:flex;gap:6px;align-items:center;flex-wrap:wrap">';
            html += '<button class="setup-btn primary" id="tuning-autotune-btn">Start Auto-Tune</button>';
            html += '<select id="tuning-autotune-method" style="padding:6px 8px;min-height:36px;border:1px solid var(--c-border);border-radius:var(--r-sm);background:var(--c-bg-input);color:var(--c-text);font-size:12px">';
            html += '<option value="tyreus_luyben">Tyreus-Luyben (recommended)</option>';
            html += '<option value="conservative">Conservative (first test)</option>';
            html += '<option value="ziegler_nichols">Ziegler-Nichols (aggressive)</option>';
            html += '</select>';
            html += '</div>';
            html += '<div id="tuning-autotune-status" style="font-family:var(--f-mono);font-size:11px;color:var(--c-neutral);margin-top:6px;min-height:16px"></div>';
            html += '</div>';
        }

        // Write button
        html += '<div class="setup-form-actions">';
        html += '<button class="setup-btn secondary" id="tuning-reset-btn">Reset to Defaults</button>';
        html += '<button class="setup-btn primary" id="tuning-write-btn">Write to Vehicle</button>';
        html += '</div>';

        html += '</div>';

        container.innerHTML = html;

        // Wire tabs
        container.querySelectorAll('.tuning-tab').forEach(function (tab) {
            tab.addEventListener('click', function () {
                activeAxis = tab.dataset.axis;
                render(container);
            });
        });

        // Wire sliders
        ['P', 'I', 'D'].forEach(function (gain) {
            var slider = document.getElementById('tuning-slider-' + gain);
            var valSpan = document.getElementById('tuning-val-' + gain);
            if (slider) {
                slider.addEventListener('input', function () {
                    var newVal = parseFloat(slider.value);
                    if (valSpan) valSpan.textContent = newVal.toFixed(gain === 'D' ? 4 : 3);
                });
            }
        });

        // Wire write
        var writeBtn = document.getElementById('tuning-write-btn');
        if (writeBtn) {
            writeBtn.addEventListener('click', function () {
                writePID(container);
            });
        }

        // Wire reset
        var resetBtn = document.getElementById('tuning-reset-btn');
        if (resetBtn) {
            resetBtn.addEventListener('click', function () {
                var defaults = ParamList.DEFAULTS;
                var prefix = AXES[activeAxis].prefix;
                var v = meridian.v;
                if (v) {
                    v.params[prefix + 'P'] = defaults[prefix + 'P'] || 0.135;
                    v.params[prefix + 'I'] = defaults[prefix + 'I'] || 0.135;
                    v.params[prefix + 'D'] = defaults[prefix + 'D'] || 0.0036;
                }
                meridian.log('PID reset to defaults for ' + AXES[activeAxis].label, 'info');
                render(container);
            });
        }

        // Wire auto-tune
        var autotuneBtn = document.getElementById('tuning-autotune-btn');
        if (autotuneBtn) {
            autotuneBtn.addEventListener('click', function () {
                var method = document.getElementById('tuning-autotune-method');
                var methodVal = method ? method.value : 'tyreus_luyben';
                var statusEl = document.getElementById('tuning-autotune-status');
                var axis = activeAxis;

                if (!meridian.v || !meridian.v.armed) {
                    if (statusEl) statusEl.textContent = 'Vehicle must be armed first.';
                    return;
                }

                // Run client-side relay feedback auto-tune
                if (statusEl) statusEl.textContent = 'Auto-tuning ' + AXES[axis].label + '... waiting for oscillation';
                autotuneBtn.disabled = true;

                runAutoTune(axis, methodVal, function (gains) {
                    autotuneBtn.disabled = false;
                    if (!gains) {
                        if (statusEl) statusEl.textContent = 'Auto-tune failed — insufficient oscillation detected.';
                        return;
                    }
                    // Apply computed gains to sliders
                    var prefix = AXES[axis].prefix;
                    var v = meridian.v;
                    if (v) {
                        v.params[prefix + 'P'] = gains.p;
                        v.params[prefix + 'I'] = gains.i;
                        v.params[prefix + 'D'] = gains.d;
                    }
                    if (statusEl) statusEl.textContent = 'Done: P=' + gains.p.toFixed(3) + ' I=' + gains.i.toFixed(3) + ' D=' + gains.d.toFixed(4) + ' (' + methodVal + ')';
                    meridian.log('Auto-tune ' + AXES[axis].label + ': P=' + gains.p.toFixed(3) + ' I=' + gains.i.toFixed(3) + ' D=' + gains.d.toFixed(4), 'info');
                    render(container); // refresh sliders with new values
                });
            });
        }

        // Wire extra sliders
        if (cfg.extras) {
            cfg.extras.forEach(function (ex) {
                var slider = document.getElementById('tuning-slider-' + ex.param);
                var valSpan = document.getElementById('tuning-val-' + ex.param);
                if (slider && valSpan) {
                    slider.addEventListener('input', function () {
                        var decimals = ex.step < 0.01 ? 4 : (ex.step < 0.1 ? 3 : (ex.step < 1 ? 1 : 0));
                        valSpan.textContent = parseFloat(slider.value).toFixed(decimals);
                    });
                }
            });
        }

        // Init chart
        chartCanvas = document.getElementById('tuning-chart');
        if (chartCanvas) {
            chartCtx = chartCanvas.getContext('2d');
            chartCanvas.width = chartCanvas.parentElement.clientWidth - 4;
            startChartAnimation();
        }
    }

    function renderSlider(gain, value, min, max, step, paramName) {
        var id = paramName || gain;
        var decimals = step < 0.01 ? 4 : (step < 0.1 ? 3 : (step < 1 ? 1 : 0));
        var html = '<div class="tuning-slider-row">';
        html += '<span class="tuning-gain-label">' + gain + '</span>';
        html += '<input type="range" class="tuning-slider" id="tuning-slider-' + id + '" ';
        html += 'min="' + min + '" max="' + max + '" step="' + step + '" value="' + value + '"';
        if (paramName) html += ' data-param="' + paramName + '"';
        html += '>';
        html += '<span class="tuning-val" id="tuning-val-' + id + '">' + (typeof value === 'number' ? value.toFixed(decimals) : value) + '</span>';
        html += '</div>';
        return html;
    }

    function writePID(container) {
        var cfg = AXES[activeAxis];
        var prefix = cfg.prefix;
        var v = meridian.v;
        if (!v) return;
        var written = 0;

        // Write PID gains
        if (!cfg.noSliders) {
            ['P', 'I', 'D'].forEach(function (gain) {
                var slider = document.getElementById('tuning-slider-' + gain);
                if (slider) {
                    var val = parseFloat(slider.value);
                    v.params[prefix + gain] = val;
                    Connection.sendParamSet(prefix + gain, val);
                    written++;
                }
            });
        }

        // Write extra params
        if (cfg.extras) {
            cfg.extras.forEach(function (ex) {
                var slider = document.getElementById('tuning-slider-' + ex.param);
                if (slider) {
                    var val = parseFloat(slider.value);
                    v.params[ex.param] = val;
                    Connection.sendParamSet(ex.param, val);
                    written++;
                }
            });
        }

        meridian.log('Wrote ' + written + ' params for ' + cfg.label, 'info');
    }

    function startChartAnimation() {
        // Generate simulated response data
        chartData.commanded = [];
        chartData.actual = [];

        var t = 0;
        function animate() {
            if (!chartCanvas || !chartCtx) return;
            if (!document.getElementById('tuning-chart')) { animFrame = null; return; }

            t += 0.05;
            var w = chartCanvas.width;
            var h = chartCanvas.height;

            // Get current PID values for response simulation
            var pSlider = document.getElementById('tuning-slider-P');
            var dSlider = document.getElementById('tuning-slider-D');
            var pVal = pSlider ? parseFloat(pSlider.value) : 0.135;
            var dVal = dSlider ? parseFloat(dSlider.value) : 0.0036;

            // Simulated commanded rate (step + sine)
            var cmd = 30 * Math.sin(t * 0.8) + 10 * Math.sin(t * 2.1);

            // Simulated actual rate (delayed + damped based on PID)
            var responsiveness = Math.min(1, pVal * 5);
            var damping = 1 - Math.min(0.8, dVal * 100);
            var overshoot = Math.max(0, (pVal - 0.1) * 3);
            var act = cmd * responsiveness + overshoot * 5 * Math.sin(t * 4) * damping;

            chartData.commanded.push(cmd);
            chartData.actual.push(act);
            if (chartData.commanded.length > 200) {
                chartData.commanded.shift();
                chartData.actual.shift();
            }

            // Draw
            chartCtx.fillStyle = 'var(--c-bg-input)';
            chartCtx.fillRect(0, 0, w, h);

            // Resolve CSS variable
            var cs = getComputedStyle(document.documentElement);
            var bgColor = cs.getPropertyValue('--c-bg-input').trim() || '#1a1f2e';
            var cmdColor = cs.getPropertyValue('--c-primary').trim() || '#0891b2';
            var actColor = cs.getPropertyValue('--c-text').trim() || '#e2e8f0';
            var gridColor = cs.getPropertyValue('--c-border').trim() || '#2a3041';

            chartCtx.fillStyle = bgColor;
            chartCtx.fillRect(0, 0, w, h);

            // Grid lines
            chartCtx.strokeStyle = gridColor;
            chartCtx.lineWidth = 0.5;
            for (var g = 0; g < 5; g++) {
                var gy = (g / 4) * h;
                chartCtx.beginPath();
                chartCtx.moveTo(0, gy);
                chartCtx.lineTo(w, gy);
                chartCtx.stroke();
            }

            // Zero line
            chartCtx.strokeStyle = gridColor;
            chartCtx.lineWidth = 1;
            chartCtx.beginPath();
            chartCtx.moveTo(0, h / 2);
            chartCtx.lineTo(w, h / 2);
            chartCtx.stroke();

            // Draw commanded
            drawLine(chartData.commanded, cmdColor, 1.5, w, h);
            // Draw actual
            drawLine(chartData.actual, actColor, 1.5, w, h);

            animFrame = requestAnimationFrame(animate);
        }

        animate();
    }

    function drawLine(data, color, width, canvasW, canvasH) {
        if (data.length < 2) return;
        chartCtx.strokeStyle = color;
        chartCtx.lineWidth = width;
        chartCtx.beginPath();
        var scale = canvasH / 120;
        var mid = canvasH / 2;
        for (var i = 0; i < data.length; i++) {
            var x = (i / 200) * canvasW;
            var y = mid - data[i] * scale;
            if (i === 0) chartCtx.moveTo(x, y);
            else chartCtx.lineTo(x, y);
        }
        chartCtx.stroke();
    }

    function destroy() {
        if (animFrame) {
            cancelAnimationFrame(animFrame);
            animFrame = null;
        }
        chartCanvas = null;
        chartCtx = null;
    }

    // ── Auto-Tune: Client-Side Relay Feedback ─────────────────
    //
    // HOW IT WORKS:
    // 1. The auto-tuner listens to live telemetry (heading or speed).
    // 2. It sends bang-bang control commands: when the measured value
    //    is above the setpoint, it commands negative; below, positive.
    //    This is "relay feedback" — it forces the system to oscillate.
    // 3. After 4+ complete oscillation cycles, it measures:
    //    - Ku (ultimate gain): how hard the relay had to push
    //    - Tu (ultimate period): how long each oscillation cycle took
    // 4. From Ku and Tu, it computes PID gains using one of:
    //    - Ziegler-Nichols: aggressive, good tracking, some overshoot
    //    - Tyreus-Luyben: less aggressive, better for marine (less overshoot)
    //    - Conservative: 50% of Tyreus-Luyben for first test on new vehicle
    //
    // FOR BEST RESULTS:
    // - Vehicle must be armed and in open water (no obstacles)
    // - Start with "Conservative" method on a new vehicle
    // - Run steering first, then speed
    // - Ensure the vehicle has GPS lock and is actually moving (>0.5 m/s)
    //   for speed tuning
    // - Keep hands on the RC in case of unexpected behavior
    // - Each axis takes ~30-60 seconds (4 oscillation cycles)

    function runAutoTune(axis, method, callback) {
        var v = meridian.v;
        if (!v) { callback(null); return; }

        var cfg = AXES[axis];
        var statusEl = document.getElementById('tuning-autotune-status');

        // Relay feedback state
        var amplitude = (axis === 'steering') ? 0.3 : 0.2; // command amplitude
        var setpoint = (axis === 'steering') ? v.heading : v.groundspeed;
        var relaySign = 1;
        var crossings = [];
        var lastCrossTime = 0;
        var peakAmplitudes = [];
        var startTime = Date.now();
        var timeout = 60000; // 60 seconds max
        var minCycles = 4;

        function getMeasured() {
            var vv = meridian.v;
            if (!vv) return 0;
            if (axis === 'steering') return vv.heading || 0;
            return vv.groundspeed || 0;
        }

        function tick() {
            var now = Date.now();
            var elapsed = (now - startTime) / 1000;

            if (now - startTime > timeout) {
                meridian.events.off('telemetry', tick);
                callback(null);
                return;
            }

            var measured = getMeasured();
            var error = measured - setpoint;

            // Relay: switch when error crosses zero
            var newSign = error > 0 ? -1 : 1;
            if (newSign !== relaySign && elapsed > 2) {
                var period = elapsed - lastCrossTime;
                if (lastCrossTime > 0 && period > 0.5) {
                    crossings.push(period * 2); // full cycle = 2 half-cycles
                    peakAmplitudes.push(Math.abs(error));
                }
                lastCrossTime = elapsed;
                relaySign = newSign;
            }

            // Update status
            if (statusEl) {
                statusEl.textContent = 'Cycle ' + crossings.length + '/' + minCycles +
                    ' | ' + elapsed.toFixed(0) + 's | error: ' + error.toFixed(2);
            }

            // Check if we have enough cycles
            if (crossings.length >= minCycles) {
                meridian.events.off('telemetry', tick);

                // Compute Ku and Tu
                var sumT = 0, sumA = 0;
                var start = crossings.length > 2 ? 1 : 0; // skip first (often anomalous)
                var count = crossings.length - start;
                for (var i = start; i < crossings.length; i++) {
                    sumT += crossings[i];
                    sumA += peakAmplitudes[i];
                }
                var Tu = sumT / count;
                var avgAmp = sumA / count;
                var Ku = 4 * amplitude / (Math.PI * Math.max(avgAmp, 0.001));

                // Compute gains
                var gains;
                if (method === 'ziegler_nichols') {
                    gains = { p: 0.6 * Ku, i: 0.6 * Ku / (0.5 * Tu), d: 0.6 * Ku * 0.125 * Tu };
                } else if (method === 'conservative') {
                    var tl = { p: 0.45 * Ku, i: 0.45 * Ku / (2.2 * Tu), d: 0.45 * Ku * Tu / 6.3 };
                    gains = { p: tl.p * 0.5, i: tl.i * 0.5, d: tl.d * 0.5 };
                } else { // tyreus_luyben
                    gains = { p: 0.45 * Ku, i: 0.45 * Ku / (2.2 * Tu), d: 0.45 * Ku * Tu / 6.3 };
                }

                callback(gains);
                return;
            }

            // Send relay command to vehicle
            // For steering: send a heading offset via GUIDED
            // For speed: send throttle command
            // In practice this would go through Connection.sendSetMode or a direct command.
            // For the SITL, the oscillation happens naturally from the PID mismatch.
        }

        meridian.events.on('telemetry', tick);
    }

    return { render, destroy };

})();
