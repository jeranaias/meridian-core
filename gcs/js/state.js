/* ============================================================
   state.js — Multi-vehicle state + event bus
   Meier revision: meridian.vehicles[sysid], not flat namespace
   ============================================================ */

'use strict';

(function () {

    // --- Event Bus ---
    const _listeners = {};

    // T2-5: Track last-emit timestamps for throttled events
    const _throttleLast = {};

    const events = {
        on(event, fn) {
            if (!_listeners[event]) _listeners[event] = [];
            _listeners[event].push(fn);
        },
        off(event, fn) {
            if (!_listeners[event]) return;
            _listeners[event] = _listeners[event].filter(f => f !== fn);
        },
        emit(event, data) {
            if (_listeners[event]) {
                for (const fn of _listeners[event]) {
                    try { fn(data); } catch (e) { console.error('Event handler error:', event, e); }
                }
            }
        },
        // T2-5: Coalescing emit — skips if same event fired within intervalMs.
        // Used for high-rate telemetry events ('attitude', 'position') to cap at 20Hz.
        // Safety-critical events ('heartbeat', 'failsafe', 'command_ack') must use emit().
        emitThrottled(event, data, intervalMs) {
            const now = Date.now();
            if (_throttleLast[event] && (now - _throttleLast[event]) < intervalMs) return;
            _throttleLast[event] = now;
            this.emit(event, data);
        },
    };

    // --- Default vehicle state ---
    function createVehicleState(sysid) {
        return {
            sysid: sysid,
            connected: false,
            armed: false,
            modeNum: 0,
            modeName: 'UNKNOWN',
            vehicleClass: 'copter', // 'copter', 'plane', 'boat', 'rover', 'sub'
            systemStatus: 0,

            // Attitude (radians)
            roll: 0, pitch: 0, yaw: 0,
            rollspeed: 0, pitchspeed: 0, yawspeed: 0,
            targetRoll: null, targetPitch: null,

            // Position
            lat: 0, lon: 0,
            alt: 0, relativeAlt: 0,
            vx: 0, vy: 0, vz: 0,
            hdg: 0,

            // Home
            homeLat: null, homeLon: null, homeAlt: null,

            // VFR HUD
            airspeed: 0, groundspeed: 0,
            heading: 0, throttle: 0,
            climb: 0,
            targetAlt: null, targetSpeed: null,

            // Battery
            voltage: 0, current: 0, batteryPct: -1,
            mah: 0,

            // GPS
            fixType: 0, satellites: 0,
            hdop: 99.9, vdop: 99.9,
            gpsLat: 0, gpsLon: 0,

            // EKF
            ekfVelVar: 0, ekfPosVar: 0,
            ekfHgtVar: 0, ekfMagVar: 0,
            ekfTerrVar: 0, ekfFlags: 0,

            // RC
            rcChannels: [],
            rcRssi: 0,

            // Mission
            missionCount: 0,
            missionSeq: 0,
            missionItems: [],

            // Params
            params: {},
            paramCount: 0,

            // Trail
            trail: [],

            // Terrain
            terrainAlt: 0,
            terrainClearance: 0,

            // Servo outputs
            servoOutputs: [],

            // Arm denied reason (from STATUSTEXT PreArm messages)
            armDeniedReason: '',

            // T1-27: Vibration data
            vibrationX: 0, vibrationY: 0, vibrationZ: 0,
            clipping0: 0, clipping1: 0, clipping2: 0,
            imuVibe: 0,

            // T1-27: Fence status
            fenceBreach: false,
            fenceBreachCount: 0,
            fenceBreachType: 0,

            // T1-27: Autopilot version
            autopilotVersion: null,

            // T1-27: Battery status (detailed)
            cellVoltages: [],

            // T1-12: Remote ID
            remoteId: {
                uasId: '',
                uasType: 0,
                operatorId: '',
                lat: 0,
                lon: 0,
                alt: 0,
            },

            // T3-1: Agricultural spray telemetry
            sprayFlowRate: null,   // L/min — null = no data
            sprayTankLevel: null,  // %
            sprayActive: false,    // boolean spray on/off
            sprayArea: null,       // ha area sprayed

            // T3-3: Thermal camera data
            thermalMinTemp: undefined,  // °C minimum scene temperature
            thermalMaxTemp: undefined,  // °C maximum scene temperature
            thermalSpotTemp: undefined, // °C center spot temperature

            // Timestamps
            lastHeartbeat: 0,
            lastAttitude: 0,
            lastPosition: 0,
            lastBattery: 0,
            flightStartTime: null,
        };
    }

    // --- Global state (Meier: multi-vehicle from day 1) ---
    const meridian = {
        activeVehicleId: 1,
        vehicles: {
            1: createVehicleState(1),
        },

        // Active vehicle shortcut (Meier: return null not {} for safety)
        get v() {
            return this.vehicles[this.activeVehicleId] || null;
        },

        // Connection
        connectionState: 0, // 0=disconnected, 1=connecting, 2=connected
        protocol: 'mavlink',

        // Settings (Meier addition)
        settings: {
            units: 'metric',
            mapProvider: 'carto-dark',
            guidedAltMax: 120,
            guidedDistMax: 1000,
            reconnectDelay: 3000,
            adsbServer: '',
            showAdsb: true,
            showUncertainty: true,
            showTrajectory: true,
            showSprayWidget: false,
            // Krug Round 2 fix: LAND removed from default common modes
            commonModes: [0, 2, 5, 6, 3], // Stabilize, AltHold, Loiter, RTL, Auto
            autoRecord: true,
            operatorRegistration: '', // T1-12: Remote ID operator registration number
            // T3-3: Thermal camera widget
            showThermalWidget: false,
            // T3-4: AIS vessel tracker
            aisServer: '',       // WebSocket URL for AIS data source
            ownMmsi: '',         // MMSI of own vessel for home-point tracking
            // T3-13: EU compliance
            euUasClass: 'C0',    // UAS C-class: C0-C6
            euOpCategory: 'open_a1', // Operational category
        },

        // T1-5: ADSB traffic contacts — ICAO -> {lat, lon, alt, heading, callsign, lastSeen}
        adsb: {},

        // Recording (Oborne fix: will use IndexedDB, not in-memory)
        tlog: {
            recording: false,
            byteCount: 0,
            startTime: null,
            filename: '',
        },

        // Messages
        messages: [],

        // Events
        events: events,

        // Demo mode
        demo: false,

        // Helpers
        createVehicleState: createVehicleState,

        log(msg, level) {
            const entry = {
                time: Date.now(),
                text: msg,
                level: level || 'info',
            };
            this.messages.push(entry);
            if (this.messages.length > 200) this.messages.shift();
            events.emit('message', entry);
        },
    };

    // Load settings from localStorage
    try {
        const saved = localStorage.getItem('meridian_settings');
        if (saved) {
            Object.assign(meridian.settings, JSON.parse(saved));
        }
    } catch (e) { /* ignore */ }

    // Save settings on change
    meridian.saveSettings = function () {
        try {
            localStorage.setItem('meridian_settings', JSON.stringify(this.settings));
        } catch (e) { /* ignore */ }
    };

    // --- Message handler: dispatches incoming telemetry to vehicle state ---
    function handleMessage(msg, sysid) {
        sysid = sysid || meridian.activeVehicleId;
        if (!meridian.vehicles[sysid]) {
            meridian.vehicles[sysid] = createVehicleState(sysid);
        }
        const v = meridian.vehicles[sysid];
        const now = Date.now();

        switch (msg.type) {
            case 'heartbeat':
                v.connected = true;
                const wasArmed = v.armed;
                const prevMode = v.modeName;
                v.armed = msg.armed;
                if (msg.custom_mode !== undefined) v.modeNum = msg.custom_mode;
                v.modeName = msg.mode || 'UNKNOWN';
                v.systemStatus = msg.system_status || 0;
                v.lastHeartbeat = now;

                // Vehicle class detection (boat, copter, plane, rover, sub)
                if (msg.vehicle_class && v.vehicleClass !== msg.vehicle_class) {
                    v.vehicleClass = msg.vehicle_class;
                    events.emit('vehicle_class', v.vehicleClass);
                    // Update map icon
                    if (window.FlyMap && FlyMap.setVehicleType) {
                        FlyMap.setVehicleType(v.vehicleClass);
                    }
                }
                if (msg.armed && !v.flightStartTime) v.flightStartTime = now;
                if (!msg.armed) v.flightStartTime = null;

                // Failsafe detection: unexpected mode changes while armed
                if (wasArmed && v.armed && prevMode !== v.modeName) {
                    const fsTargets = ['RTL', 'LAND', 'BRAKE', 'SMART_RTL'];
                    if (fsTargets.includes(v.modeName) && !v._userModeChange) {
                        meridian.log('FAILSAFE: mode changed to ' + v.modeName, 'error');
                        events.emit('failsafe', { mode: v.modeName, prev: prevMode });
                    }
                }
                v._userModeChange = false;

                // Unexpected disarm in flight
                if (wasArmed && !v.armed && v.relativeAlt > 2) {
                    meridian.log('WARNING: disarmed at ' + v.relativeAlt.toFixed(0) + 'm altitude', 'error');
                    events.emit('failsafe', { type: 'disarm_in_flight', alt: v.relativeAlt });
                }

                events.emit('heartbeat', v);
                break;

            case 'attitude':
                v.roll = msg.roll;
                v.pitch = msg.pitch;
                v.yaw = msg.yaw;
                v.rollspeed = msg.rollspeed;
                v.pitchspeed = msg.pitchspeed;
                v.yawspeed = msg.yawspeed;
                v.lastAttitude = now;
                // T2-5: Throttle to 20Hz max (50ms) — state is always updated above
                events.emitThrottled('attitude', v, 50);
                break;

            case 'position':
                v.lat = msg.lat;
                v.lon = msg.lon;
                v.alt = msg.alt;
                v.relativeAlt = msg.relative_alt;
                v.vx = msg.vx;
                v.vy = msg.vy;
                v.vz = msg.vz;
                v.hdg = msg.hdg;
                v.lastPosition = now;
                // Update trail
                if (msg.lat !== 0 && msg.lon !== 0) {
                    v.trail.push([msg.lat, msg.lon]);
                    if (v.trail.length > 300) v.trail.shift();
                }
                // Set home on first position if not set
                if (v.homeLat === null && msg.lat !== 0) {
                    v.homeLat = msg.lat;
                    v.homeLon = msg.lon;
                    v.homeAlt = msg.alt;
                }
                // T2-5: Throttle to 20Hz max (50ms) — state is always updated above
                events.emitThrottled('position', v, 50);
                break;

            case 'vfr_hud':
                v.airspeed = msg.airspeed;
                v.groundspeed = msg.groundspeed;
                v.heading = msg.heading;
                v.throttle = msg.throttle;
                v.alt = msg.alt;
                v.climb = msg.climb;
                events.emit('vfr_hud', v);
                break;

            case 'battery':
                v.voltage = msg.voltage || msg.voltage_battery;
                v.current = msg.current || msg.current_battery;
                v.batteryPct = msg.remaining !== undefined ? msg.remaining : msg.battery_remaining;
                v.lastBattery = now;
                events.emit('battery', v);
                break;

            case 'sys_status':
                v.voltage = msg.voltage_battery;
                v.current = msg.current_battery;
                v.batteryPct = msg.battery_remaining;
                v.lastBattery = now;
                events.emit('battery', v);
                break;

            case 'gps_raw':
                v.fixType = msg.fix_type;
                v.satellites = msg.satellites;
                v.hdop = (msg.eph || 9999) / 100;
                v.vdop = (msg.epv || 9999) / 100;
                v.gpsLat = msg.lat;
                v.gpsLon = msg.lon;
                events.emit('gps', v);
                break;

            case 'ekf_status':
                v.ekfVelVar = msg.velocity_variance;
                v.ekfPosVar = msg.pos_horiz_variance;
                v.ekfHgtVar = msg.pos_vert_variance;
                v.ekfMagVar = msg.compass_variance;
                v.ekfTerrVar = msg.terrain_variance;
                v.ekfFlags = msg.flags;
                events.emit('ekf', v);
                break;

            case 'rc_channels':
                v.rcChannels = msg.channels || [];
                v.rcRssi = msg.rssi;
                events.emit('rc', v);
                break;

            case 'param_value':
                v.params[msg.name] = msg.value;
                v.paramCount = msg.param_count || v.paramCount;
                events.emit('param', msg);
                break;

            case 'command_ack': {
                // T1-3: Match ACK to pending command, log result
                const ACK_RESULTS = {
                    0: 'ACCEPTED',
                    1: 'TEMPORARILY_REJECTED',
                    2: 'DENIED',
                    3: 'UNSUPPORTED',
                    4: 'FAILED',
                };
                const resultName = ACK_RESULTS[msg.result] || ('UNKNOWN_' + msg.result);
                if (msg.result !== 0) {
                    meridian.log('Command ' + msg.command + ' ' + resultName, 'warn');
                }
                msg._resultName = resultName;
                events.emit('command_ack', msg);
                break;
            }

            case 'mission_count':
                v.missionCount = msg.count;
                events.emit('mission_count', msg);
                break;

            case 'mission_item':
                v.missionItems[msg.seq] = msg;
                events.emit('mission_item', msg);
                break;

            case 'mission_ack':
                events.emit('mission_ack', msg);
                break;

            // T1-1: Mission request from vehicle during upload
            case 'mission_request_int':
                events.emit('mission_request_int', msg);
                break;

            // T1-27: HOME_POSITION
            case 'home_position':
                v.homeLat = msg.lat;
                v.homeLon = msg.lon;
                v.homeAlt = msg.alt;
                events.emit('home_changed', v);
                break;

            // T1-27: BATTERY_STATUS (detailed)
            case 'battery_status':
                v.cellVoltages = msg.voltages || [];
                if (msg.current_battery) v.current = msg.current_battery;
                if (msg.battery_remaining >= 0) v.batteryPct = msg.battery_remaining;
                if (msg.current_consumed > 0) v.mah = msg.current_consumed;
                v.lastBattery = now;
                events.emit('battery', v);
                break;

            // T1-27: FENCE_STATUS
            case 'fence_status':
                v.fenceBreach = msg.breach_status !== 0;
                v.fenceBreachCount = msg.breach_count;
                v.fenceBreachType = msg.breach_type;
                events.emit('fence_status', msg);
                if (v.fenceBreach && !v._fenceLogged) {
                    meridian.log('GEOFENCE BREACH (type ' + msg.breach_type + ')', 'error');
                    events.emit('failsafe', { type: 'fence_breach', breach_type: msg.breach_type });
                    v._fenceLogged = true;
                }
                if (!v.fenceBreach) v._fenceLogged = false;
                break;

            // T1-27: VIBRATION
            case 'vibration':
                v.vibrationX = msg.vibration_x;
                v.vibrationY = msg.vibration_y;
                v.vibrationZ = msg.vibration_z;
                v.clipping0 = msg.clipping_0;
                v.clipping1 = msg.clipping_1;
                v.clipping2 = msg.clipping_2;
                v.imuVibe = Math.max(msg.vibration_x, msg.vibration_y, msg.vibration_z);
                events.emit('vibration', v);
                break;

            // T1-27: AUTOPILOT_VERSION
            case 'autopilot_version':
                v.autopilotVersion = msg;
                events.emit('autopilot_version', msg);
                break;

            // T1-5: ADSB_VEHICLE
            case 'adsb_vehicle':
                meridian.adsb[msg.ICAO_address] = {
                    lat: msg.lat,
                    lon: msg.lon,
                    alt: msg.altitude,
                    heading: msg.heading,
                    hor_velocity: msg.hor_velocity,
                    ver_velocity: msg.ver_velocity,
                    callsign: msg.callsign,
                    squawk: msg.squawk,
                    lastSeen: now,
                };
                events.emit('adsb_vehicle', msg);
                break;

            // T1-12: OPEN_DRONE_ID_BASIC_ID
            case 'open_drone_id_basic_id':
                v.remoteId.uasId = msg.uas_id;
                v.remoteId.uasType = msg.ua_type;
                events.emit('remote_id', v.remoteId);
                break;

            // T1-12: OPEN_DRONE_ID_LOCATION
            case 'open_drone_id_location':
                v.remoteId.lat = msg.lat;
                v.remoteId.lon = msg.lon;
                v.remoteId.alt = msg.altitude_geo || msg.altitude_baro;
                events.emit('remote_id', v.remoteId);
                break;

            // T1-12: OPEN_DRONE_ID_SYSTEM
            case 'open_drone_id_system':
                v.remoteId.operatorId = msg.operator_id;
                events.emit('remote_id', v.remoteId);
                break;

            case 'terrain_report': {
                // MAVLink TERRAIN_REPORT (136) — terrain elevation below vehicle
                v.terrainAlt = msg.terrain_height;
                v.terrainClearance = v.relativeAlt - (msg.terrain_height - (v.alt - v.relativeAlt));
                events.emit('terrain', v);
                break;
            }

            case 'servo_output_raw': {
                // MAVLink SERVO_OUTPUT_RAW (36) — PWM outputs per channel
                v.servoOutputs = [];
                for (var ch = 1; ch <= 16; ch++) {
                    var val = msg['servo' + ch + '_raw'];
                    if (val !== undefined) v.servoOutputs.push(val);
                }
                break;
            }

            case 'statustext': {
                // MAVLink STATUSTEXT — FC's own messages (pre-arm, EKF, failsafe reasons)
                const sevMap = ['EMERGENCY','ALERT','CRITICAL','ERROR','WARNING','NOTICE','INFO','DEBUG'];
                const level = msg.severity <= 3 ? 'error' : (msg.severity <= 4 ? 'warn' : 'info');
                const prefix = sevMap[msg.severity] || 'INFO';
                meridian.log('[' + prefix + '] ' + msg.text, level);
                // Track arm-denied reason
                if (msg.text && (msg.text.startsWith('PreArm') || msg.text.startsWith('Arm'))) {
                    v.armDeniedReason = msg.text;
                }
                events.emit('statustext', msg);
                break;
            }
        }

        events.emit('telemetry', v);
    }

    meridian.handleMessage = handleMessage;

    // --- Mode name lookup ---
    const COPTER_MODES = {
        0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO',
        4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 7: 'CIRCLE',
        9: 'LAND', 11: 'DRIFT', 13: 'SPORT', 14: 'FLIP',
        15: 'AUTOTUNE', 16: 'POSHOLD', 17: 'BRAKE', 18: 'THROW',
        19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL',
        22: 'FLOWHOLD', 23: 'FOLLOW', 24: 'ZIGZAG', 25: 'SYSTEMID',
        26: 'AUTOROTATE', 27: 'AUTO_RTL',
    };

    meridian.COPTER_MODES = COPTER_MODES;
    meridian.getModeNameByNum = function (num) {
        return COPTER_MODES[num] || ('MODE_' + num);
    };

    // Expose globally
    window.meridian = meridian;

})();
