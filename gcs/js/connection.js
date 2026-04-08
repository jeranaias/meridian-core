/* ============================================================
   connection.js — WebSocket / TCP Connection Manager
   Handles MNP (native) and MAVLink (SITL) protocols.
   ============================================================ */

'use strict';

window.Connection = (function () {

    const STATE = { DISCONNECTED: 0, CONNECTING: 1, CONNECTED: 2 };

    let ws = null;
    let state = STATE.DISCONNECTED;
    let protocol = 'mnp'; // 'mnp' (Meridian Native) or 'mavlink' (legacy)
    let reconnectTimer = null;
    let heartbeatTimer = null;
    let targetUrl = '';
    let mnpBuffer = new Uint8Array(4096);
    let mnpBufLen = 0;
    let mavParser = null;

    // T1-3: Pending command tracking for ACK matching
    const pendingCommands = new Map(); // key = command ID, value = { time, name }
    const ACK_TIMEOUT_MS = 3000;

    // Callbacks
    let onMessage = null;
    let onStateChange = null;

    function setState(s) {
        state = s;
        if (onStateChange) onStateChange(s);
        const M = window.meridian;
        if (M) M.connectionState = s;
    }

    function log(msg, level) {
        if (window.meridian && window.meridian.log) {
            window.meridian.log(msg, level || 'info');
        }
    }

    function connect(url, proto) {
        if (ws) disconnect();

        protocol = proto || 'mnp';
        targetUrl = url;

        // For TCP connections (tcp://host:port), we use WebSocket proxy or direct WS
        // SITL typically needs a ws:// bridge. User provides ws:// URL.
        let wsUrl = url;
        if (url.startsWith('tcp://')) {
            // Convert tcp:// to ws:// (assume user has a ws bridge or we connect directly)
            wsUrl = url.replace('tcp://', 'ws://');
        }

        setState(STATE.CONNECTING);
        log('Connecting to ' + wsUrl + ' (' + protocol + ')...');

        try {
            ws = new WebSocket(wsUrl);
            ws.binaryType = 'arraybuffer';
        } catch (e) {
            log('Connection failed: ' + e.message, 'error');
            setState(STATE.DISCONNECTED);
            scheduleReconnect();
            return;
        }

        if (protocol === 'mavlink') {
            mavParser = new MAVLink.FrameParser();
        }

        ws.onopen = function () {
            setState(STATE.CONNECTED);
            log('Connected to ' + wsUrl, 'info');
            clearReconnectTimer();

            // Start GCS heartbeat for MAVLink (1Hz)
            if (protocol === 'mavlink') {
                startHeartbeat();
            }
        };

        ws.onmessage = function (evt) {
            if (!(evt.data instanceof ArrayBuffer)) return;
            const data = new Uint8Array(evt.data);

            // Record raw bytes to IndexedDB (Oborne)
            if (window.Tlog) Tlog.recordBytes(data);

            if (protocol === 'mnp') {
                handleMnpData(data);
            } else {
                handleMavlinkData(data);
            }
        };

        ws.onerror = function (evt) {
            log('Connection error', 'error');
        };

        ws.onclose = function () {
            setState(STATE.DISCONNECTED);
            log('Disconnected', 'warn');
            stopHeartbeat();
            scheduleReconnect();
        };
    }

    function disconnect() {
        clearReconnectTimer();
        stopHeartbeat();
        if (ws) {
            ws.onclose = null; // Prevent reconnect
            ws.close();
            ws = null;
        }
        setState(STATE.DISCONNECTED);
    }

    function send(data) {
        if (!ws || ws.readyState !== WebSocket.OPEN) return false;
        ws.send(data.buffer ? data.buffer : data);
        return true;
    }

    // T1-3: Track a sent command for ACK matching with timeout
    function trackCommand(commandId, commandName) {
        pendingCommands.set(commandId, { time: Date.now(), name: commandName || ('CMD_' + commandId) });
        setTimeout(function () {
            if (pendingCommands.has(commandId)) {
                pendingCommands.delete(commandId);
                log('Command ' + (commandName || commandId) + ' — no ACK received (timeout)', 'warn');
            }
        }, ACK_TIMEOUT_MS);
    }

    // T1-3: Handle incoming COMMAND_ACK and resolve pending
    function handleCommandAck(msg) {
        if (!msg || msg.command === undefined) return;
        if (pendingCommands.has(msg.command)) {
            pendingCommands.delete(msg.command);
        }
    }

    // ---- MNP Framing ----

    function handleMnpData(data) {
        // T0-13: Cap at 64KB to prevent unbounded growth
        if (mnpBufLen + data.length > 65536) {
            meridian.log('MNP buffer overflow — resetting', 'warn');
            mnpBufLen = 0;
            return;
        }
        if (mnpBufLen + data.length > mnpBuffer.length) {
            const newBuf = new Uint8Array(Math.min(Math.max(mnpBuffer.length * 2, mnpBufLen + data.length), 65536));
            newBuf.set(mnpBuffer.subarray(0, mnpBufLen));
            mnpBuffer = newBuf;
        }
        mnpBuffer.set(data, mnpBufLen);
        mnpBufLen += data.length;

        // Extract COBS frames (delimited by 0x00)
        while (true) {
            let zeroIdx = -1;
            for (let i = 0; i < mnpBufLen; i++) {
                if (mnpBuffer[i] === 0x00) { zeroIdx = i; break; }
            }
            if (zeroIdx < 0) break;

            if (zeroIdx > 0) {
                const frame = mnpBuffer.slice(0, zeroIdx);
                const msg = MNP.decode(frame);
                if (msg && onMessage) onMessage(msg);
            }

            // Consume frame + delimiter
            mnpBuffer.copyWithin(0, zeroIdx + 1, mnpBufLen);
            mnpBufLen -= (zeroIdx + 1);
        }
    }

    // ---- MAVLink Framing ----

    function handleMavlinkData(data) {
        mavParser.push(data);
        const messages = mavParser.extract();
        for (const msg of messages) {
            if (onMessage) onMessage(msg);
        }
    }

    // ---- GCS Heartbeat (MAVLink) ----

    function startHeartbeat() {
        stopHeartbeat();
        heartbeatTimer = setInterval(function () {
            send(MAVLink.encodeGcsHeartbeat());
        }, 1000);
    }

    function stopHeartbeat() {
        if (heartbeatTimer) {
            clearInterval(heartbeatTimer);
            heartbeatTimer = null;
        }
    }

    // ---- Auto-reconnect ----

    function scheduleReconnect() {
        clearReconnectTimer();
        if (!targetUrl) return;
        reconnectTimer = setTimeout(function () {
            if (state === STATE.DISCONNECTED && targetUrl) {
                log('Reconnecting...', 'info');
                connect(targetUrl, protocol);
            }
        }, 3000);
    }

    function clearReconnectTimer() {
        if (reconnectTimer) {
            clearTimeout(reconnectTimer);
            reconnectTimer = null;
        }
    }

    // ---- T1-1: Mission Upload/Download Protocol State Machine ----

    const MISSION_TIMEOUT = 3000;
    const MISSION_MAX_RETRIES = 3;

    // Upload state
    let uploadState = null; // { waypoints, retries, timer, listener }

    function startMissionUpload(waypoints) {
        cancelMissionTransfer();
        uploadState = {
            waypoints: waypoints,
            retries: 0,
            timer: null,
            complete: false,
        };

        // Listen for MISSION_REQUEST_INT from vehicle
        function onMissionRequest(msg) {
            if (!uploadState || uploadState.complete) return;
            clearTimeout(uploadState.timer);
            var seq = msg.seq;
            if (seq >= 0 && seq < uploadState.waypoints.length) {
                var wp = uploadState.waypoints[seq];
                send(MAVLink.encodeMissionItemIntFull(wp));
                uploadState.retries = 0;
                // Set timeout for next request or ACK
                uploadState.timer = setTimeout(function () { retryUpload(); }, MISSION_TIMEOUT);
            }
        }

        function onMissionAck(msg) {
            if (!uploadState) return;
            clearTimeout(uploadState.timer);
            uploadState.complete = true;
            meridian.events.off('mission_request_int', onMissionRequest);
            meridian.events.off('mission_ack', onMissionAck);
            if (msg.result === 0) {
                log('Mission upload complete (' + uploadState.waypoints.length + ' items)', 'info');
            } else {
                log('Mission upload rejected (error ' + msg.result + ')', 'error');
            }
            uploadState = null;
        }

        function retryUpload() {
            if (!uploadState || uploadState.complete) return;
            uploadState.retries++;
            if (uploadState.retries > MISSION_MAX_RETRIES) {
                log('Mission upload failed — no response after ' + MISSION_MAX_RETRIES + ' retries', 'error');
                meridian.events.off('mission_request_int', onMissionRequest);
                meridian.events.off('mission_ack', onMissionAck);
                uploadState = null;
                return;
            }
            log('Mission upload retry ' + uploadState.retries + '/' + MISSION_MAX_RETRIES, 'warn');
            send(MAVLink.encodeMissionCount(uploadState.waypoints.length));
            uploadState.timer = setTimeout(function () { retryUpload(); }, MISSION_TIMEOUT);
        }

        meridian.events.on('mission_request_int', onMissionRequest);
        meridian.events.on('mission_ack', onMissionAck);

        // Send MISSION_COUNT to start
        send(MAVLink.encodeMissionCount(waypoints.length));
        uploadState.timer = setTimeout(function () { retryUpload(); }, MISSION_TIMEOUT);
    }

    // Download state
    let downloadState = null;

    function startMissionDownload() {
        cancelMissionTransfer();
        downloadState = {
            expected: 0,
            items: [],
            retries: 0,
            timer: null,
            nextSeq: 0,
        };

        function onMissionCount(msg) {
            if (!downloadState) return;
            clearTimeout(downloadState.timer);
            if (msg.count === 0) {
                log('Vehicle has no mission', 'info');
                cleanup();
                return;
            }
            downloadState.expected = msg.count;
            downloadState.items = new Array(msg.count);
            downloadState.nextSeq = 0;
            downloadState.retries = 0;
            log('Downloading ' + msg.count + ' mission items...', 'info');
            requestNextItem();
        }

        function onMissionItem(msg) {
            if (!downloadState) return;
            clearTimeout(downloadState.timer);
            downloadState.items[msg.seq] = msg;
            downloadState.retries = 0;
            downloadState.nextSeq = msg.seq + 1;

            if (downloadState.nextSeq >= downloadState.expected) {
                // All items received — send MISSION_ACK and populate
                var ackBuf = new ArrayBuffer(3);
                var ackDv = new DataView(ackBuf);
                ackDv.setUint8(0, 1); // target_system
                ackDv.setUint8(1, 1); // target_component
                ackDv.setUint8(2, 0); // MAV_MISSION_ACCEPTED
                send(MAVLink.buildFrame(47, new Uint8Array(ackBuf)));

                if (window.Mission) Mission.setItems(downloadState.items);
                log('Mission downloaded (' + downloadState.expected + ' items)', 'info');
                cleanup();
            } else {
                requestNextItem();
            }
        }

        function requestNextItem() {
            if (!downloadState) return;
            send(MAVLink.encodeMissionRequestInt(downloadState.nextSeq));
            downloadState.timer = setTimeout(function () { retryDownload(); }, MISSION_TIMEOUT);
        }

        function retryDownload() {
            if (!downloadState) return;
            downloadState.retries++;
            if (downloadState.retries > MISSION_MAX_RETRIES) {
                log('Mission download failed — no response after ' + MISSION_MAX_RETRIES + ' retries', 'error');
                cleanup();
                return;
            }
            log('Mission download retry ' + downloadState.retries + '/' + MISSION_MAX_RETRIES, 'warn');
            if (downloadState.expected === 0) {
                // Still waiting for count
                send(MAVLink.encodeMissionRequestList());
            } else {
                requestNextItem();
            }
            downloadState.timer = setTimeout(function () { retryDownload(); }, MISSION_TIMEOUT);
        }

        function cleanup() {
            meridian.events.off('mission_count', onMissionCount);
            meridian.events.off('mission_item', onMissionItem);
            if (downloadState && downloadState.timer) clearTimeout(downloadState.timer);
            downloadState = null;
        }

        meridian.events.on('mission_count', onMissionCount);
        meridian.events.on('mission_item', onMissionItem);

        send(MAVLink.encodeMissionRequestList());
        downloadState.timer = setTimeout(function () { retryDownload(); }, MISSION_TIMEOUT);
    }

    function cancelMissionTransfer() {
        if (uploadState) {
            clearTimeout(uploadState.timer);
            uploadState = null;
        }
        if (downloadState) {
            clearTimeout(downloadState.timer);
            downloadState = null;
        }
    }

    // ================================================================
    // T3-9: Multi-connection pool — simultaneous WebSocket per vehicle
    // Each connection keyed by sysid or URL.
    // ================================================================

    // Map: sysid -> { ws, url, protocol, mavParser, linkQuality, lastHeartbeat, sysid }
    const pool = new Map();

    function addConnection(url, proto, sysid) {
        if (pool.has(sysid)) {
            log('Pool: connection for sysid ' + sysid + ' already exists — replacing', 'warn');
            removeConnection(sysid);
        }

        let wsUrl = url;
        if (url.startsWith('tcp://')) wsUrl = url.replace('tcp://', 'ws://');

        let entry = {
            ws: null,
            url: wsUrl,
            protocol: proto || 'mnp',
            mavParser: null,
            linkQuality: -1,
            lastHeartbeat: 0,
            sysid: sysid,
            state: STATE.CONNECTING,
        };

        try {
            entry.ws = new WebSocket(wsUrl);
            entry.ws.binaryType = 'arraybuffer';
        } catch (e) {
            log('Pool: connection to ' + wsUrl + ' failed: ' + e.message, 'error');
            return;
        }

        if (entry.protocol === 'mavlink') {
            entry.mavParser = new MAVLink.FrameParser();
        }

        entry.ws.onopen = function () {
            entry.state = STATE.CONNECTED;
            log('Pool: sysid ' + sysid + ' connected to ' + wsUrl, 'info');
            updatePoolStatusDisplay();
        };

        entry.ws.onmessage = function (evt) {
            if (!(evt.data instanceof ArrayBuffer)) return;
            var data = new Uint8Array(evt.data);
            if (window.Tlog) Tlog.recordBytes(data);

            if (entry.protocol === 'mavlink' && entry.mavParser) {
                entry.mavParser.push(data);
                var msgs = entry.mavParser.extract();
                for (var i = 0; i < msgs.length; i++) {
                    var msg = msgs[i];
                    // Track link quality from RC_CHANNELS_RAW or RADIO_STATUS
                    if (msg.type === 'radio_status' || msg.type === 'rc_channels_raw') {
                        if (msg.rssi != null) entry.linkQuality = msg.rssi;
                    }
                    if (msg.type === 'heartbeat') entry.lastHeartbeat = Date.now();
                    if (onMessage) onMessage(msg);
                }
            }
        };

        entry.ws.onerror = function () {
            log('Pool: sysid ' + sysid + ' connection error', 'error');
        };

        entry.ws.onclose = function () {
            entry.state = STATE.DISCONNECTED;
            log('Pool: sysid ' + sysid + ' disconnected', 'warn');
            updatePoolStatusDisplay();
        };

        pool.set(sysid, entry);
        updatePoolStatusDisplay();
        log('Pool: opened connection for sysid ' + sysid + ' \u2192 ' + wsUrl + ' (' + (proto || 'mnp') + ')', 'info');
    }

    function removeConnection(sysid) {
        var entry = pool.get(sysid);
        if (!entry) return;
        if (entry.ws) {
            entry.ws.onclose = null;
            entry.ws.close();
        }
        pool.delete(sysid);
        updatePoolStatusDisplay();
        log('Pool: closed connection for sysid ' + sysid, 'info');
    }

    function sendTo(sysid, data) {
        var entry = pool.get(sysid);
        if (!entry || !entry.ws || entry.ws.readyState !== WebSocket.OPEN) return false;
        entry.ws.send(data.buffer ? data.buffer : data);
        return true;
    }

    function getPoolStatus() {
        var statuses = [];
        pool.forEach(function (entry) {
            statuses.push({
                sysid: entry.sysid,
                url: entry.url,
                state: entry.state,
                linkQuality: entry.linkQuality,
                lastHeartbeat: entry.lastHeartbeat,
            });
        });
        return statuses;
    }

    function updatePoolStatusDisplay() {
        var el = document.getElementById('pool-status');
        if (!el) return;
        var statuses = getPoolStatus();
        if (statuses.length === 0) { el.style.display = 'none'; return; }
        el.style.display = 'flex';
        el.innerHTML = statuses.map(function (s) {
            var cls = s.state === STATE.CONNECTED ? 'pool-connected' :
                      s.state === STATE.CONNECTING ? 'pool-connecting' : 'pool-disconnected';
            var lq = s.linkQuality >= 0 ? ' ' + s.linkQuality + '%' : '';
            return '<span class="pool-entry ' + cls + '" title="sysid ' + s.sysid + ' — ' + s.url + '">' +
                   'V' + s.sysid + lq +
                   '</span>';
        }).join('');
    }

    // ---- Public API ----

    return {
        STATE,
        connect,
        disconnect,
        send,

        get state() { return state; },
        get protocol() { return protocol; },

        set onMessage(fn) { onMessage = fn; },
        set onStateChange(fn) { onStateChange = fn; },

        // T3-9: Connection pool
        addConnection,
        removeConnection,
        sendTo,
        getPoolStatus,

        // T1-3: Expose pending command tracking
        trackCommand,
        handleCommandAck,

        // Convenience send for current protocol
        sendArm() {
            if (protocol === 'mavlink') {
                trackCommand(400, 'ARM');
                return send(MAVLink.encodeArm());
            }
            return send(MNP.encodeArm());
        },
        sendDisarm() {
            if (protocol === 'mavlink') {
                trackCommand(400, 'DISARM');
                return send(MAVLink.encodeDisarm());
            }
            return send(MNP.encodeDisarm());
        },
        sendTakeoff(alt) {
            if (protocol === 'mavlink') {
                trackCommand(22, 'TAKEOFF');
                return send(MAVLink.encodeTakeoff(alt));
            }
            return send(MNP.encodeTakeoff(alt));
        },
        sendLand() {
            if (protocol === 'mavlink') {
                trackCommand(21, 'LAND');
                return send(MAVLink.encodeLand());
            }
            return send(MNP.encodeLand());
        },
        sendRtl() {
            if (protocol === 'mavlink') {
                trackCommand(20, 'RTL');
                return send(MAVLink.encodeRtl());
            }
            return send(MNP.encodeRtl());
        },
        sendSetMode(mode) {
            if (protocol === 'mavlink') {
                trackCommand(176, 'SET_MODE');
                return send(MAVLink.encodeSetMode(mode));
            }
            // mode can be a number (mode index) or string (mode name)
            var idx;
            if (typeof mode === 'number') {
                idx = mode;
            } else {
                idx = MNP.MODES.indexOf(String(mode).toUpperCase());
                if (idx < 0) idx = 0;
            }
            return send(MNP.encodeSetMode(idx));
        },
        sendGoto(lat, lon, alt) {
            if (protocol === 'mavlink') return send(MAVLink.encodeSetPositionTargetGlobalInt(lat, lon, alt));
            return send(MNP.encodeGoto(lat, lon, alt));
        },
        sendParamRequestList() {
            if (protocol === 'mavlink') return send(MAVLink.encodeParamRequestList());
            return false; // MNP: iterate with get_param
        },
        sendParamSet(name, value) {
            if (protocol === 'mavlink') return send(MAVLink.encodeParamSet(name, value));
            return send(MNP.encodeSetParam(name, value));
        },
        sendMissionRequestList() {
            if (protocol === 'mavlink') {
                startMissionDownload();
                return true;
            }
            return send(MNP.encodeMissionRequest());
        },
        sendMissionUpload(waypoints) {
            if (protocol === 'mavlink') {
                startMissionUpload(waypoints);
                return true;
            }
            send(MNP.encodeMissionCount(waypoints.length));
            waypoints.forEach(function (wp, i) {
                send(MNP.encodeMissionItem(i, wp.lat, wp.lon, wp.alt));
            });
            return true;
        },
    };

})();
