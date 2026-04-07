/* ============================================================
   mnp.js — Meridian Native Protocol (MNP) Codec
   COBS framing + binary message serialization compatible
   with Rust postcard/serde format used by Meridian FC.
   ============================================================ */

'use strict';

window.MNP = (function () {

    // ---- Message IDs (match Meridian FC enum variants) ----
    const MSG = {
        // Telemetry (FC → GCS)
        HEARTBEAT:   0x01,
        ATTITUDE:    0x02,
        POSITION:    0x03,
        BATTERY:     0x04,
        GPS_RAW:     0x05,
        VFR_HUD:     0x06,
        EKF_STATUS:  0x07,
        RC_CHANNELS: 0x08,
        PARAM_VALUE: 0x09,
        MISSION_ACK: 0x0A,

        // Commands (GCS → FC)
        CMD_ARM:       0x80,
        CMD_DISARM:    0x81,
        CMD_TAKEOFF:   0x82,
        CMD_LAND:      0x83,
        CMD_RTL:       0x84,
        CMD_SET_MODE:  0x85,
        CMD_GOTO:      0x86,
        CMD_SET_PARAM: 0x87,
        CMD_GET_PARAM: 0x88,
        CMD_MISSION_ITEM: 0x89,
        CMD_MISSION_COUNT: 0x8A,
        CMD_MISSION_REQ:   0x8B,
    };

    // Flight modes (index matches FC enum)
    const MODES = [
        'STABILIZE', 'ALT_HOLD', 'LOITER', 'RTL',
        'AUTO', 'LAND', 'GUIDED', 'ACRO',
    ];

    // ---- COBS Encode / Decode ----

    function cobsEncode(data) {
        const out = [];
        let codeIdx = 0;
        let code = 1;
        out.push(0); // placeholder for first code byte

        for (let i = 0; i < data.length; i++) {
            if (data[i] === 0) {
                out[codeIdx] = code;
                codeIdx = out.length;
                out.push(0); // placeholder
                code = 1;
            } else {
                out.push(data[i]);
                code++;
                if (code === 0xFF) {
                    out[codeIdx] = code;
                    codeIdx = out.length;
                    out.push(0);
                    code = 1;
                }
            }
        }
        out[codeIdx] = code;
        out.push(0); // frame delimiter
        return new Uint8Array(out);
    }

    function cobsDecode(data) {
        const out = [];
        let i = 0;
        while (i < data.length) {
            let code = data[i++];
            if (code === 0) break; // end of frame
            for (let j = 1; j < code && i < data.length; j++) {
                out.push(data[i++]);
            }
            if (code < 0xFF && i < data.length) {
                out.push(0);
            }
        }
        // COBS decode naturally adds a trailing zero from the last code byte.
        // Remove it only if the last code was < 0xFF (meaning a zero was appended).
        // Fixed: previous version unconditionally removed trailing 0x00 which could
        // corrupt messages whose last data byte is legitimately 0x00.
        return new Uint8Array(out);
    }

    // ---- Binary Reader (little-endian, postcard-compatible) ----

    class BinaryReader {
        constructor(buf) {
            this.view = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
            this.pos = 0;
        }
        u8()  { const v = this.view.getUint8(this.pos); this.pos += 1; return v; }
        u16() { const v = this.view.getUint16(this.pos, true); this.pos += 2; return v; }
        u32() { const v = this.view.getUint32(this.pos, true); this.pos += 4; return v; }
        i16() { const v = this.view.getInt16(this.pos, true); this.pos += 2; return v; }
        i32() { const v = this.view.getInt32(this.pos, true); this.pos += 4; return v; }
        f32() { const v = this.view.getFloat32(this.pos, true); this.pos += 4; return v; }
        remaining() { return this.view.byteLength - this.pos; }
        // Postcard varint (LEB128)
        varint() {
            let result = 0, shift = 0;
            while (this.pos < this.view.byteLength) {
                const b = this.view.getUint8(this.pos++);
                result |= (b & 0x7F) << shift;
                if ((b & 0x80) === 0) break;
                shift += 7;
            }
            return result;
        }
        str() {
            const len = this.varint();
            const bytes = new Uint8Array(this.view.buffer, this.view.byteOffset + this.pos, len);
            this.pos += len;
            return new TextDecoder().decode(bytes);
        }
    }

    // ---- Binary Writer ----

    class BinaryWriter {
        constructor() { this.buf = []; }
        u8(v)  { this.buf.push(v & 0xFF); }
        u16(v) { this.buf.push(v & 0xFF, (v >> 8) & 0xFF); }
        u32(v) { this.buf.push(v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF); }
        i32(v) { this.u32(v); }
        f32(v) {
            const ab = new ArrayBuffer(4);
            new DataView(ab).setFloat32(0, v, true);
            const bytes = new Uint8Array(ab);
            for (let i = 0; i < 4; i++) this.buf.push(bytes[i]);
        }
        bytes() { return new Uint8Array(this.buf); }
    }

    // ---- Parsers (FC → GCS) ----

    // Vehicle class IDs
    var VEHICLE_CLASSES = ['copter', 'plane', 'rover', 'boat', 'sub'];

    function parseHeartbeat(r) {
        var armed = r.u8() !== 0;
        var modeIdx = r.u8();
        var status = r.u8();
        // Vehicle class: encoded in top 4 bits of status byte
        var vehicleClass = VEHICLE_CLASSES[(status >> 4) & 0x0F] || 'copter';
        return {
            type: 'heartbeat',
            armed: armed,
            mode: MODES[modeIdx] || 'UNKNOWN',
            system_status: status & 0x0F,
            vehicle_class: vehicleClass,
        };
    }

    function parseAttitude(r) {
        return {
            type: 'attitude',
            roll:  r.f32(),   // radians
            pitch: r.f32(),
            yaw:   r.f32(),
            rollspeed:  r.f32(),
            pitchspeed: r.f32(),
            yawspeed:   r.f32(),
        };
    }

    function parsePosition(r) {
        return {
            type: 'position',
            lat: r.i32() / 1e7,    // degE7
            lon: r.i32() / 1e7,
            alt: r.i32() / 1000,    // mm → m
            relative_alt: r.i32() / 1000,
            vx: r.i16() / 100,     // cm/s → m/s
            vy: r.i16() / 100,
            vz: r.i16() / 100,
            hdg: r.u16() / 100,    // cdeg → deg
        };
    }

    function parseBattery(r) {
        return {
            type: 'battery',
            voltage: r.u16() / 1000,  // mV → V
            current: r.i16() / 100,   // cA → A
            remaining: r.u8(),         // %
        };
    }

    function parseGpsRaw(r) {
        return {
            type: 'gps_raw',
            fix_type: r.u8(),
            lat: r.i32() / 1e7,
            lon: r.i32() / 1e7,
            alt: r.i32() / 1000,
            eph: r.u16(),
            epv: r.u16(),
            satellites: r.u8(),
        };
    }

    function parseVfrHud(r) {
        return {
            type: 'vfr_hud',
            airspeed:    r.f32(),
            groundspeed: r.f32(),
            heading:     r.i16(),
            throttle:    r.u16(),
            alt:         r.f32(),
            climb:       r.f32(),
        };
    }

    function parseEkfStatus(r) {
        return {
            type: 'ekf_status',
            velocity_variance:  r.f32(),
            pos_horiz_variance: r.f32(),
            pos_vert_variance:  r.f32(),
            compass_variance:   r.f32(),
            terrain_variance:   r.f32(),
            flags: r.u16(),
        };
    }

    function parseRcChannels(r) {
        const count = r.u8();
        const channels = [];
        for (let i = 0; i < count && r.remaining() >= 2; i++) {
            channels.push(r.u16());
        }
        return { type: 'rc_channels', rssi: r.remaining() >= 1 ? r.u8() : 255, channels };
    }

    function parseParamValue(r) {
        return {
            type: 'param_value',
            name: r.str(),
            value: r.f32(),
            param_type: r.u8(),
            param_count: r.u16(),
            param_index: r.u16(),
        };
    }

    function parseMissionAck(r) {
        return { type: 'mission_ack', result: r.u8() };
    }

    // ---- Decode one MNP frame ----

    function decode(cobsFrame) {
        const payload = cobsDecode(cobsFrame);
        if (payload.length < 1) return null;

        const r = new BinaryReader(payload);
        const msgId = r.u8();

        try {
            switch (msgId) {
                case MSG.HEARTBEAT:   return parseHeartbeat(r);
                case MSG.ATTITUDE:    return parseAttitude(r);
                case MSG.POSITION:    return parsePosition(r);
                case MSG.BATTERY:     return parseBattery(r);
                case MSG.GPS_RAW:     return parseGpsRaw(r);
                case MSG.VFR_HUD:     return parseVfrHud(r);
                case MSG.EKF_STATUS:  return parseEkfStatus(r);
                case MSG.RC_CHANNELS: return parseRcChannels(r);
                case MSG.PARAM_VALUE: return parseParamValue(r);
                case MSG.MISSION_ACK: return parseMissionAck(r);
                default:
                    return { type: 'unknown', id: msgId };
            }
        } catch (e) {
            console.warn('MNP parse error for msg 0x' + msgId.toString(16), e);
            return null;
        }
    }

    // ---- Encoders (GCS → FC) ----

    function encodeArm() {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_ARM);
        return cobsEncode(w.bytes());
    }

    function encodeDisarm() {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_DISARM);
        return cobsEncode(w.bytes());
    }

    function encodeTakeoff(altMeters) {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_TAKEOFF);
        w.f32(altMeters);
        return cobsEncode(w.bytes());
    }

    function encodeLand() {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_LAND);
        return cobsEncode(w.bytes());
    }

    function encodeRtl() {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_RTL);
        return cobsEncode(w.bytes());
    }

    function encodeSetMode(modeIndex) {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_SET_MODE);
        w.u8(modeIndex);
        return cobsEncode(w.bytes());
    }

    function encodeGoto(lat, lon, alt) {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_GOTO);
        w.i32(Math.round(lat * 1e7));
        w.i32(Math.round(lon * 1e7));
        w.f32(alt);
        return cobsEncode(w.bytes());
    }

    function encodeSetParam(name, value) {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_SET_PARAM);
        const nameBytes = new TextEncoder().encode(name);
        // Postcard-style length-prefixed string
        let len = nameBytes.length;
        while (len >= 0x80) { w.u8((len & 0x7F) | 0x80); len >>= 7; }
        w.u8(len & 0x7F);
        for (const b of nameBytes) w.u8(b);
        w.f32(value);
        return cobsEncode(w.bytes());
    }

    function encodeGetParam(name) {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_GET_PARAM);
        const nameBytes = new TextEncoder().encode(name);
        let len = nameBytes.length;
        while (len >= 0x80) { w.u8((len & 0x7F) | 0x80); len >>= 7; }
        w.u8(len & 0x7F);
        for (const b of nameBytes) w.u8(b);
        return cobsEncode(w.bytes());
    }

    function encodeMissionItem(seq, lat, lon, alt) {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_MISSION_ITEM);
        w.u16(seq);
        w.i32(Math.round(lat * 1e7));
        w.i32(Math.round(lon * 1e7));
        w.f32(alt);
        return cobsEncode(w.bytes());
    }

    function encodeMissionCount(count) {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_MISSION_COUNT);
        w.u16(count);
        return cobsEncode(w.bytes());
    }

    function encodeMissionRequest() {
        const w = new BinaryWriter();
        w.u8(MSG.CMD_MISSION_REQ);
        return cobsEncode(w.bytes());
    }

    // ---- Public API ----

    return {
        MSG, MODES,
        cobsEncode, cobsDecode,
        decode,
        encodeArm, encodeDisarm,
        encodeTakeoff, encodeLand, encodeRtl,
        encodeSetMode, encodeGoto,
        encodeSetParam, encodeGetParam,
        encodeMissionItem, encodeMissionCount, encodeMissionRequest,
    };

})();
