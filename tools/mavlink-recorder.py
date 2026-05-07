#!/usr/bin/env python3
"""
mavlink-recorder.py — Persistent MAVLink data recorder for the Vanguard USV.

Subscribes to the running mavlink-ws-bridge.py and writes every frame to
disk. Designed to run as a Windows Scheduled Task on the USV laptop so
every flight session — bench tests, tank tests, water trials — produces
a complete record we can analyze offline.

Outputs (under --log-dir, default `C:\\Users\\vangu\\Documents\\Vanguard-Backups\\flightlogs`):

  <run_id>.tlog
      Standard MAVLink telemetry log: sequence of
        (uint64 big-endian timestamp_us, raw MAVLink v2 frame).
      Mission Planner / MAVExplorer / pymavlink can replay it directly.

  <run_id>.jsonl
      Decoded structured events for easy programmatic analysis:
        {"t": 1715040000.123, "msgid": 0, "name": "HEARTBEAT",
         "armed": false, "mode_num": 4, "system_status": 5}
      One line per "interesting" message. Rotated alongside the .tlog.

  <run_id>.summary.json
      Written at file rotation: counts, time range, peaks.

Rotation: every 60 min OR every 100 MB OR on Ctrl-C / signal.

Usage:
    python tools/mavlink-recorder.py
    python tools/mavlink-recorder.py --ws ws://localhost:5760 --log-dir C:\\path\\to\\logs

Requires:
    pip install websockets
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import signal
import struct
import sys
import time
from pathlib import Path
from typing import Optional

# Reconfigure stdout/stderr to UTF-8 so any incidental non-ASCII chars
# in print() output don't crash the process when running under
# Start-Process with redirected output (Windows defaults to cp1252).
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

try:
    import websockets
except ImportError:
    print("ERROR: pip install websockets", file=sys.stderr)
    sys.exit(2)


# ────────────────────────────────────────────────────────────
#  MAVLink message decoders — only the ones we want to capture
#  in JSONL. Raw frames go to .tlog regardless.
# ────────────────────────────────────────────────────────────

MAV_MODE_FLAG_SAFETY_ARMED = 0x80


def _decode_heartbeat(p: bytes) -> Optional[dict]:
    if len(p) < 8: return None
    custom_mode, _t, _ap, base_mode, sys_status, _ver = struct.unpack_from("<IBBBBB", p)
    return {
        "armed": bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
        "mode_num": int(custom_mode),
        "system_status": sys_status,
    }


def _decode_sys_status(p: bytes) -> Optional[dict]:
    if len(p) < 31: return None
    # Wire order (size desc): u32 onboard_control_sensors_present, ..._enabled, ..._health,
    # u16 load, voltage_battery, current_battery (sx), drop_rate_comm, errors_comm, ...
    voltage = struct.unpack_from("<H", p, 14)[0]
    current = struct.unpack_from("<h", p, 16)[0]
    pct = struct.unpack_from("<b", p, 30)[0]
    return {"voltage_v": voltage / 1000.0, "current_a": current / 100.0, "battery_pct": pct}


def _decode_gps_raw_int(p: bytes) -> Optional[dict]:
    if len(p) < 30: return None
    lat = struct.unpack_from("<i", p, 8)[0]
    lon = struct.unpack_from("<i", p, 12)[0]
    fix = struct.unpack_from("<B", p, 28)[0]
    sats = struct.unpack_from("<B", p, 29)[0]
    return {"fix_type": fix, "satellites": sats, "lat": lat / 1e7, "lon": lon / 1e7}


def _decode_global_position(p: bytes) -> Optional[dict]:
    if len(p) < 28: return None
    lat = struct.unpack_from("<i", p, 4)[0] / 1e7
    lon = struct.unpack_from("<i", p, 8)[0] / 1e7
    alt = struct.unpack_from("<i", p, 12)[0] / 1000.0
    rel = struct.unpack_from("<i", p, 16)[0] / 1000.0
    hdg = struct.unpack_from("<H", p, 26)[0] / 100.0
    return {"lat": lat, "lon": lon, "alt_m": alt, "rel_alt_m": rel, "heading_deg": hdg}


def _decode_attitude(p: bytes) -> Optional[dict]:
    if len(p) < 28: return None
    roll, pitch, yaw = struct.unpack_from("<3f", p, 4)
    return {"roll_rad": roll, "pitch_rad": pitch, "yaw_rad": yaw}


def _decode_vfr_hud(p: bytes) -> Optional[dict]:
    if len(p) < 20: return None
    # f32 airspeed, groundspeed, alt, climb; i16 heading; u16 throttle
    airspeed, groundspeed, alt, climb = struct.unpack_from("<4f", p, 0)
    heading = struct.unpack_from("<h", p, 16)[0]
    throttle = struct.unpack_from("<H", p, 18)[0]
    return {
        "airspeed_ms": airspeed, "groundspeed_ms": groundspeed,
        "alt_m": alt, "climb_ms": climb,
        "heading_deg": heading, "throttle_pct": throttle,
    }


def _decode_battery_status(p: bytes) -> Optional[dict]:
    if len(p) < 36: return None
    current_consumed_mah = struct.unpack_from("<i", p, 0)[0]
    energy_consumed_hj = struct.unpack_from("<i", p, 4)[0]
    temp = struct.unpack_from("<h", p, 8)[0] / 100.0
    cells = []
    for c in range(10):
        mv = struct.unpack_from("<H", p, 10 + c * 2)[0]
        if mv and mv != 0xFFFF:
            cells.append(mv / 1000.0)
    current_a = struct.unpack_from("<h", p, 30)[0] / 100.0
    out = {
        "consumed_mah": current_consumed_mah,
        "energy_kj": energy_consumed_hj * 0.0001,
        "temp_c": temp,
        "cells_v": cells,
        "current_a": current_a,
    }
    if len(p) >= 36:
        out["battery_pct"] = struct.unpack_from("<b", p, 35)[0]
    if len(p) >= 40:
        out["time_remaining_s"] = struct.unpack_from("<i", p, 36)[0]
    return out


def _decode_vibration(p: bytes) -> Optional[dict]:
    if len(p) < 32: return None
    vx, vy, vz = struct.unpack_from("<3f", p, 8)
    c0, c1, c2 = struct.unpack_from("<3I", p, 20)
    return {"vibration_xyz": [vx, vy, vz], "clipping": [c0, c1, c2]}


def _decode_ekf_status_report(p: bytes) -> Optional[dict]:
    """EKF_STATUS_REPORT (193). Wire order (size desc): 5 floats then uint16.
    Variance values: 0=perfect, ~1.0 = at the threshold of usable, >1.0 bad."""
    if len(p) < 22: return None
    vel, posH, posV, compass, terrain = struct.unpack_from("<5f", p, 0)
    flags = struct.unpack_from("<H", p, 20)[0]
    out = {"flags": flags, "vel_var": vel, "pos_horiz_var": posH,
           "pos_vert_var": posV, "compass_var": compass, "terrain_var": terrain}
    if len(p) >= 24:
        out["airspeed_var"] = struct.unpack_from("<f", p, 22)[0]
    return out


def _decode_power_status(p: bytes) -> Optional[dict]:
    """POWER_STATUS (125). u16 Vcc, u16 Vservo, u16 flags."""
    if len(p) < 6: return None
    Vcc, Vservo, flags = struct.unpack_from("<HHH", p, 0)
    return {"vcc_v": Vcc / 1000.0, "vservo_v": Vservo / 1000.0, "flags": flags}


def _decode_scaled_pressure(p: bytes) -> Optional[dict]:
    """SCALED_PRESSURE (29). u32 time_ms, f32 abs_pressure, f32 diff_pressure, i16 temp."""
    if len(p) < 14: return None
    t_ms, abs_p, diff_p = struct.unpack_from("<Iff", p, 0)
    temp = struct.unpack_from("<h", p, 12)[0]
    return {"abs_pressure_hpa": abs_p, "diff_pressure_hpa": diff_p, "temp_c": temp / 100.0}


def _decode_meminfo(p: bytes) -> Optional[dict]:
    """MEMINFO (152) ardupilot-flavored. u16 brkval, u16 freemem (sometimes
    extended with u32 freemem32 in newer)."""
    if len(p) < 4: return None
    brkval, freemem = struct.unpack_from("<HH", p, 0)
    out = {"brkval": brkval, "freemem_kb": freemem}
    if len(p) >= 8:
        out["freemem32"] = struct.unpack_from("<I", p, 4)[0]
    return out


def _decode_local_position_ned(p: bytes) -> Optional[dict]:
    if len(p) < 28: return None
    t_ms, x, y, z, vx, vy, vz = struct.unpack_from("<I6f", p, 0)
    return {"x_m": x, "y_m": y, "z_m": z, "vx_ms": vx, "vy_ms": vy, "vz_ms": vz}


def _decode_servo_output_raw(p: bytes) -> Optional[dict]:
    # MAVLink v2 truncates trailing zero bytes; the original message is
    # u32 time + u8 port + 16xU16 = 37 bytes, but we may receive
    # anywhere from ~6 bytes upward. Pad the payload to 37 bytes (zeros)
    # before decoding so we don't crash on truncated frames.
    if len(p) < 5: return None
    padded = p + b"\x00" * max(0, 37 - len(p))
    pwm = list(struct.unpack_from("<16H", padded, 5))
    return {"pwm_us": pwm}


def _decode_statustext(p: bytes) -> Optional[dict]:
    if len(p) < 2: return None
    sev = p[0]
    text = bytes(p[1:51]).split(b"\x00", 1)[0].decode("ascii", errors="replace")
    return {"severity": sev, "text": text}


def _decode_command_ack(p: bytes) -> Optional[dict]:
    if len(p) < 3: return None
    cmd, result = struct.unpack_from("<HB", p, 0)
    return {"command": cmd, "result": result}


def _decode_esc_telemetry(p: bytes, base: int) -> Optional[dict]:
    # ESC_TELEMETRY_n_TO_n+3 (ids 11030..11033). Wire order: u16×20 → u8×4
    if len(p) < 44: return None
    out = []
    for i in range(4):
        out.append({
            "esc": base + i + 1,
            "voltage_v": struct.unpack_from("<H", p, 0 + i * 2)[0] / 100.0,
            "current_a": struct.unpack_from("<H", p, 8 + i * 2)[0] / 100.0,
            "totalcurrent_mah": struct.unpack_from("<H", p, 16 + i * 2)[0],
            "rpm": struct.unpack_from("<H", p, 24 + i * 2)[0] * 100,
            "count": struct.unpack_from("<H", p, 32 + i * 2)[0],
            "temp_c": p[40 + i],
        })
    return {"escs": out}


# (msgid -> (name, decoder, throttle interval seconds))
# Throttle = max one JSONL entry per N seconds for that msg type.
# Raw .tlog is always full-rate; this is just to keep JSONL readable.
DECODERS = {
    0:    ("HEARTBEAT",            _decode_heartbeat,                None),  # transition events handled below
    1:    ("SYS_STATUS",           _decode_sys_status,               2.0),
    24:   ("GPS_RAW_INT",          _decode_gps_raw_int,              2.0),
    29:   ("SCALED_PRESSURE",      _decode_scaled_pressure,          5.0),
    30:   ("ATTITUDE",             _decode_attitude,                 0.5),
    32:   ("LOCAL_POSITION_NED",   _decode_local_position_ned,       1.0),
    33:   ("GLOBAL_POSITION_INT",  _decode_global_position,          1.0),
    36:   ("SERVO_OUTPUT_RAW",     _decode_servo_output_raw,         1.0),
    74:   ("VFR_HUD",              _decode_vfr_hud,                  1.0),
    77:   ("COMMAND_ACK",          _decode_command_ack,              None),  # always log
    125:  ("POWER_STATUS",         _decode_power_status,             5.0),
    147:  ("BATTERY_STATUS",       _decode_battery_status,           1.0),
    152:  ("MEMINFO",              _decode_meminfo,                  10.0),  # only every 10s
    193:  ("EKF_STATUS_REPORT",    _decode_ekf_status_report,        2.0),
    241:  ("VIBRATION",            _decode_vibration,                2.0),
    253:  ("STATUSTEXT",           _decode_statustext,               None),  # always log
    11030: ("ESC_TELEMETRY_1_TO_4",  lambda p: _decode_esc_telemetry(p, 0),  1.0),
    11031: ("ESC_TELEMETRY_5_TO_8",  lambda p: _decode_esc_telemetry(p, 4),  1.0),
    11032: ("ESC_TELEMETRY_9_TO_12", lambda p: _decode_esc_telemetry(p, 8),  1.0),
    11033: ("ESC_TELEMETRY_13_TO_16",lambda p: _decode_esc_telemetry(p, 12), 1.0),
}


# ────────────────────────────────────────────────────────────
#  Recorder
# ────────────────────────────────────────────────────────────

class Recorder:
    def __init__(self, ws_url: str, log_dir: Path,
                 rotate_minutes: int = 60, rotate_mb: int = 100):
        self.ws_url = ws_url
        self.log_dir = log_dir
        self.rotate_seconds = rotate_minutes * 60
        self.rotate_bytes = rotate_mb * 1024 * 1024

        self.tlog_f = None
        self.jsonl_f = None
        self.run_id = None
        self.run_started_at = 0
        self.tlog_bytes = 0
        self.jsonl_bytes = 0

        self.last_heartbeat = None  # for arm-transition detection
        self.last_emit_t = {}       # per-msgid throttle

        self.stats = {
            "frames": 0,
            "by_msgid": {},
            "first_t": None,
            "last_t": None,
            "events": 0,
        }

        log_dir.mkdir(parents=True, exist_ok=True)

    def _open_run(self):
        ts = time.strftime("%Y%m%d-%H%M%S")
        self.run_id = ts
        self.tlog_path = self.log_dir / f"{ts}.tlog"
        self.jsonl_path = self.log_dir / f"{ts}.jsonl"
        self.summary_path = self.log_dir / f"{ts}.summary.json"
        self.tlog_f = open(self.tlog_path, "wb")
        self.jsonl_f = open(self.jsonl_path, "w", encoding="utf-8")
        self.run_started_at = time.time()
        self.tlog_bytes = 0
        self.jsonl_bytes = 0
        self.stats = {
            "frames": 0,
            "by_msgid": {},
            "first_t": None,
            "last_t": None,
            "events": 0,
        }
        self._log_meta({"event": "run_start", "ws": self.ws_url})
        print(f"[recorder] opened run {self.run_id}")
        print(f"           tlog:  {self.tlog_path}")
        print(f"           jsonl: {self.jsonl_path}")

    def _close_run(self):
        if not self.tlog_f:
            return
        try:
            self._log_meta({"event": "run_end"})
            duration = time.time() - self.run_started_at
            summary = dict(self.stats)
            summary.update({
                "run_id": self.run_id,
                "duration_s": round(duration, 2),
                "tlog_bytes": self.tlog_bytes,
                "jsonl_bytes": self.jsonl_bytes,
            })
            self.summary_path.write_text(json.dumps(summary, indent=2))
        finally:
            try: self.tlog_f.close()
            except Exception: pass
            try: self.jsonl_f.close()
            except Exception: pass
            self.tlog_f = self.jsonl_f = None
            print(f"[recorder] closed run {self.run_id}: {summary['duration_s']}s, "
                  f"{summary['frames']} frames, {self.tlog_bytes} bytes")

    def _maybe_rotate(self):
        now = time.time()
        if (now - self.run_started_at >= self.rotate_seconds
                or self.tlog_bytes >= self.rotate_bytes):
            self._close_run()
            self._open_run()

    def _log_meta(self, obj: dict) -> None:
        if not self.jsonl_f: return
        obj["_t"] = round(time.time(), 3)
        line = json.dumps(obj, separators=(",", ":")) + "\n"
        self.jsonl_f.write(line)
        self.jsonl_f.flush()
        self.jsonl_bytes += len(line.encode("utf-8"))

    def _record_frame(self, frame: bytes, ts: float) -> None:
        # tlog: u64 timestamp_us BE + frame bytes
        ts_us_be = struct.pack(">Q", int(ts * 1_000_000))
        self.tlog_f.write(ts_us_be)
        self.tlog_f.write(frame)
        self.tlog_bytes += 8 + len(frame)
        self.stats["frames"] += 1
        if self.stats["first_t"] is None:
            self.stats["first_t"] = round(ts, 3)
        self.stats["last_t"] = round(ts, 3)

    def _record_event(self, ts: float, msgid: int, name: str, fields: dict) -> None:
        evt = {"t": round(ts, 3), "msgid": msgid, "name": name}
        evt.update(fields)
        line = json.dumps(evt, separators=(",", ":"), default=str) + "\n"
        self.jsonl_f.write(line)
        # flush so the file is readable in near-real-time and data
        # isn't lost if the process is killed
        if self.stats["events"] % 20 == 0:
            self.jsonl_f.flush()
        self.jsonl_bytes += len(line.encode("utf-8"))
        self.stats["events"] += 1

    def handle_frame(self, frame: bytes) -> None:
        try:
            if len(frame) < 12 or frame[0] != 0xFD:
                return
            plen = frame[1]
            msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            payload = frame[10:10 + plen]
            ts = time.time()

            # Always write to .tlog regardless of decoder success/failure
            self._record_frame(frame, ts)
            self.stats["by_msgid"][str(msgid)] = self.stats["by_msgid"].get(str(msgid), 0) + 1

            if msgid not in DECODERS:
                self._maybe_rotate()
                return
            name, decoder, throttle_s = DECODERS[msgid]
            try:
                decoded = decoder(payload)
            except Exception as e:
                # Bad payload should never kill the recorder
                self._log_meta({"event": "decode_error", "msgid": msgid,
                                "name": name, "error": str(e),
                                "payload_len": len(payload)})
                self._maybe_rotate()
                return
            if decoded is None:
                self._maybe_rotate()
                return

            # Throttle high-rate JSONL emits
            should_emit = True
            if throttle_s is not None:
                last = self.last_emit_t.get(msgid, 0)
                if ts - last < throttle_s and msgid != 0:
                    should_emit = False
                else:
                    self.last_emit_t[msgid] = ts

            # HEARTBEAT — emit on arm or mode change regardless of throttle
            if msgid == 0:
                armed = decoded.get("armed")
                mode = decoded.get("mode_num")
                last_armed = self.last_heartbeat["armed"] if self.last_heartbeat else None
                last_mode = self.last_heartbeat["mode_num"] if self.last_heartbeat else None
                transition = (last_armed != armed) or (last_mode != mode)
                self.last_heartbeat = {"armed": armed, "mode_num": mode}
                if transition or self.last_emit_t.get(0, 0) + 5.0 < ts:
                    self._record_event(ts, msgid, name, decoded)
                    self.last_emit_t[msgid] = ts
                self._maybe_rotate()
                return

            if should_emit:
                self._record_event(ts, msgid, name, decoded)

            self._maybe_rotate()
        except Exception as e:
            # Last-resort: if anything in the handler explodes, log it and keep going
            try:
                self._log_meta({"event": "handle_frame_error", "error": str(e)})
            except Exception:
                pass


# ────────────────────────────────────────────────────────────
#  Main loop
# ────────────────────────────────────────────────────────────

async def run(args):
    recorder = Recorder(args.ws, Path(args.log_dir),
                        rotate_minutes=args.rotate_minutes,
                        rotate_mb=args.rotate_mb)
    recorder._open_run()

    stop = asyncio.Event()

    def _request_stop(*_):
        print("[recorder] stop requested")
        stop.set()
    try:
        signal.signal(signal.SIGINT, _request_stop)
        signal.signal(signal.SIGTERM, _request_stop)
    except Exception:
        pass  # Windows signal limitations

    backoff = 1.0
    try:
        while not stop.is_set():
            try:
                async with websockets.connect(args.ws, open_timeout=10,
                                              ping_interval=20, ping_timeout=20) as ws:
                    backoff = 1.0
                    recorder._log_meta({"event": "ws_connected", "url": args.ws})
                    print(f"[recorder] connected -> {args.ws}", flush=True)
                    while not stop.is_set():
                        try:
                            msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                        except asyncio.TimeoutError:
                            continue
                        if isinstance(msg, bytes):
                            # Frames may be batched; split on 0xFD start markers
                            buf = msg
                            i = 0
                            while i < len(buf):
                                if buf[i] != 0xFD:
                                    i += 1; continue
                                if i + 12 > len(buf): break
                                plen = buf[i + 1]
                                end = i + 10 + plen + 2
                                if end > len(buf): break
                                recorder.handle_frame(buf[i:end])
                                i = end
            except Exception as e:
                if stop.is_set(): break
                recorder._log_meta({"event": "ws_error", "error": str(e), "backoff_s": backoff})
                print(f"[recorder] ws error: {e} — reconnecting in {backoff:.1f}s")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 30.0)
    finally:
        recorder._close_run()


def main():
    p = argparse.ArgumentParser(description="Persistent MAVLink data recorder")
    p.add_argument("--ws", default="ws://localhost:5760",
                   help="MAVLink WebSocket URL (default: ws://localhost:5760)")
    p.add_argument("--log-dir", default=r"C:\Users\vangu\Documents\Vanguard-Backups\flightlogs",
                   help="Directory for tlog/jsonl files (created if missing)")
    p.add_argument("--rotate-minutes", type=int, default=60,
                   help="Rotate logs every N minutes (default 60)")
    p.add_argument("--rotate-mb", type=int, default=100,
                   help="Rotate logs at this size threshold (default 100 MB)")
    args = p.parse_args()
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
