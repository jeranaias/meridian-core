#!/usr/bin/env python3
"""
analyze_flightlog.py — Post-session analyzer for the Vanguard MAVLink recorder.

Parses the .jsonl files written by tools/mavlink-recorder.py and produces
an operational report: session timeline, mode transitions, battery and
vibration profiles, ESC summary, pre-arm failure timeline, and anomaly
flags. Intended to be run after every bench/tank/water session so we
can iterate on tuning + predictive maintenance with real numbers.

Usage:
    # Single file
    python tools/analyze_flightlog.py path/to/20260507-082626.jsonl

    # All sessions in a directory
    python tools/analyze_flightlog.py path/to/flightlogs/

    # JSON output for scripting
    python tools/analyze_flightlog.py path/to/file.jsonl --json

    # Pull a session off the USV first then analyze
    python C:/Users/jesse/bin/usv-ssh.py --get \\
        C:/Users/vangu/Documents/Vanguard-Backups/flightlogs/20260507-082626.jsonl ./
    python tools/analyze_flightlog.py ./20260507-082626.jsonl

The recorder's .jsonl format: one event per line, each with fields
{t, msgid, name, ...decoded fields}.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator, Optional

# UTF-8 stdout so "✗" / "✓" / "⚠" don't crash on Windows cp1252
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


# ──────────────────────────────────────────────────────────────
#  Parsing
# ──────────────────────────────────────────────────────────────

def iter_events(path: Path) -> Iterator[dict]:
    """Yield each parsed event in the .jsonl, skipping malformed lines."""
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                # Truncated final line is normal if the recorder was killed
                # mid-write. Don't spam — just skip.
                continue


# ──────────────────────────────────────────────────────────────
#  Aggregators
# ──────────────────────────────────────────────────────────────

ROVER_MODES = {0:"MANUAL",1:"ACRO",3:"STEERING",4:"HOLD",5:"LOITER",6:"FOLLOW",
               7:"SIMPLE",10:"AUTO",11:"RTL",12:"SMART_RTL",15:"GUIDED",16:"INITIALISING"}
SEV_NAME = {0:"EMERG",1:"ALERT",2:"CRIT",3:"ERROR",4:"WARN",5:"NOTICE",6:"INFO",7:"DEBUG"}
ACK_RESULT = {0:"ACCEPTED",1:"TEMP_REJECTED",2:"DENIED",3:"UNSUPPORTED",4:"FAILED",
              5:"IN_PROGRESS",6:"CANCELLED"}


class Aggregator:
    def __init__(self):
        self.first_t: Optional[float] = None
        self.last_t: Optional[float] = None
        self.event_count = 0
        self.by_msgid: dict[str, int] = {}

        # State transitions
        self.arm_transitions: list[tuple[float, bool, str]] = []  # (t, armed, mode)
        self.mode_transitions: list[tuple[float, str]] = []
        self.gps_fix_transitions: list[tuple[float, int]] = []

        # Battery
        self.batt_pct_samples: list[tuple[float, float]] = []   # (t, pct)
        self.voltage_samples: list[tuple[float, float]] = []
        self.current_samples: list[tuple[float, float]] = []
        self.consumed_mah: float = 0
        self.cell_min_v: Optional[float] = None
        self.cell_min_at_t: Optional[float] = None

        # Vibration
        self.vibe_xyz: list[tuple[float, list[float]]] = []
        self.clipping_max = [0, 0, 0]

        # ESC telemetry
        self.esc_temp_max: dict[int, int] = {}
        self.esc_current_max: dict[int, float] = {}
        self.esc_rpm_max: dict[int, int] = {}

        # Throttle from VFR_HUD
        self.throttle_samples: list[tuple[float, int]] = []
        self.groundspeed_samples: list[tuple[float, float]] = []

        # STATUSTEXT
        self.statustexts: list[dict] = []
        self.prearm_failures: dict[str, list[float]] = {}  # msg → [t, t, ...]

        # COMMAND_ACK
        self.command_acks: list[dict] = []

        # Position
        self.position_samples: list[tuple[float, float, float]] = []  # (t, lat, lon)

    def feed(self, e: dict) -> None:
        t = e.get("t")
        if t is None:
            return
        if self.first_t is None or t < self.first_t:
            self.first_t = t
        if self.last_t is None or t > self.last_t:
            self.last_t = t
        self.event_count += 1
        name = e.get("name", "")
        msgid = e.get("msgid")
        self.by_msgid[name or str(msgid)] = self.by_msgid.get(name or str(msgid), 0) + 1

        if name == "HEARTBEAT":
            armed = e.get("armed")
            mn = e.get("mode_num", -1)
            mode = ROVER_MODES.get(mn, f"mode{mn}")
            # Track transitions only (not every heartbeat)
            if not self.arm_transitions or self.arm_transitions[-1][1] != armed:
                self.arm_transitions.append((t, armed, mode))
            if not self.mode_transitions or self.mode_transitions[-1][1] != mode:
                self.mode_transitions.append((t, mode))
        elif name == "GPS_RAW_INT":
            fix = e.get("fix_type", 0)
            if not self.gps_fix_transitions or self.gps_fix_transitions[-1][1] != fix:
                self.gps_fix_transitions.append((t, fix))
        elif name == "SYS_STATUS":
            v = e.get("voltage_v")
            i = e.get("current_a")
            pct = e.get("battery_pct")
            if v is not None: self.voltage_samples.append((t, v))
            if i is not None: self.current_samples.append((t, i))
            if pct is not None and pct >= 0: self.batt_pct_samples.append((t, pct))
        elif name == "BATTERY_STATUS":
            v_cells = e.get("cells_v") or []
            curr = e.get("current_a")
            mah = e.get("consumed_mah")
            pct = e.get("battery_pct")
            if v_cells:
                m = min(v_cells)
                if self.cell_min_v is None or m < self.cell_min_v:
                    self.cell_min_v = m
                    self.cell_min_at_t = t
            if curr is not None: self.current_samples.append((t, curr))
            if mah is not None: self.consumed_mah = max(self.consumed_mah, mah)
            if pct is not None and pct >= 0: self.batt_pct_samples.append((t, pct))
        elif name == "VIBRATION":
            xyz = e.get("vibration_xyz") or []
            if len(xyz) == 3:
                self.vibe_xyz.append((t, xyz))
            cl = e.get("clipping") or []
            for i, c in enumerate(cl[:3]):
                if c > self.clipping_max[i]:
                    self.clipping_max[i] = c
        elif name == "VFR_HUD":
            thr = e.get("throttle_pct")
            gs = e.get("groundspeed_ms")
            if thr is not None: self.throttle_samples.append((t, thr))
            if gs is not None: self.groundspeed_samples.append((t, gs))
        elif name == "GLOBAL_POSITION_INT":
            lat = e.get("lat")
            lon = e.get("lon")
            if lat and lon:
                self.position_samples.append((t, lat, lon))
        elif name == "STATUSTEXT":
            sev = e.get("severity", 7)
            text = e.get("text", "")
            self.statustexts.append({"t": t, "severity": sev, "text": text})
            if text.startswith("PreArm:"):
                key = text[len("PreArm:"):].strip()
                self.prearm_failures.setdefault(key, []).append(t)
        elif name == "COMMAND_ACK":
            self.command_acks.append({
                "t": t,
                "command": e.get("command"),
                "result": e.get("result"),
            })
        elif name and name.startswith("ESC_TELEMETRY"):
            for esc in (e.get("escs") or []):
                idx = esc.get("esc")
                if idx is None: continue
                self.esc_temp_max[idx] = max(self.esc_temp_max.get(idx, 0), esc.get("temp_c", 0))
                self.esc_current_max[idx] = max(self.esc_current_max.get(idx, 0.0), esc.get("current_a", 0.0))
                self.esc_rpm_max[idx] = max(self.esc_rpm_max.get(idx, 0), esc.get("rpm", 0))


# ──────────────────────────────────────────────────────────────
#  Anomaly detection (heuristics — good enough for first pass)
# ──────────────────────────────────────────────────────────────

def detect_anomalies(agg: Aggregator) -> list[str]:
    out: list[str] = []
    # Battery sag
    if agg.voltage_samples:
        vmax = max(v for _, v in agg.voltage_samples)
        vmin = min(v for _, v in agg.voltage_samples)
        if vmax > 0 and (vmax - vmin) / vmax > 0.18:
            out.append(f"Battery sagged {(vmax - vmin):.2f}V ({(vmax - vmin) / vmax * 100:.0f}%) "
                       f"under load — check pack health or current limits")
    # Vibration high
    if agg.vibe_xyz:
        peaks = [max(xyz) for _, xyz in agg.vibe_xyz]
        if peaks and statistics.mean(peaks) > 30:
            out.append(f"Mean vibration peak {statistics.mean(peaks):.1f} m/s² — above 30 threshold; "
                       f"check IMU mounting / prop balance")
        if peaks and max(peaks) > 60:
            out.append(f"Peak vibration {max(peaks):.1f} m/s² — exceeded 60 critical, EKF may have rejected fusion")
    # Clipping
    if max(agg.clipping_max) > 100:
        out.append(f"IMU clipping high (max counts {agg.clipping_max}) — physical mount or vibration issue")
    # ESC temp
    for idx, t in agg.esc_temp_max.items():
        if t >= 100:
            out.append(f"ESC{idx} temp peaked at {t}°C — at thermal limit")
        elif t >= 80:
            out.append(f"ESC{idx} temp peaked at {t}°C — above warning threshold")
    # Cell voltage
    if agg.cell_min_v is not None and agg.cell_min_v < 3.50:
        out.append(f"Lowest cell {agg.cell_min_v:.3f}V — below LiPo critical (3.50V)")
    elif agg.cell_min_v is not None and agg.cell_min_v < 3.65:
        out.append(f"Lowest cell {agg.cell_min_v:.3f}V — below LiPo warn (3.65V)")
    # Failed commands
    for ack in agg.command_acks:
        if ack["result"] not in (0, 5):  # ACCEPTED or IN_PROGRESS
            name = ACK_RESULT.get(ack["result"], f"R{ack['result']}")
            out.append(f"Command {ack['command']} returned {name} at t={ack['t']:.1f}")
    return out


# ──────────────────────────────────────────────────────────────
#  Reporting
# ──────────────────────────────────────────────────────────────

def fmt_t(t: float) -> str:
    try:
        return datetime.fromtimestamp(t, tz=timezone.utc).strftime("%H:%M:%S")
    except (OSError, OverflowError, ValueError):
        return f"{t:.0f}"


def fmt_duration(seconds: float) -> str:
    s = int(seconds)
    h, rem = divmod(s, 3600)
    m, s = divmod(rem, 60)
    if h: return f"{h}h{m:02d}m{s:02d}s"
    if m: return f"{m}m{s:02d}s"
    return f"{s}s"


def render_text_report(agg: Aggregator, source: str) -> str:
    lines = []
    a = lines.append
    sep = "─" * 72
    a(sep)
    a(f"  VANGUARD FLIGHT LOG — {source}")
    a(sep)
    if agg.first_t is None:
        a("  EMPTY — no decoded events.")
        a(sep)
        return "\n".join(lines)

    duration = (agg.last_t or 0) - (agg.first_t or 0)
    a(f"  Session: {fmt_t(agg.first_t)} → {fmt_t(agg.last_t)}  ({fmt_duration(duration)})")
    a(f"  Events: {agg.event_count}  ({len(agg.by_msgid)} message types)")
    a("")

    # State transitions
    a("  STATE TRANSITIONS")
    a("  " + "─" * 30)
    if agg.arm_transitions:
        for t, armed, mode in agg.arm_transitions[-12:]:
            a(f"    {fmt_t(t)}   {'ARMED  ' if armed else 'DISARM '}  {mode}")
    else:
        a("    (no arm/disarm events seen)")
    a("")

    if agg.mode_transitions:
        a("  MODE TIMELINE")
        a("  " + "─" * 30)
        for t, m in agg.mode_transitions[-15:]:
            a(f"    {fmt_t(t)}   {m}")
        a("")

    if agg.gps_fix_transitions:
        FIX = ["NoGPS","NoFix","2D","3D","DGPS","RTKf","RTK"]
        a("  GPS FIX TRANSITIONS")
        a("  " + "─" * 30)
        for t, f in agg.gps_fix_transitions[-10:]:
            a(f"    {fmt_t(t)}   {FIX[f] if f < 7 else f'fix{f}'}")
        a("")

    # Battery
    if agg.voltage_samples or agg.current_samples or agg.batt_pct_samples:
        a("  BATTERY PROFILE")
        a("  " + "─" * 30)
        if agg.voltage_samples:
            v_min = min(v for _, v in agg.voltage_samples)
            v_max = max(v for _, v in agg.voltage_samples)
            v_end = agg.voltage_samples[-1][1]
            a(f"    voltage    min {v_min:.2f}V  max {v_max:.2f}V  end {v_end:.2f}V")
        if agg.current_samples:
            c_max = max(c for _, c in agg.current_samples)
            c_avg = statistics.mean([c for _, c in agg.current_samples])
            a(f"    current    avg {c_avg:.1f}A  peak {c_max:.1f}A")
        if agg.batt_pct_samples:
            p_start = agg.batt_pct_samples[0][1]
            p_end = agg.batt_pct_samples[-1][1]
            a(f"    %          start {p_start}%  end {p_end}%  drop {p_start - p_end:+}%")
        if agg.consumed_mah:
            a(f"    consumed   {agg.consumed_mah} mAh")
        if agg.cell_min_v is not None:
            a(f"    min cell   {agg.cell_min_v:.3f}V at {fmt_t(agg.cell_min_at_t)}")
        a("")

    # Vibration
    if agg.vibe_xyz:
        peaks = [max(xyz) for _, xyz in agg.vibe_xyz]
        a("  VIBRATION PROFILE")
        a("  " + "─" * 30)
        a(f"    peak m/s²  axis-max {max(peaks):.1f}  mean {statistics.mean(peaks):.1f}  median {statistics.median(peaks):.1f}")
        a(f"    clipping   max counts per IMU: {agg.clipping_max}")
        a("")

    # ESC
    if agg.esc_temp_max or agg.esc_current_max:
        a("  ESC TELEMETRY")
        a("  " + "─" * 30)
        for idx in sorted(set(list(agg.esc_temp_max.keys()) + list(agg.esc_current_max.keys()))):
            t = agg.esc_temp_max.get(idx, 0)
            c = agg.esc_current_max.get(idx, 0.0)
            r = agg.esc_rpm_max.get(idx, 0)
            a(f"    ESC{idx}       peak {t}°C  {c:.1f}A  {r} rpm")
        a("")

    # Throttle / speed
    if agg.throttle_samples or agg.groundspeed_samples:
        a("  CONTROL / MOTION")
        a("  " + "─" * 30)
        if agg.throttle_samples:
            ts = [t for _, t in agg.throttle_samples if t > 0]
            if ts:
                a(f"    throttle   peak {max(ts)}%  active mean {statistics.mean(ts):.1f}%  samples {len(ts)}")
        if agg.groundspeed_samples:
            gs = [s for _, s in agg.groundspeed_samples]
            if gs:
                a(f"    gnd speed  peak {max(gs):.2f} m/s  mean {statistics.mean(gs):.2f} m/s")
        a("")

    # Pre-arm
    if agg.prearm_failures:
        a("  PRE-ARM FAILURES")
        a("  " + "─" * 30)
        for k, ts in sorted(agg.prearm_failures.items()):
            first = fmt_t(ts[0])
            last = fmt_t(ts[-1])
            a(f"    {len(ts):>3}× {k}   (first {first}, last {last})")
        a("")

    # STATUSTEXT severity ≤ 4
    sig_msgs = [s for s in agg.statustexts if s["severity"] <= 4]
    if sig_msgs:
        a(f"  WARNINGS / ERRORS  ({len(sig_msgs)} of {len(agg.statustexts)} STATUSTEXTs)")
        a("  " + "─" * 30)
        # Dedupe by text and show first occurrence
        seen = {}
        for s in sig_msgs:
            if s["text"] not in seen:
                seen[s["text"]] = s
        for text, s in list(seen.items())[:20]:
            sev = SEV_NAME.get(s["severity"], "?")
            a(f"    [{sev:5}] {text}")
        if len(seen) > 20:
            a(f"    ... and {len(seen) - 20} more")
        a("")

    # Anomalies
    anomalies = detect_anomalies(agg)
    if anomalies:
        a("  ⚠ ANOMALIES DETECTED")
        a("  " + "─" * 30)
        for line in anomalies:
            a(f"    • {line}")
        a("")
    else:
        a("  ✓ No anomalies detected.")
        a("")

    # Message-type frequency
    a("  MESSAGE COUNTS (top 10)")
    a("  " + "─" * 30)
    for name, count in sorted(agg.by_msgid.items(), key=lambda x: -x[1])[:10]:
        a(f"    {name:<25} {count}")
    a("")
    a(sep)
    return "\n".join(lines)


def to_json_summary(agg: Aggregator, source: str) -> dict:
    return {
        "source": source,
        "first_t": agg.first_t,
        "last_t": agg.last_t,
        "duration_s": (agg.last_t - agg.first_t) if agg.first_t else 0,
        "event_count": agg.event_count,
        "by_msgid": agg.by_msgid,
        "arm_transitions": agg.arm_transitions,
        "mode_transitions": agg.mode_transitions,
        "gps_fix_transitions": agg.gps_fix_transitions,
        "battery": {
            "voltage_min": min((v for _, v in agg.voltage_samples), default=None),
            "voltage_max": max((v for _, v in agg.voltage_samples), default=None),
            "current_peak": max((c for _, c in agg.current_samples), default=None),
            "consumed_mah": agg.consumed_mah,
            "cell_min_v": agg.cell_min_v,
        },
        "vibration": {
            "peak_max": max((max(xyz) for _, xyz in agg.vibe_xyz), default=None),
            "peak_mean": (statistics.mean([max(xyz) for _, xyz in agg.vibe_xyz])
                          if agg.vibe_xyz else None),
            "clipping_max": agg.clipping_max,
        },
        "esc": {f"esc{i}": {
            "temp_c_max": agg.esc_temp_max.get(i),
            "current_a_max": agg.esc_current_max.get(i),
            "rpm_max": agg.esc_rpm_max.get(i),
        } for i in sorted(set(list(agg.esc_temp_max.keys()) + list(agg.esc_current_max.keys())))},
        "prearm_failures": {k: len(v) for k, v in agg.prearm_failures.items()},
        "warnings_errors": [
            {"t": s["t"], "severity": s["severity"], "text": s["text"]}
            for s in agg.statustexts if s["severity"] <= 4
        ],
        "anomalies": detect_anomalies(agg),
    }


# ──────────────────────────────────────────────────────────────
#  CLI
# ──────────────────────────────────────────────────────────────

def analyze(path: Path, as_json: bool = False) -> Any:
    agg = Aggregator()
    for e in iter_events(path):
        try:
            agg.feed(e)
        except Exception:
            continue
    if as_json:
        return to_json_summary(agg, str(path))
    return render_text_report(agg, str(path))


def main():
    p = argparse.ArgumentParser(description="Analyze a Vanguard MAVLink recorder .jsonl session")
    p.add_argument("path", help="path to .jsonl file or directory of them")
    p.add_argument("--json", action="store_true", help="Emit JSON instead of text report")
    args = p.parse_args()

    target = Path(args.path)
    if target.is_dir():
        # Walk all jsonl files in dir, sorted by name (chronological by timestamp)
        files = sorted(target.glob("*.jsonl"))
        if not files:
            print(f"No .jsonl files in {target}", file=sys.stderr)
            sys.exit(1)
        for f in files:
            r = analyze(f, args.json)
            if args.json:
                print(json.dumps(r, indent=2, default=str))
            else:
                print(r)
                print()
    elif target.is_file():
        r = analyze(target, args.json)
        if args.json:
            print(json.dumps(r, indent=2, default=str))
        else:
            print(r)
    else:
        print(f"not found: {target}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
