#!/usr/bin/env python3
"""
usv-sim-suite.py - Comprehensive USV simulation suite for Vanguard.

Runs 12 operational scenarios, logs telemetry for each, serves results
on a WebSocket so the Meridian GCS can replay them live.

Usage:
    python tools/usv-sim-suite.py                    # Run all scenarios
    python tools/usv-sim-suite.py --scenario 3       # Run specific scenario
    python tools/usv-sim-suite.py --replay 1          # Replay scenario 1 on GCS
    python tools/usv-sim-suite.py --report            # Print summary report

Each scenario produces a .json log file in results/ that can be replayed
through the GCS for visual review.
"""

import argparse
import json
import math
import os
import struct
import time
import asyncio
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("sim")

# == Reuse the SITL boat physics ==============================

class Boat:
    def __init__(self, lat, lon, heading_deg):
        self.lat = lat
        self.lon = lon
        self.speed = 0.0
        self.heading = math.radians(heading_deg)
        self.yaw_rate = 0.0
        self.thrust = 0.0
        self.nozzle = 0.0
        self.armed = True
        self.battery = 92.0
        self.voltage = 25.1
        self.current_n = 0.0
        self.current_e = 0.0
        self.waypoints = []
        self.wp_idx = 0
        self.loiter_center = None
        self.mode = 'AUTO'
        self.gps_healthy = True
        self.comms_healthy = True

    def set_current(self, speed_ms, from_deg):
        to_rad = (from_deg + 180.0) * math.pi / 180.0
        self.current_n = speed_ms * math.cos(to_rad)
        self.current_e = speed_ms * math.sin(to_rad)

    def step(self, throttle, steering, dt):
        if not self.armed:
            self.speed *= 0.95
            self.yaw_rate *= 0.9
            return
        target = max(0, min(1, throttle)) * 450.0
        self.thrust += (dt / 1.0) * (target - self.thrust)
        target_n = max(-1, min(1, steering)) * math.radians(20)
        rate = 1.5 * dt
        d = target_n - self.nozzle
        self.nozzle += max(-rate, min(rate, d))
        fwd = self.thrust * math.cos(self.nozzle) - 50.0 * self.speed * abs(self.speed)
        torque = self.thrust * math.sin(self.nozzle) * 1.5 - 25.0 * self.yaw_rate * abs(self.yaw_rate)
        self.speed += fwd / 180.0 * dt
        self.yaw_rate += torque / 40.0 * dt
        self.heading += self.yaw_rate * dt
        self.heading %= 2 * math.pi
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        self.lat += vn * dt / 6371000.0 * (180 / math.pi)
        self.lon += ve * dt / (6371000.0 * math.cos(math.radians(self.lat))) * (180 / math.pi)
        # Realistic battery drain: 3300mAh, ~15A at cruise, ~25A at full
        # At cruise (50% throttle): 15A * dt/3600 * 1000/3300 * 100 = 0.126%/s
        amps = 5.0 + throttle * 20.0  # 5A idle + 20A at full throttle
        mah_used = amps * dt / 3.6  # dt in seconds, convert to mAh
        self.battery = max(0, self.battery - mah_used / 33.0)  # 3300mAh = 33 per %
        self.voltage = 22.0 + self.battery / 100.0 * 3.2

    def autopilot_to_wp(self, dt):
        if self.wp_idx >= len(self.waypoints):
            return 0.0, 0.0, True
        wp = self.waypoints[self.wp_idx]
        dlat = wp[0] - self.lat
        dlon = wp[1] - self.lon
        dn = dlat * 111320
        de = dlon * 111320 * math.cos(math.radians(self.lat))
        dist = math.sqrt(dn * dn + de * de)
        bearing = math.atan2(de, dn)
        if dist < 2.0:
            self.wp_idx += 1
            return 0.1, 0.0, self.wp_idx >= len(self.waypoints)

        # Cross-track error correction: measure how far off the direct line we are
        # and bias the target bearing to compensate for current drift.
        # This is a simplified L1-style correction.
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        gs = math.sqrt(vn * vn + ve * ve)
        if gs > 0.3:
            # Ground track direction
            track = math.atan2(ve, vn)
            # Drift angle: difference between heading and ground track
            drift = track - self.heading
            while drift > math.pi: drift -= 2 * math.pi
            while drift < -math.pi: drift += 2 * math.pi
            # Counter-steer: aim upstream of waypoint
            # Full correction + extra for convergence
            corrected_bearing = bearing - drift * 1.2  # 120% overcorrection to converge
        else:
            corrected_bearing = bearing

        err = corrected_bearing - self.heading
        while err > math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        if dist > 20:
            thr = 0.55
        elif dist > 8:
            thr = 0.45
        else:
            thr = max(0.25, dist * 0.04)
        return thr, max(-1, min(1, err * 1.0)), False

    def autopilot_loiter(self, center, dt):
        dlat = center[0] - self.lat
        dlon = center[1] - self.lon
        dist = math.sqrt((dlat * 111320)**2 + (dlon * 111320 * math.cos(math.radians(self.lat)))**2)
        if dist < 2.0:
            return 0.05, 0.0
        bearing = math.atan2(dlon * math.cos(math.radians(self.lat)), dlat)
        err = bearing - self.heading
        while err > math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return min(0.35, dist * 0.06), max(-1, min(1, err * 1.0))

    def hdg_deg(self):
        return (math.degrees(self.heading) % 360 + 360) % 360

    def gnd_speed(self):
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        return math.sqrt(vn * vn + ve * ve)

    def distance_to(self, lat, lon):
        dlat = lat - self.lat
        dlon = lon - self.lon
        return math.sqrt((dlat * 111320)**2 + (dlon * 111320 * math.cos(math.radians(self.lat)))**2)


# == Telemetry Logger =========================================

class TelemetryLog:
    def __init__(self):
        self.entries = []

    def record(self, t, boat, extra=None):
        entry = {
            't': round(t, 2),
            'lat': round(boat.lat, 7),
            'lon': round(boat.lon, 7),
            'hdg': round(boat.hdg_deg(), 1),
            'spd': round(boat.gnd_speed(), 2),
            'batt': round(boat.battery, 1),
            'volt': round(boat.voltage, 2),
            'thrust': round(boat.thrust, 1),
            'nozzle': round(math.degrees(boat.nozzle), 1),
            'yaw_rate': round(math.degrees(boat.yaw_rate), 1),
            'mode': boat.mode,
            'wp_idx': boat.wp_idx,
        }
        if extra:
            entry.update(extra)
        self.entries.append(entry)

    def save(self, filename):
        os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
        with open(filename, 'w') as f:
            json.dump(self.entries, f, indent=1)
        log.info(f"Saved {len(self.entries)} entries to {filename}")


# == Scenario Definitions =====================================

# Base position: Botany Bay, Sydney
BASE_LAT = -33.9700
BASE_LON = 151.2000

def run_scenario(num):
    """Run a single scenario. Returns (name, log, metrics)."""
    scenarios = {
        1: scenario_transit_ab,
        2: scenario_station_keep,
        3: scenario_relay_reposition,
        4: scenario_rtl_from_waypoint,
        5: scenario_current_compensation,
        6: scenario_gps_degraded,
        7: scenario_comms_loss,
        8: scenario_low_battery_transit,
        9: scenario_heavy_sea_state,
        10: scenario_multi_wp_survey,
        11: scenario_loiter_with_drift,
        12: scenario_full_mission_relay,
    }
    if num not in scenarios:
        log.error(f"Unknown scenario {num}")
        return None
    return scenarios[num]()


def scenario_transit_ab():
    """Scenario 1: Simple A->B transit in calm water."""
    name = "Transit A->B (calm)"
    boat = Boat(BASE_LAT, BASE_LON, 45)
    boat.waypoints = [
        (BASE_LAT + 0.002, BASE_LON + 0.003),  # ~350m NE
    ]
    tlog = TelemetryLog()
    metrics = {'max_crosstrack': 0, 'transit_time': 0, 'final_dist': 0}

    dt = 0.1
    for i in range(3000):  # 5 minutes max
        t = i * dt
        thr, steer, done = boat.autopilot_to_wp(dt)
        boat.step(thr, steer, dt)
        if i % 10 == 0:
            tlog.record(t, boat)
        if done:
            metrics['transit_time'] = t
            break

    metrics['final_dist'] = boat.distance_to(*boat.waypoints[-1])
    log.info(f"[1] {name}: time={metrics['transit_time']:.0f}s dist={metrics['final_dist']:.1f}m")
    return name, tlog, metrics


def scenario_station_keep():
    """Scenario 2: Hold position as relay station for 10 minutes."""
    name = "Station keeping (relay position, 10 min)"
    boat = Boat(BASE_LAT + 0.002, BASE_LON + 0.002, 0)
    boat.set_current(0.3, 240)  # mild SW current
    center = (boat.lat, boat.lon)
    tlog = TelemetryLog()
    max_drift = 0
    drifts = []

    dt = 0.1
    for i in range(6000):  # 10 minutes
        t = i * dt
        thr, steer = boat.autopilot_loiter(center, dt)
        boat.step(thr, steer, dt)
        drift = boat.distance_to(*center)
        if drift > max_drift:
            max_drift = drift
        if i % 50 == 0:
            drifts.append(drift)
        if i % 10 == 0:
            tlog.record(t, boat, {'drift_m': round(drift, 2)})

    avg_drift = sum(drifts) / len(drifts) if drifts else 0
    metrics = {'max_drift': round(max_drift, 2), 'avg_drift': round(avg_drift, 2), 'battery_used': round(92 - boat.battery, 1)}
    log.info(f"[2] {name}: max_drift={max_drift:.1f}m avg={avg_drift:.1f}m batt_used={metrics['battery_used']:.1f}%")
    return name, tlog, metrics


def scenario_relay_reposition():
    """Scenario 3: Transit to relay position, hold, then reposition to new relay point."""
    name = "Relay reposition (transit -> hold -> transit -> hold)"
    boat = Boat(BASE_LAT, BASE_LON, 90)
    relay_1 = (BASE_LAT + 0.002, BASE_LON + 0.002)
    relay_2 = (BASE_LAT + 0.004, BASE_LON + 0.001)
    boat.waypoints = [relay_1]
    boat.set_current(0.2, 180)
    tlog = TelemetryLog()
    phase = 'transit1'
    hold_start = 0
    metrics = {}

    dt = 0.1
    for i in range(12000):  # 20 min
        t = i * dt
        if phase == 'transit1':
            thr, steer, done = boat.autopilot_to_wp(dt)
            if done:
                phase = 'hold1'
                hold_start = t
                boat.mode = 'LOITER'
                metrics['transit1_time'] = round(t, 1)
        elif phase == 'hold1':
            thr, steer = boat.autopilot_loiter(relay_1, dt)
            if t - hold_start > 120:  # hold for 2 min
                phase = 'transit2'
                boat.waypoints = [relay_2]
                boat.wp_idx = 0
                boat.mode = 'AUTO'
        elif phase == 'transit2':
            thr, steer, done = boat.autopilot_to_wp(dt)
            if done:
                phase = 'hold2'
                hold_start = t
                boat.mode = 'LOITER'
                metrics['transit2_time'] = round(t - (metrics.get('transit1_time', 0) + 120), 1)
        elif phase == 'hold2':
            thr, steer = boat.autopilot_loiter(relay_2, dt)
            if t - hold_start > 120:
                break
        else:
            thr, steer = 0, 0

        boat.step(thr, steer, dt)
        if i % 10 == 0:
            tlog.record(t, boat, {'phase': phase})

    metrics['total_time'] = round(i * dt, 1)
    metrics['battery_used'] = round(92 - boat.battery, 1)
    log.info(f"[3] {name}: phases complete, batt_used={metrics['battery_used']:.1f}%")
    return name, tlog, metrics


def scenario_rtl_from_waypoint():
    """Scenario 4: Transit to waypoint, then RTL."""
    name = "RTL from waypoint"
    boat = Boat(BASE_LAT, BASE_LON, 45)
    home = (BASE_LAT, BASE_LON)
    wp = (BASE_LAT + 0.003, BASE_LON + 0.003)
    boat.waypoints = [wp]
    tlog = TelemetryLog()
    phase = 'transit'
    metrics = {}

    dt = 0.1
    for i in range(6000):
        t = i * dt
        if phase == 'transit':
            thr, steer, done = boat.autopilot_to_wp(dt)
            if done:
                phase = 'rtl'
                boat.waypoints = [home]
                boat.wp_idx = 0
                boat.mode = 'RTL'
                metrics['outbound_time'] = round(t, 1)
        elif phase == 'rtl':
            thr, steer, done = boat.autopilot_to_wp(dt)
            if done:
                metrics['rtl_time'] = round(t - metrics['outbound_time'], 1)
                break
        else:
            thr, steer = 0, 0

        boat.step(thr, steer, dt)
        if i % 10 == 0:
            tlog.record(t, boat, {'phase': phase})

    metrics['final_dist_home'] = round(boat.distance_to(*home), 1)
    log.info(f"[4] {name}: out={metrics.get('outbound_time',0):.0f}s rtl={metrics.get('rtl_time',0):.0f}s home_dist={metrics['final_dist_home']:.1f}m")
    return name, tlog, metrics


def scenario_current_compensation():
    """Scenario 5: Transit A->B with strong cross-current."""
    name = "Transit with 1.5 m/s cross-current"
    boat = Boat(BASE_LAT, BASE_LON, 0)
    boat.waypoints = [(BASE_LAT + 0.003, BASE_LON)]  # straight north
    boat.set_current(1.5, 270)  # strong current from west (pushing east)
    tlog = TelemetryLog()
    max_crosstrack = 0

    dt = 0.1
    for i in range(6000):
        t = i * dt
        thr, steer, done = boat.autopilot_to_wp(dt)
        boat.step(thr, steer, dt)
        # Crosstrack: east deviation from straight-north line
        crosstrack = abs(boat.lon - BASE_LON) * 111320 * math.cos(math.radians(boat.lat))
        if crosstrack > max_crosstrack:
            max_crosstrack = crosstrack
        if i % 10 == 0:
            tlog.record(t, boat, {'crosstrack_m': round(crosstrack, 2)})
        if done:
            break

    metrics = {'max_crosstrack': round(max_crosstrack, 1), 'transit_time': round(i * dt, 1)}
    log.info(f"[5] {name}: max_crosstrack={max_crosstrack:.1f}m time={metrics['transit_time']:.0f}s")
    return name, tlog, metrics


def scenario_gps_degraded():
    """Scenario 6: GPS loss during station keeping - should hold heading and reduce speed."""
    name = "GPS degradation during station keeping"
    boat = Boat(BASE_LAT + 0.002, BASE_LON + 0.002, 0)
    center = (boat.lat, boat.lon)
    tlog = TelemetryLog()
    max_drift = 0

    dt = 0.1
    for i in range(3000):  # 5 min
        t = i * dt
        # GPS fails at 60 seconds
        if t > 60 and t < 180:
            boat.gps_healthy = False
            # Without GPS, autopilot should reduce throttle and hold heading
            thr = 0.05  # minimal thrust
            steer = 0.0  # hold heading
        else:
            boat.gps_healthy = True
            thr, steer = boat.autopilot_loiter(center, dt)

        boat.step(thr, steer, dt)
        drift = boat.distance_to(*center)
        if drift > max_drift:
            max_drift = drift
        if i % 10 == 0:
            tlog.record(t, boat, {'gps_ok': boat.gps_healthy, 'drift_m': round(drift, 2)})

    metrics = {'max_drift_during_gps_loss': round(max_drift, 1), 'recovered': boat.gps_healthy}
    log.info(f"[6] {name}: max_drift={max_drift:.1f}m recovered={metrics['recovered']}")
    return name, tlog, metrics


def scenario_comms_loss():
    """Scenario 7: Silvus radio loss - should hold position."""
    name = "Comms loss (Silvus radio) - hold position"
    boat = Boat(BASE_LAT + 0.002, BASE_LON + 0.002, 45)
    boat.set_current(0.3, 200)
    center = (boat.lat, boat.lon)
    tlog = TelemetryLog()
    max_drift = 0

    dt = 0.1
    for i in range(3000):
        t = i * dt
        # Comms lost at 30s, recovered at 150s
        if t > 30 and t < 150:
            boat.comms_healthy = False
            boat.mode = 'HOLD'
            thr, steer = boat.autopilot_loiter(center, dt)
        else:
            boat.comms_healthy = True
            boat.mode = 'LOITER'
            thr, steer = boat.autopilot_loiter(center, dt)

        boat.step(thr, steer, dt)
        drift = boat.distance_to(*center)
        if drift > max_drift:
            max_drift = drift
        if i % 10 == 0:
            tlog.record(t, boat, {'comms_ok': boat.comms_healthy, 'drift_m': round(drift, 2)})

    metrics = {'max_drift': round(max_drift, 1), 'comms_restored': boat.comms_healthy}
    log.info(f"[7] {name}: max_drift={max_drift:.1f}m")
    return name, tlog, metrics


def scenario_low_battery_transit():
    """Scenario 8: Battery drops below threshold during transit - triggers RTL."""
    name = "Low battery RTL during transit"
    boat = Boat(BASE_LAT, BASE_LON, 45)
    boat.battery = 25.0  # start low
    home = (BASE_LAT, BASE_LON)
    boat.waypoints = [(BASE_LAT + 0.005, BASE_LON + 0.005)]  # far away
    tlog = TelemetryLog()
    rtl_triggered = False
    phase = 'transit'

    dt = 0.1
    for i in range(6000):
        t = i * dt
        if boat.battery < 15.0 and not rtl_triggered:
            rtl_triggered = True
            phase = 'rtl'
            boat.waypoints = [home]
            boat.wp_idx = 0
            boat.mode = 'RTL'
            log.info(f"  Battery failsafe at {t:.0f}s, batt={boat.battery:.1f}%")

        if phase == 'transit':
            thr, steer, done = boat.autopilot_to_wp(dt)
        elif phase == 'rtl':
            thr, steer, done = boat.autopilot_to_wp(dt)
            if done:
                break
        else:
            thr, steer = 0, 0

        boat.step(thr, steer, dt)
        if i % 10 == 0:
            tlog.record(t, boat, {'phase': phase})

    metrics = {
        'rtl_triggered': rtl_triggered,
        'final_dist_home': round(boat.distance_to(*home), 1),
        'final_battery': round(boat.battery, 1),
    }
    log.info(f"[8] {name}: rtl={rtl_triggered} home_dist={metrics['final_dist_home']:.1f}m batt={metrics['final_battery']:.1f}%")
    return name, tlog, metrics


def scenario_heavy_sea_state():
    """Scenario 9: Station keeping in heavy chop (simulated heading/speed noise)."""
    name = "Heavy sea state - station keeping"
    boat = Boat(BASE_LAT + 0.002, BASE_LON + 0.002, 0)
    center = (boat.lat, boat.lon)
    boat.set_current(0.5, 220)
    tlog = TelemetryLog()
    max_drift = 0

    dt = 0.1
    for i in range(6000):  # 10 min
        t = i * dt
        thr, steer = boat.autopilot_loiter(center, dt)

        # Add sea state noise: random heading kicks and speed variation
        heading_noise = 0.03 * math.sin(t * 2.1) + 0.02 * math.sin(t * 5.3) + 0.01 * math.sin(t * 11.7)
        speed_noise = 0.1 * math.sin(t * 1.7) + 0.05 * math.sin(t * 4.9)

        boat.step(thr + speed_noise * 0.1, steer + heading_noise, dt)
        drift = boat.distance_to(*center)
        if drift > max_drift:
            max_drift = drift
        if i % 10 == 0:
            tlog.record(t, boat, {'drift_m': round(drift, 2), 'sea_noise': round(heading_noise, 3)})

    metrics = {'max_drift': round(max_drift, 1), 'battery_used': round(92 - boat.battery, 1)}
    log.info(f"[9] {name}: max_drift={max_drift:.1f}m batt_used={metrics['battery_used']:.1f}%")
    return name, tlog, metrics


def scenario_multi_wp_survey():
    """Scenario 10: Multi-waypoint survey pattern (grid)."""
    name = "Multi-WP survey grid (8 waypoints)"
    boat = Boat(BASE_LAT, BASE_LON, 0)
    # Simple grid pattern
    boat.waypoints = [
        (BASE_LAT + 0.001, BASE_LON),
        (BASE_LAT + 0.001, BASE_LON + 0.002),
        (BASE_LAT + 0.002, BASE_LON + 0.002),
        (BASE_LAT + 0.002, BASE_LON),
        (BASE_LAT + 0.003, BASE_LON),
        (BASE_LAT + 0.003, BASE_LON + 0.002),
        (BASE_LAT + 0.004, BASE_LON + 0.002),
        (BASE_LAT + 0.004, BASE_LON),
    ]
    tlog = TelemetryLog()

    dt = 0.1
    for i in range(18000):  # 30 min max
        t = i * dt
        thr, steer, done = boat.autopilot_to_wp(dt)
        boat.step(thr, steer, dt)
        if i % 10 == 0:
            tlog.record(t, boat)
        if done:
            break

    wps_reached = boat.wp_idx
    metrics = {
        'waypoints_reached': wps_reached,
        'total_waypoints': len(boat.waypoints),
        'time': round(i * dt, 1),
        'battery_used': round(92 - boat.battery, 1),
    }
    log.info(f"[10] {name}: {wps_reached}/{len(boat.waypoints)} WPs in {metrics['time']:.0f}s")
    return name, tlog, metrics


def scenario_loiter_with_drift():
    """Scenario 11: Long-duration loiter with changing current direction."""
    name = "Loiter with rotating current (20 min)"
    boat = Boat(BASE_LAT + 0.002, BASE_LON + 0.002, 0)
    center = (boat.lat, boat.lon)
    tlog = TelemetryLog()
    max_drift = 0
    drifts = []

    dt = 0.1
    for i in range(12000):  # 20 min
        t = i * dt
        # Rotating current: 0.4 m/s, direction rotates 360° over 10 minutes
        current_dir = (t / 600.0) * 360.0  # full rotation in 10 min
        boat.set_current(0.4, current_dir)

        thr, steer = boat.autopilot_loiter(center, dt)
        boat.step(thr, steer, dt)
        drift = boat.distance_to(*center)
        if drift > max_drift:
            max_drift = drift
        drifts.append(drift)
        if i % 10 == 0:
            tlog.record(t, boat, {'drift_m': round(drift, 2), 'current_dir': round(current_dir % 360, 0)})

    avg_drift = sum(drifts) / len(drifts)
    metrics = {'max_drift': round(max_drift, 1), 'avg_drift': round(avg_drift, 2), 'battery_used': round(92 - boat.battery, 1)}
    log.info(f"[11] {name}: max_drift={max_drift:.1f}m avg={avg_drift:.1f}m")
    return name, tlog, metrics


def scenario_full_mission_relay():
    """Scenario 12: Full relay mission - transit to station, hold as relay, RTL."""
    name = "Full relay mission (transit -> relay 15min -> RTL)"
    boat = Boat(BASE_LAT, BASE_LON, 60)
    home = (BASE_LAT, BASE_LON)
    relay_pos = (BASE_LAT + 0.003, BASE_LON + 0.004)
    boat.waypoints = [relay_pos]
    boat.set_current(0.3, 210)
    tlog = TelemetryLog()
    phase = 'transit'
    hold_start = 0
    metrics = {}

    dt = 0.1
    for i in range(18000):  # 30 min max
        t = i * dt

        if phase == 'transit':
            thr, steer, done = boat.autopilot_to_wp(dt)
            if done:
                phase = 'relay'
                hold_start = t
                boat.mode = 'LOITER'
                metrics['transit_time'] = round(t, 1)
                log.info(f"  On station at {t:.0f}s")
        elif phase == 'relay':
            thr, steer = boat.autopilot_loiter(relay_pos, dt)
            if t - hold_start > 900:  # 15 minutes relay
                phase = 'rtl'
                boat.waypoints = [home]
                boat.wp_idx = 0
                boat.mode = 'RTL'
                metrics['relay_time'] = 900
                log.info(f"  RTL at {t:.0f}s")
        elif phase == 'rtl':
            thr, steer, done = boat.autopilot_to_wp(dt)
            if done:
                metrics['rtl_time'] = round(t - (metrics.get('transit_time', 0) + 900), 1)
                break
        else:
            thr, steer = 0, 0

        boat.step(thr, steer, dt)
        if i % 10 == 0:
            tlog.record(t, boat, {'phase': phase})

    metrics['total_time'] = round(i * dt, 1)
    metrics['battery_remaining'] = round(boat.battery, 1)
    metrics['final_dist_home'] = round(boat.distance_to(*home), 1)
    log.info(f"[12] {name}: total={metrics['total_time']:.0f}s batt={metrics['battery_remaining']:.1f}% home={metrics['final_dist_home']:.1f}m")
    return name, tlog, metrics


# == Report ====================================================

def print_report(results):
    print("\n" + "=" * 70)
    print("  VANGUARD USV SIMULATION REPORT")
    print("  " + time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 70)

    for i, (name, tlog, metrics) in enumerate(results, 1):
        print(f"\n{'=' * 60}")
        print(f"  Scenario {i}: {name}")
        print(f"{'=' * 60}")
        print(f"  Telemetry entries: {len(tlog.entries)}")
        for k, v in metrics.items():
            print(f"  {k}: {v}")

    print(f"\n{'=' * 70}")
    print(f"  {len(results)} scenarios complete")

    # Pass/fail summary
    print(f"\n  PASS/FAIL SUMMARY:")
    passes = 0
    for i, (name, tlog, metrics) in enumerate(results, 1):
        status = "PASS"
        if metrics.get('max_drift', 0) > 15:
            status = "FAIL (drift > 15m)"
        if metrics.get('max_crosstrack', 0) > 20:
            status = "FAIL (crosstrack > 20m)"
        if metrics.get('final_dist_home', 0) > 10 and 'rtl' in name.lower():
            status = "FAIL (didn't reach home)"
        if metrics.get('waypoints_reached', 99) < metrics.get('total_waypoints', 0):
            status = "FAIL (incomplete mission)"
        if status == "PASS":
            passes += 1
        print(f"    [{i:2d}] {status:30s} {name}")

    print(f"\n  {passes}/{len(results)} scenarios PASSED")
    print("=" * 70)


# == Replay via WebSocket ======================================

async def replay_scenario(scenario_num, ws_port=5760):
    """Replay a scenario log through WebSocket for GCS visualization."""
    try:
        import websockets
    except ImportError:
        log.error("pip install websockets")
        return

    logfile = f"results/scenario_{scenario_num}.json"
    if not os.path.exists(logfile):
        log.error(f"No log file: {logfile}. Run the scenario first.")
        return

    with open(logfile) as f:
        entries = json.load(f)

    log.info(f"Replaying scenario {scenario_num}: {len(entries)} entries on ws://0.0.0.0:{ws_port}")

    # Import COBS from the SITL server
    def cobs_encode(data):
        out = bytearray([0])
        code_idx, code = 0, 1
        for b in data:
            if b == 0:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
            else:
                out.append(b)
                code += 1
                if code == 0xFF:
                    out[code_idx] = code
                    code_idx = len(out)
                    out.append(0)
                    code = 1
        out[code_idx] = code
        out.append(0)
        return bytes(out)

    VCLASS_BOAT = 3 << 4
    clients = set()

    async def handle(ws):
        clients.add(ws)
        log.info("GCS connected for replay")
        try:
            async for _ in ws:
                pass
        except:
            pass
        finally:
            clients.discard(ws)

    async def broadcast(data):
        if clients:
            await asyncio.gather(*[ws.send(data) for ws in clients], return_exceptions=True)

    async def play():
        await asyncio.sleep(2)  # wait for GCS connect
        for i, e in enumerate(entries):
            # Build MNP messages from log entry
            mode_map = {'AUTO': 4, 'LOITER': 2, 'HOLD': 2, 'RTL': 3, 'MANUAL': 0}
            mode_idx = mode_map.get(e.get('mode', 'AUTO'), 0)
            armed = 1

            # Heartbeat
            hb = struct.pack("<BBB", 0x01, armed, mode_idx)
            hb += struct.pack("<B", (VCLASS_BOAT) | 4)
            await broadcast(cobs_encode(hb))

            # Position
            pos = struct.pack("<B iiii hhh H", 0x03,
                int(e['lat'] * 1e7), int(e['lon'] * 1e7), 0, 0,
                0, 0, 0, int(e['hdg'] * 100))
            await broadcast(cobs_encode(pos))

            # VFR HUD
            hud = struct.pack("<B ff h H ff", 0x06,
                e['spd'], e['spd'], int(e['hdg']), int(e.get('thrust', 0) / 4.5),
                0.0, 0.0)
            await broadcast(cobs_encode(hud))

            # Battery
            batt = struct.pack("<B hh B", 0x04,
                int(e['volt'] * 1000), int(18 * 100), int(e['batt']))
            await broadcast(cobs_encode(batt))

            # Pace: 10 entries per second (10x real-time for 1Hz logs)
            if i < len(entries) - 1:
                dt = entries[i + 1]['t'] - e['t']
                await asyncio.sleep(min(dt * 0.1, 0.2))  # 10x speed, cap at 200ms

            if i % 50 == 0:
                log.info(f"  Replay: {i}/{len(entries)} t={e['t']:.0f}s")

        log.info("Replay complete")

    async with websockets.serve(handle, "0.0.0.0", ws_port):
        await play()
        await asyncio.sleep(5)  # keep open a bit


# == Main ======================================================

def main():
    parser = argparse.ArgumentParser(description="Vanguard USV simulation suite")
    parser.add_argument("--scenario", "-s", type=int, help="Run specific scenario (1-12)")
    parser.add_argument("--replay", "-r", type=int, help="Replay scenario on GCS (ws://localhost:5760)")
    parser.add_argument("--report", action="store_true", help="Print results report")
    parser.add_argument("--all", "-a", action="store_true", help="Run all scenarios")
    args = parser.parse_args()

    os.makedirs("results", exist_ok=True)

    if args.replay:
        asyncio.run(replay_scenario(args.replay))
        return

    if args.scenario:
        scenarios = [args.scenario]
    elif args.all or not args.report:
        scenarios = list(range(1, 13))
    else:
        scenarios = []

    results = []
    for num in scenarios:
        result = run_scenario(num)
        if result:
            name, tlog, metrics = result
            tlog.save(f"results/scenario_{num}.json")
            results.append(result)

    if results or args.report:
        # Load any existing results for report
        if not results and args.report:
            for num in range(1, 13):
                logfile = f"results/scenario_{num}.json"
                if os.path.exists(logfile):
                    with open(logfile) as f:
                        entries = json.load(f)
                    results.append((f"Scenario {num}", type('', (), {'entries': entries})(), {}))

        if results:
            print_report(results)


if __name__ == "__main__":
    main()
