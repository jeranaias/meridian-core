#!/usr/bin/env python3
"""
mnp-sitl-server.py — Jet boat SITL with MNP over WebSocket.

Runs jet boat physics and sends MNP telemetry that the Meridian GCS
understands natively. No MAVLink. Pure MNP.

Usage:
    pip install websockets
    python tools/mnp-sitl-server.py
    # GCS: http://localhost:8080 → connect ws://localhost:5760 (protocol: mnp)
"""

import asyncio
import math
import struct
import logging

try:
    import websockets
except ImportError:
    print("pip install websockets")
    exit(1)

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("sitl")

# ── MNP Message IDs (must match gcs/js/mnp.js MSG enum) ──────

MSG_HEARTBEAT   = 0x01
MSG_ATTITUDE    = 0x02
MSG_POSITION    = 0x03
MSG_BATTERY     = 0x04
MSG_GPS_RAW     = 0x05
MSG_VFR_HUD     = 0x06
MSG_EKF_STATUS  = 0x07

# Commands from GCS
CMD_ARM         = 0x80
CMD_DISARM      = 0x81
CMD_SET_MODE    = 0x85

# Modes (index must match mnp.js MODES array)
MODE_STABILIZE = 0  # Manual
MODE_ALT_HOLD  = 1
MODE_LOITER    = 2
MODE_RTL       = 3
MODE_AUTO      = 4
MODE_LAND      = 5
MODE_GUIDED    = 6

# ── COBS ──────────────────────────────────────────────────────

def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    out.append(0)
    code_idx = 0
    code = 1
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
    out.append(0)  # delimiter
    return bytes(out)

def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0:
            break
        i += 1
        for _ in range(code - 1):
            if i < len(data):
                out.append(data[i])
                i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    if out and out[-1] == 0:
        out = out[:-1]
    return bytes(out)

# ── MNP Message Builders ─────────────────────────────────────
# Each must match the parser in gcs/js/mnp.js exactly.

# Vehicle class in top 4 bits of status byte:
# 0=copter, 1=plane, 2=rover, 3=boat, 4=sub
VCLASS_BOAT = 3 << 4

def mnp_heartbeat(armed: bool, mode_idx: int, status: int = 4, vehicle_class: int = VCLASS_BOAT) -> bytes:
    payload = struct.pack("<BBB", MSG_HEARTBEAT, 1 if armed else 0, mode_idx)
    payload += struct.pack("<B", (vehicle_class & 0xF0) | (status & 0x0F))
    return cobs_encode(payload)

def mnp_attitude(roll: float, pitch: float, yaw: float,
                 roll_spd: float = 0, pitch_spd: float = 0, yaw_spd: float = 0) -> bytes:
    payload = struct.pack("<B ffffff", MSG_ATTITUDE, roll, pitch, yaw, roll_spd, pitch_spd, yaw_spd)
    return cobs_encode(payload)

def mnp_position(lat: float, lon: float, alt: float, rel_alt: float,
                 vx: float, vy: float, vz: float, hdg: float) -> bytes:
    # lat/lon in degE7, alt in mm, vel in cm/s, hdg in cdeg
    payload = struct.pack("<B iiii hhh H", MSG_POSITION,
        int(lat * 1e7), int(lon * 1e7), int(alt * 1000), int(rel_alt * 1000),
        int(vx * 100), int(vy * 100), int(vz * 100), int(hdg * 100))
    return cobs_encode(payload)

def mnp_battery(voltage: float, current: float, remaining: int) -> bytes:
    # voltage in mV, current in cA, remaining in %
    payload = struct.pack("<B hh B", MSG_BATTERY,
        int(voltage * 1000), int(current * 100), max(0, min(100, remaining)))
    return cobs_encode(payload)

def mnp_gps_raw(fix: int, lat: float, lon: float, alt: float,
                hdop: int, vdop: int, sats: int) -> bytes:
    payload = struct.pack("<B B iii HH B", MSG_GPS_RAW,
        fix, int(lat * 1e7), int(lon * 1e7), int(alt * 1000),
        hdop, vdop, sats)
    return cobs_encode(payload)

def mnp_vfr_hud(airspeed: float, groundspeed: float, heading: int,
                throttle: int, alt: float, climb: float) -> bytes:
    payload = struct.pack("<B ff h H ff", MSG_VFR_HUD,
        airspeed, groundspeed, heading, throttle, alt, climb)
    return cobs_encode(payload)

def mnp_ekf(vel_var: float, pos_var: float, hgt_var: float,
            mag_var: float, terr_var: float, flags: int) -> bytes:
    payload = struct.pack("<B fffff H", MSG_EKF_STATUS,
        vel_var, pos_var, hgt_var, mag_var, terr_var, flags)
    return cobs_encode(payload)

# ── Jet Boat Physics ─────────────────────────────────────────

class JetBoat:
    def __init__(self, lat, lon, heading_deg):
        self.lat = lat
        self.lon = lon
        self.speed = 0.0
        self.heading = math.radians(heading_deg)
        self.yaw_rate = 0.0
        self.thrust = 0.0
        self.nozzle = 0.0
        self.armed = False
        self.mode = MODE_STABILIZE
        self.battery = 92.0
        self.voltage = 25.1
        self.current_n = 0.2  # mild current
        self.current_e = 0.15
        self.waypoints = []
        self.wp_idx = 0
        self.loiter_center = None

    def step(self, throttle, steering, dt):
        if not self.armed:
            self.speed *= 0.95
            self.yaw_rate *= 0.9
            return

        # Spool
        target = max(0, min(1, throttle)) * 50.0
        self.thrust += (dt / 0.8) * (target - self.thrust)
        # Nozzle
        target_n = max(-1, min(1, steering)) * math.radians(25)
        rate = 2.0 * dt
        d = target_n - self.nozzle
        self.nozzle += max(-rate, min(rate, d))
        # Forces
        fwd = self.thrust * math.cos(self.nozzle) - 15.0 * self.speed * abs(self.speed)
        torque = self.thrust * math.sin(self.nozzle) * 0.6 - 8.0 * self.yaw_rate * abs(self.yaw_rate)
        self.speed += fwd / 30.0 * dt
        self.yaw_rate += torque / 3.0 * dt
        self.heading += self.yaw_rate * dt
        self.heading %= 2 * math.pi
        # Position
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        self.lat += vn * dt / 6371000.0 * (180 / math.pi)
        self.lon += ve * dt / (6371000.0 * math.cos(math.radians(self.lat))) * (180 / math.pi)
        # Battery
        self.battery = max(0, self.battery - 0.0003 * dt * (throttle + 0.1))
        self.voltage = 22.0 + self.battery / 100.0 * 3.2

    def autopilot(self, dt):
        if self.mode == MODE_AUTO and self.wp_idx < len(self.waypoints):
            wp = self.waypoints[self.wp_idx]
            dlat = wp[0] - self.lat
            dlon = wp[1] - self.lon
            dist = math.sqrt((dlat * 111320)**2 + (dlon * 111320 * math.cos(math.radians(self.lat)))**2)
            bearing = math.atan2(dlon * math.cos(math.radians(self.lat)), dlat)
            if dist < 5.0:
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    # Mission done — loiter at last WP
                    self.loiter_center = (self.lat, self.lon)
                    self.mode = MODE_LOITER
                    log.info("Mission complete — loitering")
                    return 0.1, 0.0
                log.info(f"WP {self.wp_idx} reached → WP {self.wp_idx + 1}")
            err = bearing - self.heading
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            return min(0.6, dist * 0.04), max(-1, min(1, err * 1.5))

        elif self.mode == MODE_LOITER and self.loiter_center:
            cx, cy = self.loiter_center
            dlat = cx - self.lat
            dlon = cy - self.lon
            dist = math.sqrt((dlat * 111320)**2 + (dlon * 111320 * math.cos(math.radians(self.lat)))**2)
            if dist < 3.0:
                return 0.05, 0.0
            bearing = math.atan2(dlon * math.cos(math.radians(self.lat)), dlat)
            err = bearing - self.heading
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            return min(0.35, dist * 0.06), max(-1, min(1, err * 1.2))

        return 0.0, 0.0

    def hdg_deg(self):
        return (math.degrees(self.heading) % 360 + 360) % 360

    def gnd_speed(self):
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        return math.sqrt(vn * vn + ve * ve)

    def vel_ned(self):
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        return vn, ve

# ── Server ────────────────────────────────────────────────────

class SitlServer:
    def __init__(self):
        # Middle of Botany Bay, Sydney — actually in the water
        self.boat = JetBoat(-33.9700, 151.2000, 45.0)
        # No hardcoded waypoints — user places them via GCS Plan tab
        self.boat.waypoints = []
        self.clients = set()
        self.tick = 0

    async def handle_client(self, ws):
        self.clients.add(ws)
        log.info(f"GCS connected ({len(self.clients)} clients)")
        try:
            async for msg in ws:
                if isinstance(msg, bytes):
                    self.handle_cmd(msg)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)
            log.info(f"GCS disconnected ({len(self.clients)} clients)")

    def handle_cmd(self, data):
        try:
            payload = cobs_decode(data)
            if len(payload) < 1:
                return
            cmd = payload[0]
            if cmd == CMD_ARM:
                self.boat.armed = True
                log.info("ARMED")
            elif cmd == CMD_DISARM:
                self.boat.armed = False
                log.info("DISARMED")
            elif cmd == CMD_SET_MODE:
                if len(payload) > 1:
                    self.boat.mode = payload[1]
                    names = {0: 'MANUAL', 2: 'LOITER', 3: 'RTL', 4: 'AUTO'}
                    log.info(f"Mode → {names.get(self.boat.mode, self.boat.mode)}")
                    if self.boat.mode == MODE_AUTO:
                        self.boat.wp_idx = 0
                        self.boat.loiter_center = None
                        log.info(f"Mission: {len(self.boat.waypoints)} waypoints")
            elif cmd == 0x8A:  # CMD_MISSION_COUNT
                if len(payload) >= 3:
                    count = struct.unpack("<H", payload[1:3])[0]
                    self.boat.waypoints = [None] * count
                    log.info(f"Mission upload: expecting {count} waypoints")
            elif cmd == 0x89:  # CMD_MISSION_ITEM
                if len(payload) >= 13:
                    seq = struct.unpack("<H", payload[1:3])[0]
                    lat = struct.unpack("<i", payload[3:7])[0] / 1e7
                    lon = struct.unpack("<i", payload[7:11])[0] / 1e7
                    if seq < len(self.boat.waypoints):
                        self.boat.waypoints[seq] = (lat, lon)
                        log.info(f"  WP {seq+1}: ({lat:.6f}, {lon:.6f})")
                    # Check if mission is complete
                    if all(wp is not None for wp in self.boat.waypoints):
                        log.info(f"Mission upload complete: {len(self.boat.waypoints)} waypoints")
        except Exception as e:
            log.debug(f"cmd parse error: {e}")

    async def broadcast(self, data):
        if self.clients:
            await asyncio.gather(
                *[ws.send(data) for ws in self.clients],
                return_exceptions=True
            )

    async def sim_loop(self):
        dt = 0.02  # 50Hz
        while True:
            b = self.boat
            thr, steer = b.autopilot(dt)
            b.step(thr, steer, dt)
            self.tick += 1

            # 10Hz telemetry
            if self.tick % 5 == 0 and self.clients:
                vn, ve = b.vel_ned()
                mode_names = {0: 'STABILIZE', 2: 'LOITER', 3: 'RTL', 4: 'AUTO', 5: 'LAND', 6: 'GUIDED'}

                await self.broadcast(mnp_heartbeat(b.armed, b.mode))
                await self.broadcast(mnp_attitude(0.0, 0.0, b.heading, 0.0, 0.0, b.yaw_rate))
                await self.broadcast(mnp_position(b.lat, b.lon, 0.0, 0.0, vn, ve, 0.0, b.hdg_deg()))
                await self.broadcast(mnp_battery(b.voltage, 18.0 if b.armed else 0.2, int(b.battery)))
                await self.broadcast(mnp_gps_raw(3, b.lat, b.lon, 0.0, 90, 120, 14))
                await self.broadcast(mnp_vfr_hud(b.gnd_speed(), b.gnd_speed(), int(b.hdg_deg()), int(thr * 100), 0.0, 0.0))
                await self.broadcast(mnp_ekf(0.05, 0.08, 0.03, 0.02, 0.01, 0x1FF))

            # Status every 5s
            if self.tick % 250 == 0:
                wp_str = f"WP {b.wp_idx+1}/{len(b.waypoints)}" if b.mode == MODE_AUTO else ("LOITER" if b.mode == MODE_LOITER else "idle")
                log.info(f"{'ARM' if b.armed else 'DSRM'} hdg={b.hdg_deg():.0f}° spd={b.gnd_speed():.1f}m/s batt={b.battery:.0f}% {wp_str} ({b.lat:.6f},{b.lon:.6f})")

            await asyncio.sleep(dt)

    async def run(self, port=5760):
        log.info("═══════════════════════════════════════════")
        log.info("  Meridian SITL — Jet Boat (MNP)")
        log.info(f"  Position: Botany Bay, Sydney (in the water)")
        log.info(f"  Mission: A → B then loiter")
        log.info(f"  WebSocket: ws://0.0.0.0:{port}")
        log.info("═══════════════════════════════════════════")
        log.info("Connect GCS → Settings → Connection → ws://localhost:5760")
        log.info("Then: arm → set mode AUTO → watch it go")
        log.info("")

        async with websockets.serve(self.handle_client, "0.0.0.0", port):
            await self.sim_loop()

if __name__ == "__main__":
    try:
        asyncio.run(SitlServer().run())
    except KeyboardInterrupt:
        log.info("Stopped.")
