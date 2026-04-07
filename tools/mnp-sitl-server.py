#!/usr/bin/env python3
"""
mnp-sitl-server.py — SITL jet boat simulator with MNP WebSocket server.

Runs a simplified jet boat physics simulation and serves MNP telemetry
over WebSocket so the Meridian GCS can connect and fly a mission.

This is the full end-to-end demo:
    [Meridian GCS] ←MNP/WebSocket→ [This SITL] ←physics→ [Simulated Jet Boat]

Usage:
    python tools/mnp-sitl-server.py
    # Then open GCS at http://localhost:8080, connect to ws://localhost:5760

The SITL simulates:
    - Jet boat at a fixed starting position
    - GPS, compass, IMU telemetry
    - Two-waypoint A→B mission
    - Water current from the southwest
    - MNP framing (COBS-encoded messages over WebSocket)
"""

import asyncio
import json
import math
import struct
import time
import logging

try:
    import websockets
except ImportError:
    print("ERROR: websockets required. Install with: pip install websockets")
    exit(1)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("mnp-sitl")

# ─── COBS Encoding ──────────────────────────────────────────

def cobs_encode(data: bytes) -> bytes:
    """COBS encode a byte sequence (no zero bytes in output)."""
    output = bytearray()
    code_idx = 0
    output.append(0)  # placeholder for first code byte
    code = 1

    for byte in data:
        if byte == 0:
            output[code_idx] = code
            code_idx = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_idx] = code
                code_idx = len(output)
                output.append(0)
                code = 1

    output[code_idx] = code
    output.append(0)  # frame delimiter
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    """COBS decode a byte sequence."""
    output = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        if code == 0:
            break
        idx += 1
        for _ in range(code - 1):
            if idx < len(data):
                output.append(data[idx])
                idx += 1
        if code < 0xFF and idx < len(data):
            output.append(0)
    if output and output[-1] == 0:
        output = output[:-1]
    return bytes(output)


# ─── Jet Boat Physics ───────────────────────────────────────

class JetBoatSim:
    def __init__(self, lat, lon, heading_deg):
        # Position
        self.lat = lat
        self.lon = lon
        # Velocity
        self.speed = 0.0           # forward speed m/s
        self.heading = math.radians(heading_deg)
        self.yaw_rate = 0.0        # rad/s
        # Engine
        self.thrust = 0.0          # actual thrust (spooled)
        self.nozzle_angle = 0.0    # current nozzle deflection rad
        # Params
        self.mass = 30.0
        self.max_thrust = 50.0
        self.max_nozzle = math.radians(25)
        self.nozzle_arm = 0.6
        self.drag = 15.0
        self.yaw_drag = 8.0
        self.yaw_inertia = 3.0
        self.spool_tc = 0.8
        # Current
        self.current_n = 0.3       # 0.3 m/s from southwest
        self.current_e = 0.3
        # Telemetry
        self.armed = False
        self.mode = "MANUAL"
        self.battery_pct = 85.0
        self.voltage = 24.2
        self.satellites = 14
        self.hdop = 0.9
        # Mission
        self.waypoints = []
        self.current_wp = 0
        self.mission_active = False

    def step(self, throttle_cmd, steering_cmd, dt):
        # Spool up
        target_thrust = max(0, min(1, throttle_cmd)) * self.max_thrust
        alpha = dt / (self.spool_tc + dt)
        self.thrust += alpha * (target_thrust - self.thrust)

        # Nozzle
        target_nozzle = max(-1, min(1, steering_cmd)) * self.max_nozzle
        nozzle_rate = 2.0 * dt
        delta = target_nozzle - self.nozzle_angle
        if abs(delta) > nozzle_rate:
            self.nozzle_angle += nozzle_rate if delta > 0 else -nozzle_rate
        else:
            self.nozzle_angle = target_nozzle

        # Forces
        thrust_fwd = self.thrust * math.cos(self.nozzle_angle)
        thrust_lat = self.thrust * math.sin(self.nozzle_angle)
        drag_fwd = -self.drag * self.speed * abs(self.speed)
        yaw_torque = thrust_lat * self.nozzle_arm
        yaw_drag_t = -self.yaw_drag * self.yaw_rate * abs(self.yaw_rate)

        # Integration
        accel = (thrust_fwd + drag_fwd) / self.mass
        self.speed += accel * dt
        self.yaw_rate += (yaw_torque + yaw_drag_t) / self.yaw_inertia * dt
        self.heading += self.yaw_rate * dt
        self.heading = self.heading % (2 * math.pi)

        # Position (including current)
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        self.lat += (vn * dt) / 6371000.0 * (180.0 / math.pi)
        self.lon += (ve * dt) / (6371000.0 * math.cos(math.radians(self.lat))) * (180.0 / math.pi)

        # Battery drain
        self.battery_pct = max(0, self.battery_pct - 0.0005 * dt * (throttle_cmd + 0.1))
        self.voltage = 22.0 + self.battery_pct / 100.0 * 3.0

    def run_autopilot(self, dt):
        """Simple autopilot: navigate to current waypoint."""
        if not self.mission_active or self.current_wp >= len(self.waypoints):
            return 0.0, 0.0

        wp = self.waypoints[self.current_wp]
        # Distance and bearing to waypoint
        dlat = wp[0] - self.lat
        dlon = wp[1] - self.lon
        dist = math.sqrt((dlat * 111320)**2 + (dlon * 111320 * math.cos(math.radians(self.lat)))**2)
        bearing = math.atan2(dlon * math.cos(math.radians(self.lat)), dlat)

        # Check waypoint reached
        if dist < 5.0:  # 5m acceptance radius
            self.current_wp += 1
            if self.current_wp >= len(self.waypoints):
                self.mission_active = False
                log.info("Mission complete!")
                return 0.0, 0.0
            log.info(f"Waypoint {self.current_wp} reached, navigating to WP {self.current_wp + 1}")
            return 0.0, 0.0

        # Heading error
        heading_error = bearing - self.heading
        while heading_error > math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi

        # P controller for heading
        steering = max(-1, min(1, heading_error * 1.5))
        # Throttle based on distance
        throttle = min(0.6, dist * 0.05)

        return throttle, steering

    def heading_deg(self):
        return (math.degrees(self.heading) % 360 + 360) % 360

    def ground_speed(self):
        vn = self.speed * math.cos(self.heading) + self.current_n
        ve = self.speed * math.sin(self.heading) + self.current_e
        return math.sqrt(vn**2 + ve**2)


# ─── MNP Message Builder ────────────────────────────────────

def build_heartbeat(boat):
    """Build an MNP heartbeat-like telemetry message."""
    # Message type 1: Telemetry bundle
    # Format: type(1) + lat(f64) + lon(f64) + alt(f32) + heading(f32) + speed(f32) +
    #         yaw_rate(f32) + battery_pct(f32) + voltage(f32) + mode(u8) + armed(u8) +
    #         satellites(u8) + hdop(f32) + nozzle_angle(f32) + thrust(f32)
    mode_id = {"MANUAL": 0, "HOLD": 1, "AUTO": 3, "RTL": 6, "LOITER": 5}.get(boat.mode, 0)
    payload = struct.pack("<B dd ffff ff BB B ff",
        1,  # message type: telemetry
        boat.lat, boat.lon,
        0.0,  # alt (surface vehicle)
        boat.heading_deg(), boat.ground_speed(),
        boat.yaw_rate, boat.battery_pct,
        boat.voltage, mode_id, 1 if boat.armed else 0,
        boat.satellites, boat.hdop,
        math.degrees(boat.nozzle_angle), boat.thrust,
    )
    return cobs_encode(payload)


# ─── WebSocket Server ────────────────────────────────────────

class MnpSitlServer:
    def __init__(self):
        # Start position: Sydney Harbour
        self.boat = JetBoatSim(-33.8568, 151.2153, 45.0)
        self.clients = set()
        self.tick = 0

        # Set up A→B mission
        self.boat.waypoints = [
            (-33.8555, 151.2170),  # Point A: ~200m northeast
            (-33.8540, 151.2190),  # Point B: ~400m northeast
        ]

    async def handle_client(self, websocket):
        self.clients.add(websocket)
        remote = websocket.remote_address
        log.info(f"GCS connected: {remote[0]}:{remote[1]}")

        try:
            async for message in websocket:
                if isinstance(message, bytes):
                    self.handle_command(message)
                elif isinstance(message, str):
                    self.handle_text_command(message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            log.info(f"GCS disconnected")

    def handle_command(self, data):
        """Handle incoming MNP command from GCS."""
        try:
            decoded = cobs_decode(data)
            if len(decoded) < 1:
                return
            msg_type = decoded[0]
            if msg_type == 10:  # Arm command
                self.boat.armed = True
                log.info("Vehicle ARMED")
            elif msg_type == 11:  # Disarm command
                self.boat.armed = False
                self.boat.mission_active = False
                log.info("Vehicle DISARMED")
            elif msg_type == 20:  # Start mission
                self.boat.mission_active = True
                self.boat.current_wp = 0
                self.boat.mode = "AUTO"
                log.info(f"Mission started: {len(self.boat.waypoints)} waypoints")
            elif msg_type == 21:  # Set mode
                if len(decoded) > 1:
                    modes = {0: "MANUAL", 1: "HOLD", 3: "AUTO", 5: "LOITER", 6: "RTL"}
                    self.boat.mode = modes.get(decoded[1], "MANUAL")
                    log.info(f"Mode: {self.boat.mode}")
        except Exception as e:
            log.debug(f"Command parse error: {e}")

    def handle_text_command(self, text):
        """Handle text commands (for easy GCS testing)."""
        text = text.strip().lower()
        if text == "arm":
            self.boat.armed = True
            log.info("Armed via text command")
        elif text == "disarm":
            self.boat.armed = False
            log.info("Disarmed via text command")
        elif text == "auto":
            self.boat.mission_active = True
            self.boat.current_wp = 0
            self.boat.mode = "AUTO"
            log.info("Auto mode via text command")
        elif text == "hold":
            self.boat.mode = "HOLD"
            self.boat.mission_active = False
            log.info("Hold mode via text command")

    async def sim_loop(self):
        """Main simulation loop at 50Hz."""
        dt = 0.02  # 50Hz
        while True:
            # Run autopilot if in AUTO mode
            if self.boat.armed and self.boat.mission_active:
                throttle, steering = self.boat.run_autopilot(dt)
                self.boat.step(throttle, steering, dt)
            elif self.boat.armed:
                self.boat.step(0.0, 0.0, dt)
            # else: stationary

            self.tick += 1

            # Send telemetry at 10Hz (every 5th tick)
            if self.tick % 5 == 0 and self.clients:
                msg = build_heartbeat(self.boat)
                await asyncio.gather(
                    *[ws.send(msg) for ws in self.clients],
                    return_exceptions=True
                )

            # Log status every 5 seconds
            if self.tick % 250 == 0:
                b = self.boat
                wp_str = f"WP {b.current_wp + 1}/{len(b.waypoints)}" if b.mission_active else "idle"
                log.info(
                    f"[{b.mode}] hdg={b.heading_deg():.0f}° spd={b.ground_speed():.1f}m/s "
                    f"batt={b.battery_pct:.0f}% nozzle={math.degrees(b.nozzle_angle):.1f}° "
                    f"thrust={b.thrust:.0f}N {wp_str} "
                    f"pos=({b.lat:.6f}, {b.lon:.6f})"
                )

            await asyncio.sleep(dt)

    async def run(self, host="0.0.0.0", port=5760):
        log.info(f"MNP SITL Jet Boat Server")
        log.info(f"Position: Sydney Harbour ({self.boat.lat:.4f}, {self.boat.lon:.4f})")
        log.info(f"Mission: {len(self.boat.waypoints)} waypoints (A → B)")
        log.info(f"Current: 0.42 m/s from SW")
        log.info(f"WebSocket: ws://{host}:{port}")
        log.info(f"")
        log.info(f"Commands: 'arm', 'disarm', 'auto', 'hold' (send as text over WS)")
        log.info(f"Or connect Meridian GCS → Settings → Connection → ws://localhost:{port}")

        async with websockets.serve(self.handle_client, host, port):
            await self.sim_loop()


def main():
    server = MnpSitlServer()
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        log.info("Server stopped.")


if __name__ == "__main__":
    main()
