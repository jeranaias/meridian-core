"""Synthetic N2K stream generator. Emits decoded values directly into a
BoatState so the MAVLink side can be developed without real Orca traffic.

Simulates a boat moving at the dock near Botany Bay, with a light breeze
and 5 m of water. Spawns AIS contacts periodically for testing the
multi-vessel rendering.
"""
import math
import random
import threading
import time

from .state import BoatState, AisContact


BOAT_LAT = -33.9700
BOAT_LON = 151.2000


class MockSource:
    """Drives a BoatState with plausible synthetic data. Run with .start()."""

    def __init__(self, state: BoatState, rng_seed: int = 0xC0FFEE) -> None:
        self.state = state
        self.rng = random.Random(rng_seed)
        self._stop = threading.Event()
        self._threads = []
        self._t0 = time.time()
        self._next_mmsi = 503_000_100

    def start(self) -> None:
        targets = [
            ("gps", self._gps_loop, 0.1),       # 10 Hz
            ("hdg", self._hdg_loop, 0.1),       # 10 Hz
            ("depth", self._depth_loop, 0.5),   # 2 Hz
            ("wind", self._wind_loop, 1.0),     # 1 Hz
            ("ais", self._ais_loop, 2.0),       # 0.5 Hz per contact
        ]
        for name, fn, dt in targets:
            t = threading.Thread(target=fn, args=(dt,), daemon=True, name=f"mock-{name}")
            t.start()
            self._threads.append(t)

    def stop(self) -> None:
        self._stop.set()

    # --- individual generators ----------------------------------------------
    def _gps_loop(self, dt: float) -> None:
        while not self._stop.is_set():
            t = time.time() - self._t0
            # Drift in a slow lissajous around the dock — ±5 m
            dlat = 5.0 * math.sin(t * 0.05) / 111320
            dlon = 5.0 * math.cos(t * 0.07) / (111320 * math.cos(math.radians(BOAT_LAT)))
            self.state.update_gps(
                lat=BOAT_LAT + dlat,
                lon=BOAT_LON + dlon,
                alt_m=0.0,
                sog_mps=0.25 + self.rng.uniform(-0.05, 0.05),
                cog_deg=(math.degrees(t * 0.05) + 45) % 360,
                hdop=0.9,
                vdop=1.1,
                fix_type=3,
                satellites=14 + self.rng.randint(-1, 1),
            )
            self.state.packets_seen += 1
            self.state.packets_decoded += 1
            if self._stop.wait(dt): return

    def _hdg_loop(self, dt: float) -> None:
        while not self._stop.is_set():
            t = time.time() - self._t0
            hdg = (math.radians(45) + 0.2 * math.sin(t * 0.3)) % (2 * math.pi)
            self.state.update_heading(
                true_rad=hdg,
                rate_rad_s=0.2 * 0.3 * math.cos(t * 0.3),
                mag_variation_rad=math.radians(12.4),  # east mag var near Sydney
            )
            self.state.packets_seen += 1
            self.state.packets_decoded += 1
            if self._stop.wait(dt): return

    def _depth_loop(self, dt: float) -> None:
        while not self._stop.is_set():
            t = time.time() - self._t0
            self.state.update_depth(
                meters_below_transducer=5.0 + 0.3 * math.sin(t * 0.15),
                offset_m=0.5,
                range_m=200.0,
            )
            self.state.packets_seen += 1
            self.state.packets_decoded += 1
            if self._stop.wait(dt): return

    def _wind_loop(self, dt: float) -> None:
        while not self._stop.is_set():
            t = time.time() - self._t0
            self.state.update_wind(
                speed_mps=6.0 + 1.0 * math.sin(t * 0.1),
                direction_rad=math.radians(225) + 0.1 * math.sin(t * 0.2),
                reference="true",
            )
            self.state.packets_seen += 1
            self.state.packets_decoded += 1
            if self._stop.wait(dt): return

    def _ais_loop(self, dt: float) -> None:
        """Maintain 3-5 synthetic AIS contacts moving around Botany Bay."""
        names = [
            ("MV TEST CARGO",  "VHTX", 70, 180, 28),
            ("FERRY COLLAROY", "VKCY", 60, 35, 9),
            ("F/V TESTING",    "VKSH", 30, 18, 5),
            ("TUG MOCKER",     "VKRL", 52, 28, 9),
        ]
        # Seed initial positions
        for name, call, ship_type, length, beam in names:
            c = AisContact(
                mmsi=self._next_mmsi,
                lat=BOAT_LAT + self.rng.uniform(-0.01, 0.01),
                lon=BOAT_LON + self.rng.uniform(-0.01, 0.01),
                cog_deg=self.rng.uniform(0, 360),
                sog_mps=self.rng.uniform(2.0, 6.0),
                heading_deg=0.0,
                nav_status=0,
                ship_type=ship_type,
                name=name,
                call_sign=call,
                length_m=length, beam_m=beam,
                class_letter="A" if ship_type >= 60 else "B",
            )
            c.heading_deg = c.cog_deg
            self.state.update_ais(c)
            self._next_mmsi += 1

        while not self._stop.is_set():
            # Advance each contact along its COG
            for mmsi, c in self.state.snapshot_ais().items():
                crad = math.radians(c.cog_deg)
                d_lat = c.sog_mps * dt * math.cos(crad) / 111320
                d_lon = c.sog_mps * dt * math.sin(crad) / (111320 * math.cos(math.radians(c.lat)))
                c2 = AisContact(**c.__dict__)
                c2.lat += d_lat
                c2.lon += d_lon
                c2.heading_deg = c2.cog_deg + self.rng.uniform(-1, 1)
                self.state.update_ais(c2)
            if self._stop.wait(dt): return
