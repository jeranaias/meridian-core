"""Signal K client.

Connects to a Signal K server (e.g., Orca's Signal K endpoint or an
OpenPlotter node) via WebSocket and streams deltas. Maps Signal K paths to
our BoatState structures.

Signal K is JSON-based, so this is simpler than the N2K/CAN path — we just
parse JSON messages and route by path.

Subscription strategy: we subscribe to the paths we care about, no more.

Reference paths:
  navigation.position                 →  {latitude, longitude, altitude}
  navigation.courseOverGroundTrue     →  radians
  navigation.speedOverGround          →  m/s
  navigation.headingTrue              →  radians
  navigation.rateOfTurn               →  rad/s
  environment.depth.belowTransducer   →  m
  environment.wind.speedTrue          →  m/s
  environment.wind.directionTrue      →  radians
"""
import asyncio
import json
import logging
from dataclasses import dataclass
from typing import Optional

try:
    import websockets
except ImportError:
    websockets = None

from ..state import BoatState


log = logging.getLogger(__name__)


SUBSCRIBE_MSG = {
    "context": "vessels.self",
    "subscribe": [
        {"path": "navigation.position",                "period": 100},
        {"path": "navigation.courseOverGroundTrue",    "period": 100},
        {"path": "navigation.speedOverGround",         "period": 100},
        {"path": "navigation.headingTrue",             "period": 100},
        {"path": "navigation.rateOfTurn",              "period": 200},
        {"path": "environment.depth.belowTransducer",  "period": 500},
        {"path": "environment.wind.speedTrue",         "period": 1000},
        {"path": "environment.wind.directionTrue",     "period": 1000},
    ],
}


@dataclass
class SignalKConfig:
    url: str = "ws://orca.local/signalk/v1/stream"
    subscribe: str = "none"          # "none" + explicit subscribe message
    auth_token: Optional[str] = None


class SignalKClient:
    """Async Signal K WebSocket client."""

    def __init__(self, cfg: SignalKConfig, state: BoatState) -> None:
        self.cfg = cfg
        self.state = state
        self._stop = asyncio.Event()

    async def run(self) -> None:
        if websockets is None:
            raise RuntimeError("websockets package not installed")
        url = self.cfg.url
        if "?" not in url:
            url += "?subscribe=" + self.cfg.subscribe
        else:
            url += "&subscribe=" + self.cfg.subscribe

        extra_headers = {}
        if self.cfg.auth_token:
            extra_headers["Authorization"] = f"Bearer {self.cfg.auth_token}"

        log.info("Signal K: connecting to %s", url)
        backoff = 1.0
        while not self._stop.is_set():
            try:
                async with websockets.connect(
                    url,
                    additional_headers=extra_headers,
                    ping_interval=20, ping_timeout=10,
                    close_timeout=3,
                ) as ws:
                    log.info("Signal K connected")
                    backoff = 1.0
                    await ws.send(json.dumps(SUBSCRIBE_MSG))
                    async for raw in ws:
                        self._handle_message(raw)
            except Exception as e:
                log.warning("Signal K connection error: %s", e)
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 30.0)

    def stop(self) -> None:
        self._stop.set()

    # --- message handling ---------------------------------------------------
    def _handle_message(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except Exception:
            return
        self.state.packets_seen += 1
        if "updates" not in msg:
            return
        any_decoded = False
        for upd in msg["updates"]:
            for val in upd.get("values", []):
                if self._apply_delta(val.get("path"), val.get("value")):
                    any_decoded = True
        if any_decoded:
            self.state.packets_decoded += 1

    def _apply_delta(self, path: str, value) -> bool:
        import math
        if path == "navigation.position":
            if isinstance(value, dict):
                self.state.update_gps(
                    lat=float(value.get("latitude", 0.0)),
                    lon=float(value.get("longitude", 0.0)),
                    alt_m=float(value.get("altitude", 0.0)),
                    fix_type=3,
                )
                return True
        elif path == "navigation.courseOverGroundTrue":
            self.state.update_gps(cog_deg=math.degrees(float(value)) % 360)
            return True
        elif path == "navigation.speedOverGround":
            self.state.update_gps(sog_mps=float(value))
            return True
        elif path == "navigation.headingTrue":
            self.state.update_heading(true_rad=float(value))
            return True
        elif path == "navigation.rateOfTurn":
            self.state.update_heading(rate_rad_s=float(value))
            return True
        elif path == "environment.depth.belowTransducer":
            self.state.update_depth(meters_below_transducer=float(value))
            return True
        elif path == "environment.wind.speedTrue":
            self.state.update_wind(speed_mps=float(value))
            return True
        elif path == "environment.wind.directionTrue":
            self.state.update_wind(direction_rad=float(value), reference="true")
            return True
        return False
