"""meridian-orca-bridge daemon entry point.

Usage:
  python -m meridian_orca --mock --mavlink-out udpout:127.0.0.1:14550
  python -m meridian_orca --yd-udp 0.0.0.0:1457 --mavlink-out serial:COM10:115200
  python -m meridian_orca --signalk ws://orca.local/signalk/v1/stream --mavlink-out udpout:127.0.0.1:14550
"""
import argparse
import asyncio
import logging
import signal
import sys
import time

from .state import BoatState
from .mavlink.out import MavlinkOut, MavlinkConfig
from .mock import MockSource
from .protocol.yacht_devices_raw import YachtDevicesRawUdp, RawUdpConfig


def parse_args():
    p = argparse.ArgumentParser(
        prog="meridian-orca-bridge",
        description="Orca Core 2 N2K-over-Ethernet → MAVLink GPS_INPUT bridge",
    )
    src = p.add_mutually_exclusive_group()
    src.add_argument("--mock", action="store_true",
                     help="Internal synthetic source — no real Orca needed")
    src.add_argument("--yd-udp", metavar="HOST:PORT", default=None,
                     help="Yacht Devices RAW UDP listener")
    src.add_argument("--signalk", metavar="WS_URL", default=None,
                     help="Signal K WebSocket source")

    p.add_argument("--mavlink-out", metavar="ENDPOINT", default="udpout:127.0.0.1:14550",
                   help="MAVLink output endpoint")
    p.add_argument("--target-sys", type=int, default=1,
                   help="Target MAVLink system (Cube sysid, default 1)")
    p.add_argument("--no-gps", action="store_true", help="Don't emit GPS_INPUT")
    p.add_argument("--no-depth", action="store_true", help="Don't emit DISTANCE_SENSOR")
    p.add_argument("--no-wind", action="store_true", help="Don't emit WIND_COV")
    p.add_argument("--bathy", action="store_true",
                   help="Enable bathymetric map-matching (tertiary position source)")
    p.add_argument("--bathy-chart", metavar="PATH", default=None,
                   help="Path to .npz chart file. If omitted with --bathy, uses procedural realistic chart for demo.")

    p.add_argument("-v", "--verbose", action="count", default=0,
                   help="-v = INFO, -vv = DEBUG")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=[logging.WARNING, logging.INFO, logging.DEBUG][min(args.verbose, 2)],
        format="%(asctime)s %(name)-30s %(levelname)s %(message)s",
    )
    log = logging.getLogger("main")

    # --- state + outputs ----------------------------------------------------
    state = BoatState()

    mav_cfg = MavlinkConfig(
        endpoint=args.mavlink_out,
        target_system=args.target_sys,
        emit_gps_input=not args.no_gps,
        emit_distance_sensor=not args.no_depth,
        emit_wind_cov=not args.no_wind,
    )
    mav = MavlinkOut(mav_cfg, state)
    mav.start()

    # --- optional bathy estimator -------------------------------------------
    bathy = None
    if args.bathy:
        from .bathy_match import RealisticChart, GridChart
        from .bathy_estimator import BathyEstimator, BathyEstimatorConfig
        if args.bathy_chart:
            import numpy as np
            log.info("loading bathy chart from %s", args.bathy_chart)
            z = np.load(args.bathy_chart)
            chart = GridChart(z["data"].tolist(),
                              float(z["lat_min"]), float(z["lon_min"]),
                              float(z["dlat"]), float(z["dlon"]))
        else:
            log.info("no chart path given — using procedural realistic chart (demo)")
            chart = GridChart.from_realistic(
                RealisticChart(),
                -33.980, 151.190, -33.960, 151.210, resolution_m=5.0,
            )
        bathy = BathyEstimator(chart, state, BathyEstimatorConfig())
        bathy.start()
        log.info("bathy estimator enabled (tertiary position source)")

    # --- input source -------------------------------------------------------
    mock = None
    yd = None
    sk_task = None

    if args.mock:
        log.info("starting mock source (10 Hz GPS, 2 Hz depth, 1 Hz wind, 4 AIS contacts)")
        mock = MockSource(state)
        mock.start()
    elif args.yd_udp:
        host, port = args.yd_udp.split(":")
        yd = YachtDevicesRawUdp(RawUdpConfig(host, int(port)), state)
        yd.start()
    elif args.signalk:
        from .protocol.signalk import SignalKClient, SignalKConfig
        sk = SignalKClient(SignalKConfig(url=args.signalk), state)
        # Run Signal K in its own asyncio loop
        import threading
        def _run_sk():
            asyncio.run(sk.run())
        t = threading.Thread(target=_run_sk, daemon=True, name="signalk")
        t.start()
    else:
        log.error("no source selected; use --mock / --yd-udp / --signalk")
        return 2

    # --- status print loop --------------------------------------------------
    stop = {"flag": False}
    def shutdown(sig, frm):
        log.info("shutdown requested")
        stop["flag"] = True
    signal.signal(signal.SIGINT, shutdown)
    try:
        signal.signal(signal.SIGTERM, shutdown)
    except AttributeError:
        pass

    t0 = time.time()
    last_print = 0.0
    while not stop["flag"]:
        now = time.time()
        if now - last_print >= 5.0:
            stats = state.stats()
            gps = state.snapshot_gps()
            hdg = state.snapshot_heading()
            dep = state.snapshot_depth()
            bathy_str = ""
            if bathy:
                est = bathy.latest_estimate
                if est:
                    import math as _m
                    delta_m = _m.hypot(
                        (est.lat - gps.lat) * 111320.0,
                        (est.lon - gps.lon) * 111320.0 * _m.cos(_m.radians(est.lat)),
                    ) if gps.updated_at else 0.0
                    bathy_str = f"  bathy: Δgps={delta_m:.1f}m spread={est.spread_m:.0f}m {'OK' if est.healthy else 'NG'}"
            log.info(
                "packets=%d/%d  fix=%d sats=%d  pos=%.6f,%.6f  sog=%.2f  hdg=%.0f°  depth=%.1fm  ais=%d%s",
                stats["packets_decoded"], stats["packets_seen"],
                gps.fix_type, gps.satellites,
                gps.lat, gps.lon, gps.sog_mps,
                __import__("math").degrees(hdg.true_rad) % 360,
                dep.meters_below_transducer,
                stats["ais_contacts"],
                bathy_str,
            )
            last_print = now
        time.sleep(0.5)

    # Teardown
    if mock: mock.stop()
    if yd: yd.stop()
    if bathy: bathy.stop()
    mav.close()
    log.info("stopped cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
