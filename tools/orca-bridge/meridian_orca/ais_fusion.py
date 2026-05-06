"""Bathy + AIS fusion — Stage 7.

When GPS is jammed, bathy map-matching localizes us to ~20m. Adding
observations of AIS targets (other vessels broadcasting their GPS
position) tightens that further — the target's lat/lon is absolute,
our radar/visual gives range/bearing, and the geometry constrains
where we can be.

For each AIS target with a corresponding range/bearing measurement:

    expected_range   = dist(particle, target_latlon)
    expected_bearing = atan2(target_e - particle_e, target_n - particle_n)

A particle's log-likelihood gains:

    log_lik += -0.5 * ((meas_range - expected_range) / σ_r)²
               -0.5 * ((meas_bearing - expected_bearing) / σ_b)²

with bearing difference wrapped to [-π, π].

Range measurement typically comes from radar (±5 m); bearing from radar
or AIS direction-finding (±2°). Even one AIS target cuts position
uncertainty in half. Two independent targets give a full 2-D fix
comparable to GPS.

Sensor integration posture
--------------------------
- AIS receiver: VHF, passive, gives target lat/lon/cog/sog — no infra
- Marine radar: 1-10 ms range, 1-3° bearing — standard aids-to-nav gear
- Visual bearing: optical or EO/IR tracker (Orca's native compass feed)

The fusion is SENSOR-AGNOSTIC — any source that gives
(target_known_latlon, measured_range, measured_bearing) with
uncertainties works.
"""
import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .bathy_match import METERS_PER_DEG_LAT, meters_per_deg_lon


@dataclass
class AisFixObservation:
    """One AIS target + range/bearing measurement from our vessel.

    target_lat/lon: absolute position reported by the target's AIS (degrees)
    range_m:       measured line-of-sight range (meters)
    range_sigma_m: 1-sigma uncertainty on range (meters)
    bearing_deg:   measured bearing FROM our vessel TO target, true north
                   (degrees clockwise; 0=N, 90=E)
    bearing_sigma_deg: 1-sigma uncertainty on bearing (degrees)
    max_gate_m:    residual beyond this is treated as outlier (nulls the
                   observation for that particle instead of driving it to
                   log-inf)
    robust_scale_m: Cauchy likelihood half-width; larger = softer / more
                   outlier-tolerant. Prevents any single observation from
                   crushing the posterior to one particle. Tuned so the
                   observation can't outvote ~10 bathy depth matches.
    """
    target_lat: float
    target_lon: float
    range_m: float
    range_sigma_m: float = 5.0
    bearing_deg: float = 0.0
    bearing_sigma_deg: float = 2.0
    max_gate_m: float = 80.0
    has_bearing: bool = True
    robust_scale_m: float = 10.0


def ais_log_likelihood(particle_lat: float, particle_lon: float,
                       obs: AisFixObservation,
                       current_spread_m: float = 0.0) -> float:
    """Scalar log-likelihood of `obs` given the particle at (lat, lon).

    current_spread_m: filter's current 1-sigma position uncertainty. Used
        to adapt the Cauchy half-width so that AIS contribution is soft
        while the filter is diffuse (prevents single-particle collapse)
        and sharpens as the filter tightens.
    """
    dn_m = (obs.target_lat - particle_lat) * METERS_PER_DEG_LAT
    de_m = (obs.target_lon - particle_lon) * meters_per_deg_lon(particle_lat)
    expected_range = math.hypot(dn_m, de_m)
    expected_bearing_rad = math.atan2(de_m, dn_m)

    # Adaptive Cauchy scale: never tighter than 2 × particle_spread.
    # When the filter is uncertain (spread > scale), AIS is soft, preserving
    # diversity. When the filter has converged (spread < scale/2), AIS
    # contributes at its nominal sharpness.
    scale_m = max(obs.robust_scale_m, 2.0 * current_spread_m)

    r_res = obs.range_m - expected_range
    if abs(r_res) > obs.max_gate_m:
        return 0.0
    log_lik = -math.log1p((r_res / scale_m) ** 2)

    # Bearing residual (wrapped) — Cauchy likelihood, adaptive half-width.
    if obs.has_bearing:
        meas_bearing_rad = math.radians(obs.bearing_deg)
        b_res = meas_bearing_rad - expected_bearing_rad
        while b_res > math.pi:
            b_res -= 2 * math.pi
        while b_res < -math.pi:
            b_res += 2 * math.pi
        # Adaptive: sensor sigma OR angular spread at this range, whichever
        # larger. Prevents bearing from dominating when filter is diffuse.
        sigma_sensor_rad = math.radians(obs.bearing_sigma_deg)
        sigma_spread_rad = (current_spread_m / max(expected_range, 1.0)
                            if current_spread_m > 0 else 0.0)
        sigma_rad = max(sigma_sensor_rad, sigma_spread_rad)
        b_meter_res = expected_range * b_res
        if abs(b_meter_res) > obs.max_gate_m:
            return log_lik
        log_lik += -math.log1p((b_res / sigma_rad) ** 2)

    return log_lik


def ais_log_likelihood_batch(particle_lats: np.ndarray,
                             particle_lons: np.ndarray,
                             obs: AisFixObservation,
                             current_spread_m: float = 0.0) -> np.ndarray:
    """Vectorized log-likelihood for CNN filter's numpy particle array.

    current_spread_m: filter's current 1-sigma position uncertainty —
    used to adapt the Cauchy half-width. See `ais_log_likelihood` docs.
    """
    m_per_lon = meters_per_deg_lon(float(particle_lats.mean()))
    dn_m = (obs.target_lat - particle_lats) * METERS_PER_DEG_LAT
    de_m = (obs.target_lon - particle_lons) * m_per_lon
    expected_range = np.hypot(dn_m, de_m)
    expected_bearing_rad = np.arctan2(de_m, dn_m)

    scale_m = max(obs.robust_scale_m, 2.0 * current_spread_m)
    r_res = obs.range_m - expected_range
    log_lik = -np.log1p((r_res / scale_m) ** 2)
    gate = np.abs(r_res) > obs.max_gate_m
    log_lik[gate] = 0.0

    if obs.has_bearing:
        meas_bearing_rad = math.radians(obs.bearing_deg)
        b_res = meas_bearing_rad - expected_bearing_rad
        b_res = ((b_res + math.pi) % (2 * math.pi)) - math.pi
        sigma_sensor_rad = math.radians(obs.bearing_sigma_deg)
        if current_spread_m > 0:
            spread_angular = current_spread_m / np.maximum(expected_range, 1.0)
            sigma_rad = np.maximum(sigma_sensor_rad, spread_angular)
        else:
            sigma_rad = sigma_sensor_rad
        bearing_meter_res = expected_range * np.abs(b_res)
        bearing_gate = bearing_meter_res > obs.max_gate_m
        bearing_contrib = np.where(
            bearing_gate, 0.0,
            -np.log1p((b_res / sigma_rad) ** 2)
        )
        bearing_contrib[gate] = 0.0
        log_lik += bearing_contrib

    return log_lik


# ===========================================================================
# Simulated sensor — for demo/testing without real radar
# ===========================================================================

def simulate_ais_observation(truth_lat: float, truth_lon: float,
                             target_lat: float, target_lon: float,
                             range_sigma_m: float = 5.0,
                             bearing_sigma_deg: float = 2.0,
                             rng: Optional[object] = None) -> AisFixObservation:
    """Synthesize a noisy AIS+radar observation from truth position."""
    import random as _r
    if rng is None:
        rng = _r.Random()
    dn_m = (target_lat - truth_lat) * METERS_PER_DEG_LAT
    de_m = (target_lon - truth_lon) * meters_per_deg_lon(truth_lat)
    true_range = math.hypot(dn_m, de_m)
    true_bearing_deg = math.degrees(math.atan2(de_m, dn_m))
    return AisFixObservation(
        target_lat=target_lat, target_lon=target_lon,
        range_m=true_range + rng.gauss(0, range_sigma_m),
        range_sigma_m=range_sigma_m,
        bearing_deg=true_bearing_deg + rng.gauss(0, bearing_sigma_deg),
        bearing_sigma_deg=bearing_sigma_deg,
    )


# ===========================================================================
# Demo
# ===========================================================================

def _demo():
    """Show the fusion win: bathy-only vs bathy+1AIS vs bathy+2AIS."""
    import random as _r
    import statistics
    from .bathy_match import (BathyMatch, BathyMatchConfig,
                              RealisticChart, GridChart)
    from .bathy_cnn_filter import BathyCNNFilter, CNNFilterConfig

    print("Bathy + AIS fusion demo — Stage 7", flush=True)
    print("Target layout: two stationary AIS targets 600m E and 400m NW "
          "of truth track", flush=True)

    print("\nBaking chart...", flush=True)
    chart = GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        lat_min=-33.985, lon_min=151.185,
        lat_max=-33.955, lon_max=151.215,
        resolution_m=10.0,
    )
    MODEL = r"D:\projects\meridian\tools\orca-bridge\tests\fixtures\bathy_matcher.pt"

    def trial(kind: str, seed: int, n_targets: int,
              duration: int = 90) -> Optional[float]:
        rng = _r.Random(seed)
        lat = -33.970 + rng.uniform(-0.002, 0.002)
        lon = 151.198 + rng.uniform(-0.002, 0.002)
        heading = rng.uniform(0, 2 * math.pi)
        speed = rng.uniform(1.5, 2.5)
        vn = speed * math.cos(heading)
        ve = speed * math.sin(heading)
        ang = rng.uniform(0, 2 * math.pi)
        seed_lat = lat + 30 * math.cos(ang) / METERS_PER_DEG_LAT
        seed_lon = lon + 30 * math.sin(ang) / meters_per_deg_lon(lat)

        # Two simulated AIS targets at FIXED positions (anchored vessels)
        tgt1_lat = lat + 600.0 / METERS_PER_DEG_LAT   # 600m N at trial start
        tgt1_lon = lon + 0.0
        tgt2_lat = lat + 0.0
        tgt2_lon = lon + 400.0 / meters_per_deg_lon(lat)  # 400m E

        if kind == "bootstrap":
            cfg = BathyMatchConfig(
                n_particles=1000, init_spread_m=40.0,
                process_noise_m_per_sqrt_s=0.3, depth_noise_m=0.5,
                mcc_enabled=True, regularization_m=0.5, resample_threshold=0.6)
            pf = BathyMatch(chart, cfg, seed=seed)
        else:
            cfg = CNNFilterConfig()
            pf = BathyCNNFilter(chart, MODEL, cfg, seed=seed)
        pf.initialize(seed_lat, seed_lon)

        errs = []
        for t in range(duration):
            lat += vn / METERS_PER_DEG_LAT
            lon += ve / meters_per_deg_lon(lat)
            d = chart.depth(lat, lon)
            if math.isnan(d) or d < 0.5:
                break
            obs_depth = d + rng.gauss(0, 0.3)

            # Build AIS observations for this step
            ais_obs: List[AisFixObservation] = []
            if n_targets >= 1:
                ais_obs.append(simulate_ais_observation(
                    lat, lon, tgt1_lat, tgt1_lon, rng=rng))
            if n_targets >= 2:
                ais_obs.append(simulate_ais_observation(
                    lat, lon, tgt2_lat, tgt2_lon, rng=rng))

            est = pf.step(vn, ve, 1.0, obs_depth, ais_observations=ais_obs)
            err = math.hypot(
                (lat - est.lat) * METERS_PER_DEG_LAT,
                (lon - est.lon) * meters_per_deg_lon(lat),
            )
            errs.append(err)

        if len(errs) < 30:
            return None
        return statistics.median(errs[len(errs) // 2:])

    print(f"\n{'kind':<12} {'no AIS':>9} {'+1 AIS':>9} {'+2 AIS':>9}",
          flush=True)
    for kind in ["bootstrap", "cnn"]:
        row = []
        for n_targets in [0, 1, 2]:
            meds = []
            for i in range(10):
                v = trial(kind, 0x7000 + i, n_targets)
                if v is not None:
                    meds.append(v)
            if meds:
                row.append(statistics.median(meds))
            else:
                row.append(float("nan"))
        print(f"{kind:<12} {row[0]:7.1f}m  {row[1]:7.1f}m  {row[2]:7.1f}m",
              flush=True)


if __name__ == "__main__":
    _demo()
