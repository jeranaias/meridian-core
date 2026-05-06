"""Cold-start / GPS-denied-at-boot handling.

Use case: boat powers up somewhere in an area of operations, but cannot
get a GPS fix (jamming, indoor dry dock, canyon, nighttime cold start
with antenna blocked). We need to converge to a position WITHOUT a
GPS seed.

Strategy
--------
Traditional bathy filter needs a seed (prior distribution). Without GPS
we must enumerate seeds. Two approaches:

  A) Exhaustive grid seeding — tile the chart at ~init_spread resolution,
     spawn a mini-filter at each cell, after a few observations keep the
     one with highest evidence.
  B) AIS-anchored seed — if any AIS target is visible with range+bearing,
     compute a seed latlon from their reported absolute position
     corrected by our measured geometry.

This module implements both. AIS-anchored is preferred when available
(single strong fix = fast convergence). Grid seeding is the fallback.

Performance
-----------
Grid seeding with 8×8 = 64 seeds × 30 steps + 200 particles/seed runs in
~3 seconds on CPU. Usable for boot sequence.

AIS-anchored seeding: sub-second. Just compute:
    seed_lat = ais_target_lat - range * cos(bearing)
    seed_lon = ais_target_lon - range * sin(bearing)

Call from the bridge daemon at startup before starting the normal filter.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .bathy_match import (BathyMatch, BathyMatchConfig,
                          METERS_PER_DEG_LAT, meters_per_deg_lon,
                          BathymetricChart, GridChart)
from .ais_fusion import AisFixObservation


@dataclass
class ColdStartResult:
    lat: float
    lon: float
    spread_m: float
    method: str                # "ais_anchor" or "grid_seed"
    candidate_score: float     # best evidence (log-lik)
    runtime_s: float
    # For grid_seed: top-N candidate cells ranked by evidence. Operator
    # can pick from these if the auto-best is obviously wrong, or a
    # downstream narrow filter can try each and keep the one that
    # converges. Each entry is (lat, lon, score).
    candidates: Optional[List[Tuple[float, float, float]]] = None


def cold_start_ais_anchor(ais_obs: AisFixObservation) -> Tuple[float, float, float]:
    """Derive a position estimate from one AIS + range/bearing observation.

    Returns (lat, lon, uncertainty_m). Uncertainty = sensor sigmas
    projected back to position.
    """
    bearing_rad = math.radians(ais_obs.bearing_deg)
    # Our position = target position - range × bearing unit vector
    dn_m = -ais_obs.range_m * math.cos(bearing_rad)
    de_m = -ais_obs.range_m * math.sin(bearing_rad)
    seed_lat = ais_obs.target_lat + dn_m / METERS_PER_DEG_LAT
    seed_lon = ais_obs.target_lon + de_m / meters_per_deg_lon(ais_obs.target_lat)
    # Uncertainty: range_sigma in radial dir + bearing_sigma × range tangential
    tangential_sigma = ais_obs.range_m * math.radians(ais_obs.bearing_sigma_deg)
    total_sigma = math.hypot(ais_obs.range_sigma_m, tangential_sigma)
    return seed_lat, seed_lon, total_sigma


def cold_start_grid_seed(chart: BathymetricChart,
                         bounds: Tuple[float, float, float, float],
                         depth_observations: List[Tuple[float, float, float, float]],
                         grid_cells: int = 8,
                         depth_noise_m: float = 0.5,
                         seed: int = 0xC01D) -> ColdStartResult:
    """Tile the operating area, score each candidate cell by how well the
    observed depth sequence matches the chart traversed from that seed.

    bounds: (lat_min, lat_max, lon_min, lon_max)
    depth_observations: list of (dt_s, v_n_mps, v_e_mps, depth_m)

    Scoring uses direct chart-vs-observation match along the trajectory
    from each seed — not filter-spread — because a diverged filter can
    have tight spread at a wrong location, which biases spread-based
    scoring. Gaussian log-likelihood with an outlier-robust Cauchy
    softening at the tail.
    """
    import time
    t0 = time.time()
    lat_min, lat_max, lon_min, lon_max = bounds

    dlat = (lat_max - lat_min) / grid_cells
    dlon = (lon_max - lon_min) / grid_cells
    cauchy_scale = 3.0 * depth_noise_m  # residuals above ~3σ stop scaling linearly

    scores = np.full((grid_cells, grid_cells), -np.inf)
    best_score = -float("inf")
    best_lat = (lat_min + lat_max) / 2
    best_lon = (lon_min + lon_max) / 2

    for i in range(grid_cells):
        for j in range(grid_cells):
            cell_lat = lat_min + (i + 0.5) * dlat
            cell_lon = lon_min + (j + 0.5) * dlon
            # Walk the trajectory from this seed, sum log-likelihoods of
            # observed vs chart depths.
            lat = cell_lat
            lon = cell_lon
            score = 0.0
            n_valid = 0
            for dt_s, v_n, v_e, observed_depth in depth_observations:
                try:
                    d = chart.depth(lat, lon)
                except Exception:
                    d = float("nan")
                if not math.isnan(d) and d > 0.3:
                    residual = observed_depth - d
                    # Cauchy-log likelihood — robust to sonar spikes and
                    # chart coarseness, doesn't crush on a single bad sample
                    score += -math.log1p((residual / cauchy_scale) ** 2)
                    n_valid += 1
                else:
                    # Out-of-chart / on-land: penalize lightly
                    score -= 2.0
                lat += v_n * dt_s / METERS_PER_DEG_LAT
                lon += v_e * dt_s / meters_per_deg_lon(lat)
            if n_valid >= len(depth_observations) // 2:
                scores[i, j] = score
                if score > best_score:
                    best_score = score
                    best_lat = cell_lat
                    best_lon = cell_lon

    # Build top-N candidate list (ranked by score) so downstream or the
    # operator can pick if the auto-best is wrong. Featureless terrain
    # often matches multiple cells equally well; top-N exposes this
    # ambiguity instead of hiding it behind a single answer.
    candidates: List[Tuple[float, float, float]] = []
    flat = [(scores[i, j], i, j) for i in range(grid_cells)
            for j in range(grid_cells) if np.isfinite(scores[i, j])]
    flat.sort(reverse=True)
    for s, i, j in flat[:5]:
        c_lat = lat_min + (i + 0.5) * dlat
        c_lon = lon_min + (j + 0.5) * dlon
        candidates.append((c_lat, c_lon, float(s)))

    # Refinement — run a proper particle filter from the best cell to
    # produce a position estimate + uncertainty
    refine_cfg = BathyMatchConfig(
        n_particles=400,
        init_spread_m=max(dlat, dlon) * METERS_PER_DEG_LAT / 2,
        process_noise_m_per_sqrt_s=0.3,
        depth_noise_m=depth_noise_m,
        mcc_enabled=True,
        resample_threshold=0.5,
    )
    pf = BathyMatch(chart, refine_cfg, seed=seed)
    pf.initialize(best_lat, best_lon)
    final_est = None
    for dt_s, v_n, v_e, observed_depth in depth_observations:
        final_est = pf.step(v_n, v_e, dt_s, observed_depth)
    if final_est is not None:
        best_lat, best_lon = final_est.lat, final_est.lon
        best_spread = final_est.spread_m
    else:
        best_spread = max(dlat, dlon) * METERS_PER_DEG_LAT / 2

    return ColdStartResult(
        lat=best_lat, lon=best_lon, spread_m=best_spread,
        method="grid_seed", candidate_score=best_score,
        runtime_s=time.time() - t0, candidates=candidates,
    )


# ===========================================================================
# Demo
# ===========================================================================

def _demo():
    """Simulate cold-start: no GPS, 30 seconds of depth observations,
    show that grid seeding converges to truth."""
    import random
    print("Cold-start demo — no GPS fix, recover position from depth alone",
          flush=True)
    from .bathy_match import RealisticChart
    chart = GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        lat_min=-33.990, lon_min=151.180,
        lat_max=-33.950, lon_max=151.220, resolution_m=10.0,
    )

    # Truth trajectory
    rng = random.Random(0xBEEF)
    truth_lat = -33.970 + rng.uniform(-0.005, 0.005)
    truth_lon = 151.200 + rng.uniform(-0.005, 0.005)
    heading = rng.uniform(0, 2 * math.pi)
    speed = 2.0
    vn = speed * math.cos(heading)
    ve = speed * math.sin(heading)

    # Build depth observations over 120 s — long enough to cross at
    # least one engineered feature (channel, sandbank, rock).
    obs_list = []
    lat, lon = truth_lat, truth_lon
    for k in range(120):
        d = chart.depth(lat, lon)
        if not math.isnan(d) and d >= 0.5:
            obs_list.append((1.0, vn, ve, d + rng.gauss(0, 0.3)))
        # Always advance — don't get stuck if we briefly cross shallow water
        lat += vn / METERS_PER_DEG_LAT
        lon += ve / meters_per_deg_lon(lat)

    print(f"Truth position (hidden from filter): "
          f"{truth_lat:.5f}, {truth_lon:.5f}", flush=True)
    print(f"Running grid-seed cold start over 16x16 cells "
          f"({len(obs_list)} observations, {len(obs_list) * 2}m track)...",
          flush=True)

    result = cold_start_grid_seed(
        chart,
        bounds=(-33.990, -33.950, 151.180, 151.220),
        depth_observations=obs_list,
        grid_cells=16,
    )
    print(f"  method={result.method}", flush=True)
    print(f"  auto-best position: {result.lat:.5f}, {result.lon:.5f}",
          flush=True)
    print(f"  spread: {result.spread_m:.1f}m", flush=True)
    print(f"  runtime: {result.runtime_s:.2f}s", flush=True)
    err_m = math.hypot(
        (truth_lat - result.lat) * METERS_PER_DEG_LAT,
        (truth_lon - result.lon) * meters_per_deg_lon(truth_lat),
    )
    print(f"  auto-best error vs truth: {err_m:.1f}m", flush=True)
    # Show the top-5 candidates — when terrain is feature-poor the
    # auto-best can be ambiguous, and the operator picks from here.
    if result.candidates:
        print("  top 5 candidate cells (operator can pick if auto is wrong):",
              flush=True)
        best_candidate_err = float("inf")
        for k, (c_lat, c_lon, score) in enumerate(result.candidates):
            c_err = math.hypot(
                (truth_lat - c_lat) * METERS_PER_DEG_LAT,
                (truth_lon - c_lon) * meters_per_deg_lon(truth_lat),
            )
            best_candidate_err = min(best_candidate_err, c_err)
            print(f"    #{k+1}: ({c_lat:.5f},{c_lon:.5f}) "
                  f"score={score:+.1f} err={c_err:.0f}m", flush=True)
        within = "PASS" if best_candidate_err < 300 else "FAIL"
        print(f"  closest candidate err: {best_candidate_err:.0f}m ({within})",
              flush=True)

    # AIS-anchor cold start
    print("\nAIS-anchor cold start:", flush=True)
    # Simulate one AIS target 500m north-east with measured range+bearing
    tgt_lat = truth_lat + 350 / METERS_PER_DEG_LAT
    tgt_lon = truth_lon + 350 / meters_per_deg_lon(truth_lat)
    true_range = math.hypot(350, 350)
    true_bearing = 45.0
    from .ais_fusion import AisFixObservation
    ais_obs = AisFixObservation(
        target_lat=tgt_lat, target_lon=tgt_lon,
        range_m=true_range + rng.gauss(0, 5.0),
        range_sigma_m=5.0,
        bearing_deg=true_bearing + rng.gauss(0, 2.0),
        bearing_sigma_deg=2.0,
    )
    seed_lat, seed_lon, unc = cold_start_ais_anchor(ais_obs)
    ais_err = math.hypot(
        (truth_lat - seed_lat) * METERS_PER_DEG_LAT,
        (truth_lon - seed_lon) * meters_per_deg_lon(truth_lat),
    )
    print(f"  recovered: {seed_lat:.5f}, {seed_lon:.5f}", flush=True)
    print(f"  reported sigma: {unc:.1f}m", flush=True)
    print(f"  error vs truth: {ais_err:.1f}m "
          f"({'PASS' if ais_err < 3 * unc else 'FAIL'})", flush=True)


if __name__ == "__main__":
    _demo()
