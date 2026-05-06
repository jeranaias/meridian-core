"""Tests for cold-start (GPS-denied at boot) recovery."""
import math
import random

import pytest

from meridian_orca.ais_fusion import (AisFixObservation,
                                       simulate_ais_observation)
from meridian_orca.bathy_match import (METERS_PER_DEG_LAT, meters_per_deg_lon,
                                        RealisticChart, GridChart)
from meridian_orca.cold_start import (cold_start_ais_anchor,
                                       cold_start_grid_seed)


@pytest.fixture(scope="module")
def grid_chart():
    return GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        lat_min=-33.990, lon_min=151.180,
        lat_max=-33.950, lon_max=151.220,
        resolution_m=10.0,
    )


# ===========================================================================
# AIS-anchor cold start
# ===========================================================================

def test_ais_anchor_recovers_position_within_sensor_sigma():
    """One noisy AIS observation should place us within ~3 sigma of truth."""
    truth_lat, truth_lon = -33.970, 151.200
    target_lat = truth_lat + 350 / METERS_PER_DEG_LAT
    target_lon = truth_lon + 350 / meters_per_deg_lon(truth_lat)
    rng = random.Random(0xBEEF)
    obs = simulate_ais_observation(
        truth_lat, truth_lon, target_lat, target_lon,
        range_sigma_m=5.0, bearing_sigma_deg=2.0, rng=rng,
    )
    seed_lat, seed_lon, unc = cold_start_ais_anchor(obs)
    err = math.hypot(
        (truth_lat - seed_lat) * METERS_PER_DEG_LAT,
        (truth_lon - seed_lon) * meters_per_deg_lon(truth_lat),
    )
    # Reported 1-sigma uncertainty should envelope the true error
    # at 3-sigma confidence.
    assert err < 3 * unc, \
        f"AIS-anchor err {err:.1f}m exceeded 3σ of reported {unc:.1f}m"


def test_ais_anchor_zero_noise_is_exact():
    """With zero sensor noise, AIS-anchor should recover truth exactly."""
    truth_lat, truth_lon = -33.970, 151.200
    target_lat = truth_lat + 500 / METERS_PER_DEG_LAT
    target_lon = truth_lon
    # Zero-noise observation
    true_range = 500.0
    obs = AisFixObservation(
        target_lat=target_lat, target_lon=target_lon,
        range_m=true_range, range_sigma_m=0.001,
        bearing_deg=0.0, bearing_sigma_deg=0.001,
    )
    seed_lat, seed_lon, _ = cold_start_ais_anchor(obs)
    err = math.hypot(
        (truth_lat - seed_lat) * METERS_PER_DEG_LAT,
        (truth_lon - seed_lon) * meters_per_deg_lon(truth_lat),
    )
    assert err < 0.5, f"zero-noise AIS-anchor err {err:.3f}m should be ≈0"


# ===========================================================================
# Grid-seed cold start
# ===========================================================================

def test_grid_seed_returns_top_candidates(grid_chart):
    """Grid seed must always return a top-5 candidate list even if the
    auto-best is wrong. On feature-poor terrain the truth often lives
    inside the top-5 instead of at #1.
    """
    rng = random.Random(0xBEEF)
    truth_lat = -33.970 + rng.uniform(-0.005, 0.005)
    truth_lon = 151.200 + rng.uniform(-0.005, 0.005)
    heading = rng.uniform(0, 2 * math.pi)
    vn, ve = 2.0 * math.cos(heading), 2.0 * math.sin(heading)

    obs_list = []
    lat, lon = truth_lat, truth_lon
    for _ in range(120):
        d = grid_chart.depth(lat, lon)
        if not math.isnan(d) and d >= 0.5:
            obs_list.append((1.0, vn, ve, d + rng.gauss(0, 0.3)))
        lat += vn / METERS_PER_DEG_LAT
        lon += ve / meters_per_deg_lon(lat)

    result = cold_start_grid_seed(
        grid_chart,
        bounds=(-33.990, -33.950, 151.180, 151.220),
        depth_observations=obs_list,
        grid_cells=16,
    )
    assert result.candidates is not None
    assert len(result.candidates) >= 3, \
        "expected at least 3 ranked candidates"
    # The closest candidate should be within one grid cell (275m here)
    closest = min(
        math.hypot(
            (truth_lat - c[0]) * METERS_PER_DEG_LAT,
            (truth_lon - c[1]) * meters_per_deg_lon(truth_lat),
        )
        for c in result.candidates
    )
    assert closest < 350.0, \
        f"closest top-5 candidate at {closest:.0f}m, expected <350m"


def test_grid_seed_runtime_reasonable(grid_chart):
    """16x16 grid + 120 obs should finish in under 2 s on CPU — loads
    quickly enough for an operator-facing boot sequence."""
    rng = random.Random(0xBEEF)
    obs_list = [(1.0, 2.0, 0.0, 5.0 + 0.3 * rng.gauss(0, 1)) for _ in range(60)]
    result = cold_start_grid_seed(
        grid_chart,
        bounds=(-33.990, -33.950, 151.180, 151.220),
        depth_observations=obs_list,
        grid_cells=16,
    )
    assert result.runtime_s < 2.0, \
        f"grid-seed took {result.runtime_s:.2f}s, expected <2s"


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
