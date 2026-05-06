"""Integration tests for Stage 7 — bathy + AIS fusion."""
import math
import random

import numpy as np
import pytest

from meridian_orca.ais_fusion import (AisFixObservation, ais_log_likelihood,
                                       ais_log_likelihood_batch,
                                       simulate_ais_observation)
from meridian_orca.bathy_match import (METERS_PER_DEG_LAT, meters_per_deg_lon,
                                        BathyMatch, BathyMatchConfig,
                                        RealisticChart, GridChart)
from meridian_orca.bathy_cnn_filter import BathyCNNFilter, CNNFilterConfig

MODEL_PATH = r"D:\projects\meridian\tools\orca-bridge\tests\fixtures\bathy_matcher.pt"


# ===========================================================================
# Geometry sanity — scalar + batch implementations must agree
# ===========================================================================

def test_scalar_and_batch_agree_on_geometry():
    """ais_log_likelihood and ais_log_likelihood_batch must produce
    the same output for the same input (no bearing spread)."""
    truth_lat, truth_lon = -33.970, 151.200
    target_lat = truth_lat + 400 / METERS_PER_DEG_LAT
    target_lon = truth_lon + 300 / meters_per_deg_lon(truth_lat)
    obs = AisFixObservation(
        target_lat=target_lat, target_lon=target_lon,
        range_m=500.0, range_sigma_m=5.0,
        bearing_deg=math.degrees(math.atan2(300, 400)),
        bearing_sigma_deg=2.0,
    )
    lats = np.array([truth_lat - 10 / METERS_PER_DEG_LAT,
                     truth_lat,
                     truth_lat + 10 / METERS_PER_DEG_LAT])
    lons = np.array([truth_lon, truth_lon, truth_lon])
    scalar = np.array([ais_log_likelihood(lats[i], lons[i], obs,
                                           current_spread_m=0.0)
                       for i in range(3)])
    batch = ais_log_likelihood_batch(lats, lons, obs, current_spread_m=0.0)
    np.testing.assert_allclose(scalar, batch, atol=1e-6)


def test_particle_at_truth_has_max_log_lik():
    """The particle at truth position should have the highest log-lik."""
    truth_lat, truth_lon = -33.970, 151.200
    target_lat = truth_lat + 500 / METERS_PER_DEG_LAT
    target_lon = truth_lon
    obs = simulate_ais_observation(
        truth_lat, truth_lon, target_lat, target_lon,
        range_sigma_m=1.0, bearing_sigma_deg=0.5,
        rng=random.Random(0xBEEF),
    )
    lats = np.array([truth_lat + d / METERS_PER_DEG_LAT
                      for d in [-30, -10, 0, 10, 30]])
    lons = np.full_like(lats, truth_lon)
    log_liks = ais_log_likelihood_batch(lats, lons, obs, current_spread_m=20.0)
    # Truth particle (index 2) should be the max
    assert int(np.argmax(log_liks)) == 2, \
        f"expected truth particle (idx 2) to win, got {log_liks}"


def test_adaptive_scale_preserves_diversity():
    """With spread_m=50, the log-lik differences across a 40m spread
    should be smaller than with spread_m=0 (sharper)."""
    truth_lat, truth_lon = -33.970, 151.200
    obs = AisFixObservation(
        target_lat=truth_lat + 500 / METERS_PER_DEG_LAT,
        target_lon=truth_lon,
        range_m=500.0, range_sigma_m=5.0,
        bearing_deg=0.0, bearing_sigma_deg=2.0,
    )
    lats = np.linspace(truth_lat - 20 / METERS_PER_DEG_LAT,
                        truth_lat + 20 / METERS_PER_DEG_LAT, 5)
    lons = np.full_like(lats, truth_lon)
    sharp = ais_log_likelihood_batch(lats, lons, obs, current_spread_m=0.0)
    soft = ais_log_likelihood_batch(lats, lons, obs, current_spread_m=50.0)
    # Range of log-liks is smaller in soft mode
    assert (soft.max() - soft.min()) < (sharp.max() - sharp.min()), \
        f"adaptive scale should soften; sharp_range={sharp.max()-sharp.min():.3f} "\
        f"soft_range={soft.max()-soft.min():.3f}"


# ===========================================================================
# Gating — far-off AIS shouldn't crash log-lik to -inf
# ===========================================================================

def test_outlier_gate_returns_neutral():
    """A wildly wrong range measurement (out of max_gate) should
    contribute zero, not crush the particle."""
    truth_lat, truth_lon = -33.970, 151.200
    # Target is 500m away but obs says 5000m — clearly wrong
    obs = AisFixObservation(
        target_lat=truth_lat + 500 / METERS_PER_DEG_LAT,
        target_lon=truth_lon,
        range_m=5000.0, range_sigma_m=5.0,
        max_gate_m=80.0,
    )
    ll = ais_log_likelihood(truth_lat, truth_lon, obs)
    assert ll == 0.0, \
        f"outlier-gated residual should return 0.0, got {ll}"


# ===========================================================================
# Filter integration — bootstrap and CNN both accept ais_observations
# ===========================================================================

@pytest.fixture(scope="module")
def grid_chart():
    return GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        lat_min=-33.980, lon_min=151.190,
        lat_max=-33.960, lon_max=151.210,
        resolution_m=10.0,
    )


def test_bootstrap_accepts_ais_observations(grid_chart):
    """No crash / exception when passing ais_observations to BathyMatch.step."""
    cfg = BathyMatchConfig(n_particles=200, init_spread_m=40.0,
                            mcc_enabled=True)
    pf = BathyMatch(grid_chart, cfg, seed=0x1234)
    pf.initialize(-33.970, 151.200)
    obs = AisFixObservation(
        target_lat=-33.970 + 500 / METERS_PER_DEG_LAT,
        target_lon=151.200,
        range_m=500.0, range_sigma_m=5.0,
        bearing_deg=0.0,
    )
    est = pf.step(2.0, 0.0, 1.0, 5.0, ais_observations=[obs])
    assert not math.isnan(est.lat)
    assert not math.isnan(est.lon)
    assert est.spread_m > 0


def test_cnn_accepts_ais_observations(grid_chart):
    """No crash when passing AIS obs to the CNN filter."""
    try:
        import torch  # noqa: F401
    except ImportError:
        pytest.skip("torch not available")
    cfg = CNNFilterConfig(n_particles=200, init_spread_m=40.0)
    pf = BathyCNNFilter(grid_chart, MODEL_PATH, cfg, seed=0x1234)
    pf.initialize(-33.970, 151.200)
    obs = AisFixObservation(
        target_lat=-33.970 + 500 / METERS_PER_DEG_LAT,
        target_lon=151.200,
        range_m=500.0, range_sigma_m=5.0,
        bearing_deg=0.0,
    )
    est = pf.step(2.0, 0.0, 1.0, 5.0, ais_observations=[obs])
    assert not math.isnan(est.lat)
    assert not math.isnan(est.lon)
    assert est.spread_m > 0


def test_bootstrap_ais_fusion_does_not_collapse(grid_chart):
    """Regression test — the 18 Apr collapse bug (bootstrap+AIS blowing
    up to 100m+) stays fixed.

    Runs 30 seconds of bathy + 2 AIS targets; asserts final error < 40m.
    """
    rng = random.Random(0xBEEF)
    lat = -33.970
    lon = 151.200
    vn, ve = 2.0, 0.0
    seed_lat = lat + 30 / METERS_PER_DEG_LAT
    seed_lon = lon
    tgt1_lat = lat + 500 / METERS_PER_DEG_LAT
    tgt1_lon = lon
    tgt2_lat = lat
    tgt2_lon = lon + 400 / meters_per_deg_lon(lat)

    cfg = BathyMatchConfig(n_particles=1000, init_spread_m=40.0,
                            process_noise_m_per_sqrt_s=0.3,
                            depth_noise_m=0.5, mcc_enabled=True,
                            regularization_m=0.5, resample_threshold=0.6)
    pf = BathyMatch(grid_chart, cfg, seed=0x1234)
    pf.initialize(seed_lat, seed_lon)

    errs = []
    for t in range(90):
        lat += vn / METERS_PER_DEG_LAT
        lon += ve / meters_per_deg_lon(lat)
        d = grid_chart.depth(lat, lon)
        obs_depth = d + rng.gauss(0, 0.3)
        ais_obs = [
            simulate_ais_observation(lat, lon, tgt1_lat, tgt1_lon, rng=rng),
            simulate_ais_observation(lat, lon, tgt2_lat, tgt2_lon, rng=rng),
        ]
        est = pf.step(vn, ve, 1.0, obs_depth, ais_observations=ais_obs)
        errs.append(math.hypot(
            (lat - est.lat) * METERS_PER_DEG_LAT,
            (lon - est.lon) * meters_per_deg_lon(lat),
        ))
    # Regression bar: pre-fix behavior was 100m+ steady-state; post-fix
    # median over the second half must stay below 40m.
    import statistics
    median_err = statistics.median(errs[len(errs) // 2:])
    assert median_err < 40.0, \
        f"bootstrap+AIS fusion regressed: median err {median_err:.1f}m"


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
