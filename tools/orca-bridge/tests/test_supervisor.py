"""Tests for the parallel-filter supervisor."""
import math
import random

import pytest

from meridian_orca.bathy_match import (METERS_PER_DEG_LAT, meters_per_deg_lon,
                                        RealisticChart, GridChart)
from meridian_orca.bathy_supervisor import (BathySupervisor, SupervisorConfig,
                                             SupervisorEstimate)

MODEL_PATH = r"D:\projects\meridian\tools\orca-bridge\tests\fixtures\bathy_matcher.pt"


@pytest.fixture(scope="module")
def grid_chart():
    return GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        lat_min=-33.985, lon_min=151.185,
        lat_max=-33.955, lon_max=151.215,
        resolution_m=10.0,
    )


def test_supervisor_returns_valid_estimate(grid_chart):
    """Basic smoke: supervisor runs both filters and returns a usable
    position estimate."""
    try:
        import torch  # noqa: F401
    except ImportError:
        pytest.skip("torch required for CNN filter")
    sup = BathySupervisor(grid_chart, MODEL_PATH, seed=0x1234)
    sup.initialize(-33.970, 151.200)
    rng = random.Random(0xBEEF)
    lat, lon = -33.970, 151.200
    vn, ve = 2.0, 0.0
    est = None
    for _ in range(30):
        lat += vn / METERS_PER_DEG_LAT
        lon += ve / meters_per_deg_lon(lat)
        d = grid_chart.depth(lat, lon)
        est = sup.step(vn, ve, 1.0, d + rng.gauss(0, 0.3))
    assert est is not None
    assert not math.isnan(est.lat) and not math.isnan(est.lon)
    assert est.source in ("cnn", "bootstrap")
    assert est.spread_m > 0


def test_supervisor_reports_source(grid_chart):
    """The supervisor tells us which filter is currently publishing —
    essential for UI and for downstream consumers that want to know."""
    try:
        import torch  # noqa: F401
    except ImportError:
        pytest.skip("torch required for CNN filter")
    sup = BathySupervisor(grid_chart, MODEL_PATH, seed=0x1234)
    sup.initialize(-33.970, 151.200)
    est = sup.step(2.0, 0.0, 1.0, 5.0)
    # With the default SupervisorConfig preferred_primary="cnn",
    # the initial publish should be cnn if CNN is healthy.
    assert est.source in ("cnn", "bootstrap")


def test_supervisor_cross_check_spread_populated(grid_chart):
    """The supervisor should always report the secondary filter's spread
    for cross-check / UI display — a diverging secondary is the first
    warning that the active primary might also be about to go."""
    try:
        import torch  # noqa: F401
    except ImportError:
        pytest.skip("torch required for CNN filter")
    sup = BathySupervisor(grid_chart, MODEL_PATH, seed=0x1234)
    sup.initialize(-33.970, 151.200)
    rng = random.Random(0xBEEF)
    lat, lon = -33.970, 151.200
    vn, ve = 2.0, 0.0
    for _ in range(10):
        lat += vn / METERS_PER_DEG_LAT
        lon += ve / meters_per_deg_lon(lat)
        d = grid_chart.depth(lat, lon)
        est = sup.step(vn, ve, 1.0, d + rng.gauss(0, 0.3))
    assert est.secondary_spread_m >= 0
    assert not math.isnan(est.secondary_spread_m)


def test_supervisor_accepts_ais_observations(grid_chart):
    """The supervisor must forward AIS observations to both filters
    transparently."""
    try:
        import torch  # noqa: F401
    except ImportError:
        pytest.skip("torch required for CNN filter")
    from meridian_orca.ais_fusion import simulate_ais_observation
    sup = BathySupervisor(grid_chart, MODEL_PATH, seed=0x1234)
    sup.initialize(-33.970, 151.200)
    rng = random.Random(0xBEEF)
    tgt_lat = -33.970 + 500 / METERS_PER_DEG_LAT
    tgt_lon = 151.200
    ais = [simulate_ais_observation(
        -33.970, 151.200, tgt_lat, tgt_lon, rng=rng)]
    est = sup.step(2.0, 0.0, 1.0, 5.0, ais_observations=ais)
    assert est is not None
    assert not math.isnan(est.lat)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
