"""Chart gradient computation + gradient-aware weighting.

Given a bathymetric chart, compute the per-cell gradient magnitude. High-
gradient areas (steep terrain, channel edges, rocks) carry more
positioning information than flat featureless bottoms. Using gradient
to modulate particle weight:

  - In flat areas: down-weight depth-match contribution (avoid false
    precision from coincidental matches)
  - In feature-rich areas: up-weight depth-match (trust the signal)

This gives the filter automatic preference for informative terrain. In
practice it reduces false-positive-lock in sand basins and improves
convergence where structure exists.

Math
----
For each chart cell, |grad depth| = sqrt((d/dx)² + (d/dy)²).
We normalize by global mean gradient to get a dimensionless info-weight
in range [0, ~5]. Particle contribution to log-likelihood gets scaled by
this factor at the particle's current position.
"""
import math
from typing import Optional

import numpy as np

from .bathy_match import (
    BathymetricChart, GridChart,
    METERS_PER_DEG_LAT, meters_per_deg_lon,
)


class GradientField:
    """Per-cell gradient magnitude for a GridChart, with lookup."""

    def __init__(self, chart: GridChart, smooth_sigma: float = 1.0) -> None:
        self.chart = chart
        arr = np.array(chart.data, dtype=np.float64)
        # Mask land (depth < 0.5) to avoid edge spikes dominating
        mask = arr > 0.5
        # Compute gradient using centered differences (numpy handles boundaries)
        # dlat_m and dlon_m are the cell sizes in meters (approx; use mean lat)
        m_per_lat = METERS_PER_DEG_LAT
        m_per_lon = meters_per_deg_lon(chart.lat_min + chart.dlat * chart.n_rows / 2)
        dy = abs(chart.dlat) * m_per_lat
        dx = abs(chart.dlon) * m_per_lon
        grad_y = np.gradient(arr, axis=0) / dy
        grad_x = np.gradient(arr, axis=1) / dx
        mag = np.sqrt(grad_y ** 2 + grad_x ** 2)
        # Blank out land
        mag[~mask] = 0.0
        # Normalize by mean gradient (over water cells)
        water_mag = mag[mask]
        if water_mag.size > 0:
            mean_g = float(water_mag.mean()) + 1e-6
        else:
            mean_g = 1.0
        self.info_weight = (mag / mean_g).astype(np.float32)
        # Clip to conservative range — wider ranges hurt on synthetic tests,
        # tune in [0.8, 1.3] to start, widen with real-data validation.
        np.clip(self.info_weight, 0.8, 1.3, out=self.info_weight)

    def at(self, lat: float, lon: float) -> float:
        """Bilinear interpolation of info weight at (lat, lon)."""
        c = self.chart
        i = (lat - c.lat_min) / c.dlat
        j = (lon - c.lon_min) / c.dlon
        if i < 0 or i >= c.n_rows - 1 or j < 0 or j >= c.n_cols - 1:
            return 1.0  # Out of chart: neutral weight
        i0, j0 = int(i), int(j)
        di, dj = i - i0, j - j0
        w = self.info_weight
        a00 = w[i0][j0]
        a01 = w[i0][j0 + 1]
        a10 = w[i0 + 1][j0]
        a11 = w[i0 + 1][j0 + 1]
        return float(a00 * (1 - di) * (1 - dj) + a01 * (1 - di) * dj
                     + a10 * di * (1 - dj) + a11 * di * dj)

    def at_many(self, lats: np.ndarray, lons: np.ndarray) -> np.ndarray:
        """Vectorized batch version — used by particle filters."""
        c = self.chart
        i = (lats - c.lat_min) / c.dlat
        j = (lons - c.lon_min) / c.dlon
        valid = (i >= 0) & (i < c.n_rows - 1) & (j >= 0) & (j < c.n_cols - 1)
        out = np.ones_like(lats, dtype=np.float64)
        if not np.any(valid):
            return out
        i0 = np.clip(i.astype(int), 0, c.n_rows - 2)
        j0 = np.clip(j.astype(int), 0, c.n_cols - 2)
        di = i - i0; dj = j - j0
        w = self.info_weight
        a00 = w[i0, j0]
        a01 = w[i0, j0 + 1]
        a10 = w[i0 + 1, j0]
        a11 = w[i0 + 1, j0 + 1]
        out_valid = (a00 * (1 - di) * (1 - dj) + a01 * (1 - di) * dj
                     + a10 * di * (1 - dj) + a11 * di * dj)
        out[valid] = out_valid[valid]
        return out
