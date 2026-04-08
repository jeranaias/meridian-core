#!/usr/bin/env python3
"""
terrain.py — Bathymetry and terrain elevation service for Meridian.

Downloads ETOPO 2022 data from NOAA's THREDDS OPeNDAP server for any
bounding box, caches locally as compressed numpy arrays.  Provides
depth queries for the SITL autopilot and GCS waypoint validation.

    Depth convention:
        negative = below sea level (water)
        positive = above sea level (land)
        0        = sea level

Usage as library:
    from terrain import TerrainDB
    db = TerrainDB("data/terrain")
    db.ensure_region(-34.02, 151.15, -33.93, 151.25)  # Botany Bay
    depth = db.get_depth(-33.97, 151.20)   # → -8.3 (metres below sea level)
    ok    = db.is_water(-33.97, 151.20)    # → True

Usage standalone (prefetch a region):
    python tools/terrain.py --lat -33.97 --lon 151.20 --radius 5
"""

import os
import math
import hashlib
import logging
import numpy as np
from scipy.interpolate import RegularGridInterpolator

log = logging.getLogger("terrain")

# NOAA ETOPO 2022 — 60 arc-second bed elevation via OPeNDAP
# Sea-floor (bed) elevation: negative in ocean, positive on land.
# 60 arc-second ≈ 1.8 km resolution.  Free, no registration, single global file.
ETOPO_BASE = (
    "https://www.ngdc.noaa.gov/thredds/dodsC/global/"
    "ETOPO2022/60s/60s_bed_elev_netcdf/"
    "ETOPO_2022_v1_60s_N90W180_bed.nc"
)

# Grid: lat -90 to +90, lon -180 to +180 in 1/60° steps
ETOPO_STEP = 1.0 / 60.0  # 60 arc-seconds in degrees
ETOPO_LAT_MIN = -90.0
ETOPO_LON_MIN = -180.0
ETOPO_LAT_N = 10801
ETOPO_LON_N = 21601


def _lat_to_idx(lat):
    return (lat - ETOPO_LAT_MIN) / ETOPO_STEP

def _lon_to_idx(lon):
    return (lon - ETOPO_LON_MIN) / ETOPO_STEP


class TerrainTile:
    """One cached rectangular grid of elevation data."""

    def __init__(self, lat_min, lon_min, lat_max, lon_max, lats, lons, elev):
        self.lat_min = lat_min
        self.lon_min = lon_min
        self.lat_max = lat_max
        self.lon_max = lon_max
        self.lats = lats
        self.lons = lons
        self.elev = elev
        self._interp = RegularGridInterpolator(
            (lats, lons), elev,
            method="linear", bounds_error=False, fill_value=None
        )

    def contains(self, lat, lon):
        return (self.lat_min <= lat <= self.lat_max and
                self.lon_min <= lon <= self.lon_max)

    def query(self, lat, lon):
        """Return elevation in metres (negative = below sea level)."""
        return float(self._interp((lat, lon)))


class TerrainDB:
    """
    Terrain / bathymetry database.

    Tiles are fetched from NOAA and cached in `cache_dir` as compressed
    numpy files.  Multiple tiles can be loaded; queries check all tiles.
    """

    def __init__(self, cache_dir="data/terrain"):
        self.cache_dir = cache_dir
        self.tiles = []
        os.makedirs(cache_dir, exist_ok=True)
        self._load_cached()

    # ── Public API ───────────────────────────────────────────

    def get_depth(self, lat, lon):
        """
        Elevation at (lat, lon) in metres.
        Negative = water depth, positive = land height.
        Returns None if no data available.
        """
        for tile in self.tiles:
            if tile.contains(lat, lon):
                return tile.query(lat, lon)
        return None

    def is_water(self, lat, lon):
        """True if the position is below sea level."""
        d = self.get_depth(lat, lon)
        if d is None:
            return None
        return d < 0.0

    def is_navigable(self, lat, lon, min_depth=-1.0):
        """True if depth is sufficient for navigation."""
        d = self.get_depth(lat, lon)
        if d is None:
            return None
        return d <= min_depth

    def check_path(self, lat1, lon1, lat2, lon2, step_m=50.0, min_depth=-1.0):
        """
        Check a path for navigability.
        Returns (ok, first_bad_lat, first_bad_lon, depth_at_bad)
        or (True, None, None, None).
        """
        dist = _haversine(lat1, lon1, lat2, lon2)
        n_steps = max(2, int(dist / step_m) + 1)

        for i in range(n_steps + 1):
            t = i / n_steps
            lat = lat1 + t * (lat2 - lat1)
            lon = lon1 + t * (lon2 - lon1)
            d = self.get_depth(lat, lon)
            if d is not None and d > min_depth:
                return False, lat, lon, d
        return True, None, None, None

    def depth_along_path(self, lat1, lon1, lat2, lon2, step_m=20.0):
        """Return list of (distance_m, depth_m) along a path."""
        dist = _haversine(lat1, lon1, lat2, lon2)
        n_steps = max(2, int(dist / step_m) + 1)
        profile = []
        for i in range(n_steps + 1):
            t = i / n_steps
            lat = lat1 + t * (lat2 - lat1)
            lon = lon1 + t * (lon2 - lon1)
            d = self.get_depth(lat, lon)
            profile.append((t * dist, d if d is not None else 0.0))
        return profile

    def ensure_region(self, lat_min, lon_min, lat_max, lon_max, margin_km=2.0):
        """
        Ensure terrain data covers the bounding box.
        Downloads from NOAA if not cached.
        """
        margin_deg = margin_km / 111.0
        lat_min -= margin_deg
        lat_max += margin_deg
        lon_min -= margin_deg
        lon_max += margin_deg

        for tile in self.tiles:
            if (tile.lat_min <= lat_min and tile.lat_max >= lat_max and
                    tile.lon_min <= lon_min and tile.lon_max >= lon_max):
                return

        log.info(f"Fetching terrain: ({lat_min:.4f},{lon_min:.4f}) → "
                 f"({lat_max:.4f},{lon_max:.4f})")
        tile = self._fetch_etopo(lat_min, lon_min, lat_max, lon_max)
        if tile:
            self.tiles.append(tile)
            self._save_tile(tile)

    # ── ETOPO download ───────────────────────────────────────

    def _fetch_etopo(self, lat_min, lon_min, lat_max, lon_max):
        """Download a region from NOAA ETOPO 2022 15s via OPeNDAP ASCII."""
        try:
            import requests
        except ImportError:
            log.error("requests library required: pip install requests")
            return None

        i_lat_min = max(0, int(math.floor(_lat_to_idx(lat_min))))
        i_lat_max = min(ETOPO_LAT_N - 1, int(math.ceil(_lat_to_idx(lat_max))))
        i_lon_min = max(0, int(math.floor(_lon_to_idx(lon_min))))
        i_lon_max = min(ETOPO_LON_N - 1, int(math.ceil(_lon_to_idx(lon_max))))

        n_lat = i_lat_max - i_lat_min + 1
        n_lon = i_lon_max - i_lon_min + 1
        log.info(f"ETOPO 60s grid: {n_lat}×{n_lon} cells")

        # OPeNDAP ASCII subset
        url = (f"{ETOPO_BASE}.ascii?"
               f"lat[{i_lat_min}:1:{i_lat_max}],"
               f"lon[{i_lon_min}:1:{i_lon_max}],"
               f"z[{i_lat_min}:1:{i_lat_max}][{i_lon_min}:1:{i_lon_max}]")

        log.info("Downloading from NOAA ETOPO 2022 (60 arc-second)...")
        try:
            resp = requests.get(url, timeout=120)
            resp.raise_for_status()
        except Exception as e:
            log.error(f"ETOPO download failed: {e}")
            return None

        return self._parse_opendap(resp.text, n_lat, n_lon)

    def _parse_opendap(self, text, n_lat, n_lon):
        """
        Parse OPeNDAP ASCII response.

        Format:
            Dataset { ... } name;
            ---
            lat[N]
            val, val, val, ...

            lon[M]
            val, val, val, ...

            z.z[N][M]
            [0], v, v, v, ...
            [1], v, v, v, ...
            ...

            z.lat[N]
            ...
            z.lon[M]
            ...
        """
        lines = text.strip().split("\n")
        lats = None
        lons = None
        elev_rows = []
        i = 0

        while i < len(lines):
            line = lines[i].strip()

            # First lat array (standalone, before z.lat)
            if line.startswith("lat[") and not line.startswith("lat[lat"):
                i += 1
                lats = self._parse_values(lines, i)
                i += 1
                continue

            # First lon array
            if line.startswith("lon[") and not line.startswith("lon[lon"):
                i += 1
                lons = self._parse_values(lines, i)
                i += 1
                continue

            # Elevation grid: z.z[N][M]
            if line.startswith("z.z[") or line.startswith("z["):
                i += 1
                while i < len(lines):
                    row_line = lines[i].strip()
                    if not row_line or row_line.startswith("z.") or row_line.startswith("Dataset"):
                        break
                    # Format: [row_idx], val1, val2, ...
                    if row_line.startswith("["):
                        bracket_end = row_line.index("]")
                        vals_str = row_line[bracket_end + 1:].strip().lstrip(",")
                    else:
                        vals_str = row_line
                    row = [float(v.strip()) for v in vals_str.split(",") if v.strip()]
                    if row:
                        elev_rows.append(row)
                    i += 1
                continue

            i += 1

        if lats is None or lons is None or not elev_rows:
            log.error(f"Parse failed: lats={'OK' if lats is not None else 'MISSING'}, "
                      f"lons={'OK' if lons is not None else 'MISSING'}, "
                      f"rows={len(elev_rows)}")
            return None

        lats = np.array(lats)
        lons = np.array(lons)
        elev = np.array(elev_rows, dtype=np.float32)

        if elev.shape != (len(lats), len(lons)):
            log.error(f"Shape mismatch: elev {elev.shape} vs ({len(lats)},{len(lons)})")
            return None

        log.info(f"Terrain loaded: {len(lats)}×{len(lons)} grid, "
                 f"depth range {elev.min():.0f}m to {elev.max():.0f}m")

        return TerrainTile(
            float(lats.min()), float(lons.min()),
            float(lats.max()), float(lons.max()),
            lats, lons, elev
        )

    def _parse_values(self, lines, i):
        """Parse a line of comma-separated floats."""
        if i >= len(lines):
            return None
        vals = []
        for v in lines[i].strip().split(","):
            v = v.strip()
            if v:
                try:
                    vals.append(float(v))
                except ValueError:
                    pass
        return vals if vals else None

    # ── Tile cache ───────────────────────────────────────────

    def _tile_filename(self, tile):
        key = f"{tile.lat_min:.4f}_{tile.lon_min:.4f}_{tile.lat_max:.4f}_{tile.lon_max:.4f}"
        h = hashlib.md5(key.encode()).hexdigest()[:12]
        return f"etopo60s_{h}.npz"

    def _save_tile(self, tile):
        path = os.path.join(self.cache_dir, self._tile_filename(tile))
        np.savez_compressed(path,
                            lats=tile.lats, lons=tile.lons, elev=tile.elev,
                            bounds=np.array([tile.lat_min, tile.lon_min,
                                             tile.lat_max, tile.lon_max]))
        log.info(f"Cached: {path} ({os.path.getsize(path) / 1024:.0f} KB)")

    def _load_cached(self):
        """Load all cached tiles from disk."""
        if not os.path.exists(self.cache_dir):
            return
        for fname in os.listdir(self.cache_dir):
            if fname.endswith(".npz"):
                path = os.path.join(self.cache_dir, fname)
                try:
                    data = np.load(path)
                    bounds = data["bounds"]
                    tile = TerrainTile(
                        float(bounds[0]), float(bounds[1]),
                        float(bounds[2]), float(bounds[3]),
                        data["lats"], data["lons"], data["elev"]
                    )
                    self.tiles.append(tile)
                    log.info(f"Loaded: {fname} "
                             f"({tile.lat_min:.3f},{tile.lon_min:.3f}) → "
                             f"({tile.lat_max:.3f},{tile.lon_max:.3f}), "
                             f"{len(tile.lats)}×{len(tile.lons)} grid")
                except Exception as e:
                    log.warning(f"Failed to load {fname}: {e}")


def _haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ── CLI ──────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    p = argparse.ArgumentParser(description="Fetch terrain/bathymetry data")
    p.add_argument("--lat", type=float, required=True, help="Center latitude")
    p.add_argument("--lon", type=float, required=True, help="Center longitude")
    p.add_argument("--radius", type=float, default=5.0, help="Radius in km")
    p.add_argument("--cache", default="data/terrain", help="Cache directory")
    args = p.parse_args()

    margin = args.radius / 111.0
    db = TerrainDB(args.cache)
    db.ensure_region(
        args.lat - margin, args.lon - margin,
        args.lat + margin, args.lon + margin,
        margin_km=1.0
    )

    # Test queries
    d = db.get_depth(args.lat, args.lon)
    if d is not None:
        status = "WATER" if d < 0 else "LAND"
        print(f"\nCenter ({args.lat}, {args.lon}): {d:.1f}m ({status})")

        # Sample a grid around the center
        print(f"\nDepth grid (5×5 around center, ~500m steps):")
        step = 0.005  # ~500m
        for di in range(-2, 3):
            row = []
            for dj in range(-2, 3):
                dd = db.get_depth(args.lat + di * step, args.lon + dj * step)
                if dd is None:
                    row.append("  ???")
                elif dd >= 0:
                    row.append(f" +{dd:3.0f}")
                else:
                    row.append(f" {dd:4.0f}")
            print("  ".join(row))
    else:
        print(f"\nNo data for ({args.lat}, {args.lon})")
