"""Chart loaders — produce a `GridChart` from real bathymetric data files.

Supported formats:
- `.npz` (numpy)                       — our native cache format (fast reload)
- `.nc`  (NetCDF)                      — GEBCO 2024, NOAA, EMODnet standard
- `.tif` (GeoTIFF)                     — ENC export, survey processor output
- `.csv` (lat,lon,depth)               — simplest fallback
- `.xyz` (space-separated lon/lat/depth) — common survey format

Also provides a GEBCO HTTP fetcher for on-demand bounding-box downloads —
useful in the field when we know the operating area and want a chart
without lugging the full GEBCO global grid (~8 GB).
"""
import logging
import os
import struct
from dataclasses import dataclass
from typing import Optional, Tuple

from .bathy_match import GridChart


log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Format detection
# ---------------------------------------------------------------------------

def load_chart(path: str) -> GridChart:
    """Dispatch by file extension."""
    ext = os.path.splitext(path.lower())[1]
    if ext == ".npz":
        return load_npz(path)
    if ext in (".nc", ".nc4", ".ncdf"):
        return load_netcdf(path)
    if ext in (".tif", ".tiff", ".geotiff"):
        return load_geotiff(path)
    if ext == ".csv":
        return load_csv_grid(path)
    if ext == ".xyz":
        return load_xyz_grid(path)
    raise ValueError(f"unsupported chart format: {ext}")


# ---------------------------------------------------------------------------
# .npz native cache (fast path)
# ---------------------------------------------------------------------------

def load_npz(path: str) -> GridChart:
    """Native numpy .npz format.

    Required fields:
        data    2D float array, shape (nrows, ncols), depth in m (positive down)
        lat_min float
        lon_min float
        dlat    float (degrees per row)
        dlon    float (degrees per col)
    """
    try:
        import numpy as np
    except ImportError as e:
        raise RuntimeError("numpy required for .npz loading") from e
    z = np.load(path)
    data = z["data"]
    return GridChart(
        data=data.tolist() if hasattr(data, "tolist") else data,
        lat_min=float(z["lat_min"]),
        lon_min=float(z["lon_min"]),
        dlat=float(z["dlat"]),
        dlon=float(z["dlon"]),
    )


def save_npz(chart: GridChart, path: str) -> None:
    """Persist a GridChart to .npz for fast reload."""
    try:
        import numpy as np
    except ImportError as e:
        raise RuntimeError("numpy required for .npz saving") from e
    np.savez_compressed(
        path,
        data=np.array(chart.data, dtype=np.float32),
        lat_min=chart.lat_min,
        lon_min=chart.lon_min,
        dlat=chart.dlat,
        dlon=chart.dlon,
    )


# ---------------------------------------------------------------------------
# NetCDF — GEBCO, EMODnet, NOAA standard
# ---------------------------------------------------------------------------

def load_netcdf(path: str, depth_var: Optional[str] = None,
                lat_var: Optional[str] = None, lon_var: Optional[str] = None,
                flip_depth_sign: bool = True) -> GridChart:
    """Load a NetCDF bathymetry file (GEBCO, EMODnet, etc.).

    GEBCO 2024 convention:
        elevation  (lat, lon) — positive up (so seafloor is negative)
        lat        (lat,)     — degrees north, ascending
        lon        (lon,)     — degrees east,  ascending
    We invert sign so `data[i][j]` is depth in meters (positive down).
    """
    try:
        from netCDF4 import Dataset
    except ImportError as e:
        raise RuntimeError("netCDF4 required — `pip install netCDF4`") from e

    with Dataset(path, "r") as ds:
        # Auto-detect variable names
        if depth_var is None:
            for name in ("elevation", "depth", "z", "Band1", "bathymetry"):
                if name in ds.variables:
                    depth_var = name; break
        if lat_var is None:
            for name in ("lat", "latitude", "y"):
                if name in ds.variables:
                    lat_var = name; break
        if lon_var is None:
            for name in ("lon", "longitude", "x"):
                if name in ds.variables:
                    lon_var = name; break
        if not (depth_var and lat_var and lon_var):
            raise ValueError(f"could not find lat/lon/depth vars in {path}")

        lats = ds.variables[lat_var][:]
        lons = ds.variables[lon_var][:]
        data = ds.variables[depth_var][:]

    # Convert to a GridChart: data[i][j] where i=lat, j=lon, positive-down
    if hasattr(data, "filled"):       # masked array
        data = data.filled(0.0)
    data_list = data.tolist()
    if flip_depth_sign:
        data_list = [[-v if v != 0 else v for v in row] for row in data_list]

    dlat = float(lats[1] - lats[0])
    dlon = float(lons[1] - lons[0])
    # If dlat negative (descending lat), flip rows
    if dlat < 0:
        data_list = list(reversed(data_list))
        dlat = -dlat
        lat_min = float(lats[-1])
    else:
        lat_min = float(lats[0])
    lon_min = float(lons[0])

    log.info("loaded NetCDF chart: %d×%d, lat %.4f..%.4f, lon %.4f..%.4f",
             len(data_list), len(data_list[0]) if data_list else 0,
             lat_min, lat_min + dlat * (len(data_list) - 1),
             lon_min, lon_min + dlon * (len(data_list[0]) - 1) if data_list else 0)
    return GridChart(data_list, lat_min, lon_min, dlat, dlon)


# ---------------------------------------------------------------------------
# GeoTIFF — survey pipeline output
# ---------------------------------------------------------------------------

def load_geotiff(path: str, band: int = 1, flip_depth_sign: bool = True) -> GridChart:
    """Load a GeoTIFF bathymetry file."""
    try:
        import rasterio
    except ImportError as e:
        raise RuntimeError("rasterio required — `pip install rasterio`") from e

    with rasterio.open(path) as ds:
        data = ds.read(band)
        t = ds.transform
        # Transform: t[0]=dlon, t[4]=dlat, t[2]=lon_min, t[5]=lat_max
        dlon = t[0]
        dlat = t[4]
        # GeoTIFF typically has origin at top-left, so dlat is negative
        lon_min = t[2]
        if dlat < 0:
            # Flip vertically so data[0] is southernmost row
            data = data[::-1, :]
            lat_min = t[5] + dlat * data.shape[0]
            dlat = -dlat
        else:
            lat_min = t[5]

    data_list = data.tolist()
    if flip_depth_sign:
        data_list = [[-v if v != 0 else v for v in row] for row in data_list]
    log.info("loaded GeoTIFF chart: %d×%d", data.shape[0], data.shape[1])
    return GridChart(data_list, lat_min, lon_min, dlat, dlon)


# ---------------------------------------------------------------------------
# CSV / XYZ — simple tabular formats
# ---------------------------------------------------------------------------

def load_csv_grid(path: str, lat_col: int = 0, lon_col: int = 1,
                  depth_col: int = 2, has_header: bool = True,
                  delimiter: str = ",") -> GridChart:
    """Load a gridded CSV of lat,lon,depth rows. Assumes a regular grid."""
    points = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        if has_header:
            f.readline()
        for line in f:
            parts = line.strip().split(delimiter)
            if len(parts) <= max(lat_col, lon_col, depth_col):
                continue
            try:
                lat = float(parts[lat_col])
                lon = float(parts[lon_col])
                depth = float(parts[depth_col])
                points.append((lat, lon, depth))
            except ValueError:
                continue
    return _points_to_grid(points)


def load_xyz_grid(path: str) -> GridChart:
    """Load an XYZ (space-delimited lon/lat/depth) file."""
    return load_csv_grid(path, lat_col=1, lon_col=0, depth_col=2,
                         has_header=False, delimiter=" ")


def _points_to_grid(points) -> GridChart:
    """Bin (lat, lon, depth) points into a regular grid. Assumes points
    were sampled on a grid (i.e., unique lat and lon values form axes)."""
    if not points:
        raise ValueError("no valid points in chart file")
    lats = sorted(set(p[0] for p in points))
    lons = sorted(set(p[1] for p in points))
    lat_idx = {v: i for i, v in enumerate(lats)}
    lon_idx = {v: j for j, v in enumerate(lons)}
    data = [[0.0] * len(lons) for _ in range(len(lats))]
    for lat, lon, depth in points:
        data[lat_idx[lat]][lon_idx[lon]] = depth
    dlat = (lats[-1] - lats[0]) / max(1, len(lats) - 1)
    dlon = (lons[-1] - lons[0]) / max(1, len(lons) - 1)
    return GridChart(data, lats[0], lons[0], dlat, dlon)


# ---------------------------------------------------------------------------
# GEBCO on-demand fetcher
# ---------------------------------------------------------------------------

GEBCO_WCS_URL = (
    "https://www.gebco.net/data_and_products/gebco_web_services/web_map_service/mapserv?"
    "service=WMS&version=1.1.1&request=GetMap&layers=GEBCO_2024_Grid&srs=EPSG:4326&"
    "format=image/tiff&width={width}&height={height}&bbox={lon_min},{lat_min},{lon_max},{lat_max}"
)


def fetch_gebco(lat_min: float, lon_min: float,
                lat_max: float, lon_max: float,
                cache_path: str = "gebco_tile.npz",
                resolution_arcsec: float = 15.0,
                skip_if_exists: bool = True) -> GridChart:
    """Download a GEBCO 2024 tile for the bounding box, cache it, return
    as a GridChart.

    GEBCO 2024 has 15-arc-second resolution globally (~450m at equator,
    ~350m at mid latitudes). Free, no API key. Terms require attribution.

    For finer resolution (1-5m), use NOAA ENC (US), UKHO (UK), or
    customer-supplied multibeam data loaded via load_netcdf / load_geotiff.
    """
    if skip_if_exists and os.path.exists(cache_path):
        log.info("using cached GEBCO tile %s", cache_path)
        return load_npz(cache_path)
    try:
        import urllib.request
        import tempfile
        import numpy as np
    except ImportError as e:
        raise RuntimeError("urllib + numpy required for GEBCO fetch") from e

    # Compute pixel size for requested resolution
    deg_per_sec = resolution_arcsec / 3600.0
    width = max(10, int((lon_max - lon_min) / deg_per_sec))
    height = max(10, int((lat_max - lat_min) / deg_per_sec))
    url = GEBCO_WCS_URL.format(
        width=width, height=height,
        lon_min=lon_min, lat_min=lat_min, lon_max=lon_max, lat_max=lat_max,
    )
    log.info("fetching GEBCO: %d×%d px for bbox (%.4f,%.4f)-(%.4f,%.4f)",
             width, height, lat_min, lon_min, lat_max, lon_max)

    tmp = tempfile.NamedTemporaryFile(suffix=".tif", delete=False)
    tmp.close()
    try:
        urllib.request.urlretrieve(url, tmp.name)
        chart = load_geotiff(tmp.name)
    finally:
        try: os.unlink(tmp.name)
        except: pass

    save_npz(chart, cache_path)
    log.info("GEBCO tile cached to %s (%d rows × %d cols)",
             cache_path, chart.n_rows, chart.n_cols)
    return chart
