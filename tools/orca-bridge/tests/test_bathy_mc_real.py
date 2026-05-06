"""Monte Carlo on REAL NOAA ETOPO Botany Bay chart, head-to-head:
bootstrap+MCC (current production) vs patch-match."""
import math, random, statistics, sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from meridian_orca.chart_loader import load_npz
from meridian_orca.bathy_match import BathyMatch, BathyMatchConfig, METERS_PER_DEG_LAT, meters_per_deg_lon
from meridian_orca.bathy_patch import PatchMatchFilter, PatchFilterConfig


def run_trial(filter_kind, chart, seed, duration_s=60, dt=1.0,
              init_offset_m=30.0, outlier_period=None):
    rng = random.Random(seed)
    for _ in range(100):
        lat = chart.lat_min + chart.dlat * rng.uniform(4, chart.n_rows - 5)
        lon = chart.lon_min + chart.dlon * rng.uniform(4, chart.n_cols - 5)
        if chart.depth(lat, lon) > 2.0: break
    else:
        return None
    true_lat, true_lon = lat, lon
    heading = rng.uniform(0, 2 * math.pi)
    speed = rng.uniform(1.5, 2.5)
    vel_n = speed * math.cos(heading); vel_e = speed * math.sin(heading)
    ang = rng.uniform(0, 2 * math.pi)
    seed_lat = true_lat + init_offset_m * math.cos(ang) / METERS_PER_DEG_LAT
    seed_lon = true_lon + init_offset_m * math.sin(ang) / meters_per_deg_lon(true_lat)

    if filter_kind == "bootstrap":
        cfg = BathyMatchConfig(n_particles=2000, init_spread_m=40.0,
            process_noise_m_per_sqrt_s=0.3, depth_noise_m=0.5,
            regularization_m=0.5, resample_threshold=0.6, mcc_enabled=True)
        pf = BathyMatch(chart, cfg, seed=seed)
    else:
        cfg = PatchFilterConfig(n_particles=1000, init_spread_m=40.0,
            process_noise_m_per_sqrt_s=0.25, depth_noise_m=0.5,
            regularization_m=0.3, resample_threshold=0.5,
            patch_window_s=8.0, patch_max_samples=8, mcc_enabled=True)
        pf = PatchMatchFilter(chart, cfg, seed=seed)
    pf.initialize(seed_lat, seed_lon)

    errs = []
    for step in range(int(duration_s / dt)):
        true_lat += vel_n * dt / METERS_PER_DEG_LAT
        true_lon += vel_e * dt / meters_per_deg_lon(true_lat)
        d = chart.depth(true_lat, true_lon)
        if math.isnan(d) or d < 0.5:
            break
        obs = d + rng.gauss(0, 0.3)
        if outlier_period and step % outlier_period == 0:
            obs += rng.choice([-3.0, 3.0])
        est = pf.step(vel_n, vel_e, dt, obs)
        err = math.hypot(
            (true_lat - est.lat) * METERS_PER_DEG_LAT,
            (true_lon - est.lon) * meters_per_deg_lon(true_lat),
        )
        errs.append(err)
    if len(errs) < 30: return None
    # Take the second half after convergence
    steady = errs[30:]
    return {
        "median": statistics.median(steady),
        "mean": statistics.mean(steady),
        "max": max(steady),
        "diverged": max(steady) > 150,
    }


def mc(chart, label, n=20, outlier=None):
    print(f"\n=== {label} (real NOAA ETOPO Botany Bay) ===")
    for kind in ["bootstrap", "patch"]:
        res = []
        for i in range(n):
            r = run_trial(kind, chart, 0x1000 + i, outlier_period=outlier)
            if r: res.append(r)
        if not res:
            print(f"  {kind}: no valid trials"); continue
        medians = [r["median"] for r in res]
        div = sum(1 for r in res if r["diverged"])
        print(f"  {kind:<12} n={len(res):2d}  "
              f"median-of-medians={statistics.median(medians):5.1f}m  "
              f"worst={max(medians):5.1f}m  diverged={div}/{len(res)}")


if __name__ == "__main__":
    chart = load_npz(r"D:\projects\meridian\tools\orca-bridge\tests\fixtures\etopo_botany.npz")
    mc(chart, "clean sonar", n=20, outlier=None)
    mc(chart, "outlier every 10", n=20, outlier=10)
    mc(chart, "harsh outlier every 5", n=20, outlier=5)
