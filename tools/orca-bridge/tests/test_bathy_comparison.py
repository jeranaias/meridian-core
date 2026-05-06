"""Head-to-head: bootstrap PF vs. RBPF on identical trajectories."""
import math
import random
import statistics
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from meridian_orca.bathy_match import (
    BathyMatch, BathyMatchConfig, RealisticChart, GridChart,
    METERS_PER_DEG_LAT, meters_per_deg_lon,
)
from meridian_orca.bathy_rbpf import RBPF, RBPFConfig


def run_trial_bootstrap(chart, rng_seed, duration_s=60, dt=1.0, init_offset_m=30.0,
                        outlier_period=None):
    rng = random.Random(rng_seed)
    true_lat = -33.970 + rng.uniform(-0.003, 0.003)
    true_lon = 151.200 + rng.uniform(-0.003, 0.003)
    heading = rng.uniform(0, 2 * math.pi)
    speed = rng.uniform(1.5, 2.5)
    vel_n = speed * math.cos(heading)
    vel_e = speed * math.sin(heading)
    ang = rng.uniform(0, 2 * math.pi)
    seed_lat = true_lat + init_offset_m * math.cos(ang) / METERS_PER_DEG_LAT
    seed_lon = true_lon + init_offset_m * math.sin(ang) / meters_per_deg_lon(true_lat)

    cfg = BathyMatchConfig(n_particles=2000, init_spread_m=40.0,
                           process_noise_m_per_sqrt_s=0.3, depth_noise_m=0.4,
                           regularization_m=0.5, resample_threshold=0.6)
    pf = BathyMatch(chart, cfg, seed=rng_seed)
    pf.initialize(seed_lat, seed_lon)

    errs = []
    for step in range(int(duration_s / dt)):
        true_lat += vel_n * dt / METERS_PER_DEG_LAT
        true_lon += vel_e * dt / meters_per_deg_lon(true_lat)
        true_depth = chart.depth(true_lat, true_lon)
        if math.isnan(true_depth): continue
        true_depth += rng.gauss(0, 0.3)
        if outlier_period and step % outlier_period == 0:
            true_depth += rng.choice([-3.0, 3.0])
        est = pf.step(vel_n, vel_e, dt, true_depth)
        err = math.hypot(
            (true_lat - est.lat) * METERS_PER_DEG_LAT,
            (true_lon - est.lon) * meters_per_deg_lon(true_lat),
        )
        errs.append(err)
    if len(errs) < 20: return None
    steady = errs[15:]
    return {
        "mean": statistics.mean(steady),
        "median": statistics.median(steady),
        "max": max(steady),
        "diverged": max(steady) > 150,
    }


def run_trial_rbpf(chart, rng_seed, duration_s=60, dt=1.0, init_offset_m=30.0,
                   outlier_period=None):
    rng = random.Random(rng_seed)
    true_lat = -33.970 + rng.uniform(-0.003, 0.003)
    true_lon = 151.200 + rng.uniform(-0.003, 0.003)
    heading = rng.uniform(0, 2 * math.pi)
    speed = rng.uniform(1.5, 2.5)
    vel_n = speed * math.cos(heading)
    vel_e = speed * math.sin(heading)
    ang = rng.uniform(0, 2 * math.pi)
    seed_lat = true_lat + init_offset_m * math.cos(ang) / METERS_PER_DEG_LAT
    seed_lon = true_lon + init_offset_m * math.sin(ang) / meters_per_deg_lon(true_lat)

    cfg = RBPFConfig(n_particles=2000, init_spread_m=40.0,
                     pos_noise_m_per_sqrt_s=0.3, depth_noise_m=0.4,
                     regularization_m=0.5, resample_threshold=0.6,
                     mcc_enabled=True)
    pf = RBPF(chart, cfg, seed=rng_seed)
    pf.initialize(seed_lat, seed_lon, vel_n=vel_n, vel_e=vel_e)

    errs = []
    for step in range(int(duration_s / dt)):
        true_lat += vel_n * dt / METERS_PER_DEG_LAT
        true_lon += vel_e * dt / meters_per_deg_lon(true_lat)
        true_depth = chart.depth(true_lat, true_lon)
        if math.isnan(true_depth): continue
        true_depth += rng.gauss(0, 0.3)
        if outlier_period and step % outlier_period == 0:
            true_depth += rng.choice([-3.0, 3.0])
        # Feed GPS-like velocity measurement
        m_vn = vel_n + rng.gauss(0, 0.2)
        m_ve = vel_e + rng.gauss(0, 0.2)
        est = pf.step(m_vn, m_ve, dt, true_depth)
        err = math.hypot(
            (true_lat - est.lat) * METERS_PER_DEG_LAT,
            (true_lon - est.lon) * meters_per_deg_lon(true_lat),
        )
        errs.append(err)
    if len(errs) < 20: return None
    steady = errs[15:]
    return {
        "mean": statistics.mean(steady),
        "median": statistics.median(steady),
        "max": max(steady),
        "diverged": max(steady) > 150,
    }


def monte_carlo_compare(chart, n_trials, outlier_period=None, label=""):
    bootstrap_res, rbpf_res = [], []
    for i in range(n_trials):
        b = run_trial_bootstrap(chart, 0x1000 + i, outlier_period=outlier_period)
        r = run_trial_rbpf(chart, 0x1000 + i, outlier_period=outlier_period)
        if b: bootstrap_res.append(b)
        if r: rbpf_res.append(r)

    def pct(vals, p):
        s = sorted(vals)
        return s[min(int(p * len(s)), len(s) - 1)]

    print(f"\n=== {label} ({n_trials} trials) ===")
    for name, res in [("Bootstrap PF (2000 p)", bootstrap_res), ("RBPF + MCC (2000 p)", rbpf_res)]:
        if not res:
            print(f"  {name}: no valid results")
            continue
        means = [r["mean"] for r in res]
        print(f"  {name:<25} median={statistics.median(means):5.1f}m  "
              f"P95={pct(means, 0.95):5.1f}m  "
              f"worst={max(means):5.1f}m  "
              f"diverged={sum(1 for r in res if r['diverged'])}/{len(res)}")


if __name__ == "__main__":
    chart = GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        -33.980, 151.190, -33.960, 151.210, resolution_m=5.0,
    )
    print("Building realistic chart... done.")
    monte_carlo_compare(chart, n_trials=20, outlier_period=None,
                        label="clean sonar (no outliers)")
    monte_carlo_compare(chart, n_trials=20, outlier_period=10,
                        label="sonar with outlier every 10 steps")
    monte_carlo_compare(chart, n_trials=20, outlier_period=5,
                        label="sonar with outlier every 5 steps (harsh)")
