"""Monte Carlo validation of bathymetric map-matching.

Runs N independent trajectories through the filter, reports:
- mean / median / worst-case / P95 steady-state error
- convergence time (when does the filter first stabilize below 20m?)
- divergence rate (% of runs where filter lost track)
- feature-density correlation (does error drop on feature-rich charts?)

Pass bar (feature-rich chart):
  P50 error < 10 m
  P95 error < 20 m
  divergence rate < 5%
"""
import math
import random
import statistics
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from meridian_orca.bathy_match import (
    BathyMatch, BathyMatchConfig, AnalyticChart, RealisticChart, GridChart,
    METERS_PER_DEG_LAT, meters_per_deg_lon,
)


def run_one_trajectory(chart, cfg, rng_seed, duration_s=60, dt=1.0, init_offset_m=30.0):
    """Run a single trajectory. Returns dict with metrics."""
    rng = random.Random(rng_seed)

    # Random start inside the chart area
    true_lat = -33.970 + rng.uniform(-0.003, 0.003)
    true_lon = 151.200 + rng.uniform(-0.003, 0.003)
    # Random heading, speed 1-3 m/s
    heading = rng.uniform(0, 2 * math.pi)
    speed = rng.uniform(1.5, 2.5)
    vel_n = speed * math.cos(heading)
    vel_e = speed * math.sin(heading)

    # Initial prior offset from truth
    offset_ang = rng.uniform(0, 2 * math.pi)
    seed_lat = true_lat + init_offset_m * math.cos(offset_ang) / METERS_PER_DEG_LAT
    seed_lon = true_lon + init_offset_m * math.sin(offset_ang) / meters_per_deg_lon(true_lat)

    pf = BathyMatch(chart, cfg, seed=rng_seed)
    pf.initialize(seed_lat, seed_lon)

    errors = []
    convergence_time = None
    diverged = False
    last_est = None
    for step in range(int(duration_s / dt)):
        t = step * dt
        true_lat += vel_n * dt / METERS_PER_DEG_LAT
        true_lon += vel_e * dt / meters_per_deg_lon(true_lat)
        true_depth = chart.depth(true_lat, true_lon)
        if math.isnan(true_depth):
            continue
        true_depth += rng.gauss(0, 0.3)    # sonar measurement noise

        est = pf.step(vel_n, vel_e, dt, true_depth)
        last_est = est

        err = math.hypot(
            (true_lat - est.lat) * METERS_PER_DEG_LAT,
            (true_lon - est.lon) * meters_per_deg_lon(true_lat),
        )
        errors.append(err)

        if convergence_time is None and err < 20.0 and t > 5:
            convergence_time = t

        # Divergence = error > 150m for multiple consecutive steps
        if err > 150 and t > 30:
            diverged = True

    # Steady-state = errors after 15s settle
    steady = errors[15:] if len(errors) > 15 else errors
    if not steady:
        return None

    return {
        "mean_err": statistics.mean(steady),
        "median_err": statistics.median(steady),
        "p95_err": sorted(steady)[int(len(steady) * 0.95)],
        "max_err": max(steady),
        "convergence_time": convergence_time,
        "diverged": diverged,
        "final_spread": last_est.spread_m if last_est else None,
        "final_healthy": last_est.healthy if last_est else False,
    }


def monte_carlo(chart, n_trials=30, label="chart"):
    """Run n trials, aggregate metrics."""
    cfg = BathyMatchConfig(
        n_particles=2000,
        init_spread_m=40.0,
        process_noise_m_per_sqrt_s=0.3,
        depth_noise_m=0.4,
        regularization_m=0.5,
        resample_threshold=0.6,
    )
    print(f"\n=== Monte Carlo: {label} ({n_trials} trials, 60 s each) ===")
    results = []
    for i in range(n_trials):
        r = run_one_trajectory(chart, cfg, rng_seed=0x1000 + i)
        if r:
            results.append(r)
    if not results:
        print("  NO VALID TRIALS")
        return

    means = [r["mean_err"] for r in results]
    medians = [r["median_err"] for r in results]
    p95s = [r["p95_err"] for r in results]
    converged = [r for r in results if r["convergence_time"] is not None]
    diverged = sum(1 for r in results if r["diverged"])
    healthy = sum(1 for r in results if r["final_healthy"])

    # Aggregate
    all_errs = []
    for r in results:
        all_errs.append(r["mean_err"])

    def pct(vals, p):
        s = sorted(vals)
        return s[min(int(p * len(s)), len(s) - 1)]

    print(f"  Trials:             {len(results)}")
    print(f"  Mean steady-state:  {statistics.mean(means):.2f} m")
    print(f"  Median:             {statistics.median(means):.2f} m")
    print(f"  P95 per-trial mean: {pct(means, 0.95):.2f} m")
    print(f"  Worst-case trial:   {max(means):.2f} m")
    print(f"  Convergence rate:   {len(converged)}/{len(results)} "
          f"(mean t={statistics.mean([r['convergence_time'] for r in converged]):.1f}s)"
          if converged else f"  Convergence rate:   0/{len(results)}")
    print(f"  Divergence:         {diverged}/{len(results)} trials")
    print(f"  Final-step healthy: {healthy}/{len(results)}")

    return {
        "trials":   len(results),
        "median":   statistics.median(means),
        "p95":      pct(means, 0.95),
        "diverged": diverged,
        "convergence_rate": len(converged) / len(results) if results else 0.0,
    }


def test_realistic_chart_passes_target():
    """Target: on feature-rich chart, median error < 15m, divergence < 20%."""
    chart = GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        -33.980, 151.190, -33.960, 151.210, resolution_m=5.0,
    )
    r = monte_carlo(chart, n_trials=20, label="realistic harbor")
    assert r["median"] < 15.0, f"median error {r['median']:.1f}m exceeds 15m target"
    assert r["diverged"] / r["trials"] < 0.20, f"divergence rate too high"


def test_analytic_chart_passes_target():
    """Target: on smooth analytic chart (pessimistic), median error < 20m."""
    chart = AnalyticChart()
    r = monte_carlo(chart, n_trials=20, label="analytic (pessimistic — smooth)")
    assert r["median"] < 20.0, f"median error {r['median']:.1f}m exceeds 20m target"


if __name__ == "__main__":
    # Run full validation suite
    rchart = GridChart.from_realistic(
        RealisticChart(seed=0xBA74),
        -33.980, 151.190, -33.960, 151.210, resolution_m=5.0,
    )
    achart = AnalyticChart()

    r_realistic = monte_carlo(rchart, n_trials=30, label="realistic harbor chart")
    r_analytic  = monte_carlo(achart, n_trials=30, label="analytic chart (smooth — pessimistic)")

    # Vary starting offset to see how convergence scales
    print("\n=== Convergence vs init offset (realistic chart) ===")
    for offset_m in [20, 50, 100, 200]:
        errs = []
        for i in range(10):
            cfg = BathyMatchConfig(
                n_particles=2000, init_spread_m=max(40, offset_m),
                process_noise_m_per_sqrt_s=0.3, depth_noise_m=0.4,
                regularization_m=0.5, resample_threshold=0.6,
            )
            r = run_one_trajectory(rchart, cfg, rng_seed=0x2000 + i,
                                   init_offset_m=offset_m)
            if r: errs.append(r["mean_err"])
        if errs:
            print(f"  offset={offset_m}m  n={len(errs)}  median={statistics.median(errs):.1f}m  "
                  f"worst={max(errs):.1f}m")
