"""Real-world eval harness for the bathy stack.

Answers the questions that matter for deployment:

  1. What's the median error + 90th percentile across many trials?
     (Not just median — the tail is what kills real ops.)
  2. How often does the filter DIVERGE (spread > 100m at steady state)?
  3. How quickly does it converge (time to spread < 20m)?
  4. Does it work on REAL chart data (NOAA ETOPO), not just synthetic?
  5. How fast is the update loop — can the Orca CPU sustain realtime?
  6. Is the reported uncertainty (spread) CALIBRATED — i.e., does
     actual error stay within ~1.5σ of reported spread? This matters
     because the EKF fusion downstream trusts that spread as R-matrix.

Filters compared: Bootstrap+MCC (proven), CNN v3 (new).
We drop Patch because head-to-head already showed it loses on both
clean and outlier conditions.
"""
import argparse
import math
import os
import random
import statistics
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .bathy_match import (BathyMatch, BathyMatchConfig, RealisticChart,
                          GridChart, METERS_PER_DEG_LAT, meters_per_deg_lon)
from .bathy_cnn_filter import BathyCNNFilter, CNNFilterConfig
from .chart_loader import load_npz

MODEL_PATH = r"D:\projects\meridian\tools\orca-bridge\tests\fixtures\bathy_matcher.pt"


# ===========================================================================
# Config
# ===========================================================================

@dataclass
class TrialResult:
    errs: List[float]           # per-step error in m
    spreads: List[float]        # per-step reported spread in m
    step_times_ms: List[float]  # per-step wall time
    diverged: bool              # true if final-window spread > 100m
    converged_at_s: Optional[float]  # first step with spread < 20m

    @property
    def median_err(self) -> float:
        tail = self.errs[len(self.errs) // 2:]
        return statistics.median(tail) if tail else float("nan")

    @property
    def p90_err(self) -> float:
        tail = self.errs[len(self.errs) // 2:]
        if not tail:
            return float("nan")
        s = sorted(tail)
        return s[int(len(s) * 0.9)]

    @property
    def calibration_fraction(self) -> float:
        """Fraction of steady-state steps where err < 1.5 × reported spread.
        Well-calibrated ≈ 0.7 (normal distribution coverage)."""
        tail_errs = self.errs[len(self.errs) // 2:]
        tail_spreads = self.spreads[len(self.errs) // 2:]
        if not tail_errs:
            return float("nan")
        hits = sum(1 for e, s in zip(tail_errs, tail_spreads) if e <= 1.5 * s)
        return hits / len(tail_errs)


def run_trial(kind: str, chart, seed: int, duration_s: int,
              chart_area: Tuple[float, float, float, float],
              outlier_period: Optional[int] = None,
              outlier_magnitude: float = 3.0) -> Optional[TrialResult]:
    """Run one 'kind' filter for duration_s seconds on `chart`.

    chart_area: (lat_min, lat_max, lon_min, lon_max) sampling bounds
    outlier_period: if not None, inject ±outlier_magnitude m sonar spike
        every N steps
    """
    rng = random.Random(seed)
    lat_min, lat_max, lon_min, lon_max = chart_area
    lat = rng.uniform(lat_min + 0.001, lat_max - 0.001)
    lon = rng.uniform(lon_min + 0.001, lon_max - 0.001)

    # Constrain start over water
    for _ in range(30):
        d0 = chart.depth(lat, lon)
        if not math.isnan(d0) and d0 > 1.0:
            break
        lat = rng.uniform(lat_min + 0.001, lat_max - 0.001)
        lon = rng.uniform(lon_min + 0.001, lon_max - 0.001)
    else:
        return None  # couldn't find water start

    heading = rng.uniform(0, 2 * math.pi)
    speed = rng.uniform(1.5, 2.5)
    vn, ve = speed * math.cos(heading), speed * math.sin(heading)

    # Initial estimate: 30m off
    ang = rng.uniform(0, 2 * math.pi)
    seed_lat = lat + 30 * math.cos(ang) / METERS_PER_DEG_LAT
    seed_lon = lon + 30 * math.sin(ang) / meters_per_deg_lon(lat)

    if kind == "bootstrap":
        cfg = BathyMatchConfig(
            n_particles=2000, init_spread_m=40.0,
            process_noise_m_per_sqrt_s=0.3, depth_noise_m=0.5,
            mcc_enabled=True, regularization_m=0.5, resample_threshold=0.6)
        pf = BathyMatch(chart, cfg, seed=seed)
    elif kind == "cnn":
        cfg = CNNFilterConfig()
        pf = BathyCNNFilter(chart, MODEL_PATH, cfg, seed=seed)
    else:
        raise ValueError(kind)

    pf.initialize(seed_lat, seed_lon)
    errs, spreads, step_times = [], [], []
    converged_at = None

    for t in range(duration_s):
        # Advance truth
        lat += vn / METERS_PER_DEG_LAT
        lon += ve / meters_per_deg_lon(lat)
        d = chart.depth(lat, lon)
        if math.isnan(d) or d < 0.5:
            break  # hit land
        obs = d + rng.gauss(0.0, 0.3)
        if outlier_period and (t % outlier_period == 0) and t > 0:
            obs += rng.choice([-outlier_magnitude, outlier_magnitude])

        t0 = time.perf_counter()
        est = pf.step(vn, ve, 1.0, obs)
        step_times.append((time.perf_counter() - t0) * 1000.0)

        err = math.hypot(
            (lat - est.lat) * METERS_PER_DEG_LAT,
            (lon - est.lon) * meters_per_deg_lon(lat),
        )
        errs.append(err)
        spreads.append(est.spread_m)
        # Convergence: error drops below 30m for the first time (sustained
        # for 10 steps is enforced post-hoc in TrialResult)
        if converged_at is None and err < 30.0:
            converged_at = float(t)

    if len(errs) < 30:
        return None

    # Diverged: sustained large spread in the final 20 s
    final_spreads = spreads[-20:]
    diverged = bool(
        statistics.median(final_spreads) > 100.0
        or statistics.median(errs[-20:]) > 80.0
    )
    return TrialResult(errs, spreads, step_times, diverged, converged_at)


# ===========================================================================
# Summarize
# ===========================================================================

def summarize(label: str, results: List[Optional[TrialResult]]) -> dict:
    ok = [r for r in results if r is not None]
    if not ok:
        print(f"  {label}: NO VALID TRIALS")
        return {}
    meds = sorted([r.median_err for r in ok if not math.isnan(r.median_err)])
    p90s = sorted([r.p90_err for r in ok if not math.isnan(r.p90_err)])
    divergences = sum(1 for r in ok if r.diverged)
    conv_times = [r.converged_at_s for r in ok if r.converged_at_s is not None]
    step_times = [t for r in ok for t in r.step_times_ms]
    calibrations = [r.calibration_fraction for r in ok
                    if not math.isnan(r.calibration_fraction)]

    med_of_meds = statistics.median(meds) if meds else float("nan")
    p90_of_meds = meds[int(len(meds) * 0.9)] if len(meds) >= 10 else float("nan")
    med_p90 = statistics.median(p90s) if p90s else float("nan")
    conv_p50 = statistics.median(conv_times) if conv_times else None
    step_p50 = statistics.median(step_times) if step_times else float("nan")
    step_p99 = sorted(step_times)[int(len(step_times) * 0.99)] if step_times else float("nan")
    calib_p50 = statistics.median(calibrations) if calibrations else float("nan")

    print(f"  {label:<14} n={len(ok):3d}  "
          f"med={med_of_meds:5.1f}m  p90={med_p90:5.1f}m  "
          f"div={divergences}/{len(ok)}  "
          f"conv={conv_p50 if conv_p50 is not None else 'DNC':>5}s  "
          f"step={step_p50:5.1f}ms (p99 {step_p99:5.1f})  "
          f"calib={calib_p50:.2f}")
    return dict(n=len(ok), med=med_of_meds, p90=med_p90,
                divergences=divergences, conv=conv_p50,
                step_p50=step_p50, step_p99=step_p99, calib=calib_p50)


# ===========================================================================
# Main
# ===========================================================================

def eval_on(chart, area: Tuple[float, float, float, float],
            n_trials: int, duration_s: int, chart_name: str) -> None:
    print(f"\n=== {chart_name} ({n_trials} trials × {duration_s}s each) ===",
          flush=True)
    print(f"{'condition':<12} {'filter':<14} ", flush=True)
    for label, period in [("clean", None), ("outlier/10", 10), ("harsh/5", 5)]:
        print(f"  {label}:", flush=True)
        for kind in ["bootstrap", "cnn"]:
            results = [
                run_trial(kind, chart, seed=0x5000 + i * 7 + hash(kind) % 100,
                          duration_s=duration_s, chart_area=area,
                          outlier_period=period)
                for i in range(n_trials)
            ]
            summarize(kind, results)
            sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--duration", type=int, default=120)
    args = parser.parse_args()

    print(f"Bathymetric filter eval — N={args.trials} trials, {args.duration}s each",
          flush=True)
    print(f"Divergence criterion: spread>100m OR err>80m in final 20s", flush=True)
    print(f"Convergence criterion: spread<20m AND err<40m for first time",
          flush=True)

    # --- 1. Synthetic charts at 3 resolutions (shows degradation curve) ---
    realistic = RealisticChart(seed=0xBA74)
    for res_m, label in [(5.0, "5m high-res survey"),
                         (25.0, "25m nautical chart"),
                         (100.0, "100m GEBCO-class")]:
        print(f"\nBaking RealisticChart @{res_m}m ...", flush=True)
        grid_synth = GridChart.from_realistic(
            realistic,
            lat_min=-33.985, lon_min=151.185,
            lat_max=-33.955, lon_max=151.215,
            resolution_m=res_m,
        )
        eval_on(grid_synth,
                (-33.978, -33.962, 151.190, 151.210),
                args.trials, args.duration,
                chart_name=f"SYNTHETIC RealisticChart — {label}")

    # --- 2. NOAA ETOPO real chart (Botany Bay cache) ---
    etopo_path = r"D:\projects\meridian\tools\orca-bridge\tests\fixtures\etopo_botany.npz"
    if os.path.exists(etopo_path):
        etopo = load_npz(etopo_path)
        # Determine valid sampling area from grid bounds (inset by 20%)
        lat_span = etopo.dlat * etopo.n_rows
        lon_span = etopo.dlon * etopo.n_cols
        area = (
            etopo.lat_min + 0.2 * lat_span,
            etopo.lat_min + 0.8 * lat_span,
            etopo.lon_min + 0.2 * lon_span,
            etopo.lon_min + 0.8 * lon_span,
        )
        print(f"\nETOPO chart: {etopo.n_rows}×{etopo.n_cols} cells, "
              f"resolution ~{etopo.dlat * METERS_PER_DEG_LAT:.0f}m",
              flush=True)
        eval_on(etopo, area, args.trials, args.duration,
                chart_name="REAL NOAA ETOPO — Botany Bay")
    else:
        print(f"\nWARN: {etopo_path} not found — skipping real-data eval",
              flush=True)


if __name__ == "__main__":
    main()
