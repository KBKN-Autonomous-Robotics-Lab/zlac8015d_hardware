#!/usr/bin/env python3
"""Estimate LADRC b0 from b0_identification CSV without numerical differentiation.

For each pulse trial and wheel, fit

    omega(t) - omega(0) = b0 * integral(I_actual dt) + f * t

by least squares.  The nuisance coefficient f captures approximately constant
load/friction during the short pulse.  This avoids differentiating quantized
velocity feedback and uses the measured motor current rather than only I_cmd.
"""

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


def fit_trial(rows, current_key, velocity_key, ignore_s):
    rows = sorted(rows, key=lambda r: float(r["phase_time_s"]))
    rows = [r for r in rows if float(r["phase_time_s"]) >= ignore_s]
    if len(rows) < 8:
        return None

    t0 = float(rows[0]["phase_time_s"])
    omega0 = float(rows[0][velocity_key])

    q = 0.0
    last_t = t0
    last_i = float(rows[0][current_key])
    samples = []

    for row in rows:
        t_abs = float(row["phase_time_s"])
        t = t_abs - t0
        current = float(row[current_key])
        omega = float(row[velocity_key])

        dt = t_abs - last_t
        if dt < 0.0:
            continue
        q += 0.5 * (last_i + current) * dt
        last_t = t_abs
        last_i = current

        if t > 0.0:
            samples.append((q, t, omega - omega0))

    if len(samples) < 6:
        return None

    qq = sum(qi * qi for qi, _, _ in samples)
    qt = sum(qi * ti for qi, ti, _ in samples)
    tt = sum(ti * ti for _, ti, _ in samples)
    qy = sum(qi * yi for qi, _, yi in samples)
    ty = sum(ti * yi for _, ti, yi in samples)

    det = qq * tt - qt * qt
    if abs(det) < 1e-12:
        return None

    b0 = (qy * tt - ty * qt) / det
    disturbance = (qq * ty - qt * qy) / det

    y = [yi for _, _, yi in samples]
    y_hat = [b0 * qi + disturbance * ti for qi, ti, _ in samples]
    ss_res = sum((a - b) ** 2 for a, b in zip(y, y_hat))
    y_mean = sum(y) / len(y)
    ss_tot = sum((a - y_mean) ** 2 for a in y)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")

    mean_abs_current = sum(abs(float(r[current_key])) for r in rows) / len(rows)
    return {
        "b0": b0,
        "disturbance": disturbance,
        "r2": r2,
        "samples": len(samples),
        "mean_abs_current_A": mean_abs_current,
    }


def summarize(values):
    if not values:
        return None
    return {
        "n": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "std": statistics.stdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="CSV produced by b0_identification")
    parser.add_argument(
        "--ignore", type=float, default=0.0,
        help="Ignore this many seconds at the beginning of each pulse (default 0.0)")
    parser.add_argument(
        "--min-r2", type=float, default=0.80,
        help="Only include trials with R^2 >= this value in the recommended b0")
    parser.add_argument(
        "--output", default="b0_trial_estimates.csv",
        help="Per-trial fit output CSV")
    args = parser.parse_args()

    grouped = defaultdict(list)
    with open(args.csv_file, newline="") as f:
        reader = csv.DictReader(f)
        required = {
            "trial", "phase", "phase_time_s", "target_current_A",
            "left_actual_A", "right_actual_A",
            "left_omega_rad_s", "right_omega_rad_s",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"Missing CSV columns: {sorted(missing)}")
        for row in reader:
            if row["phase"] == "pulse":
                grouped[int(row["trial"])].append(row)

    results = []
    for trial, rows in sorted(grouped.items()):
        target = float(rows[0]["target_current_A"])
        for wheel, current_key, velocity_key in (
            ("left", "left_actual_A", "left_omega_rad_s"),
            ("right", "right_actual_A", "right_omega_rad_s"),
        ):
            fit = fit_trial(rows, current_key, velocity_key, args.ignore)
            if fit is None:
                continue
            results.append({
                "trial": trial,
                "wheel": wheel,
                "target_current_A": target,
                **fit,
            })

    if not results:
        raise SystemExit("No usable pulse trials were found")

    with open(args.output, "w", newline="") as f:
        fieldnames = [
            "trial", "wheel", "target_current_A", "mean_abs_current_A",
            "b0", "disturbance", "r2", "samples",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    print("Per-trial model: Δω = b0 * ∫I_actual dt + f * t")
    print(f"Wrote: {args.output}\n")

    for wheel in ("left", "right"):
        wheel_results = [r for r in results if r["wheel"] == wheel]
        accepted = [
            r for r in wheel_results
            if math.isfinite(r["r2"]) and r["r2"] >= args.min_r2 and r["b0"] > 0.0
        ]
        all_positive = [r for r in wheel_results if r["b0"] > 0.0]
        chosen = accepted if accepted else all_positive
        stats = summarize([r["b0"] for r in chosen])

        print(f"[{wheel}]")
        print(f"  trials fitted : {len(wheel_results)}")
        print(f"  accepted R^2≥{args.min_r2:.2f}: {len(accepted)}")
        if stats is None:
            print("  No positive b0 estimates. Check signs/feedback/PDO data.\n")
            continue
        print(f"  b0 mean       : {stats['mean']:.6f} (rad/s^2)/A")
        print(f"  b0 median     : {stats['median']:.6f} (rad/s^2)/A")
        print(f"  b0 std        : {stats['std']:.6f}")
        print(f"  range         : [{stats['min']:.6f}, {stats['max']:.6f}]")
        print(f"  RECOMMENDED b0: {stats['median']:.6f} (median of accepted trials)\n")


if __name__ == "__main__":
    main()
