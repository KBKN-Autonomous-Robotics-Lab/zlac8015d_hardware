#!/usr/bin/env python3
"""Robust global LADRC input-gain identification for ZLAC8015D v4 data.

The v4 acquisition program applies short, independently stopped, interleaved
positive/negative current pulses.  For each wheel, all accepted trials are
fitted together using the integral model

    Δω = b0 * ∫I_actual dt
         + f_pos * t * 1(target > 0)
         + f_neg * t * 1(target < 0)
         + c_v * ∫ω dt

The direction-specific f terms absorb approximately constant directional load /
Coulomb-friction effects.  c_v captures first-order speed-dependent effects.
Pooling multiple current magnitudes and both signs separates b0 much better than
fitting b0 and a constant disturbance from one nearly constant-current pulse.

No numerical differentiation of quantized velocity is used.
"""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

try:
    import numpy as np
except ImportError as exc:
    raise SystemExit(
        "numpy is required. Install with: sudo apt install python3-numpy"
    ) from exc


def load_pulses(path):
    grouped = defaultdict(list)
    with open(path, newline="") as f:
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
    return grouped


def prepare_trial(rows, current_key, velocity_key, fit_start, fit_end):
    rows = sorted(rows, key=lambda r: float(r["phase_time_s"]))
    if len(rows) < 8:
        return None

    t_raw = np.asarray([float(r["phase_time_s"]) for r in rows], dtype=float)
    i_raw = np.asarray([float(r[current_key]) for r in rows], dtype=float)
    w_raw = np.asarray([float(r[velocity_key]) for r in rows], dtype=float)

    t0 = t_raw[0]
    t = t_raw - t0
    target = float(rows[0]["target_current_A"])
    w0 = w_raw[0]

    q = np.zeros_like(t)
    wint = np.zeros_like(t)
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        if dt <= 0.0:
            q[k] = q[k - 1]
            wint[k] = wint[k - 1]
            continue
        q[k] = q[k - 1] + 0.5 * (i_raw[k - 1] + i_raw[k]) * dt
        wint[k] = wint[k - 1] + 0.5 * (w_raw[k - 1] + w_raw[k]) * dt

    use = (t >= fit_start) & (t <= fit_end)
    if int(use.sum()) < 8:
        return None

    return {
        "target": target,
        "start_speed": w0,
        "end_speed": w_raw[-1],
        "delta_speed": w_raw[-1] - w0,
        "mean_current": float(np.mean(i_raw)),
        "mean_abs_current": float(np.mean(np.abs(i_raw))),
        "t": t[use],
        "q": q[use],
        "wint": wint[use],
        "y": (w_raw - w0)[use],
        "samples": int(use.sum()),
    }


def fit_wheel(
    grouped, wheel, current_key, velocity_key, fit_start, fit_end,
    max_start_speed, min_mean_abs_current
):
    qc = []
    accepted = []

    for trial, rows in sorted(grouped.items()):
        tr = prepare_trial(rows, current_key, velocity_key, fit_start, fit_end)
        if tr is None:
            qc.append({
                "trial": trial, "wheel": wheel, "accepted": 0,
                "reason": "too_few_samples"
            })
            continue

        reasons = []
        if abs(tr["start_speed"]) > max_start_speed:
            reasons.append("start_speed")
        if tr["mean_abs_current"] < min_mean_abs_current:
            reasons.append("low_current")
        if tr["target"] == 0.0:
            reasons.append("zero_target")

        ok = not reasons
        qc.append({
            "trial": trial,
            "wheel": wheel,
            "target_current_A": tr["target"],
            "start_speed_rad_s": tr["start_speed"],
            "end_speed_rad_s": tr["end_speed"],
            "delta_speed_rad_s": tr["delta_speed"],
            "mean_current_A": tr["mean_current"],
            "mean_abs_current_A": tr["mean_abs_current"],
            "fit_samples": tr["samples"],
            "accepted": int(ok),
            "reason": "ok" if ok else "+".join(reasons),
        })
        if ok:
            accepted.append((trial, tr))

    if not accepted:
        return None, qc

    has_pos = any(tr["target"] > 0 for _, tr in accepted)
    has_neg = any(tr["target"] < 0 for _, tr in accepted)

    names = ["b0"]
    if has_pos:
        names.append("f_pos")
    if has_neg:
        names.append("f_neg")
    names.append("c_v")

    X_parts = []
    y_parts = []

    for _, tr in accepted:
        cols = [tr["q"]]
        if has_pos:
            cols.append(tr["t"] if tr["target"] > 0 else np.zeros_like(tr["t"]))
        if has_neg:
            cols.append(tr["t"] if tr["target"] < 0 else np.zeros_like(tr["t"]))
        cols.append(tr["wint"])
        X_parts.append(np.column_stack(cols))
        y_parts.append(tr["y"])

    X = np.vstack(X_parts)
    y = np.concatenate(y_parts)

    beta, residuals, rank, svals = np.linalg.lstsq(X, y, rcond=None)
    yhat = X @ beta
    resid = y - yhat

    ss_res = float(resid @ resid)
    y_centered = y - float(np.mean(y))
    ss_tot = float(y_centered @ y_centered)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-15 else float("nan")

    dof = len(y) - X.shape[1]
    sigma2 = ss_res / dof if dof > 0 else float("nan")
    xtx_inv = np.linalg.pinv(X.T @ X)
    cov = sigma2 * xtx_inv if math.isfinite(sigma2) else np.full_like(xtx_inv, np.nan)
    se = np.sqrt(np.maximum(np.diag(cov), 0.0))

    # Scale-invariant-ish conditioning diagnostic: normalize each regressor
    # column to unit 2-norm before computing condition number.
    norms = np.linalg.norm(X, axis=0)
    Xn = X.copy()
    for j, nrm in enumerate(norms):
        if nrm > 0:
            Xn[:, j] /= nrm
    cond = float(np.linalg.cond(Xn))

    coef = {name: float(beta[i]) for i, name in enumerate(names)}
    coef_se = {name: float(se[i]) for i, name in enumerate(names)}

    result = {
        "wheel": wheel,
        "b0": coef["b0"],
        "b0_se": coef_se["b0"],
        "b0_ci95_low": coef["b0"] - 1.96 * coef_se["b0"],
        "b0_ci95_high": coef["b0"] + 1.96 * coef_se["b0"],
        "f_pos": coef.get("f_pos", float("nan")),
        "f_neg": coef.get("f_neg", float("nan")),
        "c_v": coef["c_v"],
        "r2": r2,
        "condition_number": cond,
        "rank": int(rank),
        "parameters": X.shape[1],
        "samples": len(y),
        "accepted_trials": len(accepted),
        "positive_trials": sum(tr["target"] > 0 for _, tr in accepted),
        "negative_trials": sum(tr["target"] < 0 for _, tr in accepted),
    }
    return result, qc


def write_csv(path, rows, fieldnames):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in rows:
            w.writerow(row)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv_file", help="CSV from b0_identification v4")
    p.add_argument("--fit-start", type=float, default=0.02,
                   help="First pulse time used for regression [s] (default 0.02)")
    p.add_argument("--fit-end", type=float, default=0.18,
                   help="Last pulse time used for regression [s] (default 0.18)")
    p.add_argument("--max-start-speed", type=float, default=0.20,
                   help="Reject trial if |omega_start| exceeds this [rad/s]")
    p.add_argument("--min-current", type=float, default=0.05,
                   help="Reject trial if mean |I_actual| is below this [A]")
    p.add_argument("--output", default="b0_global_fit.csv",
                   help="Global fit summary CSV")
    p.add_argument("--trial-output", default="b0_trial_qc.csv",
                   help="Per-trial quality-control CSV")
    args = p.parse_args()

    if args.fit_start < 0 or args.fit_end <= args.fit_start:
        raise SystemExit("Require 0 <= --fit-start < --fit-end")

    grouped = load_pulses(args.csv_file)
    if not grouped:
        raise SystemExit("No pulse rows found")

    results = []
    qc_rows = []
    for wheel, current_key, velocity_key in (
        ("left", "left_actual_A", "left_omega_rad_s"),
        ("right", "right_actual_A", "right_omega_rad_s"),
    ):
        fit, qc = fit_wheel(
            grouped, wheel, current_key, velocity_key,
            args.fit_start, args.fit_end,
            args.max_start_speed, args.min_current
        )
        qc_rows.extend(qc)
        if fit is not None:
            results.append(fit)

    if not results:
        raise SystemExit("No wheel had enough accepted trials for fitting")

    fit_fields = [
        "wheel", "b0", "b0_se", "b0_ci95_low", "b0_ci95_high",
        "f_pos", "f_neg", "c_v", "r2", "condition_number",
        "rank", "parameters", "samples", "accepted_trials",
        "positive_trials", "negative_trials",
    ]
    write_csv(args.output, results, fit_fields)

    qc_fields = [
        "trial", "wheel", "target_current_A",
        "start_speed_rad_s", "end_speed_rad_s", "delta_speed_rad_s",
        "mean_current_A", "mean_abs_current_A", "fit_samples",
        "accepted", "reason",
    ]
    # Normalize sparse rows from too_few_samples.
    for row in qc_rows:
        for key in qc_fields:
            row.setdefault(key, "")
    write_csv(args.trial_output, qc_rows, qc_fields)

    print("Global integral model:")
    print("  Δω = b0*∫I_actual dt + f_dir*t + c_v*∫ω dt")
    print(f"Fit window: {args.fit_start:.3f} ... {args.fit_end:.3f} s")
    print(f"Wrote fit: {Path(args.output).expanduser()}")
    print(f"Wrote QC : {Path(args.trial_output).expanduser()}\n")

    for r in results:
        print(f"[{r['wheel']}]")
        print(f"  accepted trials : {r['accepted_trials']} "
              f"(+{r['positive_trials']} / -{r['negative_trials']})")
        print(f"  b0              : {r['b0']:.6f} (rad/s^2)/A")
        print(f"  95% CI          : [{r['b0_ci95_low']:.6f}, {r['b0_ci95_high']:.6f}]")
        print(f"  R^2             : {r['r2']:.6f}")
        print(f"  condition number: {r['condition_number']:.3f}")
        if r["condition_number"] > 30:
            print("  WARNING: regressors are still strongly correlated; "
                  "increase excitation diversity before using b0.")
        if r["b0"] <= 0 or r["b0_ci95_low"] <= 0:
            print("  WARNING: b0 is not reliably positive; do NOT use this value in LADRC.")
        if r["positive_trials"] == 0 or r["negative_trials"] == 0:
            print("  WARNING: only one current direction was available; "
                  "rerun acquisition with --bidirectional.")
        print()


if __name__ == "__main__":
    main()
