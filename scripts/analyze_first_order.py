#!/usr/bin/env python3

import argparse
import csv
import math
from collections import defaultdict

import numpy as np


def load_csv(path):
    trials = defaultdict(lambda: {"step": [], "coast": []})

    with open(path, newline="") as f:
        reader = csv.DictReader(f)

        required = {
            "trial",
            "phase",
            "phase_time_s",
            "left_actual_A",
            "right_actual_A",
            "left_omega_rad_s",
            "right_omega_rad_s",
        }

        missing = required.difference(reader.fieldnames or [])

        if missing:
            raise SystemExit(
                f"Missing CSV columns: {sorted(missing)}"
            )

        for row in reader:
            phase = row["phase"]

            if phase not in ("step", "coast"):
                continue

            trial = int(row["trial"])

            trials[trial][phase].append(row)

    return trials


def arrays(rows, side):
    rows = sorted(
        rows,
        key=lambda r: float(r["phase_time_s"])
    )

    t = np.asarray(
        [float(r["phase_time_s"]) for r in rows],
        dtype=float
    )

    I = np.asarray(
        [float(r[f"{side}_actual_A"]) for r in rows],
        dtype=float
    )

    w = np.asarray(
        [float(r[f"{side}_omega_rad_s"]) for r in rows],
        dtype=float
    )

    return t, I, w


def cumulative_trapezoid(t, x):
    q = np.zeros_like(t)

    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]

        if dt > 0.0:
            q[k] = (
                q[k - 1]
                + 0.5 * (x[k - 1] + x[k]) * dt
            )
        else:
            q[k] = q[k - 1]

    return q


def fit_coast(
    rows,
    side,
    coast_start,
    min_speed,
    max_speed
):
    t, I, w = arrays(rows, side)

    if len(t) < 10:
        return None

    # Ignore the beginning of coast because current does not
    # immediately fall to exactly zero.
    use = (
        (t >= coast_start)
        & (np.abs(w) >= min_speed)
    )

    if max_speed > 0.0:
        use &= np.abs(w) <= max_speed

    idx = np.where(use)[0]

    if len(idx) < 20:
        return None

    # Use one contiguous selected region.
    i0 = idx[0]
    i1 = idx[-1] + 1

    t = t[i0:i1]
    w = w[i0:i1]
    I = I[i0:i1]

    # Reset local time.
    t = t - t[0]

    w0 = w[0]

    int_w = cumulative_trapezoid(t, w)

    # Model:
    #
    # w(t) - w0 = -a * integral(w dt) + d_c * t
    #
    # regression coefficients:
    # beta[0] = -a
    # beta[1] = d_c

    X = np.column_stack([
        int_w,
        t
    ])

    y = w - w0

    beta, _, rank, _ = np.linalg.lstsq(
        X, y, rcond=None
    )

    a = -float(beta[0])
    d_coast = float(beta[1])

    yhat = X @ beta

    residual = y - yhat

    ss_res = float(
        residual @ residual
    )

    yc = y - np.mean(y)

    ss_tot = float(
        yc @ yc
    )

    r2 = (
        1.0 - ss_res / ss_tot
        if ss_tot > 1e-15
        else float("nan")
    )

    return {
        "a": a,
        "d_coast": d_coast,
        "coast_r2": r2,
        "coast_samples": len(t),
        "coast_start_speed": float(w[0]),
        "coast_end_speed": float(w[-1]),
        "coast_duration": float(t[-1]),
        "coast_mean_abs_current": float(
            np.mean(np.abs(I))
        ),
        "coast_rank": int(rank),
    }


def fit_step(
    rows,
    side,
    a,
    fit_start
):
    t, I, w = arrays(rows, side)

    if len(t) < 10:
        return None

    use = t >= fit_start

    idx = np.where(use)[0]

    if len(idx) < 20:
        return None

    i0 = idx[0]

    t = t[i0:]
    I = I[i0:]
    w = w[i0:]

    t = t - t[0]

    w0 = w[0]

    int_w = cumulative_trapezoid(t, w)
    int_I = cumulative_trapezoid(t, I)

    # Model:
    #
    # w - w0 + a*integral(w dt)
    #     = b*integral(I dt) + d_s*t
    #
    # beta[0] = b
    # beta[1] = d_s

    y = (
        w
        - w0
        + a * int_w
    )

    X = np.column_stack([
        int_I,
        t
    ])

    beta, _, rank, _ = np.linalg.lstsq(
        X, y, rcond=None
    )

    b = float(beta[0])
    d_step = float(beta[1])

    yhat = X @ beta

    residual = y - yhat

    ss_res = float(
        residual @ residual
    )

    yc = y - np.mean(y)

    ss_tot = float(
        yc @ yc
    )

    r2 = (
        1.0 - ss_res / ss_tot
        if ss_tot > 1e-15
        else float("nan")
    )

    return {
        "b": b,
        "d_step": d_step,
        "step_r2": r2,
        "step_samples": len(t),
        "step_mean_current": float(np.mean(I)),
        "step_mean_abs_current": float(
            np.mean(np.abs(I))
        ),
        "step_start_speed": float(w[0]),
        "step_end_speed": float(w[-1]),
        "step_rank": int(rank),
    }


def main():
    p = argparse.ArgumentParser()

    p.add_argument(
        "csv_file",
        help="CSV produced by first_order_identification"
    )

    p.add_argument(
        "--coast-start",
        type=float,
        default=0.20,
        help=(
            "Ignore this many seconds after current "
            "command is changed to 0 A "
            "(default 0.20)"
        )
    )

    p.add_argument(
        "--min-coast-speed",
        type=float,
        default=2.0,
        help=(
            "Ignore coast samples below this |omega| "
            "[rad/s] (default 2.0)"
        )
    )

    p.add_argument(
        "--max-coast-speed",
        type=float,
        default=0.0,
        help=(
            "Optional upper coast speed limit [rad/s]. "
            "0 means disabled."
        )
    )

    p.add_argument(
        "--step-start",
        type=float,
        default=0.02,
        help=(
            "Ignore initial step samples before this "
            "time [s] (default 0.02)"
        )
    )

    p.add_argument(
        "--output",
        default="first_order_fit.csv"
    )

    args = p.parse_args()

    trials = load_csv(args.csv_file)

    if not trials:
        raise SystemExit(
            "No step/coast trial data found."
        )

    output_rows = []

    print()
    print(
        "Model: omega_dot = -a*omega + b*I + d"
    )

    print(
        "a: identified from coast-down"
    )

    print(
        "b: identified from acceleration with a fixed"
    )

    print()

    for trial in sorted(trials):

        for side in ("left", "right"):

            coast_rows = trials[trial]["coast"]
            step_rows = trials[trial]["step"]

            if not coast_rows or not step_rows:
                continue

            coast = fit_coast(
                coast_rows,
                side,
                args.coast_start,
                args.min_coast_speed,
                args.max_coast_speed
            )

            if coast is None:
                print(
                    f"trial {trial} {side}: "
                    "coast fit failed"
                )
                continue

            if (
                not math.isfinite(coast["a"])
                or coast["a"] <= 0.0
            ):
                print(
                    f"trial {trial} {side}: "
                    f"invalid a={coast['a']}"
                )
                continue

            step = fit_step(
                step_rows,
                side,
                coast["a"],
                args.step_start
            )

            if step is None:
                print(
                    f"trial {trial} {side}: "
                    "step fit failed"
                )
                continue

            result = {
                "trial": trial,
                "wheel": side,

                **coast,
                **step,
            }

            result["T_s"] = (
                1.0 / coast["a"]
            )

            result["K_rad_s_per_A"] = (
                step["b"] / coast["a"]
            )

            output_rows.append(result)

            print(
                f"[trial {trial} {side}]"
            )

            print(
                f"  a             = "
                f"{coast['a']:.6f} 1/s"
            )

            print(
                f"  T=1/a         = "
                f"{result['T_s']:.6f} s"
            )

            print(
                f"  b             = "
                f"{step['b']:.6f} "
                f"(rad/s^2)/A"
            )

            print(
                f"  K=b/a         = "
                f"{result['K_rad_s_per_A']:.6f} "
                f"(rad/s)/A"
            )

            print(
                f"  coast R^2     = "
                f"{coast['coast_r2']:.6f}"
            )

            print(
                f"  step R^2      = "
                f"{step['step_r2']:.6f}"
            )

            print(
                f"  coast speed   = "
                f"{coast['coast_start_speed']:.3f}"
                f" -> "
                f"{coast['coast_end_speed']:.3f} "
                f"rad/s"
            )

            print(
                f"  coast mean |I|= "
                f"{coast['coast_mean_abs_current']:.4f} A"
            )

            print()

    if not output_rows:
        raise SystemExit(
            "No successful first-order fits."
        )

    fieldnames = list(
        output_rows[0].keys()
    )

    with open(
        args.output,
        "w",
        newline=""
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames
        )

        writer.writeheader()
        writer.writerows(output_rows)

    print(
        f"Wrote: {args.output}"
    )


if __name__ == "__main__":
    main()
