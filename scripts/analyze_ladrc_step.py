#!/usr/bin/env python3
import argparse
import csv
import math
from collections import defaultdict

def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv_file")
    args = p.parse_args()

    rows = []
    with open(args.csv_file, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit("Empty CSV")

    for wheel in ("left", "right"):
        refk = f"omega_ref_{wheel}_rad_s"
        wk = f"omega_{wheel}_rad_s"
        ik = f"current_actual_{wheel}_A"
        ick = f"current_cmd_{wheel}_A"
        z1k = f"z1_{wheel}_rad_s"
        z2k = f"z2_{wheel}_rad_s2"

        step = [r for r in rows if r["phase"] == "step" and abs(float(r[refk])) > 1e-9]
        if not step:
            print(f"[{wheel}] not excited")
            continue

        ref = float(step[0][refk])
        w = [float(r[wk]) for r in step]
        t = [float(r["phase_time_s"]) for r in step]
        ia = [float(r[ik]) for r in step]
        ic = [float(r[ick]) for r in step]
        z1 = [float(r[z1k]) for r in step]
        z2 = [float(r[z2k]) for r in step]

        err2 = [(ref-x)**2 for x in w]
        rmse = math.sqrt(sum(err2)/len(err2))
        peak = max(w) if ref >= 0 else min(w)
        overshoot = (peak-ref)/abs(ref)*100.0 if ref != 0 else float("nan")
        final_n = max(1, int(round(0.2 * len(w))))
        final_mean = sum(w[-final_n:])/final_n
        max_i_actual = max(abs(x) for x in ia)
        max_i_cmd = max(abs(x) for x in ic)
        max_abs_z2 = max(abs(x) for x in z2)
        z1_rmse = math.sqrt(sum((a-b)**2 for a,b in zip(w,z1))/len(w))

        # First crossing of 90% reference.
        rise90 = float("nan")
        threshold = 0.9 * ref
        for tt, ww in zip(t, w):
            if (ref >= 0 and ww >= threshold) or (ref < 0 and ww <= threshold):
                rise90 = tt
                break

        print(f"[{wheel}]")
        print(f"  reference       : {ref:.3f} rad/s")
        print(f"  final mean      : {final_mean:.3f} rad/s")
        print(f"  step RMSE       : {rmse:.3f} rad/s")
        print(f"  overshoot       : {overshoot:.1f} %")
        print(f"  t90             : {rise90:.3f} s" if math.isfinite(rise90) else "  t90             : not reached")
        print(f"  max |I_actual|  : {max_i_actual:.3f} A")
        print(f"  max |I_cmd|     : {max_i_cmd:.3f} A")
        print(f"  z1 tracking RMSE: {z1_rmse:.3f} rad/s")
        print(f"  max |z2|        : {max_abs_z2:.3f} rad/s^2")
        print()

if __name__ == "__main__":
    main()
