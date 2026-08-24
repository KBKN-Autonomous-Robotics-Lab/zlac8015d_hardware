# ZLAC8015D LADRC b0 identification v4

This version keeps the verified PDO layout:

- TPDO0 `0x181`: `606C:01` left actual velocity I32 + `606C:02` right actual velocity I32, 5 ms.
- TPDO1 `0x281`: `6077:01` left actual current I16 + `6077:02` right actual current I16, 5 ms.
- ROS-forward sign correction is applied to the left wheel.
- Identification current commands use the individually verified `6071:01` and `6071:02` objects.

## Why v4

v3 used a fixed zero-current settling time and fitted each almost-constant-current pulse independently.
That made `b0 * integral(I)` strongly collinear with a constant-disturbance `f*t` term and allowed
physically invalid negative `b0` values even with high R².

v4 changes both acquisition and analysis:

1. Before every pulse, command 0 A and wait until BOTH wheels satisfy `|omega| <= stop-threshold`
   continuously for `stop-hold`.
2. If they do not stop before `stop-timeout`, skip the trial without applying current.
3. With `--bidirectional`, interleave positive and negative pulses; the sign order is reversed every repeat.
4. Pool all accepted trials for each wheel into one integral regression.
5. Fit

       Delta omega = b0 * integral(I_actual dt)
                   + f_dir * t
                   + c_v * integral(omega dt)

   where `f_dir` has separate positive- and negative-pulse coefficients.
6. Report a 95% CI for b0 and a normalized-regressor condition number.

## Recommended lifted-wheel validation experiment

Start conservatively:

```bash
ros2 run zlac8015d_hardware b0_identification \
  --run \
  --can can0 \
  --node 1 \
  --currents 0.5,1.0,1.5 \
  --repeats 3 \
  --pulse 0.20 \
  --sample-hz 200 \
  --max-current 1.5 \
  --stop-threshold 0.15 \
  --stop-hold 0.50 \
  --stop-timeout 15.0 \
  --bidirectional \
  --output ~/b0_identification_v4.csv
```

If the free-spinning wheels require more than 15 s to coast below 0.15 rad/s, increase
`--stop-timeout` rather than relaxing the threshold immediately.

Analyze:

```bash
ros2 run zlac8015d_hardware analyze_b0.py \
  ~/b0_identification_v4.csv \
  --fit-start 0.02 \
  --fit-end 0.18 \
  --max-start-speed 0.20 \
  --output ~/b0_global_fit.csv \
  --trial-output ~/b0_trial_qc.csv
```

Inspect:

```bash
cat ~/b0_global_fit.csv
column -s, -t ~/b0_trial_qc.csv | less -S
```

Do not put b0 into LADRC merely because R² is high. Prefer:
- b0 > 0,
- 95% CI entirely above 0,
- several accepted positive and negative trials,
- condition number preferably below about 30,
- repeatability when the experiment is rerun.

Lifted-wheel b0 is primarily for validating signs, feedback, and controller implementation.
Final tuning should be repeated in the mechanically relevant operating condition.
