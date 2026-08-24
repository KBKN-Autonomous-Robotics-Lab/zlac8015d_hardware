# ZLAC8015D LADRC implementation notes

This revision adds:

1. `b0_identification`: standalone SocketCAN current-pulse data acquisition.
2. `analyze_b0.py`: estimates left/right `b0` without differentiating velocity.
3. A selectable `ladrc_torque` mode in the ros2_control hardware plugin.
4. Explicit 5 ms velocity/current TPDO configuration and 20 ms position TPDO.
5. Current saturation, current slew-rate limiting, and velocity-feedback watchdog.

## 0. Important operating assumptions

- CANopen Node-ID = 1 unless changed in URDF / CLI.
- SocketCAN interface = `can0` unless changed.
- The current robot's verified sign convention is retained:
  - ROS forward left wheel is the negative driver direction.
  - ROS forward right wheel is the positive driver direction.
- The existing code's empirically verified combined velocity/current ordering is retained:
  Low 16 bit = left, High 16 bit = right.
  Some ZLAC8015D manual revisions describe the combined 606Ch:03 velocity ordering differently.
  Before free-running LADRC, verify the received left/right wheel velocities at low speed.
- `control_mode` remains `velocity` by default. Do not change it to `ladrc_torque`
  until `b0_left/right` have been identified.

## 1. Build

From the ROS 2 workspace root:

```bash
colcon build --packages-select zlac8015d_hardware zlac_can_bringup
source install/setup.bash
```

Bring the CAN interface up at the bitrate configured in the ZLAC8015D (1 Mbit/s is preferred for 200 Hz operation when the driver is configured accordingly):

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

Do not run `ros2_control_node` at the same time as `b0_identification`.

## 2. b0 identification experiment

The default test schedule is:

- current magnitudes: 1.0, 1.5, 2.0, 2.5, 3.0 A
- repetitions: 5 per current
- pulse duration: 0.30 s
- zero-current settle: 1.00 s
- command/logging loop: 200 Hz

The program requires `--run` deliberately so that an accidental invocation does not command motor current.

```bash
ros2 run zlac8015d_hardware b0_identification \
  --run \
  --can can0 \
  --node 1 \
  --currents 1.0,1.5,2.0,2.5,3.0 \
  --repeats 5 \
  --pulse 0.30 \
  --settle 1.00 \
  --sample-hz 200 \
  --max-current 3.0 \
  --output ~/b0_identification.csv
```

Start with a lower `--max-current` if the motor/robot rating or test environment requires it.
The CANopen object 6071h may accept much larger values, but that is not a safe-current specification for the motor.

For a later bidirectional validation test:

```bash
ros2 run zlac8015d_hardware b0_identification \
  --run --currents 1.0,1.5,2.0 --repeats 3 --bidirectional \
  --max-current 2.0 --output ~/b0_bidirectional.csv
```

Ctrl-C commands zero current before shutdown.

## 3. CSV columns

The acquisition program logs:

- `host_time_s`
- `trial`
- `phase`
- `phase_time_s`
- `target_current_A`
- `left_cmd_A`, `right_cmd_A`
- `left_actual_A`, `right_actual_A`
- `left_omega_rad_s`, `right_omega_rad_s`
- `left_position_rad`, `right_position_rad`

`actual_A` is used for identification because it includes the driver's internal current-loop/slope dynamics.

## 4. b0 estimation

Run:

```bash
ros2 run zlac8015d_hardware analyze_b0.py \
  ~/b0_identification.csv \
  --min-r2 0.80 \
  --output ~/b0_trial_estimates.csv
```

The estimator does not compute a noisy finite-difference acceleration. For each pulse it fits

```text
omega(t) - omega(0) = b0 * integral(I_actual dt) + f * t
```

where `f` is a nuisance constant-disturbance term over the short pulse. The script reports a per-trial fit and recommends the median positive `b0` among accepted trials.

The unit is:

```text
(rad/s^2) / A
```

Inspect the per-trial results. Large current-dependence of `b0`, low R^2, or systematic left/right discrepancies are useful experimental findings rather than values to hide.

## 5. Enable LADRC after identification

Edit `zlac_can_bringup/urdf/orange_go_2025.urdf`:

```xml
<param name="control_mode">ladrc_torque</param>
<param name="ladrc_b0_left">PUT_LEFT_B0_HERE</param>
<param name="ladrc_b0_right">PUT_RIGHT_B0_HERE</param>
<param name="ladrc_wc">5.0</param>
<param name="ladrc_wo">20.0</param>
<param name="current_limit_a">2.0</param>
<param name="current_rate_limit_a_per_s">20.0</param>
<param name="feedback_timeout_ms">30.0</param>
```

The controller manager is already changed to 200 Hz in `controllers.yaml`.
`diff_drive_controller.publish_rate` remains 50 Hz; navigation does not need to run at 200 Hz.

## 6. Implemented discrete LADRC

For each wheel:

```text
omega_dot = f + b0 * I
```

LESO:

```text
e_obs = omega_measured - z1
beta1 = 2 * wo
beta2 = wo^2
z1 <- z1 + dt * (z2 + b0 * I_actual + beta1 * e_obs)
z2 <- z2 + dt * (beta2 * e_obs)
```

Control law:

```text
a_des = wc * (omega_ref - z1)
I_adrc = (a_des - z2) / b0
```

Then:

```text
I_adrc -> current saturation -> current slew-rate limiter -> ZLAC8015D 6071h:03
```

The observer uses actual current feedback (`6077h:03`) when available; otherwise it temporarily uses the previous current command.

Initial values:

```text
wc = 5 rad/s
wo = 20 rad/s
```

These are starting values, not final tuned values.

## 7. Safety / first LADRC test

Recommended first test sequence:

1. Verify TPDO left/right signs at very low motion.
2. Set `current_limit_a` to 0.5-1.0 A for the first closed-loop test.
3. Command a small wheel-speed reference.
4. Confirm `z1` follows measured velocity and current sign accelerates the correct wheel direction.
5. Increase current limit gradually only after sign and stability are confirmed.
6. Then run flat-floor step-response experiments for `wc` and `wo` tuning.

In LADRC torque mode, if velocity feedback has been received and then becomes older than `feedback_timeout_ms`, the hardware plugin sends zero torque and returns a hardware error.

## 8. Files changed / added

Hardware package:

- `include/zlac8015d_hardware/zlac8015d_system.hpp` modified
- `src/zlac8015d_system.cpp` modified
- `src/b0_identification.cpp` added
- `scripts/analyze_b0.py` added
- `CMakeLists.txt` modified
- `LADRC_IMPLEMENTATION.md` added

Bringup package:

- `config/controllers.yaml`: controller manager 50 -> 200 Hz
- `urdf/orange_go_2025.urdf`: CAN/LADRC/safety parameters added; mode remains `velocity` initially
