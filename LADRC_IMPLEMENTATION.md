# ZLAC8015D LADRC / b0 identification (experimentally verified PDO revision)

This revision is aligned with the actual ZLAC8015D behavior observed on the robot.

## Verified CANopen objects and PDO layout

The following were confirmed on the actual driver:

- `6061h = 4` after switching to Profile Torque Mode.
- `6071h:02 = 1000` commands +1.0 A to the right motor and the wheel rotates.
- `606Ch:01/:02` update correctly and are 32-bit signed values in 0.1 rpm.
- `6077h:01/:02` update correctly and are 16-bit signed values in 0.1 A.
- `606Ch:03` and `6077h:03` remained zero on this unit and are therefore not used.

The software configures:

```text
TPDO0  COB-ID 0x180 + node  (node 1 -> 0x181), 5 ms
  bytes 0..3 : 606Ch:01 left actual velocity  I32 [0.1 rpm]
  bytes 4..7 : 606Ch:02 right actual velocity I32 [0.1 rpm]

TPDO1  COB-ID 0x280 + node  (node 1 -> 0x281), 5 ms
  bytes 0..1 : 6077h:01 left actual current  I16 [0.1 A]
  bytes 2..3 : 6077h:02 right actual current I16 [0.1 A]

TPDO2  COB-ID 0x380 + node  (node 1 -> 0x381), 20 ms
  bytes 0..3 : 6064h:01 left position  I32
  bytes 4..7 : 6064h:02 right position I32
```

Both 0x181 and 0x281 were observed at approximately 5 ms intervals after this mapping was applied.

For LADRC torque commands, RPDO1 is configured as:

```text
RPDO1  COB-ID 0x300 + node  (node 1 -> 0x301)
  bytes 0..1 : 6071h:01 left target current  I16 [mA]
  bytes 2..3 : 6071h:02 right target current I16 [mA]
```

The `6071h:03` combined target object is not used in the revised implementation.

## Sign convention

ROS forward is positive on both wheels.

The actual robot requires the left driver sign to be reversed:

```text
ROS left velocity/current  = - driver left velocity/current
ROS right velocity/current = + driver right velocity/current
```

Therefore identified `b0_left` and `b0_right` can both be positive numbers in the ROS-forward coordinate system.

## CAN interface used on the current machine

The current USB-CAN adapter is SLCAN and has been used at 500 kbit/s:

```bash
sudo pkill slcand 2>/dev/null
sudo ip link set can0 down 2>/dev/null
sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ip link set can0 up
ip link show can0
```

`-s6` corresponds to 500 kbit/s for slcand. Do not also run `ip link set can0 type can bitrate ...` for this SLCAN interface.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Do not run the normal ros2_control bringup at the same time as `b0_identification`.

## b0 identification

First low-current check:

```bash
ros2 run zlac8015d_hardware b0_identification \
  --run \
  --can can0 \
  --node 1 \
  --currents 1.0 \
  --repeats 1 \
  --pulse 0.30 \
  --settle 1.0 \
  --sample-hz 200 \
  --max-current 1.0 \
  --output ~/b0_test_1A.csv
```

Then, after verifying direction and feedback, a low-current identification set can be run first:

```bash
ros2 run zlac8015d_hardware b0_identification \
  --run --can can0 --node 1 \
  --currents 1.0,1.5 \
  --repeats 3 \
  --pulse 0.30 \
  --settle 1.0 \
  --sample-hz 200 \
  --max-current 1.5 \
  --output ~/b0_identification_low.csv
```

The program deliberately uses SDO writes to the individually verified `6071h:01` and `6071h:02` objects only at the pulse edges. Feedback acquisition remains PDO-based at 200 Hz.

CSV contains commanded current, actual current, velocity and position for both wheels.

## b0 calculation

```bash
ros2 run zlac8015d_hardware analyze_b0.py \
  ~/b0_identification_low.csv \
  --min-r2 0.80 \
  --output ~/b0_trial_estimates.csv
```

The estimator fits each pulse using

```text
omega(t) - omega(0) = b0 * integral(I_actual dt) + f * t
```

rather than finite-difference differentiation.

Use the recommended left/right values as `ladrc_b0_left` and `ladrc_b0_right`.

## LADRC mode

The ros2_control command interface remains `velocity`, so `diff_drive_controller` and `/cmd_vel` do not change.

Inside the hardware plugin:

```text
wheel velocity reference
        -> LADRC
        -> left/right current command [A]
        -> RPDO1 6071h:01/:02 [mA]
        -> ZLAC8015D Profile Torque Mode
```

LESO:

```text
e = omega_measured - z1
beta1 = 2 * wo
beta2 = wo^2
z1 += dt * (z2 + b0 * I_actual + beta1 * e)
z2 += dt * (beta2 * e)
```

Controller:

```text
a_des = wc * (omega_ref - z1)
I_cmd = (a_des - z2) / b0
```

Then current saturation and current slew-rate limiting are applied.

Starting values:

```text
wc = 5 rad/s
wo = 20 rad/s
```

These are initial tuning values, not final research results.

## First LADRC activation

Keep `control_mode=velocity` until b0 has been measured.

Then set in the hardware parameters:

```xml
<param name="control_mode">ladrc_torque</param>
<param name="ladrc_b0_left">...</param>
<param name="ladrc_b0_right">...</param>
<param name="ladrc_wc">5.0</param>
<param name="ladrc_wo">20.0</param>
<param name="current_limit_a">0.5</param>
<param name="current_rate_limit_a_per_s">20.0</param>
<param name="feedback_timeout_ms">30.0</param>
```

Use a low current limit such as 0.5 A for the first closed-loop test. Increase only after confirming feedback sign, current direction and closed-loop stability.
