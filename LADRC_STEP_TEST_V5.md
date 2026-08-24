# LADRC step-response validation (v5)

Verified CAN layout retained:
- TPDO0 0x181: left/right actual velocity, I32 + I32, 5 ms
- TPDO1 0x281: left/right actual current, I16 + I16, 5 ms
- RPDO1 0x301: left/right target current, I16 + I16
- Left sign is inverted only at the driver/ROS convention boundary.

Default LADRC validation parameters:
- b0_left = b0_right = 25 (rad/s^2)/A
- wc = 2 rad/s
- wo = 8 rad/s
- current limit = 0.5 A
- current slew limit = 5 A/s
- loop/log rate = 200 Hz

The CSV contains:
omega_ref, omega, I_actual, I_cmd, z1, z2 for both wheels, plus feedback ages.

## Build

```bash
cd ~/ros2_ws
rm -rf build/zlac8015d_hardware install/zlac8015d_hardware
source /opt/ros/humble/setup.bash
colcon build --packages-select zlac8015d_hardware --symlink-install
source ~/ros2_ws/install/setup.bash
```

Do NOT run ros2_control/controllers simultaneously with this standalone test.

## Recommended sequence

1. Right wheel only, 5 rad/s:

```bash
ros2 run zlac8015d_hardware ladrc_step_response \
  --run --can can0 --node 1 \
  --wheel right --reference 5.0 \
  --pre 1.0 --step 1.0 --post 1.0 \
  --sample-hz 200 \
  --b0-left 25 --b0-right 25 \
  --wc 2 --wo 8 \
  --current-limit 0.5 \
  --current-rate-limit 5 \
  --output ~/ladrc_step_right_5.csv
```

2. Left wheel only, 5 rad/s.
3. Both wheels, 5 rad/s.
4. Only after the 5 rad/s tests are stable, both wheels at 10 rad/s.

The post phase sets omega_ref=0 while LADRC remains active, so it records controlled
deceleration. At the end the program sends 0 A and disables operation.

## Analyze

```bash
ros2 run zlac8015d_hardware analyze_ladrc_step.py ~/ladrc_step_right_5.csv
```

For the first tests, reject/stop the experiment if:
- measured omega initially moves opposite to positive reference,
- |I_cmd| remains saturated at 0.5 A without the expected acceleration,
- velocity feedback watchdog trips,
- oscillation grows rather than decays,
- z1 diverges from omega or |z2| grows rapidly without settling.
