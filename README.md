# Raspi Files for DASC Lab Social Navigation Rover

This repository contains the Raspberry Pi side scripts used to drive the DASC Lab social navigation rover through PX4 Offboard actuator control.

The main bridge subscribes to ROS 2 `/cmd_vel` velocity commands and publishes normalized left/right motor commands to PX4 through `px4_msgs/ActuatorMotors`.

> Safety note: several scripts in this repository request PX4 Offboard mode, arm the vehicle, and publish direct motor commands. Always test with the rover lifted or in a clear motion-capture area first.

## Repository Structure

| File | Purpose |
| --- | --- |
| `start_microXRCE_comm.sh` | Starts the Micro XRCE-DDS Agent on `/dev/ttyUSB0` at `921600` baud for communication between the Raspberry Pi and PX4 Cube Orange. |
| `cmd_vel_to_actuator_motors.py` | Main runtime bridge. Converts `/cmd_vel` into PX4 actuator motor commands with turn-priority saturation. |
| `actuator_min_ctrl.py` | Small direct-motor smoke test. It requests Offboard mode, arms PX4, briefly sends low forward motor output, then stops. |
| `fake_visual_odom.py` | Publishes fake visual odometry to PX4. This was used to suppress PX4 warnings and is not currently needed. |
| `test_code/actuator_max_calibration_runner.py` | Calibration runner for measuring max forward speed and max yaw rate with Vicon. |

## Requirements

- Raspberry Pi connected to PX4 Cube Orange over serial.
- PX4 configured for uXRCE-DDS / ROS 2 communication.
- ROS 2 environment with:
  - `rclpy`
  - `geometry_msgs`
  - `std_msgs`
  - `px4_msgs`
- `MicroXRCEAgent` installed and available in `PATH`.

Before running the scripts, source ROS 2 and the workspace that provides `px4_msgs`, for example:

```bash
source /opt/ros/<ros_distro>/setup.bash
source ~/px4_ros_com_ws/install/setup.bash
```

Replace `<ros_distro>` and the workspace path with the setup used on the rover.

## Quick Start

Start the Micro XRCE-DDS Agent:

```bash
bash start_microXRCE_comm.sh
```

Open a second terminal, source the ROS 2 environment, then run a low-power motor test:

```bash
python3 actuator_min_ctrl.py
```

If the rover direction and PX4 communication look correct, run the main bridge:

```bash
python3 cmd_vel_to_actuator_motors.py
```

The bridge listens on `/cmd_vel`. For a simple manual test, publish a small forward command at 10 Hz:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Press `Ctrl+C` to stop. The scripts send zero motor commands and disarm during shutdown.

## Main Bridge Behavior

`cmd_vel_to_actuator_motors.py` maps velocity commands to differential-drive motor outputs:

```text
forward = linear.x / v_real_max
turn    = angular.z / omega_real_max

left  = forward - turn
right = forward + turn
```

When a command is too large for the motor limits, turning is prioritized over forward motion. This is useful for obstacle avoidance and CBF-style controllers, where keeping turning authority is usually more important than maintaining forward speed.

Default PX4 topics:

| Parameter | Default |
| --- | --- |
| `cmd_vel_topic` | `/cmd_vel` |
| `offboard_topic` | `/px4_1/fmu/in/offboard_control_mode` |
| `motor_topic` | `/px4_1/fmu/in/actuator_motors` |
| `vehicle_command_topic` | `/px4_1/fmu/in/vehicle_command` |

<!-- Useful runtime parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `v_real_max` | `1.25` | Measured real forward speed when both motors are commanded to `+1.0`, in m/s. |
| `omega_real_max` | `3.35` | Measured real in-place yaw rate when motors are commanded in opposite directions, in rad/s. |
| `motor_total_max` | `1.00` | Max absolute motor command. Use `0.60` for first tests after changing bridge logic. |
| `output_scale` | `1.00` | Optional global scale applied after saturation. |
| `cmd_timeout` | `0.30` | Stop motors if no `/cmd_vel` arrives for this many seconds. |
| `swap_left_right` | `False` | Swap left and right motor outputs. |
| `invert_left` | `False` | Invert the left motor output. |
| `invert_right` | `False` | Invert the right motor output. |
| `invert_angular` | `False` | Invert the sign of `angular.z` before mapping. |

Example with conservative motor output:

```bash
python3 cmd_vel_to_actuator_motors.py --ros-args -p motor_total_max:=0.6
```

Example with a custom command topic:

```bash
python3 cmd_vel_to_actuator_motors.py --ros-args -p cmd_vel_topic:=/nav/cmd_vel
``` -->

## Calibration

Use `test_code/actuator_max_calibration_runner.py` to measure physical max speed and yaw rate. It sends direct normalized motor commands and publishes phase labels on `/calib_trial` so a laptop/Vicon logger can tag the data.

Default sequence:

1. Stop.
2. Forward max: `left=+1.0`, `right=+1.0`.
3. Stop.
4. Spin left max: `left=-1.0`, `right=+1.0`.
5. Stop.
6. Spin right max: `left=+1.0`, `right=-1.0`.
7. Stop.

Run:

```bash
python3 test_code/actuator_max_calibration_runner.py
```

Optional conservative calibration:

```bash
python3 test_code/actuator_max_calibration_runner.py --ros-args -p motor_abs:=0.6 -p test_duration_sec:=3.0
```

After measuring the rover in Vicon, update the runtime values passed to `cmd_vel_to_actuator_motors.py`:

```bash
python3 cmd_vel_to_actuator_motors.py --ros-args -p v_real_max:=1.25 -p omega_real_max:=3.35
```

## Troubleshooting

- If no PX4 topics appear, check that `MicroXRCEAgent` is running and that the serial device is correct. The default script uses `/dev/ttyUSB0`.
- If the rover does not move, confirm PX4 entered Offboard mode and accepted arming.
- If forward and turning directions are wrong, try `swap_left_right`, `invert_left`, `invert_right`, or `invert_angular`.
- If the rover stops while commands are being sent, make sure `/cmd_vel` is published faster than the `cmd_timeout` value.
