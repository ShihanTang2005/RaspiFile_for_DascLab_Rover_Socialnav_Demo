#!/usr/bin/env python3
"""
actuator_max_calibration_runner.py

Run this on the rover Raspberry Pi.

Purpose:
  Send known normalized ActuatorMotors commands directly to PX4 so you can
  measure the real max forward speed and max in-place angular speed with Vicon.

Default sequence:
  1) stop
  2) forward max:     left=+1.0, right=+1.0
  3) stop
  4) spin left max:   left=-1.0, right=+1.0
  5) stop
  6) spin right max:  left=+1.0, right=-1.0
  7) stop

It also publishes a JSON label on /calib_trial so the laptop logger can tag data.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from px4_msgs.msg import OffboardControlMode, VehicleCommand, ActuatorMotors


VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
ARMING_MAGIC = 21196.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ActuatorMaxCalibrationRunner(Node):
    def __init__(self):
        super().__init__("actuator_max_calibration_runner")

        # PX4 topics
        self.declare_parameter("offboard_topic", "/px4_1/fmu/in/offboard_control_mode")
        self.declare_parameter("motor_topic", "/px4_1/fmu/in/actuator_motors")
        self.declare_parameter("vehicle_command_topic", "/px4_1/fmu/in/vehicle_command")

        # Trial label topic, visible to laptop if ROS2 network is connected
        self.declare_parameter("trial_topic", "/calib_trial")

        # Test parameters
        self.declare_parameter("motor_abs", 1.0)           # normalized actuator magnitude
        self.declare_parameter("warmup_sec", 2.0)          # offboard setpoint warmup
        self.declare_parameter("post_arm_wait_sec", 1.0)
        self.declare_parameter("test_duration_sec", 4.0)   # each max-command window
        self.declare_parameter("stop_duration_sec", 1.5)   # stop between trials
        self.declare_parameter("repeat", 1)
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("include_reverse", False)

        self.motor_abs = float(self.get_parameter("motor_abs").value)
        self.motor_abs = clamp(abs(self.motor_abs), 0.0, 1.0)

        self.warmup_sec = float(self.get_parameter("warmup_sec").value)
        self.post_arm_wait_sec = float(self.get_parameter("post_arm_wait_sec").value)
        self.test_duration_sec = float(self.get_parameter("test_duration_sec").value)
        self.stop_duration_sec = float(self.get_parameter("stop_duration_sec").value)
        self.repeat = int(self.get_parameter("repeat").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.include_reverse = bool(self.get_parameter("include_reverse").value)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            self.get_parameter("offboard_topic").value,
            10,
        )
        self.motor_pub = self.create_publisher(
            ActuatorMotors,
            self.get_parameter("motor_topic").value,
            10,
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            self.get_parameter("vehicle_command_topic").value,
            10,
        )
        self.trial_pub = self.create_publisher(
            String,
            self.get_parameter("trial_topic").value,
            10,
        )

        self.sequence = self.build_sequence()

        self.start_time = time.time()
        self.phase_index = 0
        self.phase_start = self.start_time
        self.offboard_requested = False
        self.armed_requested = False
        self.finished = False

        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

        self.get_logger().warn(
            "Actuator max calibration runner started. "
            "Make sure the rover is lifted/clear first, then place it in the Vicon area."
        )
        self.get_logger().info(f"motor_abs={self.motor_abs:.2f}")
        self.get_logger().info("Sequence:")
        for name, left, right, duration in self.sequence:
            self.get_logger().info(
                f"  {name:>16s}: left={left:+.2f}, right={right:+.2f}, duration={duration:.1f}s"
            )

    def build_sequence(self):
        m = self.motor_abs
        seq = []

        def stop(label="stop"):
            seq.append((label, 0.0, 0.0, self.stop_duration_sec))

        for r in range(max(1, self.repeat)):
            suffix = f"_r{r+1}" if self.repeat > 1 else ""

            stop(f"stop_before_forward{suffix}")
            seq.append((f"forward_max{suffix}", +m, +m, self.test_duration_sec))

            if self.include_reverse:
                stop(f"stop_before_reverse{suffix}")
                seq.append((f"reverse_max{suffix}", -m, -m, self.test_duration_sec))

            stop(f"stop_before_spin_left{suffix}")
            seq.append((f"spin_left_max{suffix}", -m, +m, self.test_duration_sec))

            stop(f"stop_before_spin_right{suffix}")
            seq.append((f"spin_right_max{suffix}", +m, -m, self.test_duration_sec))

        stop("final_stop")
        return seq

    def ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.ts()
        msg.command = int(command)
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.ts()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = True
        self.offboard_pub.publish(msg)

    def publish_motors(self, left, right):
        msg = ActuatorMotors()
        now = self.ts()
        msg.timestamp = now
        msg.timestamp_sample = now

        msg.control = [float("nan")] * 12
        msg.control[0] = float(left)
        msg.control[1] = float(right)

        # motor 1 and 2 reversible
        msg.reversible_flags = 3
        self.motor_pub.publish(msg)

    def publish_trial_label(self, phase_name, left, right):
        msg = String()
        msg.data = json.dumps({
            "stamp_wall": time.time(),
            "phase": phase_name,
            "left": float(left),
            "right": float(right),
            "motor_abs": float(self.motor_abs),
        })
        self.trial_pub.publish(msg)

    def loop(self):
        self.publish_offboard_mode()

        now = time.time()
        elapsed = now - self.start_time

        # Warmup: send zero actuator setpoints before OFFBOARD request.
        if elapsed < self.warmup_sec:
            self.publish_motors(0.0, 0.0)
            self.publish_trial_label("warmup_zero", 0.0, 0.0)
            return

        if not self.offboard_requested:
            self.get_logger().warn("Requesting OFFBOARD mode")
            self.send_command(VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            self.offboard_requested = True
            self.phase_start = now
            return

        if self.auto_arm and not self.armed_requested:
            if now - self.phase_start >= self.post_arm_wait_sec:
                self.get_logger().warn("Force arming")
                self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, ARMING_MAGIC)
                self.armed_requested = True
                self.phase_start = now
            self.publish_motors(0.0, 0.0)
            self.publish_trial_label("waiting_arm_zero", 0.0, 0.0)
            return

        if self.finished:
            self.publish_motors(0.0, 0.0)
            self.publish_trial_label("finished_zero", 0.0, 0.0)
            return

        if self.phase_index >= len(self.sequence):
            self.get_logger().warn("Calibration sequence finished. Sending final stop.")
            self.finished = True
            self.publish_motors(0.0, 0.0)
            self.publish_trial_label("finished_zero", 0.0, 0.0)
            return

        phase_name, left, right, duration = self.sequence[self.phase_index]

        if now - self.phase_start >= duration:
            self.get_logger().info(f"Finished phase: {phase_name}")
            self.phase_index += 1
            self.phase_start = now
            self.publish_motors(0.0, 0.0)
            self.publish_trial_label("transition_zero", 0.0, 0.0)
            return

        self.publish_motors(left, right)
        self.publish_trial_label(phase_name, left, right)

        self.get_logger().info(
            f"phase={phase_name}, left={left:+.2f}, right={right:+.2f}, "
            f"t={now - self.phase_start:.2f}/{duration:.2f}s",
            throttle_duration_sec=0.5,
        )

    def safe_stop(self):
        self.get_logger().warn("Safe stop: sending zero motors and disarm.")
        for _ in range(30):
            self.publish_offboard_mode()
            self.publish_motors(0.0, 0.0)
            self.publish_trial_label("safe_stop_zero", 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.03)

        if self.auto_arm:
            self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, ARMING_MAGIC)


def main():
    rclpy.init()
    node = ActuatorMaxCalibrationRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()