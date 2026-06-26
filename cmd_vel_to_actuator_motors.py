#!/usr/bin/env python3
"""
cmd_vel_to_actuator_motors_turn_priority.py

Raspberry Pi side bridge:
  /cmd_vel Twist(linear.x=v [m/s], angular.z=omega [rad/s])
      -> PX4 ActuatorMotors normalized left/right commands.

This version uses the calibrated real-world max speed/yaw-rate:

  v_real_max:
      real forward speed when actuator left=+1, right=+1

  omega_real_max:
      real yaw rate when actuator left=-1, right=+1 or left=+1, right=-1

Mapping:

  forward = v / v_real_max
  turn    = omega / omega_real_max

  left  = forward - turn
  right = forward + turn

Important saturation policy:
  If abs(forward) + abs(turn) exceeds motor_total_max,
  keep turn first and reduce forward.

Reason:
  For obstacle avoidance / CBF, turning authority is usually more safety-critical
  than forward speed. When commands are physically infeasible, the rover should
  slow down rather than lose turning ability.
"""

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, VehicleCommand, ActuatorMotors


VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
ARMING_MAGIC = 21196.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CmdVelToActuatorMotors(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_actuator_motors")

        # -------------------------
        # Topics
        # -------------------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("offboard_topic", "/px4_1/fmu/in/offboard_control_mode")
        self.declare_parameter("motor_topic", "/px4_1/fmu/in/actuator_motors")
        self.declare_parameter("vehicle_command_topic", "/px4_1/fmu/in/vehicle_command")

        # -------------------------
        # Calibration constants
        # -------------------------
        # These are physical actuator norm=1 limits measured by Vicon.
        # They are NOT the controller's desired v_max / w_max.
        #
        # From your latest max-actuator calibration:
        #   forward_max:    about 1.26 m/s
        #   spin max:       conservative about 2.96 rad/s
        #
        # Rounded defaults:
        self.declare_parameter("v_real_max", 1.25)       # m/s
        self.declare_parameter("omega_real_max", 3.00)   # rad/s

        # -------------------------
        # Safety limits
        # -------------------------
        # For the FIRST test after changing this bridge, keep this at 0.60.
        # After confirming direction and behavior, you may change it to 1.00.
        self.declare_parameter("motor_total_max", 1.00)

        # Optional global scaling after turn-priority limiting.
        # Usually keep 1.0. Set 0.5 if you want to make everything extra gentle.
        self.declare_parameter("output_scale", 1.00)

        # -------------------------
        # Direction fixes
        # -------------------------
        # Use these if the rover turns the wrong way or left/right are swapped.
        self.declare_parameter("swap_left_right", False)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("invert_angular", False)

        # If no cmd_vel arrives for this long, stop.
        self.declare_parameter("cmd_timeout", 0.30)

        # -------------------------
        # Read parameters
        # -------------------------
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.v_real_max = float(self.get_parameter("v_real_max").value)
        self.omega_real_max = float(self.get_parameter("omega_real_max").value)

        self.motor_total_max = float(self.get_parameter("motor_total_max").value)
        self.output_scale = float(self.get_parameter("output_scale").value)

        self.motor_total_max = clamp(abs(self.motor_total_max), 0.0, 1.0)
        self.output_scale = clamp(abs(self.output_scale), 0.0, 1.0)

        self.swap_left_right = bool(self.get_parameter("swap_left_right").value)
        self.invert_left = bool(self.get_parameter("invert_left").value)
        self.invert_right = bool(self.get_parameter("invert_right").value)
        self.invert_angular = bool(self.get_parameter("invert_angular").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        # -------------------------
        # State
        # -------------------------
        self.latest_v = 0.0
        self.latest_w = 0.0
        self.last_cmd_time = 0.0
        self.have_cmd = False

        self.count = 0
        self.offboard_requested = False
        self.armed_requested = False

        self.last_map_info = "init"

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_cb,
            10,
        )

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

        # 50 Hz loop
        self.timer = self.create_timer(0.02, self.loop)

        self.get_logger().info("turn-priority calibrated cmd_vel -> actuator_motors bridge started.")
        self.get_logger().info(f"Subscribing: {self.cmd_vel_topic}")
        self.get_logger().info(
            f"v_real_max={self.v_real_max:.3f} m/s, "
            f"omega_real_max={self.omega_real_max:.3f} rad/s, "
            f"motor_total_max={self.motor_total_max:.3f}, "
            f"output_scale={self.output_scale:.3f}"
        )
        self.get_logger().info(
            f"direction options: swap_left_right={self.swap_left_right}, "
            f"invert_left={self.invert_left}, invert_right={self.invert_right}, "
            f"invert_angular={self.invert_angular}"
        )

    def ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def cmd_cb(self, msg: Twist):
        self.latest_v = float(msg.linear.x)
        self.latest_w = float(msg.angular.z)
        self.last_cmd_time = time.time()
        self.have_cmd = True

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

    def twist_to_forward_turn(self, v, w):
        """
        Convert cmd_vel into normalized forward/turn components.
        """
        if self.invert_angular:
            w = -w

        if self.v_real_max > 1.0e-6:
            forward = v / self.v_real_max
        else:
            forward = 0.0

        if self.omega_real_max > 1.0e-6:
            turn = w / self.omega_real_max
        else:
            turn = 0.0

        # Safety clamp individual components before combined limiting.
        # This prevents absurd commands from producing absurd intermediate values.
        forward = clamp(forward, -1.0, 1.0)
        turn = clamp(turn, -1.0, 1.0)

        return forward, turn

    def apply_turn_priority_limit(self, forward, turn):
        """
        Enforce actuator feasibility while preserving turning authority first.

        For left = forward - turn, right = forward + turn,
        the necessary/sufficient condition for both wheels within +/-limit is:

            abs(forward) + abs(turn) <= limit

        If the command violates this, keep turn and reduce forward.

        If turn alone exceeds limit, clamp turn and set forward to zero.
        """
        limit = self.motor_total_max

        # Clamp turn first because turn has priority.
        turn_limited = clamp(turn, -limit, limit)

        # Whatever remains can be used for forward.
        allowed_forward = limit - abs(turn_limited)

        if allowed_forward <= 1.0e-9:
            forward_limited = 0.0
        else:
            forward_limited = math.copysign(
                min(abs(forward), allowed_forward),
                forward,
            )

        # Optional global output scale after feasibility limiting.
        forward_limited *= self.output_scale
        turn_limited *= self.output_scale

        return forward_limited, turn_limited

    def forward_turn_to_left_right(self, forward, turn):
        left = forward - turn
        right = forward + turn
        return left, right

    def apply_direction_options(self, left, right):
        if self.swap_left_right:
            left, right = right, left

        if self.invert_left:
            left = -left
        if self.invert_right:
            right = -right

        return left, right

    def cmd_vel_to_left_right(self, v, w):
        raw_forward, raw_turn = self.twist_to_forward_turn(v, w)

        limited_forward, limited_turn = self.apply_turn_priority_limit(
            raw_forward,
            raw_turn,
        )

        left, right = self.forward_turn_to_left_right(
            limited_forward,
            limited_turn,
        )

        left, right = self.apply_direction_options(left, right)

        # Final hard safety clamp after direction options.
        left = clamp(left, -self.motor_total_max, self.motor_total_max)
        right = clamp(right, -self.motor_total_max, self.motor_total_max)

        self.last_map_info = (
            f"raw_forward={raw_forward:+.3f}, raw_turn={raw_turn:+.3f}, "
            f"limited_forward={limited_forward:+.3f}, limited_turn={limited_turn:+.3f}"
        )

        return left, right

    def loop(self):
        self.publish_offboard_mode()

        # First 2 seconds: publish zero setpoints to warm up PX4 offboard.
        if self.count < 100:
            self.publish_motors(0.0, 0.0)
            self.count += 1
            return

        if self.count == 100 and not self.offboard_requested:
            self.get_logger().info("Requesting OFFBOARD")
            self.send_command(VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            self.offboard_requested = True

        if self.count == 150 and not self.armed_requested:
            self.get_logger().info("Force arming")
            self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, ARMING_MAGIC)
            self.armed_requested = True

        # cmd timeout safety
        if (not self.have_cmd) or (time.time() - self.last_cmd_time > self.cmd_timeout):
            left, right = 0.0, 0.0
            self.last_map_info = "timeout_or_no_cmd"
        else:
            left, right = self.cmd_vel_to_left_right(self.latest_v, self.latest_w)

        self.publish_motors(left, right)

        self.get_logger().info(
            f"cmd_vel v={self.latest_v:.3f}, w={self.latest_w:.3f} -> "
            f"motors left={left:+.3f}, right={right:+.3f} | {self.last_map_info}",
            throttle_duration_sec=0.5,
        )

        self.count += 1

    def safe_stop(self):
        self.get_logger().info("Safe stopping...")
        for _ in range(20):
            self.publish_offboard_mode()
            self.publish_motors(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, ARMING_MAGIC)


def main():
    rclpy.init()
    node = CmdVelToActuatorMotors()
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