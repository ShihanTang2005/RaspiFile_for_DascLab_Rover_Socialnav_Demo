#!/usr/bin/env python3

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

        # Topics
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("offboard_topic", "/px4_1/fmu/in/offboard_control_mode")
        self.declare_parameter("motor_topic", "/px4_1/fmu/in/actuator_motors")
        self.declare_parameter("vehicle_command_topic", "/px4_1/fmu/in/vehicle_command")

        # cmd_vel 输入上限，需要和 laptop controller 保持一致
        self.declare_parameter("v_input_max", 0.1)   # m/s
        self.declare_parameter("w_input_max", 0.8)   # rad/s

        # 输出到 ActuatorMotors 的最大幅度，先保守一点
        self.declare_parameter("motor_forward_max", 0.16)
        self.declare_parameter("motor_turn_max", 0.45)
        self.declare_parameter("motor_total_max", 0.5)

        # 如果左右反了 / 转向反了，可以改这些
        self.declare_parameter("swap_left_right", False)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("invert_angular", False)

        # 如果多久没收到 cmd_vel，就停
        self.declare_parameter("cmd_timeout", 0.30)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.v_input_max = float(self.get_parameter("v_input_max").value)
        self.w_input_max = float(self.get_parameter("w_input_max").value)

        self.motor_forward_max = float(self.get_parameter("motor_forward_max").value)
        self.motor_turn_max = float(self.get_parameter("motor_turn_max").value)
        self.motor_total_max = float(self.get_parameter("motor_total_max").value)

        self.swap_left_right = bool(self.get_parameter("swap_left_right").value)
        self.invert_left = bool(self.get_parameter("invert_left").value)
        self.invert_right = bool(self.get_parameter("invert_right").value)
        self.invert_angular = bool(self.get_parameter("invert_angular").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.latest_v = 0.0
        self.latest_w = 0.0
        self.last_cmd_time = 0.0
        self.have_cmd = False

        self.count = 0
        self.offboard_requested = False
        self.armed_requested = False

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

        self.get_logger().info("cmd_vel -> actuator_motors bridge started.")
        self.get_logger().info(f"Subscribing: {self.cmd_vel_topic}")
        self.get_logger().info(
            f"motor_forward_max={self.motor_forward_max}, "
            f"motor_turn_max={self.motor_turn_max}, "
            f"motor_total_max={self.motor_total_max}"
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

    def cmd_vel_to_left_right(self, v, w):
        if self.invert_angular:
            w = -w

        # 先不直接用 wheel_base 物理模型，因为 actuator_motors 不是 m/s。
        # 这里做工程归一化：
        # v = v_input_max -> forward component = motor_forward_max
        # w = w_input_max -> turn component = motor_turn_max
        v_norm = clamp(v / self.v_input_max, -1.0, 1.0) if self.v_input_max > 1e-6 else 0.0
        w_norm = clamp(w / self.w_input_max, -1.0, 1.0) if self.w_input_max > 1e-6 else 0.0

        forward = v_norm * self.motor_forward_max
        turn = w_norm * self.motor_turn_max

        # ROS convention:
        # angular.z > 0 means turn left.
        # differential convention:
        # left slower/backward, right faster/forward.
        left = forward - turn
        right = forward + turn

        if self.swap_left_right:
            left, right = right, left

        if self.invert_left:
            left = -left
        if self.invert_right:
            right = -right

        left = clamp(left, -self.motor_total_max, self.motor_total_max)
        right = clamp(right, -self.motor_total_max, self.motor_total_max)

        return left, right

    def loop(self):
        self.publish_offboard_mode()

        # 前 2 秒只发 0，给 PX4 offboard setpoint 预热
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
        else:
            left, right = self.cmd_vel_to_left_right(self.latest_v, self.latest_w)

        self.publish_motors(left, right)

        self.get_logger().info(
            f"cmd_vel v={self.latest_v:.3f}, w={self.latest_w:.3f} -> "
            f"motors left={left:.3f}, right={right:.3f}",
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