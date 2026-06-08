#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, ActuatorMotors

VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
ARMING_MAGIC = 21196.0

class DirectMotorTest(Node):
    def __init__(self):
        super().__init__('direct_motor_test')

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/px4_1/fmu/in/offboard_control_mode',
            10
        )
        self.motor_pub = self.create_publisher(
            ActuatorMotors,
            '/px4_1/fmu/in/actuator_motors',
            10
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/px4_1/fmu/in/vehicle_command',
            10
        )

        self.count = 0
        self.timer = self.create_timer(0.02, self.loop)

    def ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.ts()
        msg.command = command
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
        msg.timestamp = self.ts()
        msg.timestamp_sample = self.ts()

        msg.control = [float('nan')] * 12
        msg.control[0] = float(left)
        msg.control[1] = float(right)

        msg.reversible_flags = 3
        self.motor_pub.publish(msg)

    def loop(self):
        self.publish_offboard_mode()

        # 前 2 秒只预热 offboard setpoint
        if self.count < 100:
            self.publish_motors(0.0, 0.0)

        elif self.count == 100:
            self.get_logger().info('Requesting OFFBOARD')
            self.send_command(VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            self.publish_motors(0.0, 0.0)

        elif self.count == 150:
            self.get_logger().info('Force arming')
            self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, ARMING_MAGIC)
            self.publish_motors(0.0, 0.0)

        # 小油门测试
        elif 200 <= self.count < 400:
            self.publish_motors(0.20, 0.20)

        else:
            self.publish_motors(0.0, 0.0)

        self.count += 1

    def safe_stop(self):
        for _ in range(20):
            self.publish_offboard_mode()
            self.publish_motors(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, ARMING_MAGIC)


def main():
    rclpy.init()
    node = DirectMotorTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()