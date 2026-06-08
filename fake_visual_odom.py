#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry


class FakeVisualOdom(Node):
    def __init__(self):
        super().__init__('fake_visual_odom')
        self.pub = self.create_publisher(
            VehicleOdometry,
            '/px4_1/fmu/in/vehicle_visual_odometry',
            10
        )
        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

    def ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def loop(self):
        msg = VehicleOdometry()
        now = self.ts()

        msg.timestamp = now
        msg.timestamp_sample = now

        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [0.0, 0.0, 0.0]
        msg.q = [1.0, 0.0, 0.0, 0.0]

        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        msg.velocity = [0.0, 0.0, 0.0]
        msg.angular_velocity = [0.0, 0.0, 0.0]

        msg.position_variance = [0.01, 0.01, 0.01]
        msg.orientation_variance = [0.01, 0.01, 0.01]
        msg.velocity_variance = [0.01, 0.01, 0.01]

        msg.reset_counter = 0
        msg.quality = 100

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeVisualOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()