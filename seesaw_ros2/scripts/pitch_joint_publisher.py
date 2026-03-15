#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def quat_to_pitch(x, y, z, w):
    sinp = 2.0 * (w * y - z * x)
    sinp = clamp(sinp, -1.0, 1.0)
    return math.asin(sinp)


class PitchJointPublisher(Node):
    def __init__(self):
        super().__init__('pitch_joint_publisher')

        self.joint_name = self.declare_parameter('joint_name', 'pitch_joint').value
        self.invert = self.declare_parameter('invert', False).value
        self.offset_rad = float(self.declare_parameter('offset_rad', 0.0).value)

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)

        self.get_logger().info(
            f'pitch_joint_publisher up: joint_name={self.joint_name} invert={self.invert} offset_rad={self.offset_rad:.4f}'
        )

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return

        pitch = quat_to_pitch(q.x, q.y, q.z, q.w)
        if self.invert:
            pitch = -pitch
        pitch += self.offset_rad

        js = JointState()
        js.header.stamp = msg.header.stamp if (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0) else self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [pitch]
        js.velocity = []
        js.effort = []

        self.pub.publish(js)


def main():
    rclpy.init()
    node = PitchJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
