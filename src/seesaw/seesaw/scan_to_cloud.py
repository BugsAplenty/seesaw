import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64, Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def fixed_mount_matrix(plane, rot_rad):
    """Fixed rotation of the raw lidar frame caused by side-mounting / plane remap.

    'vertical' (lidar on its side): raw scan (x, y, 0) maps to
    (x*cos - y*sin, 0, x*sin + y*cos) -- the bake-in math.
    Extended to a proper rotation (det = +1) so it can also be a TF.
    'horizontal': in-plane rotation about Z.
    """
    c, s = math.cos(rot_rad), math.sin(rot_rad)
    if plane == 'vertical':
        return np.array([[c, -s, 0.0],
                         [0.0, 0.0, -1.0],
                         [s,  c, 0.0]])
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])


def mat3_to_quat(R):
    """Rotation matrix -> (x, y, z, w) quaternion (Shepperd's method)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return np.array([(R[2, 1] - R[1, 2]) / s,
                         (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s,
                         0.25 * s])
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return np.array([0.25 * s,
                        (R[0, 1] + R[1, 0]) / s,
                        (R[0, 2] + R[2, 0]) / s,
                         (R[2, 1] - R[1, 2]) / s])
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return np.array([(R[0, 1] + R[1, 0]) / s,
                         0.25 * s,
                        (R[1, 2] + R[2, 1]) / s,
                         (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return np.array([(R[0, 2] + R[2, 0]) / s,
                    (R[1, 2] + R[2, 1]) / s,
                     0.25 * s,
                     (R[1, 0] - R[0, 1]) / s])


class ScanToCloud(Node):
    """Fuses the lidar's 2D LaserScan with servo yaw into a 3D cloud."""

    def __init__(self):
        super().__init__('scan_to_cloud')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('tilt_topic', '/sc15_driver/tilt')
        self.declare_parameter('tilt_units', 'radians')  # 'degrees' or 'radians'
        self.declare_parameter('output_topic', '/points')
        self.declare_parameter('scan_plane', 'vertical')  # 'vertical' or 'horizontal'
        self.declare_parameter('plane_rotation_deg', 90.0)
        self.declare_parameter('pivot_x', 0.0)
        self.declare_parameter('pivot_y', 0.0)
        self.declare_parameter('pivot_z', 0.0)
        self.declare_parameter('accum_seconds', 3.0)
        self.declare_parameter('max_range', 12.0)  # meters; 0 disables
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('mount_frame', 'lidar_mount')
        self.declare_parameter('laser_frame', 'laser')

        p = lambda n: self.get_parameter(n).value

        self.pivot = np.array([p('pivot_x'), p('pivot_y'), p('pivot_z')],
                              dtype=np.float64)
        self.max_range = float(p('max_range'))
        self.accum_seconds = float(p('accum_seconds'))
        self.degrees = str(p('tilt_units')) == 'degrees'
        self.base_frame = str(p('base_frame'))
        self.mount_frame = str(p('mount_frame'))
        self.laser_frame = str(p('laser_frame'))

        # Fixed mount rotation (side-mount flip + in-plane rotation), as matrix
        # for the point math and quaternion for the static TF.
        self.M_fixed = fixed_mount_matrix(str(p('scan_plane')),
                                          math.radians(float(p('plane_rotation_deg'))))
        self.q_fixed = mat3_to_quat(self.M_fixed)

        self.latest_tilt = 0.0
        self.point_buffer = deque()
        self.time_buffer = deque()

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        self.scan_sub = self.create_subscription(LaserScan, str(p('scan_topic')),
                                                 self.scan_callback, qos)
        self.servo_sub = self.create_subscription(Float64, str(p('tilt_topic')),
                                                  self.servo_callback, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, str(p('output_topic')), qos)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.send_static_tf()

        self.timer = self.create_timer(0.05, self.publish_cloud_and_tf)
        self.get_logger().info(
            f"scan_to_cloud started: plane={p('scan_plane')} "
            f"rot={p('plane_rotation_deg')} deg, pivot={self.pivot.tolist()}, "
            f"scan={p('scan_topic')} tilt={p('tilt_topic')} "
            f"({'degrees' if self.degrees else 'radians'})")

    # --- helpers ---------------------------------------------------------

    def send_static_tf(self):
        """Fixed lidar_mount -> laser transform (the side-mount + plane rotation)."""
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = self.mount_frame
        st.child_frame_id = self.laser_frame
        st.transform.rotation.x = float(self.q_fixed[0])
        st.transform.rotation.y = float(self.q_fixed[1])
        st.transform.rotation.z = float(self.q_fixed[2])
        st.transform.rotation.w = float(self.q_fixed[3])
        self.static_tf_broadcaster.sendTransform(st)

    # --- callbacks -------------------------------------------------------

    def servo_callback(self, msg):
        self.latest_tilt = math.radians(msg.data) if self.degrees else msg.data

    def scan_callback(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return

        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if self.max_range > 0.0:
            valid &= ranges <= self.max_range
        r, a = ranges[valid], angles[valid]
        if r.size == 0:
            return

        # Points in the laser frame (scan plane, z = 0)
        pts = np.column_stack((r * np.cos(a), r * np.sin(a),
                               np.zeros_like(r)))

        # 1. Fixed mount rotation (side-mount flip + in-plane rotation)
        # 2. Translate into the servo axis frame
        p = pts @ self.M_fixed.T + self.pivot

        # 3. Servo yaw about Z, using the latest angle for the whole batch
        yaw = self.latest_tilt
        c, s = math.cos(yaw), math.sin(yaw)
        out = np.empty_like(p)
        out[:, 0] = c * p[:, 0] - s * p[:, 1]
        out[:, 1] = s * p[:, 0] + c * p[:, 1]
        out[:, 2] = p[:, 2]

        self.point_buffer.append(out.astype(np.float32))
        self.time_buffer.append(self.get_clock().now().nanoseconds)

    def publish_cloud_and_tf(self):
        now = self.get_clock().now()

        # Dynamic TF base -> lidar_mount: yaw about Z, lidar origin sweeps the arc.
        yaw = self.latest_tilt
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.mount_frame
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        c, s = math.cos(yaw), math.sin(yaw)
        t.transform.translation.x = float(c * self.pivot[0] - s * self.pivot[1])
        t.transform.translation.y = float(s * self.pivot[0] + c * self.pivot[1])
        t.transform.translation.z = float(self.pivot[2])
        self.tf_broadcaster.sendTransform(t)

        # Rolling window accumulation
        cutoff = now.nanoseconds - int(self.accum_seconds * 1e9)
        while self.time_buffer and self.time_buffer[0] < cutoff:
            self.time_buffer.popleft()
            self.point_buffer.popleft()
        if not self.point_buffer:
            return

        all_points = np.concatenate(self.point_buffer).astype(np.float32, copy=False)

        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.base_frame
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, all_points))


def main(args=None):
    rclpy.init(args=args)
    node = ScanToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
