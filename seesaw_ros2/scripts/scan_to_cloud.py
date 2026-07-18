#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float64
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import time

class ScanToCloud(Node):
    def __init__(self):
        super().__init__('scan_to_cloud')
        
        # --- AXIS CONFIG ---
        self.tilt_axis = 'Z'  # Servo yaws around Z
        
        # --- SCAN PLANE FLIP CONFIG ---
        # 'vertical' = LiDAR is on its side, scanning a vertical wall (XZ plane)
        self.scan_plane = 'vertical'
        
        # Rotate the raw scan plane within itself (e.g., 90 deg to swap up/out)
        self.declare_parameter('plane_rotation_deg', 85.0)
        self.plane_rot = math.radians(self.get_parameter('plane_rotation_deg').value)
        
        # --- PIVOT OFFSET CONFIG ---
        self.declare_parameter('pivot_x', 0.0)
        self.declare_parameter('pivot_y', 0.0)
        self.declare_parameter('pivot_z', 0.0)
        self.pivot = np.array([
            self.get_parameter('pivot_x').value,
            self.get_parameter('pivot_y').value,
            self.get_parameter('pivot_z').value
        ], dtype=np.float32)
        
        # Parameters
        self.declare_parameter('accum_seconds', 3.0)
        self.accum_seconds = self.get_parameter('accum_seconds').value
        
        # State (Use lists of numpy arrays for fast accumulation)
        self.latest_servo_tilt = 0.0
        self.time_buffer = []
        self.point_buffer = []
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # QoS for sensor data
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(PointCloud2, '/raw_points', self.scan_callback, qos)
        self.servo_sub = self.create_subscription(Float64, '/servo_angle', self.servo_callback, 10)
        
        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, '/points', qos)
        
        # Timer to publish the accumulated cloud and TF at 20Hz
        self.timer = self.create_timer(0.05, self.publish_cloud_and_tf)
        
        self.get_logger().info(f"scan_to_cloud node started. Tilt: {self.tilt_axis}, Plane: {self.scan_plane}, Rot: {self.plane_rot}, Pivot: {self.pivot}")

    def servo_callback(self, msg):
        self.latest_servo_tilt = math.radians(msg.data)

    def scan_callback(self, msg):
        # Use the latest servo angle for the whole scan batch
        tilt = self.latest_servo_tilt
        yaw_rad = np.full(1, tilt) # We will broadcast this single angle to all points
        
        # Zero-copy read of the PointCloud2 data using numpy
        points_raw = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        x_raw = points_raw[:, 0]
        y_raw = points_raw[:, 1]
        
        if self.scan_plane == 'vertical':
            # Flip the horizontal scan (X, Y) into a vertical scan (X, Z)
            x_vert = x_raw
            z_vert = y_raw
            
            # Apply in-plane rotation (swaps up and out if 90 degrees)
            c_pr = math.cos(self.plane_rot)
            s_pr = math.sin(self.plane_rot)
            x_rot = x_vert * c_pr - z_vert * s_pr
            z_rot = x_vert * s_pr + z_vert * c_pr
            
            points_xyz = np.vstack((x_rot, np.zeros_like(x_rot), z_rot)).T
            valid = (points_xyz[:, 0] != 0.0) | (points_xyz[:, 2] != 0.0)
        else:
            points_xyz = np.vstack((x_raw, y_raw, np.zeros_like(x_raw))).T
            valid = (points_xyz[:, 0] != 0.0) | (points_xyz[:, 1] != 0.0)
            
        points_xyz = points_xyz[valid]
        
        if len(points_xyz) == 0:
            return

        # 1. Translate to Servo frame (base_link)
        points_servo = points_xyz + self.pivot
        
        # 2. Apply Yaw Rotation (Z-axis) using the single latest servo angle
        c_yaw = math.cos(yaw_rad[0])
        s_yaw = math.sin(yaw_rad[0])
        
        x_final = c_yaw * points_servo[:, 0] - s_yaw * points_servo[:, 1]
        y_final = s_yaw * points_servo[:, 0] + c_yaw * points_servo[:, 1]
        z_final = points_servo[:, 2]
        
        points_rotated = np.vstack((x_final, y_final, z_final)).T
        
        # Accumulate in lists
        current_time = time.time()
        self.time_buffer.append(current_time)
        self.point_buffer.append(points_rotated)
    
    def publish_cloud_and_tf(self):
        # 1. Broadcast TF (base_link -> lidar_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        
        c = math.cos(self.latest_servo_tilt)
        s = math.sin(self.latest_servo_tilt)
        
        # Z-axis rotation (Yaw)
        R = np.array([[ c, -s, 0], [ s, c, 0], [0, 0, 1]])
        t.transform.rotation.z = math.sin(self.latest_servo_tilt / 2.0)
        t.transform.rotation.w = math.cos(self.latest_servo_tilt / 2.0)
            
        # The lidar origin sweeps in an arc around the base_link
        pivot_rotated = R @ self.pivot
        
        t.transform.translation.x = float(pivot_rotated[0])
        t.transform.translation.y = float(pivot_rotated[1])
        t.transform.translation.z = float(pivot_rotated[2])
            
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publish Point Cloud
        if not self.point_buffer:
            return
            
        cutoff_time = time.time() - self.accum_seconds
        
        # Prune old chunks
        while self.time_buffer and self.time_buffer[0] < cutoff_time:
            self.time_buffer.pop(0)
            self.point_buffer.pop(0)
            
        if not self.point_buffer:
            return
            
        # Combine all numpy chunks instantly
        all_points = np.vstack(self.point_buffer)
        
        # Construct PointCloud2 manually (Much faster than create_cloud_xyz32)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link' 
        
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = all_points.shape[0]
        cloud_msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12 # 3 floats * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = all_points.tobytes()
        
        self.cloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()