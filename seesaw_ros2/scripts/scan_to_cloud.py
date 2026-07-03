#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Float64
import numpy as np
import math
import time

class ScanToCloud(Node):
    def __init__(self):
        super().__init__('scan_to_cloud')
        
        # --- TILT AXIS CONFIG ---
        # 'Y' = Tilting forward/backward (Pitch)
        # 'X' = Tilting left/right (Roll)
        self.tilt_axis = 'Y'  
        
        # --- PIVOT OFFSET CONFIG ---
        # Distance (in meters) from the Servo axis to the Lidar optical center.
        # If the back of the lidar faces the servo, the lidar is forward of the servo.
        # e.g., if it's 5cm forward, pivot_x = 0.05
        self.declare_parameter('pivot_x', 0.0)
        self.declare_parameter('pivot_y', 0.0)
        self.declare_parameter('pivot_z', 0.0)
        self.pivot = np.array([
            self.get_parameter('pivot_x').value,
            self.get_parameter('pivot_y').value,
            self.get_parameter('pivot_z').value
        ])
        
        # Parameters
        self.declare_parameter('accum_seconds', 3.0)
        self.accum_seconds = self.get_parameter('accum_seconds').value
        
        # State
        self.latest_servo_tilt = 0.0
        self.point_buffer = []  # List of (time, x, y, z)
        
        # QoS for sensor data
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.servo_sub = self.create_subscription(Float64, '/servo_angle', self.servo_callback, 10)
        
        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, '/points', qos)
        
        # Timer to publish the accumulated cloud at 20Hz
        self.timer = self.create_timer(0.05, self.publish_cloud)
        
        self.get_logger().info(f"scan_to_cloud node started. Tilt: {self.tilt_axis}, Pivot: {self.pivot}")

    def servo_callback(self, msg):
        self.latest_servo_tilt = math.radians(msg.data)

    def scan_callback(self, msg):
        tilt = self.latest_servo_tilt
        
        # Convert LaserScan to numpy arrays
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)
        
        # Filter out invalid ranges
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        
        if len(ranges) == 0:
            return
            
        # 1. Calculate raw coordinates on the carousel plane (Z=0)
        x_raw = ranges * np.cos(angles)
        y_raw = ranges * np.sin(angles)
        z_raw = np.zeros_like(ranges)
        
        # Stack into Nx3 matrix
        points_raw = np.vstack((x_raw, y_raw, z_raw)).T  # Shape: (N, 3)
        
        # 2. Translate points to the Servo's frame (origin at servo axis)
        # If lidar is at pivot, a point in lidar frame is: P_servo = P_lidar + pivot
        points_servo = points_raw + self.pivot
        
        # 3. Apply Servo Tilt Rotation
        c = math.cos(tilt)
        s = math.sin(tilt)
        
        if self.tilt_axis == 'Y':
            # Rotation around Y axis
            R = np.array([
                [ c, 0, s],
                [ 0, 1, 0],
                [-s, 0, c]
            ])
        else:
            # Rotation around X axis
            R = np.array([
                [1, 0,  0],
                [0, c, -s],
                [0, s,  c]
            ])
            
        # Apply rotation: (3x3) @ (3xN) -> (3xN)
        points_rotated = (R @ points_servo.T).T
        
        current_time = time.time()
        
        # Add points to buffer
        for i in range(len(ranges)):
            self.point_buffer.append((current_time, float(points_rotated[i, 0]), float(points_rotated[i, 1]), float(points_rotated[i, 2])))

    def publish_cloud(self):
        if not self.point_buffer:
            return
            
        # Prune old points
        cutoff_time = time.time() - self.accum_seconds
        self.point_buffer = [p for p in self.point_buffer if p[0] >= cutoff_time]
        
        if not self.point_buffer:
            return
            
        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'lidar_link'
        
        # Extract XYZ coordinates
        points = [(p[1], p[2], p[3]) for p in self.point_buffer]
        
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
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