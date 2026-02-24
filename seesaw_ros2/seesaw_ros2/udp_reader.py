#!/usr/bin/env python3
import rclpy, socket, struct, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class UDPLidar(Node):
    def __init__(self, port=12345):
        super().__init__('udp_lidar')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        
        self.scans = [float('inf')] * 3601
        self.timer = self.create_timer(0.02, self.publish_scan)
        self.points = []
        self.get_logger().info(f'🚀 ESP32 LiDAR UDP:{port}')

    def spin_once(self):
        try:
            data, addr = self.sock.recvfrom(1500)
            np = len(data) // 20
            for i in range(np):
                offset = i * 20
                ts, phi_i, angle_i, dist_i, qual = struct.unpack_from('<IIIIB', data, offset)
                if qual < 10 or dist_i == 0: continue
                
                angle_deg = angle_i / 100.0
                dist_m = dist_i / 1000.0
                bucket = int(angle_deg * 10) % 3600
                self.scans[bucket] = min(self.scans[bucket], dist_m)
        except: pass

    def publish_scan(self):
        self.spin_once()
        if any(f < 12.0 for f in self.scans):
            msg = LaserScan(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id='lidar_link'),
                angle_min=float(0.0),                    # ← EXPLICIT float()
                angle_max=float(2*math.pi),              # ← EXPLICIT float()
                angle_increment=float(2*math.pi/3600),   # ← EXPLICIT float()
                range_min=float(0.01),
                range_max=float(12.0),
                ranges=self.scans
            )
            self.pub.publish(msg)
            self.scans = [float('inf')] * 3601

def main():
    rclpy.init()
    node = UDPLidar(12345)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': main()
