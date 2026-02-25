#!/usr/bin/env python3
import socket, struct
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('0.0.0.0', 12346))
print("Listening IMU port 12346...")

count = 0
while True:
    data, addr = sock.recvfrom(64)
    print(f"IMU pkt #{count}: {len(data)}B from {addr}")
    if len(data) >= 28:
        ts = struct.unpack_from('<I', data, 0)[0]
        ax, ay, az, gx, gy, gz = struct.unpack_from('<6f', data, 4)
        print(f"  ts={ts} accel=({ax:.2f},{ay:.2f},{az:.2f}) gyro=({gx:.2f},{gy:.2f},{gz:.2f})")
    count += 1
