import socket
import struct
import time
import csv
from collections import deque

# NEW BATCH FORMAT
HEADER_SIZE = 6   # uint32_t seq + uint16_t count  
POINT_SIZE = 12   # float dist + float angle + float phi + uint8_t quality (packed)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", 12345))
print("🚀 LiDAR BATCH Decoder | Port 12345 | Ctrl+C to stop")

stats = {'packets': 0, 'points': 0, 'scans': 0, 'errors': 0}
scan_buffer = []
csv_file = None
csv_writer = None

def print_radar(scan):
    print("\n" + "="*70)
    print(f"SCAN #{stats['scans']:3d} | {len(scan):4d} points")
    print("="*70)
    for i, (dist, angle, phi, qual) in enumerate(scan[:10]):
        print(f"  {i+1:2d}: {angle:7.1f}° {dist:6.0f}mm phi:{phi:6.1f}° Q:{qual:3d}")

try:
    with open('lidar_scans.csv', 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['seq', 'dist_mm', 'angle_deg', 'phi_deg', 'quality'])
        
        start_time = time.time()
        while True:
            data, addr = s.recvfrom(1024)  # Batch max ~520B
            stats['packets'] += 1
            
            if len(data) < HEADER_SIZE:
                stats['errors'] += 1
                print(f"\r⚠️ Short packet {len(data)}B from {addr}")
                continue
            
            # NEW HEADER: uint32_t seq + uint16_t count
            seq, count = struct.unpack_from("<IH", data, 0)
            offset = HEADER_SIZE
            
            packet_points = []
            for i in range(count):
                if offset + POINT_SIZE > len(data):
                    stats['errors'] += 1
                    break
                    
                # NEW POINT: float dist, angle, phi + uint8_t quality
                dist, angle, phi, quality = struct.unpack_from("<fffB", data, offset)
                offset += POINT_SIZE
                
                if 0 < dist <= 12000 and 0 <= angle < 360 and quality > 50:
                    packet_points.append((dist, angle, phi, quality))
                    csv_writer.writerow([seq, dist, angle, phi, quality])
                    scan_buffer.append((dist, angle, phi, quality))
                    stats['points'] += 1
            
            # Scan detection (simplified)
            if len(scan_buffer) >= 360 and packet_points:
                if packet_points[-1][1] < 10:  # New scan start
                    print_radar(scan_buffer[-360:])
                    stats['scans'] += 1
                    scan_buffer = scan_buffer[-90:]
            
            # Live stats
            now = time.time()
            if now - start_time >= 1.0:
                print(f"\r📊 {stats['packets']:4d}pkts/s | {stats['points']:5d}pts/s | "
                      f"{stats['scans']:3d} scans | err:{stats['errors']} | {addr} | seq:{seq}")
                start_time = now
                stats['packets'] = stats['points'] = 0

except KeyboardInterrupt:
    print(f"\n📈 FINAL: {stats['packets']*60:4d}pkts/min | "
          f"{stats['points']*60:6d}pts/min | Scans: {stats['scans']}")
