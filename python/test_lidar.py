import time
from rplidar import RPLidar

# Connect to USB0
lidar = RPLidar('/dev/ttyUSB0')

try:
    print('Starting Lidar Stream...')
    print('Press Ctrl+C to stop')
    
    # Get basic info
    info = lidar.get_info()
    print(f"Device Info: {info}")
    
    # Start scanning
    for i, scan in enumerate(lidar.iter_scans()):
        print(f'\n--- Scan {i} ---')
        
        # Print first 5 valid points of this scan
        valid_points = 0
        for (_, angle, distance) in scan:
            if distance > 0:
                print(f"Angle: {angle:.1f}° | Dist: {distance:.0f} mm")
                valid_points += 1
                if valid_points >= 5:
                    break
        
        # Stop after 20 scans
        if i >= 20:
            break
            
except KeyboardInterrupt:
    print('Stopping...')
except Exception as e:
    print(f"Error: {e}")
    
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
