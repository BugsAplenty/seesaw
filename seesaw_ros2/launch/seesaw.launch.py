import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('seesaw_ros2'),
        'rviz',
        'seesaw.rviz'
    )

    return LaunchDescription([
        # 1. C++ UDP Reader Node
        Node(
            package='seesaw_ros2',
            executable='udp_reader',
            name='udp_reader',
            parameters=[{
                'lidar_port': 12345,
                'publish_ms': 50,
            }],
            output='screen'
        ),
        
        # 2. Python Scan to 3D Cloud Node
        Node(
            package='seesaw_ros2',
            executable='scan_to_cloud.py',
            name='scan_to_cloud',
            parameters=[{
                'accum_seconds': 30.0,
                'pivot_x': 0.04,  
                'pivot_z': 0.05,  
            }],
            output='screen'
        ),
        
        # 3. RViz2 with pre-loaded config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])