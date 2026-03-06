import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = os.path.dirname(os.path.realpath(__file__)) + '/..'
    urdf_path = os.path.join(pkg_share, 'urdf', 'seesaw.urdf')
    
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='seesaw_ros2',
            executable='udp_reader',
            name='udp_reader',
            output='screen',
            parameters=[{
                'lidar_port': 12345,
                'imu_port': 12346,
                'publish_cloud': True,
                'publish_scan': True,
                'cloud_publish_ms': 80,
                'imu_publish_ms': 5,
                'tilt_axis': 'axis_y',
                'imu_frame': 'imu_link',
            }]
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
            }],
            remappings=[
                ('imu/data_raw', '/imu/data_raw'),  # udp_reader publishes here
                ('imu/data', '/imu/data'),          # filtered output
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])
