import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('seesaw_ros2')
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
                'cloud_frame': 'base_link',
                'scan_frame': 'lidar_link',
                'imu_frame': 'imu_link',
                'scan_use_raw_lidar_frame': True,
                'zero_gyro_z': True,
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
                ('imu/data_raw', '/imu/data_raw'),
                ('imu/data', '/imu/data'),
            ]
        ),

        Node(
            package='seesaw_ros2',
            executable='pitch_joint_publisher.py',
            name='pitch_joint_publisher',
            output='screen',
            parameters=[{
                'joint_name': 'pitch_joint',
                'invert': False,
                'offset_rad': 0.0,
            }]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
