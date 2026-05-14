from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link']
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rko_lio',
                    executable='online_node',
                    name='rko_lio_online_node',
                    output='screen',
                    parameters=[
                        '/home/razboy/seesaw/seesaw_ros2/config/rko_lio.yaml',
                        {
                            'lidar_topic': '/points',
                            'imu_topic': '/imu/data',
                            'base_frame': 'base_link',
                            'deskew': False,
                        }
                    ]
                )
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
