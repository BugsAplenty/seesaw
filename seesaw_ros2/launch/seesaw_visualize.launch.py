from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # UDP LiDAR/IMU
        Node(
            package='seesaw_ros2',
            executable='udp_reader',
            name='udp_reader',
            output='screen',
            parameters=[{
                'lidar_port': 8888,
                'imu_port': 8889
            }]
        ),
        # TF from URDF (absolute path)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open('/home/razboy/seesaw/seesaw_ros2/urdf/seesaw.urdf').read()
            }]
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
