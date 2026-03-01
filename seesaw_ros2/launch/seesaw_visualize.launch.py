from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your UDP Reader
        Node(
            package='seesaw_ros2',
            executable='udp_reader',
            name='udp_reader',
            output='screen'
        ),
        
        # --- ADD THE MADGWICK FILTER ---
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,          # MPU6050 has no compass
                'publish_tf': True,        # Let the filter broadcast the TF!
                'world_frame': 'enu',      # East-North-Up coordinate frame
                'fixed_frame': 'odom',     # The stationary room
                'reverse_tf': False
            }],
            # Remap to read from your UDP node
            remappings=[
                ('/imu/data_raw', '/imu')
            ]
        ),
        # TF from URDF
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
