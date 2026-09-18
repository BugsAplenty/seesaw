import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("seesaw")
    params_file = os.path.join(share, "config", "seesaw.yaml")

    # The lidar node is declared inline rather than including rplidar.launch.py:
    # upstream hardcodes /dev/ttyUSB0 and accepts no launch arguments.
    lidar = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_composition",
        output="screen",
        condition=IfCondition(LaunchConfiguration("lidar")),
        parameters=[{
            "serial_port": LaunchConfiguration("lidar_port"),
            "serial_baudrate": 115200,  # A1 / A2 family (A1M8)
            "frame_id": "laser",
            "inverted": False,          # flip if the cloud looks mirror-imaged
            "angle_compensate": True,
            # "scan_mode": "Boost",      # 8K samples/s; uncomment for max density
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("lidar", default_value="true",
                              description="Start the RPLidar A1M8 driver"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz2"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0",
                              description="Serial device of the RPLidar A1M8 "
                                          "(e.g. /dev/serial/by-id/usb-Silicon_Labs_CP210x_...)"),
        DeclareLaunchArgument("servo_port", default_value="/dev/ttyACM0",
                              description="Serial device of the SC15 servo bus "
                                          "(e.g. /dev/serial/by-id/usb-1a86_USB_Single_Serial_...)"),

        Node(
            package="seesaw",
            executable="sc15_driver",
            name="sc15_driver",
            output="screen",
            # launch argument wins over seesaw.yaml's port:
            parameters=[params_file, {"port": LaunchConfiguration("servo_port")}],
        ),
        Node(
            package="seesaw",
            executable="scan_to_cloud",
            name="scan_to_cloud",
            output="screen",
            parameters=[params_file],
        ),
        # Fixed offset from the robot/body to the rocking mount; edit z as needed.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_lidar_mount",
            arguments=["--x", "0", "--y", "0", "--z", "0.15",
                       "--frame-id", "base_link", "--child-frame-id", "lidar_mount"],
        ),
        lidar,
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
