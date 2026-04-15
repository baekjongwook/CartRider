from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("cartrider_rmd_sdk"), "param", "motors.yaml"
    )

    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="CAN interface name for RMD hardware node",
    )

    hardware_node = Node(
        package="cartrider_rmd_sdk",
        executable="hardware_node",
        parameters=[config, {"can_interface": LaunchConfiguration("can_interface")}],
        output="screen",
    )

    return LaunchDescription([can_interface_arg, hardware_node])
