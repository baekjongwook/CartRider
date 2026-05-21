from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    override_param_file_arg = DeclareLaunchArgument(
        "override_param_file",
        default_value="",
    )

    param_file = LaunchConfiguration("override_param_file")

    hardware_node = Node(
        package="cartrider_dynamixel_sdk",
        executable="hardware_node",
        name="hardware_node",
        output="screen",
        parameters=[param_file],
    )

    return LaunchDescription(
        [
            override_param_file_arg,
            hardware_node,
        ]
    )
