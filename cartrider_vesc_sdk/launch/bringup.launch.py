from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    default_param_file = os.path.join(
        get_package_share_directory("cartrider_vesc_sdk"), "param", "motors.yaml"
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the VESC hardware node.",
    )

    override_param_file_arg = DeclareLaunchArgument(
        "override_param_file",
        default_value=default_param_file,
        description="Override parameter file. Defaults to motors.yaml itself.",
    )

    hardware_node = Node(
        package="cartrider_vesc_sdk",
        executable="hardware_node",
        namespace=LaunchConfiguration("namespace"),
        name="vesc_hardware_node",
        parameters=[
            default_param_file,
            LaunchConfiguration("override_param_file"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            namespace_arg,
            override_param_file_arg,
            hardware_node,
        ]
    )
