from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    controller_pkg_share = get_package_share_directory("cartrider_drive_controller")
    rmd_pkg_share = get_package_share_directory("cartrider_rmd_sdk")
    vesc_pkg_share = get_package_share_directory("cartrider_vesc_sdk")

    frontbot_param_file = os.path.join(controller_pkg_share, "param", "frontbot.yaml")

    rmd_launch_file = os.path.join(rmd_pkg_share, "launch", "bringup.launch.py")

    vesc_launch_file = os.path.join(vesc_pkg_share, "launch", "bringup.launch.py")

    frontbot_node = Node(
        package="cartrider_drive_controller",
        executable="frontbot_control_node",
        name="frontbot_control_node",
        parameters=[frontbot_param_file],
        output="screen",
    )

    rmd_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rmd_launch_file),
        launch_arguments={
            "override_param_file": frontbot_param_file,
        }.items(),
    )

    vesc_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vesc_launch_file),
        launch_arguments={
            "override_param_file": frontbot_param_file,
        }.items(),
    )

    return LaunchDescription(
        [
            rmd_bringup,
            vesc_bringup,
            frontbot_node,
        ]
    )
