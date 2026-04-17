from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    controller_pkg_share = get_package_share_directory("cartrider_drive_controller")
    rmd_pkg_share = get_package_share_directory("cartrider_rmd_sdk")

    rearbot_param_file = os.path.join(controller_pkg_share, "param", "rearbot.yaml")

    rmd_launch_file = os.path.join(rmd_pkg_share, "launch", "bringup.launch.py")

    rearbot_node = Node(
        package="cartrider_drive_controller",
        executable="rearbot_control_node",
        name="rearbot_control_node",
        parameters=[rearbot_param_file],
        output="screen",
    )

    odom_node = Node(
        package="cartrider_drive_controller",
        executable="rearbot_odom_node",
        name="rearbot_odom_node",
        parameters=[rearbot_param_file],
        output="screen",
    )

    rmd_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rmd_launch_file),
        launch_arguments={
            "override_param_file": rearbot_param_file,
        }.items(),
    )

    return LaunchDescription(
        [
            rmd_bringup,
            rearbot_node,
            odom_node,
        ]
    )
