from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory("cartrider_drive_controller")
    param_file = os.path.join(pkg_share, "param", "frontbot.yaml")

    frontbot_node = Node(
        package="cartrider_drive_controller",
        executable="frontbot_control_node",
        name="frontbot_control_node",
        parameters=[param_file],
    )

    rmd_pkg_share = get_package_share_directory("cartrider_rmd_sdk")
    vesc_pkg_share = get_package_share_directory("cartrider_vesc_sdk")

    rmd_launch = os.path.join(rmd_pkg_share, "launch", "bringup.launch.py")
    vesc_launch = os.path.join(vesc_pkg_share, "launch", "bringup.launch.py")

    rmd_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rmd_launch),
        launch_arguments={"can_interface": "can_front_rmd"}.items(),
    )

    vesc_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vesc_launch),
        launch_arguments={"can_interface": "can_front_vesc"}.items(),
    )

    return LaunchDescription(
        [
            rmd_bringup,
            vesc_bringup,
            frontbot_node,
        ]
    )
