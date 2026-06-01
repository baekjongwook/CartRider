from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    controller_pkg_share = get_package_share_directory("cartrider_drive_controller")
    cap_sim_pkg_share = get_package_share_directory("cap_sim_2026")
    rmd_pkg_share = get_package_share_directory("cartrider_rmd_sdk")
    dynamixel_pkg_share = get_package_share_directory("cartrider_dynamixel_sdk")

    rearbot_param_file = os.path.join(controller_pkg_share, "param", "rearbot.yaml")
    frontbot_param_file = os.path.join(controller_pkg_share, "param", "frontbot.yaml")

    joy_launch_file = os.path.join(controller_pkg_share, "launch", "joy.launch.py")
    domain_bridge_launch_file = os.path.join(
        cap_sim_pkg_share,
        "launch",
        "front2rear_domain_bridge.launch.py",
    )
    rmd_launch_file = os.path.join(rmd_pkg_share, "launch", "bringup.launch.py")
    dynamixel_launch_file = os.path.join(
        dynamixel_pkg_share, "launch", "bringup.launch.py"
    )

    rearbot_node = Node(
        package="cartrider_drive_controller",
        executable="rearbot_control_node",
        name="rearbot_control_node",
        parameters=[
            frontbot_param_file,
            rearbot_param_file,
        ],
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
            "namespace": "",
            "override_param_file": rearbot_param_file,
        }.items(),
    )

    dynamixel_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dynamixel_launch_file),
        launch_arguments={
            "override_param_file": rearbot_param_file,
        }.items(),
    )

    joy_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch_file),
    )

    domain_bridge_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(domain_bridge_launch_file),
    )

    return LaunchDescription(
        [
            domain_bridge_bringup,
            joy_bringup,
            rmd_bringup,
            dynamixel_bringup,
            rearbot_node,
            # odom_node,
        ]
    )
