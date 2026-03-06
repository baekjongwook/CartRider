from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('cartrider_drive_controller')
    param_file = os.path.join(pkg_share, 'param', 'rearbot.yaml')

    rearbot_node = Node(
        package='cartrider_drive_controller',
        executable='rearbot_control_node',
        name='rearbot_control_node',
        parameters=[param_file]
    )

    odom_node = Node(
        package='cartrider_drive_controller',
        executable='rearbot_odom_node',
        name='rearbot_odom_node',
        parameters=[param_file]
    )

    rmd_pkg_share = get_package_share_directory('cartrider_rmd_sdk')
    rmd_launch = os.path.join(rmd_pkg_share, 'launch', 'bringup.launch.py')

    rmd_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rmd_launch)
    )

    return LaunchDescription([
        rmd_bringup,
        rearbot_node,
        odom_node
    ])