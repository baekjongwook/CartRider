from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("cartrider_drive_controller")
    param_file = os.path.join(pkg_share, "param", "joy.yaml")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[param_file],
    )

    teleop_node = Node(
        package="cartrider_drive_controller",
        executable="teleop_joystick",
        name="teleop_joystick",
        output="screen",
        parameters=[param_file],
    )

    return LaunchDescription([joy_node, teleop_node])
