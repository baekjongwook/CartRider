from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('cartrider_vesc_sdk'),
        'param',
        'motors.yaml'
    )

    return LaunchDescription([

        Node(
            package='cartrider_vesc_sdk',
            executable='hardware_node',
            parameters=[config],
            output='screen'
        )
    ])