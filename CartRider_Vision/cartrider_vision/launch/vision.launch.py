from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    base_to_zed_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed_tf",
        arguments=[
            "0.202417",
            "0.001281",
            "1.121693",
            "0.0",
            "0.0",
            "0.0",
            "base_link",
            "zed_camera_link",
        ],
    )

    vision_node = Node(
        package="cartrider_vision",
        executable="vision",
        name="vision",
        output="screen",
    )

    return LaunchDescription(
        [
            base_to_zed_tf,
            vision_node,
        ]
    )
