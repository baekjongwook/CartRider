from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    rs_x = LaunchConfiguration("rs_x")
    rs_y = LaunchConfiguration("rs_y")
    rs_z = LaunchConfiguration("rs_z")
    rs_roll = LaunchConfiguration("rs_roll")
    rs_pitch = LaunchConfiguration("rs_pitch")
    rs_yaw = LaunchConfiguration("rs_yaw")

    base_frame = LaunchConfiguration("base_frame")
    rs_frame = LaunchConfiguration("rs_frame")

    base_to_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_rs_tf",
        arguments=[
            rs_x,
            rs_y,
            rs_z,
            rs_roll,
            rs_pitch,
            rs_yaw,
            base_frame,
            rs_frame,
        ],
    )

    rs_node = Node(
        package="cartrider_vision",
        executable="rs_node",
        name="rs_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "base_frame": "base_link",
                "cart_pose_topic": "/rs/cart_pose",
                "marker_topic": "/rs/cart_marker",
                "show_window": True,
                "window_name": "RS ArUco Cart Pose",
                "marker_length_m": 0.20,
                "id1_yaw_deg": 0.0,
                "id3_yaw_deg": 90.0,
                "id0_yaw_deg": 180.0,
                "id2_yaw_deg": 270.0,
                "id1_offset_x": 0.20,
                "id1_offset_y": 0.00,
                "id3_offset_x": 0.00,
                "id3_offset_y": -0.20,
                "id0_offset_x": -0.30,
                "id0_offset_y": 0.00,
                "id2_offset_x": 0.00,
                "id2_offset_y": 0.20,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rs_x", default_value="0.224400"),
            DeclareLaunchArgument("rs_y", default_value="0.000000"),
            DeclareLaunchArgument("rs_z", default_value="0.614729"),
            DeclareLaunchArgument("rs_roll", default_value="0.0"),
            DeclareLaunchArgument("rs_pitch", default_value="0.0"),
            DeclareLaunchArgument("rs_yaw", default_value="0.0"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("rs_frame", default_value="camera_link"),
            base_to_rs_tf,
            rs_node,
        ]
    )
