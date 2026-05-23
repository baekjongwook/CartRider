from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    rs_x = LaunchConfiguration("rs_x")
    rs_y = LaunchConfiguration("rs_y")
    rs_z = LaunchConfiguration("rs_z")
    rs_roll = LaunchConfiguration("rs_roll")
    rs_pitch = LaunchConfiguration("rs_pitch")
    rs_yaw = LaunchConfiguration("rs_yaw")

    base_frame = LaunchConfiguration("base_frame")
    rs_frame = LaunchConfiguration("rs_frame")
    rs_serial_no = LaunchConfiguration("rs_serial_no")

    depth_profile = LaunchConfiguration("depth_profile")
    rgb_profile = LaunchConfiguration("rgb_profile")

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_name": "rs",
            "camera_namespace": "rear",
            "serial_no": rs_serial_no,

            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "enable_sync": "true",

            "pointcloud.enable": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",

            "depth_module.profile": depth_profile,
            "rgb_camera.profile": rgb_profile,

            "publish_tf": "true",
            "base_frame_id": rs_frame,
            "depth_frame_id": "rear_rs_depth_frame",
            "color_frame_id": "rear_rs_color_frame",
            "depth_optical_frame_id": "rear_rs_depth_optical_frame",
            "color_optical_frame_id": "rear_rs_color_optical_frame",
            "aligned_depth_to_color_frame_id": "rear_rs_aligned_depth_to_color_frame",
        }.items(),
    )

    base_to_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace="rear",
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
        namespace="rear",
        name="rs_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": "/rear/rs/color/image_raw",
                "depth_topic": "/rear/rs/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/rear/rs/color/camera_info",

                "base_frame": base_frame,
                "cart_pose_topic": "/rear/rs/cart_pose",
                "marker_topic": "/rear/rs/cart_marker",

                "show_window": ParameterValue(
                    LaunchConfiguration("show_window"),
                    value_type=bool,
                ),
                "window_name": "Rear RS ArUco Cart Pose",
                "marker_length_m": 0.20,

                "id1_yaw_deg": 180.0,
                "id3_yaw_deg": 270.0,
                "id0_yaw_deg": 0.0,
                "id2_yaw_deg": 90.0,

                "id1_offset_x": 0.0,
                "id1_offset_y": 0.0,
                "id3_offset_x": 0.0,
                "id3_offset_y": 0.0,
                "id0_offset_x": 0.0,
                "id0_offset_y": 0.0,
                "id2_offset_x": 0.0,
                "id2_offset_y": 0.0,

                "yaw_snap_enable": ParameterValue(
                    LaunchConfiguration("yaw_snap_enable"),
                    value_type=bool,
                ),
                "yaw_snap_zero_min_deg": ParameterValue(
                    LaunchConfiguration("yaw_snap_zero_min_deg"),
                    value_type=float,
                ),
                "yaw_snap_zero_max_deg": ParameterValue(
                    LaunchConfiguration("yaw_snap_zero_max_deg"),
                    value_type=float,
                ),
                "yaw_snap_180_min_deg": ParameterValue(
                    LaunchConfiguration("yaw_snap_180_min_deg"),
                    value_type=float,
                ),
                "yaw_snap_180_max_deg": ParameterValue(
                    LaunchConfiguration("yaw_snap_180_max_deg"),
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rs_serial_no", default_value=""),

            DeclareLaunchArgument("rs_x", default_value="0.224400"),
            DeclareLaunchArgument("rs_y", default_value="0.000000"),
            DeclareLaunchArgument("rs_z", default_value="0.614729"),
            DeclareLaunchArgument("rs_roll", default_value="0.0"),
            DeclareLaunchArgument("rs_pitch", default_value="0.0"),
            DeclareLaunchArgument("rs_yaw", default_value="0.0"),

            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("rs_frame", default_value="rear_rs_link"),

            DeclareLaunchArgument("depth_profile", default_value="640x480x15"),
            DeclareLaunchArgument("rgb_profile", default_value="640x480x15"),

            DeclareLaunchArgument("show_window", default_value="true"),

            DeclareLaunchArgument("yaw_snap_enable", default_value="false"),
            DeclareLaunchArgument("yaw_snap_zero_min_deg", default_value="-5.0"),
            DeclareLaunchArgument("yaw_snap_zero_max_deg", default_value="5.0"),
            DeclareLaunchArgument("yaw_snap_180_min_deg", default_value="175.0"),
            DeclareLaunchArgument("yaw_snap_180_max_deg", default_value="180.0"),

            realsense,
            base_to_rs_tf,
            rs_node,
        ]
    )
