from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
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

    realsense = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    )
                ),
                launch_arguments={
                    "camera_name": "camera",
                    "camera_namespace": "rear",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "align_depth.enable": "true",
                    "pointcloud.enable": "true",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    "rgb_camera.color_profile": "424x240x6",
                    "depth_module.depth_profile": "480x270x6",
                }.items(),
            ),
        ],
        forwarding=False,
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
        output="screen",
    )

    rs_aruco_node = Node(
        package="cartrider_vision",
        executable="rs_aruco_node",
        namespace="rear",
        name="rs_aruco_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": "camera/color/image_raw",
                "depth_topic": "camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "camera/color/camera_info",
                "base_frame": base_frame,
                "cart_pose_topic": "cart_pose",
                "marker_topic": "cart_marker",
                "show_window": ParameterValue(
                    LaunchConfiguration("show_window"),
                    value_type=bool,
                ),
                "window_name": "Rear RS ArUco Cart Pose",
                "marker_length_m": 0.20,
                "depth_min_m": 0.15,
                "depth_max_m": 5.0,
                "depth_margin_px": 4,
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

    rs_pcd_node = Node(
        package="cartrider_vision",
        executable="rs_pcd_node",
        namespace="rear",
        name="rs_pcd_node",
        output="screen",
        parameters=[
            {
                "input_cloud_topic": "camera/depth/color/points",
                "output_cloud_topic": "camera/depth/voxel_cloud",
                "target_frame": base_frame,
                "voxel_size": 0.05,
                "crop_min_x": 0.2,
                "crop_max_x": 2.5,
                "crop_min_y": -1.0,
                "crop_max_y": 1.0,
                "crop_min_z": 0.05,
                "crop_max_z": 1.5,
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
            DeclareLaunchArgument("show_window", default_value="true"),
            DeclareLaunchArgument("yaw_snap_enable", default_value="false"),
            DeclareLaunchArgument("yaw_snap_zero_min_deg", default_value="-5.0"),
            DeclareLaunchArgument("yaw_snap_zero_max_deg", default_value="5.0"),
            DeclareLaunchArgument("yaw_snap_180_min_deg", default_value="175.0"),
            DeclareLaunchArgument("yaw_snap_180_max_deg", default_value="180.0"),
            realsense,
            base_to_rs_tf,
            rs_aruco_node,
            rs_pcd_node,
        ]
    )
