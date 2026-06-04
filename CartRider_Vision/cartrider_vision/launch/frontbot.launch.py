from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    front_rs_x = LaunchConfiguration("front_rs_x")
    front_rs_y = LaunchConfiguration("front_rs_y")
    front_rs_z = LaunchConfiguration("front_rs_z")
    front_rs_roll = LaunchConfiguration("front_rs_roll")
    front_rs_pitch = LaunchConfiguration("front_rs_pitch")
    front_rs_yaw = LaunchConfiguration("front_rs_yaw")

    rear_rs_x = LaunchConfiguration("rear_rs_x")
    rear_rs_y = LaunchConfiguration("rear_rs_y")
    rear_rs_z = LaunchConfiguration("rear_rs_z")
    rear_rs_roll = LaunchConfiguration("rear_rs_roll")
    rear_rs_pitch = LaunchConfiguration("rear_rs_pitch")
    rear_rs_yaw = LaunchConfiguration("rear_rs_yaw")

    base_frame = LaunchConfiguration("base_frame")
    front_rs_frame = LaunchConfiguration("front_rs_frame")
    rear_rs_frame = LaunchConfiguration("rear_rs_frame")

    front_realsense = GroupAction(
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
                    "camera_name": "front_camera",
                    "camera_namespace": "front",
                    "serial_no": "'247122073878'",
                    "enable_color": "false",
                    "enable_depth": "true",
                    "align_depth.enable": "false",
                    "pointcloud.enable": "true",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    "rgb_camera.color_profile": "424x240x15",
                    "depth_module.depth_profile": "480x270x15",
                }.items(),
            ),
        ],
        forwarding=False,
    )

    rear_realsense = GroupAction(
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
                    "camera_name": "rear_camera",
                    "camera_namespace": "front",
                    "serial_no": "'332322071504'",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "align_depth.enable": "true",
                    "pointcloud.enable": "false",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    "rgb_camera.color_profile": "424x240x15",
                    "depth_module.depth_profile": "480x270x15",
                }.items(),
            ),
        ],
        forwarding=False,
    )

    base_to_front_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace="front",
        name="base_to_front_rs_tf",
        arguments=[
            front_rs_x,
            front_rs_y,
            front_rs_z,
            front_rs_roll,
            front_rs_pitch,
            front_rs_yaw,
            base_frame,
            front_rs_frame,
        ],
        output="screen",
    )

    base_to_rear_rs_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace="front",
        name="base_to_rear_rs_tf",
        arguments=[
            rear_rs_x,
            rear_rs_y,
            rear_rs_z,
            rear_rs_roll,
            rear_rs_pitch,
            rear_rs_yaw,
            base_frame,
            rear_rs_frame,
        ],
        output="screen",
    )

    front_rs_pcd_node = Node(
        package="cartrider_vision",
        executable="rs_pcd_node",
        namespace="front",
        name="front_rs_pcd_node",
        output="screen",
        parameters=[
            {
                "input_cloud_topic": "front_camera/depth/points",
                "output_cloud_topic": "front_camera/depth/voxel_cloud",
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

    rear_rs_aruco_node = Node(
        package="cartrider_vision",
        executable="rs_aruco_node",
        namespace="front",
        name="rear_rs_aruco_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": "rear_camera/color/image_raw",
                "depth_topic": "rear_camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "rear_camera/color/camera_info",
                "base_frame": base_frame,
                "target_pose_topic": "target_pose",
                "target_type_topic": "target_type",
                "marker_topic": "marker",
                "show_window": ParameterValue(
                    LaunchConfiguration("show_window"),
                    value_type=bool,
                ),
                "window_name": "FrontBot Rear RS ArUco Docking Pose",
                "cart_marker_length_m": 0.20,
                "robot_marker_length_m": 0.10,
                "depth_min_m": 0.15,
                "depth_max_m": 5.0,
                "depth_margin_px": 4,
                "robot_marker_id": 4,
                "id1_yaw_deg": 0.0,
                "id3_yaw_deg": 90.0,
                "id0_yaw_deg": 180.0,
                "id2_yaw_deg": 270.0,
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
            DeclareLaunchArgument("front_rs_x", default_value="0.197550"),
            DeclareLaunchArgument("front_rs_y", default_value="0.000000"),
            DeclareLaunchArgument("front_rs_z", default_value="0.367290"),
            DeclareLaunchArgument("front_rs_roll", default_value="0.0"),
            DeclareLaunchArgument("front_rs_pitch", default_value="0.0"),
            DeclareLaunchArgument("front_rs_yaw", default_value="0.0"),
            DeclareLaunchArgument("rear_rs_x", default_value="0.105628"),
            DeclareLaunchArgument("rear_rs_y", default_value="0.000000"),
            DeclareLaunchArgument("rear_rs_z", default_value="0.432740"),
            DeclareLaunchArgument("rear_rs_roll", default_value="0.0"),
            DeclareLaunchArgument("rear_rs_pitch", default_value="-0.174533"),
            DeclareLaunchArgument("rear_rs_yaw", default_value="0.0"),
            DeclareLaunchArgument("base_frame", default_value="front/base_link"),
            DeclareLaunchArgument("front_rs_frame", default_value="front_camera_link"),
            DeclareLaunchArgument("rear_rs_frame", default_value="rear_camera_link"),
            DeclareLaunchArgument("show_window", default_value="true"),
            DeclareLaunchArgument("yaw_snap_enable", default_value="false"),
            DeclareLaunchArgument("yaw_snap_zero_min_deg", default_value="-5.0"),
            DeclareLaunchArgument("yaw_snap_zero_max_deg", default_value="5.0"),
            DeclareLaunchArgument("yaw_snap_180_min_deg", default_value="175.0"),
            DeclareLaunchArgument("yaw_snap_180_max_deg", default_value="180.0"),
            front_realsense,
            rear_realsense,
            base_to_front_rs_tf,
            base_to_rear_rs_tf,
            front_rs_pcd_node,
            rear_rs_aruco_node,
        ]
    )
