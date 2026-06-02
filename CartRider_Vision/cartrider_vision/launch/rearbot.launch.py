from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    ExecuteProcess,
)
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

    zed_param_overrides = (
        "general.grab_resolution:=VGA;"
        "general.grab_frame_rate:=10;"
        "general.pub_resolution:=CUSTOM;"
        "general.pub_downscale_factor:=3.0;"
        "general.pub_frame_rate:=10.0;"
        "general.self_calib:=false;"
        "general.publish_status:=false;"
        "video.publish_rgb:=true;"
        "video.publish_left_right:=false;"
        "video.publish_raw:=false;"
        "video.publish_gray:=false;"
        "video.publish_stereo:=false;"
        "depth.depth_mode:=PERFORMANCE;"
        "depth.depth_stabilization:=0;"
        "depth.openni_depth_mode:=false;"
        "depth.point_cloud_freq:=1.0;"
        "depth.point_cloud_res:=REDUCED;"
        "depth.publish_depth_map:=true;"
        "depth.publish_depth_info:=false;"
        "depth.publish_point_cloud:=false;"
        "depth.publish_depth_confidence:=false;"
        "depth.publish_disparity:=false;"
        "sensors.publish_imu_tf:=false;"
        "sensors.sensors_pub_rate:=10.0;"
        "sensors.publish_imu:=false;"
        "sensors.publish_imu_raw:=false;"
        "sensors.publish_cam_imu_transf:=false;"
        "sensors.publish_mag:=false;"
        "sensors.publish_baro:=false;"
        "sensors.publish_temp:=false;"
        "pos_tracking.pos_tracking_enabled:=false;"
        "pos_tracking.publish_tf:=false;"
        "pos_tracking.publish_map_tf:=false;"
        "pos_tracking.publish_odom_pose:=false;"
        "pos_tracking.publish_pose_cov:=false;"
        "pos_tracking.publish_cam_path:=false;"
        "mapping.mapping_enabled:=false;"
        "object_detection.od_enabled:=false;"
        "body_tracking.bt_enabled:=false;"
        "stream_server.stream_enabled:=false;"
        "debug.sdk_verbose:=0"
    )

    zed_camera = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "source /opt/ros/humble/setup.bash && "
                "if [ -f /home/baek/zed_ws/install/local_setup.bash ]; then "
                "source /home/baek/zed_ws/install/local_setup.bash; "
                "fi && "
                "export CUDA_HOME=/usr/local/cuda-13.0 && "
                "export CUDACXX=/usr/local/cuda-13.0/bin/nvcc && "
                "export PATH=/usr/local/cuda-13.0/bin:$PATH && "
                "export LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:$LD_LIBRARY_PATH && "
                "echo 'Starting ZED lightweight mode...' && "
                f"ros2 launch zed_wrapper zed_camera.launch.py "
                f"camera_model:=zed2 "
                f'param_overrides:="{zed_param_overrides}"'
            ),
        ],
        output="screen",
    )

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
                    "rgb_camera.color_profile": "424x240x15",
                    "depth_module.depth_profile": "480x270x15",
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
                "target_pose_topic": "target_pose",
                "target_type_topic": "target_type",
                "marker_topic": "marker",
                "show_window": ParameterValue(
                    LaunchConfiguration("show_window"),
                    value_type=bool,
                ),
                "window_name": "Rear RS ArUco Cart Pose",
                "cart_marker_length_m": 0.20,
                "robot_marker_length_m": 0.12,
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

    base_to_zed_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed_tf",
        arguments=[
            "0.202417",
            "0.001281",
            "1.121693",
            "0.0",
            "0.087156",
            "0.0",
            "0.996195",
            "base_link",
            "zed_camera_link",
        ],
        output="screen",
    )

    zed_yolo_node = Node(
        package="cartrider_vision",
        executable="zed_yolo_node",
        name="zed_yolo_node",
        output="screen",
        parameters=[
            {
                "base_frame": "base_link",
                "rgb_topic": "/zed/zed_node/rgb/color/rect/image",
                "depth_topic": "/zed/zed_node/depth/depth_registered",
                "camera_info_topic": "/zed/zed_node/rgb/color/rect/camera_info",
                "aruco_target_pose_topic": "/rear/target_pose",
                "conf_thres": 0.25,
                "process_hz": 10.0,
                "cart_target_publish_period_sec": 1.0,
                "require_aruco_detection": True,
                "aruco_detection_timeout_sec": 1.5,
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
            zed_camera,
            realsense,
            base_to_rs_tf,
            rs_aruco_node,
            base_to_zed_tf,
            zed_yolo_node,
        ]
    )
