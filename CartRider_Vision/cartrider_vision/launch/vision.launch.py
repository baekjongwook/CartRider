from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "zed_camera.launch.py",
            )
        ),
        launch_arguments={
            "camera_model": "zed2",
            "param_overrides": (
                "general.grab_resolution:=VGA;"
                "general.grab_frame_rate:=15;"
                "general.pub_resolution:=CUSTOM;"
                "general.pub_downscale_factor:=2.0;"
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
            ),
        }.items(),
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

    vision_node = Node(
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
                "nav_goal_topic": "/vision/nav_goal_pose",
                "target_camera_topic": "/vision/zed_yolo/target_camera",
                "target_base_topic": "/vision/zed_yolo/target_base",
                "target_ground_topic": "/vision/zed_yolo/target_ground",
                "marker_topic": "/vision/zed_yolo/marker",
                "model_package": "cartrider_vision",
                "model_file": "cart_1400.pt",
                "process_hz": 2.0,
                "yolo_imgsz": 320,
                "target_class_id": 0,
                "conf_thres": 0.4,
                "confirm_time_sec": 2.0,
                "confirm_min_count": 3,
                "confirm_avg_conf_thres": 0.55,
                "confirm_xy_std_max_m": 0.35,
                "show_debug_window": True,
                "publish_once": True,
            }
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("CUDA_HOME", "/usr/local/cuda-13.0"),
            SetEnvironmentVariable("CUDACXX", "/usr/local/cuda-13.0/bin/nvcc"),
            SetEnvironmentVariable(
                "PATH",
                "/usr/local/cuda-13.0/bin:" + os.environ.get("PATH", ""),
            ),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                "/usr/local/cuda-13.0/lib64:" + os.environ.get("LD_LIBRARY_PATH", ""),
            ),
            zed_launch,
            base_to_zed_tf,
            vision_node,
        ]
    )
