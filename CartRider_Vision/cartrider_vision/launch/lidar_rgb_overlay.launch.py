from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():

    # Velodyne
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne'),
                'launch',
                'velodyne-all-nodes-VLP16-launch.py'
            )
        )
    )

    # RealSense (RGB only)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'rgb_camera.profile': '640x480x15',
        }.items()
    )

    # Overlay node
    overlay = Node(
        package='vision',
        executable='lidar_rgb_overlay',
        output='screen',
        parameters=[{'show_window': True}]
    )

    return LaunchDescription([
        velodyne_launch,
        realsense_launch,
        overlay
    ])