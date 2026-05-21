from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            'enable_depth': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'depth_module.profile': '640x480x15',
            'rgb_camera.profile': '640x480x15',
            'align_depth.enable': 'true',
        }.items()
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '0', '0', '0',   # x y z
            '0', '0', '0',   # roll pitch yaw
            'base_link',
            'camera_link'
        ],
        output='screen'
    )

    yolo_monitor = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            (
                'source /opt/ros/humble/setup.bash && '
                'source ~/ros2_ws/install/setup.bash && '
                'source ~/yolo_env/bin/activate && '
                'python3 ~/ros2_ws/src/vision/vision/depth_roi_monitor.py'
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg='🚀 Launching RealSense + YOLO monitor'),
        realsense_launch,
        static_tf_camera,
        yolo_monitor,
    ])