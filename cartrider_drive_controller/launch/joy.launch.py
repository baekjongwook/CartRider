from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    teleop_node = Node(
        package='cartrider_control',
        executable='teleop_joystick', 
        name='teleop_joystick',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])