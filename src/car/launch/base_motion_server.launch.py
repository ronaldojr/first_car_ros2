from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car',
            executable='base_motion_server',
            name='base_motion_server',
            parameters=[{
                'speed_linear': 0.3,
                'speed_angular': 1.0,
                'duration': 1.0,   # default for straight/strafe moves
            }],
            output='screen'
        )
    ])
