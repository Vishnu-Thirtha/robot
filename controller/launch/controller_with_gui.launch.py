from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            executable='path_follower',
            parameters=[{
                'standalone': False
            }],
            output='screen'
        ),
        Node(
            package='controller',
            executable='controller_gui',
            output='screen'
        )
    ])
