from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your custom URDF XACRO
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot_custom'),
        'urdf',
        'turtlebot3_waffle_rgbd.urdf.xacro'
    )

    return LaunchDescription([
        # 1️⃣ Start Gazebo with ROS plugins
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # 2️⃣ Delay robot spawn slightly to let Gazebo start
        TimerAction(
            period=5.0,  # seconds
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-file', urdf_file, '-entity', 'turtlebot3'],
                    output='screen'
                )
            ]
        )
    ])
