import os

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    proj_dir = get_package_share_directory('turtlebot_widowx')

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(proj_dir, 'launch', 'spawn_robot.launch.py')
        )
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'use_sim_time': True,
            'stamped':True,
        }],
        remappings=[
            ('cmd_vel','/diffdrive_controller/cmd_vel')
        ]
    )

    delayed_teleop = TimerAction(
        period = 5.0,
        actions=[teleop]
    )

    return LaunchDescription([
        spawn_robot,
        delayed_teleop
    ])