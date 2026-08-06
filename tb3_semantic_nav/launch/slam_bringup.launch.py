#!/usr/bin/env python3
"""One-shot mapping session: simulation + slam_toolbox + RViz.

Drive with:  ros2 run turtlebot3_teleop teleop_keyboard
Save with:   ros2 run nav2_map_server map_saver_cli -f <pkg>/maps/<name>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_semantic_nav')
    launch_dir = os.path.join(pkg_share, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    robot_model = LaunchConfiguration('robot_model')
    camera = LaunchConfiguration('camera')
    scan_topic = LaunchConfiguration('scan_topic')
    use_rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('robot_model', default_value='waffle'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Bridge camera streams. Costs render time; leave false unless perception needs it.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'world': world,
                              'robot_model': robot_model, 'gui': gui,
                              'camera': camera}.items(),
        ),

        # Gazebo must be up and publishing /clock before slam_toolbox latches onto
        # sim time, otherwise it starts on wall time and every scan is timestamped wrong.
        TimerAction(period=8.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'scan_topic': scan_topic}.items(),
            ),
        ]),

        TimerAction(period=10.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]),
    ])
