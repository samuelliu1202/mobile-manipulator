#!/usr/bin/env python3
"""Autonomous navigation session: simulation + Nav2 (AMCL) + RViz.

Requires a saved map. Build one first with slam_bringup.launch.py.
Set the initial pose in RViz with "2D Pose Estimate", then send goals with "Nav2 Goal".
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
    default_map = os.path.join(pkg_share, 'maps', 'turtlebot3_world.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    robot_model = LaunchConfiguration('robot_model')
    camera = LaunchConfiguration('camera')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('robot_model', default_value='waffle'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Bridge camera streams. Costs render time; leave false unless perception needs it.'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'world': world,
                              'robot_model': robot_model, 'gui': gui,
                              'camera': camera}.items(),
        ),

        # Nav2's lifecycle manager gives up if the nodes it configures cannot yet see
        # /clock and TF, so let Gazebo settle before bringing the stack up.
        TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 'map': map_yaml}.items(),
            ),
        ]),

        TimerAction(period=12.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'nav2.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]),
    ])
