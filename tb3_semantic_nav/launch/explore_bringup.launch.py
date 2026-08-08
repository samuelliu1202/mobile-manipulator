#!/usr/bin/env python3
"""Autonomous exploration: simulation + slam_toolbox + Nav2 (no AMCL, no map_server).

This is the combination that can actually map a multi-room house. Reactive driving
(`explore_drive`) loops in whichever room it starts in, and a hand-rolled waypoint
driver (`tools/map_tour.py`) drives straight at its goal and oscillates against the
wall beside a doorway. Nav2's planner routes *around* walls, so pairing it with
frontier goals is what gets the robot through doors.

Drive it with:
    python3 tools/frontier_explore.py

Key difference from nav2_bringup.launch.py: it includes nav2_bringup's
`navigation_launch.py` rather than `bringup_launch.py`, i.e. the nav stack WITHOUT
localization. slam_toolbox supplies both `/map` and the `map -> odom` transform, so an
AMCL and a map_server would be redundant and would fight it for the `map` frame.

Camera defaults to false: mapping does not need it and rendering costs ~7-25% RTF.
Pass camera:=true only if you want detections during the run.
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
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml')
    default_world = os.path.join(pkg_share, 'worlds', 'turtlebot3_house.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    robot_model = LaunchConfiguration('robot_model')
    camera = LaunchConfiguration('camera')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('robot_model', default_value='waffle_rgbd'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='mapping does not need it; costs render time'),
        # Deliberately NOT named 'params_file'. slam.launch.py declares an argument of
        # that name for the SLAM config, and a launch configuration already set by the
        # PARENT wins over a child's DeclareLaunchArgument default. Sharing the name fed
        # nav2_params.yaml to slam_toolbox, which then silently fell back to its own
        # defaults -- 25 m max laser range on a 3.5 m lidar -- and mapped badly.
        DeclareLaunchArgument('nav2_params_file', default_value=default_params),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'world': world,
                              'robot_model': robot_model, 'gui': gui,
                              'camera': camera}.items(),
        ),

        # slam_toolbox must own the map frame before Nav2's costmaps come up, or the
        # global costmap activates with no map -> odom and the lifecycle manager stalls
        # without retrying (the deadlock documented for AMCL applies equally here).
        TimerAction(period=8.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'params_file': slam_params_file}.items(),
            ),
        ]),

        TimerAction(period=14.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, 'navigation_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'params_file': nav2_params_file,
                                  'autostart': 'true'}.items(),
            ),
        ]),

        TimerAction(period=18.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]),
    ])
