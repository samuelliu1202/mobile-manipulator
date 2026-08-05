#!/usr/bin/env python3
"""slam_toolbox in async mapping mode.

Async (not sync) on purpose: it processes scans on a background thread so Nav2's
20 Hz controller and 5 Hz costmap threads are never starved behind a scan match.

`scan_topic` is a launch argument rather than a hardcoded value because Phase 6
repoints SLAM at /scan_filtered (people masked out) while the Nav2 local costmap
keeps consuming raw /scan.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_semantic_nav')
    default_params = os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Phase 6 switches this to /scan_filtered.'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time,
                                      'scan_topic': scan_topic}],
        ),
    ])
