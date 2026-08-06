#!/usr/bin/env python3
"""slam_toolbox in async mapping mode.

Async (not sync) on purpose: it processes scans on a background thread so Nav2's
20 Hz controller and 5 Hz costmap threads are never starved behind a scan match.

`scan_topic` is a launch argument rather than a hardcoded value because Phase 6
repoints SLAM at /scan_filtered (people masked out) while the Nav2 local costmap
keeps consuming raw /scan.

IMPORTANT: slam_toolbox is a LIFECYCLE node. Spawning it as a plain Node leaves it
`unconfigured` forever -- it subscribes to nothing, declares no parameters (they are
declared in on_configure), and never publishes /map. It also logs no error, so the
symptom is just silence. The configure/activate events below are mandatory; this
mirrors slam_toolbox's own online_async_launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_semantic_nav')
    default_params = os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': scan_topic,
                'use_lifecycle_manager': use_lifecycle_manager,
            },
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] slam_toolbox is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Phase 6 switches this to /scan_filtered.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Drive the lifecycle transitions here. Ignored if a lifecycle '
                        'manager owns the node.'),
        DeclareLaunchArgument('use_lifecycle_manager', default_value='false'),
        slam_node,
        configure_event,
        activate_event,
    ])
