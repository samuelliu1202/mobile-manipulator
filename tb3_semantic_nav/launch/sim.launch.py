#!/usr/bin/env python3
"""Gazebo Harmonic + TurtleBot3 bringup.

A fork of turtlebot3_gazebo's turtlebot3_world.launch.py + spawn_turtlebot3.launch.py.
Forked rather than reused because upstream derives both the model directory and the
bridge YAML from $TURTLEBOT3_MODEL inside its *own* (read-only) share directory, so
there is no way to point it at a custom model such as the Phase 3 `waffle_rgbd`.

Two invariants this file exists to protect:
  1. Exactly ONE ros_gz_bridge parameter_bridge in the whole launch tree. Including a
     bridge twice yields duplicate /scan publishers and duplicate odom->base_link TF,
     which is what wrecked localization in the earlier turtlebot_widowx setup.
  2. `robot_model` selects the model SDF, the URDF, and the bridge YAML together, so
     they can never drift out of sync.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'),
    DeclareLaunchArgument(
        'world', default_value='',
        description='Absolute path to a .world/.sdf. Empty = turtlebot3_world.world.'),
    DeclareLaunchArgument(
        'robot_model', default_value='waffle',
        description='waffle | waffle_rgbd (Phase 3). Selects model SDF, URDF and bridge YAML.'),
    DeclareLaunchArgument('x_pose', default_value='-2.0'),
    DeclareLaunchArgument('y_pose', default_value='-0.5'),
    DeclareLaunchArgument('gui', default_value='true',
                          description='Run the Gazebo GUI client. false = headless.'),
]


def _setup(context, *_args, **_kwargs):
    pkg_share = get_package_share_directory('tb3_semantic_nav')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    robot_model = LaunchConfiguration('robot_model').perform(context)
    world = LaunchConfiguration('world').perform(context)
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    gui = LaunchConfiguration('gui')

    if not world:
        world = os.path.join(tb3_gazebo_share, 'worlds', 'turtlebot3_world.world')

    model_sdf = os.path.join(pkg_share, 'models', f'turtlebot3_{robot_model}', 'model.sdf')
    urdf_path = os.path.join(pkg_share, 'urdf', f'turtlebot3_{robot_model}.urdf')
    bridge_yaml = os.path.join(pkg_share, 'config', f'{robot_model}_bridge.yaml')

    for label, path in (('model SDF', model_sdf), ('URDF', urdf_path), ('bridge YAML', bridge_yaml)):
        if not os.path.exists(path):
            raise RuntimeError(
                f"robot_model='{robot_model}': {label} not found at {path}")

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    sim_time = {'use_sim_time': use_sim_time.lower() in ('true', '1')}

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v2 {world}', 'on_exit_shutdown': 'true'}.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2'}.items(),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{**sim_time, 'robot_description': robot_desc}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'turtlebot3', '-file', model_sdf,
                   '-x', x_pose, '-y', y_pose, '-z', '0.01'],
    )

    # THE single bridge. Do not add another anywhere in this launch tree.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[sim_time],
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}'],
    )

    # Images go over image_bridge rather than parameter_bridge so they pick up
    # image_transport (compressed topics) for free.
    image_topics = ['/camera/image_raw']
    if robot_model.endswith('rgbd'):
        image_topics.append('/camera/depth/image_raw')
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        output='screen',
        parameters=[sim_time],
        arguments=image_topics,
    )

    return [gz_server, gz_client, robot_state_publisher, spawn, bridge, image_bridge]


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_semantic_nav')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    # Our models first so a fork shadows an upstream model of the same name;
    # turtlebot3_gazebo/models is required regardless because the waffle SDF pulls
    # meshes via model://turtlebot3_common/...
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join(p for p in [
            os.path.join(pkg_share, 'models'),
            os.path.join(tb3_gazebo_share, 'models'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ] if p),
    )

    return LaunchDescription([*ARGS, resource_path, OpaqueFunction(function=_setup)])
