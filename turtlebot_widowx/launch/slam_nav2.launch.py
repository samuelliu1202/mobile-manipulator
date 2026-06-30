import os

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    proj_dir = get_package_share_directory('turtlebot_widowx')
    slam_params = os.path.join(proj_dir, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(proj_dir, 'rviz', 'nav.rviz')

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(proj_dir, 'launch', 'spawn_robot.launch.py')
        )
    )

    # online_async: processes scans in a background thread so Nav2's
    # controller (20Hz) and costmap (5Hz) threads are not starved.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'True'
        }.items()
    )

    # Nav2 without localization_launch.py — SLAM Toolbox publishes the
    # map->odom TF and the live /map topic that costmaps subscribe to.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(proj_dir, 'launch', 'nav2.launch.py')
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        spawn_robot,
        TimerAction(period=5.0, actions=[slam]),   # let Gazebo finish loading
        TimerAction(period=8.0, actions=[nav2]),   # SLAM must publish /map before Nav2 starts
        rviz,
    ])
