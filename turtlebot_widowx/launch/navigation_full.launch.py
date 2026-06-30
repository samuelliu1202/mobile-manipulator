import os

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    proj_dir = get_package_share_directory('turtlebot_widowx')

    rviz_config = os.path.join(proj_dir, 'rviz', 'nav.rviz')

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(proj_dir, 'launch', 'spawn_robot.launch.py')
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(proj_dir, 'launch', 'localization.launch.py')
        )
    )

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
        # Delay localization so odom TF is in the cache before AMCL's message filter runs.
        # Without this, early scans are dropped ("timestamp earlier than all data in cache").
        TimerAction(period=5.0, actions=[localization]),
        nav2,
        rviz,
    ])
