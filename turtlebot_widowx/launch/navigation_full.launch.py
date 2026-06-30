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

    # Localization (AMCL + map_server) needs: map file, /scan, odom→base_link TF.
    # Gazebo takes ~5-10s to load and publish all of these, so delay startup.
    localization = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(proj_dir, 'launch', 'localization.launch.py')
            )
        )]
    )

    # Nav2 needs AMCL to have published map→odom TF before accepting goals.
    nav2 = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(proj_dir, 'launch', 'nav2.launch.py')
            )
        )]
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
        localization,
        nav2,
        rviz,
    ])