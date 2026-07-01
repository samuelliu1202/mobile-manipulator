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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(proj_dir, 'launch', 'nav2.launch.py')
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(proj_dir, 'launch', 'localization.launch.py')
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        spawn_robot,
        nav2,
        localization,
        rviz
    ])