import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    proj_dir = get_package_share_directory('turtlebot_widowx')
    nav2_params = os.path.join(proj_dir, 'config', 'nav2_params.yaml')

    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'cmd_vel_topic': '/diffdrive_controller/cmd_vel'
        }.items()
    )


    return LaunchDescription([
        nav2
    ])