import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    proj_dir = get_package_share_directory('turtlebot_widowx')
    local_params = os.path.join(proj_dir, 'config', 'localization_params.yaml')
    map_file = os.path.join(proj_dir, 'maps', 'test4_map.yaml')

    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py'
            ])
        ),
        launch_arguments={
            'params_file': local_params,
            'use_sim_time': 'true',
            'map': map_file
        }.items()
    )


    return LaunchDescription([
        localization
    ])