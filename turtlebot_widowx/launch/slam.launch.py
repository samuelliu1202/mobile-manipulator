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

    '''
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            slam_params,
        ],
    )
    '''
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_sync_launch.py'
            ])
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'True'
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    '''delayed_rviz = TimerAction(
        period = .0,
        actions=[rviz]
    )'''


    return LaunchDescription([
        slam,
        rviz
    ])