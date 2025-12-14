import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tb4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    proj_dir = get_package_share_directory('turtlebot_widowx')

    world_file = os.path.join(proj_dir, 'worlds', 'test.sdf')

    gazebo_launch = ExecuteProcess(cmd=['gz','sim','-r',world_file], output='screen')

    tb4_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb4_gz_bringup, 'launch', 'turtlebot4_spawn.launch.py')
            ),
            launch_arguments={
                'x':'0.0',
                'y':'0.0',
                'z':'0.0',
                'yaw':'0.0',
                'model':'standard',
            }.items()
        )
    
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}]
    )
    

    return LaunchDescription([
        gazebo_launch,
        tb4_launch,
        clock_bridge
    ])