import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    proj_dir = get_package_share_directory('turtlebot_widowx')

    world_file = os.path.join(proj_dir, 'worlds', 'test4.sdf')

    #gazebo_launch = ExecuteProcess(cmd=['gz','sim','-r',world_file], output='screen')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r -v 4 '+world_file
        }.items()
    )

    tb4_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_gz_bringup'), 
                'launch', 
                'ros_gz_bridge.launch.py'
            ])
        ),
        launch_arguments={
            'model':'standard',
        }.items()
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_gz_bringup'), 
                'launch', 
                'turtlebot4_spawn.launch.py'
            ])
        ),
        launch_arguments={
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
        tb4_robot,
        robot_spawn,
        clock_bridge
    ])