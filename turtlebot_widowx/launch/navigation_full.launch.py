import os

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# Gazebo world origin vs map origin offset:
#   map YAML origin = [-8.690, -6.657]  (world position of map pixel 0,0)
#   robot spawns at Gazebo world (0, 0)
#   → robot in map frame: (0 - (-8.690), 0 - (-6.657)) = (8.690, 6.657)
INITIAL_POSE_X = '8.690'
INITIAL_POSE_Y = '6.657'


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

    # Publish the robot's initial pose to /initialpose after AMCL has activated.
    # Using topic pub is more reliable than the set_initial_pose YAML param because
    # AMCL subscribes to /initialpose throughout its lifecycle (not just at activate).
    initial_pose_pub = TimerAction(
        period=13.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once', '/initialpose',
                'geometry_msgs/msg/PoseWithCovarianceStamped',
                (
                    '{"header": {"frame_id": "map"}, '
                    '"pose": {"pose": {'
                    '"position": {"x": ' + INITIAL_POSE_X + ', "y": ' + INITIAL_POSE_Y + ', "z": 0.0}, '
                    '"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}, '
                    '"covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, '
                    '0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}'
                )
            ],
            output='screen'
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
        initial_pose_pub,
        nav2,
        rviz,
    ])