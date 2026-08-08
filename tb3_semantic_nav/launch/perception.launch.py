#!/usr/bin/env python3
"""Perception nodes only: YOLO detector.

Deliberately separate from the simulation launches so it can also run against a
rosbag with no Gazebo at all, which is the fast way to iterate on detection:

    ros2 bag play percep_run --loop --clock
    ros2 launch tb3_semantic_nav perception.launch.py

Alongside a live sim, remember the camera is gated:

    ros2 launch tb3_semantic_nav nav2_bringup.launch.py robot_model:=waffle_rgbd camera:=true
    ros2 launch tb3_semantic_nav perception.launch.py

Must be launched from the venv that has ultralytics installed -- ament_python bakes
the interpreter into the console-script shebang at build time, so a node built outside
the venv will not find ultralytics no matter how many times you pip install it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_semantic_nav')
    default_params = os.path.join(pkg_share, 'config', 'perception.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    model = LaunchConfiguration('model')
    device = LaunchConfiguration('device')
    imgsz = LaunchConfiguration('imgsz')
    rate = LaunchConfiguration('detection_rate_hz')
    threads = LaunchConfiguration('torch_threads')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        # These defaults MUST match config/perception.yaml. Because the overrides
        # below are applied after the params file, a launch argument always wins --
        # including when it is just sitting at its default. Diverging values here
        # would silently shadow the yaml rather than defer to it.
        DeclareLaunchArgument('model', default_value='yolo11n_openvino_model',
                              description='.pt checkpoint or *_openvino_model dir'),
        DeclareLaunchArgument('device', default_value='intel:gpu.0',
                              description='cpu | intel:cpu | intel:gpu.0 iGPU | intel:gpu.1 Arc (BROKEN: returns all zeros)'),
        DeclareLaunchArgument('imgsz', default_value='640',
                              description='must match the imgsz the IR was exported at'),
        DeclareLaunchArgument('detection_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('torch_threads', default_value='6'),

        # These are read by OpenMP/oneDNN at import time, so a torch.set_num_threads()
        # call inside the node is already too late for some of the thread pools. Setting
        # them here is what actually keeps inference off Gazebo's cores.
        SetEnvironmentVariable('OMP_NUM_THREADS', threads),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', threads),
        SetEnvironmentVariable('MKL_NUM_THREADS', threads),

        Node(
            package='tb3_semantic_nav',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    # value_type is required: a LaunchConfiguration is always a string,
                    # and without an explicit type an int parameter receives "320" and
                    # is rejected (or silently ignored). Note also that params_file must
                    # keep everything under /** or these overrides lose to it -- see the
                    # comment at the top of config/perception.yaml.
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'model': ParameterValue(model, value_type=str),
                    'device': ParameterValue(device, value_type=str),
                    'imgsz': ParameterValue(imgsz, value_type=int),
                    'detection_rate_hz': ParameterValue(rate, value_type=float),
                    'torch_threads': ParameterValue(threads, value_type=int),
                },
            ],
        ),
    ])
