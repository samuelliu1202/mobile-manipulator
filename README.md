# Mobile Manipulator Simulation Project
## Overview
This ROS2 project develops an autonomous mobile manipulation system with TurtleBot 4 mobile robot and WidowX 250 robotic arm.

## Tech Stack
* ROS2 Jazzy - robot middleware
* Gazebo Harmonic - physical simulation
* RViz2 - visualization

## Progress
### Completed
* Launch Turtlebot4 robot into Gazebo world and move with teleop keyboard
* Implement SLAM using slam_toolbox package
* Add Nav2 and Localization
### Current
* Fixing noisy localization during autonomous navigation
### Future
* Create YOLO-based object detection pipeline
* Combine camera object detection data with LiDAR data for SLAM mapping
* Modify Turtlebot URDF file to add WidowX robotic arm
* Develop autonomous manipulation for robotic arm using MoveIt2 and object detection data
* Implement multi-robot coordination

## Installation
### System Prerequisites
* Ubuntu 24.04 LTS
* ROS2 Jazzy: `sudo apt install ros-jazzy-desktop`
* Gazebo Harmonic: `sudo apt-get install gz-harmonic`

### Setup
#### Clone Repository
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/samuelliu1202/mobile-manipulator.git
```

#### Install dependencies
```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src
```

#### Build
```
cd ~/ros2_ws
colcon build --packages-select turtlebot_widowx
source install/setup.bash
```

## Quick Start
Launch SLAM with teleop control: `ros2 launch turtlebot_widowx slam_teleop.launch.py`
Launch Nav2 with SLAM map data: `ros2 launch turtlebot_widowx navigation_full.launch.py`

## Resources
* [TurtleBot 4 Docs](https://turtlebot.github.io/turtlebot4-user-manual/)
* [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
* [Nav2](https://github.com/ros-navigation/navigation2)

## Authors
Samuel Liu - syl63@cam.ac.uk
