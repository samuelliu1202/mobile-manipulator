# Mobile Manipulator Simulation Project
This project simulates the autonomous navigation and manipulation of a Turtlebot 4 mobile robot with a WidowX 250 robotic arm.

## Description
### Tools Used
* ROS2 Jazzy
* Gazebo Harmonic
* RViz2

## Progress
### Completed
* Launch Turtlebot4 robot into Gazebo world and move with teleop keyboard
### Current
* Debugging SLAM pipeline in simulation
* Integrating Nav2 for autonomous navigation within map
### Future
* Combine camera object detection (ex: YOLO) data and combine with LiDAR data for SLAM mapping
* Develop autonomous manipulation for WidowX robotic arm using MoveIt2 and object detection
* Implement multi-robot coordination with other robots

## Dependencies
### ROS Packages
* SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
* Nav2: https://github.com/ros-navigation/navigation2
* MoveIt2: https://github.com/moveit/moveit2
* Turtlebot 4: https://github.com/turtlebot/turtlebot4_simulator
* WidowX 250: https://github.com/Interbotix/interbotix_ros_manipulators

## Authors
Samuel Liu - syl63@cam.ac.uk
