# Mobile Manipulator Simulation Project

An autonomous mobile robot built on ROS2, combining navigation with semantic perception:
a TurtleBot3 Waffle doing SLAM and Nav2 in Gazebo, extended with a YOLO + depth + LiDAR
pipeline for semantic mapping and dynamic-object filtering. A WidowX 250 arm and
LLM-driven task planning are planned for later phases.

## Tech Stack
* ROS2 Jazzy - robot middleware
* Gazebo Harmonic (gz-sim 8.10) - physical simulation
* slam_toolbox - SLAM
* Nav2 + AMCL - navigation and localization
* RViz2 - visualization

## Packages

### `tb3_semantic_nav/` — TurtleBot3 navigation + semantic perception *(active)*
Primary development package. TurtleBot3 Waffle SLAM and Nav2, built as the base for the
perception pipeline.

**Working:**
- Single-bridge Gazebo Harmonic bringup (forked from `turtlebot3_gazebo`)
- SLAM with `slam_toolbox`; map of `turtlebot3_world` saved
- Nav2 + AMCL, verified 3/3 autonomous goals within 2 cm of target

**In progress:** forking a `waffle_rgbd` model to add a depth camera (the stock waffle is RGB-only).

See `tb3_semantic_nav/README.md` for launch arguments and design notes.

## Branches
* `main` — the TurtleBot3 line (this work).
* `feat/turtlebot4` — the earlier TurtleBot 4 (Create3 + RPLIDAR + OAK-D) attempt, preserved
  with full history including the Cyclone DDS stability work. It reached working SLAM + Nav2
  but localization was noisy; the root causes are documented in `tb3_semantic_nav/README.md`
  so they are not repeated.

## Progress
### Completed
* TurtleBot3 Waffle in Gazebo Harmonic, single-bridge launch stack
* SLAM with slam_toolbox; map of `turtlebot3_world` saved
* Nav2 + AMCL localization, verified 3/3 autonomous goals
### Current
* Adding an RGBD camera via a forked `waffle_rgbd` model
### Future
* YOLO object detection node publishing `vision_msgs/Detection2DArray`
* Project detections to 3D by fusing depth with LiDAR range; publish a semantic marker map
* Mask dynamic objects (people) out of the scan so they never enter the static map
* LLM task planner (natural language -> Nav2 goals)
* Add the WidowX 250 arm to the robot description
* MoveIt2 arm control for pick-and-place driven by detections
* End-to-end demo: "find and pick up the red cup"

## Installation
### System Prerequisites
* Ubuntu 24.04 LTS
* ROS2 Jazzy: `sudo apt install ros-jazzy-desktop`
* Gazebo Harmonic: `sudo apt-get install gz-harmonic`
* TurtleBot3 packages:
```bash
sudo apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-navigation2 \
  ros-jazzy-turtlebot3-teleop ros-jazzy-turtlebot3-description
```
* Cyclone DDS, recommended for Nav2 stability under WSL2 (the default Fast DDS drops
  `/scan` and TF when multicast misbehaves):
```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Setup
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/samuelliu1202/mobile-manipulator.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src
colcon build --packages-select tb3_semantic_nav --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
```

## Quick Start
Map an environment: `ros2 launch tb3_semantic_nav slam_bringup.launch.py`\
Then drive it: `ros2 run turtlebot3_teleop teleop_keyboard`\
Save the map: `ros2 run nav2_map_server map_saver_cli -f src/tb3_semantic_nav/maps/<name>`\
Navigate a saved map: `ros2 launch tb3_semantic_nav nav2_bringup.launch.py map:=<abs path>.yaml`

Pass `gui:=false` to run Gazebo headless. Worth doing: the stock waffle's 1920x1080 camera
holds the simulation to ~0.3x real time, so headless is noticeably faster until the
`waffle_rgbd` model drops the resolution.

## Resources
* [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
* [TurtleBot4 Docs](https://turtlebot.github.io/turtlebot4-user-manual/) (for `feat/turtlebot4`)
* [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
* [Nav2](https://docs.nav2.org/)
