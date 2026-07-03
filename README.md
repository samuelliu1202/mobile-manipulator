# Mobile Manipulator Simulation Project

An autonomous mobile manipulation system built on ROS2, combining a mobile base with a robotic arm for navigation, object detection, and pick-and-place tasks.

---

## Packages

### `turtlebot_widowx/` — TurtleBot4 + WidowX 250 exploration (ROS2 Jazzy)
Initial exploration of the TurtleBot4 simulation stack with Nav2 and planned WidowX 250 arm integration.

**What was built:**
- SLAM mapping with `slam_toolbox` in a custom Gazebo Harmonic world
- Nav2 autonomous navigation with AMCL localization
- Custom AMCL parameter tuning (`transform_tolerance`, `initial_pose`, particle filter)
- Cyclone DDS configuration for Nav2 stability in Gazebo

**Tech stack:** ROS2 Jazzy · Gazebo Harmonic · Nav2 · AMCL · Cyclone DDS

### `turtlebot3_nav/` — TurtleBot3 mission system (ROS2 Humble) *(in progress)*
Primary development package. Built on top of the TurtleBot3 simulation stack with custom nodes for perception, manipulation, and LLM-driven task planning.

**Planned:**
- YOLO-based object detection from camera feed
- LLM task planner (natural language → Nav2 goals)
- MoveIt2 arm control for pick-and-place
- End-to-end demo: "find and pick up the red cup"

**Tech stack:** ROS2 Humble · Gazebo Classic · Nav2 · YOLO · MoveIt2

---

## Installation

### turtlebot_widowx (ROS2 Jazzy + Ubuntu 24.04)
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/samuelliu1202/mobile-manipulator.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src
colcon build --packages-select turtlebot_widowx
source install/setup.bash

# Also install Cyclone DDS for Nav2 stability:
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

### turtlebot3_nav (ROS2 Humble + Ubuntu 22.04)
```bash
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations \
                 ros-humble-turtlebot3-navigation2
export TURTLEBOT3_MODEL=waffle_pi

mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
git clone https://github.com/samuelliu1202/mobile-manipulator.git
cd ~/robot_ws
colcon build --packages-select turtlebot3_nav
source install/setup.bash
```

## Resources
- [TurtleBot4 Docs](https://turtlebot.github.io/turtlebot4-user-manual/)
- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Nav2 Docs](https://docs.nav2.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
