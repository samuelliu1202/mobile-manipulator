# tb3_semantic_nav

TurtleBot3 SLAM + Nav2 in Gazebo Harmonic, built to carry a YOLO / depth / LiDAR
semantic perception pipeline on top.

**Stack:** ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8.10) · slam_toolbox · Nav2

## Status

| Phase | What | State |
|---|---|---|
| 0 | Install TB3 packages, verify sensors render | done |
| 1 | Stock SLAM + Nav2 baseline | done |
| 2 | This package: forked sim bringup + wrapper launches | done |
| 3 | `waffle_rgbd` model with a depth camera | not started |
| 4 | YOLO detector node → `vision_msgs/Detection2DArray` | not started |
| 5 | Semantic projector (RGBD + LiDAR fusion) → `MarkerArray` | not started |
| 6 | Dynamic-object filtering → `/scan_filtered` | not started |
| 7 | Arm attachment hook | not started |

## Quick start

```bash
export TURTLEBOT3_MODEL=waffle          # already in ~/.bashrc
colcon build --packages-select tb3_semantic_nav --symlink-install
source install/setup.bash
```

**Map an environment:**
```bash
ros2 launch tb3_semantic_nav slam_bringup.launch.py
ros2 run turtlebot3_teleop teleop_keyboard          # drive it around
ros2 run nav2_map_server map_saver_cli -f src/tb3_semantic_nav/maps/<name>
```

**Navigate a saved map:**
```bash
ros2 launch tb3_semantic_nav nav2_bringup.launch.py map:=<abs path>/<name>.yaml
# RViz: "2D Pose Estimate" to seed AMCL, then "Nav2 Goal"
```

**Useful arguments** (all launches): `world:=<abs path>`, `robot_model:=waffle|waffle_rgbd`,
`gui:=false` (headless), `rviz:=false`, `scan_topic:=/scan_filtered` (Phase 6).

## Layout

```
launch/
  sim.launch.py            gz sim + spawn + THE single bridge + robot_state_publisher
  slam.launch.py           slam_toolbox async mapping
  nav2.launch.py           nav2_bringup against a saved map
  slam_bringup.launch.py   sim + slam + rviz
  nav2_bringup.launch.py   sim + nav2 + rviz
config/
  slam_toolbox_mapping.yaml
  nav2_params.yaml         seeded from turtlebot3_navigation2/param/waffle.yaml
  waffle_bridge.yaml       gz <-> ROS topic bridge
models/turtlebot3_waffle/  forked from turtlebot3_gazebo (Phase 3 adds waffle_rgbd)
urdf/                      forked TB3 URDF (already has camera_depth/optical frames)
```

## Design notes worth keeping

**One bridge, always.** `sim.launch.py` starts exactly one `ros_gz_bridge parameter_bridge`.
The previous TurtleBot4 attempt included the upstream bridge launch twice, producing duplicate
`/scan` publishers and duplicate `odom→base_link` TF — the main cause of its localization jitter.
Verify with `ros2 topic info /scan --verbose`: publisher count must be 1.

**Why the sim launch is forked rather than reused.** `turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py`
derives both the model directory and the bridge YAML from `$TURTLEBOT3_MODEL` inside its own
read-only share directory, so a custom model like `waffle_rgbd` cannot be pointed at without forking.
`robot_model` here selects model SDF, URDF, and bridge YAML together so they cannot drift apart.

**Sensor ranges must match the hardware.** The waffle's HLS-LFCD LDS tops out at 3.5 m, so
`max_laser_range: 3.5` in the SLAM config and `obstacle_max_range: 2.5` / `raytrace_max_range: 3.0`
in the costmaps. The old TB4 config used `max_laser_range: 20.0` against a 12 m RPLIDAR.

**Known: the stock camera is a performance trap.** The upstream waffle SDF renders at
1920×1080 @ 30 Hz, which drops Gazebo to **RTF ≈ 0.27** on this machine (measured: camera-less
burger runs at RTF 0.93). Phase 3 drops it to 640×480 @ 15 Hz, which is also all CPU YOLO can
consume. Until then, expect mapping runs to take ~4× wall-clock.

**Local vs global costmap scans (Phase 6).** When `/scan_filtered` lands, SLAM / AMCL / the global
costmap consume the filtered scan so people never enter the static map, while the **local** costmap
keeps raw `/scan` so the robot still avoids the actual person.
