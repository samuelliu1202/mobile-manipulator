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
| 2.5 | Simulation performance: RTF 0.27 -> 0.99 | done |
| 3 | `waffle_rgbd` model with a depth camera | done |
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

**Run with the depth camera** (what the perception phases use):
```bash
ros2 launch tb3_semantic_nav nav2_bringup.launch.py robot_model:=waffle_rgbd camera:=true
# -> /camera/image_raw  /camera/depth/image_raw (32FC1 metres)  /camera/camera_info
```

**Useful arguments** (all launches): `world:=<abs path>`, `robot_model:=waffle|waffle_rgbd`,
`camera:=true|false` (default false), `gui:=false` (headless), `rviz:=false`,
`scan_topic:=/scan_filtered` (Phase 6).

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

**Simulation speed: RTF 0.27 → 0.99.** The upstream waffle rendered 1920×1080 @ 30 Hz, which
pinned Gazebo at RTF 0.27. Now 640×480 @ 10 Hz, physics 250 Hz instead of 1 kHz, shadows and
sensor visualization off, and — the biggest single lever — **camera streams bridged only when
`camera:=true`**, because Gazebo renders a camera only while one of its topics has a
subscriber. Gating it took SLAM/Nav2 runs from 0.59 to 0.96.

The bottleneck was never CPU: the gz server used 143–201% of a possible 2000%. Adding cores
will not help.

**Do not "fix" the software renderer.** `glxinfo -B` reports `llvmpipe` / `Accelerated: no` —
Gazebo renders on the CPU, because WSL has no `/dev/dri` node so Mesa cannot autodetect the
`d3d12` driver. That looks like a bug and is tempting to correct. It is not: hardware
acceleration measured **2× slower** on both of this machine's Intel GPUs.

| Renderer (`waffle_rgbd camera:=true gui:=false`) | RTF | gz CPU |
|---|---|---|
| llvmpipe (default, software) | **0.99** | 163% |
| `GALLIUM_DRIVER=d3d12` → Arc A370M | 0.52 | 90% |
| `GALLIUM_DRIVER=d3d12` → Iris Xe | 0.43 | 84% |

WSL's d3d12 layer has a large per-frame round-trip cost, and a 640×480 @ 10 Hz render is far
too small to amortize it. The GPU saves CPU we do not need and costs RTF we do.

This says nothing about **YOLO** — inference goes through Level Zero / OpenCL, a different
driver path from OpenGL. The Arc may still be worth using there.

**Keep the heavy camera topics out of `waffle_rgbd_bridge.yaml`.** That bridge always runs, so
a subscription there would force the camera to render on every launch and silently undo the
`camera:=false` win. Images and depth belong in `waffle_rgbd_camera_bridge.yaml`, which is
gated. The always-on bridge carries only `camera_info`.

**Measure with `check_health`, not Gazebo's own number.** gz's self-reported
`real_time_factor` reads 0.82–0.86 where true throughput is 0.54, because it does not count
stalls. `check_health` measures sim-time advance per wall second, which is also immune to the
physics step size changing.

**`slam_toolbox` is a lifecycle node.** Spawning it as a plain `Node` leaves it `unconfigured`
forever — no `/scan` subscription, no parameters (declared in `on_configure`), no `/map`, and
**no error logged**. `slam.launch.py` uses `LifecycleNode` with explicit configure/activate
events. Same care applies to anything else lifecycle-based.

**Local vs global costmap scans (Phase 6).** When `/scan_filtered` lands, SLAM / AMCL / the global
costmap consume the filtered scan so people never enter the static map, while the **local** costmap
keeps raw `/scan` so the robot still avoids the actual person.
