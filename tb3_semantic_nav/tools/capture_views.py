#!/usr/bin/env python3
"""Teleport the robot to fixed viewpoints and save a camera frame at each.

    tools/capture_views.py <out_dir> [standoffs_csv]
    tools/capture_views.py frames/ 1.5,2.5,3.5

Builds a deterministic, re-shootable benchmark image set for the detector. Far more
useful than driving: the same viewpoints can be recaptured after any camera, world, or
lighting change so results compare like with like.

Requires a running simulation with `camera:=true` on the forked house world, since the
viewpoints below reference objects that only that world contains.

NOTE: teleporting breaks odometry continuity, so never do this during a SLAM mapping
run -- it will corrupt the map. This is for still capture only.
"""
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

ROBOT = 'turtlebot3'

# (label, object x, object y, bearing from object toward the robot).
# The waffle's camera sits ~0.19 m up with a 59 deg FOV, so closer than ~1.5 m
# overflows the frame and tables are seen from underneath.
OBJECTS = [
    ('person_standing', 1.00, -4.00, -math.pi / 2),
    ('person_casual',   6.20, -1.50, math.pi),
    ('person_sitting', -2.00,  3.00, 0.0),
    ('chair_1',         0.00,  3.00, 0.0),
    ('chair_2',         5.00, -3.00, math.pi),
    ('coffee_table',   -2.60, -2.00, 0.0),
    ('trash_can',       1.88,  1.91, math.pi),
    ('table_marble',    4.88,  2.93, math.pi),
]


def teleport(x, y, yaw):
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    r = subprocess.run(
        ['gz', 'service', '-s', '/world/default/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req',
         f'name: "{ROBOT}", position: {{x: {x}, y: {y}, z: 0.05}}, '
         f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'],
        capture_output=True, text=True)
    return 'true' in (r.stdout or '').lower()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    outdir = sys.argv[1]
    standoffs = [float(v) for v in (sys.argv[2].split(',') if len(sys.argv) > 2
                                    else ['1.5', '2.5', '3.5'])]
    os.makedirs(outdir, exist_ok=True)

    rclpy.init()
    node = Node('capture_views')
    state = {'msg': None}
    node.create_subscription(Image, '/camera/image_raw',
                             lambda m: state.update(msg=m), SENSOR_QOS)

    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline and state['msg'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state['msg'] is None:
        print('NO IMAGES on /camera/image_raw -- is camera:=true set?')
        return 1

    n = 0
    for label, ox, oy, bearing in OBJECTS:
        for dist in standoffs:
            rx, ry = ox + dist * math.cos(bearing), oy + dist * math.sin(bearing)
            yaw = math.atan2(oy - ry, ox - rx)
            ok = teleport(rx, ry, yaw)

            settle = time.monotonic() + 2.0
            while time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=0.05)

            m = state['msg']
            arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            rgb = arr if m.encoding == 'rgb8' else arr[:, :, ::-1]
            name = f'{label}_d{dist:g}.png'
            PILImage.fromarray(rgb).save(os.path.join(outdir, name))
            print(f'  {name:28s} teleport={"ok" if ok else "FAIL":4s} '
                  f'robot=({rx:5.2f},{ry:5.2f}) luma={rgb.mean():5.1f}')
            n += 1

    print(f'saved {n} frames to {outdir}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
