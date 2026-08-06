#!/usr/bin/env python3
"""Automated Nav2 gate: seed AMCL, send goals, report pass/fail and final accuracy.

The scripted equivalent of clicking "2D Pose Estimate" then "Nav2 Goal" in RViz, so the
navigation stack can be verified without a human at the mouse.

Accuracy is read from the map->base_footprint TF (what AMCL believes) rather than from
/odom. Odometry is in the odom frame, which drifts and is corrected by AMCL, so comparing
an odom reading against a map-frame goal reports errors that are not real.

Usage:
    ros2 run tb3_semantic_nav nav_goal_test                       # defaults for turtlebot3_world
    ros2 run tb3_semantic_nav nav_goal_test --goals 4.0,0.0 1.5,-1.3
    ros2 run tb3_semantic_nav nav_goal_test --init 0,0 --goals 2,2 --timeout 300
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener


def _xy(text):
    x, y = text.split(',')
    return float(x), float(y)


def make_pose(nav, x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def lookup_map_pose(nav, tf_buffer):
    """Where AMCL currently believes the robot is, in the map frame."""
    try:
        tr = tf_buffer.lookup_transform(
            'map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=2.0))
        return tr.transform.translation.x, tr.transform.translation.y
    except Exception:
        return None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--init', type=_xy, default=(0.0, 0.0),
                   help='initial pose "x,y" in the map frame (default 0,0)')
    p.add_argument('--goals', type=_xy, nargs='+',
                   default=[(4.02, -0.06), (1.47, -1.26), (0.0, 0.0)],
                   help='goals as "x,y" pairs; defaults are open cells of turtlebot3_world')
    p.add_argument('--timeout', type=float, default=240.0, help='per-goal wall-clock limit, s')
    p.add_argument('--tolerance', type=float, default=0.30,
                   help='pass threshold for final distance to goal, m')
    args, ros_args = p.parse_known_args(argv)

    rclpy.init(args=ros_args)
    nav = BasicNavigator()
    tf_buffer = Buffer()
    TransformListener(tf_buffer, nav)

    # setInitialPose() takes a PoseStamped: internally it does
    # `msg.pose.pose = initial_pose.pose`, so a PoseWithCovarianceStamped crashes
    # the C type converter with an assertion failure.
    nav.setInitialPose(make_pose(nav, *args.init))

    print('waiting for Nav2 to become active...', flush=True)
    nav.waitUntilNav2Active()
    print('Nav2 active', flush=True)

    results = []
    for i, (gx, gy) in enumerate(args.goals, start=1):
        print(f'\n--- goal {i}/{len(args.goals)}: ({gx:.2f}, {gy:.2f}) ---', flush=True)
        nav.goToPose(make_pose(nav, gx, gy))
        t0 = time.time()
        last = 0.0
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.1)
            fb = nav.getFeedback()
            if fb and time.time() - last > 5.0:
                last = time.time()
                print(f'  remaining {fb.distance_remaining:.2f} m '
                      f'(wall {time.time() - t0:.0f}s)', flush=True)
            if time.time() - t0 > args.timeout:
                nav.cancelTask()
                print('  TIMEOUT -> cancelled', flush=True)
                break

        res = nav.getResult()
        for _ in range(10):
            rclpy.spin_once(nav, timeout_sec=0.1)
        pose = lookup_map_pose(nav, tf_buffer)
        if pose:
            err = math.hypot(pose[0] - gx, pose[1] - gy)
            where = f'at map=({pose[0]:+.2f}, {pose[1]:+.2f})  err={err:.3f} m'
        else:
            err, where = None, 'map pose unavailable (no map->base_footprint TF)'
        ok = res == TaskResult.SUCCEEDED and err is not None and err <= args.tolerance
        print(f'  RESULT: {res.name}  wall={time.time() - t0:.0f}s  {where}', flush=True)
        results.append((gx, gy, ok, res.name, err))

    print('\n===== SUMMARY =====', flush=True)
    for gx, gy, ok, name, err in results:
        e = f'{err:.3f} m' if err is not None else 'n/a'
        print(f'  ({gx:+.2f}, {gy:+.2f})  {"PASS" if ok else "FAIL"}  [{name}]  err={e}',
              flush=True)
    n_ok = sum(1 for r in results if r[2])
    print(f'  {n_ok}/{len(results)} goals reached within {args.tolerance} m', flush=True)

    nav.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if n_ok == len(results) else 1)


if __name__ == '__main__':
    main()
