#!/usr/bin/env python3
"""Drive a fixed waypoint tour to build a SLAM map with predictable coverage.

    python3 tools/map_tour.py [--waypoint-timeout 60] [--speed 0.20]

Why not `explore_drive`: that node is purely reactive (drive until the front cone is
blocked, then turn toward open space). In a single open room it works; in a multi-room
house it settles into a loop in whichever room it started in and never finds a doorway.
A measured run covered ~1.8% of the grid and produced arc artifacts from spinning in
place. A waypoint list gives deterministic room-to-room coverage instead.

Pose feedback comes from the `map -> base_footprint` TF, i.e. the SLAM-corrected pose,
not raw `/odom`. Odometry drift over a multi-minute run is exactly what smears a map.

Unreachable waypoints are expected -- walls are not modelled here. Each one gets a
timeout and is then skipped, which is cheaper than encoding the floor plan.
"""
import argparse
import math
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

# A snake through turtlebot3_house's open floor. Robot spawns near (-2, -0.5).
WAYPOINTS = [
    (1.0, 0.0), (4.0, 0.0), (6.0, 0.0), (6.0, -2.5), (4.0, -3.0),
    (1.0, -3.0), (-2.0, -3.0), (-5.0, -3.0), (-5.0, 0.0), (-5.0, 3.0),
    (-2.0, 3.0), (0.0, 3.0), (3.0, 3.0), (4.5, 3.0), (4.0, 6.0),
    (1.0, 6.0), (-2.0, 6.0), (-1.0, 8.0), (-2.0, 3.0), (-2.0, 0.0),
]

REACH_TOL = 0.40      # m
HEADING_TOL = 0.25    # rad
FRONT_STOP = 0.45     # m; waffle radius is 0.22


class MapTour(Node):

    def __init__(self, args):
        super().__init__('map_tour')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.speed = args.speed
        self.turn = args.turn_speed
        self.wp_timeout = args.waypoint_timeout

        self.scan = None
        self.create_subscription(LaserScan, '/scan', self._on_scan, SENSOR_QOS)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_scan(self, msg):
        self.scan = msg

    def _sector_min(self, lo_deg, hi_deg):
        """Closest return in an angular sector, ignoring inf/nan and zero-range junk."""
        s = self.scan
        if s is None:
            return math.inf
        best = math.inf
        n = len(s.ranges)
        for i in range(n):
            ang = math.degrees(s.angle_min + i * s.angle_increment)
            ang = (ang + 180.0) % 360.0 - 180.0
            if lo_deg <= ang <= hi_deg:
                r = s.ranges[i]
                if s.range_min < r < s.range_max:
                    best = min(best, r)
        return best

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        p, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw

    def drive(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(vx)
        m.twist.angular.z = float(wz)
        self.pub.publish(m)

    def wait_ready(self, timeout=60.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None and self.pose() is not None:
                return True
        return False

    def goto(self, gx, gy):
        """Turn-then-go with a reactive nudge when the front cone is blocked."""
        deadline = time.monotonic() + self.wp_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self.pose()
            if pose is None:
                continue
            x, y, yaw = pose
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist < REACH_TOL:
                self.drive(0.0, 0.0)
                return True, dist

            err = math.atan2(dy, dx) - yaw
            err = (err + math.pi) % (2 * math.pi) - math.pi
            front = self._sector_min(-25.0, 25.0)

            if front < FRONT_STOP:
                # Blocked: rotate toward whichever flank is more open. Keeps the robot
                # scanning new geometry instead of nosing into the same wall.
                left = self._sector_min(30.0, 90.0)
                right = self._sector_min(-90.0, -30.0)
                self.drive(0.0, self.turn if left > right else -self.turn)
            elif abs(err) > HEADING_TOL:
                self.drive(0.0, math.copysign(self.turn, err))
            else:
                # Ease off near the goal and steer proportionally while moving.
                self.drive(min(self.speed, 0.35 * dist + 0.05), 0.8 * err)

        self.drive(0.0, 0.0)
        pose = self.pose()
        d = math.hypot(gx - pose[0], gy - pose[1]) if pose else float('nan')
        return False, d


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.20)
    ap.add_argument('--turn-speed', type=float, default=0.9)
    ap.add_argument('--waypoint-timeout', type=float, default=60.0)
    ap.add_argument('--waypoints', default='',
                    help='override the built-in tour: "x1,y1 x2,y2 ..."')
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    waypoints = WAYPOINTS
    if args.waypoints:
        waypoints = [tuple(float(v) for v in pair.split(','))
                     for pair in args.waypoints.split()]

    rclpy.init()
    node = MapTour(args)
    if not node.wait_ready():
        node.get_logger().error('no /scan or no map->base_footprint TF; is SLAM up?')
        rclpy.shutdown()
        return 1

    reached = 0
    for i, (gx, gy) in enumerate(waypoints, 1):
        ok, d = node.goto(gx, gy)
        reached += ok
        node.get_logger().info(
            f'[{i}/{len(waypoints)}] ({gx:+.1f},{gy:+.1f}) '
            f'{"reached" if ok else "TIMEOUT"} d={d:.2f}m')

    node.drive(0.0, 0.0)
    node.get_logger().info(f'tour done: {reached}/{len(waypoints)} waypoints reached')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
