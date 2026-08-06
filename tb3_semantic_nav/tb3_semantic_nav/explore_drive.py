#!/usr/bin/env python3
"""Reactive explorer: builds a SLAM map without a human on the keyboard.

Drives forward until the front cone is blocked, then turns toward whichever side has
more free space. Useful for repeatable mapping runs and for headless/CI verification;
for hand-driving use `ros2 run turtlebot3_teleop teleop_keyboard` instead.

Runs on SIM time, so a low real-time factor stretches wall-clock but never changes the
commanded motion profile. (The stock waffle's 1920x1080 camera holds Gazebo to ~0.3x
real time, so a 120 s run takes ~7 min of wall-clock.)

Usage:
    ros2 run tb3_semantic_nav explore_drive
    ros2 run tb3_semantic_nav explore_drive --duration 180 --speed 0.22
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Explorer(Node):
    """Bounce-and-turn explorer driven off the front cone of the laser scan."""

    def __init__(self, duration, speed, turn_speed, stop_dist, clear_dist, cone_deg):
        super().__init__('explore_drive')
        # Follow the sim clock, otherwise `duration` becomes wall-clock and a low
        # real-time factor silently cuts the exploration short.
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])

        self.duration = duration
        self.speed = speed
        self.turn_speed = turn_speed
        self.stop_dist = stop_dist
        self.clear_dist = clear_dist
        self.cone_deg = cone_deg

        self.done = False
        self.scan = None
        self.t0 = None
        self.turn_dir = 0.0          # 0 = driving forward, else a locked turn direction

        # Gazebo bridges cmd_vel as TwistStamped -- see config/waffle_bridge.yaml.
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        # The gz bridge publishes /scan best-effort; a RELIABLE subscription gets nothing.
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'exploring for {duration}s of sim time')

    def _on_scan(self, msg):
        self.scan = msg

    def _sector_min(self, lo_deg, hi_deg):
        """Smallest valid range over an angular sector, degrees about the +x axis."""
        s = self.scan
        best = math.inf
        for i, r in enumerate(s.ranges):
            if not (math.isfinite(r) and s.range_min < r < s.range_max):
                continue
            ang = math.degrees(s.angle_min + i * s.angle_increment)
            ang = (ang + 180.0) % 360.0 - 180.0        # wrap to [-180, 180)
            if lo_deg <= ang <= hi_deg:
                best = min(best, r)
        return best

    def _tick(self):
        if self.scan is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        elapsed = now - self.t0
        if elapsed >= self.duration:
            self.publish(0.0, 0.0)
            self.get_logger().info(f'done after {elapsed:.1f}s sim time')
            self.done = True
            return

        front = self._sector_min(-self.cone_deg, self.cone_deg)

        if self.turn_dir == 0.0:
            if front < self.stop_dist:
                left = self._sector_min(20, 100)
                right = self._sector_min(-100, -20)
                # Lock a direction so we do not dither between left and right.
                self.turn_dir = 1.0 if left > right else -1.0
            else:
                self.publish(self.speed, 0.0)
                return

        if front > self.clear_dist:
            self.turn_dir = 0.0
            self.publish(self.speed, 0.0)
        else:
            self.publish(0.0, self.turn_speed * self.turn_dir)

    def publish(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = vx
        m.twist.angular.z = wz
        self.pub.publish(m)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    # TB3 waffle limits are 0.26 m/s and 1.82 rad/s; defaults stay well inside them.
    p.add_argument('--duration', type=float, default=120.0, help='seconds of SIM time')
    p.add_argument('--speed', type=float, default=0.18, help='forward speed, m/s')
    p.add_argument('--turn-speed', type=float, default=0.9, help='turn rate, rad/s')
    p.add_argument('--stop-dist', type=float, default=0.55, help='start turning below this, m')
    p.add_argument('--clear-dist', type=float, default=0.85, help='stop turning above this, m')
    p.add_argument('--cone-deg', type=float, default=25.0, help='half-width of the front cone')
    args, ros_args = p.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Explorer(args.duration, args.speed, args.turn_speed,
                    args.stop_dist, args.clear_dist, args.cone_deg)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish(0.0, 0.0)      # best-effort stop
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
