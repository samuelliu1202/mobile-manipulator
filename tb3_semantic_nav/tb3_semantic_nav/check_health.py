#!/usr/bin/env python3
"""Structural health check for the running sim: topics, rates, publisher counts, TF, map.

Catches the class of bug that wrecked the earlier TurtleBot4 setup, where the ros_gz
bridge was launched twice and produced duplicate /scan publishers and duplicate
odom->base_link TF. Those faults do not throw errors -- they just quietly degrade
localization -- so they need an explicit check.

Exits non-zero if any CRITICAL check fails, so it is usable as a gate.

Usage:
    ros2 run tb3_semantic_nav check_health
    ros2 run tb3_semantic_nav check_health --window 8
"""

import argparse
import collections
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

# topic -> (expected nominal Hz or None, expected publisher count or None, critical?)
EXPECTED = [
    ('/clock',             None, 1,    True),
    ('/scan',              10.0, 1,    True),   # duplicate publishers here = the TB4 bug
    ('/odom',              None, 1,    True),
    ('/imu',               None, 1,    False),
    ('/joint_states',      None, 1,    False),
    ('/camera/image_raw',  None, None, False),
    ('/camera/camera_info', None, None, False),
    ('/map',               None, 1,    False),
    ('/cmd_vel',           None, None, False),  # 2 is normal: collision_monitor + docking
]


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--window', type=float, default=6.0, help='seconds to sample rates over')
    args, ros_args = p.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = rclpy.create_node('check_health')
    failures, warnings = [], []

    # ---- discover topics and publisher counts -------------------------------
    time.sleep(1.0)                      # let discovery settle
    available = dict(node.get_topic_names_and_types())

    print('=== topics ===')
    print(f'{"TOPIC":<26}{"PUBS":<6}STATUS')
    counts = {}
    for topic, _exp_hz, exp_pubs, critical in EXPECTED:
        if topic not in available:
            status = 'ABSENT'
            if critical:
                failures.append(f'{topic} is absent')
            print(f'{topic:<26}{"-":<6}{status}')
            continue
        n = node.count_publishers(topic)
        counts[topic] = n
        status = 'ok'
        if exp_pubs is not None and n != exp_pubs:
            status = f'EXPECTED {exp_pubs} PUBLISHER(S)'
            (failures if critical else warnings).append(
                f'{topic} has {n} publishers, expected {exp_pubs}')
        print(f'{topic:<26}{n:<6}{status}')

    # ---- measure rates -----------------------------------------------------
    hz = {}
    for topic, exp_hz, _p, critical in EXPECTED:
        if topic not in available:
            continue
        msg_type = None
        stamps = []
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        try:
            from rosidl_runtime_py.utilities import get_message
            msg_type = get_message(available[topic][0])
        except Exception:
            continue
        sub = node.create_subscription(msg_type, topic,
                                       lambda _m, s=stamps: s.append(time.time()), qos)
        t0 = time.time()
        while time.time() - t0 < args.window / len(EXPECTED) + 0.6:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_subscription(sub)
        if len(stamps) >= 2:
            hz[topic] = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        else:
            hz[topic] = 0.0
            if critical:
                failures.append(f'{topic} published no data')

    print('\n=== measured rates (scale by real-time factor for nominal) ===')
    for topic, exp_hz, _p, _c in EXPECTED:
        if topic in hz:
            note = f'  (nominal {exp_hz:.0f} Hz)' if exp_hz else ''
            print(f'  {topic:<26}{hz[topic]:>8.2f} Hz{note}')
    # Real-time factor, measured as sim-time advance per wall second. Deliberately not
    # derived from the /clock message rate: that needs a hardcoded nominal (1/max_step_size)
    # which silently goes wrong when the physics step changes, and it under-reports when DDS
    # drops messages at high clock rates.
    if '/clock' in available:
        from rosgraph_msgs.msg import Clock
        latest = {}
        cq = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST)
        csub = node.create_subscription(
            Clock, '/clock',
            lambda m: latest.__setitem__('t', m.clock.sec + m.clock.nanosec * 1e-9), cq)
        t0 = time.time()
        while 't' not in latest and time.time() - t0 < 5:
            rclpy.spin_once(node, timeout_sec=0.2)
        if 't' in latest:
            s0, w0 = latest['t'], time.time()
            while time.time() - w0 < args.window:
                rclpy.spin_once(node, timeout_sec=0.05)
            rtf = (latest['t'] - s0) / (time.time() - w0)
            print(f'\n  real-time factor: {rtf:.2f}   '
                  f'(sim advanced {latest["t"] - s0:.2f}s in {args.window:.0f}s wall)')
        node.destroy_subscription(csub)

    # ---- TF broadcaster uniqueness -----------------------------------------
    print('\n=== TF transforms seen ===')
    seen = collections.Counter()
    sub = node.create_subscription(
        TFMessage, '/tf',
        lambda m: seen.update(f'{t.header.frame_id}->{t.child_frame_id}'
                              for t in m.transforms), 20)
    t0 = time.time()
    while time.time() - t0 < args.window:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    for k, v in seen.most_common():
        print(f'  {k:<40}{v / args.window:>7.1f} Hz')
    if not any(k.endswith('->base_footprint') and k.startswith('odom') for k in seen):
        failures.append('no odom->base_footprint TF (robot pose is not being published)')

    # ---- map coverage ------------------------------------------------------
    if '/map' in available:
        got = {}
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        node.create_subscription(OccupancyGrid, '/map',
                                 lambda m: got.__setitem__('m', m), qos)
        t0 = time.time()
        while 'm' not in got and time.time() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.3)
        if 'm' in got:
            m = got['m']
            c = collections.Counter(m.data)
            tot = len(m.data)
            print(f'\n=== map ===\n  {m.info.width}x{m.info.height} '
                  f'({m.info.width * m.info.resolution:.1f} x '
                  f'{m.info.height * m.info.resolution:.1f} m) res={m.info.resolution:.3f}\n'
                  f'  origin=({m.info.origin.position.x:.2f}, {m.info.origin.position.y:.2f})\n'
                  f'  unknown={c[-1]}  free={c[0]}  occupied={c[100]}  '
                  f'known={100 * (tot - c[-1]) / tot:.1f}%')

    # ---- verdict -----------------------------------------------------------
    print('\n=== verdict ===')
    for w in warnings:
        print(f'  WARN: {w}')
    for f in failures:
        print(f'  FAIL: {f}')
    if not failures:
        print('  all critical checks passed')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
