#!/usr/bin/env python3
"""Autonomous frontier exploration: map a whole multi-room house via Nav2.

    python3 tools/frontier_explore.py [--budget 1800] [--min-frontier 12]

Needs `ros2 launch tb3_semantic_nav explore_bringup.launch.py` running (sim +
slam_toolbox + Nav2 without AMCL).

Why this and not the simpler drivers already in the repo: `explore_drive` is reactive
and settles into a loop in whichever room it starts in; `tools/map_tour.py` drives
straight at a waypoint and oscillates against the wall beside a doorway. Neither can route
around an obstacle. Nav2's planner can, so the only missing piece is choosing
*where* to go -- which is what frontier detection provides.

A frontier is a known-free cell adjacent to unknown space: the boundary of what has
been seen. Driving to the nearest sizeable frontier, repeatedly, is what sweeps a
house. Exploration ends when no frontier cluster is large enough to be worth visiting.

Two details that matter in practice:

- Candidate goals are pulled *back* from the frontier edge into free space with real
  clearance. A cell right on the unknown boundary is usually against a wall, and Nav2
  rejects goals inside the costmap's inflation radius.
- Failed goals are blacklisted by radius, not by exact cell. Retrying the same
  unreachable doorway forever is the main way this kind of loop stalls.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

# slam_toolbox latches /map, so a transient-local subscription is required to get it.
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

FREE, UNKNOWN = 0, -1
OCC_THRESH = 50           # >= this is an obstacle in an OccupancyGrid
MIN_CLEARANCE_M = 0.55    # must exceed the costmap's inflation_radius (0.5), not just
                          # robot_radius (0.15). A goal inside the inflation band carries
                          # high cost and navfn reports "Failed to create plan with
                          # tolerance of: 0.5" -- measured repeatedly against live goals.
MIN_GOAL_DIST_M = 0.70    # ignore frontiers underfoot: Nav2 reports instant success, the
                          # robot never moves, and the map never grows
MIN_NEW_CELLS = 25        # below this a "successful" goal taught us nothing, so blacklist
                          # it anyway or the loop livelocks. Counted in CELLS, not percent:
                          # slam_toolbox grows the grid as it maps, so a percentage can fall
                          # while the map genuinely improves.


def distance_transform(passable_mask, res):
    """Metres from each cell to the nearest impassable cell.

    Pass a mask of "not an obstacle", NOT "known free". Unknown space is not a physical
    barrier, and treating it as one is a trap: a frontier cell is *by definition*
    adjacent to unknown, so its distance-to-non-free is always one cell. Measured on a
    live map that rejected 100% of 138 frontier cells; against obstacles only, 78 passed.

    Uses scipy when present; otherwise falls back to an iterative erosion, which is
    slower but keeps this tool free of a hard scipy dependency.
    """
    try:
        from scipy import ndimage
        return ndimage.distance_transform_edt(passable_mask) * res
    except ImportError:
        pass
    dist = np.zeros(passable_mask.shape, dtype=np.float32)
    cur = passable_mask.copy()
    for step in range(1, int(math.ceil(MIN_CLEARANCE_M / res)) + 2):
        shrunk = (
            cur
            & np.roll(cur, 1, 0) & np.roll(cur, -1, 0)
            & np.roll(cur, 1, 1) & np.roll(cur, -1, 1)
        )
        dist[shrunk & (dist == 0)] = step * res
        cur = shrunk
        if not cur.any():
            break
    return dist


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explore')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.grid = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_map(self, msg):
        self.grid = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y

    # ------------------------------------------------------------- frontiers

    def frontiers(self, min_cells):
        """Return [(x, y, cell_count)] candidate goals, nearest-first is applied later."""
        g = self.grid
        if g is None:
            return []
        h, w = g.info.height, g.info.width
        res, ox, oy = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y
        a = np.asarray(g.data, dtype=np.int16).reshape(h, w)

        free = a == FREE
        unknown = a == UNKNOWN
        # A frontier cell is free and 4-adjacent to unknown.
        pad = np.pad(unknown, 1, constant_values=False)
        touches_unknown = (pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:])
        frontier = free & touches_unknown
        if not frontier.any():
            return []

        # Clearance from OBSTACLES, not from unknown space -- see distance_transform().
        clearance = distance_transform(~(a >= OCC_THRESH), res)
        reachable = clearance >= MIN_CLEARANCE_M

        # Bin frontier cells into ~0.5 m buckets to group them into clusters cheaply;
        # full connected-components is unnecessary at this resolution.
        bucket = max(1, int(round(0.5 / res)))
        ys, xs = np.where(frontier)
        groups = {}
        for r, c in zip(ys, xs):
            groups.setdefault((r // bucket, c // bucket), []).append((r, c))

        out = []
        for cells in groups.values():
            if len(cells) < min_cells:
                continue
            # Represent the cluster by the member with the most clearance, so the goal
            # sits in open space rather than tight against the wall forming the frontier.
            r, c = max(cells, key=lambda rc: clearance[rc])
            if not reachable[r, c]:
                continue
            out.append((ox + c * res, oy + r * res, len(cells)))
        return out

    def coverage(self):
        g = self.grid
        if g is None:
            return 0, 0.0
        a = np.asarray(g.data, dtype=np.int16)
        known = int(((a == FREE) | (a >= OCC_THRESH)).sum())
        return known, 100.0 * known / max(1, a.size)


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--budget', type=float, default=1800.0, help='total wall-clock seconds')
    ap.add_argument('--goal-timeout', type=float, default=90.0)
    ap.add_argument('--min-frontier', type=int, default=6,
                    help='minimum frontier cells in a cluster to bother visiting')
    ap.add_argument('--blacklist-radius', type=float, default=0.5)
    ap.add_argument('--no-sweep', action='store_true',
                    help='skip the coarse waypoint sweep and go straight to frontiers')
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = FrontierExplorer()
    nav = BasicNavigator()

    print('waiting for /map and TF...', flush=True)
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline and (node.grid is None or node.robot_xy() is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.grid is None or node.robot_xy() is None:
        print('FAILED: no /map or no map->base_footprint TF. Is explore_bringup running?')
        return 1

    # NOT waitUntilNav2Active(): it waits on a localizer, and passing '' makes it block
    # forever on a lifecycle service for an empty node name. There is no AMCL here --
    # slam_toolbox owns the map frame -- so wait on the thing that actually matters.
    if not nav.nav_to_pose_client.wait_for_server(timeout_sec=120.0):
        print('FAILED: navigate_to_pose action server never appeared')
        return 1
    print('Nav2 action server up; exploring', flush=True)

    # Phase 1: a coarse sweep so the robot reaches every part of the house. Frontier
    # search alone is greedy and stays local; these waypoints supply global direction
    # while Nav2 supplies the path planning that a naive controller could not do.
    # Unreachable ones simply time out and are skipped -- walls are not modelled here.
    SWEEP = [(1.0, 0.0), (3.0, 0.0), (5.0, 0.0), (6.0, -2.0), (4.0, -3.0),
             (1.0, -3.0), (-1.0, -3.0), (-2.0, -1.0), (2.7, 1.5), (2.7, 3.0),
             (0.0, 3.5), (-2.0, 3.5), (2.0, 5.5), (-1.0, 7.5)]

    blacklist = []
    visited = 0
    failures = 0
    empty_rounds = 0
    start = time.monotonic()
    _, cov0 = node.coverage()
    print(f'  start coverage {cov0:.1f}%', flush=True)

    if not args.no_sweep:
        print(f'phase 1: coarse sweep, {len(SWEEP)} waypoints', flush=True)
        for i, (gx, gy) in enumerate(SWEEP, 1):
            if time.monotonic() - start > args.budget * 0.6:
                print('  sweep budget spent; moving to frontier phase', flush=True)
                break
            here = node.robot_xy()
            if here is None:
                continue
            yaw = math.atan2(gy - here[1], gx - here[0])
            nav.goToPose(make_pose(nav, gx, gy, yaw))
            t0 = time.monotonic()
            while not nav.isTaskComplete():
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() - t0 > args.goal_timeout:
                    nav.cancelTask()
                    break
            node.spin_for(2.0)
            known, cov = node.coverage()
            print(f'  sweep [{i}/{len(SWEEP)}] ({gx:+.1f},{gy:+.1f}) '
                  f'{"OK " if nav.getResult() == TaskResult.SUCCEEDED else "skip"} '
                  f'known {known} {cov:.1f}%  {time.monotonic() - start:.0f}s', flush=True)
        print('phase 2: frontier fill', flush=True)
    while time.monotonic() - start < args.budget:
        node.spin_for(1.5)                       # let /map refresh before re-deciding
        cands = node.frontiers(args.min_frontier)
        here = node.robot_xy()
        if here is None:
            continue

        cands = [c for c in cands
                 if all(math.hypot(c[0] - bx, c[1] - by) > args.blacklist_radius
                        for bx, by in blacklist)]
        # Drop frontiers the robot is already standing on. Without this the nearest
        # candidate sits at distance ~0, Nav2 instantly reports SUCCEEDED, coverage never
        # changes, and the loop spins on one goal forever -- observed live: 113 goals
        # issued, 7.5% coverage throughout, robot stationary.
        cands = [c for c in cands
                 if math.hypot(c[0] - here[0], c[1] - here[1]) >= MIN_GOAL_DIST_M]
        if not cands:
            # Do not conclude "done" on the first empty look. On a small map the filters
            # can transiently exclude everything; a spin reveals new cells cheaply and
            # often produces fresh frontiers. Only give up if that stops helping.
            if empty_rounds < 3:
                empty_rounds += 1
                print(f'  no candidates (round {empty_rounds}/3) -- spinning to reveal map',
                      flush=True)
                nav.spin(spin_dist=3.14, time_allowance=25)
                t0 = time.monotonic()
                while not nav.isTaskComplete() and time.monotonic() - t0 < 30:
                    rclpy.spin_once(node, timeout_sec=0.05)
                node.spin_for(3.0)
                continue
            print('  no reachable frontier clusters after recovery -- exploration complete',
                  flush=True)
            break
        empty_rounds = 0

        # Information gain per unit travel, NOT nearest-first. Pure nearest-first was
        # measured to nibble the local boundary and never commit to a distant region: it
        # covered 6.7 m^2 where a dumb waypoint sweep covered 33.3 m^2, because a 47-cell
        # cluster 1 m away always outranks a 200-cell cluster across the house.
        def score(c):
            # Weak distance penalty on purpose. With 1/sqrt(d) the robot still refused to
            # cross the house: a 40-cell frontier 6 m away lost to a 39-cell one 1 m away,
            # so it ground away locally and never took the 2.25 m opening it could see.
            d = max(0.5, math.hypot(c[0] - here[0], c[1] - here[1]))
            return -c[2] / (1.0 + 0.15 * d)
        gx, gy, ncells = min(cands, key=score)

        known_before, _ = node.coverage()
        yaw = math.atan2(gy - here[1], gx - here[0])
        nav.goToPose(make_pose(nav, gx, gy, yaw))

        t0 = time.monotonic()
        while not nav.isTaskComplete():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - t0 > args.goal_timeout:
                nav.cancelTask()
                break

        result = nav.getResult()
        # slam_toolbox only integrates a scan after minimum_travel_distance (0.2 m) and
        # republishes on map_update_interval (1.0 s). Measuring sooner reports +0 cells
        # for a goal that actually worked, which then wrongly blacklists it.
        node.spin_for(3.0)
        known, cov = node.coverage()
        gained = known - known_before
        # Map growth is the objective, so it decides success -- not Nav2's own verdict.
        # A goal that aborts partway can still reveal a lot (measured: +1600 cells on an
        # aborted goal), and counting those as failures tripped the consecutive-failure
        # stop and ended exploration early. Blacklist only what teaches us nothing.
        ok = gained >= MIN_NEW_CELLS or result == TaskResult.SUCCEEDED
        visited += 1
        if ok:
            failures = 0
        else:
            failures += 1
            blacklist.append((gx, gy))
        print(f'  [{visited}] ({gx:+.2f},{gy:+.2f}) cells={ncells} '
              f'{"OK " if ok else "FAIL"} known {known} (+{gained}) {cov:.1f}%  '
              f'{time.monotonic() - start:.0f}s elapsed  blacklist={len(blacklist)}',
              flush=True)

        if failures >= 8:
            print('  8 consecutive failures -- stopping', flush=True)
            break

    known, cov = node.coverage()
    print(f'\ndone: {visited} goals, coverage {cov0:.1f}% -> {cov:.1f}% '
          f'in {time.monotonic() - start:.0f}s', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
