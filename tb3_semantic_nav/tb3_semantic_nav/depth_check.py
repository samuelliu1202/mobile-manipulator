"""Validate the RGBD depth stream against the LiDAR — the Phase 5 fusion precondition.

Deprojects the depth image's centre pixel into the optical frame, transforms it into
base_scan, and compares the resulting range/bearing against the actual laser return at
that bearing. If these disagree, every semantic projection downstream will be wrong.
"""
import math
import time

import numpy as np
import rclpy
# Imported for the side effect of registering PointStamped with tf2's converter
# registry; without it Buffer.transform() raises "type is not loaded or supported".
import tf2_geometry_msgs  # noqa: F401
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener

def main(argv=None):
    rclpy.init()
    n = rclpy.create_node('depth_check')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    tf_buffer = Buffer()
    TransformListener(tf_buffer, n)

    qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.VOLATILE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=5)
    got = {}
    n.create_subscription(Image, '/camera/depth/image_raw', lambda m: got.__setitem__('d', m), qos)
    n.create_subscription(CameraInfo, '/camera/camera_info', lambda m: got.__setitem__('k', m), qos)
    n.create_subscription(LaserScan, '/scan', lambda m: got.__setitem__('s', m), qos)

    t0 = time.time()
    while not {'d', 'k', 's'} <= got.keys() and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)

    # The TransformListener needs to actually receive latched /tf_static before any
    # lookup will succeed; topics arrive well before that, so wait explicitly.
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.2)
        if tf_buffer.can_transform('base_scan', 'camera_rgb_optical_frame', rclpy.time.Time()):
            break
    else:
        print('FAIL: camera_rgb_optical_frame -> base_scan never became available')
        raise SystemExit(1)
    missing = {'d', 'k', 's'} - got.keys()
    if missing:
        print(f'FAIL: never received {missing}')
        raise SystemExit(1)

    d, k, s = got['d'], got['k'], got['s']
    fx, fy, cx, cy = k.k[0], k.k[4], k.k[2], k.k[5]
    print(f'camera_info: {k.width}x{k.height}  fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}')
    print(f'             frame_id={k.header.frame_id}')

    # Expected fx for a 640-wide image with hfov 1.02974 rad
    exp_fx = (k.width / 2.0) / math.tan(1.02974 / 2.0)
    print(f'expected fx from hfov 1.02974 -> {exp_fx:.1f}   (delta {abs(fx - exp_fx):.1f})')

    depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    finite = np.isfinite(depth) & (depth > 0)
    print(f'depth: {d.encoding} {d.width}x{d.height}  '
          f'valid={100 * finite.mean():.1f}%  '
          f'min={np.nanmin(depth[finite]):.2f} max={np.nanmax(depth[finite]):.2f} m')

    # Sample columns across the image at the row nearest the lidar plane, deproject each,
    # and compare with the laser return at the same bearing. Only columns where BOTH sensors
    # see something are scored -- the camera reaches 10 m but the lidar stops at 3.5 m, so
    # camera-only hits are expected, not errors.
    from geometry_msgs.msg import PointStamped

    v = d.height // 2
    errs, compared, cam_only = [], 0, 0
    print(f'\n{"col":>5}{"depth m":>10}{"bearing":>10}{"r_cam":>9}{"r_lidar":>9}{"delta":>8}')
    for u in range(40, d.width - 40, 40):
        patch = depth[v - 6:v + 6, u - 6:u + 6]
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size < 20:
            continue
        Z = float(np.median(patch))
        p = PointStamped()
        p.header.frame_id = k.header.frame_id
        p.header.stamp = rclpy.time.Time().to_msg()          # latest available
        p.point.x = (u - cx) * Z / fx
        p.point.y = (v - cy) * Z / fy
        p.point.z = Z
        try:
            q = tf_buffer.transform(p, 'base_scan', timeout=Duration(seconds=2.0))
        except Exception as e:
            print(f'FAIL: tf transform to base_scan failed: {e}')
            raise SystemExit(1)

        bearing = math.atan2(q.point.y, q.point.x)
        r_cam = math.hypot(q.point.x, q.point.y)
        idx = int(round((bearing - s.angle_min) / s.angle_increment)) % len(s.ranges)
        window = [s.ranges[(idx + o) % len(s.ranges)] for o in range(-2, 3)]
        valid = [r for r in window if math.isfinite(r) and s.range_min < r < s.range_max]
        if not valid:
            cam_only += 1
            print(f'{u:>5}{Z:>10.2f}{math.degrees(bearing):>9.1f}d{r_cam:>9.2f}'
                  f'{"beyond":>9}{"-":>8}   (lidar range is 3.5 m)')
            continue
        r_lidar = min(valid)
        err = abs(r_cam - r_lidar)
        errs.append(err)
        compared += 1
        print(f'{u:>5}{Z:>10.2f}{math.degrees(bearing):>9.1f}d{r_cam:>9.2f}'
              f'{r_lidar:>9.2f}{err:>8.3f}')

    print(f'\n{compared} columns comparable, {cam_only} camera-only (past lidar range)')
    if errs:
        # Median, not mean. Columns straddling a depth discontinuity are expected to
        # disagree wildly: the camera ray slips past the edge of a near object onto a far
        # wall while the lidar, ~13 cm away on the robot, still strikes the object. Those
        # are genuine parallax outliers, and rejecting them is exactly what the Phase 5
        # fusion gate is for -- they should not drag the health metric.
        med = float(np.median(errs))
        outliers = [e for e in errs if e > 0.15]
        print(f'depth-vs-lidar error: median {med:.3f} m, '
              f'{len(errs) - len(outliers)}/{len(errs)} columns within 0.15 m')
        if outliers:
            print(f'  {len(outliers)} edge/parallax outlier(s): '
                  f'{", ".join(f"{e:.2f}" for e in outliers)} m')
        print('\nVERDICT:', 'PASS - depth agrees with lidar'
              if med < 0.15 else f'FAIL - median {med:.3f} m disagreement')
    else:
        print('VERDICT: INCONCLUSIVE - nothing within lidar range; drive closer to a wall')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
