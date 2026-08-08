#!/usr/bin/env python3
"""YOLO detector: RGB image -> vision_msgs/Detection2DArray.

Backend-agnostic by design. The `model` parameter takes either a PyTorch `.pt`
checkpoint or an exported OpenVINO IR directory (`*_openvino_model/`); ultralytics
dispatches on the path. `device` then selects `cpu`, `intel:gpu`, or `intel:cpu`.
Downstream nodes see the same `vision_msgs` interface either way, which is the whole
point of keeping the backend behind a parameter -- swapping it is a config change,
never a code change.

Two deliberate choices worth not undoing:

**No cv_bridge.** `pip install ultralytics` drags in `opencv-python`, which shadows
ROS's system `cv2` 4.6 and breaks `cv_bridge` in ways that are tedious to diagnose.
Images are decoded with `np.frombuffer` instead, so that failure mode cannot occur.
Using `cv2` directly from the venv is fine -- it is only `cv_bridge` that breaks.

**Inference runs on a dedicated worker thread, not in any ROS callback.** The
subscription only stores the newest frame and a timer only sets an event, so anything
slower than the camera *drops* stale frames rather than queueing them. A queue would
turn into unbounded latency, and a detection projected against fresh TF lands where the
robot is now rather than where it was when the shutter opened.

Keeping inference out of the executor is also worth ~4x: measured identically, a
predict() call cost 13.9 ms standalone but 53-67 ms inside a MultiThreadedExecutor
callback, and stopped responding to `imgsz` entirely.
"""

import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

# Gazebo's bridge publishes sensor streams BEST_EFFORT. A RELIABLE subscription would
# silently never match it -- one of the failure modes this project has already hit twice.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)


def decode_image(msg):
    """sensor_msgs/Image -> HxWx3 BGR uint8, without cv_bridge.

    Ultralytics follows the OpenCV convention and treats a bare ndarray as BGR, so
    RGB input must be flipped or every detection runs on colour-swapped pixels.

    Every path returns a C-contiguous array. A channel-reversing slice like
    `img[:, :, ::-1]` is only a negative-stride *view*, and handing one to downstream
    OpenCV/ultralytics code forces slow non-contiguous paths or a hidden copy.
    """
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(img[:, :, ::-1]) if enc == 'rgb8' else img
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(msg.height, msg.width, 4)[:, :, :3]
        return np.ascontiguousarray(img[:, :, ::-1] if enc == 'rgba8' else img)
    if enc == 'mono8':
        gray = buf.reshape(msg.height, msg.width)
        return np.ascontiguousarray(np.repeat(gray[:, :, None], 3, axis=2))
    raise ValueError(f'unsupported image encoding: {msg.encoding}')


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('detection_rate_hz', 5.0)
        self.declare_parameter('class_filter', [''])
        self.declare_parameter('torch_threads', 6)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('stats_period_s', 10.0)

        self.model_path = self.get_parameter('model').value
        self.device = self.get_parameter('device').value
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.conf = float(self.get_parameter('conf').value)
        self.iou = float(self.get_parameter('iou').value)
        rate = float(self.get_parameter('detection_rate_hz').value)
        raw_filter = self.get_parameter('class_filter').value
        threads = int(self.get_parameter('torch_threads').value)
        image_topic = self.get_parameter('image_topic').value
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        stats_period = float(self.get_parameter('stats_period_s').value)

        self._configure_threads(threads)
        self._load_model()
        self.class_ids = self._resolve_class_filter([c for c in raw_filter if c])

        self._lock = threading.Lock()
        self._latest = None          # newest Image msg, overwritten freely
        self._latest_seq = 0         # bumped per arrival; identifies a frame
        self._last_seq = -1          # last sequence actually inferred on
        self._infer_ms = []
        self._decode_ms = []
        self._pub_ms = []
        self._frames_seen = 0
        self._frames_run = 0

        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.dbg_pub = (self.create_publisher(Image, '/yolo/debug_image', 1)
                        if self.publish_debug else None)

        self.create_subscription(Image, image_topic, self._on_image, SENSOR_QOS)
        self.create_timer(1.0 / rate, self._on_tick)
        self.create_timer(stats_period, self._on_stats)

        # Inference runs on its own plain thread, NOT as an executor callback.
        #
        # Measured: identical predict() calls took 13.9 ms standalone but 53-67 ms
        # when invoked from a MultiThreadedExecutor callback under the same load --
        # and, tellingly, the cost stopped responding to imgsz at all, which is the
        # signature of contention rather than compute. rclpy's executor loop holds
        # the GIL often enough to starve the heavily-Python pre/post-processing
        # inside ultralytics. The timer now only sets an event; this thread does the
        # work, and OpenVINO/numpy release the GIL while they compute.
        self._tick_evt = threading.Event()
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"model={self.model_path} device={self.device} imgsz={self.imgsz} "
            f"conf={self.conf} rate={rate} Hz threads={threads} "
            f"classes={'all' if not self.class_ids else raw_filter} <- {image_topic}")

    # ---------------------------------------------------------------- setup

    def _configure_threads(self, threads):
        """Cap PyTorch's thread pool.

        Left alone it grabs all 20 cores and starves Gazebo's render loop, which shows
        up as falling RTF and reads like a Gazebo fault rather than a YOLO one. OMP_NUM_THREADS
        only takes effect before torch is imported, so the launch file sets it too; this is
        the belt to that suspenders.
        """
        try:
            import torch
            torch.set_num_threads(threads)
        except ImportError:
            pass  # OpenVINO-only install; ultralytics does not require torch to infer IR

    def _load_model(self):
        from ultralytics import YOLO

        path = self.model_path
        if not os.path.isabs(path) and not os.path.exists(path):
            # Bare names like "yolo11n.pt" are fetched to CWD by ultralytics; keep them
            # in one predictable place instead of wherever the node happened to start.
            cache = os.path.expanduser('~/.cache/tb3_semantic_nav/models')
            os.makedirs(cache, exist_ok=True)
            candidate = os.path.join(cache, path)
            path = candidate if os.path.exists(candidate) else path

        t0 = time.monotonic()
        self.model = YOLO(path, task='detect')
        self.names = self.model.names
        self.get_logger().info(f'loaded {path} in {time.monotonic() - t0:.1f}s')

        # Warm up. The first inference pays lazy allocation, graph compilation, and (for
        # OpenVINO GPU) kernel JIT -- seconds, not milliseconds. Paying it here keeps that
        # cost out of the first real frame and out of the timing statistics.
        dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        t0 = time.monotonic()
        self.model.predict(dummy, imgsz=self.imgsz, device=self.device, verbose=False)
        self.get_logger().info(f'warmup inference {1e3 * (time.monotonic() - t0):.0f} ms')

    def _resolve_class_filter(self, wanted):
        """Class *names* -> model class ids, so filtering happens inside predict()."""
        if not wanted:
            return None
        inverse = {name: i for i, name in self.names.items()}
        ids, unknown = [], []
        for name in wanted:
            (ids.append(inverse[name]) if name in inverse else unknown.append(name))
        if unknown:
            self.get_logger().warn(f'class_filter names not in model: {unknown}')
        return ids or None

    # ------------------------------------------------------------- callbacks

    def _on_image(self, msg):
        with self._lock:
            self._latest = msg
            self._latest_seq += 1
            self._frames_seen += 1

    def _on_tick(self):
        """Pace only. Sim-time aware, so it also works against a bag played at --rate."""
        self._tick_evt.set()

    def _worker_loop(self):
        while not self._shutdown.is_set():
            if not self._tick_evt.wait(timeout=0.5):
                continue
            self._tick_evt.clear()
            try:
                self._run_once()
            except Exception as exc:  # a bad frame must not kill the detector
                self.get_logger().error(f'inference failed: {exc!r}',
                                        throttle_duration_sec=5.0)

    def _run_once(self):
        with self._lock:
            msg, seq = self._latest, self._latest_seq
        # A sequence counter, not id(msg): CPython recycles object addresses, so a
        # freshly allocated message can reuse the previous one's id and be mistaken
        # for a duplicate. That silently skipped ticks and held the detector near
        # 1 Hz when it was configured for 5.
        if msg is None or seq == self._last_seq:
            return
        self._last_seq = seq

        t_dec = time.monotonic()
        try:
            frame = decode_image(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=10.0)
            return

        t0 = time.monotonic()
        results = self.model.predict(
            frame, imgsz=self.imgsz, conf=self.conf, iou=self.iou,
            device=self.device, classes=self.class_ids, verbose=False)
        t1 = time.monotonic()

        self._decode_ms.append(1e3 * (t0 - t_dec))
        self._infer_ms.append(1e3 * (t1 - t0))
        self._frames_run += 1

        t2 = time.monotonic()
        self.det_pub.publish(self._to_detection_array(results[0], msg.header))

        if self.dbg_pub is not None and self.dbg_pub.get_subscription_count() > 0:
            # plot() is not cheap, so only pay for it when something is actually looking.
            self.dbg_pub.publish(self._to_image_msg(results[0].plot(), msg.header))
        self._pub_ms.append(1e3 * (time.monotonic() - t2))

    def _on_stats(self):
        if not self._infer_ms:
            self.get_logger().warn(
                f'no inference yet ({self._frames_seen} frames seen) -- '
                'is camera:=true and is anything publishing the image topic?')
            return
        infer = np.array(self._infer_ms)
        dec = np.array(self._decode_ms or [0.0])
        pub = np.array(self._pub_ms or [0.0])
        self._infer_ms, self._decode_ms, self._pub_ms = [], [], []
        self.get_logger().info(
            f'n={infer.size} infer mean={infer.mean():.0f} p95={np.percentile(infer, 95):.0f} '
            f'max={infer.max():.0f} | decode mean={dec.mean():.1f} | pub mean={pub.mean():.1f} '
            f'(ms) | frames seen={self._frames_seen} run={self._frames_run} '
            f'dropped={self._frames_seen - self._frames_run}')

    def shutdown(self):
        self._shutdown.set()
        self._tick_evt.set()
        self._worker.join(timeout=2.0)

    # --------------------------------------------------------- msg building

    def _to_detection_array(self, result, header):
        """Stamp every detection with the *source image's* header.

        Not the current time. Downstream projection transforms using this stamp, and a
        'latest' lookup would place the object where the robot is now rather than where
        it was when the frame was captured.
        """
        out = Detection2DArray()
        out.header = header

        boxes = result.boxes
        if boxes is None:
            return out

        for xywh, cls, conf in zip(boxes.xywh.tolist(),
                                   boxes.cls.tolist(),
                                   boxes.conf.tolist()):
            cx, cy, w, h = xywh
            det = Detection2D()
            det.header = header

            bbox = BoundingBox2D()
            bbox.center.position.x = float(cx)
            bbox.center.position.y = float(cy)
            bbox.center.theta = 0.0
            bbox.size_x = float(w)
            bbox.size_y = float(h)
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            # The COCO class *name*, not the integer id -- the semantic map keys off it,
            # and a bare integer is unreadable in RViz and in saved maps.
            hyp.hypothesis.class_id = str(self.names[int(cls)])
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            det.id = hyp.hypothesis.class_id

            out.detections.append(det)
        return out

    def _to_image_msg(self, bgr, header):
        msg = Image()
        msg.header = header
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = 3 * msg.width
        msg.data = np.ascontiguousarray(bgr).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    # A single-threaded executor is correct here: it only drains the subscription and
    # fires the pacing timers. Inference happens on the node's own worker thread, which
    # is what keeps rclpy's GIL-heavy spin loop from starving it.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
