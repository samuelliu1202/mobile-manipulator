#!/usr/bin/env python3
"""Benchmark a YOLO model over a directory of frames: what it detects, and how fast.

    tools/bench_yolo.py <model> <frames_dir> [device] [imgsz] [conf] [annotate_dir]

Run with the venv interpreter, e.g.
    ~/.venvs/yolo/bin/python tools/bench_yolo.py \\
        yolo11n_openvino_model320 frames/ intel:gpu.1 320 0.35

Devices: cpu | intel:cpu | intel:gpu.0 (iGPU) | intel:gpu.1 (Arc dGPU).
List what OpenVINO can see with:
    ~/.venvs/yolo/bin/python -c "import openvino; print(openvino.Core().available_devices)"

Reports detection quality and speed separately, because they fail independently: a
model that detects nothing still benchmarks fast, and on this project's world a model
can be both fast and confidently wrong.
"""
import collections
import os
import sys
import time

import numpy as np

MODEL_CACHE = os.path.expanduser('~/.cache/tb3_semantic_nav/models')


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    model_path = sys.argv[1]
    frames_dir = sys.argv[2]
    device = sys.argv[3] if len(sys.argv) > 3 else 'intel:cpu'
    imgsz = int(sys.argv[4]) if len(sys.argv) > 4 else 320
    conf = float(sys.argv[5]) if len(sys.argv) > 5 else 0.35
    annotate = sys.argv[6] if len(sys.argv) > 6 else None

    # Same bare-name resolution the node uses, so both find the same weights.
    if not os.path.isabs(model_path) and not os.path.exists(model_path):
        cand = os.path.join(MODEL_CACHE, model_path)
        if os.path.exists(cand):
            model_path = cand

    import cv2
    from ultralytics import YOLO

    files = sorted(f for f in os.listdir(frames_dir)
                   if f.lower().endswith(('.png', '.jpg', '.jpeg')))
    if not files:
        print(f'no images in {frames_dir}')
        return 1

    model = YOLO(model_path, task='detect')
    names = model.names

    # Warm up: first call pays graph compile and, on GPU, kernel JIT -- seconds, not ms.
    t_warm = time.perf_counter()
    model.predict(np.zeros((imgsz, imgsz, 3), np.uint8), imgsz=imgsz,
                  device=device, verbose=False)
    warm_ms = 1e3 * (time.perf_counter() - t_warm)

    times, hits = [], collections.Counter()
    best = collections.defaultdict(lambda: (0.0, None))
    frames_with_det = 0
    if annotate:
        os.makedirs(annotate, exist_ok=True)

    for fn in files:
        bgr = cv2.imread(os.path.join(frames_dir, fn))
        t0 = time.perf_counter()
        res = model.predict(bgr, imgsz=imgsz, conf=conf, device=device, verbose=False)[0]
        times.append(1e3 * (time.perf_counter() - t0))

        boxes = res.boxes
        if boxes is None or len(boxes) == 0:
            continue
        frames_with_det += 1
        for cls, cf in zip(boxes.cls.tolist(), boxes.conf.tolist()):
            name = names[int(cls)]
            hits[name] += 1
            if cf > best[name][0]:
                best[name] = (cf, fn)
        if annotate:
            cv2.imwrite(os.path.join(annotate, fn), res.plot())

    t = np.array(times)
    print(f'\nmodel={os.path.basename(model_path)} device={device} '
          f'imgsz={imgsz} conf={conf} n_frames={len(files)}')
    print(f'warmup: {warm_ms:.0f} ms')
    print(f'speed : mean {t.mean():6.1f} ms  p95 {np.percentile(t, 95):6.1f} ms  '
          f'min {t.min():5.1f}  -> {1000.0 / t.mean():5.1f} FPS')
    print(f'detect: {frames_with_det}/{len(files)} frames with >=1 detection, '
          f'{sum(hits.values())} detections total')
    if hits:
        print(f'{"class":18s} {"count":>5s}  {"best":>5s}  best frame')
        for name, n in hits.most_common():
            cf, fn = best[name]
            print(f'{name:18s} {n:5d}  {cf:5.2f}  {fn}')
    else:
        print('  *** NOTHING DETECTED ***')
    return 0


if __name__ == '__main__':
    sys.exit(main())
