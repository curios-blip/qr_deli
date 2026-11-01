#!/usr/bin/env python3
import glob, os, sys
import cv2, numpy as np

DIR = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/qr_delivery/range_frames')
det = cv2.QRCodeDetector()
files = sorted(glob.glob(f'{DIR}/*.png'))
if not files:
    sys.exit(f'no frames in {DIR}')
h, w = cv2.imread(files[0]).shape[:2]
print(f'camera image size: {w}x{h}\n')
print(f'{"frame":<22}{"found":<7}{"px/module":<11}{"edge":<6}decoded')
for f in files:
    img = cv2.imread(f)
    found, pts = det.detect(img)
    name = os.path.basename(f)[:-4]
    if not found or pts is None:
        print(f'{name:<22}{"no":<7}{"-":<11}{"-":<6}-')
        continue
    p = pts.reshape(-1, 2)
    side = np.mean([np.linalg.norm(p[i] - p[(i + 1) % 4]) for i in range(4)])
    edge = p.min() < 3 or p[:, 0].max() > w - 4 or p[:, 1].max() > h - 4
    text = det.decode(img, pts)[0]
    print(f'{name:<22}{"yes":<7}{side / 21:<11.1f}{"YES" if edge else "":<6}{text or "FAILED"}')
