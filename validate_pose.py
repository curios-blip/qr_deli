#!/usr/bin/env python3
"""Check the distance/angle estimator against the D4 frames (true values are in the file names)."""
import glob, os, re, sys
import cv2, numpy as np
from sign_pose import CODE_FRACTION, estimate

FOCAL = float(sys.argv[1]) if len(sys.argv) > 1 else 530.0
det = cv2.QRCodeDetector()
rows = []
for f in sorted(glob.glob(os.path.expanduser('~/qr_delivery/range_frames/*.png'))):
    m = re.match(r's(\d+)_d([\d.]+)_a(\d+)\.png', os.path.basename(f))
    img = cv2.imread(f)
    text, pts, _ = det.detectAndDecode(img)
    if not m or pts is None:
        continue
    size, d, a = int(m[1]) / 100, float(m[2]), float(m[3])
    h, w = img.shape[:2]
    e = estimate(pts, size * CODE_FRACTION, FOCAL, w / 2, h / 2)
    if e:
        rows.append((size, d, a, e[0], e[1], 1 if text else 0))
if not rows:
    sys.exit('no usable frames')
r = np.array(rows)


def report(name, x):
    if len(x) == 0:
        return
    derr = 100 * (x[:, 3] / x[:, 1] - 1)
    aerr = x[:, 4] - x[:, 2]
    print(f'{name}: {len(x)} frames | distance mean {derr.mean():+.1f}%, worst {np.abs(derr).max():.1f}% | '
          f'angle mean {aerr.mean():+.1f} deg, rms {np.sqrt((aerr ** 2).mean()):.1f} deg')


read = r[r[:, 5] == 1]
report('all detected ', r)
report('actually read', read)
sel = read if len(read) else r
print(f'suggested focal_px: {FOCAL * np.median(sel[:, 1] / sel[:, 3]):.0f}   (current {FOCAL:.0f})')
