#!/usr/bin/env python3
"""D4: QR reading success vs distance and viewing angle, for two sign sizes."""
import csv, math, os, random, subprocess, sys, time
import cv2, qrcode, rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WORLD, ROBOT = 'rooms', 'waffle_pi'          # robot name: check with `gz model --list`
N = int(sys.argv[1]) if len(sys.argv) > 1 else 5   # trials per condition
SIZES = [0.15, 0.30]                          # sign edge length (m)
DISTS = {0.15: [0.4, 0.5, 0.6, 0.7, 0.8, 0.9], 0.30: [0.8, 1.0, 1.2, 1.4, 1.6, 1.8]}        # camera to sign (m)
ANGLES = [0, 10, 20, 30, 40, 50]              # viewing angle off the sign normal (deg)
SIGN_POS = {0.15: (14.0, -3.0), 0.30: (14.0, 3.0)}   # outside the building, facing -x
BASE = os.path.expanduser('~/qr_delivery')
PNG, TEXT = f'{BASE}/signs/range_test.png', 'range_test'


def gz_service(name, reqtype, req):
    r = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/{name}',
                        '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '3000', '--req', req],
                       capture_output=True, text=True)
    return 'data: true' in r.stdout


def spawn_sign(name, x, y, size):
    sdf = (f"<sdf version='1.10'><model name='{name}'><static>true</static>"
           f"<pose>{x} {y} 0.2 1.5708 0 -1.5708</pose><link name='l'><visual name='v'>"
           f"<geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>"
           f"<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>"
           f"<pbr><metal><albedo_map>{PNG}</albedo_map><metalness>0</metalness>"
           f"<roughness>1</roughness></metal></pbr></material></visual></link></model></sdf>")
    ok = gz_service('create', 'gz.msgs.EntityFactory', f'sdf: "{sdf}"')
    print(f'spawn {name}: {"ok" if ok else "FAILED (already there?)"}')


def teleport(x, y, yaw):
    req = (f'name: "{ROBOT}", position: {{x: {x:.4f}, y: {y:.4f}, z: 0.01}}, '
           f'orientation: {{z: {math.sin(yaw / 2):.5f}, w: {math.cos(yaw / 2):.5f}}}')
    return gz_service('set_pose', 'gz.msgs.Pose', req)


class Cam(Node):
    def __init__(self):
        super().__init__('range_study')
        self.bridge, self.frame, self.t = CvBridge(), None, 0.0
        self.create_subscription(Image, '/camera/image_raw', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.t = time.time()

    def settle(self, secs):               # keep spinning so old queued frames are discarded
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def fresh_frame(self, timeout=6.0):   # first frame that arrives from now on
        start, end = time.time(), time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.t > start:
                return self.frame
        return None


def main():
    os.makedirs(f'{BASE}/range_frames', exist_ok=True)
    qrcode.make(TEXT).save(PNG)
    rclpy.init()
    cam, det, rng = Cam(), cv2.QRCodeDetector(), random.Random(1)
    for size, (sx, sy) in SIGN_POS.items():
        spawn_sign(f'range_sign_{round(size * 100)}', sx, sy, size)
    time.sleep(2)
    if not teleport(10.0, 0.0, 0.0):
        sys.exit(f'Teleport failed. Run `gz model --list`, put the robot name in ROBOT at the top of this file.')

    # warm-up: wait until the 30 cm test sign is visible and decodable
    wx, wy = SIGN_POS[0.30]
    teleport(wx - 1.0, wy, 0.0)
    for _ in range(40):
        cam.settle(1.0)
        f = cam.fresh_frame()
        if f is not None and det.detectAndDecode(f)[0] == TEXT:
            print('warm-up ok: test sign is visible')
            break
    else:
        sys.exit('Warm-up failed: cannot read the 30 cm sign at 1 m. Look at rqt_image_view.')

    rows, hits = [], {}
    for size, (sx, sy) in SIGN_POS.items():
        for d in DISTS[size]:
            for a in ANGLES:
                hits[(size, d, a)] = 0
                for k in range(N):
                    dd = d + rng.uniform(-0.02, 0.02)
                    aa = math.radians(a + rng.uniform(-1.0, 1.0))
                    x, y = sx - dd * math.cos(aa), sy + dd * math.sin(aa)
                    teleport(x, y, -aa + math.radians(rng.uniform(-1.5, 1.5)))
                    cam.settle(1.0)
                    frame = cam.fresh_frame()
                    ok = False
                    if frame is None:
                        print('  (no camera frame received)')
                    else:
                        ok = det.detectAndDecode(frame)[0] == TEXT
                        if k == 0:
                            cv2.imwrite(f'{BASE}/range_frames/s{round(size * 100)}_d{d}_a{a}.png', frame)
                    hits[(size, d, a)] += ok
                    rows.append([size, d, a, k, int(ok)])
                print(f'sign {size * 100:.0f} cm  dist {d} m  angle {a:>2} deg: {hits[(size, d, a)]}/{N}')

    with open(f'{BASE}/range_results.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['size_m', 'distance_m', 'angle_deg', 'trial', 'success'])
        w.writerows(rows)
    lines = []
    for size in SIZES:
        lines += [f'\nSign {size * 100:.0f} cm: successes out of {N} (rows = distance m, columns = angle deg)',
                  'dist  ' + ''.join(f'{a:>6}' for a in ANGLES)]
        for d in DISTS[size]:
            lines.append(f'{d:<6}' + ''.join(f'{hits[(size, d, a)]:>6}' for a in ANGLES))
    text = '\n'.join(lines)
    print(text)
    open(f'{BASE}/range_table.txt', 'w').write(text + '\n')


if __name__ == '__main__':
    main()
