#!/usr/bin/env python3
import csv, math, os, subprocess, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# reading pose ~0.9 m in front of each sign, facing it: (x, y, yaw)
ROOMS = {
    'room_101': (2.9, 0.0,  math.pi / 2),
    'room_102': (6.9, 0.0,  math.pi / 2),
    'room_103': (2.9, 0.0, -math.pi / 2),
    'room_104': (6.9, 0.0, -math.pi / 2),
}
START = (0.7, 0.0, 0.0)   # must match the spawn pose in rooms.launch.py
LOG = os.path.expanduser('~/qr_delivery/trips.csv')
SWEEP = [0.26, 0.26, -0.78, -0.26]   # relative turns: +15, +30, -15, -30 deg from arrival heading


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


def read_sign(nav, seen, window=1.5):
    seen['text'] = None
    end = time.time() + window
    while seen['text'] is None and time.time() < end:
        rclpy.spin_once(nav, timeout_sec=0.1)
    return seen['text'] or ''


ANGLES = [20, -20, 40, -40, 60, -60]   # degrees from arrival heading, tried in this order

def read_with_fallback(nav, seen):
    read, tries, cur = read_sign(nav, seen), 0, 0.0
    for a in ANGLES:
        if read:
            break
        nav.spin(spin_dist=math.radians(a - cur), time_allowance=10)
        while not nav.isTaskComplete():
            pass
        cur = a
        tries += 1
        read = read_sign(nav, seen)
    return read, tries


def main():
    rooms, yaw_off = [], 0.0
    for a in sys.argv[1:]:
        if a.startswith('yaw='):
            yaw_off = math.radians(float(a[4:]))
        else:
            rooms.append(a)
    yaw_off_deg = round(math.degrees(yaw_off))
    bad = [r for r in rooms if r not in ROOMS]
    if not rooms or bad:
        sys.exit(f'usage: trip.py room_101 [room_103 ...] [yaw=DEG]  known: {list(ROOMS)}  unknown: {bad}')

    rclpy.init()
    nav = BasicNavigator()
    nav.initial_pose_received = True   # skip the library's own wait for amcl_pose
    start = PoseWithCovarianceStamped()
    start.header.frame_id = 'map'      # stamp stays 0, so AMCL uses its own (sim) clock
    start.pose.pose.position.x, start.pose.pose.position.y = START[0], START[1]
    start.pose.pose.orientation.z = math.sin(START[2] / 2)
    start.pose.pose.orientation.w = math.cos(START[2] / 2)
    start.pose.covariance[0] = start.pose.covariance[7] = 0.25
    start.pose.covariance[35] = 0.07
    pub = nav.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    for _ in range(5):
        pub.publish(start)
        time.sleep(0.5)
    nav.waitUntilNav2Active()
    # tighter goal tolerance so the sign stays inside the camera's view (Nav2 default is 0.25 m / 0.25 rad)
    names = subprocess.run(['ros2', 'param', 'list', '/controller_server'], capture_output=True, text=True).stdout.split()
    found = [n for n in names if n.endswith('xy_goal_tolerance') or n.endswith('yaw_goal_tolerance')]
    print('goal tolerance parameters found:', found)
    for n in found:
        subprocess.run(['ros2', 'param', 'set', '/controller_server', n, '0.10'])

    seen = {'text': None}
    cam = {'frame': None}
    bridge = CvBridge()
    nav.create_subscription(Image, '/camera/image_raw',
                            lambda m: cam.update(frame=bridge.imgmsg_to_cv2(m, 'bgr8')), qos_profile_sensor_data)
    nav.create_subscription(String, '/room_sign', lambda m: seen.update(text=m.data), 10)

    new_file = not os.path.exists(LOG)
    with open(LOG, 'a', newline='') as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(['time', 'expected', 'read', 'outcome', 'tries', 'seconds', 'yaw_off_deg'])
        for room in rooms:
            t0 = time.time()
            tries = 0
            gx, gy, gyaw = ROOMS[room]
            nav.goToPose(make_pose(nav, gx, gy, gyaw + yaw_off))
            while not nav.isTaskComplete():
                pass
            if nav.getResult() != TaskResult.SUCCEEDED:
                read, outcome = '', 'NAV_FAILED'
            else:
                read, tries = read_with_fallback(nav, seen)
                outcome = 'OK' if read == room else ('UNREADABLE' if not read else 'MISMATCH')
                if not read and cam['frame'] is not None:
                    stamp = time.strftime('%H%M%S')
                    cv2.imwrite(os.path.expanduser(f'~/qr_delivery/noread_{room}_{stamp}.png'), cam['frame'])
            secs = round(time.time() - t0, 1)
            print(f'{room}: {outcome} (read "{read}", {tries} sweeps, yaw offset {yaw_off_deg} deg) in {secs}s')
            w.writerow([time.strftime('%H:%M:%S'), room, read, outcome, tries, secs, yaw_off_deg])
            f.flush()
    nav.goToPose(make_pose(nav, *START))   # go home so the next run starts at the start pose
    while not nav.isTaskComplete():
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
