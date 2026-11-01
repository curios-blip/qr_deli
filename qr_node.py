#!/usr/bin/env python3
import csv, math, os, time
import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sign_pose import CODE_FRACTION, estimate

LOG = os.path.expanduser('~/qr_delivery/sign_obs.csv')


class QRNode(Node):
    def __init__(self):
        super().__init__('qr_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('sign_m', 0.30)       # edge of the whole printed sign (m)
        self.declare_parameter('focal_px', 530.0)    # camera focal length in pixels (validate_pose.py suggests it)
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.pub = self.create_publisher(String, '/room_sign', 10)
        self.pose = None
        amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, amcl_qos)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.on_image, qos_profile_sensor_data)
        new = not os.path.exists(LOG)
        self.file = open(LOG, 'a', newline='')
        self.writer = csv.writer(self.file)
        if new:
            self.writer.writerow(['time', 'text', 'decoded', 'dist_m', 'angle_deg', 'bearing_deg',
                                  'robot_x', 'robot_y', 'robot_yaw_deg'])
        self.last_log = 0.0
        self.get_logger().info(f'QR node ready, logging sign observations to {LOG}')

    def on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, math.degrees(2 * math.atan2(p.orientation.z, p.orientation.w)))

    def on_image(self, msg):
        self.process(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def process(self, frame):
        text, pts, _ = self.detector.detectAndDecode(frame)
        if pts is None:
            return
        if text:
            self.pub.publish(String(data=text))
        now = time.time()
        if now - self.last_log < 0.2:                # at most 5 log rows per second
            return
        h, w = frame.shape[:2]
        est = estimate(pts, self.get_parameter('sign_m').value * CODE_FRACTION,
                       self.get_parameter('focal_px').value, w / 2, h / 2)
        if est is None:
            return
        self.last_log = now
        dist, angle, bearing = est
        x, y, yaw = self.pose if self.pose else (float('nan'),) * 3
        self.writer.writerow([time.strftime('%H:%M:%S'), text, int(bool(text)), f'{dist:.3f}', f'{angle:.1f}',
                              f'{bearing:.1f}', f'{x:.3f}', f'{y:.3f}', f'{yaw:.1f}'])
        self.file.flush()
        if text:
            self.get_logger().info(f'Read sign {text}: {dist:.2f} m, {angle:.0f} deg off its normal',
                                   throttle_duration_sec=1.0)


def main():
    rclpy.init()
    rclpy.spin(QRNode())


if __name__ == '__main__':
    main()
