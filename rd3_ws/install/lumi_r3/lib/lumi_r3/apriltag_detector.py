#!/usr/bin/env python3
# ──────────────────────────────────────────────────────────────────────────────
# lumi_r3  |  apriltag_detector.py
# MAHE Mobility Challenge 2026 — Round 3
# by Rino | GMIT Mandya
#
# Detects AprilTags 0-4 on arena walls.
# Publishes: /apriltag/detections  (JSON: id, dist, bearing_deg)
#            /apriltag/debug_image (annotated camera frame)
# ──────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import math


# ── Camera intrinsics (MINI R1) ───────────────────────────────────────────────
# hFOV = 1.089 rad, resolution 640x480
FX = FY = 320.0 / math.tan(1.089 / 2.0)   # = 534.6 px
CX, CY   = 320.0, 240.0
CAM_MAT  = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
DIST     = np.zeros((4, 1), dtype=np.float64)

# AprilTag physical size in the SDF (0.15m)
TAG_SIZE = 0.15
MAX_DIST = 3.5


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')
        self.bridge   = CvBridge()
        self.detector = None
        self._init_detector()

        be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Image, '/r1_mini/camera/image_raw', self.cb_image, be)
        self.pub_det = self.create_publisher(String, '/apriltag/detections', 10)
        self.pub_dbg = self.create_publisher(Image,  '/apriltag/debug_image', 10)

        self.get_logger().info(
            f'AprilTagDetector ready  backend={self.detector}  '
            f'fx={FX:.1f}  tag_size={TAG_SIZE}m')

    # ── Detector init ─────────────────────────────────────────────────────────
    def _init_detector(self):
        # Try OpenCV AprilTag dictionary (cv2 >= 4.7)
        for dict_name, dict_id in [
            ('DICT_APRILTAG_36H11', getattr(cv2.aruco, 'DICT_APRILTAG_36H11', None)),
            ('DICT_APRILTAG_16H5',  getattr(cv2.aruco, 'DICT_APRILTAG_16H5',  None)),
            ('DICT_4X4_50',        cv2.aruco.DICT_4X4_50),
        ]:
            if dict_id is None:
                continue
            try:
                d = cv2.aruco.getPredefinedDictionary(dict_id)
                p = cv2.aruco.DetectorParameters()
                self._aruco_det = cv2.aruco.ArucoDetector(d, p)
                self._aruco_dict = d
                self.detector = f'ArucoDetector/{dict_name}'
                self.get_logger().info(f'Using {self.detector}')
                return
            except Exception:
                pass

        # Legacy OpenCV
        try:
            d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36H11)
            p = cv2.aruco.DetectorParameters_create()
            self._aruco_legacy = (d, p)
            self.detector = 'legacy/DICT_APRILTAG_36H11'
            self.get_logger().warn(f'Using legacy detector: {self.detector}')
            return
        except Exception:
            pass

        self.get_logger().error('No ArUco/AprilTag detector available!')

    # ── Image callback ────────────────────────────────────────────────────────
    def cb_image(self, msg):
        if self.detector is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray  = clahe.apply(gray)

        corners, ids = self._detect(gray)
        detections   = []

        if ids is not None:
            for i, tag_id in enumerate(ids.flatten()):
                if tag_id > 4:
                    continue
                try:
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], TAG_SIZE, CAM_MAT, DIST)
                    tv   = tvec[0][0]
                    dist = float(np.linalg.norm(tv))
                    if dist > MAX_DIST:
                        continue
                    bearing = float(math.degrees(math.atan2(tv[0], tv[2])))
                    detections.append({
                        'id':          int(tag_id),
                        'dist':        round(dist, 3),
                        'bearing_deg': round(bearing, 1),
                    })
                    # Annotate
                    cv2.aruco.drawDetectedMarkers(
                        frame, [corners[i]], np.array([[tag_id]]))
                    cv2.drawFrameAxes(frame, CAM_MAT, DIST,
                                      rvec, tv.reshape(3, 1), 0.05)
                    pt = (int(corners[i][0][0][0]),
                          int(corners[i][0][0][1]) - 12)
                    cv2.putText(frame,
                        f'Tag{tag_id}  {dist:.2f}m  {bearing:+.0f}deg',
                        pt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 50), 2)
                except Exception:
                    pass

        if detections:
            m = String()
            m.data = json.dumps({'detections': detections})
            self.pub_det.publish(m)
            self.get_logger().info(
                f'Tags: {[d["id"] for d in detections]}  '
                f'dists={[d["dist"] for d in detections]}')

        try:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except Exception:
            pass

    def _detect(self, gray):
        try:
            if hasattr(self, '_aruco_det'):
                c, ids, _ = self._aruco_det.detectMarkers(gray)
            else:
                d, p = self._aruco_legacy
                c, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
            return c, ids
        except Exception:
            return [], None


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
