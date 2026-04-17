#!/usr/bin/env python3
# ──────────────────────────────────────────────────────────────────────────────
# lumi_r3  |  tile_detector.py
# Detects floor tile colours from camera bottom ROI.
# Logs green + orange tiles with timestamps for scoring.
# Publishes: /tile/colour  /tile/log  /tile/debug_image
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
import time


# ── HSV colour ranges ─────────────────────────────────────────────────────────
# Tuned for Gazebo Fortress ambient=1,1,1,1 (bright lighting)
GREEN_LO  = np.array([ 35,  50,  50])
GREEN_HI  = np.array([ 90, 255, 255])

ORANGE_LO = np.array([  5, 120,  80])
ORANGE_HI = np.array([ 25, 255, 255])

RED_LO1   = np.array([  0, 100, 100])
RED_HI1   = np.array([ 10, 255, 255])
RED_LO2   = np.array([170, 100, 100])
RED_HI2   = np.array([180, 255, 255])

# Minimum pixel area to accept detection
MIN_PX = 400

# Floor ROI = bottom 40% of camera frame (floor in front of robot)
ROI_START = 0.60


class TileDetector(Node):

    def __init__(self):
        super().__init__('tile_detector')
        self.bridge = CvBridge()

        self.green_log  = []   # (row, col, timestamp) for scoring
        self.orange_log = []
        self.all_log    = []   # every tile entered

        self.curr_tile   = (-1, -1)    # set by grid_navigator
        self.total_tiles = 0

        be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Image,  '/r1_mini/camera/image_raw',
                                 self.cb_image, be)
        self.create_subscription(String, '/grid/current_tile',
                                 self.cb_tile, 10)

        self.pub_col = self.create_publisher(String, '/tile/colour',      10)
        self.pub_log = self.create_publisher(String, '/tile/log',         10)
        self.pub_dbg = self.create_publisher(Image,  '/tile/debug_image', 10)

        self.get_logger().info('TileDetector ready')

    def cb_tile(self, msg):
        try:
            d = json.loads(msg.data)
            new_tile = (d['row'], d['col'])
            if new_tile != self.curr_tile:
                self.curr_tile = new_tile
                self.total_tiles += 1
                entry = {
                    'tile':      list(new_tile),
                    'timestamp': round(time.time(), 3),
                    'count':     self.total_tiles,
                }
                self.all_log.append(entry)
                self.get_logger().info(
                    f'[TILE] entered ({new_tile[0]},{new_tile[1]})  '
                    f'total={self.total_tiles}')
        except Exception:
            pass

    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        h, w   = frame.shape[:2]
        roi    = frame[int(h * ROI_START):, :]
        hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dbg    = roi.copy()

        # ── Detect colours ────────────────────────────────────────────────────
        green_px  = cv2.countNonZero(
            cv2.inRange(hsv, GREEN_LO,  GREEN_HI))
        orange_px = cv2.countNonZero(
            cv2.inRange(hsv, ORANGE_LO, ORANGE_HI))
        red_px    = cv2.countNonZero(cv2.bitwise_or(
            cv2.inRange(hsv, RED_LO1, RED_HI1),
            cv2.inRange(hsv, RED_LO2, RED_HI2)))

        colour = None
        if   green_px  > MIN_PX: colour = 'green'
        elif orange_px > MIN_PX: colour = 'orange'
        elif red_px    > MIN_PX: colour = 'red'

        # ── Scoring log ───────────────────────────────────────────────────────
        if colour in ('green', 'orange') and self.curr_tile[0] >= 0:
            tile_key = tuple(self.curr_tile)
            logged   = [e['tile'] for e in
                        (self.green_log if colour=='green' else self.orange_log)]
            if list(tile_key) not in logged:
                entry = {
                    'tile':      list(tile_key),
                    'colour':    colour,
                    'timestamp': round(time.time(), 3),
                }
                if colour == 'green':
                    self.green_log.append(entry)
                else:
                    self.orange_log.append(entry)
                self.get_logger().info(
                    f'[LOG] {colour.upper()} tile {tile_key}  '
                    f'(green={len(self.green_log)}  orange={len(self.orange_log)})')
                log_msg = String()
                log_msg.data = json.dumps(entry)
                self.pub_log.publish(log_msg)

        # ── Publish colour ────────────────────────────────────────────────────
        col_msg = String()
        col_msg.data = json.dumps({
            'colour':         colour,
            'green_px':       green_px,
            'orange_px':      orange_px,
            'red_px':         red_px,
            'green_logged':   len(self.green_log),
            'orange_logged':  len(self.orange_log),
            'total_tiles':    self.total_tiles,
            'current_tile':   list(self.curr_tile),
        })
        self.pub_col.publish(col_msg)

        # ── Debug image ───────────────────────────────────────────────────────
        cv2.putText(dbg, f'{colour or "none"}  G={green_px} O={orange_px}',
                    (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if colour=='green'
                    else (0, 165, 255) if colour=='orange'
                    else (200, 200, 200), 2)
        frame[int(h * ROI_START):, :] = dbg
        cv2.line(frame, (0, int(h*ROI_START)), (w, int(h*ROI_START)),
                 (0, 255, 255), 2)
        cv2.putText(frame,
            f'tile={self.curr_tile}  total={self.total_tiles}',
            (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        try:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TileDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
