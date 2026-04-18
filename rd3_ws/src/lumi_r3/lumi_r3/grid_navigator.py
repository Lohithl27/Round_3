#!/usr/bin/env python3
# ──────────────────────────────────────────────────────────────────────────────
# lumi_r3  |  grid_navigator.py  (FIXED)
# MAHE Mobility Challenge 2026 — Round 3
# by Rino | GMIT Mandya
#
# FIXES:
#   - Robot now spawns at GREEN tile (r4,c0) = (-1.35,-1.80) facing NORTH
#   - Updated TAG_ACTIONS per user spec
#   - Corrected waypoint path through the maze
#   - No Nav2 dependency — pure waypoint + reactive navigation
#
# HARDCODED AprilTag behaviours:
#   SDF tag2 → follow green
#   SDF tag4 → follow orange
#   Others retain turn behaviors.
# ──────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math, json, time
import numpy as np
from enum import Enum


# ── ARENA CONSTANTS ──────────────────────────────────────────────────────────
TILE = 0.90   # metres per tile

def tile_xy(row, col):
    return (-1.35 + col * TILE,  1.80 - row * TILE)

def xy_to_tile(x, y):
    col = int(round((x + 1.35) / TILE))
    row = int(round((1.80 - y)  / TILE))
    return (max(0, min(4, row)), max(0, min(3, col)))


# ── HARDCODED AprilTag → ACTION  ──────────────────────────────────────────────
# SDF tag ID  →  (label,  action)
# *** Verified against user instructions ***
TAG_ACTIONS = {
    4: ('Tag4', 'follow_orange'),  # Tag 4 → follow orange
    1: ('Tag1', 'turn_left'),      # Tag 1 → take left
    2: ('Tag2', 'follow_green'),   # Tag 2 → follow green
    0: ('Tag0', 'u_turn'),         # Tag 0 → u-turn
    3: ('Tag3', 'turn_right'),     # Tag 3 → take right
}

# ── Safety distances (metres) ─────────────────────────────────────────────────
SAFE_STOP_DIST     = 0.22
SAFE_SLOW_DIST     = 0.40
AVOID_TRIGGER_DIST = 0.40
MIN_SAFE_SPEED     = 0.05
SIDE_SECTOR_WEIGHT = 0.70


# ── NAVIGATION WAYPOINTS ──────────────────────────────────────────────────────
# Complete maze path derived from arena map + wall analysis.
# Arena inner walls:
#   wall_A: x=-0.9,  y=-2.25→+0.45  (blocks c0↔c1 below y=0.45)
#   wall_C: y=+1.35, x=-1.8→+0.9    (blocks r0↔r1 for c0,c1,c2)
#   wall_D: x=0.0,   y=-0.45→+1.35  (blocks c1↔c2 for r1,r2)
#   wall_E: y=-0.45, x=0.0→+0.9     (blocks r2↔r3 at c1,c2)
#   wall_F: y=-1.35, x=0.0→+1.8     (blocks r3↔r4 at c2,c3)
#   obstacle: cylinder at (0.9, 0.45)
#
WAYPOINTS = [
    # ── Phase 1: NORTH up left column ────────────────────────────────────────
    # wall_A blocks c0→c1 until y=+0.45 (between r1 and r2)
    (4, 0, 'GREEN_START'),
    (3, 0, 'north_c0'),
    (2, 0, 'north_c0'),
    (1, 0, 'north_c0_exit'),  # y=0.90 > 0.45, wall_A ended, can go east

    # ── Phase 2: EAST → find Tag2 (SDF tag4 on wall_C, visible at r1_c1) ────
    (1, 1, 'east_find_tag2'), # tag4 on wall_C visible here → fires → TURN RIGHT

    # ── Phase 3: RIGHT = SOUTH after Tag2 action ─────────────────────────────
    (2, 1, 'south_c1'),
    (3, 1, 'south_c1_bot'),   # wall_D ends at y=-0.45; at r3(y=-0.9) can go east

    # ── Phase 4: EAST along r3 → find Tag1 (SDF tag1 on wall_E) ─────────────
    # wall_E at y=-0.45, x=0→0.9 blocks north movement at c2
    # But tag1 at (0.45,-0.441) is visible from r3 looking north
    (3, 2, 'east_find_tag1'), # tag1 on wall_E visible here → fires → TURN LEFT

    # ── Phase 5: LEFT = NORTH, but wall_E blocks at c2 → go to c3 first ─────
    (3, 3, 'east_avoid_wallE'), # c3 at x=1.35 > 0.9, no wall_E blocking north

    # ── Phase 6: NORTH up right column → find Tag3 (SDF tag2 on wall_F) ─────
    # Also Tag4 (SDF tag0) on right outer wall visible here
    (2, 3, 'north_c3'),
    (1, 3, 'north_c3_tags'),  # SDF tag0 on right wall fires → U-TURN

    # ── Phase 7: U-TURN → now facing SOUTH, retrace c3 ──────────────────────
    (0, 3, 'top_c3'),         # go to top before turning back
    (1, 3, 'uturn_south'),
    (2, 3, 'south_c3'),
    (3, 3, 'south_c3'),

    # ── Phase 8: Navigate to c1 column to cross wall_F ───────────────────────
    # wall_F blocks c2,c3 from r3→r4 (x=0 to 1.8)
    # c1 at x=-0.45 < 0 → wall_F does NOT block c1
    (3, 2, 'west_r3'),
    (3, 1, 'west_r3_c1'),
    (4, 1, 'drop_to_r4'),     # cross to bottom row via c1 (free of wall_F)

    # ── Phase 9: Find Tag5 (SDF tag3 on right outer wall, near STOP) ─────────
    # SDF tag3 at (1.796,-1.875) visible from bottom row looking right
    # Fires → FOLLOW ORANGE → robot goes east

    # ── Phase 10: EAST along bottom row → RED STOP ───────────────────────────
    (4, 2, 'east_bottom'),
    (4, 3, 'RED_STOP'),
]


class Mode(Enum):
    WAIT         = 'WAITING'
    WAYPOINTS    = 'FOLLOW_WAYPOINTS'
    TURN         = 'TURNING'
    FOLLOW_GREEN = 'FOLLOW_GREEN'
    FOLLOW_ORANGE= 'FOLLOW_ORANGE'
    DONE         = 'DONE'


class GridNavigator(Node):

    def __init__(self):
        super().__init__('grid_navigator')

        # Pose — initialised to START position
        self.rx  = -1.35
        self.ry  = -1.80
        self.ryaw = 0.0             # default facing EAST; navigator steers to each wp
        self.odom_ok = False

        # LiDAR for obstacle avoidance
        self.sec = {k: 5.0 for k in ('F','FL','FR','L','R','B')}
        self.lidar_ok = False

        # Navigation state
        self.wp_idx  = 0
        self.mode    = Mode.WAIT
        self.started = False
        self.done    = False
        self.start_t = None

        # AprilTag state
        self.visited_tags = set()
        self.tag_log      = []

        # Turn state
        self.target_yaw   = 0.0
        self.turn_start_t = 0.0

        # Colour from tile_detector
        self.floor_colour = None

        # Stuck detection
        self.stuck_t = time.time()
        self.stuck_x = self.rx
        self.stuck_y = self.ry
        self.avoid_dir = 0  # lock turn direction during close-obstacle avoidance

        # Publishers
        self.vel      = self.create_publisher(Twist,  '/cmd_vel',           10)
        self.stat     = self.create_publisher(String, '/mission_status',    10)
        self.tile_pub = self.create_publisher(String, '/grid/current_tile', 10)

        # Subscribers
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Odometry,   '/r1_mini/odom',        self.cb_odom,   10)
        self.create_subscription(LaserScan,  '/r1_mini/lidar',       self.cb_lidar,  be)
        self.create_subscription(String,     '/apriltag/detections', self.cb_tag,    10)
        self.create_subscription(String,     '/tile/colour',         self.cb_colour, 10)

        self.create_timer(0.10, self.tick)
        self.create_timer(2.0,  self.pub_status)
        self.create_timer(5.0,  self._start_once)

        self.get_logger().info(
            'GridNavigator ready (FIXED)\n'
            '  Spawn : (-1.35, -1.80) = GREEN tile (bottom-left)\n'
            '  Stop  : (+1.35, -1.80) = RED tile\n'
            f'  Waypts: {len(WAYPOINTS)}\n'
            '  Tags  : 2→green  4→orange  1→left  0→uturn  3→right\n'
            '  Auto-start: 5s')
        if SAFE_SLOW_DIST <= SAFE_STOP_DIST:
            self.get_logger().warn(
                f'Invalid safety thresholds: SAFE_SLOW_DIST ({SAFE_SLOW_DIST}) '
                f'must be greater than SAFE_STOP_DIST ({SAFE_STOP_DIST})')

    # ── Startup ───────────────────────────────────────────────────────────────
    def _start_once(self):
        if not self.started:
            self.started = True
            self.start_t = time.time()
            self.mode    = Mode.WAYPOINTS
            self.get_logger().info('MISSION START — following waypoints')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def cb_odom(self, msg):
        self.rx   = msg.pose.pose.position.x
        self.ry   = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.ryaw = math.atan2(2*(q.w*q.z+q.x*q.y),
                               1-2*(q.y*q.y+q.z*q.z))
        self.odom_ok = True

    def cb_lidar(self, msg):
        n = len(msg.ranges)
        if n < 10:
            return
        r = np.array(msg.ranges, dtype=np.float32)
        r = np.where(np.isfinite(r) & (r > msg.range_min), r, 5.0)
        hw = 16

        def s(c):
            return float(np.min(r[max(0, c-hw):min(n-1, c+hw)+1]))

        fc = n//2; lc = 3*n//4; rc = n//4
        self.sec['F']  = s(fc)
        self.sec['FL'] = s((fc+lc)//2)
        self.sec['FR'] = s((fc+rc)//2)
        self.sec['L']  = s(lc)
        self.sec['R']  = s(rc)
        bk = list(range(0, hw+1)) + list(range(n-hw, n))
        self.sec['B']  = float(np.min(r[bk]))
        self.lidar_ok  = True

    def cb_tag(self, msg):
        if not self.started or self.done:
            return
        try:
            data = json.loads(msg.data)
            for det in data.get('detections', []):
                tid = det['id']
                if tid in self.visited_tags or tid not in TAG_ACTIONS:
                    continue
                self.visited_tags.add(tid)
                label, action = TAG_ACTIONS[tid]

                entry = {
                    'tag_id':    tid,
                    'label':     label,
                    'action':    action,
                    'dist_m':    det['dist'],
                    'bearing':   det['bearing_deg'],
                    'timestamp': round(time.time(), 3),
                    'pos':       [round(self.rx,3), round(self.ry,3)],
                    'tile':      list(xy_to_tile(self.rx, self.ry)),
                    'visit_no':  len(self.visited_tags),
                }
                self.tag_log.append(entry)

                self.get_logger().info(
                    f'★ TAG sdf={tid} ({label})  action={action}  '
                    f'{det["dist"]:.2f}m  visit={len(self.visited_tags)}/5')

                self._do_action(action)
        except Exception:
            pass

    def cb_colour(self, msg):
        try:
            self.floor_colour = json.loads(msg.data).get('colour')
        except Exception:
            pass

    # ── Action execution ──────────────────────────────────────────────────────
    def _do_action(self, action):
        if action == 'turn_left':
            self.target_yaw   = self._norm(self.ryaw + math.pi/2)
            self.turn_start_t = time.time()
            self.mode = Mode.TURN
            self.get_logger().info(
                f'  → TURN LEFT  to {math.degrees(self.target_yaw):.0f}°')

        elif action == 'turn_right':
            self.target_yaw   = self._norm(self.ryaw - math.pi/2)
            self.turn_start_t = time.time()
            self.mode = Mode.TURN
            self.get_logger().info(
                f'  → TURN RIGHT  to {math.degrees(self.target_yaw):.0f}°')

        elif action == 'u_turn':
            self.target_yaw   = self._norm(self.ryaw + math.pi)
            self.turn_start_t = time.time()
            self.mode = Mode.TURN
            self.get_logger().info(
                f'  → U-TURN  to {math.degrees(self.target_yaw):.0f}°')

        elif action == 'follow_green':
            self.mode = Mode.FOLLOW_GREEN
            self.get_logger().info('  → FOLLOW GREEN tiles')

        elif action == 'follow_orange':
            self.mode = Mode.FOLLOW_ORANGE
            self.get_logger().info('  → FOLLOW ORANGE tiles')

    # ── Main 10 Hz tick ───────────────────────────────────────────────────────
    def tick(self):
        if not self.odom_ok or not self.started:
            self.stop()
            return

        # Publish current tile
        row, col = xy_to_tile(self.rx, self.ry)
        tm = String()
        tm.data = json.dumps({'row': row, 'col': col})
        self.tile_pub.publish(tm)

        # Reached RED STOP tile?
        if row == 4 and col == 3 and self.mode not in (Mode.WAIT, Mode.DONE):
            self._mission_complete()
            return

        self._check_stuck()

        {
            Mode.WAYPOINTS:     self._follow_waypoints,
            Mode.TURN:          self._do_turn,
            Mode.FOLLOW_GREEN:  lambda: self._follow_colour('green'),
            Mode.FOLLOW_ORANGE: lambda: self._follow_colour('orange'),
            Mode.DONE:          self.stop,
        }.get(self.mode, self.stop)()

        if self.mode == Mode.WAYPOINTS:
            self._check_zone_entry(row, col)

    def _check_zone_entry(self, row, col):
        """Advance waypoint index if robot has reached current target tile."""
        pass  # waypoint logic handles this in _follow_waypoints

    # ── Waypoint follower ─────────────────────────────────────────────────────
    def _follow_waypoints(self):
        if self.wp_idx >= len(WAYPOINTS):
            self._mission_complete()
            return

        row, col, note = WAYPOINTS[self.wp_idx]
        tx, ty = tile_xy(row, col)
        dx, dy = tx - self.rx, ty - self.ry
        dist   = math.hypot(dx, dy)

        if dist < 0.20:
            self.get_logger().info(
                f'  WP {self.wp_idx+1}/{len(WAYPOINTS)}  ({row},{col}) "{note}" ✓')
            self.wp_idx += 1
            return

        # Obstacle avoidance override
        eff = self._front_clearance()
        if eff < AVOID_TRIGGER_DIST and self.lidar_ok:
            cmd = Twist()
            cmd.angular.z = 0.6 * self._avoid_turn_dir()
            self.vel.publish(cmd)
            return

        # Steer toward waypoint
        target_yaw = math.atan2(dy, dx)
        yaw_err    = self._norm(target_yaw - self.ryaw)
        cmd = Twist()
        cmd.angular.z = max(-1.8, min(1.8, yaw_err * 2.0))

        if abs(yaw_err) < 0.20:
            cmd.linear.x = min(0.25, dist * 0.6)
        elif abs(yaw_err) < 0.60:
            cmd.linear.x = 0.08
        # else: rotate in place
        cmd.linear.x = self._safe_forward_speed(cmd.linear.x, eff)

        self.vel.publish(cmd)

    # ── Turn execution ────────────────────────────────────────────────────────
    def _do_turn(self):
        err = self._norm(self.target_yaw - self.ryaw)
        cmd = Twist()

        # Safety timeout
        if time.time() - self.turn_start_t > 8.0:
            self.get_logger().warn('Turn timeout — resuming waypoints')
            self.mode = Mode.WAYPOINTS
            self.stop()
            return

        if abs(err) < 0.06:
            self.get_logger().info(
                f'  Turn done  yaw={math.degrees(self.ryaw):.0f}°')
            self.mode = Mode.WAYPOINTS
            self.stop()
            return

        speed = 0.85 if abs(err) > 0.5 else 0.40
        cmd.angular.z = math.copysign(speed, err)
        self.vel.publish(cmd)

    # ── Colour following ──────────────────────────────────────────────────────
    def _follow_colour(self, target):
        col = self.floor_colour

        if col == 'red':
            self._mission_complete()
            return

        # Obstacle check
        eff = self._front_clearance()
        if eff < AVOID_TRIGGER_DIST and self.lidar_ok:
            cmd = Twist()
            cmd.angular.z = 0.5 * self._avoid_turn_dir()
            self.vel.publish(cmd)
            return

        if col == target:
            cmd = Twist()
            cmd.linear.x = self._safe_forward_speed(0.20, eff)
            # Small corrections to stay centered on colour patch
            if self.sec['L'] < 0.40: cmd.angular.z = -0.20
            elif self.sec['R'] < 0.40: cmd.angular.z = 0.20
            self.vel.publish(cmd)
        else:
            # Colour not yet under camera → use waypoints as fallback
            self._follow_waypoints()

    def _front_clearance(self):
        # Use average diagonal clearance so one near side wall doesn't falsely
        # block forward movement and cause start-tile oscillation.
        side_avg = (self.sec['FL'] + self.sec['FR']) * 0.5
        return min(
            self.sec['F'],
            side_avg * SIDE_SECTOR_WEIGHT
        )

    def _avoid_turn_dir(self):
        # Keep a stable turn direction while obstacle is close to prevent
        # left-right vibration caused by noisy L/R sector comparisons.
        if self._front_clearance() >= SAFE_SLOW_DIST:
            self.avoid_dir = 0
        if self.avoid_dir == 0:
            self.avoid_dir = 1 if self.sec['L'] >= self.sec['R'] else -1
        return self.avoid_dir

    def _safe_forward_speed(self, requested, front_clearance=None):
        if requested <= 0.0 or not self.lidar_ok:
            return requested
        if front_clearance is None:
            front_clearance = self._front_clearance()
        if front_clearance < SAFE_STOP_DIST:
            return 0.0
        if front_clearance < SAFE_SLOW_DIST:
            span = SAFE_SLOW_DIST - SAFE_STOP_DIST
            if span <= 0.0:
                return 0.0
            scale = (front_clearance - SAFE_STOP_DIST) / span
            return max(MIN_SAFE_SPEED, requested * scale)
        return requested

    # ── Mission complete ──────────────────────────────────────────────────────
    def _mission_complete(self):
        if self.done:
            return
        self.done = True
        self.mode = Mode.DONE
        self.stop()

        t = round(time.time() - self.start_t, 1) if self.start_t else 0
        bar = '═' * 55
        self.get_logger().info(bar)
        self.get_logger().info('MISSION COMPLETE  —  RED STOP REACHED')
        self.get_logger().info(f'  Time      : {t}s  ({t/60:.1f} min)')
        self.get_logger().info(
            f'  Tags found: {sorted(self.visited_tags)} ({len(self.visited_tags)}/5)')
        self.get_logger().info('')
        self.get_logger().info('  ── Scoring log (AprilTag) ───────────────────')
        for e in self.tag_log:
            self.get_logger().info(
                f'    #{e["visit_no"]} SDF={e["tag_id"]} ({e["label"]:<22}) '
                f'action={e["action"]:<14} '
                f'{e["dist_m"]:.2f}m  t={e["timestamp"]}')
        self.get_logger().info(bar)

    # ── Stuck detection ───────────────────────────────────────────────────────
    def _check_stuck(self):
        if self.mode in (Mode.TURN, Mode.WAIT, Mode.DONE):
            self._reset_stuck()
            return
        moved = math.hypot(self.rx-self.stuck_x, self.ry-self.stuck_y)
        if moved > 0.07:
            self._reset_stuck()
        elif time.time() - self.stuck_t > 7.0:
            self.get_logger().warn(
                f'STUCK at ({self.rx:.2f},{self.ry:.2f}) '
                f'F={self.sec["F"]:.2f} — reversing')
            self._reset_stuck()
            cmd = Twist()
            cmd.linear.x  = -0.18
            cmd.angular.z =  0.40
            self.vel.publish(cmd)
            time.sleep(0.6)
            self.stop()

    def _reset_stuck(self):
        self.stuck_t = time.time()
        self.stuck_x = self.rx
        self.stuck_y = self.ry

    def stop(self): self.vel.publish(Twist())

    def _norm(self, a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def pub_status(self):
        t = round(time.time()-self.start_t, 1) if self.start_t else 0
        row, col = xy_to_tile(self.rx, self.ry)
        msg = String()
        msg.data = json.dumps({
            'state':        self.mode.value,
            'wp':           f'{self.wp_idx}/{len(WAYPOINTS)}',
            'current_tile': [row, col],
            'next_wp':      list(WAYPOINTS[self.wp_idx][:2]) if self.wp_idx < len(WAYPOINTS) else None,
            'tags_found':   sorted(self.visited_tags),
            'pos':          [round(self.rx,3), round(self.ry,3)],
            'yaw_deg':      round(math.degrees(self.ryaw), 1),
            'floor_colour': self.floor_colour,
            'F_lidar':      round(self.sec['F'],2),
            'elapsed_s':    t,
        })
        self.stat.publish(msg)
        self.get_logger().info(
            f'[{self.mode.value}]  wp={self.wp_idx}/{len(WAYPOINTS)}  '
            f'tile=({row},{col})  tags={len(self.visited_tags)}/5  '
            f'colour={self.floor_colour}  F={self.sec["F"]:.2f}  t={t}s')


def main(args=None):
    rclpy.init(args=args)
    node = GridNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
