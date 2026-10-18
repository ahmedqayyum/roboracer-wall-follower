#!/usr/bin/env python3

################################################################################
#
# AutoDRIVE RoboRacer - LiDAR Wall Follower
#
# Subscribes : /autodrive/roboracer_1/lidar              (sensor_msgs/LaserScan)
#              /autodrive/roboracer_1/odom               (nav_msgs/Odometry, speed only)
# Publishes  : /autodrive/roboracer_1/throttle_command   (std_msgs/Float32)
#              /autodrive/roboracer_1/steering_command   (std_msgs/Float32)
#
# Algorithm (classic F1TENTH wall following, extended for a closed track):
#
#   1. Measure the distance to a wall with two beams: one perpendicular to the
#      car (90 deg) and one swept `theta` degrees toward the front. From those
#      two ranges the wall angle `alpha` and the perpendicular distance `D` are
#      recovered, then projected one lookahead `L` ahead of the car so that the
#      controller reacts before the car actually drifts.
#   2. A PD controller on the projected distance error produces a steering
#      angle. Modes: follow the LEFT wall, the RIGHT wall, or stay CENTERED
#      between both walls (error = projected_left - projected_right).
#      A second term compares how open the front-left and front-right sectors
#      are, so the car turns in before the 90 deg beams can see the corner,
#      and a side-margin term pushes the car off a wall it is about to clip.
#   3. Speed: a target speed is chosen from the forward clearance (so the car
#      can always brake before the wall ahead) and the steering magnitude, and
#      a P controller on odometry speed turns it into throttle, braking with
#      negative throttle when the car is too fast for what is ahead.
#   4. Safety: if the forward cone gets too close the car steers full lock
#      toward the more open side. If the scan goes stale the car stops.
#
################################################################################

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32

################################################################################

# Topics
TOPIC_LIDAR = '/autodrive/roboracer_1/lidar'
TOPIC_ODOM = '/autodrive/roboracer_1/odom'
TOPIC_THROTTLE = '/autodrive/roboracer_1/throttle_command'
TOPIC_STEERING = '/autodrive/roboracer_1/steering_command'
TOPIC_BEST_LAP = '/autodrive/roboracer_1/best_lap_time'
TOPIC_LAST_LAP = '/autodrive/roboracer_1/last_lap_time'
TOPIC_LAP_COUNT = '/autodrive/roboracer_1/lap_count'
TOPIC_COLLISIONS = '/autodrive/roboracer_1/collision_count'

# Vehicle: a steering command of 1.0 is full lock, about 0.52 rad
MAX_STEER_RAD = 0.52


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


################################################################################


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follow_node')

        # ---------------------------------------------------------------- params
        # Behaviour
        self.declare_parameter('mode', 'center')            # 'left' | 'right' | 'center'
        self.declare_parameter('desired_distance', 0.8)     # [m] target wall gap (left/right modes)
        self.declare_parameter('lookahead', 0.6)            # [m] projection distance ahead of the car
        self.declare_parameter('theta_deg', 30.0)           # [deg] second beam offset from the 90 deg beam
        self.declare_parameter('beam_window', 5)            # beams averaged on each side of a target angle

        # Steering PD (output in radians of steering angle)
        self.declare_parameter('kp', 1.0)                   # gain on wall (side) error
        self.declare_parameter('kd', 0.10)
        self.declare_parameter('steer_smoothing', 0.3)      # [0,1] low pass on steering command (0 = off)

        # Corner anticipation: compare how open the front-left and front-right
        # sectors are. Ranges are clipped so distant walls do not dominate.
        self.declare_parameter('kf', 0.6)                   # gain on (front_left - front_right) [rad/m]
        self.declare_parameter('sector_lo_deg', 20.0)       # sector spans [lo, hi] degrees each side of forward
        self.declare_parameter('sector_hi_deg', 70.0)
        self.declare_parameter('sector_clip', 2.5)          # [m] ranges clipped to this inside the sectors

        # Side margin: steer away from a wall closer than this beside the car
        self.declare_parameter('side_margin', 0.35)         # [m] from the LiDAR, car half width is ~0.15
        self.declare_parameter('k_side', 2.0)               # [rad/m] gain on the margin violation

        # Speed control (needs odometry; falls back to throttle scheduling without it)
        self.declare_parameter('v_max', 4.0)                # [m/s] target speed on open straights
        self.declare_parameter('v_corner', 1.5)             # [m/s] target speed at full steering lock
        self.declare_parameter('v_crawl', 0.6)              # [m/s] lowest target, so the car never stalls
        self.declare_parameter('a_brake', 3.0)              # [m/s^2] planned deceleration toward the wall ahead
        self.declare_parameter('stop_margin', 0.4)          # [m] clearance the braking plan keeps in reserve
        self.declare_parameter('k_speed', 0.25)             # throttle per m/s of speed error
        self.declare_parameter('ff_speed', 0.06)            # throttle per m/s of target speed (feedforward)
        self.declare_parameter('brake_max', 0.5)            # largest negative throttle used for braking

        # Throttle
        self.declare_parameter('throttle_max', 0.6)         # cap on forward throttle
        self.declare_parameter('throttle_min', 0.20)        # fallback schedule only (no odometry)
        self.declare_parameter('front_cone_deg', 45.0)      # +/- cone searched for obstacles ahead
        self.declare_parameter('path_half_width', 0.35)     # [m] half width of the car's path (car ~0.3 m wide)
        self.declare_parameter('clear_far', 5.0)            # [m] clearance at which throttle_max is allowed
        self.declare_parameter('clear_near', 1.2)           # [m] clearance at which throttle_min is used
        self.declare_parameter('brake_distance', 0.6)       # [m] below this: emergency manoeuvre
        self.declare_parameter('scan_timeout', 0.5)         # [s] stop if no scan for this long
        self.declare_parameter('debug_period', 1.0)         # [s] telemetry log period (0 = off)

        p = self.get_parameter
        self.mode = str(p('mode').value).lower()
        self.desired_distance = float(p('desired_distance').value)
        self.lookahead = float(p('lookahead').value)
        self.theta = math.radians(float(p('theta_deg').value))
        self.beam_window = int(p('beam_window').value)
        self.kp = float(p('kp').value)
        self.kd = float(p('kd').value)
        self.steer_smoothing = clamp(float(p('steer_smoothing').value), 0.0, 0.99)
        self.kf = float(p('kf').value)
        self.sector_lo = math.radians(float(p('sector_lo_deg').value))
        self.sector_hi = math.radians(float(p('sector_hi_deg').value))
        self.sector_clip = float(p('sector_clip').value)
        self.side_margin = float(p('side_margin').value)
        self.k_side = float(p('k_side').value)
        self.v_max = float(p('v_max').value)
        self.v_corner = float(p('v_corner').value)
        self.v_crawl = float(p('v_crawl').value)
        self.a_brake = float(p('a_brake').value)
        self.stop_margin = float(p('stop_margin').value)
        self.k_speed = float(p('k_speed').value)
        self.ff_speed = float(p('ff_speed').value)
        self.brake_max = float(p('brake_max').value)
        self.throttle_max = float(p('throttle_max').value)
        self.throttle_min = float(p('throttle_min').value)
        self.front_cone = math.radians(float(p('front_cone_deg').value))
        self.path_half_width = float(p('path_half_width').value)
        self.clear_far = float(p('clear_far').value)
        self.clear_near = float(p('clear_near').value)
        self.brake_distance = float(p('brake_distance').value)
        self.scan_timeout = float(p('scan_timeout').value)
        self.debug_period = float(p('debug_period').value)
        self.last_debug_time = None

        if self.mode not in ('left', 'right', 'center'):
            self.get_logger().warn(f"Unknown mode '{self.mode}', falling back to 'center'")
            self.mode = 'center'

        # ----------------------------------------------------------------- state
        self.prev_error = 0.0
        self.prev_time = None
        self.steer_cmd = 0.0
        self.last_scan_time = None
        self.best_lap = None
        self.collisions = None
        self.speed = None

        # ------------------------------------------------------------------- ROS
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_throttle = self.create_publisher(Float32, TOPIC_THROTTLE, qos)
        self.pub_steering = self.create_publisher(Float32, TOPIC_STEERING, qos)
        self.create_subscription(LaserScan, TOPIC_LIDAR, self.on_scan, qos)
        self.create_subscription(Odometry, TOPIC_ODOM, self.on_odom, qos)

        # Telemetry (logging only)
        self.create_subscription(Float32, TOPIC_BEST_LAP, self.on_best_lap, qos)
        self.create_subscription(Float32, TOPIC_LAST_LAP, self.on_last_lap, qos)
        self.create_subscription(Int32, TOPIC_LAP_COUNT, self.on_lap_count, qos)
        self.create_subscription(Int32, TOPIC_COLLISIONS, self.on_collisions, qos)
        self.lap_count = None

        # Watchdog: stop the car if the LiDAR stream disappears
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f"Wall follower up | mode={self.mode} desired={self.desired_distance:.2f}m "
            f"L={self.lookahead:.2f}m kp={self.kp} kd={self.kd} "
            f"throttle=[{self.throttle_min}, {self.throttle_max}]"
        )

    # ------------------------------------------------------------------ helpers

    def range_at(self, ranges, angle_min, angle_inc, angle):
        """Robust range at `angle` [rad] (0 = ahead, +ve = left): mean of a small window."""
        n = len(ranges)
        idx = int(round((angle - angle_min) / angle_inc))
        lo = clamp(idx - self.beam_window, 0, n - 1)
        hi = clamp(idx + self.beam_window, 0, n - 1)
        window = ranges[lo:hi + 1]
        return float(np.mean(window)) if window.size else float(np.max(ranges))

    def projected_wall_distance(self, ranges, angle_min, angle_inc, side):
        """
        Two-beam wall estimate.
          b : beam perpendicular to the car (+-90 deg)
          a : beam `theta` closer to the front
        alpha = atan((a cos(theta) - b) / (a sin(theta)))  -> wall angle w.r.t. car heading
        D     = b cos(alpha)                               -> current perpendicular distance
        Dp    = D + L sin(alpha)                           -> distance a lookahead L ahead
        `side` is +1 for the left wall, -1 for the right wall.
        """
        b = self.range_at(ranges, angle_min, angle_inc, side * math.pi / 2.0)
        a = self.range_at(ranges, angle_min, angle_inc, side * (math.pi / 2.0 - self.theta))
        alpha = math.atan2(a * math.cos(self.theta) - b, a * math.sin(self.theta))
        d_now = b * math.cos(alpha)
        d_proj = d_now + self.lookahead * math.sin(alpha)
        return d_proj, d_now, alpha

    def sector_openness(self, ranges, angle_min, angle_inc, side):
        """Mean clipped range over the forward sector [lo, hi] on `side` (+1 left, -1 right)."""
        n = len(ranges)
        a0 = side * self.sector_lo
        a1 = side * self.sector_hi
        i0 = clamp(int(round((min(a0, a1) - angle_min) / angle_inc)), 0, n - 1)
        i1 = clamp(int(round((max(a0, a1) - angle_min) / angle_inc)), 0, n - 1)
        sector = np.minimum(ranges[i0:i1 + 1], self.sector_clip)
        return float(np.mean(sector)) if sector.size else self.sector_clip

    def side_min(self, ranges, angle_min, angle_inc, side):
        """Closest return beside the car, 50-135 deg on `side` (+1 left, -1 right)."""
        n = len(ranges)
        a0 = side * math.radians(50.0)
        a1 = side * math.radians(135.0)
        i0 = clamp(int(round((min(a0, a1) - angle_min) / angle_inc)), 0, n - 1)
        i1 = clamp(int(round((max(a0, a1) - angle_min) / angle_inc)), 0, n - 1)
        return float(np.min(ranges[i0:i1 + 1]))

    def front_clearance(self, ranges, angle_min, angle_inc, rmax):
        """
        Forward distance to the nearest obstacle that is actually in the car's path.
        Beams inside the front cone are kept only if their hit point lies within
        `path_half_width` laterally; the clearance is the forward component r*cos(a).
        Side walls of a corridor therefore do not count as obstacles ahead.
        """
        n = len(ranges)
        lo = clamp(int(round((-self.front_cone - angle_min) / angle_inc)), 0, n - 1)
        hi = clamp(int(round((self.front_cone - angle_min) / angle_inc)), 0, n - 1)
        r = ranges[lo:hi + 1]
        a = angle_min + np.arange(lo, hi + 1) * angle_inc
        forward = r * np.cos(a)
        lateral = np.abs(r * np.sin(a))
        in_path = (lateral <= self.path_half_width) & (forward > 0.0)
        return float(np.min(forward[in_path])) if np.any(in_path) else float(rmax)

    # ---------------------------------------------------------------- callbacks

    def on_scan(self, scan: LaserScan):
        now = self.get_clock().now()
        self.last_scan_time = now

        ranges = np.asarray(scan.ranges, dtype=float)
        if ranges.size == 0:
            return
        # Sanitize: inf / nan / out-of-range -> treat as "nothing seen" (range_max)
        rmax = scan.range_max if scan.range_max > 0.0 else 10.0
        rmin = scan.range_min if scan.range_min > 0.0 else 0.0
        bad = ~np.isfinite(ranges) | (ranges > rmax) | (ranges < rmin)
        ranges[bad] = rmax

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment if scan.angle_increment > 0.0 else (
            (scan.angle_max - scan.angle_min) / max(1, ranges.size - 1))

        # -------- lateral error (positive error => steer left)
        left_proj, left_now, left_alpha = self.projected_wall_distance(ranges, angle_min, angle_inc, +1)
        right_proj, right_now, right_alpha = self.projected_wall_distance(ranges, angle_min, angle_inc, -1)

        if self.mode == 'left':
            # Too far from the left wall -> positive error -> turn left
            error = left_proj - self.desired_distance
        elif self.mode == 'right':
            # Too far from the right wall -> negative error -> turn right
            error = self.desired_distance - right_proj
        else:
            # Stay centered: more room on the left than on the right -> turn left
            error = 0.5 * (left_proj - right_proj)

        # -------- PD steering
        dt = 0.0
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
        d_error = (error - self.prev_error) / dt if dt > 1e-4 else 0.0
        self.prev_error = error
        self.prev_time = now

        # -------- corner anticipation: steer toward the more open forward sector
        open_left = self.sector_openness(ranges, angle_min, angle_inc, +1)
        open_right = self.sector_openness(ranges, angle_min, angle_inc, -1)
        front_error = open_left - open_right

        steer_rad = self.kp * error + self.kd * d_error + self.kf * front_error

        # -------- side margin: push off a wall the car is about to clip (inside of corners)
        side_left = self.side_min(ranges, angle_min, angle_inc, +1)
        side_right = self.side_min(ranges, angle_min, angle_inc, -1)
        steer_rad += self.k_side * (max(0.0, self.side_margin - side_right) -
                                    max(0.0, self.side_margin - side_left))

        # -------- forward safety
        clearance = self.front_clearance(ranges, angle_min, angle_inc, rmax)
        emergency = clearance < self.brake_distance
        if emergency:
            # Turn hard toward the more open side, regardless of the wall error
            open_side = 1.0 if front_error >= 0.0 else -1.0
            steer_rad = open_side * MAX_STEER_RAD

        steer_cmd = clamp(steer_rad / MAX_STEER_RAD, -1.0, 1.0)
        if not emergency and self.steer_smoothing > 0.0:
            steer_cmd = self.steer_smoothing * self.steer_cmd + (1.0 - self.steer_smoothing) * steer_cmd
        self.steer_cmd = steer_cmd

        # -------- speed
        v_target = None
        if self.speed is not None:
            # Fast enough to still stop `stop_margin` short of the wall ahead,
            # and slower the harder the car is steering (or about to steer).
            v_brake = math.sqrt(2.0 * self.a_brake * max(0.0, clearance - self.stop_margin))
            turn = max(abs(steer_cmd), clamp(abs(front_error) / max(1e-3, self.sector_clip), 0.0, 1.0))
            v_turn = self.v_max - (self.v_max - self.v_corner) * turn
            v_target = max(self.v_crawl, min(self.v_max, v_brake, v_turn))
            throttle = self.ff_speed * v_target + self.k_speed * (v_target - self.speed)
            throttle = clamp(throttle, -self.brake_max, self.throttle_max)
        else:
            # No odometry: scale throttle by clearance and steering magnitude
            clear_frac = clamp((clearance - self.clear_near) / max(1e-3, self.clear_far - self.clear_near), 0.0, 1.0)
            steer_frac = 1.0 - abs(steer_cmd)
            speed_frac = min(clear_frac, steer_frac ** 2)
            throttle = self.throttle_min + (self.throttle_max - self.throttle_min) * speed_frac
            if emergency:
                throttle = min(throttle, 0.5 * self.throttle_min)

        self.publish(throttle, steer_cmd)

        # -------- telemetry
        if self.debug_period > 0.0 and (
                self.last_debug_time is None or
                (now - self.last_debug_time).nanoseconds * 1e-9 >= self.debug_period):
            self.last_debug_time = now
            self.get_logger().info(
                f"L={left_now:.2f}m({left_proj:.2f}) R={right_now:.2f}m({right_proj:.2f}) "
                f"front={clearance:.2f}m open={open_left:.2f}/{open_right:.2f} "
                f"side={side_left:.2f}/{side_right:.2f} "
                f"err={error:+.2f} ferr={front_error:+.2f} steer={steer_cmd:+.2f} "
                f"v={-1.0 if self.speed is None else self.speed:.2f}"
                f"/{-1.0 if v_target is None else v_target:.2f} "
                f"thr={throttle:+.2f}{' EMERGENCY' if emergency else ''}"
            )

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def watchdog(self):
        if self.last_scan_time is None:
            return
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        if age > self.scan_timeout:
            self.publish(0.0, 0.0)

    def on_best_lap(self, msg: Float32):
        if msg.data > 0.0 and (self.best_lap is None or abs(msg.data - self.best_lap) > 1e-3):
            self.best_lap = msg.data
            self.get_logger().info(f"Best lap: {msg.data:.3f} s")

    def on_last_lap(self, msg: Float32):
        pass  # kept for completeness; best lap + lap count give enough feedback

    def on_lap_count(self, msg: Int32):
        if self.lap_count is None or msg.data != self.lap_count:
            self.lap_count = msg.data
            if msg.data > 0:
                self.get_logger().info(f"Lap count: {msg.data}")

    def on_collisions(self, msg: Int32):
        if self.collisions is None:
            self.collisions = msg.data
        elif msg.data != self.collisions:
            self.collisions = msg.data
            self.get_logger().warn(f"Collision count: {msg.data}")

    # ------------------------------------------------------------------- output

    def publish(self, throttle, steering):
        t = Float32()
        s = Float32()
        t.data = float(clamp(throttle, -1.0, 1.0))
        s.data = float(clamp(steering, -1.0, 1.0))
        self.pub_throttle.publish(t)
        self.pub_steering.publish(s)


################################################################################


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the car stopped
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
