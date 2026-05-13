#!/usr/bin/env python3
"""
Pure-pursuit path follower with dynamic obstacle avoidance for Puzzlebot.

Subscribes:
  /path       (nav_msgs/Path)            — planned path from A* planner
  /slam_pose  (geometry_msgs/PoseStamped) — robot pose in map frame
  /scan       (sensor_msgs/LaserScan)    — LiDAR for emergency stop / slowdown

Publishes:
  /cmd_vel  (geometry_msgs/Twist) — velocity command (converted to TwistStamped
                                    by the existing twist_relay node)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan


def _yaw_from_pose(pose):
    q = pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower')

        # Tunable parameters
        self.declare_parameter('lookahead_distance',  0.40)  # m
        self.declare_parameter('linear_speed',         0.20)  # m/s
        self.declare_parameter('angular_gain',         2.00)
        self.declare_parameter('max_angular_speed',    1.80)  # rad/s
        self.declare_parameter('goal_tolerance',       0.15)  # m
        self.declare_parameter('wp_skip_dist',         0.10)  # m — advance waypoint
        self.declare_parameter('obstacle_stop_dist',   0.35)  # m — full stop
        self.declare_parameter('obstacle_slow_dist',   0.60)  # m — start slowing
        self.declare_parameter('obstacle_cone_deg',   40.0)   # half-angle of front cone
        self.declare_parameter('initial_spin_duration', 9.0)  # s; 0 to disable
        self.declare_parameter('initial_spin_speed',    0.70)  # rad/s (~360° in 9s)

        self._path: Path | None = None
        self._robot_pose: PoseStamped | None = None
        self._current_wp: int = 0

        # Obstacle state (set by scan callback, read by control loop)
        self._emergency_stop: bool = False
        self._slow_factor:    float = 1.0

        # Initial spin — starts when first slam_pose arrives (SLAM + controllers ready)
        self._spinning: bool = False
        self._spin_start_time = None

        self.create_subscription(Path,        '/path',      self._path_cb,  10)
        self.create_subscription(PoseStamped, '/slam_pose', self._pose_cb,  10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 20 Hz control loop
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            'Path follower ready — waiting for /path and /slam_pose')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _path_cb(self, msg: Path):
        self._path = msg
        self._current_wp = 0
        self.get_logger().info(
            f'New path: {len(msg.poses)} waypoints')

    def _pose_cb(self, msg: PoseStamped):
        first_pose = self._robot_pose is None
        self._robot_pose = msg
        if first_pose:
            spin_dur = self.get_parameter('initial_spin_duration').value
            if spin_dur > 0.0:
                self._spinning = True
                self._spin_start_time = self.get_clock().now()
                self.get_logger().info(
                    f'SLAM active — starting {spin_dur:.0f}s initial map spin')

    def _scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        cone_rad = math.radians(self.get_parameter('obstacle_cone_deg').value)
        front = np.abs(angles) < cone_rad
        r_front = ranges[front]
        valid = np.isfinite(r_front) & (r_front > msg.range_min) & (r_front < msg.range_max)
        r_valid = r_front[valid]

        stop_d = self.get_parameter('obstacle_stop_dist').value
        slow_d = self.get_parameter('obstacle_slow_dist').value

        if len(r_valid) == 0:
            self._emergency_stop = False
            self._slow_factor = 1.0
            return

        min_r = float(r_valid.min())
        if min_r < stop_d:
            self._emergency_stop = True
            self._slow_factor = 0.0
        elif min_r < slow_d:
            self._emergency_stop = False
            self._slow_factor = (min_r - stop_d) / (slow_d - stop_d)
        else:
            self._emergency_stop = False
            self._slow_factor = 1.0

    # ── Control loop ──────────────────────────────────────────────────────

    def _control_loop(self):
        cmd = Twist()

        if self._emergency_stop:
            self.get_logger().warn(
                'Emergency stop — obstacle too close', throttle_duration_sec=0.5)
            self._cmd_pub.publish(cmd)
            return

        # Initial 360° spin so SLAM can see all surrounding walls before navigation
        if self._spinning and self._spin_start_time is not None:
            elapsed = (self.get_clock().now() - self._spin_start_time).nanoseconds / 1e9
            spin_dur = self.get_parameter('initial_spin_duration').value
            if elapsed < spin_dur:
                cmd.angular.z = self.get_parameter('initial_spin_speed').value
                self._cmd_pub.publish(cmd)
                return
            else:
                self._spinning = False
                self.get_logger().info(
                    'Initial spin complete — ready for navigation goals')

        if self._path is None or self._robot_pose is None:
            self._cmd_pub.publish(cmd)
            return

        poses = self._path.poses
        if not poses or self._current_wp >= len(poses):
            self._cmd_pub.publish(cmd)
            return

        rx = self._robot_pose.pose.position.x
        ry = self._robot_pose.pose.position.y
        rtheta = _yaw_from_pose(self._robot_pose.pose)

        # Check if the final goal has been reached
        final = poses[-1].pose.position
        dist_to_goal = math.hypot(final.x - rx, final.y - ry)
        if dist_to_goal < self.get_parameter('goal_tolerance').value:
            self.get_logger().info(
                f'Goal reached! (dist={dist_to_goal:.3f} m) — stopping.')
            self._path = None
            self._cmd_pub.publish(cmd)
            return

        lookahead = self.get_parameter('lookahead_distance').value
        skip_d    = self.get_parameter('wp_skip_dist').value

        # Advance current waypoint index: skip points already behind the lookahead
        while self._current_wp < len(poses) - 1:
            wpx = poses[self._current_wp].pose.position.x
            wpy = poses[self._current_wp].pose.position.y
            d   = math.hypot(wpx - rx, wpy - ry)
            # Advance if we're already past this waypoint OR it's within skip_dist
            if d < skip_d:
                self._current_wp += 1
            elif d < lookahead and self._current_wp < len(poses) - 1:
                self._current_wp += 1
            else:
                break

        target = poses[self._current_wp].pose.position
        tx, ty = target.x, target.y

        # Bearing error in robot frame (pure pursuit)
        bearing = math.atan2(ty - ry, tx - rx)
        alpha   = _wrap(bearing - rtheta)

        linear  = self.get_parameter('linear_speed').value * self._slow_factor
        angular = self.get_parameter('angular_gain').value * alpha
        max_w   = self.get_parameter('max_angular_speed').value

        # Reduce speed on sharp turns for stability
        turn_factor = max(0.3, 1.0 - abs(alpha) / math.pi)
        linear *= turn_factor

        cmd.linear.x  = float(linear)
        cmd.angular.z = float(np.clip(angular, -max_w, max_w))
        self._cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
