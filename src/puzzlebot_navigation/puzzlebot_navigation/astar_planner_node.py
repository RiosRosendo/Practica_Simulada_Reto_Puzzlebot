#!/usr/bin/env python3
"""
A* global path planner for Puzzlebot autonomous navigation.

Subscribes:
  /map        (nav_msgs/OccupancyGrid) — SLAM occupancy grid (latched)
  /slam_pose  (geometry_msgs/PoseStamped) — current robot pose in map frame
  /goal_pose  (geometry_msgs/PoseStamped) — navigation goal in map frame
  /scan       (sensor_msgs/LaserScan)    — used to trigger replanning

Publishes:
  /path  (nav_msgs/Path) — planned collision-free path in map frame
"""

import math
import heapq

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan


def _yaw_from_pose(pose):
    q = pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class AStarPlannerNode(Node):

    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('inflation_radius', 0.25)   # metres
        self.declare_parameter('obstacle_threshold', 50)   # occupancy value (0-100)
        self.declare_parameter('replan_on_scan', True)
        self.declare_parameter('replan_obstacle_dist', 0.55)  # m — triggers replan

        self._map: OccupancyGrid | None = None
        self._robot_pose: PoseStamped | None = None
        self._goal_pose: PoseStamped | None = None
        self._last_obstacle_close = False

        # Match the SLAM node's default VOLATILE publisher QoS
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self.create_subscription(PoseStamped, '/slam_pose', self._pose_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)

        self._path_pub = self.create_publisher(Path, '/path', 10)

        self.get_logger().info(
            'A* planner ready — waiting for /map and /goal_pose')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        if self._map is None:
            self.get_logger().info('Map received from SLAM')
        self._map = msg
        if self._goal_pose is not None and self._robot_pose is not None:
            self._plan()

    def _pose_cb(self, msg: PoseStamped):
        first_pose = self._robot_pose is None
        self._robot_pose = msg
        if first_pose:
            self.get_logger().info(
                f'Slam pose received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        # Goal may have arrived before slam_pose was available — plan now.
        if first_pose and self._goal_pose is not None and self._map is not None:
            self._plan()

    def _goal_cb(self, msg: PoseStamped):
        self._goal_pose = msg
        self.get_logger().info(
            f'New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        if self._map is not None and self._robot_pose is not None:
            self._plan()

    def _scan_cb(self, msg: LaserScan):
        if not self.get_parameter('replan_on_scan').value:
            return
        ranges = np.array(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Monitor a ±40° front cone
        front_mask = np.abs(angles) < math.radians(40.0)
        r_front = ranges[front_mask]
        valid = np.isfinite(r_front) & (r_front > msg.range_min) & (r_front < msg.range_max)
        r_valid = r_front[valid]

        threshold = self.get_parameter('replan_obstacle_dist').value
        obstacle_close = bool(len(r_valid) > 0 and float(r_valid.min()) < threshold)

        # Only replan on the rising edge (new obstacle, not sustained)
        if obstacle_close and not self._last_obstacle_close:
            if self._map is not None and self._robot_pose is not None \
                    and self._goal_pose is not None:
                self.get_logger().info('Obstacle detected — triggering replan')
                self._plan()

        self._last_obstacle_close = obstacle_close

    # ── Grid helpers ──────────────────────────────────────────────────────

    def _world_to_grid(self, wx: float, wy: float):
        m = self._map
        gx = int((wx - m.info.origin.position.x) / m.info.resolution)
        gy = int((wy - m.info.origin.position.y) / m.info.resolution)
        return gx, gy

    def _grid_to_world(self, gx: int, gy: int):
        m = self._map
        half = m.info.resolution * 0.5
        wx = gx * m.info.resolution + m.info.origin.position.x + half
        wy = gy * m.info.resolution + m.info.origin.position.y + half
        return wx, wy

    def _build_inflated_grid(self) -> np.ndarray:
        """Return a boolean grid: True = obstacle (inflated)."""
        m = self._map
        w, h = m.info.width, m.info.height
        raw = np.array(m.data, dtype=np.int8).reshape((h, w))
        thresh = self.get_parameter('obstacle_threshold').value

        # Unknown (-1) is treated as FREE so the robot can navigate into
        # unexplored areas.  Only confirmed occupied cells (>= thresh) block
        # the path.  The scan-triggered replan loop handles actual walls.
        obstacle = (raw >= thresh)

        rad_cells = int(math.ceil(
            self.get_parameter('inflation_radius').value / m.info.resolution))

        # Morphological dilation for inflation
        from scipy.ndimage import binary_dilation
        struct = np.ones((2 * rad_cells + 1, 2 * rad_cells + 1), dtype=bool)
        return binary_dilation(obstacle, structure=struct)

    # ── A* algorithm ─────────────────────────────────────────────────────

    def _astar(self, grid: np.ndarray, start: tuple, goal: tuple):
        """
        8-connected A* on a 2-D boolean grid.
        Returns list of (gx, gy) cells from start to goal, or None if no path.
        """
        h, w = grid.shape
        sx, sy = start
        gx, gy = goal

        # Boundary and occupancy checks
        for (cx, cy), label in [((sx, sy), 'start'), ((gx, gy), 'goal')]:
            if not (0 <= cx < w and 0 <= cy < h):
                self.get_logger().warn(f'A*: {label} ({cx},{cy}) out of grid bounds')
                return None
            if grid[cy, cx]:
                self.get_logger().warn(f'A*: {label} ({cx},{cy}) is in obstacle')
                return None

        def heuristic(x, y):
            return math.hypot(gx - x, gy - y)

        # (f, g, x, y)
        open_heap = [(heuristic(sx, sy), 0.0, sx, sy)]
        came_from: dict[tuple, tuple] = {}
        g_score: dict[tuple, float] = {(sx, sy): 0.0}

        NEIGHBOURS = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_heap:
            _, g, x, y = heapq.heappop(open_heap)

            if (x, y) == (gx, gy):
                # Reconstruct path
                path = []
                cur = (x, y)
                while cur in came_from:
                    path.append(cur)
                    cur = came_from[cur]
                path.append((sx, sy))
                path.reverse()
                return path

            if g > g_score.get((x, y), float('inf')) + 1e-9:
                continue  # stale entry

            for dx, dy in NEIGHBOURS:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                if grid[ny, nx]:
                    continue
                ng = g + math.hypot(dx, dy)
                if ng < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = ng
                    came_from[(nx, ny)] = (x, y)
                    heapq.heappush(open_heap, (ng + heuristic(nx, ny), ng, nx, ny))

        return None  # no path

    # ── Path smoothing ────────────────────────────────────────────────────

    @staticmethod
    def _smooth_path(cells: list, grid: np.ndarray, weight_data=0.5, weight_smooth=0.1):
        """Gradient-descent path smoothing that keeps waypoints in free space."""
        h, w = grid.shape
        path = [[float(c[0]), float(c[1])] for c in cells]
        change = 1.0
        tol = 1e-4
        while change > tol:
            change = 0.0
            for i in range(1, len(path) - 1):
                ox, oy = path[i]
                dx = (weight_data * (cells[i][0] - path[i][0])
                      + weight_smooth * (path[i - 1][0] + path[i + 1][0] - 2 * path[i][0]))
                dy = (weight_data * (cells[i][1] - path[i][1])
                      + weight_smooth * (path[i - 1][1] + path[i + 1][1] - 2 * path[i][1]))
                nx = int(round(path[i][0] + dx))
                ny = int(round(path[i][1] + dy))
                # Don't move into obstacle
                if 0 <= nx < w and 0 <= ny < h and not grid[ny, nx]:
                    path[i][0] += dx
                    path[i][1] += dy
                change += abs(dx) + abs(dy)
        return [(int(round(p[0])), int(round(p[1]))) for p in path]

    # ── Plan & publish ────────────────────────────────────────────────────

    def _plan(self):
        try:
            grid = self._build_inflated_grid()
        except ImportError:
            self.get_logger().error(
                'scipy not available — cannot inflate obstacles. '
                'Install with: pip install scipy')
            return

        rp = self._robot_pose.pose.position
        gp = self._goal_pose.pose.position

        start = self._world_to_grid(rp.x, rp.y)
        goal  = self._world_to_grid(gp.x, gp.y)

        self.get_logger().info(
            f'A* planning: world ({rp.x:.2f},{rp.y:.2f}) → ({gp.x:.2f},{gp.y:.2f})  '
            f'grid {start} → {goal}')

        path_cells = self._astar(grid, start, goal)
        if path_cells is None:
            self.get_logger().warn('A*: no path found to goal')
            return

        # Light smoothing pass
        path_cells = self._smooth_path(path_cells, grid)

        # Build and publish nav_msgs/Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for cx, cy in path_cells:
            wx, wy = self._grid_to_world(cx, cy)
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self._path_pub.publish(path_msg)
        self.get_logger().info(
            f'Path published — {len(path_cells)} waypoints')


def main():
    rclpy.init()
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
