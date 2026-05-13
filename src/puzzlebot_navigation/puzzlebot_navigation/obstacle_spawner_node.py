#!/usr/bin/env python3
"""
Dynamic obstacle spawner for Ignition Gazebo (ROS 2 Humble).

Periodically spawns random static box obstacles inside the simulation world
using the Ignition transport `gz service` CLI to call the
`/world/<name>/create` service.  Each obstacle gets a random position,
orientation, and colour so the A* planner is forced to detect and replan.

Parameters:
  world_name      (str)   — Gazebo world name (must match the loaded world)
  spawn_interval  (float) — seconds between spawns
  max_obstacles   (int)   — total obstacles to spawn before stopping
  arena_min       (float) — lower bound of random spawn area (metres)
  arena_max       (float) — upper bound of random spawn area (metres)
  safe_radius     (float) — min distance from origin to avoid spawning on robot
"""

import math
import random
import subprocess
import rclpy
from rclpy.node import Node


_BOX_SDF_TEMPLATE = (
    '<sdf version="1.6">'
    '<model name="{name}">'
    '<static>true</static>'
    '<pose>{x} {y} 0.25 0 0 {yaw}</pose>'
    '<link name="link">'
    '<collision name="collision">'
    '<geometry><box><size>0.5 0.5 0.5</size></box></geometry>'
    '</collision>'
    '<visual name="visual">'
    '<geometry><box><size>0.5 0.5 0.5</size></box></geometry>'
    '<material>'
    '<ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>'
    '<diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>'
    '</material>'
    '</visual>'
    '</link>'
    '</model>'
    '</sdf>'
)


class ObstacleSpawnerNode(Node):

    def __init__(self):
        super().__init__('obstacle_spawner')

        self.declare_parameter('world_name',     'default')
        self.declare_parameter('spawn_interval', 20.0)
        self.declare_parameter('max_obstacles',   5)
        self.declare_parameter('arena_min',      -3.5)
        self.declare_parameter('arena_max',       3.5)
        self.declare_parameter('safe_radius',     0.8)  # keep clear of origin

        self._count = 0
        interval = self.get_parameter('spawn_interval').value
        self.create_timer(interval, self._spawn_obstacle)

        self.get_logger().info(
            f'Obstacle spawner ready — '
            f'interval={interval:.0f}s  '
            f'max={self.get_parameter("max_obstacles").value}')

    def _spawn_obstacle(self):
        max_obs = self.get_parameter('max_obstacles').value
        if self._count >= max_obs:
            return

        lo   = self.get_parameter('arena_min').value
        hi   = self.get_parameter('arena_max').value
        safe = self.get_parameter('safe_radius').value

        # Retry a few times to avoid placing directly on the robot
        for _ in range(20):
            x   = random.uniform(lo, hi)
            y   = random.uniform(lo, hi)
            if math.hypot(x, y) >= safe:
                break
        else:
            self.get_logger().warn('Could not find a safe spawn position — skipping')
            return

        yaw = random.uniform(0.0, math.pi)
        r   = random.uniform(0.4, 1.0)
        g   = random.uniform(0.2, 0.8)
        b   = random.uniform(0.1, 0.5)
        name = f'dynamic_obstacle_{self._count}'

        sdf = _BOX_SDF_TEMPLATE.format(
            name=name, x=x, y=y, yaw=yaw, r=r, g=g, b=b)

        # Escape double-quotes for shell embedding
        sdf_escaped = sdf.replace('"', '\\"')
        world = self.get_parameter('world_name').value

        cmd = (
            f'ign service '
            f'-s /world/{world}/create '
            f'--reqtype ignition.msgs.EntityFactory '
            f'--reptype ignition.msgs.Boolean '
            f'--timeout 2000 '
            f'--req \'sdf: "{sdf_escaped}"\''
        )

        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=5)

        if result.returncode == 0:
            self._count += 1
            self.get_logger().info(
                f'Spawned {name} at ({x:.2f}, {y:.2f})  '
                f'[{self._count}/{max_obs}]')
        else:
            self.get_logger().warn(
                f'Failed to spawn obstacle (rc={result.returncode}): '
                f'{result.stderr[:120]}')


def main():
    rclpy.init()
    node = ObstacleSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
