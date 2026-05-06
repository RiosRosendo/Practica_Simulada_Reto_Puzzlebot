#!/usr/bin/env python3
"""
Velocity Bridge — Convert real Jetson wheel encoder topics to MCL format.

Converts:
  /VelocityEncl  → /wl (left wheel)
  /VelocityEncnR → /wr (right wheel)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class VelocityBridge(Node):

    def __init__(self):
        super().__init__('velocity_bridge')

        # Subscribe to actual wheel velocities from Jetson
        self.create_subscription(Float32, '/VelocityEncl', self.left_callback, 10)
        self.create_subscription(Float32, '/VelocityEncnR', self.right_callback, 10)

        # Publish as /wr and /wl for MCL
        self.wr_pub = self.create_publisher(Float32, '/wr', 10)
        self.wl_pub = self.create_publisher(Float32, '/wl', 10)

        self.get_logger().info(
            'Velocity bridge started: /VelocityEncl, /VelocityEncnR → /wl, /wr'
        )

    def left_callback(self, msg):
        self.wl_pub.publish(msg)

    def right_callback(self, msg):
        self.wr_pub.publish(msg)


def main():
    rclpy.init()
    node = VelocityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
