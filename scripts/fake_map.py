#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import random

class FakeMapPublisher(Node):
    def __init__(self):
        super().__init__('fake_map_publisher')

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)

        # Timer to publish once at startup, then stop
        self.timer = self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        msg = OccupancyGrid()

        # Map metadata
        msg.header.frame_id = 'map'
        msg.info.resolution = 0.05  # 5cm per pixel
        msg.info.width = 200        # 200 pixels wide
        msg.info.height = 200       # 200 pixels tall
        msg.info.origin.position.x = -5.0  # center the map
        msg.info.origin.position.y = -5.0
        msg.info.origin.orientation.w = 1.0

        # Generate a fake city-like map
        size = 200 * 200
        data = [0] * size  # start with all free space

        # Border walls
        for x in range(200):
            data[x] = 100              # top row
            data[199*200 + x] = 100    # bottom row
        for y in range(200):
            data[y*200] = 100          # left col
            data[y*200 + 199] = 100    # right col

        # Random buildings
        for _ in range(40):
            bx = random.randint(5, 185)
            by = random.randint(5, 185)
            bw = random.randint(3, 8)
            bh = random.randint(3, 8)
            for dy in range(bh):
                for dx in range(bw):
                    idx = (by + dy) * 200 + (bx + dx)
                    if 0 <= idx < size:
                        data[idx] = 100

        # Scattered obstacles
        for _ in range(200):
            idx = random.randint(0, size - 1)
            data[idx] = random.randint(50, 100)

        msg.data = data
        self.pub.publish(msg)
        self.get_logger().info(f'Published fake map: {msg.info.width}x{msg.info.height}')

        # Cancel timer after publishing
        self.timer.cancel()

def main():
    rclpy.init()
    node = FakeMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
