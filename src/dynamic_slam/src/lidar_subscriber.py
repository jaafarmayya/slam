#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.get_logger().info('LiDAR listener started.')

    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        first, mid, last = msg.ranges[0], msg.ranges[n//2], msg.ranges[-1]

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
