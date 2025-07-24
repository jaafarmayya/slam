#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from collections import deque
from statistics import median


class MedianFilter(Node):
    """Filter moving objects from LaserScan data."""

    def __init__(self) -> None:
        super().__init__('median_filter')
        self.declare_parameter('window_size', 5)
        self.declare_parameter('threshold', 0.2)

        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.scan_history = deque(maxlen=self.window_size)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.scan_pub = self.create_publisher(LaserScan, '/static_scan', 10)
        self.get_logger().info('MedianFilter node started')

    def scan_callback(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        self.scan_history.append(ranges)

        if len(self.scan_history) < self.scan_history.maxlen:
            
            self.scan_pub.publish(msg)
            return

        filtered = ranges.copy()
        history = self.scan_history
        for i in range(len(ranges)):
            vals = [scan[i] for scan in history]
            med = median(vals)
            if abs(ranges[i] - med) > self.threshold:
                filtered[i] = float('inf')

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = filtered
        out.intensities = msg.intensities
        self.scan_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MedianFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
