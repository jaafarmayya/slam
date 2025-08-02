#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from collections import deque
from scipy.spatial import cKDTree as KDTree

class OdomScanTransformer(Node):
    def __init__(self):
        super().__init__('odom_scan_transformer')
        # storage for latest odom pose
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # parameters
        self.declare_parameter('distance_threshold', 0.05)       # meters
        self.declare_parameter('static_window_size', 5)         # number of scans in static map
        self.declare_parameter('warmup_scans', 5)               # skip filtering until this many scans
        self.dist_thresh = self.get_parameter('distance_threshold').value
        self.window_size = self.get_parameter('static_window_size').value
        self.warmup_scans = self.get_parameter('warmup_scans').value

        # buffer for last N static scans
        self.static_window = deque(maxlen=self.window_size)
        self.scan_count = 0

        # subscribe to odometry
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # subscribe to scan
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # publisher for filtered static scan
        self.pub = self.create_publisher(LaserScan, '/static_scan', 10)

        self.get_logger().info('Odom↔Scan transformer initialized with window_size=%d, threshold=%.3f' % (
            self.window_size, self.dist_thresh))

    def odom_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        self.odom_x = px
        self.odom_y = py
        self.odom_yaw = yaw

    def scan_callback(self, msg: LaserScan):
        # build ranges & angles
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # valid beams mask
        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        idxs  = np.where(valid)[0]
        if idxs.size == 0:
            self.get_logger().warning('No valid beams, skipping')
            return

        # compute world-frame points
        x_s = ranges[idxs] * np.cos(angles[idxs])
        y_s = ranges[idxs] * np.sin(angles[idxs])
        ct, st = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        x_w = ct * x_s - st * y_s + self.odom_x
        y_w = st * x_s + ct * y_s + self.odom_y
        pts_w = np.vstack((x_w, y_w)).T  # (M,2)

        self.scan_count += 1
        # warm-up: fill static_window
        if self.scan_count <= self.warmup_scans:
            self.static_window.append(pts_w.copy())
            self.get_logger().info(f'Warmup [{self.scan_count}/{self.warmup_scans}]: caching scan')
            return

        # build KDTree from accumulated static map
        static_pts = np.vstack(self.static_window)
        tree = KDTree(static_pts)

        # nearest distances
        min_dists, _ = tree.query(pts_w, k=1)

        # detect new beams compared to last static window tail
        last_static = self.static_window[-1]
        # flatten all last scan idxs via simple lookup of coordinate equality is hard; we skip new_occ for simplicity
        new_occ = np.zeros_like(min_dists, dtype=bool)

        # dynamic mask: points far from all static map points
        dynamic_mask = min_dists > self.dist_thresh
        self.get_logger().info(f'Dynamic beams: {dynamic_mask.sum()}/{len(idxs)}')

        # filter out dynamic beams
        filtered = ranges.copy()
        filtered[idxs[dynamic_mask]] = float('inf')

        # publish filtered static scan
        out = LaserScan(
            header=msg.header,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=msg.range_min,
            range_max=msg.range_max,
            ranges=filtered.tolist(),
            intensities=msg.intensities
        )
        self.pub.publish(out)

        # update static window: append current static points
        static_now = pts_w[~dynamic_mask]
        if static_now.size > 0:
            self.static_window.append(static_now.copy())
        else:
            self.get_logger().warning('All points dynamic: static map unchanged')


def main(args=None):
    rclpy.init(args=args)
    node = OdomScanTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
