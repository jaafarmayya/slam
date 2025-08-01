#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from sensor_msgs.msg import LaserScan

class OdomScanTransformer(Node):
    def __init__(self):
        super().__init__('odom_scan_transformer')
        # storage for latest odom pose
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # subscribe to odometry
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # subscribe to scan
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.prev_points = None    # will hold Nx2 array of past scan world‐points
        self.prev_idxs   = None  
        self.declare_parameter('distance_threshold', 0.05)    # meters
        self.dist_thresh = self.get_parameter('distance_threshold').value

        # publish filtered scans
        self.pub = self.create_publisher(LaserScan, '/static_scan', 10)
        self.get_logger().info('Odom↔Scan transformer initialized')

    def odom_callback(self, msg: Odometry):
        # extract pose.x, pose.y, yaw
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # convert quaternion to yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        self.odom_x = px
        self.odom_y = py
        self.odom_yaw = yaw

    def scan_callback(self, msg: LaserScan):
        # 1) build ranges & angles
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # 2) pick only valid beams
        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        idxs  = np.where(valid)[0]
        if idxs.size == 0:
            self.get_logger().warning('No valid beams, skipping')
            return

        # 3) compute all world‐frame points for this scan
        x_s = ranges[idxs] * np.cos(angles[idxs])
        y_s = ranges[idxs] * np.sin(angles[idxs])
        ct, st = np.cos(self.odom_yaw), np.sin(self.odom_yaw)
        x_w = ct * x_s - st * y_s + self.odom_x
        y_w = st * x_s + ct * y_s + self.odom_y
        pts_w = np.vstack((x_w, y_w)).T   # shape=(M,2)

        # 4) cache initial scan and return
        if self.prev_points is None or self.prev_points.shape[0] == 0:
            # nothing static from last time, so just cache this scan
            self.prev_points = pts_w.copy()
            self.prev_idxs   = idxs.copy()
            self.get_logger().info(
                'Previous static cache empty: recaching current scan and skipping filter'
            )
            return

        # 5) compute nearest‐neighbor dists to previous scan
        diff      = self.prev_points[np.newaxis, :, :] - pts_w[:, np.newaxis, :]
        dists     = np.linalg.norm(diff, axis=2)      # (M,P)
        min_dists = dists.min(axis=1)                 # (M,)

        # 6) detect new beams (free→occupied)
        new_occ = ~np.isin(idxs, self.prev_idxs)
        self.get_logger().info(f'new_occupied beams: {new_occ.sum()}/{len(idxs)}')

        # 7) build dynamic mask and filter
        dynamic_mask = (min_dists > self.dist_thresh) | new_occ
        filtered = ranges.copy()
        filtered[idxs[dynamic_mask]] = float('inf')

        # 8) publish the filtered scan
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

        # 9) update cache for next iteration
        static_idxs = idxs[~dynamic_mask]
        self.prev_points = pts_w[~dynamic_mask].copy()
        self.prev_idxs   = static_idxs.copy()

def main(args=None):
    rclpy.init(args=args)
    node = OdomScanTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
