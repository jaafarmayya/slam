#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import numpy as np


def bresenham(x0, y0, x1, y1):
    """Integer Bresenham line between (x0, y0) and (x1, y1)."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points


def quaternion_to_yaw(q):
    """Assuming flat robot: approximate yaw from quaternion."""
    # For q = (x≈0, y≈0, z, w), yaw = 2 * atan2(z, w)
    return 2.0 * math.atan2(q.z, q.w)


class DynamicGridFilter(Node):
    def __init__(self):
        super().__init__('dynamic_grid_filter')

        # Parameters
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('grid_size_x', 20.0)   # meters
        self.declare_parameter('grid_size_y', 20.0)
        self.declare_parameter('occ_thresh', 0.7)
        self.declare_parameter('free_thresh', 0.7)
        self.declare_parameter('n_min', 5)
        self.declare_parameter('h_dyn', 5)
        self.declare_parameter('m_dyn', 5)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('preprocessed_topic', '/preprocessed_scan')

        self.res = float(self.get_parameter('grid_resolution').value)
        self.size_x = float(self.get_parameter('grid_size_x').value)
        self.size_y = float(self.get_parameter('grid_size_y').value)
        self.occ_thresh = float(self.get_parameter('occ_thresh').value)
        self.free_thresh = float(self.get_parameter('free_thresh').value)
        self.n_min = int(self.get_parameter('n_min').value)
        self.h_dyn = int(self.get_parameter('h_dyn').value)
        self.m_dyn = int(self.get_parameter('m_dyn').value)
        self.global_frame = self.get_parameter('global_frame').value
        scan_topic = self.get_parameter('scan_topic').value
        preprocessed_topic = self.get_parameter('preprocessed_topic').value

        nx = int(self.size_x / self.res)
        ny = int(self.size_y / self.res)
        self.hits = np.zeros((nx, ny), dtype=np.int32)
        self.misses = np.zeros((nx, ny), dtype=np.int32)
        self.total = np.zeros((nx, ny), dtype=np.int32)
        self.is_dynamic = np.zeros((nx, ny), dtype=bool)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'DynamicGridFilter listening on {scan_topic}, publishing on {preprocessed_topic}'
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            preprocessed_topic,
            10
        )

    def scan_callback(self, scan: LaserScan):
        self.get_logger().debug(f'Received scan at time {scan.header.stamp}')

        try:
            # Use the scan stamp so TF interpolation works properly
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                scan.header.frame_id,
                scan.header.stamp
            )
        except Exception as e:
            self.get_logger().warn(
                f'No TF {self.global_frame} <- {scan.header.frame_id}: {e}. '
                f'Publishing unfiltered scan.'
            )
            self.scan_pub.publish(scan)
            return

        # Robot pose in map frame
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        yaw = quaternion_to_yaw(tf.transform.rotation)

        ix_r = int((rx + self.size_x / 2.0) / self.res)
        iy_r = int((ry + self.size_y / 2.0) / self.res)

        ranges = list(scan.ranges)  # mutable copy

        angle = scan.angle_min
        for k, r in enumerate(scan.ranges):
            if np.isinf(r) or np.isnan(r):
                angle += scan.angle_increment
                continue

            # Endpoint in laser frame
            x_l = r * math.cos(angle)
            y_l = r * math.sin(angle)

            # Transform to map (2D, yaw only)
            x_m = rx + math.cos(yaw) * x_l - math.sin(yaw) * y_l
            y_m = ry + math.sin(yaw) * x_l + math.cos(yaw) * y_l

            ix_e = int((x_m + self.size_x / 2.0) / self.res)
            iy_e = int((y_m + self.size_y / 2.0) / self.res)

            # Raytrace free cells between robot and endpoint
            for (ix, iy) in bresenham(ix_r, iy_r, ix_e, iy_e)[:-1]:  # exclude endpoint
                if self._in_bounds(ix, iy):
                    self.misses[ix, iy] += 1
                    self.total[ix, iy] += 1

            # Occupied endpoint
            if self._in_bounds(ix_e, iy_e):
                self.hits[ix_e, iy_e] += 1
                self.total[ix_e, iy_e] += 1
                self._update_class(ix_e, iy_e)

                # If endpoint cell is dynamic, hide this return
                if self.is_dynamic[ix_e, iy_e]:
                    ranges[k] = scan.range_max + 1.0  # or float('inf')

            angle += scan.angle_increment

        scan_filtered = LaserScan()
        scan_filtered.header = scan.header
        scan_filtered.angle_min = scan.angle_min
        scan_filtered.angle_max = scan.angle_max
        scan_filtered.angle_increment = scan.angle_increment
        scan_filtered.time_increment = scan.time_increment
        scan_filtered.scan_time = scan.scan_time
        scan_filtered.range_min = scan.range_min
        scan_filtered.range_max = scan.range_max
        scan_filtered.ranges = ranges
        scan_filtered.intensities = scan.intensities

        self.scan_pub.publish(scan_filtered)

    def _in_bounds(self, ix, iy):
        return 0 <= ix < self.hits.shape[0] and 0 <= iy < self.hits.shape[1]

    def _update_class(self, ix, iy):
        t = self.total[ix, iy]
        if t < self.n_min:
            return
        h = self.hits[ix, iy]
        m = self.misses[ix, iy]

        # Strong conflicts → dynamic
        if h >= self.h_dyn and m >= self.m_dyn:
            self.is_dynamic[ix, iy] = True
            return

        p_occ = h / float(t)
        p_free = m / float(t)

        if p_occ >= self.occ_thresh or p_free >= self.free_thresh:
            self.is_dynamic[ix, iy] = False


def main(args=None):
    rclpy.init(args=args)
    node = DynamicGridFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
