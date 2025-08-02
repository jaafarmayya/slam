#!/usr/bin/env python3
"""
ROS2 node: Dynamic object filter using map frame with odom fallback for LiDAR scans.
Filters out dynamic points by comparing consecutive scans in the map frame when available,
otherwise falls back to odom frame to ensure robust operation.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np


def transform_point(transform: TransformStamped, point: np.ndarray) -> np.ndarray:
    """
    Apply a ROS TransformStamped to a 3D point.
    """
    t = transform.transform.translation
    q = transform.transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    R = np.array([
        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy]
    ])
    return R.dot(point) + np.array([t.x, t.y, t.z])

class AdaptiveFrameDynamicFilter(Node):
    def __init__(self):
        super().__init__('adaptive_frame_dynamic_filter')
        # Parameters for topics, thresholds, frames
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/static_scan')
        self.declare_parameter('velocity_threshold', 0.1)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('fallback_frame', 'odom')

        self.scan_topic = self.get_parameter('scan_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.vel_thresh = self.get_parameter('velocity_threshold').value
        self.max_range = self.get_parameter('max_range').value
        self.world_frame = self.get_parameter('world_frame').value
        self.fallback_frame = self.get_parameter('fallback_frame').value

        # TF2 buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Store previous scan and transform
        self.prev_scan = None
        self.prev_transform = None
        self.prev_time = None

        # Setup ROS2 interfaces
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)
        self.get_logger().info(f'Adaptive filter init: world_frame="{self.world_frame}", fallback="{self.fallback_frame}"')

    def lookup_transform_with_fallback(self, target_frame, source_frame, time):
        """Attempt map lookup, fallback to odom if unavailable."""
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, time)
        except Exception:
            return self.tf_buffer.lookup_transform(self.fallback_frame, source_frame, time)

    def scan_callback(self, msg: LaserScan) -> None:
        # Convert ROS time
        t = rclpy.time.Time.from_msg(msg.header.stamp)
        frame_id = msg.header.frame_id

        # Get suitable transform (map or odom)
        try:
            transform = self.lookup_transform_with_fallback(self.world_frame, frame_id, t)
        except Exception as e:
            self.pub.publish(msg)
            return

        # Initialize on first scan
        if self.prev_scan is None:
            self.prev_scan = msg
            self.prev_transform = transform
            self.prev_time = msg.header.stamp
            self.pub.publish(msg)
            return

        # Compute time delta
        dt = (msg.header.stamp.sec - self.prev_time.sec) + \
             (msg.header.stamp.nanosec - self.prev_time.nanosec)*1e-9
        dt = max(dt, 1e-3)

        # Build angle and range arrays
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        cur_ranges = np.array(msg.ranges)
        prev_ranges = np.array(self.prev_scan.ranges)
        size = min(len(angles), len(cur_ranges), len(prev_ranges))
        angles = angles[:size]
        cur_ranges = cur_ranges[:size]
        prev_ranges = prev_ranges[:size]

        # Initial valid mask: within sensor and max range
        valid = (cur_ranges > msg.range_min) & (cur_ranges < self.max_range)
        valid &= (prev_ranges > self.prev_scan.range_min) & (prev_ranges < self.max_range)
        # Exclude non-finite values (inf, NaN)
        finite_cur = np.isfinite(cur_ranges)
        finite_prev = np.isfinite(prev_ranges)
        valid &= (finite_cur & finite_prev)

        # Extract only the valid indices
        idxs = np.nonzero(valid)[0]
        if idxs.size == 0:
            # Nothing valid: publish original and skip
            self.pub.publish(msg)
            return

        angles_v = angles[idxs]
        r_cur_v = cur_ranges[idxs]
        r_prev_v = prev_ranges[idxs]

        # Convert valid polar -> Cartesian in sensor frame
        pts_cur = np.vstack((r_cur_v * np.cos(angles_v),
                             r_cur_v * np.sin(angles_v),
                             np.zeros_like(angles_v))).T
        pts_prev = np.vstack((r_prev_v * np.cos(angles_v),
                              r_prev_v * np.sin(angles_v),
                              np.zeros_like(angles_v))).T

        # Transform to world frame
        pts_cur_w = np.array([transform_point(transform, p) for p in pts_cur])
        pts_prev_w = np.array([transform_point(self.prev_transform, p) for p in pts_prev])

        # Compute velocities and filter dynamic points
        distances = np.linalg.norm(pts_cur_w - pts_prev_w, axis=1)
        velocities = distances / dt

        filtered = cur_ranges.copy().tolist()
        for idx_array, vel in zip(idxs, velocities):
            if vel >= self.vel_thresh:
                filtered[idx_array] = float('inf')

        # Publish filtered scan
        out = LaserScan(
            header=msg.header,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=msg.range_min,
            range_max=self.max_range,
            ranges=filtered,
            intensities=msg.intensities
        )
        self.pub.publish(out)

        # Update stored state
        self.prev_scan = msg
        self.prev_transform = transform
        self.prev_time = msg.header.stamp


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveFrameDynamicFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
