#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import math
import threading
import csv
import os
import atexit
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
# Matplotlib imports
import matplotlib.pyplot as plt

class LocalizationError(Node):
    def __init__(self):
        super().__init__('localization_error_node')
        self.get_logger().info('Initializing LocalizationError node')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('TF2 buffer and listener set up')

        # Declare and get parameters
        self.declare_parameter('model_name', 'burger')
        self.model_name = self.get_parameter('model_name').value
        self.get_logger().info(f"Parameter 'model_name' set to: {self.model_name}")

        # storage for plotting
        self.times = []
        self.pos_errs = []
        self.ang_errs = []
        self.get_logger().info('Initialized storage for times and errors')

        # set up matplotlib
        plt.ion()
        self.fig, (self.ax_pos, self.ax_ang) = plt.subplots(2, 1, figsize=(6, 6))
        self.line_pos, = self.ax_pos.plot([], [], label='Position error (m)')
        self.line_ang, = self.ax_ang.plot([], [], label='Angle error (°)')
        self.ax_pos.set_ylabel('Error [m]')
        self.ax_ang.set_ylabel('Error [°]')
        self.ax_ang.set_xlabel('Time [s]')
        self.ax_pos.legend()
        self.ax_ang.legend()
        plt.show()
        try:
            self.fig.canvas.manager.set_window_title('Localization Error')
        except Exception:
            self.get_logger().warn('Could not set window title on Matplotlib figure')
        self.get_logger().info('Matplotlib figure and axes set up')

        # subscribe to Gazebo model states
        self.create_subscription(
            ModelStates,
            '/model_states',
            self.on_model_states,
            qos_profile=10
        )
        self.get_logger().info("Subscribed to '/gazebo/model_states'")

        # register atexit handler to save CSV
        atexit.register(self.save_csv)
        self.get_logger().info('Registered CSV save handler on exit')

        self.get_logger().info('LocalizationError node initialization complete')

    def on_model_states(self, msg: ModelStates):
        self.get_logger().info('Received ModelStates message')
        now = self.get_clock().now().nanoseconds * 1e-9  # seconds
        self.get_logger().info(f'Current time: {now:.6f}s')

        # find our robot
        try:
            idx = msg.name.index(self.model_name)
            self.get_logger().info(f"Found model '{self.model_name}' at index {idx}")
        except ValueError:
            self.get_logger().warn(f"Model '{self.model_name}' not found in message")
            return

        # ground truth
        pose = msg.pose[idx]
        x_gt, y_gt = pose.position.x, pose.position.y
        q = pose.orientation
        yaw_gt = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        self.get_logger().info(f"Ground truth pose: x={x_gt:.3f}, y={y_gt:.3f}, yaw={math.degrees(yaw_gt):.2f}°")

        # estimated via TF
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            self.get_logger().info('TF lookup successful')
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        x_est = t.transform.translation.x
        y_est = t.transform.translation.y
        q2 = t.transform.rotation
        yaw_est = math.atan2(
            2*(q2.w*q2.z + q2.x*q2.y),
            1 - 2*(q2.y*q2.y + q2.z*q2.z)
        )
        self.get_logger().info(f"Estimated pose: x={x_est:.3f}, y={y_est:.3f}, yaw={math.degrees(yaw_est):.2f}°")

        # compute errors
        dx, dy = x_est - x_gt, y_est - y_gt
        pos_err = math.hypot(dx, dy)
        ang_err = abs((yaw_est - yaw_gt + math.pi) % (2*math.pi) - math.pi)
        ang_err_deg = math.degrees(ang_err)
        self.get_logger().info(f"Computed errors: pos_err={pos_err:.3f}m, ang_err={ang_err_deg:.2f}°")

        # store
        self.times.append(now)
        self.pos_errs.append(pos_err)
        self.ang_errs.append(ang_err_deg)
        self.get_logger().info('Appended errors to storage')

        # update plots
        self.line_pos.set_data(self.times, self.pos_errs)
        self.line_ang.set_data(self.times, self.ang_errs)
        for ax in (self.ax_pos, self.ax_ang):
            ax.relim()
            ax.autoscale_view()
        self.get_logger().info('Updated plot data and rescaled axes')

        # redraw
        threading.Thread(target=self.fig.canvas.draw_idle).start()
        self.get_logger().info('Triggered Matplotlib redraw')

        # log summary
        self.get_logger().info(
            f"[Err] t={now:.1f}s → pos: {pos_err:.3f}m, ang: {ang_err_deg:.1f}°"
        )
    
    
    def save_csv(self):
        """Called on exit to dump the recorded errors."""
        self.get_logger().info('Saving CSV log of errors')
        if not self.times:
            self.get_logger().warn('No data to save; CSV will not be created')
            return

        try:
            package_share_dir = get_package_share_directory('dynamic_slam')  
            data_dir = os.path.join(package_share_dir, 'data')
            os.makedirs(data_dir, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"localization_error_{timestamp}.csv"
            output_path = os.path.join(data_dir, filename)

            with open(output_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time_s', 'pos_err_m', 'ang_err_deg'])
                for t, p, a in zip(self.times, self.pos_errs, self.ang_errs):
                    writer.writerow([f"{t:.6f}", f"{p:.6f}", f"{a:.6f}"])

            self.get_logger().info(f"Successfully saved error log to {output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationError()
    node.get_logger().info('Entering spin')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # save_csv will be called by atexit

if __name__ == '__main__':
    main()
