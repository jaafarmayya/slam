#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import threading
import csv
import os
import atexit
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import matplotlib.pyplot as plt

class LocalizationError(Node):
    def __init__(self):
        super().__init__('localization_error_node')
        self.declare_parameter('model_name', 'burger')
        self.model_name = self.get_parameter('model_name').value

        self.gt_x = self.gt_y = self.gt_yaw = None
        self.times = []
        self.pos_errs = []
        self.ang_errs = []
        self.gt_xs = []
        self.gt_ys = []
        self.gt_yaws = []
        self.est_xs = []
        self.est_ys = []
        self.est_yaws = []

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
            pass

        self.create_subscription(ModelStates, '/model_states', self.on_model_states, qos_profile=10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.on_slam_pose, qos_profile=10)

        atexit.register(self.save_csv)

    def on_model_states(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        p = msg.pose[idx]
        self.gt_x = p.position.x
        self.gt_y = p.position.y
        q = p.orientation
        self.gt_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

    def on_slam_pose(self, msg: PoseWithCovarianceStamped):
        if self.gt_x is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q2 = msg.pose.pose.orientation
        yaw_est = math.atan2(
            2*(q2.w*q2.z + q2.x*q2.y),
            1 - 2*(q2.y*q2.y + q2.z*q2.z)
        )
        dx, dy = px - self.gt_x, py - self.gt_y
        pos_err = math.hypot(dx, dy)
        ang_err = abs((yaw_est - self.gt_yaw + math.pi) % (2*math.pi) - math.pi)
        ang_err_deg = math.degrees(ang_err)

        self.times.append(now)
        self.pos_errs.append(pos_err)
        self.ang_errs.append(ang_err_deg)
        self.gt_xs.append(self.gt_x)
        self.gt_ys.append(self.gt_y)
        self.gt_yaws.append(self.gt_yaw)
        self.est_xs.append(px)
        self.est_ys.append(py)
        self.est_yaws.append(yaw_est)

        self.line_pos.set_data(self.times, self.pos_errs)
        self.line_ang.set_data(self.times, self.ang_errs)
        for ax in (self.ax_pos, self.ax_ang):
            ax.relim()
            ax.autoscale_view()
        threading.Thread(target=self.fig.canvas.draw_idle).start()

        self.get_logger().info(f"[Err] t={now:.1f}s → pos: {pos_err:.3f}m, ang: {ang_err_deg:.1f}°")

    def save_csv(self):
        if not self.times:
            return
        try:
            pkg_dir = get_package_share_directory('dynamic_slam')
            data_dir = os.path.join(pkg_dir, 'data')
            os.makedirs(data_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = f"localization_error_{ts}.csv"
            path = os.path.join(data_dir, fname)
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'time_s',
                    'gt_x', 'gt_y', 'gt_yaw',
                    'est_x', 'est_y', 'est_yaw',
                    'pos_err_m', 'ang_err_deg'
                ])
                for i in range(len(self.times)):
                    writer.writerow([
                        f"{self.times[i]:.6f}",
                        f"{self.gt_xs[i]:.6f}", f"{self.gt_ys[i]:.6f}", f"{self.gt_yaws[i]:.6f}",
                        f"{self.est_xs[i]:.6f}", f"{self.est_ys[i]:.6f}", f"{self.est_yaws[i]:.6f}",
                        f"{self.pos_errs[i]:.6f}", f"{self.ang_errs[i]:.6f}"
                    ])
            self.get_logger().info(f"Saved error log to {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
