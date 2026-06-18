#!/usr/bin/env python3
"""
Data Logger Node — Probabilistic Robot Lab
==========================================
Subscribes to /odom, /pose_kf, /pose_ekf, /pose_pf and logs them to a
timestamped CSV file at 10 Hz.

Output: ~/prob_ros_ws/logs/filter_data_YYYYMMDD_HHMMSS.csv

Run plot_results.py on the CSV afterwards to generate paper plots:
    python3 ~/prob_ros_ws/src/probabilistic_robot_lab/scripts/plot_results.py
"""

import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


def quat_to_yaw(qz: float, qw: float) -> float:
    return 2.0 * math.atan2(qz, qw)


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')

        # ------------------------------------------------------------------
        # Output file
        # ------------------------------------------------------------------
        log_dir = os.path.expanduser('~/prob_ros_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._path = os.path.join(log_dir, f'filter_data_{stamp}.csv')

        self._file   = open(self._path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            'time_s',
            'odom_x', 'odom_y', 'odom_theta',
            'kf_x',  'kf_y',  'kf_theta',  'kf_cov_xx',  'kf_cov_yy',  'kf_cov_tt',
            'ekf_x', 'ekf_y', 'ekf_theta', 'ekf_cov_xx', 'ekf_cov_yy', 'ekf_cov_tt',
            'pf_x',  'pf_y',  'pf_theta',  'pf_cov_xx',  'pf_cov_yy',  'pf_cov_tt',
        ])

        # ------------------------------------------------------------------
        # Latest cached values (None until first message arrives)
        # ------------------------------------------------------------------
        self._odom = None
        self._kf   = None
        self._ekf  = None
        self._pf   = None
        self._t0   = None

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(Odometry,
            '/odom',     self._odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped,
            '/pose_kf',  self._kf_cb,  10)
        self.create_subscription(PoseWithCovarianceStamped,
            '/pose_ekf', self._ekf_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped,
            '/pose_pf',  self._pf_cb,  10)

        # Log at 10 Hz
        self.create_timer(0.1, self._log_row)

        self.get_logger().info(f'DataLogger started → {self._path}')

    # ------------------------------------------------------------------
    # Callbacks — cache latest values
    # ------------------------------------------------------------------
    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._odom = (p.x, p.y, quat_to_yaw(o.z, o.w))

    def _pose_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        c = msg.pose.covariance
        return (p.x, p.y, quat_to_yaw(o.z, o.w), c[0], c[7], c[35])

    def _kf_cb(self,  msg): self._kf  = self._pose_cb(msg)
    def _ekf_cb(self, msg): self._ekf = self._pose_cb(msg)
    def _pf_cb(self,  msg): self._pf  = self._pose_cb(msg)

    # ------------------------------------------------------------------
    # Timer callback — write one row to CSV
    # ------------------------------------------------------------------
    def _log_row(self):
        # Need at least odom + one filter before writing
        if self._odom is None:
            return
        if self._kf is None and self._ekf is None and self._pf is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        nan = float('nan')
        empty = (nan,) * 6

        self._writer.writerow([
            f'{t:.3f}',
            *[f'{v:.5f}' for v in self._odom],
            *[f'{v:.5f}' for v in (self._kf  or empty)],
            *[f'{v:.5f}' for v in (self._ekf or empty)],
            *[f'{v:.5f}' for v in (self._pf  or empty)],
        ])
        self._file.flush()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._file.close()
        self.get_logger().info(f'DataLogger: saved → {self._path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
