#!/usr/bin/env python3
"""
Auto Navigation Node
====================
Automatically sets the robot's initial pose in AMCL and then drives it
through a fixed sequence of waypoints via the Nav2 NavigateToPose action.

All three filter nodes observe the same trajectory — required for fair
comparison in the lab paper.

Usage (automatic via launch file — no manual interaction needed):
    ros2 launch probabilistic_robot_lab filters.launch.py

To adjust the route, edit INITIAL_POSE and WAYPOINTS below.
The robot spawns at (0, 0) in the tb3_sandbox map by default.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


# ---------------------------------------------------------------------------
# Robot spawn position in the map frame [x, y, theta_deg]
# Check by echoing /odom right after startup; default TB3 sandbox = (0, 0, 0)
# ---------------------------------------------------------------------------
INITIAL_POSE = (0.0, 0.0, 0.0)

# ---------------------------------------------------------------------------
# Fixed waypoints [x, y, theta_deg] in map frame.
# Simple cross pattern — navigable in the tb3_sandbox world.
# Nav2 plans around the inner obstacles automatically.
# ---------------------------------------------------------------------------
WAYPOINTS = [
    ( 0.8,  0.0,   0.0),
    ( 0.0,  0.8,  90.0),
    (-0.8,  0.0, 180.0),
    ( 0.0, -0.8, 270.0),
    ( 0.0,  0.0,   0.0),   # return to start
]


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # Publisher for AMCL initial pose
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._waypoint_idx = 0
        self._pose_published = 0      # how many times we've published the initial pose
        self._MAX_POSE_PUB = 3        # publish it a few times so AMCL definitely catches it
        self._navigation_started = False

        # Poll every second until Nav2 is reachable, then start the sequence
        self._poll_timer = self.create_timer(1.0, self._poll)
        self.get_logger().info('AutoNav: waiting for Nav2 action server...')

    # ------------------------------------------------------------------
    def _poll(self):
        """Wait for Nav2, then publish initial pose, then start navigation."""
        if not self._nav_client.wait_for_server(timeout_sec=0.1):
            return  # Nav2 not ready yet

        # Publish initial pose up to _MAX_POSE_PUB times (AMCL may need a few)
        if self._pose_published < self._MAX_POSE_PUB:
            self._publish_initial_pose()
            self._pose_published += 1
            return

        # Initial pose sent — cancel poll timer and start navigating after a
        # short delay so AMCL has time to process the pose.
        self._poll_timer.cancel()
        self.get_logger().info('AutoNav: initial pose set — starting in 3 s...')
        self.create_timer(3.0, self._start_navigation)

    # ------------------------------------------------------------------
    def _publish_initial_pose(self):
        x, y, theta_deg = INITIAL_POSE
        theta = math.radians(theta_deg)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        # Standard AMCL initial covariance
        msg.pose.covariance[0]  = 0.25   # x
        msg.pose.covariance[7]  = 0.25   # y
        msg.pose.covariance[35] = 0.068  # theta

        self._initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'AutoNav: published initial pose ({x}, {y}, {theta_deg}°) '
            f'[{self._pose_published + 1}/{self._MAX_POSE_PUB}]')

    # ------------------------------------------------------------------
    def _start_navigation(self):
        self.get_logger().info(
            f'AutoNav: starting route — {len(WAYPOINTS)} waypoints.')
        self._send_next_waypoint()

    # ------------------------------------------------------------------
    def _send_next_waypoint(self):
        if self._waypoint_idx >= len(WAYPOINTS):
            self.get_logger().info('AutoNav: all waypoints completed.')
            return

        x, y, theta_deg = WAYPOINTS[self._waypoint_idx]
        theta = math.radians(theta_deg)

        self.get_logger().info(
            f'AutoNav: → waypoint {self._waypoint_idx + 1}/{len(WAYPOINTS)}: '
            f'({x:.1f}, {y:.1f}, {theta_deg:.0f}°)')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'AutoNav: waypoint {self._waypoint_idx + 1} rejected — skipping.')
            self._waypoint_idx += 1
            self._send_next_waypoint()
            return
        goal_handle.get_result_async().add_done_callback(self._goal_done_cb)

    def _goal_done_cb(self, future):
        self.get_logger().info(
            f'AutoNav: waypoint {self._waypoint_idx + 1} reached.')
        self._waypoint_idx += 1
        self._send_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = AutoNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()