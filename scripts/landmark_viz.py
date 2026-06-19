#!/usr/bin/env python3
"""
Landmark Visualizer
===================
Subscribes to /scan and /odom, runs the same gating logic as the filter nodes,
and publishes a MarkerArray on /landmark_markers for RViz.

Green sphere + line = landmark detected this scan
Grey  sphere        = landmark not detected
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg

# Must match landmark_map.hpp — keep in sync!
LANDMARK_MAP = [
    ( 0.91, -0.59, "LM_P1"),
    ( 3.08, -0.65, "LM_P2"),
    ( 3.14,  1.53, "LM_P3"),
    ( 0.94,  1.58, "LM_P4"),
]
LM_GATE_M  = 0.30
LM_SIGMA_R = 0.12


def wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a


class LandmarkViz(Node):
    def __init__(self):
        super().__init__('landmark_viz')
        self._pose = (0.0, 0.0, 0.0)   # x, y, theta in odom frame
        self._marker_pub = self.create_publisher(MarkerArray, '/landmark_markers', 10)
        self.create_subscription(Odometry,   '/odom',  self._odom_cb,  10)
        self.create_subscription(LaserScan,  '/scan',  self._scan_cb,  10)
        self.get_logger().info('LandmarkViz started')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._pose = (p.x, p.y, 2.0 * math.atan2(o.z, o.w))

    def _scan_cb(self, msg):
        rx, ry, rth = self._pose
        detected = set()

        for lx, ly, label in LANDMARK_MAP:
            dx    = lx - rx
            dy    = ly - ry
            r_exp = math.sqrt(dx*dx + dy*dy)

            if r_exp < msg.range_min + 0.05 or r_exp > msg.range_max - 0.05:
                continue

            bearing = wrap(math.atan2(dy, dx) - rth)
            if bearing < msg.angle_min or bearing > msg.angle_max:
                continue

            idx = round((bearing - msg.angle_min) / msg.angle_increment)
            if idx < 0 or idx >= len(msg.ranges):
                continue

            r_meas = msg.ranges[idx]
            if not math.isfinite(r_meas):
                continue
            if r_meas < msg.range_min or r_meas > msg.range_max:
                continue
            if abs(r_meas - r_exp) > LM_GATE_M:
                continue

            detected.add(label)
            self.get_logger().info(
                f'[LM] {label}  r_exp={r_exp:.3f}  r_meas={r_meas:.3f}  Δ={abs(r_meas-r_exp):.3f}',
                throttle_duration_sec=1.0)

        self._publish_markers(rx, ry, detected)

    def _publish_markers(self, rx, ry, detected):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 0

        for lx, ly, label in LANDMARK_MAP:
            hit = label in detected

            # Sphere at landmark position
            s = Marker()
            s.header.frame_id = 'odom'
            s.header.stamp    = now
            s.ns, s.id        = 'lm_spheres', mid; mid += 1
            s.type            = Marker.SPHERE
            s.action          = Marker.ADD
            s.pose.position.x = lx
            s.pose.position.y = ly
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.25
            s.color.a = 1.0
            s.color.r = 0.0 if hit else 0.5
            s.color.g = 1.0 if hit else 0.5
            s.color.b = 0.0
            arr.markers.append(s)

            # Line robot → landmark when detected
            if hit:
                l = Marker()
                l.header.frame_id = 'odom'
                l.header.stamp    = now
                l.ns, l.id        = 'lm_lines', mid; mid += 1
                l.type            = Marker.LINE_STRIP
                l.action          = Marker.ADD
                l.scale.x         = 0.03
                l.color.a         = 0.8
                l.color.g         = 1.0
                p1 = geometry_msgs.msg.Point(); p1.x = rx;  p1.y = ry
                p2 = geometry_msgs.msg.Point(); p2.x = lx;  p2.y = ly
                l.points          = [p1, p2]
                arr.markers.append(l)

        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()