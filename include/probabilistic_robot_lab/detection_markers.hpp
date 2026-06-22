#pragma once
#include <cmath>
#include <string>
#include <vector>
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/time.hpp"
#include "probabilistic_robot_lab/landmark_map.hpp"
#include "probabilistic_robot_lab/landmark_detector.hpp"
#include "probabilistic_robot_lab/wall_detector.hpp"

/*
 * Detection Markers — shared RViz visualization for KF/EKF/PF
 * =============================================================
 * Replaces the old standalone landmark_viz.py, which had two problems:
 *   1) It reimplemented its own (stale, pre-rearchitecture) detection
 *      logic using /odom directly — not what the filters actually see.
 *   2) It only added the "hit" line marker conditionally, so once drawn
 *      it was never cleared when detection later failed (no DELETE was
 *      ever sent for that marker id) — markers froze on screen forever.
 *
 * This header fixes both: it draws directly from each node's own
 * detected_/walls_ (the exact data used for the correction step), and
 * it publishes EVERY marker id on EVERY call — DELETE when not hit,
 * ADD/MODIFY when hit — so RViz state can never go stale. A short
 * lifetime is set as an extra safety net in case a node stops publishing.
 */

// How far to extend a wall marker on each side of the robot's perpendicular
// foot point on the (infinite) wall line. Purely visual, no effect on data.
static constexpr double WALL_MARKER_HALF_LEN = 1.2;

inline visualization_msgs::msg::MarkerArray
buildDetectionMarkers(double rx, double ry,
                       const std::vector<LandmarkObs>& detected,
                       const std::vector<WallObs>& walls,
                       const std::string& ns_prefix,
                       float hit_r, float hit_g, float hit_b,
                       const rclcpp::Time& stamp)
{
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray arr;
    int mid = 0;
    static constexpr double MARKER_LIFETIME_SEC = 0.5;

    auto makeHeader = [&](Marker& m) {
        m.header.frame_id = "odom";
        m.header.stamp     = stamp;
        m.lifetime         = rclcpp::Duration::from_seconds(MARKER_LIFETIME_SEC);
    };

    // ---- Landmarks: one sphere (always) + one line (hit only) per entry ----
    for (const auto& lm : LANDMARK_MAP) {
        bool hit = false;
        for (const auto& d : detected) {
            if (d.mx == lm.x && d.my == lm.y) { hit = true; break; }
        }

        Marker s; makeHeader(s);
        s.ns = ns_prefix + "_lm_spheres"; s.id = mid++;
        s.type   = Marker::SPHERE;
        s.action = Marker::ADD;
        s.pose.position.x = lm.x;
        s.pose.position.y = lm.y;
        s.pose.orientation.w = 1.0;
        s.scale.x = s.scale.y = s.scale.z = 0.22;
        s.color.a = 1.0;
        if (hit) { s.color.r = hit_r; s.color.g = hit_g; s.color.b = hit_b; }
        else     { s.color.r = s.color.g = s.color.b = 0.5f; }
        arr.markers.push_back(s);

        Marker l; makeHeader(l);
        l.ns = ns_prefix + "_lm_lines"; l.id = mid++;
        l.type   = Marker::LINE_STRIP;
        l.action = hit ? Marker::ADD : Marker::DELETE;   // <-- explicit clear
        if (hit) {
            l.scale.x = 0.025;
            l.color.a = 0.8; l.color.r = hit_r; l.color.g = hit_g; l.color.b = hit_b;
            geometry_msgs::msg::Point p1, p2;
            p1.x = rx; p1.y = ry;
            p2.x = lm.x; p2.y = lm.y;
            l.points = {p1, p2};
        }
        arr.markers.push_back(l);
    }

    // ---- Walls: one segment (always) + one robot->foot line (hit only) ----
    for (const auto& w : WALL_MAP) {
        bool hit = false;
        for (const auto& wo : walls) {
            if (wo.aw == w.alpha && wo.rw == w.rho) { hit = true; break; }
        }

        const double ca = std::cos(w.alpha), sa = std::sin(w.alpha);
        const double d  = rx * ca + ry * sa - w.rho;     // signed dist robot->line
        const double fx = rx - d * ca, fy = ry - d * sa; // foot of perpendicular
        const double dx = -sa, dy = ca;                  // unit dir along the line

        Marker seg; makeHeader(seg);
        seg.ns = ns_prefix + "_walls"; seg.id = mid++;
        seg.type   = Marker::LINE_STRIP;
        seg.action = Marker::ADD;   // segment itself always drawn (known map wall)
        seg.scale.x = 0.04;
        seg.color.a = hit ? 0.9f : 0.30f;
        if (hit) { seg.color.r = hit_r; seg.color.g = hit_g; seg.color.b = hit_b; }
        else     { seg.color.r = seg.color.g = seg.color.b = 0.5f; }
        geometry_msgs::msg::Point p1, p2;
        p1.x = fx - WALL_MARKER_HALF_LEN * dx; p1.y = fy - WALL_MARKER_HALF_LEN * dy;
        p2.x = fx + WALL_MARKER_HALF_LEN * dx; p2.y = fy + WALL_MARKER_HALF_LEN * dy;
        seg.points = {p1, p2};
        arr.markers.push_back(seg);

        Marker l; makeHeader(l);
        l.ns = ns_prefix + "_wall_lines"; l.id = mid++;
        l.type   = Marker::LINE_STRIP;
        l.action = hit ? Marker::ADD : Marker::DELETE;   // <-- explicit clear
        if (hit) {
            l.scale.x = 0.02;
            l.color.a = 0.6; l.color.r = hit_r; l.color.g = hit_g; l.color.b = hit_b;
            geometry_msgs::msg::Point q1, q2;
            q1.x = rx; q1.y = ry;
            q2.x = fx; q2.y = fy;
            l.points = {q1, q2};
        }
        arr.markers.push_back(l);
    }

    return arr;
}
