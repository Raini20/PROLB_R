#pragma once
#include <vector>
#include <cmath>
#include <limits>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "probabilistic_robot_lab/landmark_map.hpp"

/*
 * Shared Landmark Detector
 * ========================
 * Included by KF, EKF, and PF nodes so all three filters use *identical*
 * landmark observations ("same input data for all filters").
 *
 * Two clearly separated stages — this separation is the whole point:
 *
 *   1) detectCylinders()    — POSE-INDEPENDENT feature extraction.
 *        A pillar is detected because the laser actually sees a compact,
 *        free-standing object, NOT because the current estimate expects one
 *        to be there.  The pose estimate mu_ is never used here.
 *
 *   2) associateLandmarks() — POSE-DEPENDENT data association.
 *        Each detected cylinder is projected into the world frame using the
 *        current pose estimate and matched to the NEAREST known map landmark.
 *        The pose is used only to decide *which* known pillar a real
 *        detection corresponds to.  If no map landmark is close enough the
 *        detection is rejected (unknown feature) instead of silently
 *        confirming a wrong pose.
 *
 * The measurement model in the filter nodes is range-only:
 *   h(mu, m_j) = || m_j - [x, y] ||   (range to the pillar CENTER)
 * so the detector converts the measured surface range to a centre range by
 * adding the known pillar radius (LM_RADIUS).  bearing is returned as well in
 * case a 2-D (range+bearing) update is added later.
 */

// Raw cylinder detection, expressed in the ROBOT frame.
struct CylinderObs {
    double range;    // distance to the estimated pillar CENTER [m]
    double bearing;  // bearing of the pillar center in robot frame [rad]
};

// Observation after association to a known map landmark.
// (Same fields the filter nodes already expect: mx, my, range.)
struct LandmarkObs {
    double mx;     // world x of the associated landmark [m]
    double my;     // world y of the associated landmark [m]
    double range;  // measured range to that landmark CENTER [m]
};

// Local angle wrap (named *_ld to avoid clashing with each node's wrapAngle).
inline double wrapAngle_ld(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// -----------------------------------------------------------------------
// STAGE 1 — Feature extraction (POSE-INDEPENDENT)
// -----------------------------------------------------------------------
// Segment the scan into clusters of neighbouring beams (split on range
// discontinuities), then keep only the clusters that look like a
// free-standing pillar:
//   * compact  : physical chord width within [LM_MIN_WIDTH, LM_MAX_WIDTH]
//   * isolated : a discontinuity ended the cluster on both sides
//                (walls produce wide clusters and are filtered by width)
// NOTE: a cluster straddling the 0/2pi wrap of a 360° lidar is not merged
// here; acceptable for a lab, but worth a sentence in the report.
inline std::vector<CylinderObs>
detectCylinders(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
    std::vector<CylinderObs> out;
    const int N = static_cast<int>(msg->ranges.size());
    if (N == 0) return out;

    int i = 0;
    while (i < N) {
        float r = msg->ranges[i];
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) { ++i; continue; }

        // Grow a cluster of contiguous beams with small range jumps.
        const int start = i;
        double sum_r = 0.0;
        int    cnt   = 0;
        float  prev  = r;
        while (i < N) {
            float ri = msg->ranges[i];
            if (!std::isfinite(ri) || ri < msg->range_min || ri > msg->range_max) break;
            if (std::abs(ri - prev) > LM_CLUSTER_GAP) break;  // discontinuity -> new object
            sum_r += ri; ++cnt; prev = ri; ++i;
        }
        const int end = i - 1;

        if (cnt < LM_MIN_PTS) continue;                  // too few points to trust

        // Cluster geometry: mean range, angular span, physical chord width.
        const double r_mean = sum_r / cnt;
        const double ang_w  = (end - start) * msg->angle_increment;
        const double width  = 2.0 * r_mean * std::sin(ang_w / 2.0);   // chord [m]
        if (width < LM_MIN_WIDTH || width > LM_MAX_WIDTH) continue;    // not a pillar

        // Bearing of the cluster centre.
        const double bearing =
            wrapAngle_ld(msg->angle_min + 0.5 * (start + end) * msg->angle_increment);

        // Surface range -> centre range (laser hits the front of the pillar).
        const double range_center = r_mean + LM_RADIUS;

        out.push_back({range_center, bearing});
    }
    return out;
}

// -----------------------------------------------------------------------
// STAGE 2 — Data association (POSE-DEPENDENT, matching only)
// -----------------------------------------------------------------------
// Project each detected pillar centre into the world frame with the current
// pose estimate (x, y, theta) and match it to the nearest known landmark.
//
// With many identical pillars the danger is not detection but AMBIGUOUS
// assignment: a detection can fall near two map pillars at once.  We accept
// a match only if
//   * the nearest landmark is within LM_ASSOC_GATE, AND
//   * the SECOND nearest is at least LM_ASSOC_MARGIN farther away.
// Otherwise the detection is ambiguous and dropped — skipping a correction
// is far cheaper than locking onto the wrong pillar.  This is what keeps the
// estimate from "jumping" between identical pillars.
inline std::vector<LandmarkObs>
associateLandmarks(const std::vector<CylinderObs>& dets,
                   double x, double y, double theta)
{
    std::vector<LandmarkObs> out;
    for (const auto& d : dets) {
        // Pillar centre in world coordinates (uses the estimate only here).
        const double wx = x + d.range * std::cos(theta + d.bearing);
        const double wy = y + d.range * std::sin(theta + d.bearing);

        // Track the nearest and second-nearest map landmark.
        int    best      = -1;
        double best_d2   = std::numeric_limits<double>::max();
        double second_d2 = std::numeric_limits<double>::max();
        for (size_t j = 0; j < LANDMARK_MAP.size(); ++j) {
            const double dx = LANDMARK_MAP[j].x - wx;
            const double dy = LANDMARK_MAP[j].y - wy;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2)        { second_d2 = best_d2; best_d2 = d2; best = static_cast<int>(j); }
            else if (d2 < second_d2) { second_d2 = d2; }
        }

        if (best < 0) continue;                              // empty map
        if (best_d2 > LM_ASSOC_GATE * LM_ASSOC_GATE) continue; // nothing close enough

        // Ambiguity gate: nearest must clearly win over the runner-up.
        const double best_d   = std::sqrt(best_d2);
        const double second_d = std::sqrt(second_d2);
        if (second_d - best_d < LM_ASSOC_MARGIN) continue;   // ambiguous -> drop

        out.push_back({LANDMARK_MAP[best].x, LANDMARK_MAP[best].y, d.range});
    }
    return out;
}
