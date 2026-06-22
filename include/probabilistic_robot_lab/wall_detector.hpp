#pragma once
#include <vector>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>
#include <utility>
#include "probabilistic_robot_lab/line_ransac.hpp"

/*
 * Wall Detector (RANSAC line features for symmetry breaking)
 * ==========================================================
 * The 9 pillars sit in a translation-periodic 3x3 grid, so pillar matches
 * are locally ambiguous.  The arena WALLS are not periodic: a wall line
 * (alpha, rho) pins down the robot's absolute offset within the grid.
 *
 * Pipeline (mirrors the cylinder detector):
 *   1) extractWalls()    — POSE-INDEPENDENT RANSAC line extraction.
 *   2) associateWalls()  — match each extracted line to a known map wall.
 *
 * The line measurement model used by the filters is LINEAR in the pose:
 *      alpha_pred = alpha_w - theta
 *      rho_pred   = rho_w - (x*cos alpha_w + y*sin alpha_w)
 * so the same constant 2x3 Jacobian is exact for both KF and EKF.
 */

// A known wall, stored in the SAME world frame as LANDMARK_MAP (odom frame),
// in Hesse normal form with rho >= 0.
struct Wall {
    double      alpha;  // normal angle of the wall in world frame [rad]
    double      rho;    // perpendicular distance from world origin [m], >= 0
    const char* name;
};

// Helper: build a wall (alpha, rho) from two points clicked ON the wall
// (e.g. two RViz "Publish Point" clicks in the odom frame).
inline Wall wallFromTwoPoints(double x1, double y1,
                              double x2, double y2, const char* name) {
    double a, r;
    lineFromTwo({x1, y1}, {x2, y2}, a, r);   // already normalized to rho >= 0
    return {a, r, name};
}

// *** Measured via RViz Publish Point in the odom frame.                  ***
// The LEFT vertex of the arena — the unique outward ("normal") corner.
// Its two walls have normals ~60 deg apart, so they are individually
// identifiable, and the (alpha, rho) pair is unique to this corner of the
// whole arena.  Paired with the middle-left pillar (LM_ML in landmark_map.hpp)
// this is an unambiguous anchor that cannot be confused even under noise.
// Add more walls here later for extra coverage; unmatched extractions are
// simply dropped, never mis-applied.
static const std::vector<Wall> WALL_MAP = {
    wallFromTwoPoints( 0.3876,  2.5427,  -0.7877,  0.5404, "W_NW"),  // upper-left wall
    wallFromTwoPoints(-0.7877,  0.5404,   0.3952, -1.5022, "W_SW"),  // lower-left wall
};

// -----------------------------------------------------------------------
// Tuning
// -----------------------------------------------------------------------
static constexpr double WALL_RANSAC_THRESH = 0.03;  // inlier band [m]
static constexpr int    WALL_MIN_INLIERS   = 20;    // min support per line
static constexpr double WALL_MIN_LENGTH    = 0.50;  // min length [m] (rejects pillars)
static constexpr int    WALL_RANSAC_ITERS  = 200;
static constexpr int    WALL_MAX_LINES     = 6;    // hexagonal arena -> 6 walls

static constexpr double WALL_SIGMA_A = 0.05;   // std of measured normal angle [rad]
static constexpr double WALL_SIGMA_R = 0.05;   // std of measured distance [m]

static constexpr double WALL_GATE_A  = 0.30;   // association gate on |d alpha| [rad]
static constexpr double WALL_GATE_R  = 0.50;   // association gate on |d rho|  [m]

// Observation after association: known wall (aw, rw) + the matched, branch-
// aligned measurement (a_meas, r_meas).  The measurement branch is aligned to
// the prediction so the filter update needs NO sign normalization.
struct WallObs { double aw, rw, a_meas, r_meas; };

inline double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// STAGE 1 — RANSAC line extraction from the raw scan (pose-independent).
inline std::vector<LineFeature>
extractWalls(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
    std::vector<Pt2> pts;
    pts.reserve(msg->ranges.size());
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const float r = msg->ranges[i];
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
        const double a = msg->angle_min + i * msg->angle_increment;
        pts.push_back({r * std::cos(a), r * std::sin(a)});
    }
    RansacParams prm;
    prm.inlier_thresh = WALL_RANSAC_THRESH;
    prm.min_inliers   = WALL_MIN_INLIERS;
    prm.min_length    = WALL_MIN_LENGTH;
    prm.max_iters     = WALL_RANSAC_ITERS;
    prm.max_lines     = WALL_MAX_LINES;
    return extractLinesFromPoints(std::move(pts), prm);
}

// STAGE 2 — Associate each extracted line to a known map wall using the
// current pose estimate (matching only).
inline std::vector<WallObs>
associateWalls(const std::vector<LineFeature>& lines,
               double x, double y, double theta)
{
    std::vector<WallObs> out;
    for (const auto& L : lines) {
        int    best  = -1;
        double bestc = std::numeric_limits<double>::max();
        double bam = 0.0, brm = 0.0;

        for (size_t j = 0; j < WALL_MAP.size(); ++j) {
            const double aw = WALL_MAP[j].alpha, rw = WALL_MAP[j].rho;
            // Predicted robot-frame line (un-normalized, signed rho).
            const double ap = aw - theta;
            const double rp = rw - (x * std::cos(aw) + y * std::sin(aw));

            // Align the measured line to the prediction's branch
            // (a line and its pi-flip with negated rho are identical).
            double am = L.alpha, rm = L.rho;
            const double d_keep = std::fabs(wrapPi(am          - ap));
            const double d_flip = std::fabs(wrapPi(am + M_PI    - ap));
            if (d_flip < d_keep) { am = am + M_PI; rm = -rm; }

            const double da = wrapPi(am - ap);
            const double dr = rm - rp;
            if (std::fabs(da) > WALL_GATE_A || std::fabs(dr) > WALL_GATE_R) continue;

            const double cost = (da / WALL_GATE_A) * (da / WALL_GATE_A)
                              + (dr / WALL_GATE_R) * (dr / WALL_GATE_R);
            if (cost < bestc) { bestc = cost; best = static_cast<int>(j); bam = am; brm = rm; }
        }

        if (best < 0) continue;  // no map wall matched -> drop
        out.push_back({WALL_MAP[best].alpha, WALL_MAP[best].rho, bam, brm});
    }
    return out;
}
