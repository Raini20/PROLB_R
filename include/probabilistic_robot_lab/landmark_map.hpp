#pragma once
#include <vector>

/*
 * Shared Landmark Map
 * ===================
 * Included by KF, EKF, and PF nodes.
 * All three filters use identical landmark positions so results are
 * directly comparable ("Same input data for all filters").
 *
 * A landmark is a known, fixed reference point in the world (map) frame.
 * The measurement model used here is range-only (1-D):
 *   h(mu, m_j) = || m_j - [x, y] ||
 *
 * The scan callback in each node computes the expected bearing from the
 * current pose estimate, samples the LaserScan at that angle, and compares
 * the returned range to the expected range (gating).  Only the range is
 * used as the actual measurement — bearing is used solely for indexing
 * into the scan array.
 *
 * How to set landmark positions
 * ------------------------------
 *   1. Run the TB3 Nav2 simulation and open RViz2.
 *   2. Add a "Clicked Point" display or use Panels → Tools → Publish Point.
 *   3. Click on distinct, scan-visible obstacles (wall corners, cylinders…).
 *      The (x, y) coordinates appear on /clicked_point.
 *   4. Replace the values below and rebuild.
 *
 * Changing LANDMARK_MAP here automatically updates all three nodes.
 */

struct Landmark {
    double      x;      // world-frame x [m]
    double      y;      // world-frame y [m]
    const char* name;   // debug label (printed in RCLCPP_DEBUG)
};

// *** Adjust these positions to match your simulation map! ***
static const std::vector<Landmark> LANDMARK_MAP = {
    { 0.91, -0.59, "LM_P1"},
    { 0.9175, 0.4675, "LM_ML"},   // middle-left pillar, paired with the left corner
    //{ 3.08, -0.65, "LM_P2"},
    //{ 3.14,  1.53, "LM_P3"},
    //{ 0.94,  1.58, "LM_P4"},
};

// -----------------------------------------------------------------------
// Tuning parameters (shared by all nodes)
// -----------------------------------------------------------------------

// Max |r_measured - r_expected| [m] to accept a scan return as a detection.
// Increase if landmarks are rarely detected; decrease if false detections occur.
static constexpr double LM_GATE_M  = 0.30;

// Standard deviation of the range measurement [m].
// Q_lm = LM_SIGMA_R^2  (scalar noise variance for the 1-D range observation).
static constexpr double LM_SIGMA_R = 0.12;

// -----------------------------------------------------------------------
// Cylinder detector parameters (used by landmark_detector.hpp)
// -----------------------------------------------------------------------

// Radius of a pillar [m].  The laser measures the SURFACE; the map stores
// the CENTER, so the detector adds this to get a centre range.
// *** SET THIS TO YOUR WORLD'S PILLAR RADIUS. ***
static constexpr double LM_RADIUS      = 0.15;

// Max range jump between adjacent beams [m] before a cluster is split.
// Larger -> clusters merge across gaps; smaller -> pillars fragment.
static constexpr double LM_CLUSTER_GAP = 0.10;

// Minimum number of beams a cluster must contain to be considered.
static constexpr int    LM_MIN_PTS     = 3;

// Accepted physical chord width of a cluster [m].  Pillars are narrow;
// walls produce wide clusters and are rejected by the upper bound.
static constexpr double LM_MIN_WIDTH   = 0.05;
static constexpr double LM_MAX_WIDTH   = 0.45;

// Max distance [m] between a detected pillar (projected with the current
// pose) and a known map landmark to accept the association.
static constexpr double LM_ASSOC_GATE  = 0.60;
// Min gap [m] between the nearest and second-nearest map landmark for an
// association to count as unambiguous.  With identical, closely-spaced
// pillars this is the guard against the estimate jumping between them.
// Rule of thumb: < (minimum pillar spacing) - LM_ASSOC_GATE.
static constexpr double LM_ASSOC_MARGIN = 0.40;