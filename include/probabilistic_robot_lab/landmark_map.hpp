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
    { 3.1,  1.0, "LM_N"},
    { 2.0,  2.1, "LM_W"},
    { 0.9,  1.0, "LM_S"},
    //{ 2.0, -0.1, "LM_E"},
};

// -----------------------------------------------------------------------
// Tuning parameters (shared by all nodes)
// -----------------------------------------------------------------------

// Max |r_measured - r_expected| [m] to accept a scan return as a detection.
// Increase if landmarks are rarely detected; decrease if false detections occur.
static constexpr double LM_GATE_M  = 0.80;

// Standard deviation of the range measurement [m].
// Q_lm = LM_SIGMA_R^2  (scalar noise variance for the 1-D range observation).
static constexpr double LM_SIGMA_R = 0.05;