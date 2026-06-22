#pragma once
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>

/*
 * RANSAC Line Extraction (pure geometry, ROS-independent)
 * =======================================================
 * Extracts straight wall segments from a set of 2-D points (the laser scan
 * converted to robot-frame Cartesian).  Each segment is returned in Hesse
 * normal form:
 *
 *      x * cos(alpha) + y * sin(alpha) = rho ,   rho >= 0
 *
 * where alpha is the angle of the line's normal and rho the perpendicular
 * distance from the robot (origin) to the line.  This (alpha, rho) pair is
 * exactly the measurement consumed by the line-based filter update, which is
 * LINEAR in the pose — see wall_detector.hpp.
 *
 * Walls are distinguished from the (round) pillars by a minimum length: a
 * pillar's chord is short, a wall is long, so RANSAC_MIN_LENGTH filters the
 * pillars out automatically.
 *
 * This header is deliberately free of ROS types so the algorithm can be unit
 * tested on synthetic point sets.
 */

struct Pt2 { double x, y; };

struct LineFeature {
    double alpha;     // normal angle [rad], in (-pi, pi]
    double rho;       // perpendicular distance from origin [m], >= 0
    int    inliers;   // number of supporting points
    double length;    // extent of the inlier span along the line [m]
};

// -----------------------------------------------------------------------
// Tuning (defaults; the ROS wrapper exposes these via landmark_map.hpp).
// -----------------------------------------------------------------------
struct RansacParams {
    double inlier_thresh = 0.03;  // max perpendicular distance to count as inlier [m]
    int    min_inliers   = 20;    // min support for a valid line
    double min_length    = 0.50;  // min inlier span [m] -> rejects pillar arcs
    int    max_iters     = 200;   // RANSAC iterations per line
    int    max_lines     = 4;     // stop after this many lines
    unsigned seed        = 12345; // fixed seed -> reproducible extraction
};

// Normalize a Hesse line to rho >= 0 and alpha in (-pi, pi].
inline void normalizeLine(double& alpha, double& rho) {
    if (rho < 0.0) { rho = -rho; alpha += M_PI; }
    while (alpha >  M_PI) alpha -= 2.0 * M_PI;
    while (alpha <= -M_PI) alpha += 2.0 * M_PI;
}

// Perpendicular distance from a point to a Hesse line.
inline double pointLineDist(const Pt2& p, double alpha, double rho) {
    return std::fabs(p.x * std::cos(alpha) + p.y * std::sin(alpha) - rho);
}

// Hesse line through two points (un-refined).
inline void lineFromTwo(const Pt2& a, const Pt2& b, double& alpha, double& rho) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    alpha = std::atan2(dy, dx) + M_PI / 2.0;          // normal = direction + 90 deg
    rho   = a.x * std::cos(alpha) + a.y * std::sin(alpha);
    normalizeLine(alpha, rho);
}

// Total-least-squares Hesse fit over a set of points (eigenvector of scatter).
inline void fitLineTLS(const std::vector<Pt2>& pts, double& alpha, double& rho) {
    const int n = static_cast<int>(pts.size());
    double mx = 0.0, my = 0.0;
    for (const auto& p : pts) { mx += p.x; my += p.y; }
    mx /= n; my /= n;

    double sxx = 0.0, syy = 0.0, sxy = 0.0;
    for (const auto& p : pts) {
        const double dx = p.x - mx, dy = p.y - my;
        sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
    }
    // Normal direction = eigenvector of the smaller eigenvalue of [[sxx,sxy],[sxy,syy]].
    // Closed form: the line orientation is 0.5*atan2(2*sxy, sxx - syy); normal = +90 deg.
    const double theta_line = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    alpha = theta_line + M_PI / 2.0;
    rho   = mx * std::cos(alpha) + my * std::sin(alpha);
    normalizeLine(alpha, rho);
}

// Inlier span (length) of a point set projected along the line direction.
inline double lineLength(const std::vector<Pt2>& pts, double alpha) {
    const double dx = -std::sin(alpha), dy = std::cos(alpha);   // line direction
    double tmin = std::numeric_limits<double>::max();
    double tmax = -std::numeric_limits<double>::max();
    for (const auto& p : pts) {
        const double t = p.x * dx + p.y * dy;
        tmin = std::min(tmin, t); tmax = std::max(tmax, t);
    }
    return tmax - tmin;
}

// Sequential RANSAC: repeatedly find the best-supported line, refit it with
// TLS, accept it if long enough, then remove its inliers and continue.
inline std::vector<LineFeature>
extractLinesFromPoints(std::vector<Pt2> pts, const RansacParams& prm = {})
{
    std::vector<LineFeature> out;
    std::mt19937 rng(prm.seed);

    for (int k = 0; k < prm.max_lines; ++k) {
        const int n = static_cast<int>(pts.size());
        if (n < prm.min_inliers) break;

        std::uniform_int_distribution<int> pick(0, n - 1);
        int    best_count = 0;
        double best_a = 0.0, best_r = 0.0;

        for (int it = 0; it < prm.max_iters; ++it) {
            int i = pick(rng), j = pick(rng);
            if (i == j) continue;
            double a, r;
            lineFromTwo(pts[i], pts[j], a, r);
            int cnt = 0;
            for (const auto& p : pts)
                if (pointLineDist(p, a, r) < prm.inlier_thresh) ++cnt;
            if (cnt > best_count) { best_count = cnt; best_a = a; best_r = r; }
        }

        if (best_count < prm.min_inliers) break;

        // Collect inliers of the best model, then refine with TLS.
        std::vector<Pt2> inl;
        for (const auto& p : pts)
            if (pointLineDist(p, best_a, best_r) < prm.inlier_thresh) inl.push_back(p);

        double a, r;
        fitLineTLS(inl, a, r);

        // Re-collect inliers for the refined line (membership may shift slightly).
        inl.clear();
        std::vector<Pt2> rest;
        for (const auto& p : pts)
            (pointLineDist(p, a, r) < prm.inlier_thresh ? inl : rest).push_back(p);

        const double len = lineLength(inl, a);
        if (static_cast<int>(inl.size()) >= prm.min_inliers && len >= prm.min_length)
            out.push_back({a, r, static_cast<int>(inl.size()), len});

        pts.swap(rest);   // remove consumed points and look for the next wall
    }
    return out;
}
