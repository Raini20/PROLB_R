#pragma once
#include <cmath>
#include <vector>
// landmark_detector.hpp includes landmark_map.hpp internally, so
// including it here gives us LandmarkObs + LANDMARK_MAP in one shot.
#include "probabilistic_robot_lab/landmark_detector.hpp"
#include "probabilistic_robot_lab/wall_detector.hpp"

/*
 * anchor_detected() — combined feature gate
 * ==========================================
 * Returns true only when the anchor pillar (LM_ML) AND at least one corner
 * wall are simultaneously present in the detection lists.
 *
 * Rationale:
 *   - Pillar alone: any of the 9 identical pillars could be matched via the
 *     current pose estimate; the association is pose-dependent.
 *   - Wall alone: RANSAC extracts a line in robot-frame; mapping it to a
 *     specific map wall (via alpha/rho comparison) also uses the pose prior.
 *   - Combined: there is exactly one location in the arena where a cylinder
 *     and two wall segments with these Hesse parameters co-occur.  The
 *     correction is therefore not purely relying on the prior for identity.
 *
 * All three filter nodes include this header and gate every correction on it.
 */
inline bool anchor_detected(const std::vector<LandmarkObs>& lm_obs,
                            const std::vector<WallObs>&     wall_obs)
{
    if (wall_obs.empty()) return false;
    constexpr double LM_ML_X = 0.9175, LM_ML_Y = 0.4675, EPS = 0.01;
    for (const auto& o : lm_obs)
        if (std::fabs(o.mx - LM_ML_X) < EPS && std::fabs(o.my - LM_ML_Y) < EPS)
            return true;
    return false;
}
