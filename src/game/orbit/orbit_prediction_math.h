#pragma once

#include "core/world.h"
#include "orbitsim/trajectory_types.hpp"

#include <limits>
#include <utility>

namespace Game::OrbitPredictionMath
{
    struct OrbitalElementsEstimate
    {
        bool valid{false};
        double semi_major_axis_m{0.0};
        double eccentricity{0.0};
        double orbital_period_s{0.0};
        double periapsis_m{0.0};
        double apoapsis_m{std::numeric_limits<double>::infinity()};
    };

    double safe_length(const glm::dvec3 &v);

    double estimate_orbital_period_s(double mu_m3_s2, const glm::dvec3 &r_m, const glm::dvec3 &v_mps);

    OrbitalElementsEstimate compute_orbital_elements(double mu_m3_s2,
                                                     const glm::dvec3 &r_m,
                                                     const glm::dvec3 &v_mps);

    std::pair<double, double> select_prediction_horizon_and_dt(double mu_m3_s2,
                                                                const glm::dvec3 &r_m,
                                                                const glm::dvec3 &v_mps);

    WorldVec3 hermite_position_world(const WorldVec3 &ref_body_world,
                                     const orbitsim::TrajectorySample &a,
                                     const orbitsim::TrajectorySample &b,
                                     double t_s);
} // namespace Game::OrbitPredictionMath
