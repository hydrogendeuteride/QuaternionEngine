#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>

namespace Game::OrbitPredictionMath
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
    } // namespace

    double safe_length(const glm::dvec3 &v)
    {
        const double len2 = glm::dot(v, v);
        if (!std::isfinite(len2) || len2 <= 0.0)
        {
            return 0.0;
        }
        return std::sqrt(len2);
    }

    double estimate_orbital_period_s(const double mu_m3_s2, const glm::dvec3 &r_m, const glm::dvec3 &v_mps)
    {
        const double r = safe_length(r_m);
        const double v = safe_length(v_mps);
        if (!(mu_m3_s2 > 0.0) || !(r > 0.0) || !std::isfinite(mu_m3_s2) || !std::isfinite(r) || !std::isfinite(v))
        {
            return OrbitPredictionTuning::kEscapeDefaultPeriodS;
        }

        const double specific_energy = 0.5 * (v * v) - mu_m3_s2 / r;
        if (!std::isfinite(specific_energy) || specific_energy >= 0.0)
        {
            return OrbitPredictionTuning::kEscapeDefaultPeriodS;
        }

        const double a_m = -mu_m3_s2 / (2.0 * specific_energy);
        if (!(a_m > 0.0) || !std::isfinite(a_m))
        {
            return OrbitPredictionTuning::kEscapeDefaultPeriodS;
        }

        const double period_s = 2.0 * kPi * std::sqrt((a_m * a_m * a_m) / mu_m3_s2);
        if (!std::isfinite(period_s) || period_s <= 0.0)
        {
            return OrbitPredictionTuning::kEscapeDefaultPeriodS;
        }
        return period_s;
    }

    OrbitalElementsEstimate compute_orbital_elements(const double mu_m3_s2, const glm::dvec3 &r_m,
                                                     const glm::dvec3 &v_mps)
    {
        OrbitalElementsEstimate out{};
        if (!(mu_m3_s2 > 0.0) || !std::isfinite(mu_m3_s2))
        {
            return out;
        }

        const double r = safe_length(r_m);
        const double v2 = glm::dot(v_mps, v_mps);
        if (!(r > 0.0) || !std::isfinite(r) || !std::isfinite(v2))
        {
            return out;
        }

        const glm::dvec3 h = glm::cross(r_m, v_mps);
        const double h2 = glm::dot(h, h);

        const glm::dvec3 e_vec = (glm::cross(v_mps, h) / mu_m3_s2) - (r_m / r);
        const double e = safe_length(e_vec);
        if (!std::isfinite(e))
        {
            return out;
        }

        const double specific_energy = 0.5 * v2 - mu_m3_s2 / r;
        if (!std::isfinite(specific_energy))
        {
            return out;
        }

        out.eccentricity = std::max(0.0, e);

        if (std::abs(specific_energy) > 1e-12)
        {
            out.semi_major_axis_m = -mu_m3_s2 / (2.0 * specific_energy);
        }

        if (out.semi_major_axis_m > 0.0 && std::isfinite(out.semi_major_axis_m) && out.eccentricity < 1.0)
        {
            out.orbital_period_s = 2.0 * kPi *
                                   std::sqrt((out.semi_major_axis_m * out.semi_major_axis_m *
                                              out.semi_major_axis_m) /
                                             mu_m3_s2);
            out.periapsis_m = out.semi_major_axis_m * (1.0 - out.eccentricity);
            out.apoapsis_m = out.semi_major_axis_m * (1.0 + out.eccentricity);
        }
        else if (h2 > 0.0 && std::isfinite(h2))
        {
            const double denom = mu_m3_s2 * (1.0 + out.eccentricity);
            if (denom > 0.0 && std::isfinite(denom))
            {
                out.periapsis_m = h2 / denom;
            }
            out.orbital_period_s = 0.0;
            out.apoapsis_m = std::numeric_limits<double>::infinity();
        }

        if (!std::isfinite(out.periapsis_m) || out.periapsis_m <= 0.0)
        {
            out.periapsis_m = r;
        }

        out.valid = true;
        return out;
    }

    std::pair<double, double> select_prediction_horizon_and_dt(const double mu_m3_s2, const glm::dvec3 &r_m,
                                                                const glm::dvec3 &v_mps)
    {
        const double period_s = estimate_orbital_period_s(mu_m3_s2, r_m, v_mps);
        const double horizon_s = std::clamp(period_s * OrbitPredictionTuning::kBaseHorizonFromPeriodScale,
                                            OrbitPredictionTuning::kMinHorizonS,
                                            OrbitPredictionTuning::kMaxHorizonS);
        const double target_samples = std::clamp(horizon_s / OrbitPredictionTuning::kTargetSamplesDivisorS,
                                                 OrbitPredictionTuning::kTargetSamplesMin,
                                                 OrbitPredictionTuning::kTargetSamplesMax);
        const double dt_s = std::clamp(horizon_s / target_samples, 0.01, OrbitPredictionTuning::kMaxSampleDtS);
        return {horizon_s, dt_s};
    }

    WorldVec3 hermite_position_world(const WorldVec3 &ref_body_world,
                                     const orbitsim::TrajectorySample &a,
                                     const orbitsim::TrajectorySample &b,
                                     const double t_s)
    {
        const double ta = a.t_s;
        const double tb = b.t_s;
        const double h = tb - ta;
        if (!std::isfinite(h) || !(h > 0.0))
        {
            return ref_body_world + WorldVec3(a.position_m);
        }

        double u = (t_s - ta) / h;
        if (!std::isfinite(u))
        {
            u = 0.0;
        }
        u = std::clamp(u, 0.0, 1.0);

        const double u2 = u * u;
        const double u3 = u2 * u;

        const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
        const double h10 = u3 - (2.0 * u2) + u;
        const double h01 = (-2.0 * u3) + (3.0 * u2);
        const double h11 = u3 - u2;

        const glm::dvec3 p0 = glm::dvec3(a.position_m);
        const glm::dvec3 p1 = glm::dvec3(b.position_m);
        const glm::dvec3 m0 = glm::dvec3(a.velocity_mps) * h;
        const glm::dvec3 m1 = glm::dvec3(b.velocity_mps) * h;

        const glm::dvec3 p = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
        return ref_body_world + WorldVec3(p);
    }
} // namespace Game::OrbitPredictionMath
