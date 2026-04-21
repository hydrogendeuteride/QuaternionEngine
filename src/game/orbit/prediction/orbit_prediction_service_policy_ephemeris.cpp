#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/detail/spacecraft_propagation.hpp"
#include "orbitsim/integrators.hpp"
#include "orbitsim/trajectories.hpp"

namespace Game
{
    namespace
    {
        constexpr double kEphemerisStatePositionToleranceFloorM = 1.0e-3;
        constexpr double kEphemerisStateVelocityToleranceFloorMps = 1.0e-6;
        constexpr double kEphemerisSpinAxisTolerance = 1.0e-9;
        constexpr double kEphemerisSpinAngleToleranceRad = 1.0e-9;
        constexpr double kEphemerisSpinRateToleranceRadPerS = 1.0e-12;

        bool tolerance_covers_request(const orbitsim::AdaptiveToleranceRamp &entry,
                                      const orbitsim::AdaptiveToleranceRamp &request)
        {
            constexpr double kTolEpsilon = 1.0e-12;
            return entry.pos_near_m <= (request.pos_near_m + kTolEpsilon) &&
                   entry.pos_far_m <= (request.pos_far_m + kTolEpsilon) &&
                   entry.vel_near_mps <= (request.vel_near_mps + kTolEpsilon) &&
                   entry.vel_far_mps <= (request.vel_far_mps + kTolEpsilon) &&
                   entry.rel_pos_floor <= (request.rel_pos_floor + kTolEpsilon) &&
                   entry.rel_vel_floor <= (request.rel_vel_floor + kTolEpsilon);
        }

        bool adaptive_ephemeris_options_cover_request(const orbitsim::AdaptiveEphemerisOptions &entry,
                                                      const orbitsim::AdaptiveEphemerisOptions &request)
        {
            return entry.min_dt_s <= (request.min_dt_s + kEphemerisDtEpsilonS) &&
                   entry.max_dt_s <= (request.max_dt_s + kEphemerisDtEpsilonS) &&
                   entry.soft_max_segments >= request.soft_max_segments &&
                   entry.hard_max_segments >= request.hard_max_segments &&
                   tolerance_covers_request(entry.tolerance, request.tolerance);
        }

        bool same_config_for_ephemeris(const orbitsim::GameSimulation::Config &a,
                                       const orbitsim::GameSimulation::Config &b)
        {
            return a.gravitational_constant == b.gravitational_constant &&
                   a.softening_length_m == b.softening_length_m;
        }

        double ephemeris_state_position_tolerance_m(const orbitsim::AdaptiveEphemerisOptions &request_options)
        {
            return std::max({kEphemerisStatePositionToleranceFloorM,
                             request_options.tolerance.pos_near_m,
                             request_options.tolerance.pos_far_m});
        }

        double ephemeris_state_velocity_tolerance_mps(const orbitsim::AdaptiveEphemerisOptions &request_options)
        {
            return std::max({kEphemerisStateVelocityToleranceFloorMps,
                             request_options.tolerance.vel_near_mps,
                             request_options.tolerance.vel_far_mps});
        }

        bool vec3_within_tolerance(const orbitsim::Vec3 &a,
                                   const orbitsim::Vec3 &b,
                                   const double tolerance)
        {
            return finite_vec3(a) &&
                   finite_vec3(b) &&
                   glm::length(glm::dvec3(a - b)) <= tolerance;
        }

        bool spin_state_matches_ephemeris_sample(const orbitsim::SpinState &sample,
                                                 const orbitsim::SpinState &request)
        {
            return vec3_within_tolerance(sample.axis, request.axis, kEphemerisSpinAxisTolerance) &&
                   std::abs(sample.angle_rad - request.angle_rad) <= kEphemerisSpinAngleToleranceRad &&
                   std::abs(sample.rate_rad_per_s - request.rate_rad_per_s) <= kEphemerisSpinRateToleranceRadPerS;
        }

        bool state_matches_ephemeris_sample(const orbitsim::State &sample,
                                            const orbitsim::State &request,
                                            const orbitsim::AdaptiveEphemerisOptions &request_options)
        {
            return vec3_within_tolerance(sample.position_m,
                                         request.position_m,
                                         ephemeris_state_position_tolerance_m(request_options)) &&
                   vec3_within_tolerance(sample.velocity_mps,
                                         request.velocity_mps,
                                         ephemeris_state_velocity_tolerance_mps(request_options)) &&
                   spin_state_matches_ephemeris_sample(sample.spin, request.spin);
        }

        bool cached_ephemeris_covers_request(const OrbitPredictionService::CachedEphemerisEntry &entry,
                                             const OrbitPredictionService::EphemerisBuildRequest &request)
        {
            if (!entry.ephemeris || entry.ephemeris->empty())
            {
                return false;
            }

            const double request_end_s = request.sim_time_s + request.duration_s;
            if (!std::isfinite(request.sim_time_s) ||
                !std::isfinite(request_end_s) ||
                !(request.duration_s > 0.0))
            {
                return false;
            }

            const double entry_start_s = entry.ephemeris->t0_s();
            const double entry_end_s = entry.ephemeris->t_end_s();
            return std::isfinite(entry_start_s) &&
                   std::isfinite(entry_end_s) &&
                   entry_start_s <= (request.sim_time_s + kEphemerisDurationEpsilonS) &&
                   (entry_end_s + kEphemerisDurationEpsilonS) >= request_end_s;
        }

        bool cached_massive_body_set_matches_request(
                const OrbitPredictionService::CachedEphemerisEntry &entry,
                const OrbitPredictionService::EphemerisBuildRequest &request)
        {
            if (!entry.ephemeris ||
                entry.massive_bodies.size() != request.massive_bodies.size() ||
                entry.ephemeris->body_ids.size() != request.massive_bodies.size())
            {
                return false;
            }

            for (std::size_t i = 0; i < request.massive_bodies.size(); ++i)
            {
                const orbitsim::MassiveBody &entry_body = entry.massive_bodies[i];
                const orbitsim::MassiveBody &request_body = request.massive_bodies[i];
                if (entry_body.id != request_body.id ||
                    entry.ephemeris->body_ids[i] != request_body.id ||
                    entry_body.mass_kg != request_body.mass_kg)
                {
                    return false;
                }

                const orbitsim::State cached_state = entry.ephemeris->body_state_at(i, request.sim_time_s);
                if (!state_matches_ephemeris_sample(cached_state, request_body.state, request.adaptive_options))
                {
                    return false;
                }
            }

            return true;
        }
    } // namespace

    bool ephemeris_covers_horizon(const OrbitPredictionService::SharedCelestialEphemeris &ephemeris,
                                  const double sim_time_s,
                                  const double horizon_s)
    {
        return ephemeris &&
               !ephemeris->empty() &&
               ephemeris->t0_s() <= (sim_time_s + kEphemerisDurationEpsilonS) &&
               (ephemeris->t_end_s() + kEphemerisDurationEpsilonS) >= (sim_time_s + horizon_s);
    }

    bool compatible_cached_ephemeris(const OrbitPredictionService::CachedEphemerisEntry &entry,
                                     const OrbitPredictionService::EphemerisBuildRequest &request)
    {
        if (!entry.ephemeris || entry.ephemeris->empty())
        {
            return false;
        }

        if (!cached_ephemeris_covers_request(entry, request))
        {
            return false;
        }

        if (!same_config_for_ephemeris(entry.sim_config, request.sim_config))
        {
            return false;
        }

        if (!cached_massive_body_set_matches_request(entry, request))
        {
            return false;
        }

        if (!adaptive_ephemeris_options_cover_request(entry.adaptive_options, request.adaptive_options))
        {
            return false;
        }

        return true;
    }

    OrbitPredictionService::SharedCelestialEphemeris build_ephemeris_from_request(
            const OrbitPredictionService::EphemerisBuildRequest &request,
            const CancelCheck &cancel_requested,
            orbitsim::AdaptiveEphemerisDiagnostics *out_diag)
    {
        if (!std::isfinite(request.sim_time_s) ||
            !(request.duration_s > 0.0) ||
            !(request.adaptive_options.duration_s > 0.0))
        {
            return {};
        }

        orbitsim::GameSimulation sim(request.sim_config);
        if (!sim.set_time_s(request.sim_time_s))
        {
            return {};
        }

        if (cancel_requested && cancel_requested())
        {
            return {};
        }

        for (const orbitsim::MassiveBody &body : request.massive_bodies)
        {
            const auto body_handle =
                    (body.id != orbitsim::kInvalidBodyId)
                            ? sim.create_body_with_id(body.id, body)
                            : sim.create_body(body);
            if (!body_handle.valid())
            {
                return {};
            }
        }

        orbitsim::AdaptiveEphemerisOptions adaptive_opt = request.adaptive_options;
        adaptive_opt.cancel_requested = cancel_requested;
        adaptive_opt.duration_s = request.duration_s;

        auto ephemeris = std::make_shared<orbitsim::CelestialEphemeris>(
                orbitsim::build_celestial_ephemeris_adaptive(sim, adaptive_opt, out_diag));
        if (!ephemeris || ephemeris->empty())
        {
            return {};
        }

        return ephemeris;
    }

    OrbitPredictionService::EphemerisBuildRequest build_ephemeris_build_request(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec)
    {
        OrbitPredictionService::EphemerisBuildRequest out{};
        out.sim_time_s = request.sim_time_s;
        out.sim_config = request.sim_config;
        out.massive_bodies = request.massive_bodies;
        out.duration_s = sampling_spec.horizon_s;
        out.adaptive_options = build_adaptive_ephemeris_options(request, sampling_spec);
        return out;
    }

    CelestialPredictionSamplingSpec build_celestial_prediction_sampling_spec(
            const OrbitPredictionService::Request &request,
            const orbitsim::MassiveBody &subject_body)
    {
        CelestialPredictionSamplingSpec out{};
        if (request.massive_bodies.size() < 2)
        {
            return out;
        }

        std::vector<orbitsim::MassiveBody> candidate_bodies;
        candidate_bodies.reserve(request.massive_bodies.size() - 1);
        for (const orbitsim::MassiveBody &body : request.massive_bodies)
        {
            if (body.id != subject_body.id)
            {
                candidate_bodies.push_back(body);
            }
        }
        if (candidate_bodies.empty())
        {
            return out;
        }

        const std::size_t primary_index = orbitsim::auto_select_primary_index(
                candidate_bodies,
                subject_body.state.position_m,
                [&candidate_bodies](const std::size_t i) -> orbitsim::Vec3 { return candidate_bodies[i].state.position_m; },
                request.sim_config.softening_length_m);
        const orbitsim::MassiveBody &reference_body = candidate_bodies[primary_index];

        out.rel_pos_m = glm::dvec3(subject_body.state.position_m - reference_body.state.position_m);
        out.rel_vel_mps = glm::dvec3(subject_body.state.velocity_mps - reference_body.state.velocity_mps);
        if (!finite_vec3(out.rel_pos_m) || !finite_vec3(out.rel_vel_mps))
        {
            return out;
        }

        const double mu_ref_m3_s2 = request.sim_config.gravitational_constant * reference_body.mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return out;
        }

        const double horizon_s_auto =
                OrbitPredictionMath::select_prediction_horizon(mu_ref_m3_s2, out.rel_pos_m, out.rel_vel_mps);

        const double requested_window_s =
                std::isfinite(request.future_window_s) ? std::max(0.0, request.future_window_s) : 0.0;
        double horizon_s = std::max(OrbitPredictionTuning::kMinHorizonS, horizon_s_auto);
        horizon_s = std::max(horizon_s, std::max(1.0, requested_window_s));
        if (!(horizon_s > 0.0))
        {
            return out;
        }

        out.valid = true;
        out.horizon_s = horizon_s;
        return out;
    }
} // namespace Game
