#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/detail/spacecraft_propagation.hpp"
#include "orbitsim/integrators.hpp"

namespace Game
{
    bool request_uses_long_range_prediction_policy(const OrbitPredictionService::Request &request)
    {
        return std::isfinite(request.future_window_s) &&
               request.future_window_s > OrbitPredictionTuning::kLongRangeHorizonThresholdS;
    }

    double resolve_prediction_horizon_cap_s(const OrbitPredictionService::Request &request)
    {
        return request_uses_long_range_prediction_policy(request)
                       ? OrbitPredictionTuning::kLongRangeHorizonCapS
                       : OrbitPredictionTuning::kMaxHorizonS;
    }

    double resolve_prediction_sample_dt_cap_s(const OrbitPredictionService::Request &request)
    {
        if (request.lagrange_sensitive)
        {
            return OrbitPredictionTuning::kMaxSampleDtS;
        }

        return request_uses_long_range_prediction_policy(request)
                       ? OrbitPredictionTuning::kLongRangeMaxSampleDtS
                       : OrbitPredictionTuning::kMaxSampleDtS;
    }

    std::size_t resolve_spacecraft_segment_budget(const OrbitPredictionService::Request &request,
                                                  const double horizon_s,
                                                  const std::size_t sample_budget)
    {
        std::size_t budget = std::max<std::size_t>(1, sample_budget);
        if (!request_uses_long_range_prediction_policy(request) || !(horizon_s > 0.0))
        {
            return budget;
        }

        const std::size_t desired_long_range_budget =
                static_cast<std::size_t>(std::ceil(horizon_s / OrbitPredictionTuning::kLongRangeSegmentTargetDtS));
        budget = std::max(budget, desired_long_range_budget);
        return std::clamp<std::size_t>(budget, 1, OrbitPredictionTuning::kLongRangeMaxSegmentsHard);
    }

    void apply_lagrange_integrator_profile(orbitsim::GameSimulation::Config &sim_config)
    {
        orbitsim::DOPRI5Options &integrator = sim_config.spacecraft_integrator;
        if (!(integrator.max_step_s > 0.0) || integrator.max_step_s > OrbitPredictionTuning::kLagrangeIntegratorMaxStepS)
        {
            integrator.max_step_s = OrbitPredictionTuning::kLagrangeIntegratorMaxStepS;
        }
        integrator.abs_tol = std::min(integrator.abs_tol, OrbitPredictionTuning::kLagrangeIntegratorAbsTol);
        integrator.rel_tol = std::min(integrator.rel_tol, OrbitPredictionTuning::kLagrangeIntegratorRelTol);
    }

    std::size_t count_future_maneuver_impulses(const OrbitPredictionService::Request &request)
    {
        return static_cast<std::size_t>(std::count_if(
                request.maneuver_impulses.begin(),
                request.maneuver_impulses.end(),
                [&request](const OrbitPredictionService::ManeuverImpulse &impulse) {
                    return std::isfinite(impulse.t_s) && impulse.t_s >= request.sim_time_s;
                }));
    }

    bool request_needs_control_sensitive_prediction(const OrbitPredictionService::Request &request)
    {
        return request.thrusting || count_future_maneuver_impulses(request) > 0;
    }

    double resolve_prediction_integrator_max_step_s(const OrbitPredictionService::Request &request)
    {
        if (request_needs_control_sensitive_prediction(request))
        {
            return OrbitPredictionTuning::kPredictionIntegratorMaxStepControlledS;
        }

        return request_uses_long_range_prediction_policy(request)
                       ? OrbitPredictionTuning::kPredictionIntegratorMaxStepLongRangeS
                       : OrbitPredictionTuning::kPredictionIntegratorMaxStepS;
    }

    void apply_prediction_integrator_profile(orbitsim::GameSimulation::Config &sim_config,
                                             const OrbitPredictionService::Request &request)
    {
        orbitsim::DOPRI5Options &integrator = sim_config.spacecraft_integrator;
        const double max_step_s = resolve_prediction_integrator_max_step_s(request);
        if (!(integrator.max_step_s > 0.0) || integrator.max_step_s > max_step_s)
        {
            integrator.max_step_s = max_step_s;
        }

        integrator.max_substeps =
                std::max(integrator.max_substeps, OrbitPredictionTuning::kPredictionIntegratorMaxSubstepsSoft);
        integrator.max_substeps_hard =
                std::max({integrator.max_substeps_hard,
                          integrator.max_substeps,
                          OrbitPredictionTuning::kPredictionIntegratorMaxSubstepsHard});
        integrator.max_interval_splits =
                std::max(integrator.max_interval_splits, OrbitPredictionTuning::kPredictionIntegratorMaxIntervalSplits);
    }

    std::size_t resolve_prediction_ephemeris_max_segments(const OrbitPredictionService::Request &request)
    {
        if (request.lagrange_sensitive)
        {
            return OrbitPredictionTuning::kLagrangeEphemerisMaxSamples;
        }

        return OrbitPredictionTuning::kPredictionEphemerisMaxSegmentsHard;
    }

    double resolve_prediction_ephemeris_dt_cap_s(const OrbitPredictionService::Request &request)
    {
        double dt_cap_s = request_uses_long_range_prediction_policy(request)
                                  ? OrbitPredictionTuning::kPredictionEphemerisMaxDtLongRangeS
                                  : OrbitPredictionTuning::kPredictionEphemerisMaxDtS;
        if (request_needs_control_sensitive_prediction(request))
        {
            dt_cap_s = std::min(dt_cap_s, OrbitPredictionTuning::kPredictionEphemerisMaxDtControlledS);
        }
        if (request.lagrange_sensitive)
        {
            dt_cap_s = std::min(dt_cap_s, OrbitPredictionTuning::kLagrangeEphemerisMaxDtS);
        }
        return dt_cap_s;
    }

    double resolve_prediction_ephemeris_dt_s(const OrbitPredictionService::Request &request,
                                             const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec)
    {
        double dt_s = sampling_spec.sample_dt_s;
        dt_s = std::min(dt_s, resolve_prediction_ephemeris_dt_cap_s(request));
        if (request.celestial_ephemeris_dt_s > 0.0)
        {
            dt_s = std::min(dt_s, request.celestial_ephemeris_dt_s);
        }

        const std::size_t max_segments = std::max<std::size_t>(1, resolve_prediction_ephemeris_max_segments(request));
        const double min_dt_for_budget_s = sampling_spec.horizon_s / static_cast<double>(max_segments);
        if (std::isfinite(min_dt_for_budget_s) && min_dt_for_budget_s > 0.0)
        {
            dt_s = std::max(dt_s, min_dt_for_budget_s);
        }

        return std::max(1.0e-3, dt_s);
    }

    SpacecraftSamplingBudget build_spacecraft_sampling_budget(const OrbitPredictionService::Request &request)
    {
        SpacecraftSamplingBudget out{};
        const double future_maneuvers = static_cast<double>(count_future_maneuver_impulses(request));

        const double target_bonus = std::min(
                future_maneuvers * OrbitPredictionTuning::kSpacecraftTargetSamplesBonusPerManeuver,
                OrbitPredictionTuning::kSpacecraftTargetSamplesHardNormal -
                        OrbitPredictionTuning::kSpacecraftTargetSamplesSoftNormal);
        out.target_samples_max =
                std::clamp(OrbitPredictionTuning::kSpacecraftTargetSamplesSoftNormal + target_bonus,
                           OrbitPredictionTuning::kSpacecraftTargetSamplesSoftNormal,
                           OrbitPredictionTuning::kSpacecraftTargetSamplesHardNormal);

        const double step_bonus = std::min(
                future_maneuvers * static_cast<double>(OrbitPredictionTuning::kSpacecraftMaxStepsBonusPerManeuver),
                static_cast<double>(OrbitPredictionTuning::kSpacecraftMaxStepsHardNormal -
                                    OrbitPredictionTuning::kSpacecraftMaxStepsSoftNormal));
        out.soft_max_steps =
                std::clamp(OrbitPredictionTuning::kSpacecraftMaxStepsSoftNormal + static_cast<int>(std::lround(step_bonus)),
                           OrbitPredictionTuning::kSpacecraftMaxStepsSoftNormal,
                           OrbitPredictionTuning::kSpacecraftMaxStepsHardNormal);
        return out;
    }

    namespace
    {
        bool same_config_for_ephemeris(const orbitsim::GameSimulation::Config &a,
                                       const orbitsim::GameSimulation::Config &b)
        {
            return a.gravitational_constant == b.gravitational_constant &&
                   a.softening_length_m == b.softening_length_m;
        }

        bool same_spin_state(const orbitsim::SpinState &a, const orbitsim::SpinState &b)
        {
            return a.axis == b.axis &&
                   a.angle_rad == b.angle_rad &&
                   a.rate_rad_per_s == b.rate_rad_per_s;
        }

        bool same_state_for_ephemeris(const orbitsim::State &a, const orbitsim::State &b)
        {
            return a.position_m == b.position_m &&
                   a.velocity_mps == b.velocity_mps &&
                   same_spin_state(a.spin, b.spin);
        }

        bool same_massive_body_for_ephemeris(const orbitsim::MassiveBody &a, const orbitsim::MassiveBody &b)
        {
            return a.id == b.id &&
                   a.mass_kg == b.mass_kg &&
                   same_state_for_ephemeris(a.state, b.state);
        }

        bool same_massive_body_set_for_ephemeris(const std::vector<orbitsim::MassiveBody> &a,
                                                 const std::vector<orbitsim::MassiveBody> &b)
        {
            if (a.size() != b.size())
            {
                return false;
            }

            for (std::size_t i = 0; i < a.size(); ++i)
            {
                if (!same_massive_body_for_ephemeris(a[i], b[i]))
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
               (ephemeris->t_end_s() + kEphemerisDurationEpsilonS) >= (sim_time_s + horizon_s);
    }

    bool compatible_cached_ephemeris(const OrbitPredictionService::CachedEphemerisEntry &entry,
                                     const OrbitPredictionService::EphemerisBuildRequest &request)
    {
        if (!entry.ephemeris || entry.ephemeris->empty())
        {
            return false;
        }

        if (entry.sim_time_s != request.sim_time_s)
        {
            return false;
        }

        if (!same_config_for_ephemeris(entry.sim_config, request.sim_config))
        {
            return false;
        }

        if (!same_massive_body_set_for_ephemeris(entry.massive_bodies, request.massive_bodies))
        {
            return false;
        }

        if ((entry.duration_s + kEphemerisDurationEpsilonS) < request.duration_s)
        {
            return false;
        }

        if (entry.celestial_dt_s > (request.celestial_dt_s + kEphemerisDtEpsilonS))
        {
            return false;
        }

        return true;
    }

    OrbitPredictionService::SharedCelestialEphemeris build_ephemeris_from_request(
            const OrbitPredictionService::EphemerisBuildRequest &request,
            const CancelCheck &cancel_requested)
    {
        if (!std::isfinite(request.sim_time_s) ||
            !(request.duration_s > 0.0) ||
            !(request.celestial_dt_s > 0.0) ||
            request.max_samples == 0)
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

        auto ephemeris = std::make_shared<orbitsim::CelestialEphemeris>();
        std::vector<orbitsim::BodyId> body_ids;
        body_ids.reserve(request.massive_bodies.size());
        for (const orbitsim::MassiveBody &body : request.massive_bodies)
        {
            body_ids.push_back(body.id);
        }
        ephemeris->set_body_ids(std::move(body_ids));

        std::vector<orbitsim::MassiveBody> massive = sim.massive_bodies();
        double t_s = request.sim_time_s;
        const double t_end_s = request.sim_time_s + request.duration_s;
        std::vector<orbitsim::State> start_states;
        std::vector<orbitsim::State> end_states;

        while (t_s < t_end_s && ephemeris->segments.size() < request.max_samples)
        {
            if (cancel_requested && cancel_requested())
            {
                return {};
            }

            const double h_s = std::min(request.celestial_dt_s, t_end_s - t_s);
            orbitsim::detail::snapshot_states(massive, &start_states);
            orbitsim::symplectic4_step(
                    massive,
                    h_s,
                    request.sim_config.gravitational_constant,
                    request.sim_config.softening_length_m);
            orbitsim::detail::snapshot_states(massive, &end_states);

            ephemeris->segments.push_back(orbitsim::CelestialEphemerisSegment{
                    .t0_s = t_s,
                    .dt_s = h_s,
                    .start = start_states,
                    .end = end_states,
            });
            t_s += h_s;
        }

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
        out.celestial_dt_s = resolve_prediction_ephemeris_dt_s(request, sampling_spec);
        const std::size_t desired_segments =
                static_cast<std::size_t>(std::ceil(out.duration_s / out.celestial_dt_s));
        out.max_samples = std::max<std::size_t>(sampling_spec.max_samples, desired_segments);
        out.max_samples = std::min(out.max_samples, resolve_prediction_ephemeris_max_segments(request));
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

        const auto [horizon_s_auto, dt_s_auto] =
                OrbitPredictionMath::select_prediction_horizon_and_dt(mu_ref_m3_s2, out.rel_pos_m, out.rel_vel_mps);

        const double horizon_cap_s = resolve_prediction_horizon_cap_s(request);
        const double sample_dt_cap_s = resolve_prediction_sample_dt_cap_s(request);
        double horizon_s = std::clamp(horizon_s_auto, OrbitPredictionTuning::kMinHorizonS, horizon_cap_s);
        horizon_s = std::max(horizon_s, std::max(1.0, request.future_window_s));
        double dt_s = std::clamp(dt_s_auto, 0.01, sample_dt_cap_s);
        const int max_steps = OrbitPredictionTuning::kMaxStepsNormal;
        const double min_dt_for_step_budget = horizon_s / static_cast<double>(std::max(1, max_steps));
        if (std::isfinite(min_dt_for_step_budget) && min_dt_for_step_budget > 0.0)
        {
            dt_s = std::max(dt_s, min_dt_for_step_budget);
        }

        dt_s = std::clamp(dt_s, 0.01, sample_dt_cap_s);
        horizon_s = std::clamp(horizon_s, dt_s, horizon_cap_s);
        const int sample_count = std::clamp(static_cast<int>(std::ceil(horizon_s / dt_s)) + 1, 2, max_steps);
        if (!(horizon_s > 0.0) || !(dt_s > 0.0) || sample_count < 2)
        {
            return out;
        }

        out.valid = true;
        out.horizon_s = horizon_s;
        out.sample_dt_s = dt_s;
        out.max_samples = static_cast<std::size_t>(sample_count);
        return out;
    }
} // namespace Game
