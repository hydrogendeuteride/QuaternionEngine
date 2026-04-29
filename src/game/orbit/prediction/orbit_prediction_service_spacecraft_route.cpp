#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/maneuvers.hpp"

namespace Game
{
    namespace
    {
        using Request = OrbitPredictionService::Request;
        using Result = OrbitPredictionService::Result;
        using Status = OrbitPredictionService::Status;

        void sync_prediction_stage_counts(Result &result)
        {
            result.diagnostics.ephemeris_segment_count = result.diagnostics.ephemeris.accepted_segments;
            result.diagnostics.trajectory_segment_count = result.diagnostics.trajectory_base.accepted_segments;
            result.diagnostics.trajectory_segment_count_planned = result.diagnostics.trajectory_planned.accepted_segments;
        }

        struct TransientSpacecraftSetup
        {
            Status status{Status::Success};
            orbitsim::Spacecraft spacecraft{};
            std::optional<orbitsim::GameSimulation::SpacecraftHandle> handle{};
        };

        TransientSpacecraftSetup create_transient_spacecraft(
                orbitsim::GameSimulation &sim,
                const Request &request)
        {
            const glm::dvec3 ship_bary_pos_m = request.ship_bary_position_m;
            const glm::dvec3 ship_bary_vel_mps = request.ship_bary_velocity_mps;
            if (!finite_vec3(ship_bary_pos_m) || !finite_vec3(ship_bary_vel_mps))
            {
                return TransientSpacecraftSetup{.status = Status::InvalidInput};
            }

            orbitsim::Spacecraft spacecraft{};
            spacecraft.state = orbitsim::make_state(ship_bary_pos_m, ship_bary_vel_mps);
            spacecraft.dry_mass_kg = 1.0;

            const auto handle = sim.create_spacecraft(spacecraft);
            if (!handle.valid())
            {
                return TransientSpacecraftSetup{.status = Status::InvalidSubject};
            }

            sim.maneuver_plan() = orbitsim::ManeuverPlan{};
            return TransientSpacecraftSetup{
                    .status = Status::Success,
                    .spacecraft = spacecraft,
                    .handle = handle,
            };
        }

        bool try_apply_reusable_baseline(
                const SpacecraftPredictionRouteEnvironment &env,
                const double horizon_s,
                OrbitPredictionService::SharedCelestialEphemeris shared_ephemeris)
        {
            if (!request_can_reuse_spacecraft_baseline(env.request))
            {
                return false;
            }

            const std::optional<ReusableSpacecraftBaseline> reusable_baseline =
                    env.services.find_reusable_baseline(env.request.track_id, env.job.request_epoch);
            const double reusable_window_s =
                    std::isfinite(env.request.future_window_s)
                            ? std::max(0.0, env.request.future_window_s)
                            : 0.0;
            if (!reusable_baseline.has_value() ||
                reusable_baseline->trajectory_inertial.size() < 2 ||
                !validate_trajectory_segment_continuity(reusable_baseline->trajectory_segments_inertial) ||
                !trajectory_segments_cover_window(reusable_baseline->trajectory_segments_inertial,
                                                  env.request.sim_time_s,
                                                  reusable_window_s))
            {
                return false;
            }

            env.state.out.diagnostics.trajectory_base =
                    make_stage_diagnostics_from_segments(reusable_baseline->trajectory_segments_inertial,
                                                         horizon_s,
                                                         true);
            env.state.out.diagnostics.trajectory_sample_count = reusable_baseline->trajectory_inertial.size();
            sync_prediction_stage_counts(env.state.out);
            env.state.out.shared_ephemeris = std::move(shared_ephemeris);
            env.state.out.trajectory_segments_inertial = reusable_baseline->trajectory_segments_inertial;
            env.state.out.trajectory_inertial = reusable_baseline->trajectory_inertial;
            env.state.out.baseline_reused = true;
            return true;
        }
    } // namespace

    SpacecraftPredictionRouteOutcome solve_spacecraft_prediction_route(
            const SpacecraftPredictionRouteEnvironment &env)
    {
        SpacecraftPredictionRouteOutcome outcome{};

        TransientSpacecraftSetup spacecraft_setup = create_transient_spacecraft(env.state.sim, env.request);
        if (spacecraft_setup.status != Status::Success)
        {
            outcome.status = spacecraft_setup.status;
            return outcome;
        }

        const double horizon_s = env.sampling_spec.horizon_s;
        const orbitsim::AdaptiveSegmentOptions segment_opt =
                build_spacecraft_adaptive_segment_options(env.request,
                                                          env.sampling_spec,
                                                          env.callbacks.cancel_requested);

        EphemerisResolveOutcome spacecraft_ephemeris =
                resolve_ephemeris_for_request(env.request,
                                              env.sampling_spec,
                                              horizon_s,
                                              env.callbacks.cancel_requested,
                                              env.resolve_ephemeris);
        if (spacecraft_ephemeris.status != Status::Success)
        {
            outcome.status = spacecraft_ephemeris.status;
            return outcome;
        }

        OrbitPredictionService::SharedCelestialEphemeris shared_ephemeris =
                std::move(spacecraft_ephemeris.ephemeris);
        env.state.out.diagnostics.ephemeris = spacecraft_ephemeris.diagnostics;
        sync_prediction_stage_counts(env.state.out);

        if (env.callbacks.cancel_requested())
        {
            outcome.status = Status::Cancelled;
            return outcome;
        }

        const orbitsim::CelestialEphemeris &eph = *shared_ephemeris;
        const bool reused_baseline = try_apply_reusable_baseline(env, horizon_s, shared_ephemeris);

        if (!reused_baseline)
        {
            const Status baseline_status = solve_spacecraft_baseline_trajectory(env.request,
                                                                                 env.state.sim,
                                                                                 eph,
                                                                                 *spacecraft_setup.handle,
                                                                                 segment_opt,
                                                                                 horizon_s,
                                                                                 env.callbacks.cancel_requested,
                                                                                 env.state.out);
            if (baseline_status != Status::Success)
            {
                outcome.status = baseline_status;
                return outcome;
            }

            env.state.out.shared_ephemeris = shared_ephemeris;
        }

        if (!env.request.maneuver_impulses.empty())
        {
            PlannedPredictionRouteOutcome planned_outcome =
                    solve_planned_prediction_route(PlannedPredictionRouteEnvironment{
                            .request = env.request,
                            .callbacks = env.callbacks,
                            .out = env.state.out,
                            .ephemeris = eph,
                            .ship_state = spacecraft_setup.spacecraft.state,
                            .services = env.services,
                    });
            outcome.published_staged_preview = planned_outcome.published_staged_preview;
            if (planned_outcome.status != Status::Success)
            {
                outcome.status = planned_outcome.status;
                return outcome;
            }
        }

        env.services.store_reusable_baseline(env.request.track_id,
                                             env.job.generation_id,
                                             env.job.request_epoch,
                                             env.state.out.shared_ephemeris,
                                             env.state.out.trajectory_inertial,
                                             env.state.out.trajectory_segments_inertial);

        outcome.status = Status::Success;
        return outcome;
    }
} // namespace Game
