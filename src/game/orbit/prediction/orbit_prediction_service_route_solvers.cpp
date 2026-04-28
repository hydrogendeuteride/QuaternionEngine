#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/trajectories.hpp"

namespace Game
{
    namespace
    {
        void sync_prediction_stage_counts(OrbitPredictionService::Result &result)
        {
            result.diagnostics.ephemeris_segment_count = result.diagnostics.ephemeris.accepted_segments;
            result.diagnostics.trajectory_segment_count = result.diagnostics.trajectory_base.accepted_segments;
            result.diagnostics.trajectory_segment_count_planned = result.diagnostics.trajectory_planned.accepted_segments;
        }
    } // namespace

    EphemerisResolveOutcome resolve_ephemeris_for_request(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec,
            const double horizon_s,
            const CancelCheck &cancel_requested,
            const EphemerisResolverFn &resolve_ephemeris)
    {
        EphemerisResolveOutcome outcome{};
        outcome.ephemeris = request.shared_ephemeris;
        if (!ephemeris_covers_horizon(outcome.ephemeris, request.sim_time_s, horizon_s))
        {
            outcome.ephemeris = resolve_ephemeris(build_ephemeris_build_request(request, sampling_spec),
                                                  cancel_requested,
                                                  &outcome.diagnostics);
        }
        else
        {
            outcome.diagnostics =
                    make_stage_diagnostics_from_ephemeris(outcome.ephemeris, horizon_s, true);
        }

        if (!outcome.ephemeris || outcome.ephemeris->empty())
        {
            outcome.status = OrbitPredictionService::Status::EphemerisUnavailable;
            return outcome;
        }
        if (!validate_ephemeris_continuity(*outcome.ephemeris))
        {
            outcome.status = OrbitPredictionService::Status::ContinuityFailed;
            return outcome;
        }

        outcome.status = OrbitPredictionService::Status::Success;
        return outcome;
    }

    OrbitPredictionService::Status solve_celestial_prediction_route(
            const OrbitPredictionService::Request &request,
            orbitsim::GameSimulation &sim,
            const CancelCheck &cancel_requested,
            const EphemerisResolverFn &resolve_ephemeris,
            OrbitPredictionService::Result &out)
    {
        if (request.subject_body_id == orbitsim::kInvalidBodyId)
        {
            return OrbitPredictionService::Status::InvalidSubject;
        }

        const orbitsim::MassiveBody *subject_sim = sim.body_by_id(request.subject_body_id);
        if (!subject_sim)
        {
            return OrbitPredictionService::Status::InvalidSubject;
        }

        const CelestialPredictionSamplingSpec celestial_sampling =
                build_celestial_prediction_sampling_spec(request, *subject_sim);
        if (!celestial_sampling.valid)
        {
            return OrbitPredictionService::Status::InvalidSamplingSpec;
        }

        OrbitPredictionService::EphemerisSamplingSpec ephemeris_sampling{};
        ephemeris_sampling.valid = true;
        ephemeris_sampling.horizon_s = celestial_sampling.horizon_s;
        EphemerisResolveOutcome ephemeris =
                resolve_ephemeris_for_request(request,
                                              ephemeris_sampling,
                                              celestial_sampling.horizon_s,
                                              cancel_requested,
                                              resolve_ephemeris);
        if (ephemeris.status != OrbitPredictionService::Status::Success)
        {
            return ephemeris.status;
        }
        out.diagnostics.ephemeris = ephemeris.diagnostics;
        sync_prediction_stage_counts(out);

        if (cancel_requested())
        {
            return OrbitPredictionService::Status::Cancelled;
        }

        constexpr std::size_t kCelestialUISampleCount = 2'000;
        std::vector<orbitsim::TrajectorySample> traj_inertial =
                resample_ephemeris_uniform(*ephemeris.ephemeris,
                                           subject_sim->id,
                                           request.sim_time_s,
                                           request.sim_time_s + celestial_sampling.horizon_s,
                                           kCelestialUISampleCount);
        if (traj_inertial.size() < 2)
        {
            return OrbitPredictionService::Status::TrajectorySamplesUnavailable;
        }

        std::vector<orbitsim::TrajectorySegment> traj_segments_inertial =
                trajectory_segments_from_body_ephemeris(*ephemeris.ephemeris, subject_sim->id);
        if (traj_segments_inertial.empty())
        {
            return OrbitPredictionService::Status::TrajectorySegmentsUnavailable;
        }
        if (!validate_trajectory_segment_continuity(traj_segments_inertial))
        {
            return OrbitPredictionService::Status::ContinuityFailed;
        }

        out.shared_ephemeris = std::move(ephemeris.ephemeris);
        out.diagnostics.trajectory_base =
                make_stage_diagnostics_from_segments(traj_segments_inertial,
                                                     celestial_sampling.horizon_s);
        out.diagnostics.trajectory_sample_count = traj_inertial.size();
        sync_prediction_stage_counts(out);
        out.trajectory_inertial = std::move(traj_inertial);
        out.trajectory_segments_inertial = std::move(traj_segments_inertial);
        return OrbitPredictionService::Status::Success;
    }

    OrbitPredictionService::Status solve_spacecraft_baseline_trajectory(
            const OrbitPredictionService::Request &request,
            orbitsim::GameSimulation &sim,
            const orbitsim::CelestialEphemeris &ephemeris,
            const orbitsim::GameSimulation::SpacecraftHandle ship_handle,
            const orbitsim::AdaptiveSegmentOptions &segment_options,
            const double horizon_s,
            const CancelCheck &cancel_requested,
            OrbitPredictionService::Result &out)
    {
        orbitsim::AdaptiveSegmentDiagnostics base_diag{};
        std::vector<orbitsim::TrajectorySegment> trajectory_segments =
                orbitsim::predict_spacecraft_trajectory_segments_adaptive(
                        sim,
                        ephemeris,
                        ship_handle.id,
                        segment_options,
                        &base_diag);
        if (trajectory_segments.empty())
        {
            return OrbitPredictionService::Status::TrajectorySegmentsUnavailable;
        }
        if (!validate_trajectory_segment_continuity(trajectory_segments))
        {
            return OrbitPredictionService::Status::ContinuityFailed;
        }

        out.diagnostics.trajectory_base = make_stage_diagnostics_from_adaptive(base_diag, horizon_s);
        out.diagnostics.trajectory_base.accepted_segments = trajectory_segments.size();
        out.diagnostics.trajectory_base.covered_duration_s = prediction_segment_span_s(trajectory_segments);
        sync_prediction_stage_counts(out);

        if (cancel_requested())
        {
            return OrbitPredictionService::Status::Cancelled;
        }

        const std::size_t sample_count = prediction_sample_budget(request, trajectory_segments.size());
        std::vector<orbitsim::TrajectorySample> trajectory_samples =
                resample_segments_uniform(trajectory_segments, sample_count);
        if (trajectory_samples.size() < 2)
        {
            return OrbitPredictionService::Status::TrajectorySamplesUnavailable;
        }
        out.diagnostics.trajectory_sample_count = trajectory_samples.size();
        sync_prediction_stage_counts(out);

        if (cancel_requested())
        {
            return OrbitPredictionService::Status::Cancelled;
        }

        out.trajectory_segments_inertial = std::move(trajectory_segments);
        out.trajectory_inertial = std::move(trajectory_samples);
        return OrbitPredictionService::Status::Success;
    }
} // namespace Game
