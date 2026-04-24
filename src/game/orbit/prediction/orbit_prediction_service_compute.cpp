#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/detail/spacecraft_propagation.hpp"
#include "orbitsim/frame_utils.hpp"
#include "orbitsim/maneuvers.hpp"
#include "orbitsim/maneuvers_types.hpp"
#include "orbitsim/spacecraft_state_cache.hpp"
#include "orbitsim/trajectories.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <chrono>

namespace Game
{
    namespace
    {
        bool prediction_chunk_has_flag(const uint32_t flags,
                                       const OrbitPredictionService::PredictionChunkBoundaryFlags flag)
        {
            return (flags & static_cast<uint32_t>(flag)) != 0u;
        }

        OrbitPredictionService::PublishedChunk make_published_chunk(
                const PlannedChunkPacket &packet,
                const uint32_t chunk_id,
                const OrbitPredictionService::ChunkQualityState quality_state)
        {
            return OrbitPredictionService::PublishedChunk{
                    .chunk_id = chunk_id,
                    .quality_state = quality_state,
                    .t0_s = packet.chunk.t0_s,
                    .t1_s = packet.chunk.t1_s,
                    .includes_planned_path = !packet.segments.empty(),
                    .reused_from_cache = packet.reused_from_cache,
            };
        }

        void append_published_chunk(std::vector<OrbitPredictionService::PublishedChunk> &dst,
                                    const PlannedChunkPacket &packet,
                                    const uint32_t chunk_id,
                                    const OrbitPredictionService::ChunkQualityState quality_state)
        {
            dst.push_back(make_published_chunk(packet, chunk_id, quality_state));
        }

        OrbitPredictionService::StreamedPlannedChunk make_streamed_planned_chunk(
                const PlannedChunkPacket &packet,
                const OrbitPredictionService::PublishedChunk &published_chunk)
        {
            OrbitPredictionService::StreamedPlannedChunk streamed_chunk{};
            streamed_chunk.published_chunk = published_chunk;
            streamed_chunk.trajectory_segments_inertial = packet.segments;
            streamed_chunk.trajectory_inertial = packet.samples;
            streamed_chunk.maneuver_previews = packet.previews;
            streamed_chunk.diagnostics = packet.diagnostics;
            streamed_chunk.start_state = packet.start_state;
            streamed_chunk.end_state = packet.end_state;
            return streamed_chunk;
        }

        OrbitPredictionService::AdaptiveStageDiagnostics merge_adaptive_stage_diagnostics(
                OrbitPredictionService::AdaptiveStageDiagnostics lhs,
                const OrbitPredictionService::AdaptiveStageDiagnostics &rhs)
        {
            const double lhs_dt_weight = static_cast<double>(lhs.accepted_segments);
            const double rhs_dt_weight = static_cast<double>(rhs.accepted_segments);
            const bool lhs_has_dt = lhs_dt_weight > 0.0;
            const bool rhs_has_dt = rhs_dt_weight > 0.0;

            lhs.requested_duration_s += rhs.requested_duration_s;
            lhs.covered_duration_s += rhs.covered_duration_s;
            lhs.accepted_segments += rhs.accepted_segments;
            lhs.rejected_splits += rhs.rejected_splits;
            lhs.forced_boundary_splits += rhs.forced_boundary_splits;
            lhs.frame_resegmentation_count += rhs.frame_resegmentation_count;

            if (rhs_has_dt)
            {
                lhs.min_dt_s = lhs_has_dt ? std::min(lhs.min_dt_s, rhs.min_dt_s) : rhs.min_dt_s;
                lhs.max_dt_s = lhs_has_dt ? std::max(lhs.max_dt_s, rhs.max_dt_s) : rhs.max_dt_s;
            }

            if ((lhs_dt_weight + rhs_dt_weight) > 0.0)
            {
                const double lhs_avg = std::isfinite(lhs.avg_dt_s) ? lhs.avg_dt_s : 0.0;
                const double rhs_avg = std::isfinite(rhs.avg_dt_s) ? rhs.avg_dt_s : 0.0;
                lhs.avg_dt_s =
                        ((lhs_avg * lhs_dt_weight) + (rhs_avg * rhs_dt_weight)) / (lhs_dt_weight + rhs_dt_weight);
            }

            lhs.hard_cap_hit = lhs.hard_cap_hit || rhs.hard_cap_hit;
            lhs.cancelled = lhs.cancelled || rhs.cancelled;
            lhs.cache_reused = lhs.cache_reused || rhs.cache_reused;
            lhs.maneuver_apply_failed_count += rhs.maneuver_apply_failed_count;
            if (lhs.maneuver_apply_failed_node_id < 0 && rhs.maneuver_apply_failed_node_id >= 0)
            {
                lhs.maneuver_apply_failed_node_id = rhs.maneuver_apply_failed_node_id;
            }
            return lhs;
        }

        void append_planned_samples(std::vector<orbitsim::TrajectorySample> &dst,
                                    const std::vector<orbitsim::TrajectorySample> &src)
        {
            if (src.empty())
            {
                return;
            }

            if (!dst.empty())
            {
                const double time_epsilon_s = continuity_time_epsilon_s(src.front().t_s);
                if (std::abs(dst.back().t_s - src.front().t_s) <= time_epsilon_s)
                {
                    dst.pop_back();
                }
            }

            dst.insert(dst.end(), src.begin(), src.end());
        }

        void merge_planned_outputs(PlannedSolveOutput &dst, const PlannedSolveOutput &src)
        {
            dst.segments.insert(dst.segments.end(), src.segments.begin(), src.segments.end());
            append_planned_samples(dst.samples, src.samples);
            dst.previews.insert(dst.previews.end(), src.previews.begin(), src.previews.end());
            dst.chunk_reused.insert(dst.chunk_reused.end(), src.chunk_reused.begin(), src.chunk_reused.end());
            dst.end_state = src.end_state;
            dst.diagnostics = merge_adaptive_stage_diagnostics(dst.diagnostics, src.diagnostics);
            dst.status = src.status;
        }

        OrbitPredictionService::PublishedChunk make_published_chunk_from_plan(
                const OrbitPredictionService::PredictionChunkPlan &chunk,
                const uint32_t chunk_id,
                const bool reused_from_cache)
        {
            return OrbitPredictionService::PublishedChunk{
                    .chunk_id = chunk_id,
                    .quality_state = OrbitPredictionService::ChunkQualityState::Final,
                    .t0_s = chunk.t0_s,
                    .t1_s = chunk.t1_s,
                    .includes_planned_path = true,
                    .reused_from_cache = reused_from_cache,
            };
        }

        std::optional<std::size_t> find_planned_suffix_begin_chunk(
                const OrbitPredictionService::PredictionSolvePlan &solve_plan,
                const double anchor_time_s)
        {
            if (!solve_plan.valid || !std::isfinite(anchor_time_s))
            {
                return std::nullopt;
            }

            const double epsilon_s = continuity_time_epsilon_s(anchor_time_s);
            for (std::size_t chunk_index = 0; chunk_index < solve_plan.chunks.size(); ++chunk_index)
            {
                if (std::abs(solve_plan.chunks[chunk_index].t0_s - anchor_time_s) <= epsilon_s)
                {
                    return chunk_index;
                }
            }
            return std::nullopt;
        }

        bool planned_suffix_refine_prefix_is_valid(
                const OrbitPredictionService::Request &request,
                const OrbitPredictionService::PredictionSolvePlan &solve_plan,
                const std::size_t suffix_begin_index)
        {
            const OrbitPredictionService::Request::PlannedSuffixRefineSpec &suffix =
                    request.planned_suffix_refine;
            if (!suffix.active ||
                suffix_begin_index >= solve_plan.chunks.size() ||
                !std::isfinite(suffix.anchor_time_s) ||
                !finite_state(suffix.anchor_state_inertial) ||
                suffix.prefix_segments_inertial.empty() ||
                !validate_trajectory_segment_continuity(suffix.prefix_segments_inertial))
            {
                return false;
            }

            const double start_epsilon_s = continuity_time_epsilon_s(request.sim_time_s);
            const double anchor_epsilon_s = continuity_time_epsilon_s(suffix.anchor_time_s);
            const double prefix_end_s = prediction_segment_end_time(suffix.prefix_segments_inertial.back());
            if (std::abs(suffix.prefix_segments_inertial.front().t0_s - request.sim_time_s) > start_epsilon_s ||
                std::abs(prefix_end_s - suffix.anchor_time_s) > anchor_epsilon_s)
            {
                return false;
            }

            if (std::abs(solve_plan.chunks[suffix_begin_index].t0_s - suffix.anchor_time_s) > anchor_epsilon_s)
            {
                return false;
            }

            return states_are_continuous(suffix.prefix_segments_inertial.back().end,
                                         suffix.anchor_state_inertial);
        }

        PlannedSolveOutput make_planned_suffix_refine_prefix_output(
                const OrbitPredictionService::Request &request)
        {
            PlannedSolveOutput output{};
            output.segments = request.planned_suffix_refine.prefix_segments_inertial;
            output.previews = request.planned_suffix_refine.prefix_previews;
            output.end_state = output.segments.empty() ? orbitsim::State{} : output.segments.back().end;
            output.diagnostics =
                    make_stage_diagnostics_from_segments(output.segments,
                                                         prediction_segment_span_s(output.segments),
                                                         true);
            output.samples = resample_segments_uniform(output.segments,
                                                       prediction_sample_budget(request, output.segments.size()));
            output.status = output.samples.size() >= 2u
                                    ? OrbitPredictionService::Status::Success
                                    : OrbitPredictionService::Status::TrajectorySamplesUnavailable;
            return output;
        }

        std::vector<OrbitPredictionService::ManeuverNodePreview> collect_prefix_chunk_previews(
                const std::vector<OrbitPredictionService::ManeuverNodePreview> &previews,
                const double chunk_t0_s,
                const double chunk_t1_s)
        {
            std::vector<OrbitPredictionService::ManeuverNodePreview> out;
            for (const OrbitPredictionService::ManeuverNodePreview &preview : previews)
            {
                if (!preview.valid || !std::isfinite(preview.t_s))
                {
                    continue;
                }

                const double start_epsilon_s = continuity_time_epsilon_s(chunk_t0_s);
                const double end_epsilon_s = continuity_time_epsilon_s(chunk_t1_s);
                if (preview.t_s >= (chunk_t0_s - start_epsilon_s) &&
                    preview.t_s < (chunk_t1_s - end_epsilon_s))
                {
                    out.push_back(preview);
                }
            }
            return out;
        }

        bool build_cached_prefix_stream_chunks(
                const OrbitPredictionService::Request &request,
                const OrbitPredictionService::PredictionSolvePlan &solve_plan,
                const std::size_t suffix_begin_index,
                const PlannedSolveOutput &prefix_output,
                std::vector<OrbitPredictionService::PublishedChunk> &out_published_chunks,
                std::vector<OrbitPredictionService::StreamedPlannedChunk> &out_streamed_chunks)
        {
            out_published_chunks.clear();
            out_streamed_chunks.clear();
            if (suffix_begin_index == 0u || suffix_begin_index > solve_plan.chunks.size())
            {
                return true;
            }

            out_published_chunks.reserve(suffix_begin_index);
            out_streamed_chunks.reserve(suffix_begin_index);
            for (std::size_t chunk_index = 0u; chunk_index < suffix_begin_index; ++chunk_index)
            {
                const OrbitPredictionService::PredictionChunkPlan &chunk = solve_plan.chunks[chunk_index];
                std::vector<orbitsim::TrajectorySegment> chunk_segments =
                        slice_trajectory_segments(prefix_output.segments, chunk.t0_s, chunk.t1_s);
                if (chunk_segments.empty() || !validate_trajectory_segment_continuity(chunk_segments))
                {
                    out_published_chunks.clear();
                    out_streamed_chunks.clear();
                    return false;
                }

                std::vector<orbitsim::TrajectorySample> chunk_samples =
                        resample_segments_uniform(chunk_segments,
                                                  prediction_sample_budget(request, chunk_segments.size()));
                if (chunk_samples.size() < 2u)
                {
                    out_published_chunks.clear();
                    out_streamed_chunks.clear();
                    return false;
                }

                OrbitPredictionService::PublishedChunk published_chunk =
                        make_published_chunk_from_plan(chunk, chunk.chunk_id, true);
                OrbitPredictionService::StreamedPlannedChunk streamed_chunk{};
                streamed_chunk.published_chunk = published_chunk;
                streamed_chunk.trajectory_segments_inertial = std::move(chunk_segments);
                streamed_chunk.trajectory_inertial = std::move(chunk_samples);
                streamed_chunk.maneuver_previews =
                        collect_prefix_chunk_previews(prefix_output.previews, chunk.t0_s, chunk.t1_s);
                streamed_chunk.diagnostics =
                        make_stage_diagnostics_from_segments(streamed_chunk.trajectory_segments_inertial,
                                                             chunk.t1_s - chunk.t0_s,
                                                             true);
                streamed_chunk.start_state = streamed_chunk.trajectory_segments_inertial.front().start;
                streamed_chunk.end_state = streamed_chunk.trajectory_segments_inertial.back().end;

                out_published_chunks.push_back(std::move(published_chunk));
                out_streamed_chunks.push_back(std::move(streamed_chunk));
            }

            return true;
        }
    } // namespace

    void OrbitPredictionService::compute_prediction(const PendingJob &job)
    {
        const uint64_t generation_id = job.generation_id;
        const Request &request = job.request;
        const uint64_t request_epoch = job.request_epoch;

        const std::chrono::steady_clock::time_point compute_start = std::chrono::steady_clock::now();
        Result out{};
        out.generation_id = generation_id;
        out.track_id = request.track_id;
        out.maneuver_plan_revision = request.maneuver_plan_revision;
        out.build_time_s = request.sim_time_s;
        out.solve_quality = request.solve_quality;
        const auto cancel_requested = [this,
                                        track_id = request.track_id,
                                        generation_id,
                                        request_epoch,
                                        maneuver_plan_revision = request.maneuver_plan_revision]() {
            return !should_continue_job(track_id, generation_id, request_epoch, maneuver_plan_revision);
        };
        const auto elapsed_ms = [&compute_start]() {
            return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - compute_start).count();
        };
        const auto publish = [this, &job, &elapsed_ms](Result result) {
            result.compute_time_ms = elapsed_ms();
            return publish_completed_result(job, std::move(result));
        };
        const auto fail = [&out, &publish](const Status status) {
            out.diagnostics.status = status;
            publish(out);
            return;
        };
        const auto cancel = [&out, &publish]() {
            out.diagnostics.cancelled = true;
            out.diagnostics.status = Status::Cancelled;
            publish(out);
            return;
        };
        const auto sync_result_stage_counts = [](Result &result) {
            result.diagnostics.ephemeris_segment_count = result.diagnostics.ephemeris.accepted_segments;
            result.diagnostics.trajectory_segment_count = result.diagnostics.trajectory_base.accepted_segments;
            result.diagnostics.trajectory_segment_count_planned = result.diagnostics.trajectory_planned.accepted_segments;
        };
        const auto sync_stage_counts = [&out]() {
            out.diagnostics.ephemeris_segment_count = out.diagnostics.ephemeris.accepted_segments;
            out.diagnostics.trajectory_segment_count = out.diagnostics.trajectory_base.accepted_segments;
            out.diagnostics.trajectory_segment_count_planned = out.diagnostics.trajectory_planned.accepted_segments;
        };
        const auto ensure_single_publish_chunk_metadata = [](Result &result) {
            if (!result.published_chunks.empty())
            {
                return;
            }

            const bool has_planned_path = !result.trajectory_segments_inertial_planned.empty();
            const std::vector<orbitsim::TrajectorySegment> &segments =
                    has_planned_path ? result.trajectory_segments_inertial_planned
                                     : result.resolved_trajectory_segments_inertial();
            if (segments.empty())
            {
                return;
            }

            result.published_chunks.push_back(PublishedChunk{
                    .chunk_id = 0u,
                    .quality_state = ChunkQualityState::Final,
                    .t0_s = segments.front().t0_s,
                    .t1_s = prediction_segment_end_time(segments.back()),
                    .includes_planned_path = has_planned_path,
                    .reused_from_cache = has_planned_path
                                                 ? result.diagnostics.trajectory_planned.cache_reused
                                                 : result.baseline_reused,
            });
        };

        // Invalid inputs produce an invalid result rather than throwing on the worker thread.
        if (!std::isfinite(request.sim_time_s))
        {
            fail(Status::InvalidInput);
            return;
        }

        std::optional<EphemerisSamplingSpec> spacecraft_sampling_spec{};
        if (request.kind == RequestKind::Spacecraft)
        {
            spacecraft_sampling_spec = build_ephemeris_sampling_spec(request);
            if (!spacecraft_sampling_spec->valid)
            {
                fail(Status::InvalidSamplingSpec);
                return;
            }
        }

        orbitsim::GameSimulation::Config sim_config = request.sim_config;
        const double resolved_integrator_horizon_s =
                spacecraft_sampling_spec.has_value()
                        ? spacecraft_sampling_spec->horizon_s
                        : std::max(OrbitPredictionTuning::kMinHorizonS,
                                   std::isfinite(request.future_window_s) ? std::max(0.0, request.future_window_s)
                                                                          : 0.0);
        apply_prediction_integrator_profile(sim_config, request, resolved_integrator_horizon_s);
        if (request.lagrange_sensitive)
        {
            apply_lagrange_integrator_profile(sim_config);
        }

        orbitsim::GameSimulation sim(sim_config);
        if (!sim.set_time_s(request.sim_time_s))
        {
            fail(Status::InvalidInput);
            return;
        }

        if (cancel_requested())
        {
            cancel();
            return;
        }

        for (const orbitsim::MassiveBody &body : request.massive_bodies)
        {
            const auto body_handle =
                    (body.id != orbitsim::kInvalidBodyId)
                        ? sim.create_body_with_id(body.id, body)
                        : sim.create_body(body);
            if (!body_handle.valid())
            {
                fail(Status::InvalidSubject);
                return;
            }
        }

        if (cancel_requested())
        {
            cancel();
            return;
        }

        out.massive_bodies = sim.massive_bodies();

        // Celestial route: predict the body's inertial motion.
        if (request.kind == RequestKind::Celestial)
        {
            if (request.subject_body_id == orbitsim::kInvalidBodyId)
            {
                fail(Status::InvalidSubject);
                return;
            }

            const orbitsim::MassiveBody *subject_sim = sim.body_by_id(request.subject_body_id);
            if (!subject_sim)
            {
                fail(Status::InvalidSubject);
                return;
            }

            const CelestialPredictionSamplingSpec sampling_spec =
                    build_celestial_prediction_sampling_spec(request, *subject_sim);
            if (!sampling_spec.valid)
            {
                fail(Status::InvalidSamplingSpec);
                return;
            }

            SharedCelestialEphemeris shared_ephemeris = request.shared_ephemeris;
            AdaptiveStageDiagnostics ephemeris_diag{};
            if (!ephemeris_covers_horizon(shared_ephemeris, request.sim_time_s, sampling_spec.horizon_s))
            {
                // Keep ephemeris construction on the worker so gameplay only enqueues work.
                EphemerisSamplingSpec ephemeris_sampling_spec{};
                ephemeris_sampling_spec.valid = true;
                ephemeris_sampling_spec.horizon_s = sampling_spec.horizon_s;
                shared_ephemeris = get_or_build_ephemeris(build_ephemeris_build_request(request, ephemeris_sampling_spec),
                                                          cancel_requested,
                                                          &ephemeris_diag);
            }
            else
            {
                ephemeris_diag = make_stage_diagnostics_from_ephemeris(shared_ephemeris, sampling_spec.horizon_s, true);
            }
            if (!shared_ephemeris || shared_ephemeris->empty())
            {
                fail(Status::EphemerisUnavailable);
                return;
            }
            if (!validate_ephemeris_continuity(*shared_ephemeris))
            {
                fail(Status::ContinuityFailed);
                return;
            }
            out.diagnostics.ephemeris = ephemeris_diag;
            sync_stage_counts();

            if (cancel_requested())
            {
                cancel();
                return;
            }

            constexpr std::size_t kCelestialUISampleCount = 2'000;
            const std::size_t celestial_sample_count = kCelestialUISampleCount;
            std::vector<orbitsim::TrajectorySample> traj_inertial =
                    resample_ephemeris_uniform(*shared_ephemeris, subject_sim->id,
                                              request.sim_time_s,
                                              request.sim_time_s + sampling_spec.horizon_s,
                                              celestial_sample_count);
            if (traj_inertial.size() < 2)
            {
                fail(Status::TrajectorySamplesUnavailable);
                return;
            }

            std::vector<orbitsim::TrajectorySegment> traj_segments_inertial =
                    trajectory_segments_from_body_ephemeris(*shared_ephemeris, subject_sim->id);
            if (traj_segments_inertial.empty())
            {
                fail(Status::TrajectorySegmentsUnavailable);
                return;
            }
            if (!validate_trajectory_segment_continuity(traj_segments_inertial))
            {
                fail(Status::ContinuityFailed);
                return;
            }

            out.shared_ephemeris = shared_ephemeris;
            out.diagnostics.trajectory_base =
                    make_stage_diagnostics_from_segments(traj_segments_inertial, sampling_spec.horizon_s);
            out.diagnostics.trajectory_sample_count = traj_inertial.size();
            sync_stage_counts();
            out.trajectory_inertial = std::move(traj_inertial);
            out.trajectory_segments_inertial = std::move(traj_segments_inertial);
            ensure_single_publish_chunk_metadata(out);
            out.valid = true;
            out.diagnostics.status = Status::Success;
            publish(std::move(out));
            return;
        }

        // Spacecraft route: build a transient ship state, then predict inertial baseline and planned trajectories.
        const glm::dvec3 ship_bary_pos_m = request.ship_bary_position_m;
        const glm::dvec3 ship_bary_vel_mps = request.ship_bary_velocity_mps;
        if (!finite_vec3(ship_bary_pos_m) || !finite_vec3(ship_bary_vel_mps))
        {
            fail(Status::InvalidInput);
            return;
        }

        const EphemerisSamplingSpec sampling_spec = *spacecraft_sampling_spec;
        const double horizon_s = sampling_spec.horizon_s;
        orbitsim::Spacecraft ship_sc{};
        ship_sc.state = orbitsim::make_state(ship_bary_pos_m, ship_bary_vel_mps);
        ship_sc.dry_mass_kg = 1.0;

        const auto ship_h = sim.create_spacecraft(ship_sc);
        if (!ship_h.valid())
        {
            fail(Status::InvalidSubject);
            return;
        }

        sim.maneuver_plan() = orbitsim::ManeuverPlan{};

        const orbitsim::AdaptiveSegmentOptions segment_opt =
                build_spacecraft_adaptive_segment_options(request, sampling_spec, cancel_requested);

        SharedCelestialEphemeris shared_ephemeris = request.shared_ephemeris;
        AdaptiveStageDiagnostics ephemeris_diag{};
        if (!ephemeris_covers_horizon(shared_ephemeris, request.sim_time_s, horizon_s))
        {
            // Spacecraft requests follow the same rule: never build ephemeris on the gameplay thread.
            shared_ephemeris = get_or_build_ephemeris(build_ephemeris_build_request(request, sampling_spec),
                                                      cancel_requested,
                                                      &ephemeris_diag);
        }
        else
        {
            ephemeris_diag = make_stage_diagnostics_from_ephemeris(shared_ephemeris, horizon_s, true);
        }
        if (!shared_ephemeris || shared_ephemeris->empty())
        {
            fail(Status::EphemerisUnavailable);
            return;
        }
        if (!validate_ephemeris_continuity(*shared_ephemeris))
        {
            fail(Status::ContinuityFailed);
            return;
        }
        out.diagnostics.ephemeris = ephemeris_diag;
        sync_stage_counts();

        if (cancel_requested())
        {
            cancel();
            return;
        }
        const orbitsim::CelestialEphemeris &eph = *shared_ephemeris;
        const std::optional<ReusableBaselineCacheEntry> reusable_baseline =
                request_can_reuse_spacecraft_baseline(request)
                        ? find_reusable_baseline(request.track_id, request_epoch)
                        : std::nullopt;
        const double reusable_window_s =
                std::isfinite(request.future_window_s) ? std::max(0.0, request.future_window_s) : 0.0;

        bool reused_baseline = false;
        bool published_staged_preview = false;
        if (reusable_baseline.has_value() &&
            reusable_baseline->trajectory_inertial.size() >= 2 &&
            validate_trajectory_segment_continuity(reusable_baseline->trajectory_segments_inertial) &&
            trajectory_segments_cover_window(reusable_baseline->trajectory_segments_inertial,
                                             request.sim_time_s,
                                             reusable_window_s))
        {
            out.diagnostics.trajectory_base =
                    make_stage_diagnostics_from_segments(reusable_baseline->trajectory_segments_inertial, horizon_s, true);
            out.diagnostics.trajectory_sample_count = reusable_baseline->trajectory_inertial.size();
            sync_stage_counts();
            out.shared_ephemeris = shared_ephemeris;
            out.trajectory_segments_inertial = reusable_baseline->trajectory_segments_inertial;
            out.trajectory_inertial = reusable_baseline->trajectory_inertial;
            reused_baseline = true;
            out.baseline_reused = true;
        }

        if (!reused_baseline)
        {
            orbitsim::AdaptiveSegmentDiagnostics base_diag{};
            std::vector<orbitsim::TrajectorySegment> traj_segments_inertial_baseline =
                    orbitsim::predict_spacecraft_trajectory_segments_adaptive(sim, eph, ship_h.id, segment_opt, &base_diag);
            if (traj_segments_inertial_baseline.empty())
            {
                fail(Status::TrajectorySegmentsUnavailable);
                return;
            }
            if (!validate_trajectory_segment_continuity(traj_segments_inertial_baseline))
            {
                fail(Status::ContinuityFailed);
                return;
            }
            out.diagnostics.trajectory_base = make_stage_diagnostics_from_adaptive(base_diag, horizon_s);
            out.diagnostics.trajectory_base.accepted_segments = traj_segments_inertial_baseline.size();
            out.diagnostics.trajectory_base.covered_duration_s = prediction_segment_span_s(traj_segments_inertial_baseline);
            sync_stage_counts();

            if (cancel_requested())
            {
                cancel();
                return;
            }

            const std::size_t baseline_sample_count =
                    prediction_sample_budget(request, traj_segments_inertial_baseline.size());
            std::vector<orbitsim::TrajectorySample> traj_inertial_baseline =
                    resample_segments_uniform(traj_segments_inertial_baseline, baseline_sample_count);
            if (traj_inertial_baseline.size() < 2)
            {
                fail(Status::TrajectorySamplesUnavailable);
                return;
            }
            out.diagnostics.trajectory_sample_count = traj_inertial_baseline.size();
            sync_stage_counts();

            if (cancel_requested())
            {
                cancel();
                return;
            }

            out.shared_ephemeris = shared_ephemeris;
            out.trajectory_segments_inertial = std::move(traj_segments_inertial_baseline);
            out.trajectory_inertial = std::move(traj_inertial_baseline);
        }

        if (!request.maneuver_impulses.empty())
        {
            std::vector<ManeuverImpulse> maneuver_impulses = request.maneuver_impulses;
            std::stable_sort(maneuver_impulses.begin(),
                             maneuver_impulses.end(),
                             [](const ManeuverImpulse &a, const ManeuverImpulse &b) { return a.t_s < b.t_s; });

            // Create a modified request with sorted impulses for planned trajectory solving.
            Request planned_request = request;
            planned_request.maneuver_impulses = std::move(maneuver_impulses);

            PlannedTrajectoryContext ctx{
                    planned_request,
                    cancel_requested,
                    out,
                    eph,
                    publish,
                    // find_cached_chunk closure
                    [this](const PlannedChunkCacheKey &key,
                           const orbitsim::State &expected_start_state)
                            -> std::optional<PlannedChunkCacheEntry> {
                        std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
                        const auto cache_it = _planned_chunk_cache_by_key.find(key);
                        if (cache_it == _planned_chunk_cache_by_key.end())
                        {
                            return std::nullopt;
                        }

                        auto entry_it = cache_it->second;
                        PlannedChunkCacheEntry &entry = *entry_it;
                        if (entry.samples.size() < 2u ||
                            entry.segments.empty() ||
                            !validate_trajectory_segment_continuity(entry.segments) ||
                            (!entry.seam_validation_segments.empty() &&
                             !validate_trajectory_segment_continuity(entry.seam_validation_segments)))
                        {
                            _planned_chunk_cache_by_key.erase(cache_it);
                            _planned_chunk_cache.erase(entry_it);
                            return std::nullopt;
                        }

                        if (!states_are_continuous(entry.start_state, expected_start_state))
                        {
                            return std::nullopt;
                        }

                        _planned_chunk_cache.splice(_planned_chunk_cache.begin(),
                                                    _planned_chunk_cache,
                                                    entry_it);
                        cache_it->second = _planned_chunk_cache.begin();
                        return *_planned_chunk_cache.begin();
                    },
                    // store_cached_chunk closure
                    [this](PlannedChunkCacheEntry entry) {
                        constexpr std::size_t kMaxCachedPlannedChunks = 512u;
                        if (entry.samples.size() < 2u ||
                            entry.segments.empty() ||
                            !validate_trajectory_segment_continuity(entry.segments) ||
                            (!entry.seam_validation_segments.empty() &&
                             !validate_trajectory_segment_continuity(entry.seam_validation_segments)) ||
                            !finite_state(entry.start_state) ||
                            !finite_state(entry.end_state))
                        {
                            return;
                        }

                        std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
                        const auto existing_it = _planned_chunk_cache_by_key.find(entry.key);
                        if (existing_it != _planned_chunk_cache_by_key.end())
                        {
                            auto lru_it = existing_it->second;
                            *lru_it = std::move(entry);
                            _planned_chunk_cache.splice(_planned_chunk_cache.begin(),
                                                        _planned_chunk_cache,
                                                        lru_it);
                            existing_it->second = _planned_chunk_cache.begin();
                        }
                        else
                        {
                            _planned_chunk_cache.push_front(std::move(entry));
                            _planned_chunk_cache_by_key.emplace(_planned_chunk_cache.front().key,
                                                                _planned_chunk_cache.begin());
                        }

                        if (_planned_chunk_cache.size() > kMaxCachedPlannedChunks)
                        {
                            _planned_chunk_cache_by_key.erase(_planned_chunk_cache.back().key);
                            _planned_chunk_cache.pop_back();
                        }
                    },
                    // get_or_build_ephemeris closure
                    [this](const EphemerisBuildRequest &req, const CancelCheck &cancel_check) {
                        return get_or_build_ephemeris(req, cancel_check);
                    },
            };

            const PredictionSolvePlan solve_plan = build_prediction_solve_plan(planned_request);
            if (!solve_plan.valid)
            {
                fail(Status::InvalidInput);
                return;
            }

            const auto fail_with_planned_diagnostics =
                    [&](const Status status, const PlannedSolveOutput &planned_output) {
                        out.diagnostics.trajectory_planned = planned_output.diagnostics;
                        out.diagnostics.trajectory_sample_count_planned = planned_output.samples.size();
                        sync_stage_counts();
                        fail(status);
                    };

            std::shared_ptr<const Result::CoreData> staged_core_data{};
            const auto shared_core_data_for_staged_publish = [&out, &staged_core_data]() {
                if (staged_core_data)
                {
                    return staged_core_data;
                }

                auto core_data = std::make_shared<Result::CoreData>();
                core_data->shared_ephemeris = out.shared_ephemeris;
                core_data->massive_bodies = out.massive_bodies;
                core_data->trajectory_inertial = out.trajectory_inertial;
                core_data->trajectory_segments_inertial = out.trajectory_segments_inertial;
                staged_core_data = std::move(core_data);
                return staged_core_data;
            };
            const auto make_stage_result =
                    [&out, &sync_result_stage_counts, &shared_core_data_for_staged_publish](
                            const PlannedSolveOutput &stage_output,
                            const std::vector<PublishedChunk> &published_chunks,
                            const PublishStage publish_stage,
                            std::vector<StreamedPlannedChunk> streamed_planned_chunks = {},
                            const bool include_cumulative_planned = true) {
                        Result stage_result{};
                        stage_result.track_id = out.track_id;
                        stage_result.generation_id = out.generation_id;
                        stage_result.maneuver_plan_revision = out.maneuver_plan_revision;
                        stage_result.valid = true;
                        stage_result.baseline_reused = out.baseline_reused;
                        stage_result.solve_quality = out.solve_quality;
                        stage_result.publish_stage = publish_stage;
                        stage_result.diagnostics = out.diagnostics;
                        stage_result.build_time_s = out.build_time_s;
                        stage_result.set_shared_core_data(shared_core_data_for_staged_publish());
                        stage_result.diagnostics.trajectory_planned = stage_output.diagnostics;
                        stage_result.diagnostics.trajectory_sample_count_planned = stage_output.samples.size();
                        stage_result.diagnostics.status = Status::Success;
                        sync_result_stage_counts(stage_result);
                        if (include_cumulative_planned)
                        {
                            stage_result.trajectory_segments_inertial_planned = stage_output.segments;
                            stage_result.trajectory_inertial_planned = stage_output.samples;
                            stage_result.maneuver_previews = stage_output.previews;
                        }
                        stage_result.published_chunks = published_chunks;
                        stage_result.streamed_planned_chunks = std::move(streamed_planned_chunks);
                        return stage_result;
                    };

            const auto append_published_chunks_as_final =
                    [](std::vector<PublishedChunk> &dst,
                       const std::vector<PublishedChunk> &src) {
                        for (const PublishedChunk &chunk : src)
                        {
                            PublishedChunk final_chunk = chunk;
                            final_chunk.chunk_id = static_cast<uint32_t>(dst.size());
                            final_chunk.quality_state = ChunkQualityState::Final;
                            dst.push_back(final_chunk);
                        }
                    };

            const auto solve_stage_range =
                    [&](const std::size_t chunk_begin_index,
                        const std::size_t chunk_end_index,
                        const orbitsim::State &range_start_state,
                        const ChunkQualityState quality_state,
                        const uint32_t published_chunk_base_id,
                        PlannedSolveOutput &stage_output,
                        std::vector<PublishedChunk> &stage_published_chunks) {
                        const PlannedSolveRangeSummary summary =
                                solve_planned_chunk_range(ctx,
                                                          solve_plan,
                                                          chunk_begin_index,
                                                          chunk_end_index,
                                                          range_start_state,
                                                          [&](PlannedChunkPacket &&chunk_packet) {
                                                              const uint32_t published_chunk_id =
                                                                      published_chunk_base_id +
                                                                      static_cast<uint32_t>(stage_published_chunks.size());
                                                              append_published_chunk(stage_published_chunks,
                                                                                     chunk_packet,
                                                                                     published_chunk_id,
                                                                                     quality_state);
                                                              append_planned_chunk_packet(stage_output,
                                                                                          std::move(chunk_packet));
                                                              return true;
                                                          });
                        apply_planned_range_summary(stage_output, summary);
                        return summary;
                    };

            const auto solve_and_publish_stage =
                    [&](const std::size_t chunk_begin_index,
                        const std::size_t chunk_end_index,
                        const orbitsim::State &range_start_state,
                        const PublishStage publish_stage,
                        const ChunkQualityState quality_state,
                        const uint32_t published_chunk_base_id,
                        PlannedSolveOutput &stage_output,
                        std::vector<PublishedChunk> &stage_published_chunks) {
                        const PlannedSolveRangeSummary summary =
                                solve_planned_chunk_range(ctx,
                                                          solve_plan,
                                                          chunk_begin_index,
                                                          chunk_end_index,
                                                          range_start_state,
                                                          [&](PlannedChunkPacket &&chunk_packet) {
                                                              const uint32_t published_chunk_id =
                                                                      published_chunk_base_id +
                                                                      static_cast<uint32_t>(stage_published_chunks.size());
                                                              append_published_chunk(stage_published_chunks,
                                                                                     chunk_packet,
                                                                                     published_chunk_id,
                                                                                     quality_state);
                                                              append_planned_chunk_packet(stage_output,
                                                                                          std::move(chunk_packet));
                                                              return publish(make_stage_result(stage_output,
                                                                                               stage_published_chunks,
                                                                                               publish_stage));
                                                          });
                        apply_planned_range_summary(stage_output, summary);
                        return summary;
                    };

            const bool preview_staging_active =
                    planned_request.solve_quality == SolveQuality::FastPreview &&
                    planned_request.preview_patch.active;
            std::size_t preview_begin_index = solve_plan.chunks.size();
            std::size_t preview_end_index = solve_plan.chunks.size();
            if (preview_staging_active)
            {
                const double preview_t0_s = planned_request.preview_patch.anchor_time_s;
                const double preview_t1_s =
                        planned_request.preview_patch.anchor_time_s +
                        (2.0 * planned_request.preview_patch.exact_window_s);
                const double preview_epsilon_s = continuity_time_epsilon_s(preview_t0_s);
                for (std::size_t chunk_index = 0; chunk_index < solve_plan.chunks.size(); ++chunk_index)
                {
                    const PredictionChunkPlan &chunk = solve_plan.chunks[chunk_index];
                    const bool chunk_in_preview_range =
                            chunk.t0_s >= (preview_t0_s - preview_epsilon_s) &&
                            chunk.t1_s <= (preview_t1_s + preview_epsilon_s);
                    if (!chunk_in_preview_range)
                    {
                        if (preview_begin_index != solve_plan.chunks.size())
                        {
                            preview_end_index = chunk_index;
                            break;
                        }
                        continue;
                    }

                    if (preview_begin_index == solve_plan.chunks.size())
                    {
                        preview_begin_index = chunk_index;
                    }
                }

                if (preview_begin_index != solve_plan.chunks.size() && preview_end_index == solve_plan.chunks.size())
                {
                    preview_end_index = solve_plan.chunks.size();
                }
            }

            if (preview_begin_index != solve_plan.chunks.size() && preview_begin_index < preview_end_index)
            {
                published_staged_preview = true;
                PlannedSolveOutput prefix_stage_output{};
                std::vector<PublishedChunk> prefix_stage_chunks;
                bool prefix_stage_solved = false;

                orbitsim::State preview_start_state = ship_sc.state;
                if (planned_request.preview_patch.anchor_state_valid &&
                    finite_state(planned_request.preview_patch.anchor_state_inertial))
                {
                    preview_start_state = planned_request.preview_patch.anchor_state_inertial;
                }
                else if (preview_begin_index > 0u)
                {
                    const PlannedSolveRangeSummary prefix_summary =
                            solve_stage_range(0u,
                                              preview_begin_index,
                                              ship_sc.state,
                                              ChunkQualityState::Final,
                                              0u,
                                              prefix_stage_output,
                                              prefix_stage_chunks);
                    if (prefix_stage_output.status != Status::Success ||
                        prefix_summary.status != Status::Success)
                    {
                        fail_with_planned_diagnostics(prefix_summary.status, prefix_stage_output);
                        return;
                    }
                    preview_start_state = prefix_summary.end_state;
                    prefix_stage_solved = true;
                }

                PlannedSolveOutput preview_stage_output{};
                std::vector<PublishedChunk> preview_stage_chunks;
                const PlannedSolveRangeSummary preview_summary =
                        solve_and_publish_stage(preview_begin_index,
                                                preview_end_index,
                                                preview_start_state,
                                                PublishStage::PreviewStreaming,
                                                ChunkQualityState::PreviewPatch,
                                                0u,
                                                preview_stage_output,
                                                preview_stage_chunks);
                if (preview_stage_output.status != Status::Success || preview_summary.status != Status::Success)
                {
                    fail_with_planned_diagnostics(preview_summary.status, preview_stage_output);
                    return;
                }

                if (preview_begin_index > 0u && !prefix_stage_solved)
                {
                    const PlannedSolveRangeSummary prefix_summary =
                            solve_stage_range(0u,
                                              preview_begin_index,
                                              ship_sc.state,
                                              ChunkQualityState::Final,
                                              0u,
                                              prefix_stage_output,
                                              prefix_stage_chunks);
                    if (prefix_stage_output.status != Status::Success ||
                        prefix_summary.status != Status::Success)
                    {
                        fail_with_planned_diagnostics(prefix_summary.status, prefix_stage_output);
                        return;
                    }
                    prefix_stage_solved = true;
                }

                const orbitsim::State final_preview_start_state =
                        prefix_stage_solved ? prefix_stage_output.end_state : ship_sc.state;
                PlannedSolveOutput final_preview_stage_output{};
                std::vector<PublishedChunk> final_preview_stage_chunks;
                const PlannedSolveOutput *final_preview_output = &preview_stage_output;
                const std::vector<PublishedChunk> *final_preview_chunks = &preview_stage_chunks;
                PlannedSolveRangeSummary final_preview_summary = preview_summary;
                if (!states_are_continuous(final_preview_start_state, preview_start_state))
                {
                    final_preview_summary =
                            solve_stage_range(preview_begin_index,
                                              preview_end_index,
                                              final_preview_start_state,
                                              ChunkQualityState::Final,
                                              static_cast<uint32_t>(prefix_stage_chunks.size()),
                                              final_preview_stage_output,
                                              final_preview_stage_chunks);
                    if (final_preview_stage_output.status != Status::Success ||
                        final_preview_summary.status != Status::Success)
                    {
                        fail_with_planned_diagnostics(final_preview_summary.status, final_preview_stage_output);
                        return;
                    }
                    final_preview_output = &final_preview_stage_output;
                    final_preview_chunks = &final_preview_stage_chunks;
                }

                PlannedSolveOutput suffix_stage_output{};
                std::vector<PublishedChunk> suffix_stage_chunks;
                if (preview_end_index < solve_plan.chunks.size())
                {
                    const PlannedSolveRangeSummary suffix_summary =
                            solve_stage_range(preview_end_index,
                                              solve_plan.chunks.size(),
                                              final_preview_summary.end_state,
                                              ChunkQualityState::Final,
                                              static_cast<uint32_t>(prefix_stage_chunks.size() +
                                                                    final_preview_chunks->size()),
                                              suffix_stage_output,
                                              suffix_stage_chunks);
                    if (suffix_stage_output.status != Status::Success ||
                        suffix_summary.status != Status::Success)
                    {
                        fail_with_planned_diagnostics(suffix_summary.status, suffix_stage_output);
                        return;
                    }
                }

                PlannedSolveOutput final_stage_output{};
                if (prefix_stage_solved)
                {
                    merge_planned_outputs(final_stage_output, prefix_stage_output);
                }
                merge_planned_outputs(final_stage_output, *final_preview_output);
                if (!suffix_stage_output.segments.empty())
                {
                    merge_planned_outputs(final_stage_output, suffix_stage_output);
                }

                std::vector<PublishedChunk> final_stage_chunks;
                append_published_chunks_as_final(final_stage_chunks, prefix_stage_chunks);
                append_published_chunks_as_final(final_stage_chunks, *final_preview_chunks);
                append_published_chunks_as_final(final_stage_chunks, suffix_stage_chunks);
                publish(make_stage_result(final_stage_output,
                                          final_stage_chunks,
                                          PublishStage::Final));
            }
            else
            {
                const auto solve_full_planned_path = [&]() -> bool {
                    PlannedSolveOutput planned{};
                    std::vector<PublishedChunk> published_chunks;
                    std::vector<PublishedChunk> pending_stream_published_chunks;
                    std::vector<StreamedPlannedChunk> pending_stream_chunks;
                    const bool full_streaming_active =
                            planned_request.solve_quality == SolveQuality::Full &&
                            planned_request.full_stream_publish.active;
                    const double full_stream_min_publish_interval_s =
                            (planned_request.full_stream_publish.min_publish_interval_s > 0.0)
                                    ? planned_request.full_stream_publish.min_publish_interval_s
                                    : OrbitPredictionTuning::kFullStreamPublishMinIntervalS;
                    std::chrono::steady_clock::time_point last_full_stream_publish_tp = compute_start;
                    bool published_any_full_stream_batch = false;
                    const auto flush_full_stream_batch = [&](const bool force_publish) {
                        if (!full_streaming_active || pending_stream_published_chunks.empty())
                        {
                            return true;
                        }

                        const auto now_tp = std::chrono::steady_clock::now();
                        const double elapsed_since_last_publish_s =
                                std::chrono::duration<double>(now_tp - last_full_stream_publish_tp).count();
                        if (!force_publish &&
                            published_any_full_stream_batch &&
                            elapsed_since_last_publish_s < full_stream_min_publish_interval_s)
                        {
                            return true;
                        }

                        if (!publish(make_stage_result(planned,
                                                       pending_stream_published_chunks,
                                                       PublishStage::FullStreaming,
                                                       std::move(pending_stream_chunks),
                                                       false)))
                        {
                            pending_stream_published_chunks.clear();
                            pending_stream_chunks.clear();
                            return false;
                        }

                        pending_stream_published_chunks.clear();
                        pending_stream_chunks.clear();
                        last_full_stream_publish_tp = now_tp;
                        published_any_full_stream_batch = true;
                        return true;
                    };
                    const PlannedSolveRangeSummary planned_summary =
                            solve_planned_chunk_range(ctx,
                                                      solve_plan,
                                                      0u,
                                                      solve_plan.chunks.size(),
                                                      ship_sc.state,
                                                      [&planned,
                                                       &published_chunks,
                                                       &pending_stream_published_chunks,
                                                       &pending_stream_chunks,
                                                       &flush_full_stream_batch,
                                                       full_streaming_active](PlannedChunkPacket &&chunk_packet) {
                                                          const PublishedChunk published_chunk = make_published_chunk(
                                                                  chunk_packet,
                                                                  static_cast<uint32_t>(published_chunks.size()),
                                                                  ChunkQualityState::Final);
                                                          published_chunks.push_back(published_chunk);
                                                          if (full_streaming_active)
                                                          {
                                                              pending_stream_published_chunks.push_back(published_chunk);
                                                              pending_stream_chunks.push_back(
                                                                      make_streamed_planned_chunk(chunk_packet,
                                                                                                  published_chunk));
                                                          }
                                                          append_planned_chunk_packet(planned, std::move(chunk_packet));
                                                          return flush_full_stream_batch(false);
                                                      });
                    apply_planned_range_summary(planned, planned_summary);
                    if (planned.status != Status::Success)
                    {
                        fail_with_planned_diagnostics(planned.status, planned);
                        return false;
                    }
                    if (!flush_full_stream_batch(true))
                    {
                        cancel();
                        return false;
                    }

                    out.diagnostics.trajectory_planned = planned.diagnostics;
                    out.diagnostics.trajectory_sample_count_planned = planned.samples.size();
                    sync_stage_counts();
                    out.trajectory_segments_inertial_planned = planned.segments;
                    out.trajectory_inertial_planned = planned.samples;
                    out.maneuver_previews = planned.previews;
                    out.published_chunks = std::move(published_chunks);
                    return true;
                };

                const auto solve_planned_suffix_refine_path = [&]() -> Status {
                    const std::optional<std::size_t> suffix_begin_index =
                            find_planned_suffix_begin_chunk(solve_plan,
                                                            planned_request.planned_suffix_refine.anchor_time_s);
                    if (!suffix_begin_index.has_value() ||
                        !planned_suffix_refine_prefix_is_valid(planned_request,
                                                               solve_plan,
                                                               *suffix_begin_index))
                    {
                        return Status::InvalidInput;
                    }

                    PlannedSolveOutput prefix_output =
                            make_planned_suffix_refine_prefix_output(planned_request);
                    if (prefix_output.status != Status::Success)
                    {
                        return prefix_output.status;
                    }

                    const bool full_streaming_active =
                            planned_request.solve_quality == SolveQuality::Full &&
                            planned_request.full_stream_publish.active;
                    const double full_stream_min_publish_interval_s =
                            (planned_request.full_stream_publish.min_publish_interval_s > 0.0)
                                    ? planned_request.full_stream_publish.min_publish_interval_s
                                    : OrbitPredictionTuning::kFullStreamPublishMinIntervalS;
                    std::chrono::steady_clock::time_point last_full_stream_publish_tp = compute_start;
                    bool published_any_full_stream_batch = false;

                    if (full_streaming_active && *suffix_begin_index > 0u)
                    {
                        std::vector<PublishedChunk> prefix_stream_published_chunks;
                        std::vector<StreamedPlannedChunk> prefix_stream_chunks;
                        if (build_cached_prefix_stream_chunks(planned_request,
                                                              solve_plan,
                                                              *suffix_begin_index,
                                                              prefix_output,
                                                              prefix_stream_published_chunks,
                                                              prefix_stream_chunks) &&
                            !prefix_stream_published_chunks.empty())
                        {
                            if (!publish(make_stage_result(prefix_output,
                                                           prefix_stream_published_chunks,
                                                           PublishStage::FullStreaming,
                                                           std::move(prefix_stream_chunks),
                                                           false)))
                            {
                                return Status::Cancelled;
                            }
                            last_full_stream_publish_tp = std::chrono::steady_clock::now();
                            published_any_full_stream_batch = true;
                        }
                    }

                    PlannedSolveOutput suffix_output{};
                    std::vector<PublishedChunk> suffix_published_chunks;
                    std::vector<PublishedChunk> pending_stream_published_chunks;
                    std::vector<StreamedPlannedChunk> pending_stream_chunks;
                    const auto flush_full_stream_batch = [&](const bool force_publish) {
                        if (!full_streaming_active || pending_stream_published_chunks.empty())
                        {
                            return true;
                        }

                        const auto now_tp = std::chrono::steady_clock::now();
                        const double elapsed_since_last_publish_s =
                                std::chrono::duration<double>(now_tp - last_full_stream_publish_tp).count();
                        if (!force_publish &&
                            published_any_full_stream_batch &&
                            elapsed_since_last_publish_s < full_stream_min_publish_interval_s)
                        {
                            return true;
                        }

                        if (!publish(make_stage_result(suffix_output,
                                                       pending_stream_published_chunks,
                                                       PublishStage::FullStreaming,
                                                       std::move(pending_stream_chunks),
                                                       false)))
                        {
                            pending_stream_published_chunks.clear();
                            pending_stream_chunks.clear();
                            return false;
                        }

                        pending_stream_published_chunks.clear();
                        pending_stream_chunks.clear();
                        last_full_stream_publish_tp = now_tp;
                        published_any_full_stream_batch = true;
                        return true;
                    };

                    const PlannedSolveRangeSummary suffix_summary =
                            solve_planned_chunk_range(ctx,
                                                      solve_plan,
                                                      *suffix_begin_index,
                                                      solve_plan.chunks.size(),
                                                      planned_request.planned_suffix_refine.anchor_state_inertial,
                                                      [&suffix_output,
                                                       &suffix_published_chunks,
                                                       &pending_stream_published_chunks,
                                                       &pending_stream_chunks,
                                                       &flush_full_stream_batch,
                                                       full_streaming_active](PlannedChunkPacket &&chunk_packet) {
                                                          const PublishedChunk published_chunk = make_published_chunk(
                                                                  chunk_packet,
                                                                  chunk_packet.chunk.chunk_id,
                                                                  ChunkQualityState::Final);
                                                          suffix_published_chunks.push_back(published_chunk);
                                                          if (full_streaming_active)
                                                          {
                                                              pending_stream_published_chunks.push_back(published_chunk);
                                                              pending_stream_chunks.push_back(
                                                                      make_streamed_planned_chunk(chunk_packet,
                                                                                                  published_chunk));
                                                          }
                                                          append_planned_chunk_packet(suffix_output,
                                                                                      std::move(chunk_packet));
                                                          return flush_full_stream_batch(false);
                                                      });
                    apply_planned_range_summary(suffix_output, suffix_summary);
                    if (suffix_output.status != Status::Success)
                    {
                        return suffix_output.status;
                    }
                    if (!flush_full_stream_batch(true))
                    {
                        return Status::Cancelled;
                    }

                    PlannedSolveOutput merged_output = std::move(prefix_output);
                    merge_planned_outputs(merged_output, suffix_output);
                    if (merged_output.segments.empty() ||
                        !validate_trajectory_segment_continuity(merged_output.segments))
                    {
                        return Status::ContinuityFailed;
                    }

                    merged_output.samples =
                            resample_segments_uniform(merged_output.segments,
                                                      prediction_sample_budget(planned_request,
                                                                               merged_output.segments.size()));
                    if (merged_output.samples.size() < 2u)
                    {
                        return Status::TrajectorySamplesUnavailable;
                    }

                    std::vector<PublishedChunk> final_published_chunks;
                    final_published_chunks.reserve(*suffix_begin_index + suffix_published_chunks.size());
                    for (std::size_t chunk_index = 0u; chunk_index < *suffix_begin_index; ++chunk_index)
                    {
                        final_published_chunks.push_back(
                                make_published_chunk_from_plan(solve_plan.chunks[chunk_index],
                                                               solve_plan.chunks[chunk_index].chunk_id,
                                                               true));
                    }
                    final_published_chunks.insert(final_published_chunks.end(),
                                                  suffix_published_chunks.begin(),
                                                  suffix_published_chunks.end());

                    out.diagnostics.trajectory_planned = merged_output.diagnostics;
                    out.diagnostics.trajectory_sample_count_planned = merged_output.samples.size();
                    sync_stage_counts();
                    out.trajectory_segments_inertial_planned = std::move(merged_output.segments);
                    out.trajectory_inertial_planned = std::move(merged_output.samples);
                    out.maneuver_previews = std::move(merged_output.previews);
                    out.published_chunks = std::move(final_published_chunks);
                    return Status::Success;
                };

                bool planned_path_solved = false;
                if (planned_request.planned_suffix_refine.active)
                {
                    const Status suffix_status = solve_planned_suffix_refine_path();
                    if (suffix_status == Status::Success)
                    {
                        planned_path_solved = true;
                    }
                    else if (suffix_status == Status::Cancelled)
                    {
                        cancel();
                        return;
                    }
                }

                if (!planned_path_solved && !solve_full_planned_path())
                {
                    return;
                }
            }
        }

        store_reusable_baseline(request.track_id,
                                generation_id,
                                request_epoch,
                                out.shared_ephemeris,
                                out.trajectory_inertial,
                                out.trajectory_segments_inertial);
        if (published_staged_preview)
        {
            return;
        }
        ensure_single_publish_chunk_metadata(out);
        out.valid = true;
        out.diagnostics.status = Status::Success;
        publish(std::move(out));
        return;
    }
} // namespace Game
