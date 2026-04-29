#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <chrono>

namespace Game
{
    namespace
    {
        using PredictionClock = std::chrono::steady_clock;

        void sync_prediction_stage_counts(OrbitPredictionService::Result &result)
        {
            result.diagnostics.ephemeris_segment_count = result.diagnostics.ephemeris.accepted_segments;
            result.diagnostics.trajectory_segment_count = result.diagnostics.trajectory_base.accepted_segments;
            result.diagnostics.trajectory_segment_count_planned = result.diagnostics.trajectory_planned.accepted_segments;
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

        struct PlannedPathSolveEnvironment
        {
            const OrbitPredictionService::Request &request;
            const OrbitPredictionService::PredictionSolvePlan &solve_plan;
            PlannedTrajectoryContext &ctx;
            PublishFn publish;
            FullStreamBatchPublisher::MakeStageResultFn make_stage_result;
            PredictionClock::time_point compute_start;
        };

        struct PlannedPathSolveOutcome
        {
            OrbitPredictionService::Status status{OrbitPredictionService::Status::Success};
            PlannedSolveOutput output{};
            std::vector<OrbitPredictionService::PublishedChunk> published_chunks{};
        };

        struct FullPlannedPathChunkSink
        {
            PlannedPathSolveOutcome &outcome;
            FullStreamBatchPublisher &full_stream;
            const PlannedPathSolveEnvironment &env;

            bool operator()(PlannedChunkPacket &&chunk_packet)
            {
                const OrbitPredictionService::PublishedChunk published_chunk =
                        make_published_chunk(
                                chunk_packet,
                                static_cast<uint32_t>(outcome.published_chunks.size()),
                                OrbitPredictionService::ChunkQualityState::Final);
                outcome.published_chunks.push_back(published_chunk);
                full_stream.append(published_chunk, chunk_packet);
                append_planned_chunk_packet(outcome.output, std::move(chunk_packet));
                return full_stream.flush(outcome.output,
                                         env.make_stage_result,
                                         env.publish,
                                         false);
            }
        };

        struct SuffixRefineChunkSink
        {
            PlannedSolveOutput &suffix_output;
            std::vector<OrbitPredictionService::PublishedChunk> &suffix_published_chunks;
            FullStreamBatchPublisher &full_stream;
            const PlannedPathSolveEnvironment &env;

            bool operator()(PlannedChunkPacket &&chunk_packet)
            {
                const OrbitPredictionService::PublishedChunk published_chunk =
                        make_published_chunk(
                                chunk_packet,
                                chunk_packet.chunk.chunk_id,
                                OrbitPredictionService::ChunkQualityState::Final);
                suffix_published_chunks.push_back(published_chunk);
                full_stream.append(published_chunk, chunk_packet);
                append_planned_chunk_packet(suffix_output, std::move(chunk_packet));
                return full_stream.flush(suffix_output,
                                         env.make_stage_result,
                                         env.publish,
                                         false);
            }
        };

        PlannedPathSolveOutcome solve_full_planned_path(
                const PlannedPathSolveEnvironment &env,
                const orbitsim::State &start_state)
        {
            PlannedPathSolveOutcome outcome{};
            FullStreamBatchPublisher full_stream =
                    make_full_stream_batch_publisher(env.request, env.compute_start);

            const PlannedSolveRangeSummary summary =
                    solve_planned_chunk_range(env.ctx,
                                              env.solve_plan,
                                              0u,
                                              env.solve_plan.chunks.size(),
                                              start_state,
                                              FullPlannedPathChunkSink{
                                                      outcome,
                                                      full_stream,
                                                      env,
                                              });
            apply_planned_range_summary(outcome.output, summary);
            if (outcome.output.status != OrbitPredictionService::Status::Success)
            {
                outcome.status = outcome.output.status;
                return outcome;
            }

            if (!full_stream.flush(outcome.output, env.make_stage_result, env.publish, true))
            {
                outcome.status = OrbitPredictionService::Status::Cancelled;
                return outcome;
            }

            outcome.status = OrbitPredictionService::Status::Success;
            return outcome;
        }

        PlannedPathSolveOutcome solve_planned_suffix_refine_path(
                const PlannedPathSolveEnvironment &env)
        {
            PlannedPathSolveOutcome outcome{};
            const std::optional<std::size_t> suffix_begin_index =
                    find_planned_suffix_begin_chunk(env.solve_plan,
                                                    env.request.planned_suffix_refine.anchor_time_s);
            if (!suffix_begin_index.has_value() ||
                !planned_suffix_refine_prefix_is_valid(env.request,
                                                       env.solve_plan,
                                                       *suffix_begin_index))
            {
                outcome.status = OrbitPredictionService::Status::InvalidInput;
                return outcome;
            }

            PlannedSolveOutput prefix_output = make_planned_suffix_refine_prefix_output(env.request);
            if (prefix_output.status != OrbitPredictionService::Status::Success)
            {
                outcome.status = prefix_output.status;
                outcome.output = std::move(prefix_output);
                return outcome;
            }

            FullStreamBatchPublisher full_stream =
                    make_full_stream_batch_publisher(env.request, env.compute_start);
            if (full_stream.active && *suffix_begin_index > 0u)
            {
                std::vector<OrbitPredictionService::PublishedChunk> prefix_stream_published_chunks;
                std::vector<OrbitPredictionService::StreamedPlannedChunk> prefix_stream_chunks;
                if (build_cached_prefix_stream_chunks(env.request,
                                                      env.solve_plan,
                                                      *suffix_begin_index,
                                                      prefix_output,
                                                      prefix_stream_published_chunks,
                                                      prefix_stream_chunks) &&
                    !prefix_stream_published_chunks.empty())
                {
                    if (!env.publish(env.make_stage_result(prefix_output,
                                                           prefix_stream_published_chunks,
                                                           OrbitPredictionService::PublishStage::FullStreaming,
                                                           std::move(prefix_stream_chunks),
                                                           false)))
                    {
                        outcome.status = OrbitPredictionService::Status::Cancelled;
                        outcome.output = std::move(prefix_output);
                        return outcome;
                    }
                    full_stream.last_publish_tp = PredictionClock::now();
                    full_stream.published_any_batch = true;
                }
            }

            PlannedSolveOutput suffix_output{};
            std::vector<OrbitPredictionService::PublishedChunk> suffix_published_chunks;
            const PlannedSolveRangeSummary suffix_summary =
                    solve_planned_chunk_range(env.ctx,
                                              env.solve_plan,
                                              *suffix_begin_index,
                                              env.solve_plan.chunks.size(),
                                              env.request.planned_suffix_refine.anchor_state_inertial,
                                              SuffixRefineChunkSink{
                                                      suffix_output,
                                                      suffix_published_chunks,
                                                      full_stream,
                                                      env,
                                              });
            apply_planned_range_summary(suffix_output, suffix_summary);
            if (suffix_output.status != OrbitPredictionService::Status::Success)
            {
                outcome.status = suffix_output.status;
                outcome.output = std::move(suffix_output);
                return outcome;
            }

            if (!full_stream.flush(suffix_output, env.make_stage_result, env.publish, true))
            {
                outcome.status = OrbitPredictionService::Status::Cancelled;
                outcome.output = std::move(suffix_output);
                return outcome;
            }

            PlannedSolveOutput merged_output = std::move(prefix_output);
            merge_planned_outputs(merged_output, suffix_output);
            if (merged_output.segments.empty() ||
                !validate_trajectory_segment_continuity(merged_output.segments))
            {
                outcome.status = OrbitPredictionService::Status::ContinuityFailed;
                outcome.output = std::move(merged_output);
                return outcome;
            }

            merged_output.samples =
                    resample_segments_uniform(merged_output.segments,
                                              prediction_sample_budget(env.request,
                                                                       merged_output.segments.size()));
            if (merged_output.samples.size() < 2u)
            {
                outcome.status = OrbitPredictionService::Status::TrajectorySamplesUnavailable;
                outcome.output = std::move(merged_output);
                return outcome;
            }

            outcome.published_chunks.reserve(*suffix_begin_index + suffix_published_chunks.size());
            for (std::size_t chunk_index = 0u; chunk_index < *suffix_begin_index; ++chunk_index)
            {
                outcome.published_chunks.push_back(
                        make_published_chunk_from_plan(env.solve_plan.chunks[chunk_index],
                                                       env.solve_plan.chunks[chunk_index].chunk_id,
                                                       true));
            }
            outcome.published_chunks.insert(outcome.published_chunks.end(),
                                            suffix_published_chunks.begin(),
                                            suffix_published_chunks.end());

            outcome.status = OrbitPredictionService::Status::Success;
            outcome.output = std::move(merged_output);
            return outcome;
        }

        struct ManeuverImpulseByTime
        {
            bool operator()(const OrbitPredictionService::ManeuverImpulse &a,
                            const OrbitPredictionService::ManeuverImpulse &b) const
            {
                return a.t_s < b.t_s;
            }
        };

        struct PreviewChunkRange
        {
            std::size_t begin_index{0u};
            std::size_t end_index{0u};
            bool valid{false};
        };

        class PlannedPredictionRouteRunner
        {
        public:
            explicit PlannedPredictionRouteRunner(const PlannedPredictionRouteEnvironment &env)
                : env_(env),
                  planned_request_(make_planned_request(env.request)),
                  ctx_{
                          planned_request_,
                          env.callbacks.cancel_requested,
                          env.out,
                          env.ephemeris,
                          env.callbacks.publish,
                          env.services,
                  },
                  solve_plan_(build_prediction_solve_plan(planned_request_)),
                  staged_core_data_{env.out}
            {
            }

            PlannedPredictionRouteOutcome run()
            {
                using Status = OrbitPredictionService::Status;

                PlannedPredictionRouteOutcome route_outcome{};
                if (env_.request.maneuver_impulses.empty())
                {
                    return route_outcome;
                }

                if (!solve_plan_.valid)
                {
                    route_outcome.status = Status::InvalidInput;
                    return route_outcome;
                }

                const PreviewChunkRange preview_range = find_preview_chunk_range();
                if (preview_range.valid)
                {
                    return run_preview_staging(preview_range);
                }

                return run_final_or_suffix_refine();
            }

        private:
            using ChunkQualityState = OrbitPredictionService::ChunkQualityState;
            using PredictionChunkPlan = OrbitPredictionService::PredictionChunkPlan;
            using PublishedChunk = OrbitPredictionService::PublishedChunk;
            using PublishStage = OrbitPredictionService::PublishStage;
            using Request = OrbitPredictionService::Request;
            using Result = OrbitPredictionService::Result;
            using SolveQuality = OrbitPredictionService::SolveQuality;
            using Status = OrbitPredictionService::Status;
            using StreamedPlannedChunk = OrbitPredictionService::StreamedPlannedChunk;

            static Request make_planned_request(const Request &request)
            {
                Request planned_request = request;
                std::stable_sort(planned_request.maneuver_impulses.begin(),
                                 planned_request.maneuver_impulses.end(),
                                 ManeuverImpulseByTime{});
                return planned_request;
            }

            Status fail_with_planned_diagnostics(const Status status,
                                                 const PlannedSolveOutput &planned_output)
            {
                env_.out.diagnostics.trajectory_planned = planned_output.diagnostics;
                env_.out.diagnostics.trajectory_sample_count_planned = planned_output.samples.size();
                sync_prediction_stage_counts(env_.out);
                return status;
            }

            Result make_stage_result(
                    const PlannedSolveOutput &stage_output,
                    const std::vector<PublishedChunk> &published_chunks,
                    const PublishStage publish_stage,
                    std::vector<StreamedPlannedChunk> streamed_planned_chunks = {},
                    const bool include_cumulative_planned = true)
            {
                return make_planned_stage_result(env_.out,
                                                 stage_output,
                                                 published_chunks,
                                                 publish_stage,
                                                 staged_core_data_.get(),
                                                 std::move(streamed_planned_chunks),
                                                 include_cumulative_planned);
            }

            struct StageRangeChunkSink
            {
                PlannedPredictionRouteRunner &runner;
                ChunkQualityState quality_state{ChunkQualityState::Final};
                uint32_t published_chunk_base_id{0u};
                PlannedSolveOutput &stage_output;
                std::vector<PublishedChunk> &stage_published_chunks;
                bool publish_after_append{false};
                PublishStage publish_stage{PublishStage::Final};

                bool operator()(PlannedChunkPacket &&chunk_packet)
                {
                    const uint32_t published_chunk_id =
                            published_chunk_base_id +
                            static_cast<uint32_t>(stage_published_chunks.size());
                    append_published_chunk(stage_published_chunks,
                                           chunk_packet,
                                           published_chunk_id,
                                           quality_state);
                    append_planned_chunk_packet(stage_output,
                                                std::move(chunk_packet));
                    if (!publish_after_append)
                    {
                        return true;
                    }

                    return runner.env_.callbacks.publish(
                            runner.make_stage_result(stage_output,
                                                     stage_published_chunks,
                                                     publish_stage));
                }
            };

            PlannedSolveRangeSummary solve_stage_range(
                    const std::size_t chunk_begin_index,
                    const std::size_t chunk_end_index,
                    const orbitsim::State &range_start_state,
                    const ChunkQualityState quality_state,
                    const uint32_t published_chunk_base_id,
                    PlannedSolveOutput &stage_output,
                    std::vector<PublishedChunk> &stage_published_chunks)
            {
                const PlannedSolveRangeSummary summary =
                        solve_planned_chunk_range(ctx_,
                                                  solve_plan_,
                                                  chunk_begin_index,
                                                  chunk_end_index,
                                                  range_start_state,
                                                  StageRangeChunkSink{
                                                          *this,
                                                          quality_state,
                                                          published_chunk_base_id,
                                                          stage_output,
                                                          stage_published_chunks,
                                                          false,
                                                          PublishStage::Final,
                                                  });
                apply_planned_range_summary(stage_output, summary);
                return summary;
            }

            PlannedSolveRangeSummary solve_and_publish_stage(
                    const std::size_t chunk_begin_index,
                    const std::size_t chunk_end_index,
                    const orbitsim::State &range_start_state,
                    const PublishStage publish_stage,
                    const ChunkQualityState quality_state,
                    const uint32_t published_chunk_base_id,
                    PlannedSolveOutput &stage_output,
                    std::vector<PublishedChunk> &stage_published_chunks)
            {
                const PlannedSolveRangeSummary summary =
                        solve_planned_chunk_range(ctx_,
                                                  solve_plan_,
                                                  chunk_begin_index,
                                                  chunk_end_index,
                                                  range_start_state,
                                                  StageRangeChunkSink{
                                                          *this,
                                                          quality_state,
                                                          published_chunk_base_id,
                                                          stage_output,
                                                          stage_published_chunks,
                                                          true,
                                                          publish_stage,
                                                  });
                apply_planned_range_summary(stage_output, summary);
                return summary;
            }

            PreviewChunkRange find_preview_chunk_range() const
            {
                const bool preview_staging_active =
                        planned_request_.solve_quality == SolveQuality::FastPreview &&
                        planned_request_.preview_patch.active;
                if (!preview_staging_active)
                {
                    return {};
                }

                std::size_t preview_begin_index = solve_plan_.chunks.size();
                std::size_t preview_end_index = solve_plan_.chunks.size();
                const double preview_t0_s = planned_request_.preview_patch.anchor_time_s;
                const double preview_t1_s =
                        planned_request_.preview_patch.anchor_time_s +
                        (2.0 * planned_request_.preview_patch.exact_window_s);
                const double preview_epsilon_s = continuity_time_epsilon_s(preview_t0_s);
                for (std::size_t chunk_index = 0; chunk_index < solve_plan_.chunks.size(); ++chunk_index)
                {
                    const PredictionChunkPlan &chunk = solve_plan_.chunks[chunk_index];
                    const bool chunk_in_preview_range =
                            chunk.t0_s >= (preview_t0_s - preview_epsilon_s) &&
                            chunk.t1_s <= (preview_t1_s + preview_epsilon_s);
                    if (!chunk_in_preview_range)
                    {
                        if (preview_begin_index != solve_plan_.chunks.size())
                        {
                            preview_end_index = chunk_index;
                            break;
                        }
                        continue;
                    }

                    if (preview_begin_index == solve_plan_.chunks.size())
                    {
                        preview_begin_index = chunk_index;
                    }
                }

                if (preview_begin_index != solve_plan_.chunks.size() &&
                    preview_end_index == solve_plan_.chunks.size())
                {
                    preview_end_index = solve_plan_.chunks.size();
                }

                return PreviewChunkRange{
                        .begin_index = preview_begin_index,
                        .end_index = preview_end_index,
                        .valid = preview_begin_index != solve_plan_.chunks.size() &&
                                 preview_begin_index < preview_end_index,
                };
            }

            PlannedPredictionRouteOutcome run_preview_staging(const PreviewChunkRange &preview_range)
            {
                PlannedPredictionRouteOutcome route_outcome{};
                route_outcome.published_staged_preview = true;

                PlannedSolveOutput prefix_stage_output{};
                std::vector<PublishedChunk> prefix_stage_chunks;
                bool prefix_stage_solved = false;

                orbitsim::State preview_start_state = env_.ship_state;
                if (planned_request_.preview_patch.anchor_state_valid &&
                    planned_request_.preview_patch.anchor_state_trusted &&
                    finite_state(planned_request_.preview_patch.anchor_state_inertial))
                {
                    preview_start_state = planned_request_.preview_patch.anchor_state_inertial;
                }
                else if (preview_range.begin_index > 0u)
                {
                    const PlannedSolveRangeSummary prefix_summary =
                            solve_stage_range(0u,
                                              preview_range.begin_index,
                                              env_.ship_state,
                                              ChunkQualityState::Final,
                                              0u,
                                              prefix_stage_output,
                                              prefix_stage_chunks);
                    if (prefix_stage_output.status != Status::Success ||
                        prefix_summary.status != Status::Success)
                    {
                        route_outcome.status =
                                fail_with_planned_diagnostics(prefix_summary.status, prefix_stage_output);
                        return route_outcome;
                    }
                    preview_start_state = prefix_summary.end_state;
                    prefix_stage_solved = true;
                }

                PlannedSolveOutput preview_stage_output{};
                std::vector<PublishedChunk> preview_stage_chunks;
                const PlannedSolveRangeSummary preview_summary =
                        solve_and_publish_stage(preview_range.begin_index,
                                                preview_range.end_index,
                                                preview_start_state,
                                                PublishStage::PreviewStreaming,
                                                ChunkQualityState::PreviewPatch,
                                                0u,
                                                preview_stage_output,
                                                preview_stage_chunks);
                if (preview_stage_output.status != Status::Success ||
                    preview_summary.status != Status::Success)
                {
                    route_outcome.status =
                            fail_with_planned_diagnostics(preview_summary.status, preview_stage_output);
                    return route_outcome;
                }

                if (preview_range.begin_index > 0u && !prefix_stage_solved)
                {
                    const PlannedSolveRangeSummary prefix_summary =
                            solve_stage_range(0u,
                                              preview_range.begin_index,
                                              env_.ship_state,
                                              ChunkQualityState::Final,
                                              0u,
                                              prefix_stage_output,
                                              prefix_stage_chunks);
                    if (prefix_stage_output.status != Status::Success ||
                        prefix_summary.status != Status::Success)
                    {
                        route_outcome.status =
                                fail_with_planned_diagnostics(prefix_summary.status, prefix_stage_output);
                        return route_outcome;
                    }
                    prefix_stage_solved = true;
                }

                const orbitsim::State final_preview_start_state =
                        prefix_stage_solved ? prefix_stage_output.end_state : env_.ship_state;
                PlannedSolveOutput final_preview_stage_output{};
                std::vector<PublishedChunk> final_preview_stage_chunks;
                const PlannedSolveOutput *final_preview_output = &preview_stage_output;
                const std::vector<PublishedChunk> *final_preview_chunks = &preview_stage_chunks;
                PlannedSolveRangeSummary final_preview_summary = preview_summary;
                if (!states_are_continuous(final_preview_start_state, preview_start_state))
                {
                    final_preview_summary =
                            solve_stage_range(preview_range.begin_index,
                                              preview_range.end_index,
                                              final_preview_start_state,
                                              ChunkQualityState::Final,
                                              static_cast<uint32_t>(prefix_stage_chunks.size()),
                                              final_preview_stage_output,
                                              final_preview_stage_chunks);
                    if (final_preview_stage_output.status != Status::Success ||
                        final_preview_summary.status != Status::Success)
                    {
                        route_outcome.status =
                                fail_with_planned_diagnostics(final_preview_summary.status,
                                                              final_preview_stage_output);
                        return route_outcome;
                    }
                    final_preview_output = &final_preview_stage_output;
                    final_preview_chunks = &final_preview_stage_chunks;
                }

                PlannedSolveOutput suffix_stage_output{};
                std::vector<PublishedChunk> suffix_stage_chunks;
                if (preview_range.end_index < solve_plan_.chunks.size())
                {
                    const PlannedSolveRangeSummary suffix_summary =
                            solve_stage_range(preview_range.end_index,
                                              solve_plan_.chunks.size(),
                                              final_preview_summary.end_state,
                                              ChunkQualityState::Final,
                                              static_cast<uint32_t>(prefix_stage_chunks.size() +
                                                                    final_preview_chunks->size()),
                                              suffix_stage_output,
                                              suffix_stage_chunks);
                    if (suffix_stage_output.status != Status::Success ||
                        suffix_summary.status != Status::Success)
                    {
                        route_outcome.status =
                                fail_with_planned_diagnostics(suffix_summary.status, suffix_stage_output);
                        return route_outcome;
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
                env_.callbacks.publish(make_stage_result(final_stage_output,
                                                         final_stage_chunks,
                                                         PublishStage::Final));
                return route_outcome;
            }

            void apply_planned_outcome(PlannedPathSolveOutcome outcome)
            {
                env_.out.diagnostics.trajectory_planned = outcome.output.diagnostics;
                env_.out.diagnostics.trajectory_sample_count_planned = outcome.output.samples.size();
                sync_prediction_stage_counts(env_.out);
                env_.out.trajectory_segments_inertial_planned = std::move(outcome.output.segments);
                env_.out.trajectory_inertial_planned = std::move(outcome.output.samples);
                env_.out.maneuver_previews = std::move(outcome.output.previews);
                env_.out.published_chunks = std::move(outcome.published_chunks);
            }

            PlannedPredictionRouteOutcome run_final_or_suffix_refine()
            {
                PlannedPredictionRouteOutcome route_outcome{};
                const PlannedPathSolveEnvironment planned_env{
                        planned_request_,
                        solve_plan_,
                        ctx_,
                        env_.callbacks.publish,
                        [this](const PlannedSolveOutput &stage_output,
                               const std::vector<PublishedChunk> &published_chunks,
                               const PublishStage publish_stage,
                               std::vector<StreamedPlannedChunk> streamed_planned_chunks,
                               const bool include_cumulative_planned) {
                            return make_stage_result(stage_output,
                                                     published_chunks,
                                                     publish_stage,
                                                     std::move(streamed_planned_chunks),
                                                     include_cumulative_planned);
                        },
                        env_.callbacks.compute_start,
                };

                bool planned_path_solved = false;
                if (planned_request_.planned_suffix_refine.active)
                {
                    PlannedPathSolveOutcome suffix_outcome = solve_planned_suffix_refine_path(planned_env);
                    if (suffix_outcome.status == Status::Success)
                    {
                        apply_planned_outcome(std::move(suffix_outcome));
                        planned_path_solved = true;
                    }
                    else if (suffix_outcome.status == Status::Cancelled)
                    {
                        route_outcome.status = Status::Cancelled;
                        return route_outcome;
                    }
                }

                if (!planned_path_solved)
                {
                    PlannedPathSolveOutcome full_outcome =
                            solve_full_planned_path(planned_env, env_.ship_state);
                    if (full_outcome.status == Status::Cancelled)
                    {
                        route_outcome.status = Status::Cancelled;
                        return route_outcome;
                    }
                    if (full_outcome.status != Status::Success)
                    {
                        route_outcome.status =
                                fail_with_planned_diagnostics(full_outcome.status, full_outcome.output);
                        return route_outcome;
                    }
                    apply_planned_outcome(std::move(full_outcome));
                }

                return route_outcome;
            }

            const PlannedPredictionRouteEnvironment &env_;
            Request planned_request_{};
            PlannedTrajectoryContext ctx_;
            OrbitPredictionService::PredictionSolvePlan solve_plan_{};
            StagedCoreDataBuilder staged_core_data_;
        };

    } // namespace

    PlannedPredictionRouteOutcome solve_planned_prediction_route(
            const PlannedPredictionRouteEnvironment &env)
    {
        PlannedPredictionRouteRunner runner(env);
        return runner.run();
    }
} // namespace Game
