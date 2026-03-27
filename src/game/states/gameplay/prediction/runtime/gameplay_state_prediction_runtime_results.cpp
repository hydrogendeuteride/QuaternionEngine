#include "game/states/gameplay/gameplay_state.h"

#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace Game
{
    namespace
    {
        bool frame_specs_match(const orbitsim::TrajectoryFrameSpec &a,
                               const orbitsim::TrajectoryFrameSpec &b)
        {
            return a.type == b.type &&
                   a.primary_body_id == b.primary_body_id &&
                   a.secondary_body_id == b.secondary_body_id &&
                   a.target_spacecraft_id == b.target_spacecraft_id;
        }

        bool cache_frame_version_matches(const OrbitPredictionCache &cache,
                                         const uint64_t display_frame_key,
                                         const uint64_t display_frame_revision)
        {
            return cache.display_frame_key == display_frame_key &&
                   cache.display_frame_revision == display_frame_revision;
        }

        bool cache_frame_version_matches(const OrbitPredictionCache &a,
                                         const OrbitPredictionCache &b)
        {
            return cache_frame_version_matches(a, b.display_frame_key, b.display_frame_revision);
        }

        bool chunk_assembly_frame_version_matches(const PredictionChunkAssembly &assembly,
                                                  const uint64_t display_frame_key,
                                                  const uint64_t display_frame_revision)
        {
            return assembly.display_frame_key == display_frame_key &&
                   assembly.display_frame_revision == display_frame_revision;
        }

        void clear_preview_planned_render_artifacts(OrbitPredictionCache &cache)
        {
            cache.gpu_roots_frame_planned.reset();
            cache.render_curve_frame_planned.clear();
        }

        OrbitPredictionCache merge_reused_base_frame_cache(const OrbitPredictionCache &base_cache,
                                                           OrbitPredictionCache preview_cache)
        {
            preview_cache.trajectory_frame = base_cache.trajectory_frame;
            preview_cache.trajectory_segments_frame = base_cache.trajectory_segments_frame;
            preview_cache.gpu_roots_frame = base_cache.gpu_roots_frame;
            preview_cache.render_curve_frame = base_cache.render_curve_frame;
            if (base_cache.analysis_cache_valid &&
                base_cache.analysis_cache_body_id != orbitsim::kInvalidBodyId &&
                preview_cache.analysis_cache_body_id == base_cache.analysis_cache_body_id)
            {
                preview_cache.trajectory_analysis_bci = base_cache.trajectory_analysis_bci;
                preview_cache.trajectory_segments_analysis_bci = base_cache.trajectory_segments_analysis_bci;
                preview_cache.analysis_cache_body_id = base_cache.analysis_cache_body_id;
                preview_cache.analysis_cache_valid = base_cache.analysis_cache_valid;
            }
            if (base_cache.metrics_valid &&
                base_cache.metrics_body_id != orbitsim::kInvalidBodyId &&
                preview_cache.metrics_body_id == base_cache.metrics_body_id)
            {
                preview_cache.altitude_km = base_cache.altitude_km;
                preview_cache.speed_kmps = base_cache.speed_kmps;
                preview_cache.semi_major_axis_m = base_cache.semi_major_axis_m;
                preview_cache.eccentricity = base_cache.eccentricity;
                preview_cache.orbital_period_s = base_cache.orbital_period_s;
                preview_cache.periapsis_alt_km = base_cache.periapsis_alt_km;
                preview_cache.apoapsis_alt_km = base_cache.apoapsis_alt_km;
                preview_cache.metrics_body_id = base_cache.metrics_body_id;
                preview_cache.metrics_valid = base_cache.metrics_valid;
            }
            preview_cache.valid = preview_cache.trajectory_inertial.size() >= 2 &&
                                  preview_cache.trajectory_frame.size() >= 2 &&
                                  !preview_cache.trajectory_segments_frame.empty();
            return preview_cache;
        }

        template<typename SampleT>
        std::vector<SampleT> merge_planned_sample_prefix(const std::vector<SampleT> &previous_samples,
                                                         const std::vector<SampleT> &patch_samples,
                                                         const double patch_t0_s)
        {
            if (previous_samples.empty() || patch_samples.empty() || !std::isfinite(patch_t0_s))
            {
                return patch_samples;
            }

            constexpr double kTimeEpsilonS = 1.0e-6;
            std::vector<SampleT> merged;
            merged.reserve(previous_samples.size() + patch_samples.size());
            for (const SampleT &sample : previous_samples)
            {
                if (!std::isfinite(sample.t_s) || sample.t_s >= (patch_t0_s - kTimeEpsilonS))
                {
                    break;
                }
                merged.push_back(sample);
            }
            merged.insert(merged.end(), patch_samples.begin(), patch_samples.end());
            return merged;
        }

        std::vector<orbitsim::TrajectorySegment> merge_planned_segment_prefix(
                const std::vector<orbitsim::TrajectorySegment> &previous_segments,
                const std::vector<orbitsim::TrajectorySegment> &patch_segments,
                const double patch_t0_s)
        {
            if (previous_segments.empty() || patch_segments.empty() || !std::isfinite(patch_t0_s))
            {
                return patch_segments;
            }

            constexpr double kTimeEpsilonS = 1.0e-6;
            std::vector<orbitsim::TrajectorySegment> merged;
            merged.reserve(previous_segments.size() + patch_segments.size());
            for (const orbitsim::TrajectorySegment &segment : previous_segments)
            {
                const double segment_t1_s = segment.t0_s + segment.dt_s;
                if (!std::isfinite(segment.t0_s) || !std::isfinite(segment_t1_s) || !(segment.dt_s > 0.0))
                {
                    continue;
                }
                if (segment_t1_s > (patch_t0_s + kTimeEpsilonS))
                {
                    break;
                }
                merged.push_back(segment);
            }
            merged.insert(merged.end(), patch_segments.begin(), patch_segments.end());
            return merged;
        }

        std::vector<OrbitPredictionCache::ManeuverNodePreview> merge_maneuver_previews(
                const std::vector<OrbitPredictionCache::ManeuverNodePreview> &previous_previews,
                const std::vector<OrbitPredictionCache::ManeuverNodePreview> &patch_previews)
        {
            std::vector<OrbitPredictionCache::ManeuverNodePreview> merged = previous_previews;
            for (const OrbitPredictionCache::ManeuverNodePreview &patch_preview : patch_previews)
            {
                const auto existing_it = std::find_if(
                        merged.begin(),
                        merged.end(),
                        [&patch_preview](const OrbitPredictionCache::ManeuverNodePreview &candidate) {
                            return candidate.node_id == patch_preview.node_id;
                        });
                if (existing_it != merged.end())
                {
                    *existing_it = patch_preview;
                }
                else
                {
                    merged.push_back(patch_preview);
                }
            }

            std::stable_sort(
                    merged.begin(),
                    merged.end(),
                    [](const OrbitPredictionCache::ManeuverNodePreview &a, const OrbitPredictionCache::ManeuverNodePreview &b) {
                        if (a.t_s == b.t_s)
                        {
                            return a.node_id < b.node_id;
                        }
                        return a.t_s < b.t_s;
                    });
            return merged;
        }

        OrbitPredictionCache merge_preview_planned_prefix_cache(const OrbitPredictionCache &previous_cache,
                                                                OrbitPredictionCache preview_cache)
        {
            if (!previous_cache.valid)
            {
                return preview_cache;
            }

            double inertial_patch_t0_s = std::numeric_limits<double>::quiet_NaN();
            if (!preview_cache.trajectory_segments_inertial_planned.empty())
            {
                inertial_patch_t0_s = preview_cache.trajectory_segments_inertial_planned.front().t0_s;
            }
            else if (!preview_cache.trajectory_inertial_planned.empty())
            {
                inertial_patch_t0_s = preview_cache.trajectory_inertial_planned.front().t_s;
            }

            if (std::isfinite(inertial_patch_t0_s))
            {
                preview_cache.trajectory_inertial_planned = merge_planned_sample_prefix(
                        previous_cache.trajectory_inertial_planned,
                        preview_cache.trajectory_inertial_planned,
                        inertial_patch_t0_s);
                preview_cache.trajectory_segments_inertial_planned = merge_planned_segment_prefix(
                        previous_cache.trajectory_segments_inertial_planned,
                        preview_cache.trajectory_segments_inertial_planned,
                        inertial_patch_t0_s);
            }

            double frame_patch_t0_s = std::numeric_limits<double>::quiet_NaN();
            if (!preview_cache.trajectory_segments_frame_planned.empty())
            {
                frame_patch_t0_s = preview_cache.trajectory_segments_frame_planned.front().t0_s;
            }
            else if (!preview_cache.trajectory_frame_planned.empty())
            {
                frame_patch_t0_s = preview_cache.trajectory_frame_planned.front().t_s;
            }

            if (std::isfinite(frame_patch_t0_s))
            {
                preview_cache.trajectory_frame_planned = merge_planned_sample_prefix(
                        previous_cache.trajectory_frame_planned,
                        preview_cache.trajectory_frame_planned,
                        frame_patch_t0_s);
                preview_cache.trajectory_segments_frame_planned = merge_planned_segment_prefix(
                        previous_cache.trajectory_segments_frame_planned,
                        preview_cache.trajectory_segments_frame_planned,
                        frame_patch_t0_s);
            }

            preview_cache.maneuver_previews = merge_maneuver_previews(
                    previous_cache.maneuver_previews,
                    preview_cache.maneuver_previews);

            clear_preview_planned_render_artifacts(preview_cache);
            preview_cache.valid = preview_cache.trajectory_inertial.size() >= 2 &&
                                  preview_cache.trajectory_frame.size() >= 2 &&
                                  !preview_cache.trajectory_segments_frame.empty();
            return preview_cache;
        }

        PredictionChunkAssembly merge_planned_chunk_assembly(const PredictionChunkAssembly &previous_assembly,
                                                             PredictionChunkAssembly incoming_assembly)
        {
            if (!incoming_assembly.valid || incoming_assembly.chunks.empty())
            {
                return previous_assembly;
            }

            auto sort_and_validate = [](PredictionChunkAssembly &assembly) {
                std::sort(assembly.chunks.begin(),
                          assembly.chunks.end(),
                          [](const OrbitChunk &a, const OrbitChunk &b) {
                              if (a.t0_s == b.t0_s)
                              {
                                  return a.chunk_id < b.chunk_id;
                              }
                              return a.t0_s < b.t0_s;
                          });

                constexpr double kTimeEpsilonS = 1.0e-6;
                bool ordered = true;
                for (std::size_t i = 1; i < assembly.chunks.size(); ++i)
                {
                    const OrbitChunk &prev = assembly.chunks[i - 1];
                    const OrbitChunk &cur = assembly.chunks[i];
                    if (!(cur.t0_s + kTimeEpsilonS >= prev.t1_s))
                    {
                        ordered = false;
                        break;
                    }
                }
                assembly.valid = ordered && !assembly.chunks.empty();
            };

            if (!previous_assembly.valid ||
                previous_assembly.generation_id != incoming_assembly.generation_id ||
                !chunk_assembly_frame_version_matches(previous_assembly,
                                                     incoming_assembly.display_frame_key,
                                                     incoming_assembly.display_frame_revision))
            {
                sort_and_validate(incoming_assembly);
                return incoming_assembly;
            }

            constexpr double kTimeEpsilonS = 1.0e-6;
            PredictionChunkAssembly merged = previous_assembly;
            for (OrbitChunk &incoming_chunk : incoming_assembly.chunks)
            {
                merged.chunks.erase(
                        std::remove_if(
                                merged.chunks.begin(),
                                merged.chunks.end(),
                                [&incoming_chunk, kTimeEpsilonS](const OrbitChunk &existing_chunk) {
                                    if (existing_chunk.chunk_id == incoming_chunk.chunk_id)
                                    {
                                        return true;
                                    }
                                    const bool overlaps_in_time =
                                            incoming_chunk.t0_s < (existing_chunk.t1_s - kTimeEpsilonS) &&
                                            existing_chunk.t0_s < (incoming_chunk.t1_s - kTimeEpsilonS);
                                    return overlaps_in_time;
                                }),
                        merged.chunks.end());
                merged.chunks.push_back(std::move(incoming_chunk));
            }

            merged.generation_id = incoming_assembly.generation_id;
            merged.display_frame_key = incoming_assembly.display_frame_key;
            merged.display_frame_revision = incoming_assembly.display_frame_revision;
            sort_and_validate(merged);
            return merged;
        }

        uint64_t latest_visible_generation_id(const PredictionTrackState &track)
        {
            uint64_t latest_generation_id = track.cache.valid ? track.cache.generation_id : 0u;
            if (track.preview_overlay.cache.valid)
            {
                latest_generation_id = std::max(latest_generation_id, track.preview_overlay.cache.generation_id);
            }
            if (track.preview_overlay.chunk_assembly.valid)
            {
                latest_generation_id = std::max(latest_generation_id, track.preview_overlay.chunk_assembly.generation_id);
            }
            return latest_generation_id;
        }

        PredictionTrackState *find_track_by_id(std::vector<PredictionTrackState> &tracks, const uint64_t track_id)
        {
            for (PredictionTrackState &track : tracks)
            {
                if (track.key.track_id() == track_id)
                {
                    return &track;
                }
            }
            return nullptr;
        }

        std::pair<std::size_t, std::size_t> count_chunk_frame_data(const PredictionChunkAssembly &assembly)
        {
            std::pair<std::size_t, std::size_t> counts{0u, 0u};
            if (!assembly.valid)
            {
                return counts;
            }

            for (const OrbitChunk &chunk : assembly.chunks)
            {
                counts.first += chunk.frame_segments.size();
                counts.second += chunk.frame_samples.size();
            }
            return counts;
        }

        void record_derived_apply_debug(PredictionDragDebugTelemetry &debug,
                                        const uint64_t generation_id,
                                        const PredictionDragDebugTelemetry::TimePoint &derived_apply_start_tp,
                                        const PredictionDragDebugTelemetry::TimePoint &derived_apply_end_tp)
        {
            debug.last_derived_result_tp = derived_apply_end_tp;
            debug.last_derived_result_generation_id = generation_id;
            ++debug.derived_result_count;
            PredictionRuntimeDetail::update_last_and_peak(
                    debug.derived_apply_ms_last,
                    debug.derived_apply_ms_peak,
                    PredictionRuntimeDetail::elapsed_ms(derived_apply_start_tp, derived_apply_end_tp));
            if (debug.last_preview_request_generation_id == generation_id &&
                PredictionDragDebugTelemetry::has_time(debug.last_preview_request_tp))
            {
                PredictionRuntimeDetail::update_last_and_peak(
                        debug.request_to_derived_ms_last,
                        debug.request_to_derived_ms_peak,
                        PredictionRuntimeDetail::elapsed_ms(debug.last_preview_request_tp, derived_apply_end_tp));
            }
            if (debug.last_solver_result_generation_id == generation_id &&
                PredictionDragDebugTelemetry::has_time(debug.last_solver_result_tp))
            {
                PredictionRuntimeDetail::update_last_and_peak(
                        debug.solver_to_derived_ms_last,
                        debug.solver_to_derived_ms_peak,
                        PredictionRuntimeDetail::elapsed_ms(debug.last_solver_result_tp, derived_apply_end_tp));
            }
        }
    } // namespace

    void GameplayState::apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result)
    {
        const auto derived_apply_start_tp = PredictionDragDebugTelemetry::Clock::now();
        PredictionTrackState *track = find_track_by_id(_prediction_tracks, result.track_id);
        if (!track)
        {
            return;
        }

        if (result.generation_id < latest_visible_generation_id(*track))
        {
            return;
        }

        PredictionDragDebugTelemetry &debug = track->drag_debug;
        debug.last_result_solve_quality = result.solve_quality;
        debug.last_publish_stage = result.publish_stage;
        debug.last_generation_complete = result.generation_complete;
        PredictionRuntimeDetail::update_last_and_peak(
                debug.derived_worker_ms_last,
                debug.derived_worker_ms_peak,
                result.timings.total_ms);
        debug.derived_frame_build_ms_last = std::max(0.0, result.timings.frame_build_ms);
        debug.derived_flatten_ms_last = std::max(0.0, result.timings.flatten_ms);
        debug.flattened_planned_segments_last = result.cache.trajectory_segments_frame_planned.size();
        debug.flattened_planned_samples_last = result.cache.trajectory_frame_planned.size();
        if (debug.flattened_planned_segments_last == 0u &&
            debug.flattened_planned_samples_last == 0u &&
            result.chunk_assembly.valid)
        {
            const auto [chunk_segment_count, chunk_sample_count] = count_chunk_frame_data(result.chunk_assembly);
            debug.flattened_planned_segments_last = chunk_segment_count;
            debug.flattened_planned_samples_last = chunk_sample_count;
        }
        debug.incoming_chunk_count_last = result.chunk_assembly.valid
                                                  ? static_cast<uint32_t>(result.chunk_assembly.chunks.size())
                                                  : 0u;

        track->derived_request_pending = false;
        // Do NOT clear request_pending here — a newer solver request may already be in-flight.
        // Only the solver completion path and request submission paths manage that flag.
        const bool keep_dirty_for_followup = track->invalidated_while_pending;
        track->invalidated_while_pending = false;

        const bool preview_result = result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview;
        const bool preview_chunk_authoritative = preview_result && result.chunk_assembly.valid;

        OrbitPredictionCache cache_to_publish{};
        OrbitPredictionDerivedDiagnostics diagnostics_to_publish = result.diagnostics;
        bool have_cache_to_publish = false;
        double preview_merge_ms = 0.0;
        if (result.valid && result.cache.valid)
        {
            if (result.base_frame_reused)
            {
                const bool reusable_base_still_available =
                        track->cache.valid &&
                        track->cache.resolved_frame_spec_valid &&
                        result.cache.resolved_frame_spec_valid &&
                        frame_specs_match(track->cache.resolved_frame_spec, result.cache.resolved_frame_spec) &&
                        track->cache.shared_ephemeris == result.cache.shared_ephemeris &&
                        track->cache.trajectory_frame.size() >= 2 &&
                        !track->cache.trajectory_segments_frame.empty();
                if (reusable_base_still_available)
                {
                    cache_to_publish = merge_reused_base_frame_cache(track->cache, std::move(result.cache));
                    diagnostics_to_publish.frame_segment_count = cache_to_publish.trajectory_segments_frame.size();
                    diagnostics_to_publish.frame_sample_count = cache_to_publish.trajectory_frame.size();
                    diagnostics_to_publish.status = PredictionDerivedStatus::Success;
                    have_cache_to_publish = cache_to_publish.valid;
                }
            }
            else if (result.cache.trajectory_frame.size() >= 2)
            {
                cache_to_publish = std::move(result.cache);
                have_cache_to_publish = cache_to_publish.valid;
            }

            if (have_cache_to_publish &&
                preview_result &&
                !preview_chunk_authoritative)
            {
                const OrbitPredictionCache empty_preview_merge_base{};
                const bool can_merge_with_overlay =
                        track->preview_overlay.cache.valid &&
                        track->preview_overlay.cache.generation_id == result.generation_id &&
                        cache_frame_version_matches(track->preview_overlay.cache, cache_to_publish);
                const bool can_merge_with_stable =
                        track->cache.valid &&
                        cache_frame_version_matches(track->cache, cache_to_publish);
                const OrbitPredictionCache &preview_merge_base =
                        can_merge_with_overlay
                                ? track->preview_overlay.cache
                                : (can_merge_with_stable ? track->cache : empty_preview_merge_base);
                const auto preview_merge_start_tp = PredictionDragDebugTelemetry::Clock::now();
                cache_to_publish = merge_preview_planned_prefix_cache(preview_merge_base, std::move(cache_to_publish));
                preview_merge_ms = PredictionRuntimeDetail::elapsed_ms(
                        preview_merge_start_tp,
                        PredictionDragDebugTelemetry::Clock::now());
                have_cache_to_publish = cache_to_publish.valid;
            }
        }

        track->derived_diagnostics = diagnostics_to_publish;
        if (!have_cache_to_publish)
        {
            track->derived_diagnostics.status = PredictionDerivedStatus::MissingSolverData;
            track->dirty = true;
            const auto derived_apply_end_tp = PredictionDragDebugTelemetry::Clock::now();
            record_derived_apply_debug(debug, result.generation_id, derived_apply_start_tp, derived_apply_end_tp);
            return;
        }

        if (preview_result)
        {
            clear_preview_planned_render_artifacts(cache_to_publish);
            track->preview_overlay.cache = std::move(cache_to_publish);
        }
        else
        {
            track->cache = std::move(cache_to_publish);
            track->preview_overlay.clear();
            track->pick_cache.clear();
            track->preview_pick_cache.clear();
        }

        double chunk_merge_ms = 0.0;
        if (preview_result && result.chunk_assembly.valid)
        {
            const bool chunk_frame_changed =
                    track->preview_overlay.chunk_assembly.valid &&
                    !chunk_assembly_frame_version_matches(track->preview_overlay.chunk_assembly,
                                                          result.chunk_assembly.display_frame_key,
                                                          result.chunk_assembly.display_frame_revision);
            const auto chunk_merge_start_tp = PredictionDragDebugTelemetry::Clock::now();
            track->preview_overlay.chunk_assembly = merge_planned_chunk_assembly(
                    track->preview_overlay.chunk_assembly,
                    std::move(result.chunk_assembly));
            chunk_merge_ms = PredictionRuntimeDetail::elapsed_ms(
                    chunk_merge_start_tp,
                    PredictionDragDebugTelemetry::Clock::now());
            if (chunk_frame_changed)
            {
                track->preview_pick_cache.clear();
            }
        }
        else
        {
            track->preview_overlay.chunk_assembly.clear();
            track->preview_pick_cache.clear();
        }

        PredictionRuntimeDetail::update_last_and_peak(
                debug.preview_merge_ms_last,
                debug.preview_merge_ms_peak,
                preview_merge_ms);
        PredictionRuntimeDetail::update_last_and_peak(
                debug.chunk_merge_ms_last,
                debug.chunk_merge_ms_peak,
                chunk_merge_ms);
        const OrbitPredictionCache &debug_cache =
                (!track->preview_overlay.chunk_assembly.valid && track->preview_overlay.cache.valid)
                        ? track->preview_overlay.cache
                        : track->cache;
        debug.planned_segments_after_preview_merge = debug_cache.trajectory_segments_frame_planned.size();
        if (debug.planned_segments_after_preview_merge == 0u &&
            track->preview_overlay.chunk_assembly.valid)
        {
            const auto [chunk_segment_count, _] = count_chunk_frame_data(track->preview_overlay.chunk_assembly);
            debug.planned_segments_after_preview_merge = chunk_segment_count;
        }
        debug.merged_chunk_count_last = track->preview_overlay.chunk_assembly.valid
                                                ? static_cast<uint32_t>(track->preview_overlay.chunk_assembly.chunks.size())
                                                : 0u;

        if (preview_result)
        {
            track->preview_state = PredictionPreviewRuntimeState::PreviewStreaming;
        }
        else if (track->preview_state != PredictionPreviewRuntimeState::Idle)
        {
            track->preview_state = PredictionPreviewRuntimeState::Idle;
            track->preview_anchor.clear();
            track->preview_entered_at_s = std::numeric_limits<double>::quiet_NaN();
        }

        const bool maneuver_preview_subject =
                prediction_subject_supports_maneuvers(track->key) &&
                _maneuver_nodes_enabled &&
                !_maneuver_state.nodes.empty();
        const bool interaction_idle =
                !PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state);
        const bool schedule_full_refine =
                preview_result &&
                result.generation_complete &&
                maneuver_preview_subject &&
                _maneuver_plan_live_preview_active &&
                interaction_idle &&
                !keep_dirty_for_followup;
        track->dirty = keep_dirty_for_followup || schedule_full_refine;

        const bool freeze_maneuver_plan =
                maneuver_preview_subject &&
                _maneuver_plan_live_preview_active &&
                interaction_idle &&
                !preview_result &&
                !keep_dirty_for_followup;
        if (schedule_full_refine)
        {
            // Publish a cheap preview first, then immediately fall through to a full rebuild.
            track->preview_state = PredictionPreviewRuntimeState::AwaitFullRefine;
            _maneuver_plan_live_preview_active = false;
        }
        if (freeze_maneuver_plan)
        {
            track->preview_state = PredictionPreviewRuntimeState::Idle;
            track->preview_anchor.clear();
            track->preview_entered_at_s = std::numeric_limits<double>::quiet_NaN();
            _maneuver_plan_live_preview_active = false;
        }

        const auto derived_apply_end_tp = PredictionDragDebugTelemetry::Clock::now();
        record_derived_apply_debug(debug, result.generation_id, derived_apply_start_tp, derived_apply_end_tp);
        if (preview_result)
        {
            debug.last_preview_publish_tp = derived_apply_end_tp;
            ++debug.preview_publish_count;
        }
        else
        {
            debug.last_full_publish_tp = derived_apply_end_tp;
            ++debug.full_publish_count;
        }
    }
} // namespace Game
