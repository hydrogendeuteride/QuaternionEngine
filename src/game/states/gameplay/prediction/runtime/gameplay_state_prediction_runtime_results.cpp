#include "game/states/gameplay/gameplay_state.h"

#include "core/util/logger.h"
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
            return cache.display.display_frame_key == display_frame_key &&
                   cache.display.display_frame_revision == display_frame_revision;
        }

        bool cache_frame_version_matches(const OrbitPredictionCache &a,
                                         const OrbitPredictionCache &b)
        {
            return cache_frame_version_matches(a, b.display.display_frame_key, b.display.display_frame_revision);
        }

        OrbitPredictionCache merge_reused_base_frame_cache(const OrbitPredictionCache &base_cache,
                                                           OrbitPredictionCache reused_cache)
        {
            reused_cache.display.trajectory_frame = base_cache.display.trajectory_frame;
            reused_cache.display.trajectory_segments_frame = base_cache.display.trajectory_segments_frame;
            reused_cache.display.render_curve_frame = base_cache.display.render_curve_frame;
            if (base_cache.analysis.analysis_cache_valid &&
                base_cache.analysis.analysis_cache_body_id != orbitsim::kInvalidBodyId &&
                reused_cache.analysis.analysis_cache_body_id == base_cache.analysis.analysis_cache_body_id)
            {
                reused_cache.analysis.trajectory_analysis_bci = base_cache.analysis.trajectory_analysis_bci;
                reused_cache.analysis.trajectory_segments_analysis_bci =
                        base_cache.analysis.trajectory_segments_analysis_bci;
                reused_cache.analysis.analysis_cache_body_id = base_cache.analysis.analysis_cache_body_id;
                reused_cache.analysis.analysis_cache_valid = base_cache.analysis.analysis_cache_valid;
            }
            if (base_cache.analysis.metrics_valid &&
                base_cache.analysis.metrics_body_id != orbitsim::kInvalidBodyId &&
                reused_cache.analysis.metrics_body_id == base_cache.analysis.metrics_body_id)
            {
                reused_cache.analysis.altitude_km = base_cache.analysis.altitude_km;
                reused_cache.analysis.speed_kmps = base_cache.analysis.speed_kmps;
                reused_cache.analysis.semi_major_axis_m = base_cache.analysis.semi_major_axis_m;
                reused_cache.analysis.eccentricity = base_cache.analysis.eccentricity;
                reused_cache.analysis.orbital_period_s = base_cache.analysis.orbital_period_s;
                reused_cache.analysis.periapsis_alt_km = base_cache.analysis.periapsis_alt_km;
                reused_cache.analysis.apoapsis_alt_km = base_cache.analysis.apoapsis_alt_km;
                reused_cache.analysis.metrics_body_id = base_cache.analysis.metrics_body_id;
                reused_cache.analysis.metrics_valid = base_cache.analysis.metrics_valid;
            }
            reused_cache.identity.valid = reused_cache.solver.resolved_trajectory_inertial().size() >= 2 &&
                                          reused_cache.display.trajectory_frame.size() >= 2 &&
                                          !reused_cache.display.trajectory_segments_frame.empty();
            return reused_cache;
        }

        uint64_t latest_visible_generation_id(const PredictionTrackState &track)
        {
            return PredictionRuntimeDetail::visible_generation_id(track);
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
            if (debug.last_request_generation_id == generation_id &&
                PredictionDragDebugTelemetry::has_time(debug.last_request_tp))
            {
                PredictionRuntimeDetail::update_last_and_peak(
                        debug.request_to_derived_ms_last,
                        debug.request_to_derived_ms_peak,
                        PredictionRuntimeDetail::elapsed_ms(debug.last_request_tp, derived_apply_end_tp));
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

        void merge_chunk_assembly(PredictionChunkAssembly &dst, const PredictionChunkAssembly &src)
        {
            if (!src.valid)
            {
                return;
            }

            if (!dst.valid || dst.generation_id != src.generation_id)
            {
                dst = src;
                return;
            }

            for (const OrbitChunk &chunk : src.chunks)
            {
                auto existing = std::find_if(dst.chunks.begin(),
                                             dst.chunks.end(),
                                             [&chunk](const OrbitChunk &candidate) {
                                                 return candidate.chunk_id == chunk.chunk_id;
                                             });
                if (existing != dst.chunks.end())
                {
                    *existing = chunk;
                    continue;
                }
                dst.chunks.push_back(chunk);
            }

            std::sort(dst.chunks.begin(),
                      dst.chunks.end(),
                      [](const OrbitChunk &a, const OrbitChunk &b) { return a.chunk_id < b.chunk_id; });
            dst.valid = !dst.chunks.empty();
        }

        void merge_frame_bound_chunk_overlay(PredictionFrameBoundChunkOverlay &dst,
                                             const PredictionChunkAssembly &src,
                                             const uint64_t generation_id,
                                             const uint64_t display_frame_key,
                                             const uint64_t display_frame_revision)
        {
            if (!dst.matches_generation(generation_id, display_frame_key, display_frame_revision))
            {
                dst.reset_for_generation(generation_id, display_frame_key, display_frame_revision);
            }
            merge_chunk_assembly(dst.chunk_assembly, src);
        }

        bool cache_has_planned_data(const OrbitPredictionCache &cache)
        {
            return !cache.solver.trajectory_inertial_planned.empty() ||
                   !cache.solver.trajectory_segments_inertial_planned.empty() ||
                   !cache.display.trajectory_frame_planned.empty() ||
                   !cache.display.trajectory_segments_frame_planned.empty() ||
                   !cache.solver.maneuver_previews.empty();
        }

        bool planned_cache_matches_current_plan(const OrbitPredictionCache &cache,
                                                const uint64_t current_plan_signature)
        {
            return cache_has_planned_data(cache) &&
                   current_plan_signature != 0u &&
                   cache.identity.maneuver_plan_signature_valid &&
                   cache.identity.maneuver_plan_signature == current_plan_signature;
        }

        bool full_stream_result_is_obsolete_after_final_publish(
                const PredictionTrackState &track,
                const OrbitPredictionDerivedService::Result &result)
        {
            return result.solve_quality == OrbitPredictionService::SolveQuality::Full &&
                   result.publish_stage == OrbitPredictionService::PublishStage::FullStreaming &&
                   track.authoritative_cache.identity.valid &&
                   track.authoritative_cache.identity.generation_id >= result.generation_id;
        }

        void restore_authoritative_planned_data(const PredictionTrackState &track,
                                                OrbitPredictionCache &cache,
                                                const uint64_t current_plan_signature)
        {
            if (track.authoritative_cache.identity.valid &&
                planned_cache_matches_current_plan(track.authoritative_cache, current_plan_signature))
            {
                copy_prediction_cache_planned_data(cache, track.authoritative_cache);
                return;
            }

            if (track.cache.identity.valid &&
                planned_cache_matches_current_plan(track.cache, current_plan_signature))
            {
                copy_prediction_cache_planned_data(cache, track.cache);
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

        const bool live_fast_preview_result =
                track->supports_maneuvers &&
                result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
                maneuver_live_preview_active(true);
        if (track->latest_requested_generation_id != 0 &&
            result.generation_id < track->latest_requested_generation_id &&
            !live_fast_preview_result)
        {
            return;
        }

        if (track->latest_requested_derived_generation_id != 0 &&
            (result.generation_id != track->latest_requested_derived_generation_id ||
             result.display_frame_key != track->latest_requested_derived_display_frame_key ||
             result.display_frame_revision != track->latest_requested_derived_display_frame_revision ||
             result.analysis_body_id != track->latest_requested_derived_analysis_body_id) &&
            !live_fast_preview_result)
        {
            return;
        }
        if (full_stream_result_is_obsolete_after_final_publish(*track, result))
        {
            return;
        }

        if (track->supports_maneuvers && result.maneuver_plan_revision != _maneuver_plan_revision)
        {
            Logger::warn("Dropping stale maneuver derived result: track={} gen={} result_plan_rev={} current_plan_rev={} "
                         "latest_derived_gen={} frame_key={} frame_rev={} analysis_body={} "
                         "request_pending={} derived_pending={} invalidated={} dirty={}",
                         result.track_id,
                         result.generation_id,
                         result.maneuver_plan_revision,
                         _maneuver_plan_revision,
                         track->latest_requested_derived_generation_id,
                         result.display_frame_key,
                         result.display_frame_revision,
                         static_cast<uint32_t>(result.analysis_body_id),
                         track->request_pending,
                         track->derived_request_pending,
                         track->invalidated_while_pending,
                         track->dirty);
            if (result.generation_id == track->latest_requested_derived_generation_id &&
                result.display_frame_key == track->latest_requested_derived_display_frame_key &&
                result.display_frame_revision == track->latest_requested_derived_display_frame_revision &&
                result.analysis_body_id == track->latest_requested_derived_analysis_body_id)
            {
                track->derived_request_pending = false;
                track->invalidated_while_pending = false;
                track->dirty = true;
            }
            return;
        }

        const bool completes_latest_derived_request =
                track->latest_requested_derived_generation_id != 0 &&
                result.generation_id == track->latest_requested_derived_generation_id &&
                result.display_frame_key == track->latest_requested_derived_display_frame_key &&
                result.display_frame_revision == track->latest_requested_derived_display_frame_revision &&
                result.analysis_body_id == track->latest_requested_derived_analysis_body_id &&
                result.publish_stage == track->latest_requested_derived_publish_stage;
        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle_before_apply =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(*track);

        PredictionDragDebugTelemetry &debug = track->drag_debug;
        debug.last_result_solve_quality = result.solve_quality;
        PredictionRuntimeDetail::update_last_and_peak(
                debug.derived_worker_ms_last,
                debug.derived_worker_ms_peak,
                result.timings.total_ms);
        debug.derived_frame_build_ms_last = std::max(0.0, result.timings.frame_build_ms);
        debug.derived_flatten_ms_last = std::max(0.0, result.timings.flatten_ms);
        debug.flattened_planned_segments_last = result.cache.display.trajectory_segments_frame_planned.size();
        debug.flattened_planned_samples_last = result.cache.display.trajectory_frame_planned.size();

        if (completes_latest_derived_request)
        {
            track->derived_request_pending = false;
        }
        // Do NOT clear request_pending here — a newer solver request may already be in-flight.
        // Only the solver completion path and request submission paths manage that flag.
        // Preserve an already-queued refine request when a late preview-derived result arrives after drag end.
        const bool keep_dirty_for_followup =
                PredictionRuntimeDetail::prediction_track_should_keep_dirty_for_followup(lifecycle_before_apply);
        track->invalidated_while_pending = false;

        OrbitPredictionCache cache_to_publish{};
        OrbitPredictionDerivedDiagnostics diagnostics_to_publish = result.diagnostics;
        bool have_cache_to_publish = false;
        if (result.valid && result.cache.identity.valid)
        {
            if (result.base_frame_reused)
            {
                const bool reusable_base_still_available =
                        track->cache.identity.valid &&
                        track->cache.display.resolved_frame_spec_valid &&
                        result.cache.display.resolved_frame_spec_valid &&
                        frame_specs_match(track->cache.display.resolved_frame_spec,
                                          result.cache.display.resolved_frame_spec) &&
                        track->cache.solver.resolved_shared_ephemeris() ==
                                result.cache.solver.resolved_shared_ephemeris() &&
                        track->cache.display.trajectory_frame.size() >= 2 &&
                        !track->cache.display.trajectory_segments_frame.empty();
                if (reusable_base_still_available)
                {
                    cache_to_publish = merge_reused_base_frame_cache(track->cache, std::move(result.cache));
                    diagnostics_to_publish.frame_segment_count =
                            cache_to_publish.display.trajectory_segments_frame.size();
                    diagnostics_to_publish.frame_sample_count = cache_to_publish.display.trajectory_frame.size();
                    diagnostics_to_publish.status = PredictionDerivedStatus::Success;
                    have_cache_to_publish = cache_to_publish.identity.valid;
                }
            }
            else if (result.cache.display.trajectory_frame.size() >= 2)
            {
                cache_to_publish = std::move(result.cache);
                have_cache_to_publish = cache_to_publish.identity.valid;
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

        const bool live_preview_active_now =
                track->supports_maneuvers &&
                maneuver_live_preview_active(true);
        const bool current_plan_active = track->supports_maneuvers &&
                                         _maneuver_nodes_enabled &&
                                         !_maneuver_state.nodes.empty();
        const uint64_t current_plan_signature =
                current_plan_active ? current_maneuver_plan_signature() : 0u;
        const bool solved_plan_signature_valid =
                result.maneuver_plan_signature_valid ||
                cache_to_publish.identity.maneuver_plan_signature_valid;
        const uint64_t solved_plan_signature =
                result.maneuver_plan_signature_valid
                        ? result.maneuver_plan_signature
                        : cache_to_publish.identity.maneuver_plan_signature;
        const bool result_plan_signature_mismatch =
                current_plan_active &&
                solved_plan_signature_valid &&
                solved_plan_signature != current_plan_signature;
        const bool active_maneuver_edit =
                current_plan_active &&
                (PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state) ||
                 _maneuver_node_edit_preview.state != ManeuverNodeEditPreview::State::Idle) &&
                track->key == _prediction_selection.active_subject &&
                maneuver_live_preview_active(true);
        const bool stale_fast_preview_during_edit =
                active_maneuver_edit &&
                result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
                result_plan_signature_mismatch;
        if (active_maneuver_edit && result_plan_signature_mismatch && !stale_fast_preview_during_edit)
        {
            track->dirty = true;

            const auto derived_apply_end_tp = PredictionDragDebugTelemetry::Clock::now();
            record_derived_apply_debug(debug, result.generation_id, derived_apply_start_tp, derived_apply_end_tp);
            return;
        }

        const bool full_streaming_result =
                PredictionRuntimeDetail::prediction_track_is_full_streaming_publish(result.solve_quality,
                                                                                    result.publish_stage);
        if (cache_has_planned_data(cache_to_publish))
        {
            const bool solved_plan_matches_current =
                    !current_plan_active ||
                    !solved_plan_signature_valid ||
                    solved_plan_signature == current_plan_signature;
            if (solved_plan_signature_valid && (solved_plan_matches_current || stale_fast_preview_during_edit))
            {
                cache_to_publish.identity.maneuver_plan_signature_valid = true;
                cache_to_publish.identity.maneuver_plan_signature = solved_plan_signature;
            }
            else if (!solved_plan_matches_current)
            {
                clear_prediction_cache_planned_data(cache_to_publish);
            }
        }

        if (result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview)
        {
            const bool accept_preview_publish =
                    !track->supports_maneuvers ||
                    PredictionRuntimeDetail::prediction_track_should_accept_preview_publish(
                            lifecycle_before_apply,
                            live_preview_active_now,
                            track->preview_anchor.valid);
            if (!accept_preview_publish)
            {
                track->preview_overlay.clear();
                track->pick_cache.clear();
                if (track->preview_state != PredictionPreviewRuntimeState::AwaitFullRefine)
                {
                    track->preview_state = PredictionPreviewRuntimeState::Idle;
                    track->preview_anchor = {};
                }
                track->dirty = keep_dirty_for_followup;

                const auto derived_apply_end_tp = PredictionDragDebugTelemetry::Clock::now();
                record_derived_apply_debug(debug, result.generation_id, derived_apply_start_tp, derived_apply_end_tp);
                return;
            }

            if (!track->full_stream_overlay.matches_generation(result.generation_id,
                                                               result.display_frame_key,
                                                               result.display_frame_revision))
            {
                track->full_stream_overlay.clear();
            }
            if (!track->authoritative_cache.identity.valid &&
                track->cache.identity.valid &&
                cache_has_planned_data(track->cache))
            {
                track->authoritative_cache = track->cache;
            }

            merge_chunk_assembly(track->preview_overlay.chunk_assembly, result.chunk_assembly);
            restore_authoritative_planned_data(*track, cache_to_publish, current_plan_signature);

            track->preview_state = PredictionRuntimeDetail::prediction_track_preview_state_after_preview_publish(
                    lifecycle_before_apply,
                    result.publish_stage,
                    live_preview_active_now);

            track->cache = std::move(cache_to_publish);
            track->pick_cache.clear();
        }
        else if (full_streaming_result)
        {
            if (result_plan_signature_mismatch)
            {
                track->full_stream_overlay.clear();
                track->pick_cache.clear();
                track->dirty = true;

                const auto derived_apply_end_tp = PredictionDragDebugTelemetry::Clock::now();
                record_derived_apply_debug(debug, result.generation_id, derived_apply_start_tp, derived_apply_end_tp);
                return;
            }

            const bool already_seeded_for_generation =
                    track->full_stream_overlay.matches_generation(result.generation_id,
                                                                  result.display_frame_key,
                                                                  result.display_frame_revision);
            if (!already_seeded_for_generation)
            {
                restore_authoritative_planned_data(*track, cache_to_publish, current_plan_signature);
                track->cache = std::move(cache_to_publish);
                track->pick_cache.clear();
            }

            merge_frame_bound_chunk_overlay(track->full_stream_overlay,
                                            result.chunk_assembly,
                                            result.generation_id,
                                            result.display_frame_key,
                                            result.display_frame_revision);
            track->pick_cache.clear();
        }
        else
        {
            track->authoritative_cache = cache_to_publish;
            track->preview_overlay.clear();
            track->full_stream_overlay.clear();
            track->preview_state = PredictionPreviewRuntimeState::Idle;
            if (PredictionRuntimeDetail::prediction_track_should_clear_preview_anchor_after_final_publish(
                        live_preview_active_now))
            {
                track->preview_anchor = {};
            }
            track->cache = std::move(cache_to_publish);
            track->pick_cache.clear();
        }

        track->dirty = keep_dirty_for_followup;

        const auto derived_apply_end_tp = PredictionDragDebugTelemetry::Clock::now();
        record_derived_apply_debug(debug, result.generation_id, derived_apply_start_tp, derived_apply_end_tp);
        debug.last_publish_tp = derived_apply_end_tp;
        ++debug.publish_count;
    }
} // namespace Game
