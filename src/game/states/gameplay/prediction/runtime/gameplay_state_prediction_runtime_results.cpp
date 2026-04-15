#include "game/states/gameplay/gameplay_state.h"

#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"
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

        OrbitPredictionCache merge_reused_base_frame_cache(const OrbitPredictionCache &base_cache,
                                                           OrbitPredictionCache reused_cache)
        {
            reused_cache.trajectory_frame = base_cache.trajectory_frame;
            reused_cache.trajectory_segments_frame = base_cache.trajectory_segments_frame;
            reused_cache.render_curve_frame = base_cache.render_curve_frame;
            if (base_cache.analysis_cache_valid &&
                base_cache.analysis_cache_body_id != orbitsim::kInvalidBodyId &&
                reused_cache.analysis_cache_body_id == base_cache.analysis_cache_body_id)
            {
                reused_cache.trajectory_analysis_bci = base_cache.trajectory_analysis_bci;
                reused_cache.trajectory_segments_analysis_bci = base_cache.trajectory_segments_analysis_bci;
                reused_cache.analysis_cache_body_id = base_cache.analysis_cache_body_id;
                reused_cache.analysis_cache_valid = base_cache.analysis_cache_valid;
            }
            if (base_cache.metrics_valid &&
                base_cache.metrics_body_id != orbitsim::kInvalidBodyId &&
                reused_cache.metrics_body_id == base_cache.metrics_body_id)
            {
                reused_cache.altitude_km = base_cache.altitude_km;
                reused_cache.speed_kmps = base_cache.speed_kmps;
                reused_cache.semi_major_axis_m = base_cache.semi_major_axis_m;
                reused_cache.eccentricity = base_cache.eccentricity;
                reused_cache.orbital_period_s = base_cache.orbital_period_s;
                reused_cache.periapsis_alt_km = base_cache.periapsis_alt_km;
                reused_cache.apoapsis_alt_km = base_cache.apoapsis_alt_km;
                reused_cache.metrics_body_id = base_cache.metrics_body_id;
                reused_cache.metrics_valid = base_cache.metrics_valid;
            }
            reused_cache.valid = reused_cache.trajectory_inertial.size() >= 2 &&
                                 reused_cache.trajectory_frame.size() >= 2 &&
                                 !reused_cache.trajectory_segments_frame.empty();
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

        void merge_preview_chunk_assembly(PredictionChunkAssembly &dst, const PredictionChunkAssembly &src)
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

        if (track->latest_requested_generation_id != 0 &&
            result.generation_id < track->latest_requested_generation_id)
        {
            return;
        }

        if (track->latest_requested_derived_generation_id != 0 &&
            (result.generation_id != track->latest_requested_derived_generation_id ||
             result.display_frame_key != track->latest_requested_derived_display_frame_key ||
             result.display_frame_revision != track->latest_requested_derived_display_frame_revision ||
             result.analysis_body_id != track->latest_requested_derived_analysis_body_id))
        {
            return;
        }

        PredictionDragDebugTelemetry &debug = track->drag_debug;
        debug.last_result_solve_quality = result.solve_quality;
        PredictionRuntimeDetail::update_last_and_peak(
                debug.derived_worker_ms_last,
                debug.derived_worker_ms_peak,
                result.timings.total_ms);
        debug.derived_frame_build_ms_last = std::max(0.0, result.timings.frame_build_ms);
        debug.derived_flatten_ms_last = std::max(0.0, result.timings.flatten_ms);
        debug.flattened_planned_segments_last = result.cache.trajectory_segments_frame_planned.size();
        debug.flattened_planned_samples_last = result.cache.trajectory_frame_planned.size();

        track->derived_request_pending = false;
        // Do NOT clear request_pending here — a newer solver request may already be in-flight.
        // Only the solver completion path and request submission paths manage that flag.
        // Preserve an already-queued refine request when a late preview-derived result arrives after drag end.
        const bool keep_dirty_for_followup = track->dirty || track->invalidated_while_pending;
        track->invalidated_while_pending = false;

        OrbitPredictionCache cache_to_publish{};
        OrbitPredictionDerivedDiagnostics diagnostics_to_publish = result.diagnostics;
        bool have_cache_to_publish = false;
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

        const bool drag_preview_active_now =
                track->supports_maneuvers &&
                _maneuver_plan_live_preview_active &&
                PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state);

        if (result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview)
        {
            merge_preview_chunk_assembly(track->preview_overlay.chunk_assembly, result.chunk_assembly);
            if (track->preview_overlay.chunk_assembly.valid)
            {
                PredictionCacheInternal::flatten_chunk_assembly_to_cache(
                        cache_to_publish,
                        track->preview_overlay.chunk_assembly,
                        false);
            }

            if (result.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming)
            {
                track->preview_state = PredictionPreviewRuntimeState::PreviewStreaming;
            }
            else if (track->preview_state != PredictionPreviewRuntimeState::AwaitFullRefine)
            {
                track->preview_state =
                        drag_preview_active_now
                                ? PredictionPreviewRuntimeState::PreviewStreaming
                                : PredictionPreviewRuntimeState::Idle;
            }
        }
        else
        {
            track->preview_overlay.clear();
            track->preview_state = PredictionPreviewRuntimeState::Idle;
            if (!drag_preview_active_now)
            {
                track->preview_anchor = {};
            }
        }

        track->cache = std::move(cache_to_publish);
        track->pick_cache.clear();
        track->dirty = keep_dirty_for_followup;

        const auto derived_apply_end_tp = PredictionDragDebugTelemetry::Clock::now();
        record_derived_apply_debug(debug, result.generation_id, derived_apply_start_tp, derived_apply_end_tp);
        debug.last_publish_tp = derived_apply_end_tp;
        ++debug.publish_count;
    }
} // namespace Game
