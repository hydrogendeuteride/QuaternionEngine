#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace Game
{
    using detail::finite_vec3;

    namespace
    {
        template<typename T>
        bool contains_key(const std::vector<T> &items, const T &needle)
        {
            return std::find(items.begin(), items.end(), needle) != items.end();
        }

        OrbitPredictionService::RequestPriority classify_prediction_request_priority(
                const PredictionSelectionState &selection,
                const PredictionSubjectKey key,
                const bool is_celestial,
                const bool interactive)
        {
            if (selection.active_subject == key)
            {
                return interactive
                               ? OrbitPredictionService::RequestPriority::ActiveInteractiveTrack
                               : OrbitPredictionService::RequestPriority::ActiveTrack;
            }

            for (const auto &overlay : selection.overlay_subjects)
            {
                if (overlay == key)
                {
                    return OrbitPredictionService::RequestPriority::Overlay;
                }
            }

            return is_celestial
                           ? OrbitPredictionService::RequestPriority::BackgroundCelestial
                           : OrbitPredictionService::RequestPriority::BackgroundOrbiter;
        }

        const ManeuverNode *select_preview_anchor_node(const ManeuverPlanState &plan, const double now_s)
        {
            if (const ManeuverNode *selected = plan.find_node(plan.selected_node_id))
            {
                if (std::isfinite(selected->time_s))
                {
                    return selected;
                }
            }

            const ManeuverNode *first_future = nullptr;
            for (const ManeuverNode &node : plan.nodes)
            {
                if (!std::isfinite(node.time_s) || node.time_s < now_s)
                {
                    continue;
                }

                if (!first_future || node.time_s < first_future->time_s)
                {
                    first_future = &node;
                }
            }
            return first_future;
        }

        template<typename T>
        void hash_combine(uint64_t &seed, const T &value)
        {
            seed ^= static_cast<uint64_t>(std::hash<T>{}(value)) + 0x9e3779b97f4a7c15ULL + (seed << 6u) + (seed >> 2u);
        }

        uint64_t hash_upstream_maneuvers(const ManeuverPlanState &plan,
                                         const int anchor_node_id,
                                         const double anchor_time_s)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            for (const ManeuverNode &node : plan.nodes)
            {
                if (!std::isfinite(node.time_s) ||
                    node.id == anchor_node_id ||
                    node.time_s > anchor_time_s)
                {
                    continue;
                }

                hash_combine(seed, node.id);
                hash_combine(seed, node.time_s);
                hash_combine(seed, node.primary_body_id);
                hash_combine(seed, node.primary_body_auto);
                hash_combine(seed, node.dv_rtn_mps.x);
                hash_combine(seed, node.dv_rtn_mps.y);
                hash_combine(seed, node.dv_rtn_mps.z);
            }
            return seed;
        }

        glm::dmat3 snapshot_gizmo_basis(const ManeuverGizmoInteraction &interaction)
        {
            if (interaction.state == ManeuverGizmoInteraction::State::DragAxis)
            {
                return glm::dmat3(interaction.drag_basis_r_world,
                                  interaction.drag_basis_t_world,
                                  interaction.drag_basis_n_world);
            }
            return glm::dmat3(1.0);
        }

        bool frame_specs_match(const orbitsim::TrajectoryFrameSpec &a,
                               const orbitsim::TrajectoryFrameSpec &b)
        {
            return a.type == b.type &&
                   a.primary_body_id == b.primary_body_id &&
                   a.secondary_body_id == b.secondary_body_id &&
                   a.target_spacecraft_id == b.target_spacecraft_id;
        }

        bool frame_supports_live_base_frame_reuse(const orbitsim::TrajectoryFrameSpec &spec)
        {
            return spec.type != orbitsim::TrajectoryFrameType::Inertial &&
                   spec.type != orbitsim::TrajectoryFrameType::LVLH;
        }

        bool base_trajectory_signature_matches(const OrbitPredictionCache &cache,
                                              const OrbitPredictionService::Result &result)
        {
            if (cache.trajectory_segments_inertial.empty() || result.trajectory_segments_inertial.empty() ||
                cache.trajectory_inertial.size() < 2 || result.trajectory_inertial.size() < 2)
            {
                return false;
            }

            if (cache.trajectory_segments_inertial.size() != result.trajectory_segments_inertial.size() ||
                cache.trajectory_inertial.size() != result.trajectory_inertial.size())
            {
                return false;
            }

            const orbitsim::TrajectorySegment &cache_seg0 = cache.trajectory_segments_inertial.front();
            const orbitsim::TrajectorySegment &cache_seg1 = cache.trajectory_segments_inertial.back();
            const orbitsim::TrajectorySegment &result_seg0 = result.trajectory_segments_inertial.front();
            const orbitsim::TrajectorySegment &result_seg1 = result.trajectory_segments_inertial.back();
            if (cache_seg0.t0_s != result_seg0.t0_s ||
                cache_seg0.dt_s != result_seg0.dt_s ||
                cache_seg1.t0_s != result_seg1.t0_s ||
                cache_seg1.dt_s != result_seg1.dt_s)
            {
                return false;
            }

            const orbitsim::TrajectorySample &cache_sample0 = cache.trajectory_inertial.front();
            const orbitsim::TrajectorySample &cache_sample1 = cache.trajectory_inertial.back();
            const orbitsim::TrajectorySample &result_sample0 = result.trajectory_inertial.front();
            const orbitsim::TrajectorySample &result_sample1 = result.trajectory_inertial.back();
            return cache_sample0.t_s == result_sample0.t_s &&
                   cache_sample1.t_s == result_sample1.t_s;
        }

        bool can_reuse_existing_base_frame_cache(const PredictionTrackState &track,
                                                 const OrbitPredictionService::Result &result,
                                                 const orbitsim::TrajectoryFrameSpec &resolved_frame_spec)
        {
            return result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
                   result.baseline_reused &&
                   track.cache.valid &&
                   track.cache.resolved_frame_spec_valid &&
                   frame_supports_live_base_frame_reuse(resolved_frame_spec) &&
                   frame_specs_match(track.cache.resolved_frame_spec, resolved_frame_spec) &&
                   track.cache.shared_ephemeris == result.shared_ephemeris &&
                   track.cache.trajectory_frame.size() >= 2 &&
                   !track.cache.trajectory_segments_frame.empty() &&
                   base_trajectory_signature_matches(track.cache, result);
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

            preview_cache.gpu_roots_frame_planned.reset();
            preview_cache.render_curve_frame_planned = preview_cache.trajectory_segments_frame_planned.empty()
                                                              ? OrbitRenderCurve{}
                                                              : OrbitRenderCurve::build(
                                                                        preview_cache.trajectory_segments_frame_planned);
            preview_cache.valid = preview_cache.trajectory_inertial.size() >= 2 &&
                                  preview_cache.trajectory_frame.size() >= 2 &&
                                  !preview_cache.trajectory_segments_frame.empty();
            return preview_cache;
        }

        bool preview_anchor_matches(const PreviewAnchorCache &a, const PreviewAnchorCache &b)
        {
            return a.valid == b.valid &&
                   a.anchor_node_id == b.anchor_node_id &&
                   a.anchor_time_s == b.anchor_time_s &&
                   a.baseline_generation_id == b.baseline_generation_id &&
                   a.upstream_maneuver_hash == b.upstream_maneuver_hash &&
                   a.display_frame_snapshot.type == b.display_frame_snapshot.type &&
                   a.display_frame_snapshot.primary_body_id == b.display_frame_snapshot.primary_body_id &&
                   a.display_frame_snapshot.secondary_body_id == b.display_frame_snapshot.secondary_body_id &&
                   a.display_frame_snapshot.target_spacecraft_id == b.display_frame_snapshot.target_spacecraft_id &&
                   a.patch_window_s == b.patch_window_s &&
                   a.request_window_s == b.request_window_s &&
                   a.downstream_maneuver_node_ids == b.downstream_maneuver_node_ids;
        }
    } // namespace

    void GameplayState::sync_prediction_dirty_flag()
    {
        // Collapse only visible-track rebuild demand into one cheap UI-facing flag.
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        _prediction_dirty = false;
        for (const PredictionTrackState &track : _prediction_tracks)
        {
            if (!contains_key(visible_subjects, track.key))
            {
                continue;
            }

            if (track.dirty)
            {
                _prediction_dirty = true;
                return;
            }
        }
    }

    void GameplayState::mark_prediction_dirty()
    {
        // Force only the active/overlay-visible tracks to rebuild on the next prediction update.
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        for (PredictionTrackState &track : _prediction_tracks)
        {
            if (!contains_key(visible_subjects, track.key))
            {
                continue;
            }

            if (track.request_pending)
            {
                track.invalidated_while_pending = true;
                continue;
            }

            track.dirty = true;
        }
        sync_prediction_dirty_flag();
    }

    void GameplayState::mark_maneuver_plan_dirty()
    {
        // Maneuver edits should live-update until the latest authored plan finishes solving once.
        _maneuver_plan_live_preview_active = true;
        mark_prediction_dirty();
    }

    void GameplayState::refresh_prediction_preview_anchor(PredictionTrackState &track,
                                                          const double now_s,
                                                          const bool with_maneuvers)
    {
        const bool preview_track =
                with_maneuvers &&
                track.key == _prediction_selection.active_subject;
        const bool preview_live =
                preview_track &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);

        if (!preview_track)
        {
            track.preview_state = PredictionPreviewRuntimeState::Idle;
            track.preview_anchor.clear();
            return;
        }

        if (!preview_live)
        {
            if (track.preview_state == PredictionPreviewRuntimeState::EnterDrag ||
                track.preview_state == PredictionPreviewRuntimeState::DragPreviewPending ||
                track.preview_state == PredictionPreviewRuntimeState::PreviewStreaming)
            {
                track.preview_state = PredictionPreviewRuntimeState::AwaitFullRefine;
            }
            return;
        }

        const ManeuverNode *anchor_node = select_preview_anchor_node(_maneuver_state, now_s);
        if (!anchor_node || !std::isfinite(anchor_node->time_s))
        {
            track.preview_state = PredictionPreviewRuntimeState::Idle;
            track.preview_anchor.clear();
            return;
        }

        PreviewAnchorCache refreshed{};
        refreshed.valid = true;
        refreshed.anchor_node_id = anchor_node->id;
        refreshed.anchor_time_s = std::max(now_s, anchor_node->time_s);
        refreshed.baseline_generation_id = track.cache.generation_id;
        refreshed.upstream_maneuver_hash = hash_upstream_maneuvers(_maneuver_state,
                                                                   refreshed.anchor_node_id,
                                                                   refreshed.anchor_time_s);
        refreshed.gizmo_basis_snapshot = snapshot_gizmo_basis(_maneuver_gizmo_interaction);
        refreshed.display_frame_snapshot =
                track.cache.resolved_frame_spec_valid ? track.cache.resolved_frame_spec : _prediction_frame_selection.spec;
        refreshed.patch_window_s = std::max(maneuver_plan_preview_window_s(), maneuver_post_node_coverage_s());
        refreshed.request_window_s = std::max(0.0, (refreshed.anchor_time_s - now_s) + refreshed.patch_window_s);
        refreshed.downstream_maneuver_node_ids.reserve(_maneuver_state.nodes.size());
        const double patch_end_s = refreshed.anchor_time_s + refreshed.patch_window_s;
        for (const ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!std::isfinite(node.time_s) ||
                node.time_s < refreshed.anchor_time_s ||
                node.time_s > patch_end_s)
            {
                continue;
            }
            refreshed.downstream_maneuver_node_ids.push_back(node.id);
        }

        if (!track.cache.trajectory_inertial.empty())
        {
            orbitsim::State anchor_state{};
            if (sample_prediction_inertial_state(track.cache.trajectory_inertial, refreshed.anchor_time_s, anchor_state))
            {
                refreshed.anchor_state_inertial = anchor_state;
            }
        }

        const bool anchor_changed = !preview_anchor_matches(track.preview_anchor, refreshed);
        track.preview_anchor = std::move(refreshed);
        if (anchor_changed)
        {
            track.preview_last_anchor_refresh_at_s = now_s;
            if (!std::isfinite(track.preview_entered_at_s))
            {
                track.preview_entered_at_s = now_s;
            }

            if (track.preview_state == PredictionPreviewRuntimeState::DragPreviewPending ||
                track.preview_state == PredictionPreviewRuntimeState::PreviewStreaming)
            {
                track.invalidated_while_pending = track.request_pending || track.derived_request_pending;
            }
            track.preview_state = PredictionPreviewRuntimeState::EnterDrag;
            return;
        }

        if (track.preview_state == PredictionPreviewRuntimeState::Idle ||
            track.preview_state == PredictionPreviewRuntimeState::AwaitFullRefine)
        {
            if (!std::isfinite(track.preview_entered_at_s))
            {
                track.preview_entered_at_s = now_s;
            }
            track.preview_state = PredictionPreviewRuntimeState::EnterDrag;
        }
    }

    void GameplayState::poll_completed_prediction_results()
    {
        bool applied_result = false;
        while (auto completed = _prediction_service.poll_completed())
        {
            apply_completed_prediction_result(std::move(*completed));
            applied_result = true;
        }

        while (auto completed = _prediction_derived_service.poll_completed())
        {
            apply_completed_prediction_derived_result(std::move(*completed));
            applied_result = true;
        }

        if (applied_result)
        {
            sync_prediction_dirty_flag();
        }
    }

    void GameplayState::clear_prediction_runtime()
    {
        // Drop every cached artifact when the feature is disabled.
        for (PredictionTrackState &track : _prediction_tracks)
        {
            track.cache.clear();
            track.request_pending = false;
            track.derived_request_pending = false;
            track.latest_requested_generation_id = 0;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            track.dirty = false;
            track.invalidated_while_pending = false;
            track.preview_state = PredictionPreviewRuntimeState::Idle;
            track.preview_anchor.clear();
            track.preview_entered_at_s = std::numeric_limits<double>::quiet_NaN();
            track.preview_last_anchor_refresh_at_s = std::numeric_limits<double>::quiet_NaN();
            track.preview_last_request_at_s = std::numeric_limits<double>::quiet_NaN();
            track.auto_primary_body_id = orbitsim::kInvalidBodyId;
            track.solver_ms_last = 0.0;
            track.solver_diagnostics = {};
            track.derived_diagnostics = {};
        }
        _prediction_service.reset();
        _prediction_derived_service.reset();
        _prediction_dirty = false;
        _maneuver_plan_live_preview_active = false;
    }

    void GameplayState::clear_visible_prediction_runtime(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        // A service reset invalidates every track, even ones that are currently hidden.
        (void) visible_subjects;
        for (PredictionTrackState &track : _prediction_tracks)
        {
            track.clear_runtime();
        }
        _prediction_derived_service.reset();
        _maneuver_plan_live_preview_active = false;
    }

    void GameplayState::apply_completed_prediction_result(OrbitPredictionService::Result result)
    {
        // Queue heavy frame/metrics derivation off-thread, then swap the completed cache on the main thread.
        PredictionTrackState *track = nullptr;
        for (PredictionTrackState &candidate : _prediction_tracks)
        {
            if (candidate.key.track_id() == result.track_id)
            {
                track = &candidate;
                break;
            }
        }

        if (!track)
        {
            return;
        }

        const OrbitPredictionService::AdaptiveStageDiagnostics previous_frame_base_diagnostics =
                track->derived_diagnostics.frame_base;
        track->solver_ms_last = std::max(0.0, result.compute_time_ms);
        track->solver_diagnostics = result.diagnostics;
        track->derived_diagnostics = {};

        if (!result.valid || result.trajectory_inertial.size() < 2)
        {
            track->request_pending = !result.generation_complete;
            track->derived_request_pending = false;
            track->pending_solve_quality = result.generation_complete
                                                   ? OrbitPredictionService::SolveQuality::Full
                                                   : result.solve_quality;
            track->dirty = true;
            return;
        }

        WorldVec3 build_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 build_vel_world{0.0};
        glm::vec3 build_vel_local{0.0f};
        (void) get_prediction_subject_world_state(track->key, build_pos_world, build_vel_world, build_vel_local);

        OrbitPredictionCache resolve_cache{};
        resolve_cache.build_time_s = result.build_time_s;
        resolve_cache.shared_ephemeris = result.shared_ephemeris;
        resolve_cache.massive_bodies = result.massive_bodies;
        resolve_cache.trajectory_segments_inertial = result.trajectory_segments_inertial;
        const double reference_time_s = _orbitsim ? _orbitsim->sim.time_s() : result.build_time_s;
        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                resolve_prediction_display_frame_spec(resolve_cache, reference_time_s);

        resolve_cache.trajectory_inertial = result.trajectory_inertial;
        orbitsim::BodyId analysis_body_id =
                resolve_prediction_analysis_body_id(resolve_cache,
                                                   track->key,
                                                   reference_time_s,
                                                   track->auto_primary_body_id);
        if (analysis_body_id != orbitsim::kInvalidBodyId)
        {
            track->auto_primary_body_id = analysis_body_id;
        }

        std::vector<orbitsim::TrajectorySegment> player_lookup_segments;
        if (prediction_subject_is_player(track->key))
        {
            if (!result.trajectory_segments_inertial_planned.empty())
            {
                player_lookup_segments = result.trajectory_segments_inertial_planned;
            }
            else if (!result.trajectory_segments_inertial.empty())
            {
                player_lookup_segments = result.trajectory_segments_inertial;
            }
        }
        else if (const PredictionTrackState *player_track = player_prediction_track())
        {
            if (!player_track->cache.trajectory_segments_inertial_planned.empty())
            {
                player_lookup_segments = player_track->cache.trajectory_segments_inertial_planned;
            }
            else if (!player_track->cache.trajectory_segments_inertial.empty())
            {
                player_lookup_segments = player_track->cache.trajectory_segments_inertial;
            }
        }

        OrbitPredictionDerivedService::Request derived_request{};
        derived_request.track_id = result.track_id;
        derived_request.generation_id = result.generation_id;
        derived_request.solver_result = std::move(result);
        derived_request.reuse_existing_base_frame =
                can_reuse_existing_base_frame_cache(*track,
                                                   derived_request.solver_result,
                                                   resolved_frame_spec);
        derived_request.reused_base_frame_diagnostics = previous_frame_base_diagnostics;
        derived_request.build_pos_world = build_pos_world;
        derived_request.build_vel_world = build_vel_world;
        derived_request.sim_config = _orbitsim ? _orbitsim->sim.config() : orbitsim::GameSimulation::Config{};
        derived_request.resolved_frame_spec = resolved_frame_spec;
        derived_request.analysis_body_id = analysis_body_id;
        derived_request.player_lookup_segments_inertial = std::move(player_lookup_segments);
        _prediction_derived_service.request(std::move(derived_request));
        // Keep request_pending set until the solver publishes the final staged preview result for this generation.
        track->request_pending = !result.generation_complete;
        track->derived_request_pending = true;
        // If the input changed while this solve was in-flight, promote straight to dirty so the
        // next update tick can submit a fresh solver request without waiting for derived to finish.
        if (track->invalidated_while_pending)
        {
            track->dirty = true;
            track->invalidated_while_pending = false;
        }
        track->pending_solve_quality = result.generation_complete
                                               ? OrbitPredictionService::SolveQuality::Full
                                               : result.solve_quality;
    }

    void GameplayState::apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result)
    {
        PredictionTrackState *track = nullptr;
        for (PredictionTrackState &candidate : _prediction_tracks)
        {
            if (candidate.key.track_id() == result.track_id)
            {
                track = &candidate;
                break;
            }
        }

        if (!track)
        {
            return;
        }

        const bool stale_derived_result =
                track->cache.valid && result.generation_id < track->cache.generation_id;
        if (stale_derived_result)
        {
            return;
        }

        track->derived_request_pending = false;
        // Do NOT clear request_pending here — a newer solver request may already be in-flight.
        // Only the solver completion path and request submission paths manage that flag.
        const bool keep_dirty_for_followup = track->invalidated_while_pending;
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

            if (have_cache_to_publish &&
                result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview)
            {
                cache_to_publish = merge_preview_planned_prefix_cache(track->cache, std::move(cache_to_publish));
                have_cache_to_publish = cache_to_publish.valid;
            }
        }

        track->derived_diagnostics = diagnostics_to_publish;

        if (!have_cache_to_publish)
        {
            track->derived_diagnostics.status = PredictionDerivedStatus::MissingSolverData;
            track->dirty = true;
            return;
        }

        track->cache = std::move(cache_to_publish);
        track->pick_cache.clear();
        const bool preview_result = result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview;
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
                _maneuver_gizmo_interaction.state != ManeuverGizmoInteraction::State::DragAxis;
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
    }

    double GameplayState::prediction_display_window_s(const PredictionSubjectKey key,
                                                      const double now_s,
                                                      const bool with_maneuvers) const
    {
        const double plotted_ahead_s =
                std::max(0.0, _prediction_draw_future_segment ? prediction_future_window_s(key) : 0.0);

        if (!with_maneuvers)
        {
            return plotted_ahead_s;
        }

        double required_ahead_s = plotted_ahead_s;
        double max_node_time_s = now_s;
        for (const ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!std::isfinite(node.time_s))
            {
                continue;
            }
            max_node_time_s = std::max(max_node_time_s, node.time_s);
        }

        if (max_node_time_s > now_s)
        {
            required_ahead_s = std::max(required_ahead_s, (max_node_time_s - now_s) + maneuver_post_node_coverage_s());
        }

        return required_ahead_s;
    }

    double GameplayState::prediction_preview_patch_window_s(const PredictionTrackState &track,
                                                            const double now_s,
                                                            const bool with_maneuvers) const
    {
        (void) now_s;
        const bool preview_live =
                with_maneuvers &&
                track.key == _prediction_selection.active_subject &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);
        if (!preview_live || !track.preview_anchor.valid)
        {
            return 0.0;
        }
        return std::max(0.0, track.preview_anchor.patch_window_s);
    }

    double GameplayState::prediction_required_window_s(const PredictionTrackState &track,
                                                       const double now_s,
                                                       const bool with_maneuvers) const
    {
        const double display_window_s = prediction_display_window_s(track.key, now_s, with_maneuvers);
        const bool preview_live =
                with_maneuvers &&
                track.key == _prediction_selection.active_subject &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);
        if (!preview_live || !track.preview_anchor.valid)
        {
            return display_window_s;
        }

        // Slice 1 keeps the legacy full-horizon request path alive, but now tracks the local patch
        // window separately so follow-up slices can swap the solver over without changing runtime state.
        return std::max(display_window_s, std::max(0.0, track.preview_anchor.request_window_s));
    }

    double GameplayState::prediction_required_window_s(const PredictionSubjectKey key,
                                                       const double now_s,
                                                       const bool with_maneuvers) const
    {
        if (const PredictionTrackState *track = find_prediction_track(key))
        {
            return prediction_required_window_s(*track, now_s, with_maneuvers);
        }

        const double display_window_s = prediction_display_window_s(key, now_s, with_maneuvers);
        const bool preview_subject_matches =
                !_prediction_selection.active_subject.valid() ||
                key == _prediction_selection.active_subject;
        const bool maneuver_live_preview =
                with_maneuvers &&
                preview_subject_matches &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);
        if (!maneuver_live_preview)
        {
            return display_window_s;
        }

        const ManeuverNode *anchor_node = select_preview_anchor_node(_maneuver_state, now_s);
        if (!anchor_node || !std::isfinite(anchor_node->time_s))
        {
            return display_window_s;
        }

        const double preview_window_s = std::max(maneuver_plan_preview_window_s(), maneuver_post_node_coverage_s());
        return std::max(0.0, (std::max(now_s, anchor_node->time_s) - now_s) + preview_window_s);
    }

    bool GameplayState::should_rebuild_prediction_track(const PredictionTrackState &track,
                                                        const double now_s,
                                                        const float fixed_dt,
                                                        const bool thrusting,
                                                        const bool with_maneuvers) const
    {
        // Rebuild when cache state, thrusting, timing, or horizon coverage says we must.
        const bool maneuver_live_preview =
                with_maneuvers &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);
        bool rebuild = track.dirty || !track.cache.valid;
        if (!rebuild && track.preview_state == PredictionPreviewRuntimeState::EnterDrag)
        {
            rebuild = true;
        }
        if (!rebuild &&
            track.request_pending &&
            maneuver_live_preview &&
            (track.invalidated_while_pending ||
             track.pending_solve_quality != OrbitPredictionService::SolveQuality::FastPreview))
        {
            rebuild = true;
        }

        if (!rebuild && thrusting)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= _prediction_thrust_refresh_s;
        }

        if (!rebuild && maneuver_live_preview)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= OrbitPredictionTuning::kManeuverRefreshS;
        }

        if (!rebuild && _prediction_periodic_refresh_s > 0.0 && (!with_maneuvers || maneuver_live_preview))
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= _prediction_periodic_refresh_s;
        }

        // Throttle live gizmo drags so the async solver does not thrash every tick.
        if (rebuild &&
            track.cache.valid &&
            maneuver_live_preview &&
            _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            if (dt_since_build_s < OrbitPredictionTuning::kDragRebuildMinIntervalS)
            {
                rebuild = false;
            }
        }

        if (rebuild || !track.cache.valid || track.cache.trajectory_inertial.empty())
        {
            return rebuild;
        }

        double cache_end_s = track.cache.trajectory_inertial.back().t_s;
        if (!track.cache.trajectory_segments_inertial.empty())
        {
            const orbitsim::TrajectorySegment &last_segment = track.cache.trajectory_segments_inertial.back();
            const double segment_end_s = last_segment.t0_s + last_segment.dt_s;
            if (std::isfinite(segment_end_s))
            {
                cache_end_s = segment_end_s;
            }
        }

        const double required_ahead_s = prediction_required_window_s(track, now_s, with_maneuvers);

        // Add a small epsilon so tiny fixed-step jitter does not trigger rebuild churn.
        const double coverage_epsilon_s =
                std::max(1.0e-3, std::min(0.25, std::max(0.0, static_cast<double>(fixed_dt)) * 0.5));
        return (cache_end_s - now_s + coverage_epsilon_s) < required_ahead_s;
    }

    bool GameplayState::request_orbiter_prediction_async(PredictionTrackState &track,
                                                         const WorldVec3 &subject_pos_world,
                                                         const glm::dvec3 &subject_vel_world,
                                                         const double now_s,
                                                         const bool thrusting,
                                                         const bool with_maneuvers)
    {
        // Package the current spacecraft state into a worker request.
        refresh_prediction_preview_anchor(track, now_s, with_maneuvers);
        if (!_orbitsim)
        {
            return false;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!ref_sim || ref_sim->id == orbitsim::kInvalidBodyId)
        {
            return false;
        }

        const WorldVec3 ref_body_world = prediction_world_reference_body_world();
        const glm::dvec3 ship_rel_pos_m = glm::dvec3(subject_pos_world - ref_body_world);
        const glm::dvec3 ship_rel_vel_mps = subject_vel_world;
        const glm::dvec3 ship_bary_pos_m = ref_sim->state.position_m + ship_rel_pos_m;
        const glm::dvec3 ship_bary_vel_mps = ref_sim->state.velocity_mps + ship_rel_vel_mps;
        if (!finite_vec3(ship_bary_pos_m) || !finite_vec3(ship_bary_vel_mps))
        {
            return false;
        }

        OrbitPredictionService::Request request{};
        request.track_id = track.key.track_id();
        request.sim_time_s = now_s;
        request.sim_config = _orbitsim->sim.config();
        request.massive_bodies = _orbitsim->sim.massive_bodies();
        request.shared_ephemeris = track.cache.shared_ephemeris;
        request.ship_bary_position_m = ship_bary_pos_m;
        request.ship_bary_velocity_mps = ship_bary_vel_mps;
        request.thrusting = thrusting;
        const bool maneuver_live_preview =
                with_maneuvers &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);
        const bool interactive_request =
                track.key == _prediction_selection.active_subject && (thrusting || maneuver_live_preview);
        request.solve_quality = maneuver_live_preview
                                        ? OrbitPredictionService::SolveQuality::FastPreview
                                        : OrbitPredictionService::SolveQuality::Full;
        request.priority = classify_prediction_request_priority(
                _prediction_selection,
                track.key,
                track.is_celestial,
                interactive_request);
        request.future_window_s = prediction_required_window_s(track, now_s, with_maneuvers);
        if (request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
            track.preview_anchor.valid &&
            finite_vec3(track.preview_anchor.anchor_state_inertial.position_m) &&
            finite_vec3(track.preview_anchor.anchor_state_inertial.velocity_mps))
        {
            request.preview_patch.active = true;
            request.preview_patch.anchor_state_valid = true;
            request.preview_patch.baseline_generation_id = track.preview_anchor.baseline_generation_id;
            request.preview_patch.anchor_time_s = track.preview_anchor.anchor_time_s;
            request.preview_patch.patch_window_s = std::max(0.0, track.preview_anchor.patch_window_s);
            request.preview_patch.anchor_state_inertial = track.preview_anchor.anchor_state_inertial;
        }
        const orbitsim::TrajectoryFrameSpec display_frame_spec =
                track.cache.resolved_frame_spec_valid ? track.cache.resolved_frame_spec : _prediction_frame_selection.spec;
        request.lagrange_sensitive = prediction_frame_is_lagrange_sensitive(display_frame_spec);
        request.preferred_primary_body_id = track.auto_primary_body_id;
        if (request.preferred_primary_body_id == orbitsim::kInvalidBodyId &&
            _prediction_analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            _prediction_analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = _prediction_analysis_selection.spec.fixed_body_id;
        }
        if (request.preferred_primary_body_id == orbitsim::kInvalidBodyId &&
            display_frame_spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = display_frame_spec.primary_body_id;
        }

        // Copy currently authored maneuver nodes so the worker can include planned burns.
        if (with_maneuvers)
        {
            request.maneuver_impulses.reserve(_maneuver_state.nodes.size());
            const double request_horizon_end_s = now_s + request.future_window_s;
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (!std::isfinite(node.time_s))
                {
                    continue;
                }

                // FP-0 stays patch-bounded inside the solver. The request still has to carry every
                // downstream maneuver in the requested horizon so FP-1 can refine the remaining tail.
                if (request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
                    node.time_s > request_horizon_end_s)
                {
                    continue;
                }

                OrbitPredictionService::ManeuverImpulse impulse{};
                impulse.node_id = node.id;
                impulse.t_s = node.time_s;
                // For auto-primary nodes, let the worker resolve the dominant body at the node time
                // from the propagated state instead of baking in a stale cache-based guess here.
                impulse.primary_body_id = node.primary_body_auto
                                                  ? orbitsim::kInvalidBodyId
                                                  : resolve_maneuver_node_primary_body_id(node, node.time_s);
                impulse.dv_rtn_mps = orbitsim::Vec3{node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z};
                request.maneuver_impulses.push_back(impulse);
            }
        }

        const uint64_t generation_id = _prediction_service.request(std::move(request));
        track.latest_requested_generation_id = generation_id;
        track.request_pending = true;
        track.derived_request_pending = false;
        track.pending_solve_quality = maneuver_live_preview
                                              ? OrbitPredictionService::SolveQuality::FastPreview
                                              : OrbitPredictionService::SolveQuality::Full;
        track.invalidated_while_pending = false;
        track.preview_last_request_at_s = now_s;
        if (maneuver_live_preview)
        {
            track.preview_state = PredictionPreviewRuntimeState::DragPreviewPending;
            if (!std::isfinite(track.preview_entered_at_s))
            {
                track.preview_entered_at_s = now_s;
            }
        }
        else if (track.preview_state != PredictionPreviewRuntimeState::Idle)
        {
            track.preview_state = PredictionPreviewRuntimeState::AwaitFullRefine;
        }
        return true;
    }

    bool GameplayState::request_celestial_prediction_async(PredictionTrackState &track, const double now_s)
    {
        if (!_orbitsim)
        {
            return false;
        }

        const orbitsim::MassiveBody *body = _orbitsim->sim.body_by_id(static_cast<orbitsim::BodyId>(track.key.value));
        if (!body)
        {
            return false;
        }

        OrbitPredictionService::Request request{};
        request.kind = OrbitPredictionService::RequestKind::Celestial;
        request.track_id = track.key.track_id();
        request.sim_time_s = now_s;
        request.sim_config = _orbitsim->sim.config();
        request.massive_bodies = _orbitsim->sim.massive_bodies();
        request.shared_ephemeris = track.cache.shared_ephemeris;
        request.subject_body_id = body->id;
        request.priority = classify_prediction_request_priority(
                _prediction_selection,
                track.key,
                true,
                false);
        request.future_window_s = prediction_future_window_s(track.key);
        request.lagrange_sensitive = prediction_frame_is_lagrange_sensitive(_prediction_frame_selection.spec);
        if (_prediction_analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            _prediction_analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = _prediction_analysis_selection.spec.fixed_body_id;
        }
        if (request.preferred_primary_body_id == orbitsim::kInvalidBodyId &&
            _prediction_frame_selection.spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = _prediction_frame_selection.spec.primary_body_id;
        }

        // Celestial tracks now flow through the same worker queue as spacecraft tracks.
        const uint64_t generation_id = _prediction_service.request(std::move(request));
        track.latest_requested_generation_id = generation_id;
        track.request_pending = true;
        track.derived_request_pending = false;
        track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
        track.invalidated_while_pending = false;
        return true;
    }

    void GameplayState::update_orbiter_prediction_track(PredictionTrackState &track,
                                                        const double now_s,
                                                        const bool thrusting,
                                                        const bool with_maneuvers)
    {
        // Orbiters rebuild asynchronously from their current world-space snapshot.
        WorldVec3 subject_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 subject_vel_world{0.0};
        glm::vec3 subject_vel_local{0.0f};
        if (!get_prediction_subject_world_state(track.key, subject_pos_world, subject_vel_world, subject_vel_local))
        {
            track.cache.clear();
            track.dirty = true;
            track.request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            track.preview_state = PredictionPreviewRuntimeState::Idle;
            track.preview_anchor.clear();
            return;
        }

        const bool maneuver_live_preview =
                with_maneuvers &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);
        const bool supersede_preview_request =
                track.request_pending &&
                maneuver_live_preview &&
                (track.invalidated_while_pending ||
                 track.pending_solve_quality != OrbitPredictionService::SolveQuality::FastPreview);
        if (track.request_pending && !supersede_preview_request)
        {
            return;
        }

        const bool requested =
                request_orbiter_prediction_async(track, subject_pos_world, subject_vel_world, now_s, thrusting, with_maneuvers);
        track.dirty = !requested;
        if (!requested)
        {
            track.cache.clear();
            track.request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            track.preview_state = PredictionPreviewRuntimeState::Idle;
            track.preview_anchor.clear();
        }
    }

    void GameplayState::update_celestial_prediction_track(PredictionTrackState &track, const double now_s)
    {
        // Celestial predictions rebuild asynchronously so the gameplay thread only packages requests.
        if (!_orbitsim)
        {
            track.cache.clear();
            track.dirty = true;
            track.request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            return;
        }

        if (track.request_pending)
        {
            return;
        }

        const bool requested = request_celestial_prediction_async(track, now_s);
        // Keep drawing the previous cache until the worker publishes a replacement.
        track.dirty = !requested;
        if (!requested)
        {
            track.cache.clear();
            track.request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
        }
    }

    void GameplayState::update_prediction(GameStateContext &ctx, float fixed_dt)
    {
        (void) ctx;

        if (!_prediction_enabled)
        {
            // Fully disable runtime state when the overlay is turned off.
            clear_prediction_runtime();
            return;
        }

        // Keep the subject list aligned with the current gameplay scene.
        rebuild_prediction_subjects();
        rebuild_prediction_frame_options();
        rebuild_prediction_analysis_options();

        const double now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;

        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        if (!_orbitsim)
        {
            // Without orbit-sim there is no stable frame to predict against.
            clear_visible_prediction_runtime(visible_subjects);
            _prediction_service.reset();
            sync_prediction_dirty_flag();
            return;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!ref_sim || ref_sim->id == orbitsim::kInvalidBodyId)
        {
            // Bail out when the reference body is not ready for a stable inertial frame.
            clear_visible_prediction_runtime(visible_subjects);
            _prediction_service.reset();
            sync_prediction_dirty_flag();
            return;
        }

        // Rebuild or refresh only the subjects that are actually visible.
        for (PredictionTrackState &track : _prediction_tracks)
        {
            if (!contains_key(visible_subjects, track.key))
            {
                continue;
            }

            const bool with_maneuvers =
                    prediction_subject_supports_maneuvers(track.key) &&
                    _maneuver_nodes_enabled &&
                    !_maneuver_state.nodes.empty();
            const bool thrusting = prediction_subject_thrust_applied_this_tick(track.key);
            refresh_prediction_preview_anchor(track, now_s, with_maneuvers);
            const bool rebuild = should_rebuild_prediction_track(track, now_s, fixed_dt, thrusting, with_maneuvers);
            if (!rebuild)
            {
                continue;
            }

            if (track.key.kind == PredictionSubjectKind::Celestial)
            {
                update_celestial_prediction_track(track, now_s);
                continue;
            }

            update_orbiter_prediction_track(track, now_s, thrusting, with_maneuvers);
        }

        // Mirror the active track's last solver time into the shared debug HUD stats.
        PredictionTrackState *active_track = active_prediction_track();
        _orbit_plot_perf.solver_ms_last = active_track ? active_track->solver_ms_last : 0.0;
        sync_prediction_dirty_flag();
    }

} // namespace Game
