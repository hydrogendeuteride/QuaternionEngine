#include "game/states/gameplay/prediction/prediction_system.h"

#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/orbit_helpers.h"
#include "game/states/gameplay/prediction/runtime/prediction_invalidation_controller.h"
#include "game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.h"
#include "game/states/gameplay/prediction/runtime/prediction_runtime_controller.h"

#include <algorithm>
#include <utility>

namespace Game
{
#if defined(VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS)
    GameplayPredictionState &GameplayState::prediction_for_test()
    {
        return _prediction->state();
    }

    const GameplayPredictionState &GameplayState::prediction_for_test() const
    {
        return _prediction->state();
    }

    PredictionSystem &GameplayState::prediction_system_for_test()
    {
        return *_prediction;
    }

    const PredictionSystem &GameplayState::prediction_system_for_test() const
    {
        return *_prediction;
    }
#endif

    GameplayPredictionState &PredictionSystem::state()
    {
        return _state;
    }

    const GameplayPredictionState &PredictionSystem::state() const
    {
        return _state;
    }

    OrbitPlotBudgetSettings &PredictionSystem::budget()
    {
        return _budget;
    }

    const OrbitPlotBudgetSettings &PredictionSystem::budget() const
    {
        return _budget;
    }

    OrbitPredictionDerivedService &PredictionSystem::derived_service()
    {
        return _state.derived_service;
    }

    const OrbitPredictionDerivedService &PredictionSystem::derived_service() const
    {
        return _state.derived_service;
    }

    void PredictionSystem::reset_solver_service()
    {
        _state.service.reset();
    }

    void PredictionSystem::reset_derived_service()
    {
        _state.derived_service.reset();
    }

    void PredictionSystem::reset_services()
    {
        reset_solver_service();
        reset_derived_service();
    }

    void PredictionSystem::reset_session_state()
    {
        reset_services();
        _state.tracks.clear();
        _state.groups.clear();
        _state.selection.clear();
        _state.frame_selection.clear();
        _state.analysis_selection.clear();
        _state.dirty = true;
        _state.orbit_plot_perf = {};
    }

    void PredictionSystem::sync_subjects(const std::vector<PredictionSubjectDescriptor> &subjects,
                                          const PredictionSubjectKey preferred_active_subject)
    {
        const PredictionSubjectKey old_active = _state.selection.active_subject;
        const std::vector<PredictionSubjectKey> old_overlays = _state.selection.overlay_subjects;
        std::vector<PredictionTrackState> old_tracks = std::move(_state.tracks);

        _state.tracks.clear();
        _state.groups.clear();

        const auto find_old_track = [&](const PredictionSubjectKey key) -> PredictionTrackState * {
            auto it = std::find_if(old_tracks.begin(),
                                   old_tracks.end(),
                                   [key](const PredictionTrackState &track) { return track.key == key; });
            return (it != old_tracks.end()) ? &(*it) : nullptr;
        };

        for (const PredictionSubjectDescriptor &subject : subjects)
        {
            if (!subject.key.valid())
            {
                continue;
            }

            PredictionTrackState track{};
            if (PredictionTrackState *old = find_old_track(subject.key))
            {
                track = std::move(*old);
            }

            track.key = subject.key;
            track.label = subject.label;
            track.supports_maneuvers = subject.supports_maneuvers;
            track.is_celestial = subject.is_celestial;
            _state.tracks.push_back(std::move(track));
        }

        const auto track_exists = [&](const PredictionSubjectKey key) {
            return find_track(key) != nullptr;
        };

        if (track_exists(old_active))
        {
            _state.selection.active_subject = old_active;
        }
        else if (track_exists(preferred_active_subject))
        {
            _state.selection.active_subject = preferred_active_subject;
        }
        else if (!_state.tracks.empty())
        {
            _state.selection.active_subject = _state.tracks.front().key;
        }
        else
        {
            _state.selection.active_subject = {};
        }

        _state.selection.overlay_subjects.clear();
        _state.selection.overlay_subjects.reserve(old_overlays.size());
        for (const PredictionSubjectKey key : old_overlays)
        {
            if (!track_exists(key))
            {
                continue;
            }

            if (std::find(_state.selection.overlay_subjects.begin(),
                          _state.selection.overlay_subjects.end(),
                          key) == _state.selection.overlay_subjects.end())
            {
                _state.selection.overlay_subjects.push_back(key);
            }
        }

        if (_state.selection.overlay_subjects.empty())
        {
            for (const PredictionTrackState &track : _state.tracks)
            {
                if (track.key == _state.selection.active_subject)
                {
                    continue;
                }

                _state.selection.overlay_subjects.push_back(track.key);
            }
        }

        _state.selection.selected_group_index = -1;
        sync_visible_dirty_flag(collect_visible_subjects());
    }

    void PredictionSystem::sync_subjects(const PredictionHostContext &host)
    {
        sync_subjects(host.subjects, host.player_subject);
    }

    std::vector<PredictionSubjectKey> PredictionSystem::collect_visible_subjects() const
    {
        std::vector<PredictionSubjectKey> out;
        out.reserve(1 + _state.selection.overlay_subjects.size());

        const auto append_visible = [&](const PredictionSubjectKey key) {
            if (!key.valid() || !find_track(key))
            {
                return;
            }

            if (std::find(out.begin(), out.end(), key) == out.end())
            {
                out.push_back(key);
            }
        };

        append_visible(_state.selection.active_subject);
        for (const PredictionSubjectKey key : _state.selection.overlay_subjects)
        {
            append_visible(key);
        }

        if (out.empty() && !_state.tracks.empty())
        {
            append_visible(_state.tracks.front().key);
        }

        return out;
    }

    PredictionTrackState *PredictionSystem::find_track(const PredictionSubjectKey key)
    {
        auto it = std::find_if(_state.tracks.begin(),
                               _state.tracks.end(),
                               [key](const PredictionTrackState &track) { return track.key == key; });
        return (it != _state.tracks.end()) ? &(*it) : nullptr;
    }

    const PredictionTrackState *PredictionSystem::find_track(const PredictionSubjectKey key) const
    {
        auto it = std::find_if(_state.tracks.begin(),
                               _state.tracks.end(),
                               [key](const PredictionTrackState &track) { return track.key == key; });
        return (it != _state.tracks.end()) ? &(*it) : nullptr;
    }

    PredictionTrackState *PredictionSystem::active_track()
    {
        return find_track(_state.selection.active_subject);
    }

    const PredictionTrackState *PredictionSystem::active_track() const
    {
        return find_track(_state.selection.active_subject);
    }

    PredictionTrackState *PredictionSystem::player_track(const PredictionSubjectKey player_subject)
    {
        return find_track(player_subject);
    }

    const PredictionTrackState *PredictionSystem::player_track(const PredictionSubjectKey player_subject) const
    {
        return find_track(player_subject);
    }

    OrbitPredictionCache *PredictionSystem::effective_cache(PredictionTrackState *track)
    {
        if (!track)
        {
            return nullptr;
        }
        return track->cache.identity.valid ? &track->cache : nullptr;
    }

    const OrbitPredictionCache *PredictionSystem::effective_cache(const PredictionTrackState *track) const
    {
        if (!track)
        {
            return nullptr;
        }
        return track->cache.identity.valid ? &track->cache : nullptr;
    }

    OrbitPredictionCache *PredictionSystem::player_cache(const PredictionSubjectKey player_subject)
    {
        return effective_cache(player_track(player_subject));
    }

    const OrbitPredictionCache *PredictionSystem::player_cache(const PredictionSubjectKey player_subject) const
    {
        return effective_cache(player_track(player_subject));
    }

    OrbitPlotPerfStats &PredictionSystem::perf_stats()
    {
        return _state.orbit_plot_perf;
    }

    const OrbitPlotPerfStats &PredictionSystem::perf_stats() const
    {
        return _state.orbit_plot_perf;
    }

    bool PredictionSystem::any_visible_track_dirty(const std::vector<PredictionSubjectKey> &visible_subjects) const
    {
        return PredictionInvalidationController::any_visible_track_dirty(_state.tracks, visible_subjects);
    }

    void PredictionSystem::sync_visible_dirty_flag(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        _state.dirty = any_visible_track_dirty(visible_subjects);
    }

    void PredictionSystem::mark_visible_tracks_dirty(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        PredictionInvalidationController::mark_visible_tracks_dirty(_state.tracks, visible_subjects);
    }

    void PredictionSystem::invalidate_maneuver_plan_revision(const uint64_t maneuver_plan_revision)
    {
        PredictionInvalidationController::invalidate_maneuver_plan_revision(_state.tracks,
                                                                           _state.service,
                                                                           _state.derived_service,
                                                                           maneuver_plan_revision);
    }

    void PredictionSystem::clear_maneuver_prediction_artifacts()
    {
        PredictionInvalidationController::clear_maneuver_prediction_artifacts(_state.tracks);
    }

    void PredictionSystem::mark_maneuver_preview_dirty(PredictionTrackState &track)
    {
        PredictionLifecycleReducer::mark_dirty_for_preview(track);
    }

    void PredictionSystem::await_maneuver_preview_full_refine(PredictionTrackState &track,
                                                              const double now_s)
    {
        PredictionLifecycleReducer::await_full_refine(track, now_s);
    }

    void PredictionSystem::clear_unapplied_maneuver_drag_preview(PredictionTrackState &track)
    {
        if (track.preview_state != PredictionPreviewRuntimeState::EnterDrag &&
            track.preview_state != PredictionPreviewRuntimeState::DragPreviewPending)
        {
            return;
        }

        PredictionLifecycleReducer::reset_preview(track);
        track.preview_overlay.clear();
        track.pick_cache.clear();
    }

    void PredictionSystem::clear_maneuver_live_preview_state()
    {
        for (PredictionTrackState &track : _state.tracks)
        {
            if (!track.supports_maneuvers)
            {
                continue;
            }

            PredictionLifecycleReducer::reset_preview(track);
            track.preview_overlay.clear();
            track.pick_cache.clear();
        }
    }

    PredictionRuntimeContext PredictionSystem::build_runtime_context(const PredictionHostContext &host) const
    {
        PredictionRuntimeContext context{};
        context.orbital_scenario = host.orbital_scenario;
        context.selection = _state.selection;
        context.frame_selection = _state.frame_selection;
        context.analysis_selection = _state.analysis_selection;
        context.sampling_policy = _state.sampling_policy;
        context.maneuver_plan_windows = host.maneuver.plan_windows;
        context.maneuver_plan = host.maneuver.plan;
        context.maneuver_edit_preview = host.maneuver.edit_preview;
        context.maneuver_nodes_enabled = host.maneuver.nodes_enabled;
        context.maneuver_edit_in_progress = host.maneuver.edit_in_progress;
        context.maneuver_live_preview_available = host.maneuver.live_preview_active;
        context.maneuver_plan_revision = host.maneuver.revision;
        context.maneuver_plan_signature = host.maneuver.signature;
        context.display_frame_revision = _state.display_frame_revision;
        context.active_maneuver_preview_anchor_node_id = host.maneuver.active_preview_anchor_node_id;
        context.current_sim_time_s = host.current_sim_time_s;
        context.reference_body_world = host.reference_body_world;
        context.get_subject_world_state = host.get_subject_world_state;
        context.future_window_s = host.future_window_s;
        context.required_window_s = host.required_window_s;
        context.preview_exact_window_s = host.preview_exact_window_s;
        context.refresh_preview_anchor = host.refresh_preview_anchor;
        context.subject_is_player = host.subject_is_player;
        context.subject_thrust_applied_this_tick = host.subject_thrust_applied_this_tick;
        context.resolve_maneuver_node_primary_body_id = host.resolve_maneuver_node_primary_body_id;
        context.resolve_display_frame_spec = host.resolve_display_frame_spec;
        context.resolve_analysis_body_id = host.resolve_analysis_body_id;
        context.player_effective_cache = [this, player_subject = host.player_subject]() -> const OrbitPredictionCache * {
            return player_cache(player_subject);
        };
        return context;
    }

    void PredictionSystem::clear_runtime()
    {
        PredictionRuntimeController::clear_runtime(_state);
    }

    void PredictionSystem::clear_visible_runtime(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        PredictionRuntimeController::clear_visible_runtime(_state, visible_subjects);
    }

    bool PredictionSystem::poll_completed_results(const PredictionHostContext &host)
    {
        return PredictionRuntimeController::poll_completed_results(_state, build_runtime_context(host));
    }

    bool PredictionSystem::apply_completed_solver_result(const PredictionHostContext &host,
                                                         OrbitPredictionService::Result result)
    {
        return PredictionRuntimeController::apply_completed_solver_result(_state,
                                                                         build_runtime_context(host),
                                                                         std::move(result));
    }

    bool PredictionSystem::apply_completed_derived_result(const PredictionHostContext &host,
                                                          OrbitPredictionDerivedService::Result result)
    {
        return PredictionRuntimeController::apply_completed_derived_result(_state,
                                                                          build_runtime_context(host),
                                                                          std::move(result));
    }

    bool PredictionSystem::should_rebuild_track(const PredictionHostContext &host,
                                                const PredictionTrackState &track,
                                                const double now_s,
                                                const float fixed_dt,
                                                const bool thrusting,
                                                const bool with_maneuvers) const
    {
        return PredictionRuntimeController::should_rebuild_track(_state,
                                                                 build_runtime_context(host),
                                                                 track,
                                                                 now_s,
                                                                 fixed_dt,
                                                                 thrusting,
                                                                 with_maneuvers);
    }

    bool PredictionSystem::request_orbiter_prediction_async(const PredictionHostContext &host,
                                                            PredictionTrackState &track,
                                                            const WorldVec3 &subject_pos_world,
                                                            const glm::dvec3 &subject_vel_world,
                                                            const double now_s,
                                                            const bool thrusting,
                                                            const bool with_maneuvers,
                                                            bool *out_throttled)
    {
        return PredictionRuntimeController::request_orbiter_prediction_async(_state,
                                                                            build_runtime_context(host),
                                                                            track,
                                                                            subject_pos_world,
                                                                            subject_vel_world,
                                                                            now_s,
                                                                            thrusting,
                                                                            with_maneuvers,
                                                                            out_throttled);
    }

    bool PredictionSystem::request_celestial_prediction_async(const PredictionHostContext &host,
                                                              PredictionTrackState &track,
                                                              const double now_s)
    {
        return PredictionRuntimeController::request_celestial_prediction_async(_state,
                                                                               build_runtime_context(host),
                                                                               track,
                                                                               now_s);
    }

    void PredictionSystem::update_orbiter_prediction_track(const PredictionHostContext &host,
                                                           PredictionTrackState &track,
                                                           const double now_s,
                                                           const bool thrusting,
                                                           const bool with_maneuvers)
    {
        PredictionRuntimeController::update_orbiter_prediction_track(_state,
                                                                     build_runtime_context(host),
                                                                     track,
                                                                     now_s,
                                                                     thrusting,
                                                                     with_maneuvers);
    }

    void PredictionSystem::update_celestial_prediction_track(const PredictionHostContext &host,
                                                             PredictionTrackState &track,
                                                             const double now_s)
    {
        PredictionRuntimeController::update_celestial_prediction_track(_state,
                                                                       build_runtime_context(host),
                                                                       track,
                                                                       now_s);
    }

    void PredictionSystem::update(const PredictionHostContext &host, const float fixed_dt)
    {
        if (!_state.enabled)
        {
            clear_runtime();
            return;
        }

        sync_subjects(host);

        const double now_s = host.orbital_scenario ? host.orbital_scenario->sim.time_s() : host.current_sim_time_s;
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_subjects();
        if (!host.orbital_scenario)
        {
            clear_visible_runtime(visible_subjects);
            reset_solver_service();
            sync_visible_dirty_flag(visible_subjects);
            return;
        }

        const orbitsim::MassiveBody *ref_sim = host.orbital_scenario->world_reference_sim_body();
        if (!ref_sim || ref_sim->id == orbitsim::kInvalidBodyId)
        {
            clear_visible_runtime(visible_subjects);
            reset_solver_service();
            sync_visible_dirty_flag(visible_subjects);
            return;
        }

        PredictionRuntimeController::update_visible_tracks(_state,
                                                           build_runtime_context(host),
                                                           visible_subjects,
                                                           now_s,
                                                           fixed_dt);

        const PredictionTrackState *active = active_track();
        _state.orbit_plot_perf.solver_ms_last = active ? active->solver_ms_last : 0.0;
        sync_visible_dirty_flag(visible_subjects);
    }
} // namespace Game
