#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction/prediction_host_context_builder.h"
#include "game/states/gameplay/prediction/runtime/prediction_window_context_builder.h"

#include "core/util/logger.h"

namespace Game
{
    void GameplayPredictionAdapter::sync_prediction_dirty_flag()
    {
        // Collapse only visible-track rebuild demand into one cheap UI-facing flag.
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        _state._prediction->sync_visible_dirty_flag(visible_subjects);
    }

    void GameplayState::mark_prediction_dirty()
    {
        // Force only the active/overlay-visible tracks to rebuild on the next prediction update.
        GameplayPredictionAdapter prediction(*this);
        const std::vector<PredictionSubjectKey> visible_subjects = prediction.collect_visible_prediction_subjects();
        _prediction->mark_visible_tracks_dirty(visible_subjects);
        prediction.sync_prediction_dirty_flag();
    }

    void GameplayPredictionAdapter::mark_maneuver_plan_dirty()
    {
        const uint64_t revision = _state._maneuver.increment_revision();
        Logger::debug("Maneuver plan dirty: revision={} selected_node={} node_count={}",
                      revision,
                      _state._maneuver.plan().selected_node_id,
                      _state._maneuver.plan().nodes.size());

        _state._prediction->invalidate_maneuver_plan_revision(revision);

        _state.mark_prediction_dirty();
    }

    void GameplayPredictionAdapter::clear_maneuver_prediction_artifacts()
    {
        _state._prediction->clear_maneuver_prediction_artifacts();
    }

    void GameplayState::clear_prediction_runtime()
    {
        _prediction->clear_runtime();
    }

    void GameplayPredictionAdapter::clear_visible_prediction_runtime(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        _state._prediction->clear_visible_runtime(visible_subjects);
    }

#if defined(VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS)
    double GameplayPredictionAdapter::prediction_display_window_s(const PredictionSubjectKey key,
                                                                  const double now_s,
                                                                  const bool with_maneuvers) const
    {
        return PredictionWindowContextBuilder(context()).display_window_s(key, now_s, with_maneuvers);
    }

    void GameplayPredictionAdapter::refresh_prediction_preview_anchor(PredictionTrackState &track,
                                                                      const double now_s,
                                                                      const bool with_maneuvers) const
    {
        PredictionWindowContextBuilder(context()).refresh_preview_anchor(track, now_s, with_maneuvers);
    }

    double GameplayPredictionAdapter::prediction_preview_exact_window_s(const PredictionTrackState &track,
                                                                        const double now_s,
                                                                        const bool with_maneuvers) const
    {
        return PredictionWindowContextBuilder(context()).preview_exact_window_s(track, now_s, with_maneuvers);
    }

    double GameplayPredictionAdapter::prediction_planned_exact_window_s(const PredictionTrackState &track,
                                                                        const double now_s,
                                                                        const bool with_maneuvers) const
    {
        return PredictionWindowContextBuilder(context()).planned_exact_window_s(track, now_s, with_maneuvers);
    }

    double GameplayPredictionAdapter::prediction_authored_plan_request_window_s(const double now_s) const
    {
        return PredictionWindowContextBuilder(context()).authored_plan_request_window_s(now_s);
    }

    double GameplayPredictionAdapter::prediction_required_window_s(const PredictionTrackState &track,
                                                                   const double now_s,
                                                                   const bool with_maneuvers) const
    {
        return PredictionWindowContextBuilder(context()).required_window_s(track, now_s, with_maneuvers);
    }

    double GameplayPredictionAdapter::prediction_required_window_s(const PredictionSubjectKey key,
                                                                   const double now_s,
                                                                   const bool with_maneuvers) const
    {
        return PredictionWindowContextBuilder(context()).required_window_s(key, now_s, with_maneuvers);
    }

    PredictionTimeContext GameplayPredictionAdapter::build_prediction_time_context(const PredictionSubjectKey key,
                                                                                   const double sim_now_s,
                                                                                   const double trajectory_t0_s,
                                                                                   const double trajectory_t1_s) const
    {
        return PredictionWindowContextBuilder(context()).build_time_context(
                key,
                sim_now_s,
                trajectory_t0_s,
                trajectory_t1_s);
    }

    PredictionWindowPolicyResult GameplayPredictionAdapter::resolve_prediction_window_policy(
            const PredictionTrackState *track,
            const PredictionTimeContext &time_ctx,
            const bool with_maneuvers) const
    {
        return PredictionWindowContextBuilder(context()).resolve_policy(track, time_ctx, with_maneuvers);
    }
#endif

    bool GameplayPredictionAdapter::should_rebuild_prediction_track(const PredictionTrackState &track,
                                                                    const double now_s,
                                                                    const float fixed_dt,
                                                                    const bool thrusting,
                                                                    const bool with_maneuvers) const
    {
        return _state._prediction->should_rebuild_track(PredictionHostContextBuilder(context()).build(),
                                                        track,
                                                        now_s,
                                                        fixed_dt,
                                                        thrusting,
                                                        with_maneuvers);
    }

    void GameplayState::update_prediction(GameStateContext &ctx, float fixed_dt)
    {
        (void) ctx;

        // Keep frame metadata aligned before the prediction subsystem decides what to rebuild.
        GameplayPredictionAdapter prediction(*this);
        prediction.rebuild_prediction_frame_options();
        prediction.rebuild_prediction_analysis_options();
        _prediction->update(PredictionHostContextBuilder(GameplayPredictionAdapter::build_context(*this)).build(&ctx),
                            fixed_dt);
    }
} // namespace Game
