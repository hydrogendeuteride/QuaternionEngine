#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/maneuver_runtime_cache_builder.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game
{
    void GameplayState::refresh_maneuver_node_runtime_cache(GameStateContext &ctx)
    {
        const PredictionTrackState *player_track = player_prediction_track();
        const bool interaction_idle =
                _maneuver.gizmo_interaction().state != ManeuverGizmoInteraction::State::DragAxis;
        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                player_track
                    ? PredictionRuntimeDetail::describe_prediction_track_lifecycle(*player_track)
                    : PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot{};
        const bool hold_cached_release_state =
                player_track &&
                PredictionRuntimeDetail::prediction_track_should_hold_maneuver_node_cache(
                        lifecycle,
                        interaction_idle);

        const bool can_refresh_from_prediction =
                _orbitsim &&
                _prediction->state().selection.active_subject.valid() &&
                prediction_subject_is_player(_prediction->state().selection.active_subject);

        const OrbitPredictionCache *active_cache = can_refresh_from_prediction ? player_prediction_cache() : nullptr;
        const OrbitPredictionCache *stable_cache =
                can_refresh_from_prediction
                    ? ((player_track &&
                        player_track->authoritative_cache.identity.valid &&
                        player_track->authoritative_cache.has_planned_frame_draw_data())
                               ? &player_track->authoritative_cache
                               : ((player_track && player_track->cache.identity.valid)
                                          ? &player_track->cache
                                          : active_cache))
                    : nullptr;

        double display_time_s = std::numeric_limits<double>::quiet_NaN();
        WorldVec3 align_delta{0.0, 0.0, 0.0};
        PredictionFrameResolverContext frame_context{};

        if (active_cache && active_cache->identity.valid && active_cache->display.trajectory_frame.size() >= 2)
        {
            const auto &traj_base = active_cache->display.trajectory_frame;
            const double t0 = traj_base.front().t_s;
            const double t1 = traj_base.back().t_s;

            const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
            const double interp_dt_s =
                    (_last_sim_step_dt_s > 0.0) ? _last_sim_step_dt_s : static_cast<double>(ctx.fixed_delta_time());
            double now_s = _orbitsim->sim.time_s();
            if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
            {
                now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
            }

            if (std::isfinite(now_s) && t1 > t0)
            {
                display_time_s = std::clamp(now_s, t0, t1);
                align_delta = compute_maneuver_align_delta(ctx, *active_cache, traj_base);
                frame_context = build_prediction_frame_resolver_context();
            }
            else
            {
                active_cache = nullptr;
            }
        }

        const ManeuverRuntimeCacheInput input{
                .plan = _maneuver.plan(),
                .player_track = player_track,
                .active_cache = active_cache,
                .stable_cache = stable_cache,
                .frame_context = frame_context,
                .lifecycle = lifecycle,
                .gizmo_interaction = _maneuver.gizmo_interaction(),
                .edit_preview = _maneuver.edit_preview(),
                .basis_mode = _maneuver.settings().gizmo_basis_mode,
                .display_time_s = display_time_s,
                .current_sim_time_s = current_sim_time_s(),
                .align_delta = align_delta,
                .active_preview_anchor_node_id = active_maneuver_preview_anchor_node_id(),
                .hold_cached_release_state = hold_cached_release_state,
                .resolve_primary_body_id = [this](const ManeuverNode &node, const double query_time_s) {
                    return resolve_maneuver_node_primary_body_id(node, query_time_s);
                },
        };
        (void) ManeuverRuntimeCacheBuilder::rebuild(input);
    }
} // namespace Game
