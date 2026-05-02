#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game
{
    namespace
    {
        using namespace ManeuverUtil;
    } // namespace

    double GameplayState::current_sim_time_s() const
    {
        return _orbit.scenario_owner() ? _orbit.scenario_owner()->sim.time_s() : _fixed_time_s;
    }

    void GameplayState::begin_maneuver_node_dv_edit_preview(const int node_id)
    {
        if (_maneuver.begin_dv_edit_preview(node_id))
        {
            GameplayPredictionAdapter prediction(*this);
            if (PredictionTrackState *track = prediction.active_prediction_track())
            {
                prediction.refresh_prediction_preview_anchor(*track, current_sim_time_s(), true);
            }
        }
    }

    void GameplayState::update_maneuver_node_dv_edit_preview(const int node_id)
    {
        begin_maneuver_node_dv_edit_preview(node_id);
        if (!_maneuver.mark_edit_preview_changed(ManeuverNodeEditPreview::State::EditingDv, node_id))
        {
            return;
        }

        GameplayPredictionAdapter prediction(*this);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            _prediction->mark_maneuver_preview_dirty(*track);
            prediction.sync_prediction_dirty_flag();
        }
    }

    void GameplayState::finish_maneuver_node_dv_edit_preview(const bool changed)
    {
        const bool preview_changed =
                _maneuver.finish_edit_preview(ManeuverNodeEditPreview::State::EditingDv, changed);

        if (!preview_changed)
        {
            return;
        }

        GameplayPredictionAdapter prediction(*this);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            _prediction->await_maneuver_preview_full_refine(*track, current_sim_time_s());
        }
        (void) apply_maneuver_command(ManeuverCommand::mark_plan_dirty());
    }

    void GameplayState::begin_maneuver_node_time_edit_preview(const int node_id,
                                                              const double previous_time_s)
    {
        if (_maneuver.begin_time_edit_preview(node_id, previous_time_s))
        {
            GameplayPredictionAdapter prediction(*this);
            if (PredictionTrackState *track = prediction.active_prediction_track())
            {
                prediction.refresh_prediction_preview_anchor(*track, current_sim_time_s(), true);
            }
        }
    }

    void GameplayState::update_maneuver_node_time_edit_preview(const int node_id,
                                                               const double previous_time_s)
    {
        begin_maneuver_node_time_edit_preview(node_id, previous_time_s);
        if (!_maneuver.mark_edit_preview_changed(ManeuverNodeEditPreview::State::EditingTime, node_id))
        {
            return;
        }

        GameplayPredictionAdapter prediction(*this);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            _prediction->mark_maneuver_preview_dirty(*track);
            prediction.sync_prediction_dirty_flag();
        }
    }

    void GameplayState::finish_maneuver_node_time_edit_preview(const bool changed)
    {
        const bool preview_changed =
                _maneuver.finish_edit_preview(ManeuverNodeEditPreview::State::EditingTime, changed);

        if (!preview_changed)
        {
            return;
        }

        GameplayPredictionAdapter prediction(*this);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            _prediction->await_maneuver_preview_full_refine(*track, current_sim_time_s());
        }
        (void) apply_maneuver_command(ManeuverCommand::mark_plan_dirty());
    }

    orbitsim::BodyId GameplayState::resolve_maneuver_node_primary_body_id(const ManeuverNode &node,
                                                                           const double query_time_s) const
    {
        if (!node.primary_body_auto && node.primary_body_id != orbitsim::kInvalidBodyId)
        {
            return node.primary_body_id;
        }

        GameplayPredictionAdapter prediction(*this);
        const PredictionTrackState *player_track = prediction.player_prediction_track();
        if (const OrbitPredictionCache *player_cache = prediction.effective_prediction_cache(player_track))
        {
            const OrbitPredictionCache &cache = *player_cache;
            const auto &traj =
                    cache.solver.trajectory_inertial_planned.size() >= 2
                            ? cache.solver.trajectory_inertial_planned
                            : cache.solver.resolved_trajectory_inertial();
            const auto &bodies = cache.solver.resolved_massive_bodies();

            orbitsim::State sc_state{};
            if (!traj.empty() &&
                prediction.sample_prediction_inertial_state(traj, query_time_s, sc_state) &&
                !bodies.empty())
            {
                const orbitsim::BodyId preferred_body_id =
                        node.primary_body_auto ? orbitsim::kInvalidBodyId : node.primary_body_id;
                const orbitsim::BodyId primary_body_id = prediction.select_prediction_primary_body_id(
                        bodies,
                        &cache,
                        sc_state.position_m,
                        query_time_s,
                        preferred_body_id);
                if (primary_body_id != orbitsim::kInvalidBodyId)
                {
                    return primary_body_id;
                }
            }

            if (!cache.solver.resolved_trajectory_inertial().empty())
            {
                const orbitsim::BodyId analysis_body_id =
                        prediction.resolve_prediction_analysis_body_id(cache, player_track->key, query_time_s, node.primary_body_id);
                if (analysis_body_id != orbitsim::kInvalidBodyId)
                {
                    return analysis_body_id;
                }
            }
        }

        if (node.primary_body_id != orbitsim::kInvalidBodyId)
        {
            return node.primary_body_id;
        }

        if (_orbit.scenario_owner() && _orbit.scenario_owner()->world_reference_body())
        {
            return _orbit.scenario_owner()->world_reference_body()->sim_id;
        }

        return orbitsim::kInvalidBodyId;
    }

    ManeuverCommandResult GameplayState::apply_maneuver_command(const ManeuverCommand &command)
    {
        ManeuverCommandResult result = _maneuver.apply_command(command);
        if (!result.applied)
        {
            return result;
        }

        if (result.clear_prediction_artifacts)
        {
            GameplayPredictionAdapter(*this).clear_maneuver_prediction_artifacts();
        }
        if (result.prediction_dirty)
        {
            GameplayPredictionAdapter(*this).mark_maneuver_plan_dirty();
        }

        return result;
    }

    WorldVec3 GameplayState::compute_maneuver_align_delta(GameStateContext &ctx,
                                                          const OrbitPredictionCache &cache,
                                                          const std::vector<orbitsim::TrajectorySample> &traj_base)
    {
        if (traj_base.size() < 2)
        {
            return WorldVec3(0.0, 0.0, 0.0);
        }

        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double interp_dt_s =
                (_last_sim_step_dt_s > 0.0) ? _last_sim_step_dt_s : static_cast<double>(ctx.fixed_delta_time());
        double align_now_s = current_sim_time_s();
        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            align_now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }
        align_now_s = std::clamp(align_now_s, traj_base.front().t_s, traj_base.back().t_s);

        auto it_align = std::lower_bound(traj_base.cbegin(), traj_base.cend(), align_now_s,
                                         [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
        const size_t align_hi = static_cast<size_t>(std::distance(traj_base.cbegin(), it_align));
        if (align_hi >= traj_base.size())
        {
            return WorldVec3(0.0, 0.0, 0.0);
        }

        GameplayPredictionAdapter prediction(*this);
        WorldVec3 predicted_now = (align_hi > 0)
                                      ? prediction.prediction_sample_hermite_world(cache, traj_base[align_hi - 1],
                                                                                   traj_base[align_hi], align_now_s, align_now_s)
                                      : prediction.prediction_sample_position_world(cache, traj_base.front(), align_now_s);

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (!prediction.get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
        {
            return WorldVec3(0.0, 0.0, 0.0);
        }

        const EntityId player_eid = _orbit.player_entity();
        if (const Entity *player = _world.entities().find(player_eid))
        {
            ship_pos_world = player->get_render_physics_center_of_mass_world(alpha_f);
        }

        WorldVec3 align_delta = ship_pos_world - predicted_now;
        const double align_len = safe_length(glm::dvec3(align_delta));
        if (!std::isfinite(align_len) || align_len > 10'000.0)
        {
            align_delta = WorldVec3(0.0, 0.0, 0.0);
        }
        return align_delta;
    }
} // namespace Game
