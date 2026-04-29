#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"

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
        return _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
    }

    bool GameplayState::maneuver_live_preview_active(const bool with_maneuvers) const
    {
        if (!with_maneuvers || !_maneuver_plan_live_preview_active)
        {
            return false;
        }

        if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            return _maneuver_state.find_node(active_maneuver_preview_anchor_node_id()) != nullptr;
        }

        const bool node_edit_preview_active =
                _maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingDv ||
                _maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingTime;
        return node_edit_preview_active &&
               _maneuver_state.find_node(_maneuver_node_edit_preview.node_id) != nullptr;
    }

    int GameplayState::active_maneuver_preview_anchor_node_id() const
    {
        if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            if (_maneuver_state.find_node(_maneuver_gizmo_interaction.node_id))
            {
                return _maneuver_gizmo_interaction.node_id;
            }
            return _maneuver_state.selected_node_id;
        }

        if (_maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingDv ||
            _maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingTime)
        {
            return _maneuver_node_edit_preview.node_id;
        }

        return -1;
    }

    void GameplayState::begin_maneuver_node_dv_edit_preview(const int node_id)
    {
        if (!_maneuver_state.find_node(node_id))
        {
            cancel_maneuver_node_edit_preview();
            return;
        }

        if (_maneuver_node_edit_preview.state != ManeuverNodeEditPreview::State::EditingDv ||
            _maneuver_node_edit_preview.node_id != node_id)
        {
            _maneuver_node_edit_preview = {};
            _maneuver_node_edit_preview.state = ManeuverNodeEditPreview::State::EditingDv;
            _maneuver_node_edit_preview.node_id = node_id;
            if (PredictionTrackState *track = active_prediction_track())
            {
                refresh_prediction_preview_anchor(*track, current_sim_time_s(), true);
            }
        }
    }

    void GameplayState::update_maneuver_node_dv_edit_preview(const int node_id)
    {
        begin_maneuver_node_dv_edit_preview(node_id);
        if (_maneuver_node_edit_preview.state != ManeuverNodeEditPreview::State::EditingDv)
        {
            return;
        }

        _maneuver_node_edit_preview.changed = true;
        if (PredictionTrackState *track = active_prediction_track())
        {
            track->dirty = true;
            track->invalidated_while_pending = track->request_pending || track->derived_request_pending;
            sync_prediction_dirty_flag();
        }
    }

    void GameplayState::finish_maneuver_node_dv_edit_preview(const bool changed)
    {
        const bool preview_changed =
                changed ||
                (_maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingDv &&
                 _maneuver_node_edit_preview.changed);
        if (_maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingDv)
        {
            _maneuver_node_edit_preview = {};
        }

        if (!preview_changed)
        {
            return;
        }

        if (PredictionTrackState *track = active_prediction_track())
        {
            track->preview_state = PredictionPreviewRuntimeState::AwaitFullRefine;
        }
        mark_maneuver_plan_dirty();
    }

    void GameplayState::begin_maneuver_node_time_edit_preview(const int node_id,
                                                              const double previous_time_s)
    {
        const ManeuverNode *node = _maneuver_state.find_node(node_id);
        if (!node)
        {
            cancel_maneuver_node_edit_preview();
            return;
        }

        if (_maneuver_node_edit_preview.state != ManeuverNodeEditPreview::State::EditingTime ||
            _maneuver_node_edit_preview.node_id != node_id)
        {
            _maneuver_node_edit_preview = {};
            _maneuver_node_edit_preview.state = ManeuverNodeEditPreview::State::EditingTime;
            _maneuver_node_edit_preview.node_id = node_id;
            _maneuver_node_edit_preview.start_time_s =
                    std::isfinite(previous_time_s) ? previous_time_s : node->time_s;
            if (PredictionTrackState *track = active_prediction_track())
            {
                refresh_prediction_preview_anchor(*track, current_sim_time_s(), true);
            }
        }
    }

    void GameplayState::update_maneuver_node_time_edit_preview(const int node_id,
                                                               const double previous_time_s)
    {
        begin_maneuver_node_time_edit_preview(node_id, previous_time_s);
        if (_maneuver_node_edit_preview.state != ManeuverNodeEditPreview::State::EditingTime)
        {
            return;
        }

        _maneuver_node_edit_preview.changed = true;
        if (PredictionTrackState *track = active_prediction_track())
        {
            track->dirty = true;
            track->invalidated_while_pending = track->request_pending || track->derived_request_pending;
            sync_prediction_dirty_flag();
        }
    }

    void GameplayState::finish_maneuver_node_time_edit_preview(const bool changed)
    {
        const bool preview_changed =
                changed ||
                (_maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingTime &&
                 _maneuver_node_edit_preview.changed);
        if (_maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingTime)
        {
            _maneuver_node_edit_preview = {};
        }

        if (!preview_changed)
        {
            return;
        }

        if (PredictionTrackState *track = active_prediction_track())
        {
            track->preview_state = PredictionPreviewRuntimeState::AwaitFullRefine;
        }
        mark_maneuver_plan_dirty();
    }

    void GameplayState::cancel_maneuver_node_edit_preview()
    {
        _maneuver_node_edit_preview = {};
    }

    void GameplayState::cancel_maneuver_node_dv_edit_preview()
    {
        cancel_maneuver_node_edit_preview();
    }

    orbitsim::BodyId GameplayState::resolve_maneuver_node_primary_body_id(const ManeuverNode &node,
                                                                          const double query_time_s) const
    {
        if (!node.primary_body_auto && node.primary_body_id != orbitsim::kInvalidBodyId)
        {
            return node.primary_body_id;
        }

        const PredictionTrackState *player_track = player_prediction_track();
        if (const OrbitPredictionCache *player_cache = effective_prediction_cache(player_track))
        {
            const OrbitPredictionCache &cache = *player_cache;
            const auto &traj =
                    cache.solver.trajectory_inertial_planned.size() >= 2
                            ? cache.solver.trajectory_inertial_planned
                            : cache.solver.resolved_trajectory_inertial();
            const auto &bodies = cache.solver.resolved_massive_bodies();

            orbitsim::State sc_state{};
            if (!traj.empty() &&
                sample_prediction_inertial_state(traj, query_time_s, sc_state) &&
                !bodies.empty())
            {
                const orbitsim::BodyId preferred_body_id =
                        node.primary_body_auto ? orbitsim::kInvalidBodyId : node.primary_body_id;
                const orbitsim::BodyId primary_body_id = select_prediction_primary_body_id(
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
                        resolve_prediction_analysis_body_id(cache, player_track->key, query_time_s, node.primary_body_id);
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

        if (_orbitsim && _orbitsim->world_reference_body())
        {
            return _orbitsim->world_reference_body()->sim_id;
        }

        return orbitsim::kInvalidBodyId;
    }

    void GameplayState::finalize_maneuver_node_removal(const bool removed_selected,
                                                        const bool removed_gizmo,
                                                        const bool removed_execute,
                                                        const int hint_index)
    {
        if (removed_selected)
        {
            if (_maneuver_state.nodes.empty())
            {
                _maneuver_state.selected_node_id = -1;
            }
            else if (hint_index >= 0)
            {
                const int new_idx = std::clamp(hint_index, 0, static_cast<int>(_maneuver_state.nodes.size()) - 1);
                _maneuver_state.selected_node_id = _maneuver_state.nodes[static_cast<size_t>(new_idx)].id;
            }
            else
            {
                _maneuver_state.selected_node_id = _maneuver_state.nodes.front().id;
            }
        }

        if (removed_gizmo)
        {
            _maneuver_gizmo_interaction = {};
        }
        if (removed_execute)
        {
            _execute_node_armed = false;
            _execute_node_id = -1;
        }
        if (_maneuver_state.nodes.empty())
        {
            clear_maneuver_prediction_artifacts();
        }

        mark_maneuver_plan_dirty();
    }

    void GameplayState::remove_maneuver_node(const int node_id, const int hint_index)
    {
        const bool removed_selected = (_maneuver_state.selected_node_id == node_id);
        const bool removed_gizmo = (_maneuver_gizmo_interaction.node_id == node_id);
        const bool removed_execute = _execute_node_armed && (_execute_node_id == node_id);
        const bool removed_node_edit_preview = _maneuver_node_edit_preview.node_id == node_id;

        _maneuver_state.nodes.erase(
            std::remove_if(_maneuver_state.nodes.begin(),
                           _maneuver_state.nodes.end(),
                           [&](const ManeuverNode &n) { return n.id == node_id; }),
            _maneuver_state.nodes.end());

        if (removed_node_edit_preview)
        {
            cancel_maneuver_node_dv_edit_preview();
        }
        finalize_maneuver_node_removal(removed_selected, removed_gizmo, removed_execute, hint_index);
    }

    void GameplayState::remove_maneuver_node_suffix(const int node_id, const int hint_index)
    {
        const auto erase_begin = std::find_if(_maneuver_state.nodes.begin(),
                                              _maneuver_state.nodes.end(),
                                              [node_id](const ManeuverNode &n) { return n.id == node_id; });
        if (erase_begin == _maneuver_state.nodes.end())
        {
            return;
        }

        const auto removes_node_id = [erase_begin, this](const int candidate_id) {
            return std::any_of(erase_begin,
                               _maneuver_state.nodes.end(),
                               [candidate_id](const ManeuverNode &n) { return n.id == candidate_id; });
        };

        const bool removed_selected = removes_node_id(_maneuver_state.selected_node_id);
        const bool removed_gizmo = removes_node_id(_maneuver_gizmo_interaction.node_id);
        const bool removed_execute = _execute_node_armed && removes_node_id(_execute_node_id);
        const bool removed_node_edit_preview = removes_node_id(_maneuver_node_edit_preview.node_id);
        _maneuver_state.nodes.erase(erase_begin, _maneuver_state.nodes.end());

        if (removed_node_edit_preview)
        {
            cancel_maneuver_node_dv_edit_preview();
        }
        finalize_maneuver_node_removal(removed_selected, removed_gizmo, removed_execute, hint_index);
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

        WorldVec3 predicted_now = (align_hi > 0)
                                      ? prediction_sample_hermite_world(cache, traj_base[align_hi - 1],
                                                                        traj_base[align_hi], align_now_s, align_now_s)
                                      : prediction_sample_position_world(cache, traj_base.front(), align_now_s);

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (!get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
        {
            return WorldVec3(0.0, 0.0, 0.0);
        }

        const EntityId player_eid = player_entity();
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
