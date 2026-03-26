#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"

#include "game/orbit/orbit_prediction_math.h"
#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace Game
{
    namespace
    {
        using OrbitPredictionMath::safe_length;
        using namespace ManeuverUtil;

        bool finite3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        glm::dvec3 normalized_or(const glm::dvec3 &v, const glm::dvec3 &fallback)
        {
            const double len = OrbitPredictionMath::safe_length(v);
            if (!(len > 0.0) || !std::isfinite(len))
            {
                return fallback;
            }
            return v / len;
        }

        // Maneuver-node frame uses true RTN so editing, execution, and prediction all share one orthogonal basis:
        // - R: radial, away from the primary
        // - T: tangential / along-track in the orbital plane
        // - N: angular-momentum normal
        orbitsim::RtnFrame compute_maneuver_frame(const glm::dvec3 &r_rel_m,
                                                  const glm::dvec3 &v_rel_mps)
        {
            return orbitsim::compute_rtn_frame(orbitsim::Vec3{r_rel_m.x, r_rel_m.y, r_rel_m.z},
                                               orbitsim::Vec3{v_rel_mps.x, v_rel_mps.y, v_rel_mps.z});
        }

        double clamp_sane(double x, double lo, double hi, double fallback = 0.0)
        {
            if (!std::isfinite(x))
            {
                return fallback;
            }
            return std::clamp(x, lo, hi);
        }

        WorldVec3 hermite_position_world(const WorldVec3 &ref_body_world,
                                         const orbitsim::TrajectorySample &a,
                                         const orbitsim::TrajectorySample &b,
                                         const double t_s)
        {
            // Use sample velocities as tangents so node markers stay visually smooth on the orbit arc.
            const double ta = a.t_s;
            const double tb = b.t_s;
            const double h = tb - ta;
            if (!std::isfinite(h) || !(h > 0.0))
            {
                return ref_body_world + WorldVec3(a.position_m);
            }

            double u = (t_s - ta) / h;
            if (!std::isfinite(u))
            {
                u = 0.0;
            }
            u = std::clamp(u, 0.0, 1.0);

            const double u2 = u * u;
            const double u3 = u2 * u;

            const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
            const double h10 = u3 - (2.0 * u2) + u;
            const double h01 = (-2.0 * u3) + (3.0 * u2);
            const double h11 = u3 - u2;

            const glm::dvec3 p0 = glm::dvec3(a.position_m);
            const glm::dvec3 p1 = glm::dvec3(b.position_m);
            const glm::dvec3 m0 = glm::dvec3(a.velocity_mps) * h;
            const glm::dvec3 m1 = glm::dvec3(b.velocity_mps) * h;

            const glm::dvec3 p = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
            return ref_body_world + WorldVec3(p);
        }

        struct TrajectorySampledState
        {
            bool valid{false};
            glm::dvec3 r_rel_m{0.0, 0.0, 0.0};
            glm::dvec3 v_rel_mps{0.0, 0.0, 0.0};
            WorldVec3 position_world{0.0, 0.0, 0.0};
        };

        TrajectorySampledState sample_trajectory_state(const std::vector<orbitsim::TrajectorySample> &traj_bci,
                                                       const WorldVec3 &ref_body_world,
                                                       const double t_s)
        {
            TrajectorySampledState out{};
            if (traj_bci.size() < 2)
            {
                return out;
            }

            const double t0 = traj_bci.front().t_s;
            const double t1 = traj_bci.back().t_s;
            if (!(t1 > t0) || !std::isfinite(t_s))
            {
                return out;
            }

            const double t_clamped = std::clamp(t_s, t0, t1);

            auto it_hi = std::lower_bound(traj_bci.cbegin(),
                                          traj_bci.cend(),
                                          t_clamped,
                                          [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
            size_t i_hi = static_cast<size_t>(std::distance(traj_bci.cbegin(), it_hi));
            if (i_hi >= traj_bci.size())
            {
                return out;
            }

            size_t i_lo = (i_hi == 0) ? 0 : (i_hi - 1);
            const orbitsim::TrajectorySample &a = traj_bci[i_lo];
            const orbitsim::TrajectorySample &b = traj_bci[i_hi];

            const double ta = a.t_s;
            const double tb = b.t_s;
            const double h = tb - ta;

            double u = 0.0;
            if (std::isfinite(h) && h > 1e-9)
            {
                u = (t_clamped - ta) / h;
            }
            u = clamp_sane(u, 0.0, 1.0, 0.0);

            // Position: hermite for smoother marker placement.
            out.position_world = hermite_position_world(ref_body_world, a, b, t_clamped);

            // Basis inputs only need to be locally consistent; linear interpolation is stable enough here.
            out.r_rel_m = glm::mix(glm::dvec3(a.position_m), glm::dvec3(b.position_m), u);
            out.v_rel_mps = glm::mix(glm::dvec3(a.velocity_mps), glm::dvec3(b.velocity_mps), u);
            out.valid = finite3(out.r_rel_m) && finite3(out.v_rel_mps);
            return out;
        }
    } // namespace

    double GameplayState::current_sim_time_s() const
    {
        return _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
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
                    cache.trajectory_inertial_planned.size() >= 2 ? cache.trajectory_inertial_planned
                                                                  : cache.trajectory_inertial;

            orbitsim::State sc_state{};
            if (!traj.empty() &&
                sample_prediction_inertial_state(traj, query_time_s, sc_state) &&
                !cache.massive_bodies.empty())
            {
                const orbitsim::BodyId preferred_body_id =
                        node.primary_body_id != orbitsim::kInvalidBodyId ? node.primary_body_id : player_track->auto_primary_body_id;
                const orbitsim::BodyId primary_body_id = select_prediction_primary_body_id(
                        cache.massive_bodies,
                        &cache,
                        sc_state.position_m,
                        query_time_s,
                        preferred_body_id);
                if (primary_body_id != orbitsim::kInvalidBodyId)
                {
                    return primary_body_id;
                }
            }

            if (!cache.trajectory_inertial.empty())
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

    void GameplayState::remove_maneuver_node(const int node_id, const int hint_index)
    {
        _maneuver_state.nodes.erase(
            std::remove_if(_maneuver_state.nodes.begin(),
                           _maneuver_state.nodes.end(),
                           [&](const ManeuverNode &n) { return n.id == node_id; }),
            _maneuver_state.nodes.end());

        if (_maneuver_state.selected_node_id == node_id)
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

        if (_maneuver_gizmo_interaction.node_id == node_id)
        {
            _maneuver_gizmo_interaction = {};
        }
        if (_execute_node_armed && _execute_node_id == node_id)
        {
            _execute_node_armed = false;
            _execute_node_id = -1;
        }

        mark_maneuver_plan_dirty();
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
        _maneuver_state.nodes.erase(erase_begin, _maneuver_state.nodes.end());

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

        mark_maneuver_plan_dirty();
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
            ship_pos_world = player->get_render_position_world(alpha_f);
        }

        WorldVec3 align_delta = ship_pos_world - predicted_now;
        const double align_len = safe_length(glm::dvec3(align_delta));
        if (!std::isfinite(align_len) || align_len > 10'000.0)
        {
            align_delta = WorldVec3(0.0, 0.0, 0.0);
        }
        return align_delta;
    }

    void GameplayState::refresh_maneuver_node_runtime_cache(GameStateContext &ctx)
    {
        // Rebuild all derived node state so UI/editor code can work from the latest prediction and render interpolation.
        for (ManeuverNode &node : _maneuver_state.nodes)
        {
            node.total_dv_mps = safe_length(node.dv_rtn_mps);
            node.gizmo_valid = false;
        }

        if (!_orbitsim || !_prediction_selection.active_subject.valid() ||
            !prediction_subject_is_player(_prediction_selection.active_subject))
        {
            return;
        }

        const PredictionTrackState *player_track = player_prediction_track();
        const OrbitPredictionCache *effective_prediction_cache = player_prediction_cache();
        const OrbitPredictionCache *stable_prediction_cache =
                (player_track && player_track->cache.valid) ? &player_track->cache : effective_prediction_cache;
        const bool suppress_stale_preview =
                player_track &&
                _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis &&
                (player_track->preview_state == PredictionPreviewRuntimeState::EnterDrag ||
                 player_track->preview_state == PredictionPreviewRuntimeState::DragPreviewPending);
        const OrbitPredictionCache *prediction_cache =
                (suppress_stale_preview && stable_prediction_cache) ? stable_prediction_cache : effective_prediction_cache;
        if (!prediction_cache || !prediction_cache->valid || prediction_cache->trajectory_frame.size() < 2)
        {
            return;
        }

        const bool interaction_idle =
                _maneuver_gizmo_interaction.state != ManeuverGizmoInteraction::State::DragAxis;
        const bool hold_cached_release_state =
                player_track &&
                interaction_idle &&
                (player_track->preview_state == PredictionPreviewRuntimeState::DragPreviewPending ||
                 player_track->preview_state == PredictionPreviewRuntimeState::PreviewStreaming ||
                 player_track->preview_state == PredictionPreviewRuntimeState::AwaitFullRefine ||
                 player_track->request_pending ||
                 player_track->derived_request_pending ||
                 _maneuver_plan_live_preview_active);
        const PredictionChunkAssembly *preview_chunk_assembly =
                (!suppress_stale_preview &&
                 player_track &&
                 player_track->preview_overlay.chunk_assembly.valid &&
                 !player_track->preview_overlay.chunk_assembly.chunks.empty())
                        ? &player_track->preview_overlay.chunk_assembly
                        : nullptr;

        const auto &traj_base = prediction_cache->trajectory_frame;
        const auto &traj_planned = prediction_cache->trajectory_frame_planned;
        const double t0 = traj_base.front().t_s;
        const double t1 = traj_base.back().t_s;

        // Match orbit plot alignment logic so maneuver gizmos sit on the displayed curve.
        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double interp_dt_s =
                (_last_sim_step_dt_s > 0.0) ? _last_sim_step_dt_s : static_cast<double>(ctx.fixed_delta_time());
        double now_s = _orbitsim->sim.time_s();
        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }
        if (!std::isfinite(now_s) || !(t1 > t0))
        {
            return;
        }
        now_s = std::clamp(now_s, t0, t1);

        const WorldVec3 align_delta = compute_maneuver_align_delta(ctx, *prediction_cache, traj_base);

        const auto &traj_node_world = traj_planned.size() >= 2 ? traj_planned : traj_base;
        const auto &traj_node_inertial =
                prediction_cache->trajectory_inertial_planned.size() >= 2
                    ? prediction_cache->trajectory_inertial_planned
                    : prediction_cache->trajectory_inertial;
        const double traj_node_t0 = traj_node_world.front().t_s;
        const double traj_node_t1 = traj_node_world.back().t_s;
        std::unordered_map<int, const OrbitPredictionCache::ManeuverNodePreview *> preview_by_node_id;
        preview_by_node_id.reserve(prediction_cache->maneuver_previews.size());
        for (const OrbitPredictionCache::ManeuverNodePreview &preview : prediction_cache->maneuver_previews)
        {
            preview_by_node_id[preview.node_id] = &preview;
        }
        const orbitsim::TrajectoryFrameSpec display_frame_spec =
                prediction_cache->resolved_frame_spec_valid
                    ? prediction_cache->resolved_frame_spec
                    : resolve_prediction_display_frame_spec(*prediction_cache, now_s);
        const bool display_frame_uses_preview_anchor =
                display_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial ||
                display_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial;
        const bool drag_display_snapshots_available =
                _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis &&
                !_maneuver_gizmo_interaction.drag_display_snapshots.empty();
        const bool freeze_nonrotating_drag_snapshots =
                display_frame_uses_preview_anchor &&
                drag_display_snapshots_available;
        WorldVec3 display_origin_world_now{0.0, 0.0, 0.0};
        glm::dmat3 display_frame_to_world_now{1.0};
        const bool have_display_transform_now =
                !display_frame_uses_preview_anchor ||
                build_prediction_display_transform(*prediction_cache,
                                                   display_origin_world_now,
                                                   display_frame_to_world_now,
                                                   now_s);
        const auto transform_inertial_basis_to_world =
                [&](const glm::dvec3 &basis_inertial, const glm::dvec3 &fallback) -> glm::dvec3 {
                    // Keep maneuver axes in physical inertial/world orientation. If we remap them
                    // through the active display frame, Synodic rotation feeds back into RTN/PON
                    // and the gizmo visibly twists while editing.
                    return normalized_or(basis_inertial, fallback);
                };

        const auto sample_displayed_node_world = [&](const double sample_time_s, WorldVec3 &out_world) -> bool {
            out_world = WorldVec3(0.0, 0.0, 0.0);
            if (traj_node_world.size() < 2 || !std::isfinite(sample_time_s))
            {
                return false;
            }

            if (sample_time_s < traj_node_t0 || sample_time_s > traj_node_t1)
            {
                return false;
            }

            auto it_node_hi = std::lower_bound(traj_node_world.cbegin(),
                                               traj_node_world.cend(),
                                               sample_time_s,
                                               [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
            const size_t node_hi = static_cast<size_t>(std::distance(traj_node_world.cbegin(), it_node_hi));
            if (node_hi >= traj_node_world.size())
            {
                return false;
            }

            out_world = (node_hi > 0)
                            ? prediction_sample_hermite_world(*prediction_cache,
                                                              traj_node_world[node_hi - 1],
                                                              traj_node_world[node_hi],
                                                              sample_time_s,
                                                              now_s)
                            : prediction_sample_position_world(*prediction_cache, traj_node_world.front(), now_s);
            out_world += align_delta;
            return finite3(glm::dvec3(out_world));
        };

        const auto sample_preview_chunk_node_world = [&](const double sample_time_s, WorldVec3 &out_world) -> bool {
            out_world = WorldVec3(0.0, 0.0, 0.0);
            if (!preview_chunk_assembly || !std::isfinite(sample_time_s))
            {
                return false;
            }

            for (const OrbitChunk &chunk : preview_chunk_assembly->chunks)
            {
                if (!chunk.valid || chunk.frame_samples.size() < 2 ||
                    sample_time_s < chunk.t0_s || sample_time_s > chunk.t1_s)
                {
                    continue;
                }

                auto it_hi = std::lower_bound(chunk.frame_samples.cbegin(),
                                              chunk.frame_samples.cend(),
                                              sample_time_s,
                                              [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
                const size_t i_hi = static_cast<size_t>(std::distance(chunk.frame_samples.cbegin(), it_hi));
                if (i_hi >= chunk.frame_samples.size())
                {
                    continue;
                }

                out_world = (i_hi > 0)
                                ? prediction_sample_hermite_world(*prediction_cache,
                                                                  chunk.frame_samples[i_hi - 1],
                                                                  chunk.frame_samples[i_hi],
                                                                  sample_time_s,
                                                                  now_s)
                                : prediction_sample_position_world(*prediction_cache,
                                                                    chunk.frame_samples.front(),
                                                                    now_s);
                out_world += align_delta;
                return finite3(glm::dvec3(out_world));
            }

            return false;
        };

        const auto sample_displayed_tangent_world = [&](const double sample_time_s, glm::dvec3 &out_tangent_world) -> bool {
            out_tangent_world = glm::dvec3(0.0, 0.0, 0.0);
            if (traj_node_world.size() < 2 || !std::isfinite(sample_time_s))
            {
                return false;
            }

            const double backward_dt_s = std::min(0.25, std::max(0.0, sample_time_s - traj_node_t0));
            const double forward_dt_s = std::min(0.25, std::max(0.0, traj_node_t1 - sample_time_s));

            WorldVec3 p0_world{0.0, 0.0, 0.0};
            WorldVec3 p1_world{0.0, 0.0, 0.0};
            if (backward_dt_s >= 1.0e-4)
            {
                if (!sample_displayed_node_world(sample_time_s - backward_dt_s, p0_world) ||
                    !sample_displayed_node_world(sample_time_s, p1_world))
                {
                    return false;
                }
            }
            else if (forward_dt_s >= 1.0e-4)
            {
                if (!sample_displayed_node_world(sample_time_s, p0_world) ||
                    !sample_displayed_node_world(sample_time_s + forward_dt_s, p1_world))
                {
                    return false;
                }
            }
            else
            {
                return false;
            }

            out_tangent_world = normalized_or(glm::dvec3(p1_world - p0_world), glm::dvec3(0.0, 1.0, 0.0));
            return finite3(out_tangent_world);
        };

        const auto sample_preview_chunk_tangent_world =
                [&](const double sample_time_s, glm::dvec3 &out_tangent_world) -> bool {
                    out_tangent_world = glm::dvec3(0.0, 0.0, 0.0);
                    if (!preview_chunk_assembly || !std::isfinite(sample_time_s))
                    {
                        return false;
                    }

                    const double backward_dt_s = std::min(0.25, std::max(0.0, sample_time_s - traj_node_t0));
                    const double forward_dt_s = std::min(0.25, std::max(0.0, traj_node_t1 - sample_time_s));

                    WorldVec3 p0_world{0.0, 0.0, 0.0};
                    WorldVec3 p1_world{0.0, 0.0, 0.0};
                    if (backward_dt_s >= 1.0e-4)
                    {
                        if (!sample_preview_chunk_node_world(sample_time_s - backward_dt_s, p0_world) ||
                            !sample_preview_chunk_node_world(sample_time_s, p1_world))
                        {
                            return false;
                        }
                    }
                    else if (forward_dt_s >= 1.0e-4)
                    {
                        if (!sample_preview_chunk_node_world(sample_time_s, p0_world) ||
                            !sample_preview_chunk_node_world(sample_time_s + forward_dt_s, p1_world))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        return false;
                    }

                    out_tangent_world = normalized_or(glm::dvec3(p1_world - p0_world), glm::dvec3(0.0, 1.0, 0.0));
                    return finite3(out_tangent_world);
                };

        const auto display_frame_origin_state_at = [&](const double sample_time_s, orbitsim::State &out_state) -> bool {
            out_state = {};
            switch (display_frame_spec.type)
            {
                case orbitsim::TrajectoryFrameType::Inertial:
                    return true;
                case orbitsim::TrajectoryFrameType::BodyCenteredInertial:
                {
                    const orbitsim::MassiveBody *frame_body =
                            find_massive_body(prediction_cache->massive_bodies, display_frame_spec.primary_body_id);
                    if (!frame_body)
                    {
                        return false;
                    }

                    out_state =
                            (prediction_cache->shared_ephemeris && !prediction_cache->shared_ephemeris->empty())
                                ? prediction_cache->shared_ephemeris->body_state_at_by_id(frame_body->id, sample_time_s)
                                : frame_body->state;
                    return finite3(glm::dvec3(out_state.position_m)) && finite3(glm::dvec3(out_state.velocity_mps));
                }
                default:
                    return false;
            }
        };

        const auto sample_preview_node_world = [&](const double sample_time_s,
                                                   const glm::dvec3 &preview_position_m,
                                                   WorldVec3 &out_world) -> bool {
            out_world = WorldVec3(0.0, 0.0, 0.0);
            if (!display_frame_uses_preview_anchor || !have_display_transform_now || !std::isfinite(sample_time_s) ||
                !finite3(preview_position_m))
            {
                return false;
            }

            orbitsim::State frame_origin_state{};
            if (!display_frame_origin_state_at(sample_time_s, frame_origin_state))
            {
                return false;
            }

            const glm::dvec3 local_position_m = preview_position_m - glm::dvec3(frame_origin_state.position_m);
            out_world = display_origin_world_now + WorldVec3(display_frame_to_world_now * local_position_m) + align_delta;
            return finite3(glm::dvec3(out_world));
        };

        const auto sample_preview_tangent_world = [&](const double sample_time_s,
                                                      const glm::dvec3 &preview_velocity_mps,
                                                      glm::dvec3 &out_tangent_world) -> bool {
            out_tangent_world = glm::dvec3(0.0, 0.0, 0.0);
            if (!display_frame_uses_preview_anchor || !have_display_transform_now || !std::isfinite(sample_time_s) ||
                !finite3(preview_velocity_mps))
            {
                return false;
            }

            orbitsim::State frame_origin_state{};
            if (!display_frame_origin_state_at(sample_time_s, frame_origin_state))
            {
                return false;
            }

            const glm::dvec3 local_velocity_mps = preview_velocity_mps - glm::dvec3(frame_origin_state.velocity_mps);
            out_tangent_world = normalized_or(display_frame_to_world_now * local_velocity_mps, glm::dvec3(0.0, 1.0, 0.0));
            return finite3(out_tangent_world);
        };

        const auto resolve_node_primary_body_id =
                [&](const ManeuverNode &node, const double sample_time_s, const glm::dvec3 &inertial_position_m)
                -> orbitsim::BodyId {
                    const orbitsim::BodyId preferred_body_id =
                            resolve_maneuver_node_primary_body_id(node, sample_time_s);
                    if (preferred_body_id != orbitsim::kInvalidBodyId)
                    {
                        return preferred_body_id;
                    }
                    if (prediction_cache->massive_bodies.empty() || !std::isfinite(sample_time_s) || !finite3(inertial_position_m))
                    {
                        return orbitsim::kInvalidBodyId;
                    }

                    const auto body_position_at = [&](const std::size_t i) -> orbitsim::Vec3 {
                        const orbitsim::MassiveBody &body = prediction_cache->massive_bodies[i];
                        if (prediction_cache->shared_ephemeris && !prediction_cache->shared_ephemeris->empty())
                        {
                            return prediction_cache->shared_ephemeris->body_state_at_by_id(body.id, sample_time_s).position_m;
                        }
                        return body.state.position_m;
                    };

                    const std::size_t primary_index = orbitsim::auto_select_primary_index(
                            prediction_cache->massive_bodies,
                            orbitsim::Vec3{inertial_position_m.x, inertial_position_m.y, inertial_position_m.z},
                            body_position_at,
                            _orbitsim ? _orbitsim->sim.config().softening_length_m : 0.0);
                    if (primary_index >= prediction_cache->massive_bodies.size())
                    {
                        return orbitsim::kInvalidBodyId;
                    }
                    return prediction_cache->massive_bodies[primary_index].id;
                };
        const auto find_drag_display_snapshot = [&](const int node_id) -> const ManeuverNodeDisplaySnapshot * {
            for (const ManeuverNodeDisplaySnapshot &snapshot : _maneuver_gizmo_interaction.drag_display_snapshots)
            {
                if (snapshot.node_id == node_id)
                {
                    return &snapshot;
                }
            }
            return nullptr;
        };

        for (ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!std::isfinite(node.time_s) || node.time_s < t0 || node.time_s > t1)
            {
                continue;
            }

            const bool active_drag_node =
                    _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis &&
                    _maneuver_gizmo_interaction.node_id == node.id;
            const ManeuverNodeDisplaySnapshot *cached_drag_snapshot =
                    drag_display_snapshots_available ? find_drag_display_snapshot(node.id) : nullptr;
            const bool cached_node_state_valid =
                    finite3(glm::dvec3(node.position_world)) &&
                    finite3(node.basis_r_world) &&
                    finite3(node.basis_t_world) &&
                    finite3(node.basis_n_world) &&
                    finite3(node.maneuver_basis_r_world) &&
                    finite3(node.maneuver_basis_t_world) &&
                    finite3(node.maneuver_basis_n_world);
            const bool can_fallback_to_cached_drag_state =
                    active_drag_node &&
                    cached_drag_snapshot != nullptr;
            const bool can_fallback_to_cached_release_state =
                    hold_cached_release_state &&
                    cached_node_state_valid;
            const glm::dvec3 cached_basis_r_world = node.basis_r_world;
            const glm::dvec3 cached_basis_t_world = node.basis_t_world;
            const glm::dvec3 cached_basis_n_world = node.basis_n_world;
            const glm::dvec3 cached_maneuver_basis_r_world = node.maneuver_basis_r_world;
            const glm::dvec3 cached_maneuver_basis_t_world = node.maneuver_basis_t_world;
            const glm::dvec3 cached_maneuver_basis_n_world = node.maneuver_basis_n_world;

            const auto preview_it = preview_by_node_id.find(node.id);
            const OrbitPredictionCache::ManeuverNodePreview *preview =
                    (preview_it != preview_by_node_id.end()) ? preview_it->second : nullptr;

            glm::dvec3 r_rel_m{0.0, 0.0, 0.0};
            glm::dvec3 v_rel_mps{0.0, 0.0, 0.0};
            WorldVec3 node_position_world{0.0, 0.0, 0.0};
            bool have_runtime_state = false;
            bool use_cached_runtime_basis = false;
            double basis_time_s = node.time_s;
            if (basis_time_s > traj_node_t0)
            {
                basis_time_s = std::max(traj_node_t0, basis_time_s - 1e-3);
            }

            const bool have_preview_chunk_position_world = sample_preview_chunk_node_world(node.time_s, node_position_world);
            const bool have_preview_position_world =
                    !have_preview_chunk_position_world &&
                    preview && preview->valid &&
                    sample_preview_node_world(node.time_s, glm::dvec3(preview->inertial_position_m), node_position_world);
            if (!have_preview_chunk_position_world &&
                !have_preview_position_world &&
                !sample_displayed_node_world(node.time_s, node_position_world))
            {
                if (can_fallback_to_cached_drag_state)
                {
                    node_position_world = cached_drag_snapshot->position_world;
                }
                else if (can_fallback_to_cached_release_state)
                {
                    node_position_world = node.position_world;
                }
                else
                {
                    continue;
                }
            }

            if (preview && preview->valid)
            {
                const glm::dvec3 preview_position_m = glm::dvec3(preview->inertial_position_m);
                const orbitsim::BodyId primary_body_id =
                        resolve_node_primary_body_id(node, node.time_s, preview_position_m);
                const orbitsim::MassiveBody *primary_body = find_massive_body(prediction_cache->massive_bodies, primary_body_id);
                if (primary_body)
                {
                    const orbitsim::State primary_state =
                            (prediction_cache->shared_ephemeris && !prediction_cache->shared_ephemeris->empty())
                                ? prediction_cache->shared_ephemeris->body_state_at_by_id(primary_body_id, node.time_s)
                                : primary_body->state;
                    r_rel_m = preview_position_m - glm::dvec3(primary_state.position_m);
                    v_rel_mps = glm::dvec3(preview->inertial_velocity_mps) - glm::dvec3(primary_state.velocity_mps);
                    have_runtime_state = finite3(r_rel_m) && finite3(v_rel_mps);
                }
            }
            else
            {
                // Fallback when previews are unavailable: sample the plotted trajectory directly.
                if (traj_node_inertial.size() < 2)
                {
                    continue;
                }

                const TrajectorySampledState inertial_state =
                        sample_trajectory_state(traj_node_inertial, WorldVec3(0.0), basis_time_s);
                if (!inertial_state.valid)
                {
                    continue;
                }

                const orbitsim::BodyId primary_body_id =
                        resolve_node_primary_body_id(node, basis_time_s, inertial_state.r_rel_m);
                const orbitsim::MassiveBody *primary_body = find_massive_body(prediction_cache->massive_bodies, primary_body_id);
                if (!primary_body)
                {
                    continue;
                }

                const orbitsim::State primary_state =
                        (prediction_cache->shared_ephemeris && !prediction_cache->shared_ephemeris->empty())
                            ? prediction_cache->shared_ephemeris->body_state_at_by_id(primary_body_id, basis_time_s)
                            : primary_body->state;

                r_rel_m = inertial_state.r_rel_m - glm::dvec3(primary_state.position_m);
                v_rel_mps = inertial_state.v_rel_mps - glm::dvec3(primary_state.velocity_mps);
                have_runtime_state = finite3(r_rel_m) && finite3(v_rel_mps);
            }

            if (!have_runtime_state)
            {
                const bool cached_runtime_basis_valid =
                        finite3(cached_maneuver_basis_r_world) &&
                        finite3(cached_maneuver_basis_t_world) &&
                        finite3(cached_maneuver_basis_n_world);
                if (!(can_fallback_to_cached_drag_state || can_fallback_to_cached_release_state) ||
                    !cached_runtime_basis_valid)
                {
                    continue;
                }
                use_cached_runtime_basis = true;
            }

            if (use_cached_runtime_basis)
            {
                node.maneuver_basis_r_world = cached_maneuver_basis_r_world;
                node.maneuver_basis_t_world = cached_maneuver_basis_t_world;
                node.maneuver_basis_n_world = cached_maneuver_basis_n_world;
                node.basis_r_world = cached_basis_r_world;
                node.basis_t_world = cached_basis_t_world;
                node.basis_n_world = cached_basis_n_world;
            }
            else
            {
                const orbitsim::RtnFrame solver_frame = compute_maneuver_frame(r_rel_m, v_rel_mps);
                // Cache the authored true-RTN basis once and keep the visible/editable handles on that same basis.
                node.maneuver_basis_r_world = transform_inertial_basis_to_world(glm::dvec3(solver_frame.R.x,
                                                                                            solver_frame.R.y,
                                                                                            solver_frame.R.z),
                                                                                glm::dvec3(1.0, 0.0, 0.0));
                node.maneuver_basis_t_world = transform_inertial_basis_to_world(glm::dvec3(solver_frame.T.x,
                                                                                            solver_frame.T.y,
                                                                                            solver_frame.T.z),
                                                                                glm::dvec3(0.0, 1.0, 0.0));
                node.maneuver_basis_n_world = transform_inertial_basis_to_world(glm::dvec3(solver_frame.N.x,
                                                                                            solver_frame.N.y,
                                                                                            solver_frame.N.z),
                                                                                glm::dvec3(0.0, 0.0, 1.0));

                if (_maneuver_gizmo_basis_mode == ManeuverGizmoBasisMode::RTN)
                {
                    node.basis_r_world = node.maneuver_basis_r_world;
                    node.basis_t_world = node.maneuver_basis_t_world;
                    node.basis_n_world = node.maneuver_basis_n_world;
                }
                else
                {
                    const glm::dvec3 fallback_rtn_t(solver_frame.T.x, solver_frame.T.y, solver_frame.T.z);
                    glm::dvec3 prograde_inertial = normalized_or(v_rel_mps, fallback_rtn_t);
                    glm::dvec3 normal_inertial = normalized_or(glm::dvec3(solver_frame.N.x, solver_frame.N.y, solver_frame.N.z),
                                                               glm::dvec3(0.0, 0.0, 1.0));
                    glm::dvec3 prograde_world_fallback =
                            transform_inertial_basis_to_world(prograde_inertial, node.maneuver_basis_t_world);
                    glm::dvec3 preview_chunk_tangent_world{0.0, 0.0, 0.0};
                    glm::dvec3 preview_tangent_world{0.0, 0.0, 0.0};
                    glm::dvec3 displayed_tangent_world{0.0, 0.0, 0.0};
                    if (sample_preview_chunk_tangent_world(node.time_s, preview_chunk_tangent_world))
                    {
                        prograde_world_fallback = preview_chunk_tangent_world;
                    }
                    else if (preview && preview->valid &&
                             sample_preview_tangent_world(node.time_s,
                                                          glm::dvec3(preview->inertial_velocity_mps),
                                                          preview_tangent_world))
                    {
                        prograde_world_fallback = preview_tangent_world;
                    }
                    else if (sample_displayed_tangent_world(basis_time_s, displayed_tangent_world))
                    {
                        prograde_world_fallback = displayed_tangent_world;
                    }
                    const glm::dvec3 normal_world_fallback =
                            transform_inertial_basis_to_world(normal_inertial, node.maneuver_basis_n_world);
                    glm::dvec3 prograde_world = prograde_world_fallback;
                    glm::dvec3 normal_world = normalized_or(normal_world_fallback, node.maneuver_basis_n_world);
                    glm::dvec3 outward_world =
                            normalized_or(glm::cross(prograde_world, normal_world), node.maneuver_basis_r_world);
                    if (glm::dot(outward_world, node.maneuver_basis_r_world) < 0.0)
                    {
                        outward_world = -outward_world;
                    }
                    normal_world = normalized_or(glm::cross(outward_world, prograde_world), node.maneuver_basis_n_world);
                    if (glm::dot(normal_world, node.maneuver_basis_n_world) < 0.0)
                    {
                        outward_world = -outward_world;
                        normal_world = -normal_world;
                    }

                    node.basis_r_world = outward_world;
                    node.basis_t_world = prograde_world;
                    node.basis_n_world = normal_world;
                }
            }

            const bool freeze_active_drag_snapshot =
                    suppress_stale_preview &&
                    drag_display_snapshots_available &&
                    _maneuver_gizmo_interaction.node_id == node.id;
            const bool freeze_passive_drag_snapshot =
                    freeze_nonrotating_drag_snapshots &&
                    _maneuver_gizmo_interaction.node_id != node.id;
            const ManeuverNodeDisplaySnapshot *drag_snapshot =
                    (freeze_passive_drag_snapshot || freeze_active_drag_snapshot) ? cached_drag_snapshot : nullptr;
            if (drag_snapshot)
            {
                node_position_world = drag_snapshot->position_world;
                node.basis_r_world = drag_snapshot->basis_r_world;
                node.basis_t_world = drag_snapshot->basis_t_world;
                node.basis_n_world = drag_snapshot->basis_n_world;
            }
            else if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis &&
                     _maneuver_gizmo_interaction.node_id == node.id)
            {
                node.basis_r_world = _maneuver_gizmo_interaction.drag_basis_r_world;
                node.basis_t_world = _maneuver_gizmo_interaction.drag_basis_t_world;
                node.basis_n_world = _maneuver_gizmo_interaction.drag_basis_n_world;
            }

            // Burn direction is used purely for debug visualization; zero-DV nodes fall back to the tangential axis.
            const glm::dvec3 dv_world = compose_basis_vector(node.dv_rtn_mps,
                                                             node.maneuver_basis_r_world,
                                                             node.maneuver_basis_t_world,
                                                             node.maneuver_basis_n_world);

            node.burn_direction_world = normalized_or(dv_world, node.basis_t_world);
            node.position_world = node_position_world;

            if (!std::isfinite(node.gizmo_scale_m) || node.gizmo_scale_m <= 0.0f)
            {
                node.gizmo_scale_m = 1.0f;
            }

            node.gizmo_valid = finite3(glm::dvec3(node.position_world)) &&
                               finite3(node.basis_r_world) &&
                               finite3(node.basis_t_world) &&
                               finite3(node.basis_n_world) &&
                               finite3(node.maneuver_basis_r_world) &&
                               finite3(node.maneuver_basis_t_world) &&
                               finite3(node.maneuver_basis_n_world);
        }
    }
} // namespace Game
