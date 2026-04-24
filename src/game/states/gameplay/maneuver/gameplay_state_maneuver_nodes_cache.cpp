#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"

#include "game/orbit/orbit_prediction_math.h"

#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace Game
{
    namespace
    {
        using namespace ManeuverUtil;

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
            const orbitsim::State sampled = OrbitPredictionMath::sample_pair_state(a, b, t_clamped);

            out.position_world = OrbitPredictionMath::sample_pair_position_world(ref_body_world, a, b, t_clamped);
            out.r_rel_m = sampled.position_m;
            out.v_rel_mps = sampled.velocity_mps;
            out.valid = finite3(out.r_rel_m) && finite3(out.v_rel_mps);
            return out;
        }

        // HermiteSamplerFn: (const TrajectorySample &a, const TrajectorySample &b,
        //                    double t_s) -> WorldVec3
        //                   Called for hermite interpolation between two samples.
        // PositionSamplerFn: (const TrajectorySample &sample) -> WorldVec3
        //                    Called for single-sample position lookup.

        // Shared trajectory sampling: finds the bracketing pair at sample_time_s and uses
        // hermite interpolation to produce a smooth world-space position.
        template<typename HermiteSamplerFn, typename PositionSamplerFn>
        bool sample_traj_world_at(HermiteSamplerFn &&hermite_fn,
                                  PositionSamplerFn &&position_fn,
                                  const std::vector<orbitsim::TrajectorySample> &traj,
                                  const double sample_time_s,
                                  const WorldVec3 &align_delta,
                                  WorldVec3 &out_world)
        {
            out_world = WorldVec3(0.0, 0.0, 0.0);
            if (traj.size() < 2 || !std::isfinite(sample_time_s))
            {
                return false;
            }

            if (sample_time_s < traj.front().t_s || sample_time_s > traj.back().t_s)
            {
                return false;
            }

            auto it_hi = std::lower_bound(traj.cbegin(), traj.cend(), sample_time_s,
                                          [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
            const size_t i_hi = static_cast<size_t>(std::distance(traj.cbegin(), it_hi));
            if (i_hi >= traj.size())
            {
                return false;
            }

            out_world = (i_hi > 0)
                            ? hermite_fn(traj[i_hi - 1], traj[i_hi], sample_time_s)
                            : position_fn(traj.front());
            out_world += align_delta;
            return finite3(glm::dvec3(out_world));
        }

        // Unified tangent sampling: computes a finite-difference tangent from two nearby position samples.
        // PosSampler: (double time_s, WorldVec3 &out) -> bool
        template<typename PosSampler>
        bool sample_tangent_world_from(const std::vector<orbitsim::TrajectorySample> &ref_traj,
                                       const double sample_time_s,
                                       PosSampler &&pos_sampler,
                                       glm::dvec3 &out_tangent_world)
        {
            out_tangent_world = glm::dvec3(0.0, 0.0, 0.0);
            if (ref_traj.size() < 2 || !std::isfinite(sample_time_s))
            {
                return false;
            }

            const double traj_t0 = ref_traj.front().t_s;
            const double traj_t1 = ref_traj.back().t_s;
            const double backward_dt_s = std::min(0.25, std::max(0.0, sample_time_s - traj_t0));
            const double forward_dt_s = std::min(0.25, std::max(0.0, traj_t1 - sample_time_s));

            WorldVec3 p0_world{0.0, 0.0, 0.0};
            WorldVec3 p1_world{0.0, 0.0, 0.0};
            if (backward_dt_s >= 1.0e-4)
            {
                if (!pos_sampler(sample_time_s - backward_dt_s, p0_world) ||
                    !pos_sampler(sample_time_s, p1_world))
                {
                    return false;
                }
            }
            else if (forward_dt_s >= 1.0e-4)
            {
                if (!pos_sampler(sample_time_s, p0_world) ||
                    !pos_sampler(sample_time_s + forward_dt_s, p1_world))
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
        }

        using PreviewMap = std::unordered_map<int, const OrbitPredictionCache::ManeuverNodePreview *>;
        constexpr double kNodePreviewTimeMatchEpsilonS = 1.0e-6;

        PreviewMap build_preview_map(const OrbitPredictionCache &cache)
        {
            PreviewMap map;
            map.reserve(cache.maneuver_previews.size());
            for (const OrbitPredictionCache::ManeuverNodePreview &p : cache.maneuver_previews)
            {
                map[p.node_id] = &p;
            }
            return map;
        }

        bool preview_matches_node_time(const OrbitPredictionCache::ManeuverNodePreview *preview,
                                       const ManeuverNode &node)
        {
            return preview &&
                   preview->valid &&
                   std::isfinite(preview->t_s) &&
                   std::isfinite(node.time_s) &&
                   std::abs(preview->t_s - node.time_s) <= kNodePreviewTimeMatchEpsilonS;
        }

        bool current_plan_has_prior_future_maneuver(const std::vector<ManeuverNode> &nodes,
                                                    const int anchor_node_id,
                                                    const double anchor_time_s,
                                                    const double sim_now_s)
        {
            if (!std::isfinite(anchor_time_s) || !std::isfinite(sim_now_s))
            {
                return true;
            }

            return std::any_of(nodes.begin(),
                               nodes.end(),
                               [&](const ManeuverNode &node) {
                                   return node.id != anchor_node_id &&
                                          std::isfinite(node.time_s) &&
                                          node.time_s + kNodePreviewTimeMatchEpsilonS >= sim_now_s &&
                                          node.time_s < (anchor_time_s - kNodePreviewTimeMatchEpsilonS);
                               });
        }
    } // namespace

    void GameplayState::refresh_maneuver_node_runtime_cache(GameStateContext &ctx)
    {
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
        const OrbitPredictionCache *effective_cache = player_prediction_cache();
        const OrbitPredictionCache *stable_cache =
                (player_track && player_track->cache.valid) ? &player_track->cache : effective_cache;
        const bool interaction_idle =
                _maneuver_gizmo_interaction.state != ManeuverGizmoInteraction::State::DragAxis;
        const bool hold_cached_release_state =
                player_track && interaction_idle &&
                (player_track->request_pending || player_track->derived_request_pending);
        const OrbitPredictionCache *display_cache = effective_cache;
        const OrbitPredictionCache *pred_cache = display_cache;
        if (!display_cache || !display_cache->valid || display_cache->trajectory_frame.size() < 2)
        {
            return;
        }
        if (!pred_cache || !pred_cache->valid)
        {
            return;
        }

        const auto &traj_base = display_cache->trajectory_frame;
        const auto &traj_planned = display_cache->trajectory_frame_planned;
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
        if (!std::isfinite(now_s) || !(t1 > t0))
        {
            return;
        }
        now_s = std::clamp(now_s, t0, t1);

        const WorldVec3 align_delta = compute_maneuver_align_delta(ctx, *display_cache, traj_base);

        const auto &traj_node_world = traj_planned.size() >= 2 ? traj_planned : traj_base;
        const auto &pred_base_traj_inertial = pred_cache->resolved_trajectory_inertial();
        const auto &traj_node_inertial =
                pred_cache->trajectory_inertial_planned.size() >= 2
                    ? pred_cache->trajectory_inertial_planned
                    : pred_base_traj_inertial;
        const PreviewMap preview_by_node_id = build_preview_map(*pred_cache);
        PreviewMap stable_preview_by_node_id;
        if (stable_cache && stable_cache != pred_cache)
        {
            stable_preview_by_node_id = build_preview_map(*stable_cache);
        }
        const auto *stable_traj_node_world =
                (stable_cache && stable_cache->trajectory_frame_planned.size() >= 2)
                        ? &stable_cache->trajectory_frame_planned
                        : nullptr;
        const auto *stable_traj_node_inertial =
                (stable_cache && stable_cache->trajectory_inertial_planned.size() >= 2)
                        ? &stable_cache->trajectory_inertial_planned
                        : nullptr;
        const bool allow_base_fallback = _maneuver_state.nodes.size() <= 1;
        const bool stable_planned_prefix_available =
                false;
        const orbitsim::TrajectoryFrameSpec display_frame_spec =
                display_cache->resolved_frame_spec_valid
                    ? display_cache->resolved_frame_spec
                    : resolve_prediction_display_frame_spec(*display_cache, now_s);
        const bool display_frame_uses_drag_snapshot_time =
                display_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial ||
                display_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial;
        const bool drag_snapshots_available =
                _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis &&
                !_maneuver_gizmo_interaction.drag_display_snapshots.empty();
        const bool freeze_nonrotating_drag_snapshots =
                display_frame_uses_drag_snapshot_time && drag_snapshots_available;
        bool display_player_lookup_attempted = false;
        orbitsim::SpacecraftStateLookup display_player_lookup{};
        const auto get_display_player_lookup = [&]() -> const orbitsim::SpacecraftStateLookup * {
            if (!display_player_lookup_attempted)
            {
                display_player_lookup = build_prediction_player_lookup();
                display_player_lookup_attempted = true;
            }
            return display_player_lookup ? &display_player_lookup : nullptr;
        };

        const auto transform_preview_to_display_sample =
                [&](const OrbitPredictionCache &sample_cache,
                    const double sample_time_s,
                    const glm::dvec3 &preview_pos_m,
                    const glm::dvec3 &preview_vel_mps,
                    orbitsim::TrajectorySample &out_sample) -> bool {
                    out_sample = {};
                    if (!std::isfinite(sample_time_s) || !finite3(preview_pos_m) || !finite3(preview_vel_mps))
                    {
                        return false;
                    }

                    const orbitsim::TrajectoryFrameSpec frame_spec =
                            sample_cache.resolved_frame_spec_valid
                                ? sample_cache.resolved_frame_spec
                                : resolve_prediction_display_frame_spec(sample_cache, now_s);
                    const auto &ephemeris = sample_cache.resolved_shared_ephemeris();
                    if (frame_spec.type != orbitsim::TrajectoryFrameType::Inertial &&
                        (!ephemeris || ephemeris->empty()))
                    {
                        return false;
                    }

                    const orbitsim::SpacecraftStateLookup *player_lookup = nullptr;
                    if (frame_spec.type == orbitsim::TrajectoryFrameType::LVLH)
                    {
                        player_lookup = get_display_player_lookup();
                        if (player_lookup == nullptr)
                        {
                            return false;
                        }
                    }

                    const std::vector<orbitsim::TrajectorySample> frame_samples = orbitsim::trajectory_to_frame_spec(
                            std::vector<orbitsim::TrajectorySample>{orbitsim::TrajectorySample{
                                    .t_s = sample_time_s,
                                    .position_m = orbitsim::Vec3{preview_pos_m.x, preview_pos_m.y, preview_pos_m.z},
                                    .velocity_mps = orbitsim::Vec3{preview_vel_mps.x,
                                                                   preview_vel_mps.y,
                                                                   preview_vel_mps.z},
                            }},
                            ephemeris ? *ephemeris : orbitsim::CelestialEphemeris{},
                            sample_cache.resolved_massive_bodies(),
                            frame_spec,
                            player_lookup ? *player_lookup : orbitsim::SpacecraftStateLookup{});
                    if (frame_samples.size() != 1)
                    {
                        return false;
                    }

                    out_sample = frame_samples.front();
                    return finite3(glm::dvec3(out_sample.position_m)) &&
                           finite3(glm::dvec3(out_sample.velocity_mps));
                };

        const auto sample_preview_node_world = [&](const OrbitPredictionCache &sample_cache,
                                                   const double sample_time_s,
                                                   const glm::dvec3 &preview_pos_m,
                                                   const glm::dvec3 &preview_vel_mps,
                                                   WorldVec3 &out_world) -> bool {
            out_world = WorldVec3(0.0, 0.0, 0.0);
            orbitsim::TrajectorySample frame_sample{};
            if (!transform_preview_to_display_sample(sample_cache, sample_time_s, preview_pos_m, preview_vel_mps, frame_sample))
            {
                return false;
            }
            out_world = prediction_sample_position_world(sample_cache, frame_sample, now_s) + align_delta;
            return finite3(glm::dvec3(out_world));
        };

        const auto sample_preview_tangent_world = [&](const OrbitPredictionCache &sample_cache,
                                                      const double sample_time_s,
                                                      const glm::dvec3 &preview_pos_m,
                                                      const glm::dvec3 &preview_vel_mps,
                                                      glm::dvec3 &out_tangent_world) -> bool {
            out_tangent_world = glm::dvec3(0.0, 0.0, 0.0);
            orbitsim::TrajectorySample frame_sample{};
            if (!transform_preview_to_display_sample(sample_cache, sample_time_s, preview_pos_m, preview_vel_mps, frame_sample))
            {
                return false;
            }
            WorldVec3 display_origin_world{0.0, 0.0, 0.0};
            glm::dmat3 display_frame_to_world(1.0);
            if (!build_prediction_display_transform(sample_cache, display_origin_world, display_frame_to_world, now_s))
            {
                return false;
            }
            out_tangent_world =
                    normalized_or(display_frame_to_world * glm::dvec3(frame_sample.velocity_mps),
                                  glm::dvec3(0.0, 1.0, 0.0));
            return finite3(out_tangent_world);
        };

        const auto resolve_node_primary =
                [&](const OrbitPredictionCache &sample_cache,
                    const ManeuverNode &node,
                    const double sample_time_s,
                    const glm::dvec3 &inertial_pos_m) -> orbitsim::BodyId {
                    const orbitsim::BodyId preferred = resolve_maneuver_node_primary_body_id(node, sample_time_s);
                    if (preferred != orbitsim::kInvalidBodyId)
                    {
                        return preferred;
                    }
                    const auto &bodies = sample_cache.resolved_massive_bodies();
                    const auto &ephemeris = sample_cache.resolved_shared_ephemeris();
                    if (bodies.empty() || !std::isfinite(sample_time_s) || !finite3(inertial_pos_m))
                    {
                        return orbitsim::kInvalidBodyId;
                    }

                    const auto body_pos_at = [&](const std::size_t i) -> orbitsim::Vec3 {
                        const orbitsim::MassiveBody &body = bodies[i];
                        if (ephemeris && !ephemeris->empty())
                        {
                            return ephemeris->body_state_at_by_id(body.id, sample_time_s).position_m;
                        }
                        return body.state.position_m;
                    };

                    const std::size_t idx = orbitsim::auto_select_primary_index(
                            bodies,
                            orbitsim::Vec3{inertial_pos_m.x, inertial_pos_m.y, inertial_pos_m.z},
                            body_pos_at,
                            _orbitsim ? _orbitsim->sim.config().softening_length_m : 0.0);
                    return idx < bodies.size()
                               ? bodies[idx].id
                               : orbitsim::kInvalidBodyId;
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
                    drag_snapshots_available
                        ? find_display_snapshot(_maneuver_gizmo_interaction.drag_display_snapshots, node.id)
                        : nullptr;
            const bool cached_node_state_valid =
                    finite3(glm::dvec3(node.position_world)) &&
                    finite3(node.basis_r_world) && finite3(node.basis_t_world) && finite3(node.basis_n_world) &&
                    finite3(node.maneuver_basis_r_world) && finite3(node.maneuver_basis_t_world) && finite3(node.maneuver_basis_n_world);
            const bool can_fallback_drag = active_drag_node && cached_drag_snapshot != nullptr;
            const bool can_fallback_release = hold_cached_release_state && cached_node_state_valid;
            const glm::dvec3 cached_basis_r = node.basis_r_world;
            const glm::dvec3 cached_basis_t = node.basis_t_world;
            const glm::dvec3 cached_basis_n = node.basis_n_world;
            const glm::dvec3 cached_mbasis_r = node.maneuver_basis_r_world;
            const glm::dvec3 cached_mbasis_t = node.maneuver_basis_t_world;
            const glm::dvec3 cached_mbasis_n = node.maneuver_basis_n_world;

            // --- Resolve which cache/trajectory source to use for this node ---
            const bool use_stable_prefix =
                    stable_planned_prefix_available &&
                    false;
            const bool editing_node_time =
                    _maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingTime &&
                    _maneuver_node_edit_preview.node_id == node.id;
            const bool time_edit_uses_unplanned_source =
                    editing_node_time &&
                    !current_plan_has_prior_future_maneuver(_maneuver_state.nodes,
                                                            node.id,
                                                            node.time_s,
                                                            current_sim_time_s());
            const OrbitPredictionCache *node_pred_cache = use_stable_prefix ? stable_cache : pred_cache;
            const OrbitPredictionCache *node_disp_cache = use_stable_prefix ? stable_cache : display_cache;
            const PreviewMap *node_preview_map = use_stable_prefix ? &stable_preview_by_node_id : &preview_by_node_id;
            const std::vector<orbitsim::TrajectorySample> *node_traj_world =
                    time_edit_uses_unplanned_source
                            ? &traj_base
                    : use_stable_prefix
                            ? stable_traj_node_world
                            : (traj_planned.size() >= 2
                                       ? &traj_planned
                                       : (stable_traj_node_world
                                                  ? stable_traj_node_world
                                                  : (allow_base_fallback ? &traj_base : nullptr)));
            const std::vector<orbitsim::TrajectorySample> *node_traj_inertial =
                    time_edit_uses_unplanned_source
                            ? &pred_base_traj_inertial
                    : use_stable_prefix
                            ? stable_traj_node_inertial
                            : (pred_cache->trajectory_inertial_planned.size() >= 2
                                       ? &pred_cache->trajectory_inertial_planned
                                       : (stable_traj_node_inertial
                                                  ? stable_traj_node_inertial
                                                  : (allow_base_fallback ? &pred_base_traj_inertial : nullptr)));
            auto preview_it = node_preview_map->find(node.id);
            const OrbitPredictionCache::ManeuverNodePreview *preview =
                    (preview_it != node_preview_map->end()) ? preview_it->second : nullptr;
            bool preview_time_valid = preview_matches_node_time(preview, node);
            if (time_edit_uses_unplanned_source)
            {
                preview = nullptr;
                preview_time_valid = false;
            }

            // Try stable fallback if the active cache doesn't cover this node.
            if (!time_edit_uses_unplanned_source && !use_stable_prefix && stable_planned_prefix_available)
            {
                const auto stable_it = stable_preview_by_node_id.find(node.id);
                const bool active_ok = preview && preview->valid;
                const bool stable_ok = stable_it != stable_preview_by_node_id.end() && stable_it->second && stable_it->second->valid;
                const bool active_traj_ok = node_traj_world && !node_traj_world->empty() &&
                        node.time_s >= node_traj_world->front().t_s && node.time_s <= node_traj_world->back().t_s;
                const bool stable_traj_ok = stable_traj_node_world && !stable_traj_node_world->empty() &&
                        node.time_s >= stable_traj_node_world->front().t_s && node.time_s <= stable_traj_node_world->back().t_s;
                if ((!active_ok && stable_ok) || (!active_ok && !active_traj_ok && stable_traj_ok))
                {
                    node_pred_cache = stable_cache;
                    node_disp_cache = stable_cache;
                    node_preview_map = &stable_preview_by_node_id;
                    node_traj_world = stable_traj_node_world;
                    node_traj_inertial = stable_traj_node_inertial;
                    preview_it = node_preview_map->find(node.id);
                    preview = (preview_it != node_preview_map->end()) ? preview_it->second : nullptr;
                    preview_time_valid = preview_matches_node_time(preview, node);
                }
            }

            // Build per-node world-space samplers (wrapping private member methods).
            const auto hermite_sampler = [this, &node_disp_cache, now_s](
                const orbitsim::TrajectorySample &a, const orbitsim::TrajectorySample &b,
                double t_s) -> WorldVec3 {
                return prediction_sample_hermite_world(*node_disp_cache, a, b, t_s, now_s);
            };
            const auto position_sampler = [this, &node_disp_cache, now_s](
                const orbitsim::TrajectorySample &sample) -> WorldVec3 {
                return prediction_sample_position_world(*node_disp_cache, sample, now_s);
            };

            // --- Resolve position ---
            glm::dvec3 r_rel_m{0.0, 0.0, 0.0};
            glm::dvec3 v_rel_mps{0.0, 0.0, 0.0};
            WorldVec3 node_position_world{0.0, 0.0, 0.0};
            bool have_runtime_state = false;
            bool use_cached_basis = false;
            double basis_time_s = node.time_s;
            if (node_traj_world && !node_traj_world->empty() && basis_time_s > node_traj_world->front().t_s)
            {
                basis_time_s = std::max(node_traj_world->front().t_s, basis_time_s - 1e-3);
            }

            const bool have_preview_pos =
                    preview_time_valid &&
                    sample_preview_node_world(*node_disp_cache, node.time_s,
                                               glm::dvec3(preview->inertial_position_m),
                                               glm::dvec3(preview->inertial_velocity_mps),
                                              node_position_world);
            if (!have_preview_pos &&
                (!node_traj_world ||
                 !sample_traj_world_at(hermite_sampler, position_sampler, *node_traj_world,
                                        node.time_s, align_delta, node_position_world)))
            {
                if (can_fallback_drag)
                {
                    node_position_world = cached_drag_snapshot->position_world;
                }
                else if (can_fallback_release)
                {
                    node_position_world = node.position_world;
                }
                else
                {
                    continue;
                }
            }

            // --- Resolve orbital state for basis computation ---
            if (preview_time_valid)
            {
                const glm::dvec3 preview_pos_m = glm::dvec3(preview->inertial_position_m);
                const orbitsim::BodyId primary_id =
                        resolve_node_primary(*node_pred_cache, node, node.time_s, preview_pos_m);
                const auto &bodies = node_pred_cache->resolved_massive_bodies();
                const auto &ephemeris = node_pred_cache->resolved_shared_ephemeris();
                const orbitsim::MassiveBody *primary = find_massive_body(bodies, primary_id);
                if (primary)
                {
                    const orbitsim::State ps =
                            (ephemeris && !ephemeris->empty())
                                ? ephemeris->body_state_at_by_id(primary_id, node.time_s)
                                : primary->state;
                    r_rel_m = preview_pos_m - glm::dvec3(ps.position_m);
                    v_rel_mps = glm::dvec3(preview->inertial_velocity_mps) - glm::dvec3(ps.velocity_mps);
                    have_runtime_state = finite3(r_rel_m) && finite3(v_rel_mps);
                }
            }
            else
            {
                if (!node_traj_inertial || node_traj_inertial->size() < 2)
                {
                    continue;
                }
                const TrajectorySampledState ist =
                        sample_trajectory_state(*node_traj_inertial, WorldVec3(0.0), basis_time_s);
                if (!ist.valid)
                {
                    continue;
                }
                const orbitsim::BodyId primary_id =
                        resolve_node_primary(*node_pred_cache, node, basis_time_s, ist.r_rel_m);
                const auto &bodies = node_pred_cache->resolved_massive_bodies();
                const auto &ephemeris = node_pred_cache->resolved_shared_ephemeris();
                const orbitsim::MassiveBody *primary = find_massive_body(bodies, primary_id);
                if (!primary)
                {
                    continue;
                }
                const orbitsim::State ps =
                        (ephemeris && !ephemeris->empty())
                            ? ephemeris->body_state_at_by_id(primary_id, basis_time_s)
                            : primary->state;
                r_rel_m = ist.r_rel_m - glm::dvec3(ps.position_m);
                v_rel_mps = ist.v_rel_mps - glm::dvec3(ps.velocity_mps);
                have_runtime_state = finite3(r_rel_m) && finite3(v_rel_mps);
            }

            if (!have_runtime_state)
            {
                if (!(can_fallback_drag || can_fallback_release) ||
                    !finite3(cached_mbasis_r) || !finite3(cached_mbasis_t) || !finite3(cached_mbasis_n))
                {
                    continue;
                }
                use_cached_basis = true;
            }

            // --- Compute basis vectors ---
            if (use_cached_basis)
            {
                node.maneuver_basis_r_world = cached_mbasis_r;
                node.maneuver_basis_t_world = cached_mbasis_t;
                node.maneuver_basis_n_world = cached_mbasis_n;
                node.basis_r_world = cached_basis_r;
                node.basis_t_world = cached_basis_t;
                node.basis_n_world = cached_basis_n;
            }
            else
            {
                const orbitsim::RtnFrame sf = compute_maneuver_frame(r_rel_m, v_rel_mps);
                node.maneuver_basis_r_world = normalized_or(glm::dvec3(sf.R.x, sf.R.y, sf.R.z), glm::dvec3(1, 0, 0));
                node.maneuver_basis_t_world = normalized_or(glm::dvec3(sf.T.x, sf.T.y, sf.T.z), glm::dvec3(0, 1, 0));
                node.maneuver_basis_n_world = normalized_or(glm::dvec3(sf.N.x, sf.N.y, sf.N.z), glm::dvec3(0, 0, 1));

                if (_maneuver_gizmo_basis_mode == ManeuverGizmoBasisMode::RTN)
                {
                    node.basis_r_world = node.maneuver_basis_r_world;
                    node.basis_t_world = node.maneuver_basis_t_world;
                    node.basis_n_world = node.maneuver_basis_n_world;
                }
                else
                {
                    const glm::dvec3 fallback_rtn_t(sf.T.x, sf.T.y, sf.T.z);
                    glm::dvec3 prograde_inertial = normalized_or(v_rel_mps, fallback_rtn_t);
                    glm::dvec3 normal_inertial = normalized_or(glm::dvec3(sf.N.x, sf.N.y, sf.N.z), glm::dvec3(0, 0, 1));
                    glm::dvec3 prograde_world = normalized_or(prograde_inertial, node.maneuver_basis_t_world);

                    // Try preview tangent, then chunk tangent, then displayed tangent for more accurate prograde direction.
                    glm::dvec3 tangent_world{0.0, 0.0, 0.0};
                    if (preview_time_valid &&
                        sample_preview_tangent_world(*node_disp_cache, node.time_s,
                                                     glm::dvec3(preview->inertial_position_m),
                                                     glm::dvec3(preview->inertial_velocity_mps),
                                                     tangent_world))
                    {
                        prograde_world = tangent_world;
                    }
                    else if (node_traj_world)
                    {
                        const auto traj_pos_sampler = [&](double t, WorldVec3 &out) {
                            return sample_traj_world_at(hermite_sampler, position_sampler, *node_traj_world,
                                                        t, align_delta, out);
                        };

                        if (sample_tangent_world_from(*node_traj_world, basis_time_s, traj_pos_sampler, tangent_world))
                        {
                            prograde_world = tangent_world;
                        }
                    }

                    glm::dvec3 normal_world = normalized_or(
                            normalized_or(normal_inertial, node.maneuver_basis_n_world),
                            node.maneuver_basis_n_world);
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

            // --- Apply drag snapshot overrides ---
            const bool freeze_active_snap =
                    drag_snapshots_available && _maneuver_gizmo_interaction.node_id == node.id;
            const bool freeze_passive_snap =
                    freeze_nonrotating_drag_snapshots && _maneuver_gizmo_interaction.node_id != node.id;
            const ManeuverNodeDisplaySnapshot *drag_snap =
                    (freeze_passive_snap || freeze_active_snap) ? cached_drag_snapshot : nullptr;
            if (drag_snap)
            {
                node_position_world = drag_snap->position_world;
                node.basis_r_world = drag_snap->basis_r_world;
                node.basis_t_world = drag_snap->basis_t_world;
                node.basis_n_world = drag_snap->basis_n_world;
            }
            else if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis &&
                     _maneuver_gizmo_interaction.node_id == node.id)
            {
                node.basis_r_world = _maneuver_gizmo_interaction.drag_basis_r_world;
                node.basis_t_world = _maneuver_gizmo_interaction.drag_basis_t_world;
                node.basis_n_world = _maneuver_gizmo_interaction.drag_basis_n_world;
            }

            if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis &&
                _maneuver_gizmo_interaction.node_id == node.id)
            {
                node.maneuver_basis_r_world = _maneuver_gizmo_interaction.drag_maneuver_basis_r_world;
                node.maneuver_basis_t_world = _maneuver_gizmo_interaction.drag_maneuver_basis_t_world;
                node.maneuver_basis_n_world = _maneuver_gizmo_interaction.drag_maneuver_basis_n_world;
            }

            // --- Finalize node ---
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
                               finite3(node.basis_r_world) && finite3(node.basis_t_world) && finite3(node.basis_n_world) &&
                               finite3(node.maneuver_basis_r_world) && finite3(node.maneuver_basis_t_world) && finite3(node.maneuver_basis_n_world);
        }
    }
} // namespace Game
