#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/orbit_helpers.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "game/component/ship_controller.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

#include "orbitsim/trajectories.hpp"
#include "orbitsim/trajectory_transforms.hpp"
#include "orbitsim/coordinate_frames.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace Game
{

    // Read the player's current world-space position and velocity from the active simulation mode.
    bool GameplayState::get_player_world_state(WorldVec3 &out_pos_world,
                                               glm::dvec3 &out_vel_world,
                                               glm::vec3 &out_vel_local) const
    {
        const EntityId player_eid = player_entity();
        const Entity *player = _world.entities().find(player_eid);
        if (!player)
        {
            return false;
        }

        out_pos_world = player->position_world();
        out_vel_world = glm::dvec3(0.0);
        out_vel_local = glm::vec3(0.0f);

        if (_rails_warp_active && _orbitsim)
        {
            const OrbiterInfo *p = find_player_orbiter();
            const orbitsim::MassiveBody *ref_sim = _orbitsim->reference_sim_body();
            if (p && p->rails.active() && ref_sim)
            {
                if (const orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(p->rails.sc_id))
                {
                    out_pos_world = _scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                    out_vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
                    out_vel_local = glm::vec3(0.0f);
                    return true;
                }
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics && _physics_context && player->has_physics())
        {
            const Physics::BodyId body_id{player->physics_body_value()};
            if (_physics->is_body_valid(body_id))
            {
                out_vel_local = _physics->get_linear_velocity(body_id);
                out_vel_world = _physics_context->velocity_origin_world() + glm::dvec3(out_vel_local);
            }
        }
#endif

        return true;
    }

    // Mark the cached prediction stale so the next update requests a rebuild.
    void GameplayState::mark_prediction_dirty()
    {
        _prediction_dirty = true;
    }

    // Advance the prediction cache lifecycle and schedule async solver work when coverage is stale.
    void GameplayState::update_prediction(GameStateContext &ctx, float fixed_dt)
    {
        (void) ctx;

        if (!_prediction_enabled)
        {
            _prediction_cache.clear();
            _prediction_dirty = true;
            _prediction_service.reset();
            _prediction_request_pending = false;
            return;
        }

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);

        if (!get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
        {
            _prediction_cache.clear();
            _prediction_dirty = true;
            _prediction_service.reset();
            _prediction_request_pending = false;
            return;
        }

        const double now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
        const bool thrusting = player_thrust_applied_this_tick();

        // Apply completed async work first so the latest result can satisfy the current frame.
        if (auto completed = _prediction_service.poll_completed())
        {
            _prediction_request_pending = false;
            const bool keep_dirty_for_followup = _prediction_dirty;

            OrbitPredictionService::Result result = std::move(*completed);
            _orbit_plot_perf.solver_ms_last = std::max(0.0, result.compute_time_ms);
            if (result.valid && result.trajectory_bci.size() >= 2)
            {
                _prediction_cache.clear();
                _prediction_cache.build_time_s = now_s;
                _prediction_cache.build_pos_world = ship_pos_world;
                _prediction_cache.build_vel_world = ship_vel_world;
                _prediction_cache.trajectory_bci = std::move(result.trajectory_bci);
                _prediction_cache.trajectory_bci_planned = std::move(result.trajectory_bci_planned);
                _prediction_cache.trajectory_segments_bci = std::move(result.trajectory_segments_bci);
                _prediction_cache.trajectory_segments_bci_planned = std::move(result.trajectory_segments_bci_planned);
                _prediction_cache.maneuver_previews = std::move(result.maneuver_previews);
                _prediction_cache.altitude_km = std::move(result.altitude_km);
                _prediction_cache.speed_kmps = std::move(result.speed_kmps);
                _prediction_cache.semi_major_axis_m = result.semi_major_axis_m;
                _prediction_cache.eccentricity = result.eccentricity;
                _prediction_cache.orbital_period_s = result.orbital_period_s;
                _prediction_cache.periapsis_alt_km = result.periapsis_alt_km;
                _prediction_cache.apoapsis_alt_km = result.apoapsis_alt_km;
                _prediction_cache.valid = true;
                refresh_prediction_world_points();
                _prediction_dirty = keep_dirty_for_followup;
            }
            else
            {
                _prediction_cache.clear();
                _prediction_dirty = true;
            }
        }

        // Package the current ship/reference-body state into a solver request.
        auto request_prediction_async = [&]() -> bool {
            if (!_orbitsim)
            {
                return false;
            }

            const CelestialBodyInfo *ref_info = _orbitsim->reference_body();
            const orbitsim::MassiveBody *ref_sim = _orbitsim->reference_sim_body();
            if (!ref_info || !ref_sim || ref_sim->id == orbitsim::kInvalidBodyId || !(ref_info->mass_kg > 0.0))
            {
                return false;
            }

            const WorldVec3 ref_body_world = prediction_reference_body_world();
            const glm::dvec3 ship_rel_pos_m = glm::dvec3(ship_pos_world - ref_body_world);
            const glm::dvec3 ship_rel_vel_mps = ship_vel_world;
            const glm::dvec3 ship_bary_pos_m = ref_sim->state.position_m + ship_rel_pos_m;
            const glm::dvec3 ship_bary_vel_mps = ref_sim->state.velocity_mps + ship_rel_vel_mps;
            if (!std::isfinite(ship_bary_pos_m.x) || !std::isfinite(ship_bary_pos_m.y) || !std::isfinite(ship_bary_pos_m.z) ||
                !std::isfinite(ship_bary_vel_mps.x) || !std::isfinite(ship_bary_vel_mps.y) || !std::isfinite(ship_bary_vel_mps.z))
            {
                return false;
            }

            OrbitPredictionService::Request request{};
            request.sim_time_s = now_s;
            request.sim_config = _orbitsim->sim.config();
            request.massive_bodies = _orbitsim->sim.massive_bodies();
            request.reference_body_id = ref_sim->id;
            request.reference_body_mass_kg = ref_info->mass_kg;
            request.reference_body_radius_m = ref_info->radius_m;
            request.ship_bary_position_m = ship_bary_pos_m;
            request.ship_bary_velocity_mps = ship_bary_vel_mps;
            request.thrusting = thrusting;
            request.future_window_s = std::max(0.0, _prediction_future_window_s);
            request.max_maneuver_time_s = now_s;

            if (_maneuver_nodes_enabled && !_maneuver_state.nodes.empty())
            {
                request.maneuver_impulses.reserve(_maneuver_state.nodes.size());
                for (const ManeuverNode &node : _maneuver_state.nodes)
                {
                    if (!std::isfinite(node.time_s))
                    {
                        continue;
                    }

                    request.max_maneuver_time_s = std::max(request.max_maneuver_time_s, node.time_s);

                    OrbitPredictionService::ManeuverImpulse impulse{};
                    impulse.node_id = node.id;
                    impulse.t_s = node.time_s;
                    impulse.primary_body_id = node.primary_body_id;
                    impulse.dv_rtn_mps = orbitsim::Vec3{node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z};
                    request.maneuver_impulses.push_back(impulse);
                }
            }

            _prediction_service.request(std::move(request));
            _prediction_request_pending = true;
            return true;
        };

        // Rebuild when the cache is invalid, explicitly dirtied, or too old for current motion.
        bool rebuild_prediction = _prediction_dirty || !_prediction_cache.valid;

        if (!rebuild_prediction && thrusting)
        {
            const double dt_since_build_s = now_s - _prediction_cache.build_time_s;
            rebuild_prediction = !_prediction_cache.valid || dt_since_build_s >= _prediction_thrust_refresh_s;
        }

        if (!rebuild_prediction && _prediction_periodic_refresh_s > 0.0)
        {
            const double dt_since_build_s = now_s - _prediction_cache.build_time_s;
            rebuild_prediction = dt_since_build_s >= _prediction_periodic_refresh_s;
        }

        // While editing maneuver nodes, keep updates live but cap rebuild frequency to avoid
        // solver spikes on long horizons.
        if (rebuild_prediction &&
            _prediction_cache.valid &&
            _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            const double dt_since_build_s = now_s - _prediction_cache.build_time_s;
            if (dt_since_build_s < OrbitPredictionTuning::kDragRebuildMinIntervalS)
            {
                rebuild_prediction = false;
            }
        }

        // Ensure the cache covers the desired future window; otherwise the highlighted segment will
        // shrink and clamp at the end of the current trajectory.
        if (!rebuild_prediction && _prediction_cache.valid && !_prediction_cache.trajectory_bci.empty())
        {
            double cache_end_s = _prediction_cache.trajectory_bci.back().t_s;
            if (!_prediction_cache.trajectory_segments_bci.empty())
            {
                const orbitsim::TrajectorySegment &last_segment = _prediction_cache.trajectory_segments_bci.back();
                const double segment_end_s = last_segment.t0_s + last_segment.dt_s;
                if (std::isfinite(segment_end_s))
                {
                    cache_end_s = segment_end_s;
                }
            }
            double required_ahead_s = std::max(0.0, _prediction_draw_future_segment ? _prediction_future_window_s : 0.0);

            if (_maneuver_nodes_enabled && !_maneuver_state.nodes.empty())
            {
                double max_node_time_s = now_s;
                for (const ManeuverNode &node : _maneuver_state.nodes)
                {
                    if (std::isfinite(node.time_s))
                    {
                        max_node_time_s = std::max(max_node_time_s, node.time_s);
                    }
                }

                if (max_node_time_s > now_s)
                {
                    // Ensure we can render a useful post-node segment (planned trajectory).
                    const double post_node_window_s =
                            std::max(OrbitPredictionTuning::kPostNodeCoverageMinS, _prediction_future_window_s);
                    required_ahead_s = std::max(required_ahead_s, (max_node_time_s - now_s) + post_node_window_s);
                }
            }

            // Use a small epsilon in the "have enough coverage?" test to avoid rebuild thrashing when
            // cache_end-now and required_ahead are nearly equal (common with maneuver-planned horizons).
            // Without this, planned trajectories can trigger a rebuild every fixed tick at long windows.
            const double coverage_epsilon_s = std::max(1.0e-3, std::min(0.25, std::max(0.0, static_cast<double>(fixed_dt)) * 0.5));
            rebuild_prediction = (cache_end_s - now_s + coverage_epsilon_s) < required_ahead_s;
        }

        // Only queue one async request at a time; if one is already running, leave the dirty bit set.
        if (rebuild_prediction)
        {
            if (!_prediction_request_pending)
            {
                const bool requested = request_prediction_async();
                _prediction_dirty = !requested;
                if (!requested)
                {
                    _prediction_cache.clear();
                    _prediction_request_pending = false;
                }
            }
            else
            {
                _prediction_dirty = true;
            }
        }
    }

    // Report whether the player applied thrust during this tick in the current simulation mode.
    bool GameplayState::player_thrust_applied_this_tick() const
    {
        if (_rails_warp_active && _time_warp.mode == TimeWarpState::Mode::RailsWarp)
        {
            return _rails_thrust_applied_this_tick;
        }

        const EntityId player_eid = player_entity();
        if (!player_eid.is_valid())
        {
            return false;
        }

        const Entity *player = _world.entities().find(player_eid);
        if (!player)
        {
            return false;
        }

        const ShipController *sc = player->get_component<ShipController>();
        return sc && sc->thrust_applied_this_tick();
    }

    // Resolve the world-space origin used to place body-centered prediction samples.
    WorldVec3 GameplayState::prediction_reference_body_world() const
    {
        if (!_orbitsim)
        {
            return _scenario_config.system_center;
        }

        const CelestialBodyInfo *ref_info = _orbitsim->reference_body();
        if (ref_info && ref_info->render_entity.is_valid())
        {
            if (const Entity *ref_entity = _world.entities().find(ref_info->render_entity))
            {
                return ref_entity->position_world();
            }
        }

        return _scenario_config.system_center;
    }

    // Rebuild world-space polylines from cached body-centered trajectory samples.
    void GameplayState::refresh_prediction_world_points()
    {
        if (!_prediction_cache.valid || _prediction_cache.trajectory_bci.empty())
        {
            _prediction_cache.points_world.clear();
            _prediction_cache.points_world_planned.clear();
            return;
        }

        const WorldVec3 ref_body_world = prediction_reference_body_world();
        _prediction_cache.points_world.resize(_prediction_cache.trajectory_bci.size());

        for (size_t i = 0; i < _prediction_cache.trajectory_bci.size(); ++i)
        {
            _prediction_cache.points_world[i] =
                    ref_body_world + WorldVec3(_prediction_cache.trajectory_bci[i].position_m);
        }

        if (_prediction_cache.trajectory_bci_planned.empty())
        {
            _prediction_cache.points_world_planned.clear();
            return;
        }

        _prediction_cache.points_world_planned.resize(_prediction_cache.trajectory_bci_planned.size());
        for (size_t i = 0; i < _prediction_cache.trajectory_bci_planned.size(); ++i)
        {
            _prediction_cache.points_world_planned[i] =
                    ref_body_world + WorldVec3(_prediction_cache.trajectory_bci_planned[i].position_m);
        }
    }

} // namespace Game
