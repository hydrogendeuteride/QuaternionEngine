#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"
#include "game/states/gameplay/maneuver/maneuver_prediction_bridge.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

#include <cmath>

namespace Game
{
    namespace
    {
        using namespace ManeuverUtil;
    } // namespace

    void GameplayState::update_maneuver_nodes_time_warp(GameStateContext &ctx, const float fixed_dt)
    {
        // Drive warp-to-node by selecting the fastest rails warp step that still lands on or before the target time.
        if (!_maneuver.runtime().warp_to_time_active)
        {
            return;
        }

        if (!_orbit.scenario_owner())
        {
            _maneuver.runtime().warp_to_time_active = false;
            set_time_warp_level(ctx, _maneuver.runtime().warp_to_time_restore_level);
            return;
        }

        const double now_s = _orbit.scenario_owner()->sim.time_s();
        const double remaining_s = _maneuver.runtime().warp_to_time_target_s - now_s;
        if (!std::isfinite(remaining_s) || remaining_s <= 0.0)
        {
            _maneuver.runtime().warp_to_time_active = false;
            set_time_warp_level(ctx, _maneuver.runtime().warp_to_time_restore_level);
            return;
        }

        const double dt_base = static_cast<double>(fixed_dt);
        if (!std::isfinite(dt_base) || !(dt_base > 0.0))
        {
            return;
        }

        // Warp-to-target uses rails warp exclusively to allow clamping dt to avoid overshoot.
        int desired_level = 4; // x50

        for (int level = TimeWarpState::kMaxWarpLevel; level >= 4; --level)
        {
            const double step_s = dt_base * TimeWarpState::kWarpFactors[static_cast<size_t>(level)];
            if (std::isfinite(step_s) && step_s <= remaining_s)
            {
                desired_level = level;
                break;
            }
        }

        if (_time_warp.warp_level != desired_level)
        {
            set_time_warp_level(ctx, desired_level);
        }
    }

    void GameplayState::update_maneuver_nodes_execution(GameStateContext &ctx)
    {
        (void) ctx;

        // Burns execute as one-shot impulses once the armed node's scheduled time has arrived.
        if (!_maneuver.runtime().execute_node_armed || _maneuver.runtime().execute_node_id < 0)
        {
            return;
        }

        ManeuverNode *node = _maneuver.plan().find_node(_maneuver.runtime().execute_node_id);
        if (!node)
        {
            _maneuver.runtime().disarm_execute_node();
            return;
        }

        GameplayPredictionAdapter prediction(*this);

        const double now_s = current_sim_time_s();
        if (!std::isfinite(now_s) || now_s + 1e-4 < node->time_s)
        {
            return;
        }

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (!prediction.get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
        {
            return;
        }

        glm::dvec3 r_rel_m(0.0);
        glm::dvec3 v_rel_mps = ship_vel_world;
        const orbitsim::BodyId primary_body_id =
                ManeuverPredictionBridge::resolve_node_primary_body_id(*this, *node, now_s);
        if (_orbit.scenario_owner() && primary_body_id != orbitsim::kInvalidBodyId)
        {
            const orbitsim::MassiveBody *world_ref_sim = _orbit.scenario_owner()->world_reference_sim_body();
            const orbitsim::MassiveBody *primary_body = _orbit.scenario_owner()->sim.body_by_id(primary_body_id);
            if (world_ref_sim && primary_body)
            {
                const WorldVec3 primary_world =
                        prediction.prediction_world_reference_body_world() +
                        WorldVec3(primary_body->state.position_m - world_ref_sim->state.position_m);
                r_rel_m = glm::dvec3(ship_pos_world - primary_world);
                v_rel_mps = ship_vel_world - (primary_body->state.velocity_mps - world_ref_sim->state.velocity_mps);
            }
            else
            {
                r_rel_m = glm::dvec3(ship_pos_world - prediction.prediction_world_reference_body_world());
            }
        }
        else
        {
            r_rel_m = glm::dvec3(ship_pos_world - prediction.prediction_world_reference_body_world());
        }

        const orbitsim::RtnFrame f = compute_maneuver_frame(r_rel_m, v_rel_mps);

        const glm::dvec3 dv_rtn = node->dv_rtn_mps;
        // Nodes are authored directly in true RTN coordinates, so execution only needs the world-space basis transform.
        glm::dvec3 dv_world =
                glm::dvec3(f.R.x, f.R.y, f.R.z) * dv_rtn.x +
                glm::dvec3(f.T.x, f.T.y, f.T.z) * dv_rtn.y +
                glm::dvec3(f.N.x, f.N.y, f.N.z) * dv_rtn.z;

        if (!finite3(dv_world))
        {
            return;
        }

        const bool rails =
                _orbital_physics.rails_warp_active() &&
                _time_warp.mode == TimeWarpState::Mode::RailsWarp;
        bool applied = false;

        if (rails && _orbit.scenario_owner())
        {
            // In rails mode the authoritative velocity lives in orbitsim, so patch the sim state directly.
            const OrbiterInfo *p = _orbit.find_player_orbiter();
            if (p && p->rails.active())
            {
                if (orbitsim::Spacecraft *sc = _orbit.scenario_owner()->sim.spacecraft_by_id(p->rails.sc_id))
                {
                    sc->state.velocity_mps += orbitsim::Vec3{dv_world.x, dv_world.y, dv_world.z};
                    applied = true;
                }
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!applied)
        {
            // Otherwise push the same world-space DV into the active physics body.
            const EntityId player_eid = _orbit.player_entity();
            if (_physics && _physics_context && player_eid.is_valid())
            {
                const Entity *player = _world.entities().find(player_eid);
                if (player && player->has_physics())
                {
                    const Physics::BodyId body_id{player->physics_body_value()};
                    if (_physics->is_body_valid(body_id))
                    {
                        const glm::vec3 v_local_f = _physics->get_linear_velocity(body_id);
                        const glm::dvec3 v_origin_world = _physics_context->velocity_origin_world();
                        glm::dvec3 v_world = v_origin_world + glm::dvec3(v_local_f);
                        v_world += dv_world;
                        const glm::dvec3 v_local_d = v_world - v_origin_world;

                        _physics->set_linear_velocity(body_id, glm::vec3(v_local_d));
                        _physics->activate(body_id);
                        applied = true;
                    }
                }
            }
        }
#endif

        if (!applied)
        {
            return;
        }

        // Propagate the same delta-v to formation followers referencing the burned orbiter as leader.
        {
            const OrbiterInfo *player_orbiter = _orbit.find_player_orbiter();
            if (player_orbiter)
            {
                for (auto &follower : _orbit.orbiters())
                {
                    if (!follower.formation_hold_enabled || &follower == player_orbiter)
                    {
                        continue;
                    }
                    if (follower.formation_leader_name != player_orbiter->name)
                    {
                        continue;
                    }

                    if (rails && _orbit.scenario_owner() && follower.rails.active())
                    {
                        if (orbitsim::Spacecraft *fsc = _orbit.scenario_owner()->sim.spacecraft_by_id(follower.rails.sc_id))
                        {
                            fsc->state.velocity_mps += orbitsim::Vec3{dv_world.x, dv_world.y, dv_world.z};
                        }
                    }
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                    else if (_physics && _physics_context && follower.entity.is_valid())
                    {
                        const Entity *fe = _world.entities().find(follower.entity);
                        if (fe && fe->has_physics())
                        {
                            const Physics::BodyId fbid{fe->physics_body_value()};
                            if (_physics->is_body_valid(fbid))
                            {
                                const glm::vec3 fv_local = _physics->get_linear_velocity(fbid);
                                const glm::dvec3 fv_origin = _physics_context->velocity_origin_world();
                                glm::dvec3 fv_world = fv_origin + glm::dvec3(fv_local);
                                fv_world += dv_world;
                                _physics->set_linear_velocity(fbid, glm::vec3(fv_world - fv_origin));
                                _physics->activate(fbid);
                            }
                        }
                    }
#endif
                }
            }
        }

        // Consume the node after execution (impulse mode).
        (void) apply_maneuver_command(ManeuverCommand::remove_node(node->id));
    }
} // namespace Game
