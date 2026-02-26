#include "gameplay_state.h"
#include "orbit_helpers.h"
#include "core/game_api.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

#include <algorithm>
#include <cmath>

namespace Game
{
    using detail::finite_vec3;
    using detail::nbody_accel_body_centered;

    namespace
    {
        void update_rails_rotation(OrbiterInfo::RailsState &rs,
                                   const glm::vec3 &world_torque_dir,
                                   const float torque_strength,
                                   const float sas_damping,
                                   const bool sas_enabled,
                                   const double dt_s)
        {
            const float dt = static_cast<float>(dt_s);
            if (!(dt > 0.0f) || !std::isfinite(dt))
            {
                return;
            }

            // Apply torque input (not a physically correct rigid-body model; tuned for gameplay).
            if (glm::length(world_torque_dir) > 0.0f)
            {
                rs.angular_velocity_radps += world_torque_dir * (torque_strength * dt);
            }

            // SAS damping (mass/inertia independent).
            if (sas_enabled && glm::length(world_torque_dir) < 0.01f)
            {
                const float damping = std::max(0.0f, sas_damping);
                const float decay = std::exp(-damping * dt);
                rs.angular_velocity_radps *= decay;
                if (glm::length(rs.angular_velocity_radps) < 1e-3f)
                {
                    rs.angular_velocity_radps = glm::vec3(0.0f);
                }
            }

            const float omega = glm::length(rs.angular_velocity_radps);
            if (omega > 1e-6f)
            {
                const float angle = omega * dt;
                const glm::vec3 axis = rs.angular_velocity_radps / omega;
                rs.rotation = glm::angleAxis(angle, axis) * rs.rotation;
                rs.rotation = glm::normalize(rs.rotation);
            }
        }
    } // namespace

    // ---- Physics simulation ----

    void GameplayState::sync_celestial_render_entities(GameStateContext &ctx)
    {
        if (!_orbitsim || _orbitsim->bodies.empty())
        {
            return;
        }

        const auto &cfg = _scenario_config;

        const CelestialBodyInfo *ref_info = _orbitsim->reference_body();
        const orbitsim::MassiveBody *ref_sim = _orbitsim->reference_sim_body();
        if (!ref_info || !ref_sim)
        {
            return;
        }

        for (auto &body_info : _orbitsim->bodies)
        {
            if (body_info.sim_id == ref_info->sim_id)
            {
                continue;
            }

            const orbitsim::MassiveBody *sim_body = _orbitsim->sim.body_by_id(body_info.sim_id);
            if (!sim_body)
            {
                continue;
            }

            const WorldVec3 body_pos_world =
                    cfg.system_center + WorldVec3(sim_body->state.position_m - ref_sim->state.position_m);

            if (body_info.has_terrain && ctx.api)
            {
                (void) ctx.api->set_planet_center(body_info.name, glm::dvec3(body_pos_world));
            }

            if (!body_info.render_entity.is_valid())
            {
                continue;
            }

            if (Entity *ent = _world.entities().find(body_info.render_entity))
            {
                ent->set_position_world(body_pos_world);
                ent->set_rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
            }
        }
    }

    void GameplayState::step_physics(GameStateContext &ctx, float fixed_dt)
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_physics || !_physics_context)
        {
            return;
        }

        update_rebase_anchor();
        _world.pre_physics_step();

        const auto &cfg = _scenario_config;
        const bool use_orbitsim = _orbitsim && !_orbitsim->bodies.empty()
                                  && _orbitsim->reference_body() != nullptr;

        if (use_orbitsim)
        {
            _orbitsim->sim.step(static_cast<double>(fixed_dt));
            sync_celestial_render_entities(ctx);
        }

        auto gravity_accel_world_at = [&](const WorldVec3 &p_world) -> glm::dvec3 {
            if (!use_orbitsim)
            {
                return glm::dvec3(0.0);
            }

            const glm::dvec3 p_rel = glm::dvec3(p_world - cfg.system_center);
            return nbody_accel_body_centered(*_orbitsim, p_rel);
        };

        // Velocity-origin handling:
        //  PerStepAnchorSync: Galilean shift every step to keep anchor v_local near 0.
        //  FreeFallAnchorFrame: integrate velocity-origin from anchor acceleration.
        glm::dvec3 anchor_accel_world(0.0);
        bool have_anchor_accel = false;
        const bool per_step_sync = _velocity_origin_mode == VelocityOriginMode::PerStepAnchorSync;
        const WorldVec3 physics_origin_world = _physics_context->origin_world();

        EntityId anchor_eid = _world.rebase_anchor();
        if (!anchor_eid.is_valid())
        {
            anchor_eid = player_entity();
        }

        if (anchor_eid.is_valid())
        {
            Entity *anchor = _world.entities().find(anchor_eid);
            if (anchor && anchor->has_physics())
            {
                const Physics::BodyId anchor_body{anchor->physics_body_value()};
                if (_physics->is_body_valid(anchor_body))
                {
                    const double dt = static_cast<double>(fixed_dt);
                    if (per_step_sync)
                    {
                        const glm::vec3 v_local_f = _physics->get_linear_velocity(anchor_body);
                        const glm::dvec3 v_world = _physics_context->velocity_origin_world() + glm::dvec3(v_local_f);
                        (void) _physics_context->set_velocity_origin_world(v_world);
                        _physics->shift_velocity_origin(glm::dvec3(v_local_f));
                    }
                    else
                    {
                        const glm::dvec3 p_local_anchor = _physics->get_position(anchor_body);
                        const WorldVec3 p_world_anchor = physics_origin_world + WorldVec3(p_local_anchor);
                        anchor_accel_world = gravity_accel_world_at(p_world_anchor);
                        have_anchor_accel = true;

                        const glm::dvec3 v_origin_next =
                                _physics_context->velocity_origin_world() + anchor_accel_world * dt;
                        (void) _physics_context->set_velocity_origin_world(v_origin_next);
                    }
                }
            }
        }

        const glm::dvec3 frame_accel_world =
                (!per_step_sync && have_anchor_accel) ? anchor_accel_world : glm::dvec3(0.0);

        auto apply_gravity_accel = [&](EntityId id) {
            Entity *ent = _world.entities().find(id);
            if (!ent || !ent->has_physics())
            {
                return;
            }

            const Physics::BodyId body_id{ent->physics_body_value()};
            if (!_physics->is_body_valid(body_id))
            {
                return;
            }

            const glm::dvec3 p_local = _physics->get_position(body_id);
            const WorldVec3 p_world = local_to_world_d(p_local, physics_origin_world);

            const glm::dvec3 a_local = gravity_accel_world_at(p_world) - frame_accel_world;

            glm::vec3 v_local = _physics->get_linear_velocity(body_id);
            v_local += glm::vec3(a_local) * fixed_dt;
            _physics->set_linear_velocity(body_id, v_local);
            _physics->activate(body_id);
        };

        // Apply gravity to all orbiters
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.apply_gravity && orbiter.entity.is_valid())
            {
                apply_gravity_accel(orbiter.entity);
            }
        }

        _physics->step(fixed_dt);

        // Advance moving frame: x_world = x_origin + x_local, d/dt(x_origin) = v_origin.
        const glm::dvec3 v_origin = _physics_context->velocity_origin_world();
        if (finite_vec3(v_origin))
        {
            const double dt = static_cast<double>(fixed_dt);
            (void) _physics_context->set_origin_world(
                    _physics_context->origin_world() + WorldVec3(v_origin * dt));
        }

        _world.post_physics_step();
#endif
    }

    // ---- Time warp ----

    void GameplayState::set_time_warp_level(GameStateContext &ctx, const int level)
    {
        const int clamped = std::clamp(level, 0, TimeWarpState::kMaxWarpLevel);
        const TimeWarpState::Mode old_mode = _time_warp.mode;

        _time_warp.warp_level = clamped;
        _time_warp.mode = _time_warp.mode_for_level(clamped);

        if (old_mode == TimeWarpState::Mode::RailsWarp && _time_warp.mode != TimeWarpState::Mode::RailsWarp)
        {
            exit_rails_warp(ctx);
        }

        if (_time_warp.mode == TimeWarpState::Mode::RailsWarp && old_mode != TimeWarpState::Mode::RailsWarp)
        {
            enter_rails_warp(ctx);
            if (!_rails_warp_active)
            {
                _time_warp.warp_level = TimeWarpState::kMaxPhysicsWarpLevel;
                _time_warp.mode = _time_warp.mode_for_level(_time_warp.warp_level);
            }
        }
    }

    void GameplayState::enter_rails_warp(GameStateContext &ctx)
    {
        if (_rails_warp_active)
        {
            return;
        }

        if (!_orbitsim)
        {
            return;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->reference_sim_body();
        if (!ref_sim)
        {
            return;
        }

        bool have_player_sc = false;
        std::vector<orbitsim::SpacecraftId> created_ids;
        created_ids.reserve(_orbiters.size());

        const bool sas_down = ctx.input && ctx.input->key_down(Key::T);

        for (auto &orbiter : _orbiters)
        {
            orbiter.rails.clear();

            if (!orbiter.entity.is_valid())
            {
                continue;
            }

            Entity *ent = _world.entities().find(orbiter.entity);
            if (!ent)
            {
                continue;
            }

            WorldVec3 pos_world = ent->position_world();
            glm::dvec3 vel_world(0.0);
            glm::quat rot = ent->rotation();
            glm::vec3 ang_vel_world(0.0f);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics && _physics_context && ent->has_physics())
            {
                const Physics::BodyId body_id{ent->physics_body_value()};
                if (_physics->is_body_valid(body_id))
                {
                    pos_world = local_to_world_d(_physics->get_position(body_id), _physics_context->origin_world());
                    const glm::vec3 v_local_f = _physics->get_linear_velocity(body_id);
                    vel_world = _physics_context->velocity_origin_world() + glm::dvec3(v_local_f);
                    rot = _physics->get_rotation(body_id);
                    ang_vel_world = _physics->get_angular_velocity(body_id);
                }
            }
#endif

            const glm::dvec3 rel_pos_m = glm::dvec3(pos_world - _scenario_config.system_center);
            const glm::dvec3 rel_vel_mps = vel_world;

            orbitsim::Spacecraft sc{};
            sc.state = orbitsim::make_state(ref_sim->state.position_m + rel_pos_m,
                                            ref_sim->state.velocity_mps + rel_vel_mps);
            sc.dry_mass_kg = std::max(1.0, orbiter.mass_kg);

            const auto handle = _orbitsim->sim.create_spacecraft(std::move(sc));
            if (!handle.valid())
            {
                continue;
            }

            orbiter.rails.sc_id = handle.id;
            orbiter.rails.rotation = rot;
            orbiter.rails.angular_velocity_radps = ang_vel_world;
            orbiter.rails.sas_enabled = false;
            orbiter.rails.sas_toggle_prev_down = sas_down;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (orbiter.is_player)
            {
                if (auto *sc_comp = ent->get_component<ShipController>())
                {
                    orbiter.rails.sas_enabled = sc_comp->sas_enabled();
                }
            }
#endif

            created_ids.push_back(handle.id);

            if (orbiter.is_player)
            {
                have_player_sc = true;
            }
        }

        if (!have_player_sc)
        {
            for (orbitsim::SpacecraftId id : created_ids)
            {
                (void) _orbitsim->sim.remove_spacecraft(id);
            }
            for (auto &orbiter : _orbiters)
            {
                orbiter.rails.clear();
            }
            return;
        }

        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);
        _rails_thrust_applied_this_tick = false;
        _rails_warp_active = true;
    }

    void GameplayState::exit_rails_warp(GameStateContext &ctx)
    {
        if (!_rails_warp_active)
        {
            return;
        }

        if (!_orbitsim)
        {
            for (auto &orbiter : _orbiters)
            {
                orbiter.rails.clear();
            }
            _rails_warp_active = false;
            return;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->reference_sim_body();
        if (!ref_sim)
        {
            for (auto &orbiter : _orbiters)
            {
                if (orbiter.rails.active())
                {
                    (void) _orbitsim->sim.remove_spacecraft(orbiter.rails.sc_id);
                }
                orbiter.rails.clear();
            }
            _rails_warp_active = false;
            return;
        }

        // Choose a stable origin/velocity origin for Jolt when resuming.
        WorldVec3 anchor_pos_world = _scenario_config.system_center;
        glm::dvec3 anchor_vel_world(0.0);

        {
            const EntityId anchor_eid = select_rebase_anchor_entity();
            const OrbiterInfo *anchor_orbiter = nullptr;
            for (const auto &o : _orbiters)
            {
                if (o.entity == anchor_eid)
                {
                    anchor_orbiter = &o;
                    break;
                }
            }

            if (!anchor_orbiter)
            {
                anchor_orbiter = find_player_orbiter();
            }

            if (anchor_orbiter && anchor_orbiter->rails.active())
            {
                if (const orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(anchor_orbiter->rails.sc_id))
                {
                    anchor_pos_world = _scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                    anchor_vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
                }
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics_context)
        {
            (void) _physics_context->set_origin_world(anchor_pos_world);
            (void) _physics_context->set_velocity_origin_world(anchor_vel_world);
        }
#endif

        for (auto &orbiter : _orbiters)
        {
            if (!orbiter.rails.active())
            {
                continue;
            }

            const orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(orbiter.rails.sc_id);
            if (!sc)
            {
                continue;
            }

            const WorldVec3 pos_world = _scenario_config.system_center +
                    WorldVec3(sc->state.position_m - ref_sim->state.position_m);
            const glm::dvec3 vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
            const glm::quat rot = orbiter.rails.rotation;

            Entity *ent = _world.entities().find(orbiter.entity);
            if (ent)
            {
                ent->set_position_world(pos_world);
                ent->set_rotation(rot);
                if (ent->uses_interpolation())
                {
                    ent->interpolation().set_immediate(pos_world, rot);
                }
            }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics && _physics_context && ent && ent->has_physics())
            {
                const Physics::BodyId body_id{ent->physics_body_value()};
                if (_physics->is_body_valid(body_id))
                {
                    const glm::dvec3 pos_local = world_to_local_d(pos_world, _physics_context->origin_world());
                    _physics->set_transform(body_id, pos_local, rot);

                    const glm::dvec3 v_origin_world = _physics_context->velocity_origin_world();
                    const glm::dvec3 v_local_d = vel_world - v_origin_world;
                    _physics->set_linear_velocity(body_id, glm::vec3(v_local_d));
                    _physics->set_angular_velocity(body_id, orbiter.rails.angular_velocity_radps);
                    _physics->activate(body_id);
                }
            }

            if (orbiter.is_player && ent)
            {
                if (auto *sc_comp = ent->get_component<ShipController>())
                {
                    sc_comp->set_sas_enabled(orbiter.rails.sas_enabled);
                    const bool sas_down = ctx.input && ctx.input->key_down(Key::T);
                    sc_comp->set_sas_toggle_prev_down(sas_down);
                }
            }
#endif
        }

        for (auto &orbiter : _orbiters)
        {
            if (orbiter.rails.active())
            {
                (void) _orbitsim->sim.remove_spacecraft(orbiter.rails.sc_id);
            }
            orbiter.rails.clear();
        }

        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);
        _rails_thrust_applied_this_tick = false;
        _rails_warp_active = false;
    }

    void GameplayState::rails_warp_step(GameStateContext &ctx, const double dt_s)
    {
        _rails_thrust_applied_this_tick = false;
        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);

        if (!_orbitsim)
        {
            return;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->reference_sim_body();
        if (!ref_sim)
        {
            return;
        }

        OrbiterInfo *player_orbiter = nullptr;
        for (auto &o : _orbiters)
        {
            if (o.is_player)
            {
                player_orbiter = &o;
                break;
            }
        }

        const bool ui_capture_keyboard = build_component_context(ctx).ui_capture_keyboard;

        float thrust_force_N = 0.0f;
        float torque_strength = 0.0f;
        float sas_damping = 0.0f;

        if (player_orbiter && player_orbiter->rails.active())
        {
            Entity *player_ent = _world.entities().find(player_orbiter->entity);

            if (player_ent)
            {
                if (auto *sc_comp = player_ent->get_component<ShipController>())
                {
                    thrust_force_N = sc_comp->thrust_force();
                    torque_strength = sc_comp->torque_strength();
                    sas_damping = sc_comp->sas_damping();
                }
            }

            ThrustInput input = ShipController::read_input(ctx.input, ui_capture_keyboard,
                                                           player_orbiter->rails.sas_toggle_prev_down);
            if (input.sas_toggled)
            {
                player_orbiter->rails.sas_enabled = !player_orbiter->rails.sas_enabled;
                if (player_ent)
                {
                    if (auto *sc_comp = player_ent->get_component<ShipController>())
                    {
                        sc_comp->set_sas_enabled(player_orbiter->rails.sas_enabled);
                    }
                }
            }

            _rails_last_thrust_dir_local = input.local_thrust_dir;
            _rails_last_torque_dir_local = input.local_torque_dir;

            glm::vec3 world_torque_dir(0.0f);
            if (glm::length(input.local_torque_dir) > 0.0f)
            {
                world_torque_dir = player_orbiter->rails.rotation * input.local_torque_dir;
            }

            update_rails_rotation(player_orbiter->rails,
                                  world_torque_dir,
                                  torque_strength,
                                  sas_damping,
                                  player_orbiter->rails.sas_enabled,
                                  dt_s);

            const bool has_thrust = glm::length(input.local_thrust_dir) > 0.0f;
            if (has_thrust && thrust_force_N > 0.0f)
            {
                orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(player_orbiter->rails.sc_id);
                if (sc)
                {
                    const glm::dvec3 dir_world = glm::dvec3(player_orbiter->rails.rotation * input.local_thrust_dir);
                    const double mass_kg = std::max(1.0, sc->mass_kg());
                    const double dv = (static_cast<double>(thrust_force_N) / mass_kg) * dt_s;
                    sc->state.velocity_mps += dir_world * dv;
                    _rails_thrust_applied_this_tick = true;
                    mark_prediction_dirty();
                }
            }
        }

        _orbitsim->sim.step(dt_s);
        sync_celestial_render_entities(ctx);

        // Sync spacecraft states to entities.
        for (auto &orbiter : _orbiters)
        {
            if (!orbiter.rails.active() || !orbiter.entity.is_valid())
            {
                continue;
            }

            if (!orbiter.is_player)
            {
                update_rails_rotation(orbiter.rails,
                                      glm::vec3(0.0f),
                                      0.0f,
                                      0.0f,
                                      false,
                                      dt_s);
            }

            const orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(orbiter.rails.sc_id);
            if (!sc)
            {
                continue;
            }

            const WorldVec3 pos_world = _scenario_config.system_center +
                    WorldVec3(sc->state.position_m - ref_sim->state.position_m);

            Entity *ent = _world.entities().find(orbiter.entity);
            if (!ent)
            {
                continue;
            }

            if (ent->uses_interpolation())
            {
                ent->interpolation().store_current_as_previous();
                ent->interpolation().curr_position = pos_world;
                ent->interpolation().curr_rotation = orbiter.rails.rotation;
            }

            ent->set_position_world(pos_world);
            ent->set_rotation(orbiter.rails.rotation);
        }
    }


} // namespace Game
