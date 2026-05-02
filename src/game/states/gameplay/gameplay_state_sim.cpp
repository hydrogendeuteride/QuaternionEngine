#include "gameplay_state.h"
#include "orbit_helpers.h"
#include "core/game_api.h"
#include "core/input/input_system.h"
#include "game/component/ship_controller.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"
#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/frame_utils.hpp"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

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
        WorldVec3 body_world_from_body_local(const glm::dvec3 &body_position_local,
                                             const WorldVec3 &physics_origin_world)
        {
            return local_to_world_d(body_position_local, physics_origin_world);
        }

        glm::dvec3 body_local_from_body_world(const WorldVec3 &body_position_world,
                                              const WorldVec3 &physics_origin_world)
        {
            return world_to_local_d(body_position_world, physics_origin_world);
        }

        WorldVec3 entity_world_from_body_world(const Entity &entity,
                                               const WorldVec3 &body_position_world,
                                               const glm::quat &rotation)
        {
            return entity.entity_position_from_physics_center_of_mass_world(body_position_world, rotation);
        }

        WorldVec3 body_world_from_entity_world(const Entity &entity,
                                               const WorldVec3 &entity_position_world,
                                               const glm::quat &rotation)
        {
            return entity.physics_center_of_mass_world(entity_position_world, rotation);
        }

        // Integrates rails-warp angular velocity and orientation using gameplay-tuned torque/SAS rules.
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

    // Updates render/world-space celestial body transforms from the orbit simulation frame.
    void GameplayState::sync_celestial_render_entities(GameStateContext &ctx)
    {
        if (!_orbitsim || _orbitsim->bodies.empty())
        {
            return;
        }

        const auto &cfg = _scenario_config;

        const CelestialBodyInfo *ref_info = _orbitsim->world_reference_body();
        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
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

            // Convert barycentric simulation coordinates into the gameplay world frame centered on the reference body.
            const WorldVec3 body_pos_world =
                    cfg.system_center + WorldVec3(sim_body->state.position_m - ref_sim->state.position_m);

            if (ctx.api)
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

    void GameplayState::update_runtime_orbiter_rails()
    {
        if (_rails_warp_active)
        {
            return;
        }

        const bool can_run_runtime_rails =
                _runtime_orbiter_rails_enabled &&
                _runtime_orbiter_rails_distance_m > 0.0 &&
                _orbitsim &&
                _physics &&
                _physics_context &&
                _orbitsim->world_reference_sim_body();

        const EntityId anchor_eid = select_rebase_anchor_entity();
        const Entity *anchor_entity = _world.entities().find(anchor_eid);
        const WorldVec3 anchor_pos_world =
                anchor_entity ? anchor_entity->physics_center_of_mass_world() : _scenario_config.system_center;

        const double promote_distance_m = std::max(0.0, _runtime_orbiter_rails_distance_m);
        const double return_distance_m = promote_distance_m * kRuntimeOrbiterRailsReturnDistanceRatio;

        for (auto &orbiter : _orbiters)
        {
            if (orbiter.is_player || !orbiter.entity.is_valid())
            {
                continue;
            }

            WorldVec3 orbiter_pos_world{0.0, 0.0, 0.0};
            if (orbiter.rails.active() && _orbitsim)
            {
                const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
                const orbitsim::Spacecraft *sc = ref_sim ? _orbitsim->sim.spacecraft_by_id(orbiter.rails.sc_id) : nullptr;
                if (ref_sim && sc)
                {
                    orbiter_pos_world = _scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                }
                else if (const Entity *entity = _world.entities().find(orbiter.entity))
                {
                    orbiter_pos_world = entity->physics_center_of_mass_world();
                }
            }
            else if (const Entity *entity = _world.entities().find(orbiter.entity))
            {
                orbiter_pos_world = entity->physics_center_of_mass_world();
            }

            const double distance_m = glm::length(glm::dvec3(orbiter_pos_world - anchor_pos_world));

            if (!can_run_runtime_rails)
            {
                if (orbiter.rails.active())
                {
                    (void) demote_orbiter_from_rails(orbiter);
                }
                continue;
            }

            if (orbiter.rails.active())
            {
                if (distance_m <= return_distance_m)
                {
                    (void) demote_orbiter_from_rails(orbiter);
                }
                continue;
            }

            if (distance_m >= promote_distance_m)
            {
                (void) promote_orbiter_to_rails(orbiter);
            }
        }
    }

    void GameplayState::sync_runtime_orbiter_rails(const double dt_s)
    {
        if (_rails_warp_active || !_orbitsim)
        {
            return;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!ref_sim)
        {
            return;
        }

        for (auto &orbiter : _orbiters)
        {
            if (!orbiter.rails.active() || !orbiter.entity.is_valid())
            {
                continue;
            }

            const orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(orbiter.rails.sc_id);
            Entity *ent = _world.entities().find(orbiter.entity);
            if (!sc || !ent)
            {
                continue;
            }

            update_rails_rotation(orbiter.rails,
                                  glm::vec3(0.0f),
                                  0.0f,
                                  0.0f,
                                  false,
                                  dt_s);

            const WorldVec3 body_pos_world = _scenario_config.system_center +
                    WorldVec3(sc->state.position_m - ref_sim->state.position_m);
            const WorldVec3 entity_pos_world =
                    entity_world_from_body_world(*ent, body_pos_world, orbiter.rails.rotation);

            ent->set_position_world(entity_pos_world);
            ent->set_rotation(orbiter.rails.rotation);
            if (ent->uses_interpolation())
            {
                ent->interpolation().curr_position = entity_pos_world;
                ent->interpolation().curr_rotation = orbiter.rails.rotation;
            }
        }
    }

    bool GameplayState::promote_orbiter_to_rails(OrbiterInfo &orbiter)
    {
        if (_rails_warp_active || !_orbitsim || !_physics_context || !orbiter.entity.is_valid() || orbiter.rails.active())
        {
            return false;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        Entity *ent = _world.entities().find(orbiter.entity);
        if (!ref_sim || !ent || !ent->has_physics() || !_physics)
        {
            return false;
        }

        const Physics::BodyId body_id{ent->physics_body_value()};
        if (!_physics->is_body_valid(body_id))
        {
            return false;
        }

        const glm::quat rot = _physics->get_rotation(body_id);
        const WorldVec3 body_origin_world =
                body_world_from_body_local(_physics->get_position(body_id), _physics_context->origin_world());
        const WorldVec3 body_pos_world = body_world_from_entity_world(*ent, body_origin_world, rot);
        const glm::dvec3 vel_world = _physics_context->velocity_origin_world() +
                                     glm::dvec3(_physics->get_linear_velocity(body_id));
        const glm::vec3 ang_vel = _physics->get_angular_velocity(body_id);

        orbitsim::Spacecraft sc{};
        sc.state = orbitsim::make_state(ref_sim->state.position_m + glm::dvec3(body_pos_world - _scenario_config.system_center),
                                        ref_sim->state.velocity_mps + vel_world);
        sc.dry_mass_kg = std::max(1.0, orbiter.mass_kg);

        const auto handle = _orbitsim->sim.create_spacecraft(std::move(sc));
        if (!handle.valid())
        {
            return false;
        }

        (void) destroy_orbiter_physics_body(orbiter.render_is_gltf, *ent);

        orbiter.rails.sc_id = handle.id;
        orbiter.rails.rotation = rot;
        orbiter.rails.angular_velocity_radps = ang_vel;
        orbiter.rails.sas_enabled = false;
        orbiter.rails.sas_toggle_prev_down = false;

        ent->set_position_world(entity_world_from_body_world(*ent, body_pos_world, rot));
        ent->set_rotation(rot);
        if (ent->uses_interpolation())
        {
            ent->interpolation().set_immediate(ent->position_world(), rot);
        }
        return true;
    }

    bool GameplayState::demote_orbiter_from_rails(OrbiterInfo &orbiter)
    {
        if (_rails_warp_active || !_orbitsim || !_physics || !_physics_context || !orbiter.rails.active() ||
            !orbiter.entity.is_valid())
        {
            return false;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        const orbitsim::Spacecraft *sc = ref_sim ? _orbitsim->sim.spacecraft_by_id(orbiter.rails.sc_id) : nullptr;
        Entity *ent = _world.entities().find(orbiter.entity);
        if (!ref_sim || !sc || !ent)
        {
            return false;
        }

        const WorldVec3 body_pos_world = _scenario_config.system_center +
                WorldVec3(sc->state.position_m - ref_sim->state.position_m);
        const glm::dvec3 vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
        const glm::quat rot = orbiter.rails.rotation;
        const WorldVec3 entity_pos_world = entity_world_from_body_world(*ent, body_pos_world, rot);

        Physics::BodyId body_id{};
        if (ent->has_physics())
        {
            body_id = Physics::BodyId{ent->physics_body_value()};
        }
        if (!body_id.is_valid() || !_physics->is_body_valid(body_id))
        {
            if (ent->has_physics())
            {
                ent->clear_physics_body();
            }

            glm::vec3 origin_offset_local{0.0f, 0.0f, 0.0f};
            body_id = create_orbiter_physics_body(orbiter.render_is_gltf,
                                                  *ent,
                                                  orbiter.physics_settings,
                                                  entity_pos_world,
                                                  rot,
                                                  &origin_offset_local);
            if (!body_id.is_valid() ||
                !_world.bind_physics(ent->id(),
                                     body_id.value,
                                     orbiter.use_physics_interpolation,
                                     false,
                                     origin_offset_local))
            {
                return false;
            }
        }

        _physics->set_transform(body_id,
                                body_local_from_body_world(entity_pos_world, _physics_context->origin_world()),
                                rot);
        _physics->set_linear_velocity(body_id, glm::vec3(vel_world - _physics_context->velocity_origin_world()));
        _physics->set_angular_velocity(body_id, orbiter.rails.angular_velocity_radps);
        _physics->activate(body_id);

        ent->set_position_world(entity_pos_world);
        ent->set_rotation(rot);
        if (ent->uses_interpolation())
        {
            ent->interpolation().set_immediate(entity_pos_world, rot);
        }

        (void) _orbitsim->sim.remove_spacecraft(orbiter.rails.sc_id);
        orbiter.rails.clear();
        return true;
    }

    bool GameplayState::get_orbiter_inertial_state(const OrbiterInfo &orbiter, orbitsim::State &out_state) const
    {
        out_state = {};
        if (!_orbitsim)
        {
            return false;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!ref_sim)
        {
            return false;
        }

        WorldVec3 pos_world{0.0};
        glm::dvec3 vel_world{0.0};
        glm::vec3 vel_local{0.0f};
        if (!GameplayPredictionAdapter(*this).get_orbiter_world_state(orbiter, pos_world, vel_world, vel_local))
        {
            return false;
        }

        out_state = orbitsim::make_state(
                ref_sim->state.position_m + glm::dvec3(pos_world - _scenario_config.system_center),
                ref_sim->state.velocity_mps + vel_world);
        return true;
    }

    const orbitsim::MassiveBody *GameplayState::select_primary_body_for_state(const orbitsim::State &state) const
    {
        if (!_orbitsim || _orbitsim->sim.massive_bodies().empty())
        {
            return nullptr;
        }

        const auto body_position_at = [this](const std::size_t i) -> orbitsim::Vec3 {
            return _orbitsim->sim.massive_bodies()[i].state.position_m;
        };

        const std::size_t primary_index = orbitsim::auto_select_primary_index(
                _orbitsim->sim.massive_bodies(),
                state.position_m,
                body_position_at,
                _orbitsim->sim.config().softening_length_m);
        if (primary_index >= _orbitsim->sim.massive_bodies().size())
        {
            return nullptr;
        }

        return &_orbitsim->sim.massive_bodies()[primary_index];
    }

    bool GameplayState::build_orbiter_lvlh_frame(const OrbiterInfo &leader,
                                                 orbitsim::RotatingFrame &out_frame,
                                                 orbitsim::State *out_leader_state,
                                                 orbitsim::State *out_primary_state) const
    {
        out_frame = {};

        orbitsim::State leader_state{};
        if (!get_orbiter_inertial_state(leader, leader_state))
        {
            return false;
        }

        const orbitsim::MassiveBody *primary = select_primary_body_for_state(leader_state);
        if (!primary)
        {
            return false;
        }

        const std::optional<orbitsim::RotatingFrame> lvlh =
                orbitsim::make_lvlh_frame(primary->state, leader_state);
        if (!lvlh.has_value() || !lvlh->valid())
        {
            return false;
        }

        out_frame = *lvlh;
        if (out_leader_state)
        {
            *out_leader_state = leader_state;
        }
        if (out_primary_state)
        {
            *out_primary_state = primary->state;
        }
        return true;
    }

    void GameplayState::update_formation_hold(const double dt_s)
    {
        if (!(dt_s > 0.0) || !std::isfinite(dt_s) || !_orbitsim)
        {
            return;
        }

        for (auto &orbiter : _orbiters)
        {
            if (!orbiter.formation_hold_enabled || orbiter.formation_leader_name.empty() || orbiter.is_player)
            {
                continue;
            }

            const OrbiterInfo *leader = find_orbiter(std::string_view(orbiter.formation_leader_name));
            if (!leader || leader == &orbiter)
            {
                continue;
            }

            orbitsim::RotatingFrame leader_lvlh{};
            if (!build_orbiter_lvlh_frame(*leader, leader_lvlh))
            {
                continue;
            }

            orbitsim::State follower_state_inertial{};
            if (!get_orbiter_inertial_state(orbiter, follower_state_inertial))
            {
                continue;
            }

            const orbitsim::State follower_state_lvlh =
                    orbitsim::inertial_state_to_frame(follower_state_inertial, leader_lvlh);

            // Position error and velocity in LVLH frame.
            const glm::dvec3 pos_error =
                    glm::dvec3(follower_state_lvlh.position_m) - orbiter.formation_slot_lvlh_m;
            const glm::dvec3 vel = glm::dvec3(follower_state_lvlh.velocity_mps);

            // Exact integration of a critically-damped spring (unconditionally stable for any dt).
            //   e'' = -omega^2 * e - 2*omega * e'
            //   e(t)  = (e0 + (v0 + omega*e0)*t) * exp(-omega*t)
            //   e'(t) = (v0*(1 - omega*t) - omega^2*e0*t) * exp(-omega*t)
            const double omega = kFormationHoldOmega;
            const double odt = omega * dt_s;
            const double exp_decay = std::exp(-odt);
            const glm::dvec3 vel_after =
                    (vel * (1.0 - odt) - (omega * omega * pos_error * dt_s)) * exp_decay;
            glm::dvec3 dv_lvlh = vel_after - vel;

            if (!finite_vec3(dv_lvlh))
            {
                continue;
            }

            const double dv_len = glm::length(dv_lvlh);
            if (std::isfinite(dv_len) && dv_len > kFormationHoldMaxDvPerStepMps && dv_len > 0.0)
            {
                dv_lvlh *= (kFormationHoldMaxDvPerStepMps / dv_len);
            }

            const glm::dvec3 dv_world =
                    orbitsim::frame_vector_to_inertial(leader_lvlh, dv_lvlh);
            if (!finite_vec3(dv_world))
            {
                continue;
            }

            if (orbiter.rails.active())
            {
                orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(orbiter.rails.sc_id);
                if (!sc)
                {
                    continue;
                }

                sc->state.velocity_mps += dv_world;
                continue;
            }

            if (!_physics || !_physics_context || !orbiter.entity.is_valid())
            {
                continue;
            }

            Entity *entity = _world.entities().find(orbiter.entity);
            if (!entity || !entity->has_physics())
            {
                continue;
            }

            const Physics::BodyId body_id{entity->physics_body_value()};
            if (!_physics->is_body_valid(body_id))
            {
                continue;
            }

            // For physics mode, convert dv into an equivalent force for this tick.
            const double mass_kg = std::max(1.0, orbiter.mass_kg);
            const glm::vec3 force = glm::vec3(dv_world * mass_kg / dt_s);
            _physics->add_force(body_id, force);
            _physics->activate(body_id);
        }
    }

    // Advances one fixed-step physics tick, including orbit sim gravity and moving-origin bookkeeping.
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
                                  && _orbitsim->world_reference_body() != nullptr;

        if (use_orbitsim)
        {
            update_runtime_orbiter_rails();
            _orbitsim->sim.step(static_cast<double>(fixed_dt));
            sync_celestial_render_entities(ctx);
            sync_runtime_orbiter_rails(static_cast<double>(fixed_dt));
        }

        // Sample gravity in gameplay world coordinates by converting into the reference-body-centered orbit frame.
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
                        // Shift the velocity origin so the anchor stays near rest in local physics space.
                        const glm::vec3 v_local_f = _physics->get_linear_velocity(anchor_body);
                        const glm::dvec3 v_world = _physics_context->velocity_origin_world() + glm::dvec3(v_local_f);
                        (void) _physics_context->set_velocity_origin_world(v_world);
                        _physics->shift_velocity_origin(glm::dvec3(v_local_f));
                    }
                    else
                    {
                        // Make the moving frame free-fall with the anchor so nearby objects see only relative gravity.
                        const glm::dvec3 p_local_anchor = _physics->get_position(anchor_body);
                        const glm::quat anchor_rotation = _physics->get_rotation(anchor_body);
                        const WorldVec3 body_origin_world = physics_origin_world + WorldVec3(p_local_anchor);
                        const WorldVec3 p_world_anchor =
                                anchor->physics_center_of_mass_world(body_origin_world, anchor_rotation);
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

        // Applies gravity as a velocity update in the local physics frame used by Jolt.
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
            const glm::quat rotation = _physics->get_rotation(body_id);
            const WorldVec3 body_origin_world = body_world_from_body_local(p_local, physics_origin_world);
            const WorldVec3 p_world = ent->physics_center_of_mass_world(body_origin_world, rotation);

            // In a free-fall frame we subtract the frame acceleration so only relative/tidal acceleration remains.
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

    // Changes warp level and performs rails-warp enter/exit transitions when the mode boundary is crossed.
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

    // Converts active orbiters from physics bodies into lightweight orbit-sim spacecraft for high-rate warp.
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

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
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
            WorldVec3 body_pos_world{0.0, 0.0, 0.0};
            glm::dvec3 vel_world(0.0);
            glm::quat rot{1.0f, 0.0f, 0.0f, 0.0f};
            glm::vec3 ang_vel_world(0.0f);
            bool have_state_snapshot = false;

            if (orbiter.rails.active())
            {
                if (const orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(orbiter.rails.sc_id))
                {
                    body_pos_world = _scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                    vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
                    rot = orbiter.rails.rotation;
                    ang_vel_world = orbiter.rails.angular_velocity_radps;
                    have_state_snapshot = true;
                }
                (void) _orbitsim->sim.remove_spacecraft(orbiter.rails.sc_id);
                orbiter.rails.clear();
            }

            if (!orbiter.entity.is_valid())
            {
                continue;
            }

            Entity *ent = _world.entities().find(orbiter.entity);
            if (!ent)
            {
                continue;
            }

            if (!have_state_snapshot)
            {
                body_pos_world = body_world_from_entity_world(*ent, ent->position_world(), ent->rotation());
                rot = ent->rotation();
            }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics && _physics_context && ent->has_physics())
            {
                const Physics::BodyId body_id{ent->physics_body_value()};
                if (_physics->is_body_valid(body_id))
                {
                    rot = _physics->get_rotation(body_id);
                    const WorldVec3 body_origin_world =
                            body_world_from_body_local(_physics->get_position(body_id), _physics_context->origin_world());
                    body_pos_world = body_world_from_entity_world(*ent, body_origin_world, rot);
                    const glm::vec3 v_local_f = _physics->get_linear_velocity(body_id);
                    vel_world = _physics_context->velocity_origin_world() + glm::dvec3(v_local_f);
                    ang_vel_world = _physics->get_angular_velocity(body_id);
                }
            }
#endif

            // Convert gameplay world state into barycentric spacecraft state relative to the reference body.
            const glm::dvec3 rel_pos_m = glm::dvec3(body_pos_world - _scenario_config.system_center);
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

    // Restores orbiters from rails spacecraft back into entity/physics state when leaving rails warp.
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

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
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
            // Pick a resume anchor so the restored physics origin stays near a meaningful nearby body.
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

            const WorldVec3 body_pos_world = _scenario_config.system_center +
                    WorldVec3(sc->state.position_m - ref_sim->state.position_m);
            const glm::dvec3 vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
            const glm::quat rot = orbiter.rails.rotation;

            Entity *ent = _world.entities().find(orbiter.entity);
            const WorldVec3 entity_pos_world =
                    ent ? entity_world_from_body_world(*ent, body_pos_world, rot) : WorldVec3(0.0);
            if (ent)
            {
                ent->set_position_world(entity_pos_world);
                ent->set_rotation(rot);
                if (ent->uses_interpolation())
                {
                    ent->interpolation().set_immediate(entity_pos_world, rot);
                }
            }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics && _physics_context && ent)
            {
                Physics::BodyId body_id{};
                if (ent->has_physics())
                {
                    body_id = Physics::BodyId{ent->physics_body_value()};
                }
                if (!body_id.is_valid() || !_physics->is_body_valid(body_id))
                {
                    if (ent->has_physics())
                    {
                        ent->clear_physics_body();
                    }

                    glm::vec3 origin_offset_local{0.0f, 0.0f, 0.0f};
                    body_id = create_orbiter_physics_body(orbiter.render_is_gltf,
                                                          *ent,
                                                          orbiter.physics_settings,
                                                          entity_pos_world,
                                                          rot,
                                                          &origin_offset_local);
                    if (body_id.is_valid())
                    {
                        (void) _world.bind_physics(ent->id(),
                                                   body_id.value,
                                                   orbiter.use_physics_interpolation,
                                                   false,
                                                   origin_offset_local);
                    }
                }

                if (_physics->is_body_valid(body_id))
                {
                    // Re-express world-space warp results in the local moving frame used by the physics world.
                    const glm::dvec3 pos_local = body_local_from_body_world(entity_pos_world, _physics_context->origin_world());
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

    // Runs one rails-warp tick: player input, orbit-sim advancement, and entity transform sync.
    void GameplayState::rails_warp_step(GameStateContext &ctx, const double dt_s)
    {
        _rails_thrust_applied_this_tick = false;
        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);

        if (!_orbitsim)
        {
            return;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
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

            // Reuse ship-controller bindings, but apply them directly to the rails state instead of rigid-body physics.
            ThrustInput input = ShipController::read_input(ctx.input, &_keybinds.ship, ui_capture_keyboard,
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
                    // Rails thrust is modeled as an instantaneous dv on the spacecraft state for this tick.
                    const glm::dvec3 dir_world = glm::dvec3(player_orbiter->rails.rotation * input.local_thrust_dir);
                    const double mass_kg = std::max(1.0, sc->mass_kg());
                    const double dv = (static_cast<double>(thrust_force_N) / mass_kg) * dt_s;
                    const glm::dvec3 dv_world = dir_world * dv;
                    sc->state.velocity_mps += dv_world;
                    _rails_thrust_applied_this_tick = true;
                    mark_prediction_dirty();

                    // Propagate the same velocity change to formation followers.
                    for (auto &follower : _orbiters)
                    {
                        if (!follower.formation_hold_enabled || follower.is_player)
                        {
                            continue;
                        }
                        if (follower.formation_leader_name != player_orbiter->name)
                        {
                            continue;
                        }
                        if (!follower.rails.active())
                        {
                            continue;
                        }
                        if (orbitsim::Spacecraft *fsc = _orbitsim->sim.spacecraft_by_id(follower.rails.sc_id))
                        {
                            fsc->state.velocity_mps += dv_world;
                        }
                    }
                }
            }
        }

        update_formation_hold(dt_s);

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

            const WorldVec3 body_pos_world = _scenario_config.system_center +
                    WorldVec3(sc->state.position_m - ref_sim->state.position_m);

            Entity *ent = _world.entities().find(orbiter.entity);
            if (!ent)
            {
                continue;
            }

            const WorldVec3 entity_pos_world =
                    entity_world_from_body_world(*ent, body_pos_world, orbiter.rails.rotation);

            if (ent->uses_interpolation())
            {
                ent->interpolation().store_current_as_previous();
                ent->interpolation().curr_position = entity_pos_world;
                ent->interpolation().curr_rotation = orbiter.rails.rotation;
            }

            ent->set_position_world(entity_pos_world);
            ent->set_rotation(orbiter.rails.rotation);
        }
    }


} // namespace Game
