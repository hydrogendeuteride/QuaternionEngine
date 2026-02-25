#include "gameplay_state.h"
#include "orbit_helpers.h"
#include "game/component/ship_controller.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/util/logger.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

#include <algorithm>
#include <cmath>
#include <string>

namespace Game
{
    using detail::contact_event_type_name;
    using detail::circular_orbit_relative_state_xz;
    using detail::two_body_circular_barycentric_xz;

    // ---- Default scenario ----

    ScenarioConfig default_earth_moon_config()
    {
        ScenarioConfig cfg;
        cfg.system_center = WorldVec3{0.0, 0.0, 0.0};
        cfg.speed_scale = 1.0;
        cfg.mu_base = 3.986004418e14;

        // Earth (reference body — index 0)
        {
            ScenarioConfig::CelestialDef earth{};
            earth.name = "earth";
            earth.mass_kg = 5.972e24;
            earth.radius_m = 6'371'000.0;
            earth.atmosphere_top_m = 100'000.0;
            earth.terrain_max_m = 8'848.0;
            earth.soi_radius_m = 9.24e8;
            earth.orbit_distance_m = 0.0; // this IS the reference body
            earth.has_terrain = true;
            earth.albedo_dir = "planets/earth/albedo/L0";
            earth.height_dir = "planets/earth/height/L0";
            earth.height_max_m = 8000.0;
            earth.emission_dir = "planets/earth/emission/L0";
            earth.emission_factor = glm::vec3(2.0f, 2.0f, 2.0f);
            cfg.celestials.push_back(std::move(earth));
        }

        // Moon
        {
            ScenarioConfig::CelestialDef moon{};
            moon.name = "moon";
            moon.mass_kg = 7.342e22;
            moon.radius_m = 1'737'400.0;
            moon.soi_radius_m = 6.61e7;
            moon.orbit_distance_m = 384'400'000.0;
            moon.render_scale = 150'000.0f;
            cfg.celestials.push_back(std::move(moon));
        }

        // Ship (player)
        {
            ScenarioConfig::OrbiterDef ship{};
            ship.name = "ship";
            ship.orbit_altitude_m = 400'000.0;
            ship.is_player = true;
            ship.is_rebase_anchor = true;
            ship.primitive = GameAPI::PrimitiveType::Capsule;

            constexpr float ship_radius_m = 2.0f;
            constexpr float ship_half_height_m = 2.0f;
            constexpr float ship_uniform_scale = ship_radius_m / 0.5f;
            ship.render_scale = glm::vec3(ship_uniform_scale);

            ship.body_settings
                    .set_shape(Physics::CollisionShape::Capsule(ship_radius_m, ship_half_height_m))
                    .set_dynamic()
                    .set_layer(Physics::Layer::Player)
                    .set_gravity_scale(0.0f)
                    .set_friction(0.2f)
                    .set_restitution(0.05f)
                    .set_linear_damping(0.0f)
                    .set_angular_damping(0.0f)
                    .set_mass(10000.0f);

            cfg.orbiters.push_back(std::move(ship));
        }

        // Probe
        {
            ScenarioConfig::OrbiterDef probe{};
            probe.name = "probe";
            probe.is_player = false;
            probe.offset_from_player = glm::dvec3(0.0, -2.0, 30.0);
            probe.relative_velocity = glm::dvec3(0.0, 0.0, -10.0);
            probe.primitive = GameAPI::PrimitiveType::Sphere;
            probe.render_scale = glm::vec3(2.0f);

            probe.body_settings
                    .set_shape(Physics::CollisionShape::Sphere(1.0f))
                    .set_dynamic()
                    .set_layer(Physics::Layer::Dynamic)
                    .set_gravity_scale(0.0f)
                    .set_friction(0.2f)
                    .set_restitution(0.1f)
                    .set_linear_damping(0.0f)
                    .set_angular_damping(0.0f)
                    .set_mass(1000.0f);

            cfg.orbiters.push_back(std::move(probe));
        }

        return cfg;
    }

    // ---- Scene setup ----

    void GameplayState::setup_scene(GameStateContext &ctx)
    {
        _elapsed = 0.0f;
        _fixed_time_s = 0.0;
        _reset_requested = false;
        _contact_log.clear();
        _prediction_altitude_km.clear();
        _prediction_speed_kmps.clear();
        _prediction_points_world.clear();
        _prediction_dirty = true;

        _world.clear_rebase_anchor();
        _world.clear();

        _orbiters.clear();
        _orbitsim.reset();

        if (ctx.api)
        {
            ctx.api->clear_all_instances();
            ctx.api->clear_planets(true);
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (ctx.renderer && ctx.renderer->_context)
        {
            if (ctx.renderer->_context->physics_context == _physics_context.get())
            {
                ctx.renderer->_context->physics_context = nullptr;
            }
        }

        _physics = std::make_unique<Physics::JoltPhysicsWorld>();
        _physics->set_gravity(glm::vec3(0.0f));
        _physics_context = std::make_unique<Physics::PhysicsContext>(*_physics);

        _world.set_physics(_physics.get());
        _world.set_physics_context(_physics_context.get());

        if (ctx.renderer && ctx.renderer->_context)
        {
            ctx.renderer->_context->physics_context = _physics_context.get();
        }
#endif

        if (!ctx.api)
        {
            return;
        }

        // Initialize orbital simulation from config
        WorldVec3 player_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 player_vel_world(0.0);
        init_orbitsim(player_pos_world, player_vel_world);

        setup_environment(ctx);

        // Initialize physics origin near the player to keep local coordinates small.
        if (_physics_context)
        {
            _physics_context->set_origin_world(player_pos_world);
            _physics_context->set_velocity_origin_world(player_vel_world);
        }

        const glm::dvec3 v_origin_world =
                _physics_context ? _physics_context->velocity_origin_world() : glm::dvec3(0.0);

        // Spawn orbiter entities from config
        auto spawn_orbiter = [&](const std::string &name,
                                 const WorldVec3 &pos_world,
                                 const glm::vec3 &vel_local_f,
                                 GameAPI::PrimitiveType prim,
                                 const glm::vec3 &render_scale,
                                 const Physics::BodySettings &settings,
                                 bool is_player,
                                 EntityId &out_id) {
            Transform tr{};
            tr.position_world = pos_world;
            tr.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            tr.scale = render_scale;

            Entity *ent = nullptr;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics)
            {
                ent = _world.builder(name)
                        .transform(tr)
                        .render_primitive(prim)
                        .physics(settings)
                        .build();
            }
            else
#endif
            {
                ent = _world.builder(name)
                        .transform(tr)
                        .render_primitive(prim)
                        .build();
            }

            if (!ent)
            {
                out_id = EntityId{};
                return;
            }

            out_id = ent->id();

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics && ent->has_physics())
            {
                const Physics::BodyId body_id{ent->physics_body_value()};
                if (_physics->is_body_valid(body_id))
                {
                    _physics->set_linear_velocity(body_id, vel_local_f);
                }
            }

            if (is_player)
            {
                // Use ShipController defaults so tuning in ship_controller.h (and UI sliders) takes effect.
                ent->add_component<ShipController>();
            }
#endif
        };

        // Spawn all orbiters from config
        const auto &cfg = _scenario_config;

        bool primary_player_spawned = false;
        for (const auto &orbiter_def : cfg.orbiters)
        {
            const bool is_primary_player = orbiter_def.is_player && !primary_player_spawned;
            WorldVec3 pos_world = player_pos_world + WorldVec3(orbiter_def.offset_from_player);
            glm::dvec3 vel_world = player_vel_world + orbiter_def.relative_velocity;
            if (is_primary_player)
            {
                pos_world = player_pos_world;
                vel_world = player_vel_world;
                primary_player_spawned = true;
            }

            const glm::vec3 vel_local_f = glm::vec3(vel_world - v_origin_world);

            EntityId entity_id{};
            spawn_orbiter(orbiter_def.name,
                          pos_world,
                          vel_local_f,
                          orbiter_def.primitive,
                          orbiter_def.render_scale,
                          orbiter_def.body_settings,
                          is_primary_player,
                          entity_id);

            OrbiterInfo info{};
            info.entity = entity_id;
            info.name = orbiter_def.name;
            info.apply_gravity = true;
            info.is_player = is_primary_player;
            info.is_rebase_anchor = orbiter_def.is_rebase_anchor;
            _orbiters.push_back(std::move(info));
        }

        // Spawn celestial body render entities (non-terrain ones that need a mesh)
        if (_orbitsim)
        {
            const orbitsim::MassiveBody *ref_sim = _orbitsim->reference_sim_body();

            for (auto &body_info : _orbitsim->bodies)
            {
                // Terrain bodies are rendered by planet terrain system, not as primitive meshes.
                if (body_info.has_terrain)
                {
                    continue;
                }

                // Compute world position relative to reference body
                WorldVec3 body_pos_world = cfg.system_center;
                if (ref_sim)
                {
                    const orbitsim::MassiveBody *sim_body = _orbitsim->sim.body_by_id(body_info.sim_id);
                    if (sim_body)
                    {
                        body_pos_world = cfg.system_center +
                                WorldVec3(sim_body->state.position_m - ref_sim->state.position_m);
                    }
                }

                // Find the matching CelestialDef for render_scale
                float render_scale = 1.0f;
                for (const auto &cdef : cfg.celestials)
                {
                    if (cdef.name == body_info.name)
                    {
                        render_scale = cdef.render_scale;
                        break;
                    }
                }

                Transform tr{};
                tr.position_world = body_pos_world;
                tr.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                tr.scale = glm::vec3(render_scale);

                if (Entity *ent = _world.builder(body_info.name)
                        .transform(tr)
                        .render_primitive(GameAPI::PrimitiveType::Sphere)
                        .build())
                {
                    body_info.render_entity = ent->id();
                }
            }
        }

        // Configure explicit rebase anchor and camera target.
        {
            // Keep local velocities bounded for Jolt (float velocities) in high-speed scenarios.
            // 0 disables automatic velocity rebasing.
            GameWorld::RebaseSettings rs{};
            rs.velocity_threshold_mps = 2000.0; // 2 km/s local delta-v before rebasing
            _world.set_rebase_settings(rs);
        }
        update_rebase_anchor();

        const EntityId primary_player_eid = player_entity();

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics)
        {
            for (const auto &orbiter : _orbiters)
            {
                if (!orbiter.is_player || !orbiter.entity.is_valid())
                {
                    continue;
                }

                Entity *player = _world.entities().find(orbiter.entity);
                if (player && player->has_physics())
                {
                    const Physics::BodyId player_body{player->physics_body_value()};
                    if (_physics->is_body_valid(player_body))
                    {
                        Physics::PhysicsWorld::BodyCallbacks callbacks{};
                        callbacks.on_collision = [this](const Physics::CollisionEvent &e) {
                            if (e.type != Physics::ContactEventType::Begin)
                            {
                                return;
                            }

                            mark_prediction_dirty();

                            if (!_contact_log_enabled)
                            {
                                return;
                            }

                            ContactLogEntry entry{};
                            entry.time_s = static_cast<float>(_fixed_time_s);
                            entry.type = e.type;
                            entry.self_body = e.self.value;
                            entry.other_body = e.other.value;
                            entry.self_user_data = e.self_user_data;
                            entry.other_user_data = e.other_user_data;
                            entry.point = e.point;
                            entry.normal = e.normal;
                            entry.penetration_depth = e.penetration_depth;

                            _contact_log.push_back(entry);
                            while (_contact_log.size() > _contact_log_capacity)
                            {
                                _contact_log.pop_front();
                            }

                            if (_contact_log_print_console)
                            {
                                Logger::debug("[Collision][{}] self={} other={} depth={:.3f} p=({:.2f},{:.2f},{:.2f})",
                                              contact_event_type_name(entry.type),
                                              entry.self_body,
                                              entry.other_body,
                                              entry.penetration_depth,
                                              entry.point.x, entry.point.y, entry.point.z);
                            }
                        };
                        _physics->set_body_callbacks(player_body, callbacks);
                    }
                }
            }
        }
#endif

        std::string cam_target_name = "ship";
        if (const OrbiterInfo *player_orbiter = find_player_orbiter())
        {
            cam_target_name = player_orbiter->name;
        }
        else if (!_orbiters.empty() && !_orbiters.front().name.empty())
        {
            cam_target_name = _orbiters.front().name;
        }

        if (primary_player_eid.is_valid() || !_orbiters.empty())
        {
            GameAPI::OrbitCameraSettings orbit{};
            orbit.target.type = GameAPI::CameraTargetType::MeshInstance;
            orbit.target.name = cam_target_name;
            orbit.distance = 40.0;
            orbit.yaw = 0.6f;
            orbit.pitch = -0.35f;
            orbit.lookSensitivity = 0.0020f;
            ctx.api->set_camera_mode(GameAPI::CameraMode::Orbit);
            ctx.api->set_orbit_camera_settings(orbit);
        }

        ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
    }

    void GameplayState::setup_environment(GameStateContext &ctx)
    {
        if (ctx.renderer && ctx.renderer->_assetManager)
        {
            GameAPI::IBLPaths ibl{};
            ibl.specularCube = ctx.renderer->_assetManager->assetPath("ibl/darkstar.ktx2");
            ibl.diffuseCube = ctx.renderer->_assetManager->assetPath("ibl/darkstar.ktx2");
            ibl.brdfLut = ctx.renderer->_assetManager->assetPath("ibl/brdf_lut.ktx2");
            ibl.background = ctx.renderer->_assetManager->assetPath("ibl/darkstar.ktx2");
            ctx.api->load_global_ibl(ibl);
        }

        const auto &cfg = _scenario_config;
        bool atmosphere_set = false;
        const orbitsim::MassiveBody *ref_sim = _orbitsim ? _orbitsim->reference_sim_body() : nullptr;

        for (const auto &cdef : cfg.celestials)
        {
            if (!cdef.has_terrain)
            {
                continue;
            }

            GameAPI::PlanetTerrain planet{};
            planet.name = cdef.name;
            WorldVec3 planet_center_world = cfg.system_center;
            if (_orbitsim && ref_sim)
            {
                if (const CelestialBodyInfo *info = _orbitsim->find_body(cdef.name))
                {
                    if (const orbitsim::MassiveBody *sim_body = _orbitsim->sim.body_by_id(info->sim_id))
                    {
                        planet_center_world = cfg.system_center +
                                WorldVec3(sim_body->state.position_m - ref_sim->state.position_m);
                    }
                }
            }
            planet.center = glm::dvec3(planet_center_world);
            planet.radius_m = cdef.radius_m;
            planet.visible = true;
            planet.base_color = glm::vec4(1.0f);
            planet.metallic = 0.0f;
            planet.roughness = 1.0f;
            planet.albedo_dir = cdef.albedo_dir;
            planet.height_dir = cdef.height_dir;
            planet.height_max_m = cdef.height_max_m;
            planet.emission_dir = cdef.emission_dir;
            planet.emission_factor = cdef.emission_factor;

            (void) ctx.api->add_planet_terrain(planet);

            // Atmosphere for the first terrain body (reference body)
            if (!atmosphere_set)
            {
                ctx.api->set_planet_system_enabled(true);
                ctx.api->set_atmosphere_enabled(true);
                ctx.api->reset_atmosphere_to_earth();

                auto atmo = ctx.api->get_atmosphere_settings();
                atmo.bodyName = cdef.name;
                ctx.api->set_atmosphere_settings(atmo);
                atmosphere_set = true;
            }
        }
    }

    void GameplayState::init_orbitsim(WorldVec3 &player_pos_world, glm::dvec3 &player_vel_world)
    {
        const auto &cfg = _scenario_config;
        if (cfg.celestials.empty())
        {
            return;
        }

        auto scenario = std::make_unique<OrbitalScenario>();

        const double speed_scale = std::max(0.0, cfg.speed_scale);

        orbitsim::GameSimulation::Config sim_cfg;
        sim_cfg.gravitational_constant = orbitsim::kGravitationalConstant_SI * speed_scale * speed_scale;
        sim_cfg.softening_length_m = 0.0;
        sim_cfg.enable_events = false;

        scenario->sim = orbitsim::GameSimulation(sim_cfg);
        scenario->reference_body_index = 0;

        // Create all massive bodies in the simulation
        const auto &ref_def = cfg.celestials[0];

        orbitsim::MassiveBody ref_body{};
        ref_body.mass_kg = ref_def.mass_kg;
        ref_body.radius_m = ref_def.radius_m;
        ref_body.atmosphere_top_height_m = ref_def.atmosphere_top_m;
        ref_body.terrain_max_height_m = ref_def.terrain_max_m;
        ref_body.soi_radius_m = ref_def.soi_radius_m;

        // For multiple celestials, set up barycentric states via two-body pairs
        // relative to the reference body.
        std::vector<orbitsim::MassiveBody> sim_bodies;
        sim_bodies.push_back(ref_body);

        for (size_t i = 1; i < cfg.celestials.size(); ++i)
        {
            const auto &cdef = cfg.celestials[i];

            orbitsim::MassiveBody body{};
            body.mass_kg = cdef.mass_kg;
            body.radius_m = cdef.radius_m;
            body.atmosphere_top_height_m = cdef.atmosphere_top_m;
            body.terrain_max_height_m = cdef.terrain_max_m;
            body.soi_radius_m = cdef.soi_radius_m;
            sim_bodies.push_back(body);
        }

        // Initialize states: for each satellite, compute two-body barycentric orbit with reference
        if (sim_bodies.size() >= 2)
        {
            // For simplicity, use pairwise two-body initialization with the reference body.
            // Each satellite is placed relative to the reference body using barycentric states.
            // When there are multiple satellites, each pair (ref, satellite_i) is handled independently.
            for (size_t i = 1; i < sim_bodies.size(); ++i)
            {
                const double sep_m = std::max(ref_def.radius_m * 2.0, cfg.celestials[i].orbit_distance_m);
                const auto bary_init = two_body_circular_barycentric_xz(
                        sim_cfg.gravitational_constant,
                        sim_bodies[0].mass_kg, sim_bodies[i].mass_kg,
                        sep_m, 0.0);

                // Accumulate barycentric offset onto reference body (for multi-satellite systems
                // this is approximate, but good enough for initialization)
                sim_bodies[0].state.position_m += bary_init.state_a.position_m;
                sim_bodies[0].state.velocity_mps += bary_init.state_a.velocity_mps;
                sim_bodies[i].state = bary_init.state_b;
            }
        }

        // Register all bodies in the simulation and build CelestialBodyInfo list
        bool all_valid = true;
        for (size_t i = 0; i < sim_bodies.size(); ++i)
        {
            const auto handle = scenario->sim.create_body(sim_bodies[i]);
            if (!handle.valid())
            {
                all_valid = false;
                break;
            }

            CelestialBodyInfo info{};
            info.sim_id = handle.id;
            info.name = cfg.celestials[i].name;
            info.radius_m = cfg.celestials[i].radius_m;
            info.mass_kg = cfg.celestials[i].mass_kg;
            info.has_terrain = cfg.celestials[i].has_terrain;
            scenario->bodies.push_back(std::move(info));
        }

        if (!all_valid || scenario->bodies.empty())
        {
            _orbitsim = std::move(scenario);
            return;
        }

        // Compute player initial orbit around reference body
        const CelestialBodyInfo *ref_info = scenario->reference_body();
        const orbitsim::MassiveBody *ref_sim = scenario->reference_sim_body();
        if (ref_info && ref_sim)
        {
            // Find player orbiter def to get orbit altitude
            double player_altitude_m = 400'000.0;
            for (const auto &odef : cfg.orbiters)
            {
                if (odef.is_player)
                {
                    player_altitude_m = odef.orbit_altitude_m;
                    break;
                }
            }

            const double orbit_radius_m = ref_info->radius_m + player_altitude_m;
            const detail::OrbitRelativeState ship_rel =
                    circular_orbit_relative_state_xz(sim_cfg.gravitational_constant, ref_info->mass_kg,
                                                     std::max(1.0, orbit_radius_m), 0.0);

            player_pos_world = cfg.system_center + WorldVec3(ship_rel.position_m);
            player_vel_world = glm::dvec3(ship_rel.velocity_mps);
        }

        _orbitsim = std::move(scenario);
    }

    // ---- Orbiter helpers ----

    const OrbiterInfo *GameplayState::find_player_orbiter() const
    {
        for (const auto &o : _orbiters)
        {
            if (o.is_player)
            {
                return &o;
            }
        }
        return nullptr;
    }

    EntityId GameplayState::player_entity() const
    {
        const OrbiterInfo *p = find_player_orbiter();
        return p ? p->entity : EntityId{};
    }

    EntityId GameplayState::select_rebase_anchor_entity() const
    {
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.is_rebase_anchor && orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }

        // Fallback: first valid player.
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.is_player && orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }

        // Last resort: any valid orbiter.
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }

        return EntityId{};
    }

    void GameplayState::update_rebase_anchor()
    {
        const EntityId next_anchor = select_rebase_anchor_entity();
        if (!next_anchor.is_valid())
        {
            _world.clear_rebase_anchor();
            return;
        }

        if (next_anchor != _world.rebase_anchor())
        {
            _world.set_rebase_anchor(next_anchor);
        }
    }
} // namespace Game
