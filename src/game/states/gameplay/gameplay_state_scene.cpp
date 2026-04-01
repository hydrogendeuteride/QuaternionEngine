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
#include <utility>

namespace Game
{
    using detail::contact_event_type_name;
    using detail::circular_orbit_relative_state_xz;
    using detail::two_body_circular_barycentric_xz;

    namespace
    {
        glm::vec4 celestial_mesh_base_color(const std::string &name)
        {
            if (name == "moon")
            {
                return glm::vec4(0.74f, 0.74f, 0.78f, 1.0f);
            }
            return glm::vec4(0.86f, 0.86f, 0.88f, 1.0f);
        }
    } // namespace

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
            earth.specular_dir = "planets/earth/specular/L0";
            earth.specular_strength = 1.0f;
            earth.specular_roughness = 0.06f;
            earth.prediction_orbit_color = glm::vec3(0.60f, 0.68f, 0.82f);
            earth.has_prediction_orbit_color = true;
            cfg.celestials.push_back(std::move(earth));
        }

        // Moon
        {
            ScenarioConfig::CelestialDef moon{};
            moon.name = "moon";
            moon.mass_kg = 7.342e22;
            moon.radius_m = 1'728'418.5;
            moon.has_terrain = true;
            moon.terrain_max_m = 19'667.0;
            moon.albedo_dir = "planets/moon/albedo/L0";
            moon.height_dir = "planets/moon/height/L0";
            moon.height_max_m = 19'667.0;
            moon.soi_radius_m = 6.61e7;
            moon.orbit_distance_m = 384'400'000.0;
            moon.prediction_orbit_color = glm::vec3(0.84f, 0.84f, 0.87f);
            moon.has_prediction_orbit_color = true;
            cfg.celestials.push_back(std::move(moon));
        }

        // Ship (player)
        {
            ScenarioConfig::OrbiterDef ship{};
            ship.name = "ship";
            ship.orbit_altitude_m = 400'000.0;
            ship.prediction_group = "flight";
            ship.prediction_orbit_color = glm::vec3(0.70f, 0.35f, 1.00f);
            ship.has_prediction_orbit_color = true;
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
            probe.formation_hold_enabled = true;
            probe.formation_leader = "ship";
            probe.formation_slot_lvlh_m = glm::dvec3(0.0, -60.0, 0.0);
            probe.prediction_group = "flight";
            probe.prediction_orbit_color = glm::vec3(0.25f, 0.52f, 1.00f);
            probe.has_prediction_orbit_color = true;
            probe.is_player = false;
            probe.offset_from_player = glm::dvec3(0.0, 0.0, -60.0);
            probe.relative_velocity = glm::dvec3(0.0);
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

        // Collision experiment target
        {
            ScenarioConfig::OrbiterDef collision_test{};
            collision_test.name = "collision_test";
            collision_test.is_player = false;
            collision_test.prediction_group = "flight";
            collision_test.prediction_orbit_color = glm::vec3(1.00f, 0.25f, 0.25f);
            collision_test.has_prediction_orbit_color = true;
            collision_test.offset_from_player = glm::dvec3(0.0, -0.5, 150.0);
            collision_test.relative_velocity = glm::dvec3(0.0, 0.0, -25.0);
            collision_test.primitive = GameAPI::PrimitiveType::Cube;
            collision_test.render_scale = glm::vec3(3.0f);

            collision_test.body_settings
                    .set_shape(Physics::CollisionShape::Box(1.5f, 1.5f, 1.5f))
                    .set_dynamic()
                    .set_layer(Physics::Layer::Dynamic)
                    .set_gravity_scale(0.0f)
                    .set_friction(0.2f)
                    .set_restitution(0.1f)
                    .set_linear_damping(0.0f)
                    .set_angular_damping(0.0f)
                    .set_mass(500.0f);

            cfg.orbiters.push_back(std::move(collision_test));
        }

        return cfg;
    }

    // ---- Scene setup ----

    void GameplayState::setup_scene(GameStateContext &ctx)
    {
        _elapsed = 0.0f;
        _fixed_time_s = 0.0;
        reset_time_warp_state();
        _warp_to_time_active = false;
        _warp_to_time_target_s = 0.0;
        _warp_to_time_restore_level = 0;
        _execute_node_armed = false;
        _execute_node_id = -1;
        _maneuver_state.nodes.clear();
        _maneuver_state.selected_node_id = -1;
        _maneuver_state.next_node_id = 0;
        _maneuver_gizmo_interaction = {};
        _reset_requested = false;
        _contact_log.clear();
        _prediction_tracks.clear();
        _prediction_groups.clear();
        _prediction_selection.clear();
        _prediction_frame_selection.clear();
        _prediction_dirty = true;
        _prediction_service.reset();
        _prediction_derived_service.reset();

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
            info.mass_kg = std::max(0.0, static_cast<double>(orbiter_def.body_settings.mass));
            info.physics_settings = orbiter_def.body_settings;
            info.use_physics_interpolation = true;
            info.formation_hold_enabled = orbiter_def.formation_hold_enabled;
            info.formation_leader_name = orbiter_def.formation_leader;
            info.formation_slot_lvlh_m = orbiter_def.formation_slot_lvlh_m;
            _orbiters.push_back(std::move(info));
        }

        rebuild_prediction_subjects();

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

        sync_player_collision_callbacks();

        if (primary_player_eid.is_valid() || !_orbiters.empty())
        {
            GameAPI::OrbitCameraSettings orbit{};
            orbit.target.type = GameAPI::CameraTargetType::MeshInstance;
            orbit.distance = 40.0;
            orbit.yaw = 0.6f;
            orbit.pitch = -0.35f;
            orbit.lookSensitivity = 0.0020f;
            ctx.api->set_camera_mode(GameAPI::CameraMode::Orbit);
            ctx.api->set_orbit_camera_settings(orbit);
            sync_player_camera_target(ctx);
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
        const orbitsim::MassiveBody *ref_sim = _orbitsim ? _orbitsim->world_reference_sim_body() : nullptr;
        const auto celestial_world_position = [&](const std::string &name) {
            WorldVec3 planet_center_world = cfg.system_center;
            if (_orbitsim && ref_sim)
            {
                if (const CelestialBodyInfo *info = _orbitsim->find_body(name))
                {
                    if (const orbitsim::MassiveBody *sim_body = _orbitsim->sim.body_by_id(info->sim_id))
                    {
                        planet_center_world = cfg.system_center +
                                WorldVec3(sim_body->state.position_m - ref_sim->state.position_m);
                    }
                }
            }
            return planet_center_world;
        };

        bool any_planets_added = false;
        bool atmosphere_set = false;

        for (const auto &cdef : cfg.celestials)
        {
            if (cdef.has_terrain)
            {
                GameAPI::PlanetTerrain planet{};
                planet.name = cdef.name;
                planet.center = glm::dvec3(celestial_world_position(cdef.name));
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
                planet.specular_dir = cdef.specular_dir;
                planet.specular_strength = cdef.specular_strength;
                planet.specular_roughness = cdef.specular_roughness;

                any_planets_added |= ctx.api->add_planet_terrain(planet);

                // Atmosphere for the first terrain body (reference body)
                if (!atmosphere_set)
                {
                    ctx.api->set_atmosphere_enabled(true);
                    ctx.api->reset_atmosphere_to_earth();

                    auto atmo = ctx.api->get_atmosphere_settings();
                    atmo.bodyName = cdef.name;
                    ctx.api->set_atmosphere_settings(atmo);
                    atmosphere_set = true;
                }
                continue;
            }

            GameAPI::PlanetSphere planet{};
            planet.name = cdef.name;
            planet.center = glm::dvec3(celestial_world_position(cdef.name));
            planet.radius_m = cdef.radius_m;
            planet.visible = true;
            planet.base_color = celestial_mesh_base_color(cdef.name);
            planet.metallic = 0.0f;
            planet.roughness = 0.98f;
            planet.sectors = 64;
            planet.stacks = 32;

            any_planets_added |= ctx.api->add_planet_sphere(planet);
        }

        if (any_planets_added)
        {
            ctx.api->set_planet_system_enabled(true);
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
        scenario->world_reference_body_index = 0;

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
        const CelestialBodyInfo *ref_info = scenario->world_reference_body();
        const orbitsim::MassiveBody *ref_sim = scenario->world_reference_sim_body();
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

    OrbiterInfo *GameplayState::find_player_orbiter()
    {
        for (auto &o : _orbiters)
        {
            if (o.is_player)
            {
                return &o;
            }
        }
        return nullptr;
    }

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

    OrbiterInfo *GameplayState::find_orbiter(const EntityId entity)
    {
        if (!entity.is_valid())
        {
            return nullptr;
        }

        for (auto &orbiter : _orbiters)
        {
            if (orbiter.entity == entity)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    const OrbiterInfo *GameplayState::find_orbiter(const EntityId entity) const
    {
        if (!entity.is_valid())
        {
            return nullptr;
        }

        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.entity == entity)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    OrbiterInfo *GameplayState::find_orbiter(const std::string_view name)
    {
        if (name.empty())
        {
            return nullptr;
        }

        for (auto &orbiter : _orbiters)
        {
            if (orbiter.name == name)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    const OrbiterInfo *GameplayState::find_orbiter(const std::string_view name) const
    {
        if (name.empty())
        {
            return nullptr;
        }

        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.name == name)
            {
                return &orbiter;
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

    void GameplayState::sync_player_camera_target(GameStateContext &ctx) const
    {
        if (!ctx.api)
        {
            return;
        }

        const OrbiterInfo *player_orbiter = find_player_orbiter();
        if (!player_orbiter || player_orbiter->name.empty())
        {
            return;
        }

        GameAPI::CameraTarget target{};
        target.type = GameAPI::CameraTargetType::MeshInstance;
        target.name = player_orbiter->name;

        GameAPI::OrbitCameraSettings orbit = ctx.api->get_orbit_camera_settings();
        orbit.target = target;
        ctx.api->set_orbit_camera_settings(orbit);

        GameAPI::FollowCameraSettings follow = ctx.api->get_follow_camera_settings();
        follow.target = target;
        ctx.api->set_follow_camera_settings(follow);

        GameAPI::ChaseCameraSettings chase = ctx.api->get_chase_camera_settings();
        chase.target = target;
        ctx.api->set_chase_camera_settings(chase);
    }

    void GameplayState::sync_player_collision_callbacks()
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_physics)
        {
            return;
        }

        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (!orbiter.entity.is_valid())
            {
                continue;
            }

            const Entity *entity = _world.entities().find(orbiter.entity);
            if (!entity || !entity->has_physics())
            {
                continue;
            }

            const Physics::BodyId body_id{entity->physics_body_value()};
            if (!_physics->is_body_valid(body_id))
            {
                continue;
            }

            _physics->clear_body_callbacks(body_id);
        }

        const OrbiterInfo *player_orbiter = find_player_orbiter();
        if (!player_orbiter || !player_orbiter->entity.is_valid())
        {
            return;
        }

        const Entity *player = _world.entities().find(player_orbiter->entity);
        if (!player || !player->has_physics())
        {
            return;
        }

        const Physics::BodyId player_body{player->physics_body_value()};
        if (!_physics->is_body_valid(player_body))
        {
            return;
        }

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
#endif
    }

    bool GameplayState::set_active_player_orbiter(GameStateContext &ctx, const EntityId entity)
    {
        OrbiterInfo *target = find_orbiter(entity);
        OrbiterInfo *current = find_player_orbiter();
        if (!target || !target->entity.is_valid() || target == current)
        {
            return false;
        }

        struct ControllerSnapshot
        {
            float thrust_force{500.0f};
            float torque_strength{50.0f};
            float sas_damping{5.0f};
            bool sas_enabled{false};
            bool sas_toggle_prev_down{false};
        } snapshot{};

        if (current)
        {
            if (Entity *current_entity = _world.entities().find(current->entity))
            {
                if (ShipController *controller = current_entity->get_component<ShipController>())
                {
                    snapshot.thrust_force = controller->thrust_force();
                    snapshot.torque_strength = controller->torque_strength();
                    snapshot.sas_damping = controller->sas_damping();
                    snapshot.sas_enabled = controller->sas_enabled();
                    snapshot.sas_toggle_prev_down = ctx.input && ctx.input->key_down(Key::T);
                    (void) current_entity->remove_component<ShipController>();
                }
            }
        }

        if (!_rails_warp_active && target->rails.active())
        {
            (void) demote_orbiter_from_rails(*target);
        }

        for (auto &orbiter : _orbiters)
        {
            const bool is_active = orbiter.entity == target->entity;
            orbiter.is_player = is_active;
            orbiter.is_rebase_anchor = is_active;
        }

        for (auto &orbiter_def : _scenario_config.orbiters)
        {
            const bool is_active = orbiter_def.name == target->name;
            orbiter_def.is_player = is_active;
            orbiter_def.is_rebase_anchor = is_active;
        }

        if (Entity *target_entity = _world.entities().find(target->entity))
        {
            ShipController *controller = target_entity->get_component<ShipController>();
            if (!controller)
            {
                controller = target_entity->add_component<ShipController>();
            }
            if (controller)
            {
                controller->set_thrust_force(snapshot.thrust_force);
                controller->set_torque_strength(snapshot.torque_strength);
                controller->set_sas_damping(snapshot.sas_damping);
                controller->set_sas_enabled(target->rails.active() ? target->rails.sas_enabled : snapshot.sas_enabled);
                controller->set_sas_toggle_prev_down(snapshot.sas_toggle_prev_down);
            }
        }

        update_rebase_anchor();
        sync_player_camera_target(ctx);
        sync_player_collision_callbacks();

        _prediction_selection.active_subject = PredictionSubjectKey{PredictionSubjectKind::Orbiter, target->entity.value};
        _prediction_selection.overlay_subjects.clear();
        _prediction_selection.selected_group_index = -1;
        _execute_node_armed = false;
        _execute_node_id = -1;
        _maneuver_state.nodes.clear();
        _maneuver_state.selected_node_id = -1;
        _maneuver_state.next_node_id = 0;
        _maneuver_gizmo_interaction = {};
        mark_prediction_dirty();
        return true;
    }

    bool GameplayState::cycle_player_orbiter(GameStateContext &ctx, const int direction)
    {
        if (_orbiters.size() < 2 || direction == 0)
        {
            return false;
        }

        std::size_t current_index = 0;
        bool found_current = false;
        for (std::size_t i = 0; i < _orbiters.size(); ++i)
        {
            if (_orbiters[i].is_player)
            {
                current_index = i;
                found_current = true;
                break;
            }
        }

        if (!found_current)
        {
            for (std::size_t i = 0; i < _orbiters.size(); ++i)
            {
                if (_orbiters[i].entity.is_valid())
                {
                    current_index = i;
                    found_current = true;
                    break;
                }
            }
        }

        if (!found_current)
        {
            return false;
        }

        const std::size_t count = _orbiters.size();
        std::size_t next_index = current_index;
        const int step = direction > 0 ? 1 : -1;
        for (std::size_t attempt = 0; attempt < count; ++attempt)
        {
            const int signed_next =
                    (static_cast<int>(next_index) + step + static_cast<int>(count)) % static_cast<int>(count);
            next_index = static_cast<std::size_t>(signed_next);
            if (_orbiters[next_index].entity.is_valid())
            {
                break;
            }
        }

        if (next_index == current_index || !_orbiters[next_index].entity.is_valid())
        {
            return false;
        }

        return set_active_player_orbiter(ctx, _orbiters[next_index].entity);
    }
} // namespace Game
