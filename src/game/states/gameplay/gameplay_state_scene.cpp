#include "gameplay_state.h"
#include "orbit_helpers.h"
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

    void GameplayState::setup_scene(GameStateContext &ctx)
    {
        _elapsed = 0.0f;
        _fixed_time_s = 0.0;
        _reset_requested = false;
        _contact_log.clear();
        _prediction_update_accum_s = 0.0f;
        _prediction_altitude_km.clear();
        _prediction_speed_kmps.clear();
        _prediction_points_world.clear();

        _world.clear_rebase_anchor();
        _world.clear();

        _ship_entity = EntityId{};
        _probe_entity = EntityId{};
        _moon_entity = EntityId{};
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

        setup_environment(ctx);

        const double orbit_radius_m = _planet_radius_m + _orbit_altitude_m;
        const double speed_scale = std::max(0.0, _orbit_speed_scale);
        const double mu = _mu_base_m3ps2 * speed_scale * speed_scale;
        const double v_circ = (orbit_radius_m > 0.0) ? std::sqrt(mu / orbit_radius_m) : 0.0;

        WorldVec3 ship_pos_world = _planet_center_world + WorldVec3(orbit_radius_m, 0.0, 0.0);
        glm::dvec3 ship_vel_world_d(0.0, 0.0, v_circ);

        WorldVec3 moon_pos_world(0.0);
        bool have_moon = false;
        init_orbitsim(orbit_radius_m, speed_scale, ship_pos_world, ship_vel_world_d, moon_pos_world, have_moon);

        const glm::dvec3 probe_rel_vel_world_d(0.0, 0.0, -10.0);
        glm::dvec3 probe_vel_world_d = ship_vel_world_d + probe_rel_vel_world_d;
        WorldVec3 probe_pos_world = ship_pos_world + WorldVec3(_probe_offset_world);

        // Initialize physics origin near the ship to keep local coordinates small.
        if (_physics_context)
        {
            _physics_context->set_origin_world(ship_pos_world);
            _physics_context->set_velocity_origin_world(ship_vel_world_d);
        }

        const glm::dvec3 v_origin_world =
                _physics_context ? _physics_context->velocity_origin_world() : glm::dvec3(0.0);
        const glm::vec3 ship_vel_local_f = glm::vec3(ship_vel_world_d - v_origin_world);
        const glm::vec3 probe_vel_local_f = glm::vec3(probe_vel_world_d - v_origin_world);

        auto spawn_orbiter = [&](const std::string &name,
                                 const WorldVec3 &pos_world,
                                 const glm::vec3 &vel_local_f,
                                 GameAPI::PrimitiveType prim,
                                 const glm::vec3 &render_scale,
                                 const Physics::BodySettings &settings,
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
#endif
        };

        // Capsule primitive is radius=0.5, half-height=0.5 (uniform scale keeps it capsule-shaped).
        constexpr float ship_radius_m = 2.0f;
        constexpr float ship_half_height_m = 2.0f;
        constexpr float ship_uniform_scale = ship_radius_m / 0.5f;

        Physics::BodySettings ship_settings{};
        ship_settings.set_shape(Physics::CollisionShape::Capsule(ship_radius_m, ship_half_height_m))
                .set_dynamic()
                .set_layer(Physics::Layer::Player)
                .set_gravity_scale(0.0f)
                .set_friction(0.2f)
                .set_restitution(0.05f)
                .set_linear_damping(0.0f)
                .set_angular_damping(0.0f);

        spawn_orbiter("ship",
                      ship_pos_world,
                      ship_vel_local_f,
                      GameAPI::PrimitiveType::Capsule,
                      glm::vec3(ship_uniform_scale),
                      ship_settings,
                      _ship_entity);

        Physics::BodySettings probe_settings{};
        probe_settings.set_shape(Physics::CollisionShape::Sphere(1.0f))
                .set_dynamic()
                .set_layer(Physics::Layer::Dynamic)
                .set_gravity_scale(0.0f)
                .set_friction(0.2f)
                .set_restitution(0.1f)
                .set_linear_damping(0.0f)
                .set_angular_damping(0.0f);

        spawn_orbiter("probe",
                      probe_pos_world,
                      probe_vel_local_f,
                      GameAPI::PrimitiveType::Sphere,
                      glm::vec3(2.0f),
                      probe_settings,
                      _probe_entity);

        if (have_moon)
        {
            Transform tr{};
            tr.position_world = moon_pos_world;
            tr.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            tr.scale = glm::vec3(150'000.0f);

            if (Entity *ent = _world.builder("moon")
                    .transform(tr)
                    .render_primitive(GameAPI::PrimitiveType::Sphere)
                    .build())
            {
                _moon_entity = ent->id();
            }
        }

        _world.set_rebase_anchor(_ship_entity);
        _world.set_rebase_settings(GameWorld::RebaseSettings{});

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics && _ship_entity.is_valid())
        {
            Entity *ship = _world.entities().find(_ship_entity);
            if (ship && ship->has_physics())
            {
                const Physics::BodyId ship_body{ship->physics_body_value()};
                if (_physics->is_body_valid(ship_body))
                {
                    Physics::PhysicsWorld::BodyCallbacks callbacks{};
                    callbacks.on_collision = [this](const Physics::CollisionEvent &e) {
                        if (!_contact_log_enabled)
                        {
                            return;
                        }
                        if (e.type != Physics::ContactEventType::Begin)
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
                    _physics->set_body_callbacks(ship_body, callbacks);
                }
            }
        }
#endif

        // Orbit camera around ship instance (RMB rotate, wheel zoom).
        {
            GameAPI::OrbitCameraSettings orbit{};
            orbit.target.type = GameAPI::CameraTargetType::MeshInstance;
            orbit.target.name = "ship";
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

        GameAPI::PlanetTerrain earth{};
        earth.name = _planet_name;
        earth.center = glm::dvec3(_planet_center_world);
        earth.radius_m = _planet_radius_m;
        earth.visible = true;
        earth.base_color = glm::vec4(1.0f);
        earth.metallic = 0.0f;
        earth.roughness = 1.0f;
        earth.albedo_dir = "planets/earth/albedo/L0";
        earth.height_dir = "planets/earth/height/L0";
        earth.height_max_m = 8000.0;
        earth.emission_dir = "planets/earth/emission/L0";
        earth.emission_factor = glm::vec3(2.0f, 2.0f, 2.0f);

        (void) ctx.api->add_planet_terrain(earth);
        ctx.api->set_planet_system_enabled(true);
        ctx.api->set_atmosphere_enabled(true);
        ctx.api->reset_atmosphere_to_earth();

        auto atmo = ctx.api->get_atmosphere_settings();
        atmo.bodyName = _planet_name;
        ctx.api->set_atmosphere_settings(atmo);
    }

    void GameplayState::init_orbitsim(const double orbit_radius_m, const double speed_scale,
                                      WorldVec3 &ship_pos_world, glm::dvec3 &ship_vel_world,
                                      WorldVec3 &moon_pos_world, bool &have_moon)
    {
        auto demo = std::make_unique<OrbitsimDemo>();

        orbitsim::GameSimulation::Config cfg;
        cfg.gravitational_constant = orbitsim::kGravitationalConstant_SI * speed_scale * speed_scale;
        cfg.softening_length_m = 0.0;
        cfg.enable_events = false;

        demo->sim = orbitsim::GameSimulation(cfg);

        orbitsim::MassiveBody earth{};
        earth.mass_kg = 5.972e24;
        earth.radius_m = _planet_radius_m;
        earth.atmosphere_top_height_m = 100'000.0;
        earth.terrain_max_height_m = 8'848.0;
        earth.soi_radius_m = 9.24e8;

        orbitsim::MassiveBody moon{};
        moon.mass_kg = 7.342e22;
        moon.radius_m = 1'737'400.0;
        moon.soi_radius_m = 6.61e7;

        const double moon_sep_m = std::max(_planet_radius_m * 2.0, _moon_distance_m);
        const auto em_init = two_body_circular_barycentric_xz(cfg.gravitational_constant,
                                                              earth.mass_kg, moon.mass_kg,
                                                              moon_sep_m, 0.0);
        earth.state = em_init.state_a;
        moon.state = em_init.state_b;

        const auto earth_h = demo->sim.create_body(earth);
        const auto moon_h = demo->sim.create_body(moon);

        if (earth_h.valid() && moon_h.valid())
        {
            demo->earth_id = earth_h.id;
            demo->moon_id = moon_h.id;
            demo->earth_mass_kg = earth.mass_kg;

            const detail::OrbitRelativeState ship_rel =
                    circular_orbit_relative_state_xz(cfg.gravitational_constant, earth.mass_kg,
                                                     std::max(1.0, orbit_radius_m), 0.0);

            ship_pos_world = _planet_center_world + WorldVec3(ship_rel.position_m);
            ship_vel_world = glm::dvec3(ship_rel.velocity_mps);

            moon_pos_world = _planet_center_world + WorldVec3(moon.state.position_m - earth.state.position_m);
            have_moon = true;
        }

        _orbitsim = std::move(demo);
    }
} // namespace Game
