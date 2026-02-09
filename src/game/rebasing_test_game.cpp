#include "rebasing_test_game.h"

#include "runtime/game_runtime.h"
#include "core/engine.h"
#include "core/game_api.h"

#include "imgui.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include "orbitsim/game_sim.hpp"
#include "orbitsim/orbit_utils.hpp"

namespace Game
{
    struct OrbitsimDemo
    {
        orbitsim::GameSimulation sim{};
        orbitsim::BodyId earth_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId moon_id{orbitsim::kInvalidBodyId};
        orbitsim::SpacecraftId ship_id{orbitsim::kInvalidSpacecraftId};
    };

    namespace
    {
        constexpr glm::vec4 COLOR_PLANET_TO_SHIP{0.2f, 0.8f, 1.0f, 1.0f};
        constexpr glm::vec4 COLOR_PLANET_TO_MOON{0.8f, 0.8f, 0.9f, 0.8f};
        constexpr glm::vec4 COLOR_VELOCITY{1.0f, 0.35f, 0.1f, 1.0f};
        constexpr glm::vec4 COLOR_TRAIL{0.8f, 0.8f, 0.2f, 0.9f};
        constexpr glm::vec4 COLOR_ORBIT{0.2f, 0.9f, 0.2f, 0.6f};

        double safe_length(const glm::dvec3 &v)
        {
            const double len2 = glm::dot(v, v);
            if (!std::isfinite(len2) || len2 <= 0.0)
            {
                return 0.0;
            }
            return std::sqrt(len2);
        }

        bool changed(const glm::dvec3 &before, const glm::dvec3 &after, glm::dvec3 &out_delta)
        {
            const glm::dvec3 d = after - before;
            out_delta = d;
            const double d2 = glm::dot(d, d);
            return std::isfinite(d2) && d2 > 0.0;
        }

        struct OrbitRelativeState
        {
            orbitsim::Vec3 position_m{0.0, 0.0, 0.0};
            orbitsim::Vec3 velocity_mps{0.0, 0.0, 0.0};
        };

        OrbitRelativeState circular_orbit_relative_state_xz(const double gravitational_constant,
                                                            const double central_mass_kg,
                                                            const double orbital_radius_m,
                                                            const double arg_latitude_rad = 0.0)
        {
            if (!(central_mass_kg > 0.0) || !(orbital_radius_m > 0.0) || !std::isfinite(gravitational_constant))
            {
                return {};
            }

            const double mu = gravitational_constant * central_mass_kg;
            const double v_circ = std::sqrt(mu / orbital_radius_m);

            const double cos_u = std::cos(arg_latitude_rad);
            const double sin_u = std::sin(arg_latitude_rad);

            OrbitRelativeState out;
            out.position_m = orbitsim::Vec3{orbital_radius_m * cos_u, 0.0, orbital_radius_m * sin_u};
            out.velocity_mps = orbitsim::Vec3{-v_circ * sin_u, 0.0, v_circ * cos_u};
            return out;
        }

        orbitsim::TwoBodyBarycentricStates two_body_circular_barycentric_xz(const double gravitational_constant,
                                                                            const double mass_a_kg,
                                                                            const double mass_b_kg,
                                                                            const double separation_m,
                                                                            const double arg_latitude_rad = 0.0)
        {
            const double m_tot = mass_a_kg + mass_b_kg;
            if (!(m_tot > 0.0) || !(separation_m > 0.0) || !std::isfinite(m_tot))
            {
                return {};
            }

            const OrbitRelativeState rel = circular_orbit_relative_state_xz(gravitational_constant, m_tot, separation_m,
                                                                            arg_latitude_rad);

            const double frac_a = mass_b_kg / m_tot;
            const double frac_b = mass_a_kg / m_tot;

            orbitsim::TwoBodyBarycentricStates out;
            out.state_a = orbitsim::make_state(-frac_a * rel.position_m, -frac_a * rel.velocity_mps);
            out.state_b = orbitsim::make_state(frac_b * rel.position_m, frac_b * rel.velocity_mps);
            return out;
        }
    } // namespace

    RebasingTestGame::RebasingTestGame() = default;
    RebasingTestGame::~RebasingTestGame() = default;

    void RebasingTestGame::on_init(GameRuntime::Runtime &runtime)
    {
        _runtime = &runtime;
        auto &api = runtime.api();
        _world.set_api(&api);

        VulkanEngine *renderer = runtime.renderer();
        if (!renderer)
        {
            return;
        }

        // IBL / background
        if (renderer->_assetManager)
        {
            GameAPI::IBLPaths ibl{};
            ibl.specularCube = renderer->_assetManager->assetPath("ibl/starmap.ktx2");
            ibl.diffuseCube = renderer->_assetManager->assetPath("ibl/starmap.ktx2");
            ibl.brdfLut = renderer->_assetManager->assetPath("ibl/brdf_lut.ktx2");
            ibl.background = renderer->_assetManager->assetPath("ibl/starmap.ktx2");
            api.load_global_ibl(ibl);
        }

        setup_scene();

        // Game ImGui panel
        if (renderer->ui())
        {
            renderer->ui()->add_draw_callback([this]() { draw_ui(); });
        }
    }

    void RebasingTestGame::on_update(float dt)
    {
        (void) dt;

        if (!_runtime)
        {
            return;
        }

        if (_reset_requested)
        {
            _reset_requested = false;
            setup_scene();
        }

        apply_rebase_settings();

        auto &api = _runtime->api();
        const float alpha = _runtime->interpolation_alpha();

        // Sync all entities to render (world-space, double precision)
        _world.entities().sync_to_render(api, alpha);

        draw_debug();
    }

    void RebasingTestGame::on_fixed_update(float fixed_dt)
    {
        if (!_runtime)
        {
            return;
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_physics)
        {
            return;
        }

        if (!_physics_context)
        {
            return;
        }

        const glm::dvec3 origin_before = glm::dvec3(_physics_context->origin_world());
        const glm::dvec3 vel_origin_before = _physics_context->velocity_origin_world();

        // Pre-physics: interpolation + automatic rebasing (if configured)
        _world.pre_physics_step();

        const glm::dvec3 origin_after_rebase = glm::dvec3(_physics_context->origin_world());
        const glm::dvec3 vel_origin_after_rebase = _physics_context->velocity_origin_world();
        if (changed(origin_before, origin_after_rebase, _last_origin_delta_world))
        {
            ++_origin_rebase_count;
        }
        if (changed(vel_origin_before, vel_origin_after_rebase, _last_velocity_delta_world))
        {
            ++_velocity_rebase_count;
        }

        if (_use_orbitsim && _orbitsim)
        {
            const double dt_s = static_cast<double>(fixed_dt);
            _orbitsim->sim.step(dt_s);

            const orbitsim::MassiveBody *earth = _orbitsim->sim.body_by_id(_orbitsim->earth_id);
            const orbitsim::MassiveBody *moon = _orbitsim->sim.body_by_id(_orbitsim->moon_id);
            const orbitsim::Spacecraft *ship = _orbitsim->sim.spacecraft_by_id(_orbitsim->ship_id);
            if (earth && ship)
            {
                const orbitsim::Vec3 ship_pos_rel_m = ship->state.position_m - earth->state.position_m;
                const orbitsim::Vec3 ship_vel_rel_mps = ship->state.velocity_mps - earth->state.velocity_mps;

                const WorldVec3 ship_pos_world = _planet_center_world + WorldVec3(ship_pos_rel_m);
                const glm::dvec3 ship_vel_world_d(ship_vel_rel_mps);

                const WorldVec3 physics_origin_world = _physics_context->origin_world();
                const glm::dvec3 v_origin_world = _physics_context->velocity_origin_world();

                auto sync_body = [&](EntityId id, const WorldVec3 &pos_world, const glm::dvec3 &vel_world_d) {
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

                    const glm::dvec3 p_local = world_to_local_d(pos_world, physics_origin_world);
                    const glm::vec3 v_local_f = glm::vec3(vel_world_d - v_origin_world);

                    _physics->set_transform(body_id, p_local, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
                    _physics->set_linear_velocity(body_id, v_local_f);
                    _physics->set_angular_velocity(body_id, glm::vec3(0.0f));
                    _physics->activate(body_id);
                };

                sync_body(_ship_entity, ship_pos_world, ship_vel_world_d);

                const WorldVec3 probe_pos_world = ship_pos_world + WorldVec3(_probe_offset_world);
                sync_body(_probe_entity, probe_pos_world, ship_vel_world_d);

                if (moon)
                {
                    const orbitsim::Vec3 moon_pos_rel_m = moon->state.position_m - earth->state.position_m;
                    const WorldVec3 moon_pos_world = _planet_center_world + WorldVec3(moon_pos_rel_m);
                    if (Entity *moon_ent = _world.entities().find(_moon_entity))
                    {
                        moon_ent->set_position_world(moon_pos_world);
                        moon_ent->set_rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
                    }
                }
            }

            _world.post_physics_step();

            _trail_sample_accum_s += static_cast<double>(fixed_dt);
            if (_draw_trail && _trail_sample_interval_s > 0.0 && _trail_sample_accum_s >= _trail_sample_interval_s)
            {
                _trail_sample_accum_s = 0.0;

                if (Entity *ship_ent = _world.entities().find(_ship_entity))
                {
                    _ship_trail_world.push_back(ship_ent->position_world());
                    while (_ship_trail_world.size() > _trail_max_points)
                    {
                        _ship_trail_world.erase(_ship_trail_world.begin());
                    }
                }
            }

            return;
        }

        const double orbit_radius_m = _planet_radius_m + _orbit_altitude_m;
        const double speed_scale = std::max(0.0, _orbit_speed_scale);
        const double mu = _mu_base_m3ps2 * speed_scale * speed_scale;

        auto gravity_accel_world_at = [&](const WorldVec3 &p_world) -> glm::dvec3 {
            const glm::dvec3 r = glm::dvec3(p_world - _planet_center_world);

            const double r_len = safe_length(r);
            if (r_len <= 1.0 || !std::isfinite(r_len))
            {
                return glm::dvec3(0.0);
            }

            // a = -mu * r / |r|^3
            const double inv_r = 1.0 / r_len;
            const double inv_r3 = inv_r * inv_r * inv_r;
            const glm::dvec3 a_world = (-mu) * r * inv_r3;

            if (!std::isfinite(a_world.x) || !std::isfinite(a_world.y) || !std::isfinite(a_world.z))
            {
                return glm::dvec3(0.0);
            }

            return a_world;
        };

        // Velocity-origin integration.
        //
        // IMPORTANT: Velocity rebasing (Galilean transform) changes the meaning of v_local: velocities become
        // relative to physics_velocity_origin_world. To keep world motion correct we must also advance the
        // moving frame: x_world = physics_origin_world + x_local, with d/dt physics_origin_world = v_origin.
        //
        // For threshold-friendly behavior (no per-step velocity rebase), we can integrate v_origin using the
        // anchor's world acceleration and apply gravity in the anchor's free-fall frame:
        //   a_local = a_world - a_anchor_world.
        // This keeps the anchor's v_local near 0 without calling shift_velocity_origin every physics step.
        glm::dvec3 anchor_accel_world(0.0);
        bool have_anchor_accel = false;
        const bool wants_velocity_origin_integration =
            _enable_velocity_rebasing && _integrate_origin_from_velocity_origin;
        const WorldVec3 physics_origin_world = _physics_context->origin_world();

        if (wants_velocity_origin_integration)
        {

            Entity *anchor = _world.entities().find(_ship_entity);
            if (anchor && anchor->has_physics())
            {
                const Physics::BodyId anchor_body{anchor->physics_body_value()};
                if (_physics->is_body_valid(anchor_body))
                {
                    const double dt = static_cast<double>(fixed_dt);

                    if (_velocity_origin_mode == VelocityOriginMode::PerStepAnchorSync)
                    {
                        // Per-step: sync v_origin to the anchor's actual world velocity (v_world = v_local + v_origin),
                        // then apply the Galilean transform to keep v_local ~ 0.
                        const glm::vec3 v_local_f = _physics->get_linear_velocity(anchor_body);
                        const glm::dvec3 v_world = _physics_context->velocity_origin_world() + glm::dvec3(v_local_f);
                        (void) _physics_context->set_velocity_origin_world(v_world);
                        _physics->shift_velocity_origin(glm::dvec3(v_local_f));
                    }
                    else
                    {
                        // Threshold-friendly: integrate v_origin using the anchor's gravity acceleration, and let
                        // threshold-based velocity rebasing (if enabled) correct any accumulated drift.
                        const glm::dvec3 p_local_anchor = _physics->get_position(anchor_body);
                        const WorldVec3 p_world_anchor = physics_origin_world + WorldVec3(p_local_anchor);
                        anchor_accel_world = gravity_accel_world_at(p_world_anchor);
                        have_anchor_accel = true;

                        const glm::dvec3 v_origin_next = _physics_context->velocity_origin_world() + anchor_accel_world * dt;
                        (void) _physics_context->set_velocity_origin_world(v_origin_next);
                    }
                }
            }
        }

        const glm::dvec3 frame_accel_world =
            (wants_velocity_origin_integration && _velocity_origin_mode == VelocityOriginMode::FreeFallAnchorFrame &&
             have_anchor_accel)
                ? anchor_accel_world
                : glm::dvec3(0.0);

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
            const glm::dvec3 a_world = gravity_accel_world_at(p_world);
            const glm::dvec3 a_local = a_world - frame_accel_world;

            glm::vec3 v_local = _physics->get_linear_velocity(body_id);
            v_local += glm::vec3(a_local) * fixed_dt;
            _physics->set_linear_velocity(body_id, v_local);

            // Keep rotation stable (space sim demo, not attitude dynamics).
            _physics->set_angular_velocity(body_id, glm::vec3(0.0f));
            _physics->activate(body_id);
        };

        // Apply central gravity to the orbiting bodies (mass-independent).
        if (orbit_radius_m > 0.0)
        {
            apply_gravity_accel(_ship_entity);
            apply_gravity_accel(_probe_entity);
        }

        _physics->step(fixed_dt);

        // Advance the moving frame after stepping local physics for this dt.
        if (wants_velocity_origin_integration)
        {
            const glm::dvec3 v_origin = _physics_context->velocity_origin_world();
            if (std::isfinite(v_origin.x) && std::isfinite(v_origin.y) && std::isfinite(v_origin.z))
            {
                const double dt = static_cast<double>(fixed_dt);
                const WorldVec3 new_origin = _physics_context->origin_world() + WorldVec3(v_origin * dt);
                (void) _physics_context->set_origin_world(new_origin);
            }
        }

        _world.post_physics_step();

        _trail_sample_accum_s += static_cast<double>(fixed_dt);
        if (_draw_trail && _trail_sample_interval_s > 0.0 && _trail_sample_accum_s >= _trail_sample_interval_s)
        {
            _trail_sample_accum_s = 0.0;

            if (Entity *ship = _world.entities().find(_ship_entity))
            {
                _ship_trail_world.push_back(ship->position_world());
                while (_ship_trail_world.size() > _trail_max_points)
                {
                    _ship_trail_world.erase(_ship_trail_world.begin());
                }
            }
        }
#else
        (void) fixed_dt;
#endif
    }

    void RebasingTestGame::on_shutdown()
    {
        teardown_scene();

        _runtime = nullptr;
    }

    void RebasingTestGame::setup_scene()
    {
        if (!_runtime)
        {
            return;
        }

        auto &api = _runtime->api();

        teardown_scene();

        api.clear_all_instances();
        api.clear_planets(true);

        _ship_entity = EntityId{};
        _probe_entity = EntityId{};
        _moon_entity = EntityId{};
        _orbitsim.reset();
        _ship_trail_world.clear();
        _trail_sample_accum_s = 0.0;
        _origin_rebase_count = 0;
        _velocity_rebase_count = 0;
        _last_origin_delta_world = glm::dvec3(0.0);
        _last_velocity_delta_world = glm::dvec3(0.0);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        _physics = std::make_unique<Physics::JoltPhysicsWorld>();
        _physics->set_gravity(glm::vec3(0.0f));

        // Create PhysicsContext for coordinate system management
        _physics_context = std::make_unique<Physics::PhysicsContext>();
        _physics_context->set_physics_world(_physics.get());

        _world.set_physics(_physics.get());
        _world.set_physics_context(_physics_context.get());
#else
        _physics.reset();
        _physics_context.reset();
        _world.set_physics(nullptr);
        _world.set_physics_context(nullptr);
#endif

        // Initialize physics origin near the planet to avoid large local coordinates on creation.
        if (_physics_context)
        {
            _physics_context->set_origin_world(_planet_center_world);
            _physics_context->set_velocity_origin_world(glm::dvec3(0.0));
        }

        // Expose to EngineContext for systems that need the physics coordinate context (e.g., collider sync).
        VulkanEngine *renderer = _runtime->renderer();
        EngineContext *ctx = (renderer && renderer->_context) ? renderer->_context.get() : nullptr;
        if (ctx)
        {
            ctx->physics_context = _physics_context.get();
        }

        // Planet (terrain)
        {
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

            (void) api.add_planet_terrain(earth);
            api.set_planet_system_enabled(true);
            api.set_atmosphere_enabled(true);
            api.reset_atmosphere_to_earth();

            auto atmo = api.get_atmosphere_settings();
            atmo.bodyName = _planet_name;
            api.set_atmosphere_settings(atmo);
        }

        const double orbit_radius_m = _planet_radius_m + _orbit_altitude_m;
        const double speed_scale = std::max(0.0, _orbit_speed_scale);
        const double mu = _mu_base_m3ps2 * speed_scale * speed_scale;
        const double v_circ = (orbit_radius_m > 0.0) ? std::sqrt(mu / orbit_radius_m) : 0.0;

        WorldVec3 ship_pos_world = _planet_center_world + WorldVec3(orbit_radius_m, 0.0, 0.0);
        glm::dvec3 ship_vel_world_d(0.0, 0.0, v_circ);

        WorldVec3 probe_pos_world = ship_pos_world + WorldVec3(_probe_offset_world);
        glm::dvec3 probe_vel_world_d = ship_vel_world_d;

        WorldVec3 moon_pos_world(0.0);
        bool have_moon = false;

        if (_use_orbitsim)
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
                                                                  earth.mass_kg,
                                                                  moon.mass_kg,
                                                                  moon_sep_m,
                                                                  0.0);
            earth.state = em_init.state_a;
            moon.state = em_init.state_b;

            const auto earth_h = demo->sim.create_body(earth);
            const auto moon_h = demo->sim.create_body(moon);

            if (earth_h.valid() && moon_h.valid())
            {
                demo->earth_id = earth_h.id;
                demo->moon_id = moon_h.id;

                orbitsim::Spacecraft ship{};
                ship.dry_mass_kg = 1'000.0;
                ship.prop_mass_kg = 500.0;
                ship.engines.push_back(orbitsim::Engine{10'000.0, 320.0, 0.1});

                const OrbitRelativeState ship_rel =
                    circular_orbit_relative_state_xz(cfg.gravitational_constant,
                                                     earth.mass_kg,
                                                     std::max(1.0, orbit_radius_m),
                                                     0.0);
                ship.state = orbitsim::make_state(earth.state.position_m + ship_rel.position_m,
                                                  earth.state.velocity_mps + ship_rel.velocity_mps);

                const auto ship_h = demo->sim.create_spacecraft(ship);
                if (ship_h.valid())
                {
                    demo->ship_id = ship_h.id;

                    ship_pos_world = _planet_center_world + WorldVec3(ship_rel.position_m);
                    ship_vel_world_d = glm::dvec3(ship_rel.velocity_mps);

                    probe_pos_world = ship_pos_world + WorldVec3(_probe_offset_world);
                    probe_vel_world_d = ship_vel_world_d;

                    moon_pos_world = _planet_center_world + WorldVec3(moon.state.position_m - earth.state.position_m);
                    have_moon = true;

                    _orbitsim = std::move(demo);
                }
            }
        }

        if (_enable_velocity_rebasing && _physics_context)
        {
            _physics_context->set_velocity_origin_world(ship_vel_world_d);
        }

        const glm::dvec3 v_origin_world = _physics_context ? _physics_context->velocity_origin_world() : glm::dvec3(0.0);
        const glm::vec3 ship_vel_local_f = glm::vec3(ship_vel_world_d - v_origin_world);
        const glm::vec3 probe_vel_local_f = glm::vec3(probe_vel_world_d - v_origin_world);

        auto spawn_orbiter = [&](const std::string &name,
                                 const WorldVec3 &pos_world,
                                 const glm::vec3 &vel_local_f,
                                 const glm::vec3 &render_scale,
                                 EntityId &out_id) {
            Transform tr{};
            tr.position_world = pos_world;
            tr.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            tr.scale = render_scale;

            Entity *ent = nullptr;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics)
            {
                Physics::BodySettings settings{};
                settings.set_shape(Physics::CollisionShape::Sphere(1.0f))
                        .set_dynamic()
                        .set_gravity_scale(0.0f)
                        .set_linear_damping(0.0f)
                        .set_angular_damping(0.9f);

                ent = _world.builder(name)
                        .transform(tr)
                        .render_primitive(GameAPI::PrimitiveType::Sphere)
                        .physics(settings)
                        .build();
            }
            else
#endif
            {
                ent = _world.builder(name)
                        .transform(tr)
                        .render_primitive(GameAPI::PrimitiveType::Sphere)
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
                    _physics->set_angular_velocity(body_id, glm::vec3(0.0f));
                }
            }
#endif
        };

        spawn_orbiter("ship", ship_pos_world, ship_vel_local_f, glm::vec3(20'000.0f), _ship_entity);
        spawn_orbiter("probe", probe_pos_world, probe_vel_local_f, glm::vec3(12'000.0f), _probe_entity);

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

        if (_ship_entity.is_valid())
        {
            _world.set_rebase_anchor(_ship_entity);
        }
        apply_rebase_settings();

        {
            GameAPI::ChaseCameraSettings chase{};
            chase.target.type = GameAPI::CameraTargetType::MeshInstance;
            chase.target.name = "ship";
            chase.positionOffsetLocal = glm::vec3(0.0f, 250'000.0f, 800'000.0f);
            chase.rotationOffset = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            chase.positionLag = 6.0f;
            chase.rotationLag = 10.0f;
            api.set_camera_mode(GameAPI::CameraMode::Chase);
            api.set_chase_camera_settings(chase);
        }

        api.set_debug_draw_enabled(_debug_draw_enabled);
    }

    void RebasingTestGame::teardown_scene()
    {
        _world.clear_rebase_anchor();

        _world.clear();
        _world.set_physics(nullptr);
        _orbitsim.reset();
        _moon_entity = EntityId{};

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_runtime)
        {
            VulkanEngine *renderer = _runtime->renderer();
            EngineContext *ctx = (renderer && renderer->_context) ? renderer->_context.get() : nullptr;
            if (ctx)
            {
                if (ctx->physics_context == _physics_context.get())
                {
                    ctx->physics_context = nullptr;
                }
            }
        }
        _physics_context.reset();
#endif
        _physics.reset();
    }

    void RebasingTestGame::apply_rebase_settings()
    {
        GameWorld::RebaseSettings settings{};
        settings.origin_threshold_m = _enable_origin_rebasing ? std::max(0.0, _origin_threshold_m) : 0.0;
        settings.origin_snap_m = std::max(0.0, _origin_snap_m);
        settings.velocity_threshold_mps = _enable_velocity_rebasing ? std::max(0.0, _velocity_threshold_mps) : 0.0;
        _world.set_rebase_settings(settings);
    }

    void RebasingTestGame::draw_ui()
    {
        if (!_runtime || !_ui_open)
        {
            return;
        }

        auto &api = _runtime->api();

        if (!ImGui::Begin("Rebasing Test", &_ui_open, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::End();
            return;
        }

        ImGui::TextUnformatted("Big world + fast orbit (physics origin + velocity rebasing)");
        ImGui::Separator();

#if !(defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT)
        ImGui::TextUnformatted(
            "WARNING: Built without Jolt physics (rebasing test requires VULKAN_ENGINE_USE_JOLT=1).");
        ImGui::End();
        return;
#endif

        // Time controls
        float time_scale = _runtime->time_scale();
        if (ImGui::SliderFloat("Time scale", &time_scale, 0.0f, 10.0f, "%.2f"))
        {
            _runtime->set_time_scale(time_scale);
        }
        ImGui::Text("Fixed dt: %.6f s", _runtime->fixed_delta_time());

        ImGui::Separator();
        ImGui::Checkbox("Integrate origin from velocity origin", &_integrate_origin_from_velocity_origin);
        {
            int mode_idx = (_velocity_origin_mode == VelocityOriginMode::PerStepAnchorSync) ? 0 : 1;
            const char *modes[] = {"Per-step anchor sync", "Free-fall (threshold-friendly)"};
            if (ImGui::Combo("Velocity origin mode", &mode_idx, modes, IM_ARRAYSIZE(modes)))
            {
                _velocity_origin_mode =
                    (mode_idx == 0) ? VelocityOriginMode::PerStepAnchorSync : VelocityOriginMode::FreeFallAnchorFrame;
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Orbit (requires reset)");

        const double min0 = 0.0;
        bool orbit_changed = false;
        orbit_changed |= ImGui::Checkbox("Use orbitsim (Earth-Moon)", &_use_orbitsim);
        orbit_changed |= ImGui::DragScalar("Altitude (m)", ImGuiDataType_Double, &_orbit_altitude_m, 10'000.0f, &min0,
                                           nullptr, "%.0f");
        orbit_changed |= ImGui::DragScalar("Speed scale", ImGuiDataType_Double, &_orbit_speed_scale, 0.1f, &min0,
                                           nullptr, "%.2f");
        orbit_changed |= ImGui::DragScalar("Probe offset Y (m)", ImGuiDataType_Double, &_probe_offset_world.y, 1'000.0,
                                           nullptr, nullptr, "%.0f");
        if (_use_orbitsim)
        {
            orbit_changed |= ImGui::DragScalar("Moon distance (m)", ImGuiDataType_Double, &_moon_distance_m, 100'000.0f,
                                               &min0, nullptr, "%.0f");
        }

        if (orbit_changed)
        {
            _orbit_altitude_m = std::max(0.0, _orbit_altitude_m);
            _orbit_speed_scale = std::max(0.0, _orbit_speed_scale);
            if (_use_orbitsim)
            {
                const double min_sep_m = _planet_radius_m * 2.0;
                _moon_distance_m = std::max(min_sep_m, _moon_distance_m);
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Rebasing (live)");

        ImGui::Checkbox("Enable origin rebasing", &_enable_origin_rebasing);
        ImGui::SameLine();
        ImGui::Checkbox("Enable velocity rebasing", &_enable_velocity_rebasing);

        ImGui::DragScalar("Origin threshold (m)", ImGuiDataType_Double, &_origin_threshold_m, 1'000.0f, &min0, nullptr,
                          "%.0f");
        ImGui::DragScalar("Origin snap (m)", ImGuiDataType_Double, &_origin_snap_m, 500.0f, &min0, nullptr, "%.0f");
        ImGui::DragScalar("Velocity threshold (local m/s)", ImGuiDataType_Double, &_velocity_threshold_mps, 50.0f,
                          &min0, nullptr, "%.0f");

        ImGui::Separator();
        ImGui::TextUnformatted("Debug draw");

        if (ImGui::Checkbox("Enable debug draw", &_debug_draw_enabled))
        {
            api.set_debug_draw_enabled(_debug_draw_enabled);
        }

        ImGui::Checkbox("Orbit circle", &_draw_orbit_circle);
        ImGui::SameLine();
        ImGui::Checkbox("Trail", &_draw_trail);

        const double min_trail = 0.01;
        const double max_trail = 5.0;
        const double max_vel_vec = 60.0;
        ImGui::DragScalar("Trail sample (s)", ImGuiDataType_Double, &_trail_sample_interval_s, 0.05f, &min_trail,
                          &max_trail, "%.2f");
        ImGui::DragScalar("Velocity vector (s)", ImGuiDataType_Double, &_velocity_vector_seconds, 0.1f, &min0,
                          &max_vel_vec, "%.1f");
        ImGui::DragScalar("Debug TTL (s)", ImGuiDataType_Double, &_debug_draw_ttl_s, 0.05f, &min0, nullptr, "%.2f");

        ImGui::Separator();
        ImGui::TextUnformatted("State");

        const glm::dvec3 world_origin = api.get_world_origin();
        const glm::dvec3 phys_origin = _physics_context ? glm::dvec3(_physics_context->origin_world()) : glm::dvec3(0.0);
        const glm::dvec3 vel_origin = _physics_context ? _physics_context->velocity_origin_world() : glm::dvec3(0.0);

        ImGui::Text("World origin (m):   %.3f, %.3f, %.3f", world_origin.x, world_origin.y, world_origin.z);
        ImGui::Text("Physics origin (m):  %.3f, %.3f, %.3f", phys_origin.x, phys_origin.y, phys_origin.z);
        ImGui::Text("Vel origin (m/s):    %.3f, %.3f, %.3f", vel_origin.x, vel_origin.y, vel_origin.z);

        ImGui::Text("Origin rebases:   %llu (last delta %.1f, %.1f, %.1f)",
                    static_cast<unsigned long long>(_origin_rebase_count),
                    _last_origin_delta_world.x, _last_origin_delta_world.y, _last_origin_delta_world.z);
        ImGui::Text("Velocity rebases: %llu (last delta %.1f, %.1f, %.1f)",
                    static_cast<unsigned long long>(_velocity_rebase_count),
                    _last_velocity_delta_world.x, _last_velocity_delta_world.y, _last_velocity_delta_world.z);

        if (_use_orbitsim)
        {
            ImGui::Separator();
            ImGui::TextUnformatted("orbitsim");
            if (_orbitsim)
            {
                ImGui::Text("t (s): %.3f", _orbitsim->sim.time_s());

                const orbitsim::MassiveBody *earth = _orbitsim->sim.body_by_id(_orbitsim->earth_id);
                const orbitsim::MassiveBody *moon = _orbitsim->sim.body_by_id(_orbitsim->moon_id);
                const orbitsim::Spacecraft *ship_sc = _orbitsim->sim.spacecraft_by_id(_orbitsim->ship_id);
                if (earth && moon)
                {
                    const orbitsim::Vec3 moon_rel_m = moon->state.position_m - earth->state.position_m;
                    ImGui::Text("Moon rel (m): %.0f, %.0f, %.0f (|r|=%.0f)",
                                moon_rel_m.x,
                                moon_rel_m.y,
                                moon_rel_m.z,
                                safe_length(moon_rel_m));
                }
                if (earth && ship_sc)
                {
                    const orbitsim::Vec3 ship_rel_m = ship_sc->state.position_m - earth->state.position_m;
                    ImGui::Text("Ship rel (m): %.0f, %.0f, %.0f (|r|=%.0f)",
                                ship_rel_m.x,
                                ship_rel_m.y,
                                ship_rel_m.z,
                                safe_length(ship_rel_m));
                }
            }
            else
            {
                ImGui::TextUnformatted("Not initialized (reset required).");
            }
        }

        if (Entity *ship = _world.entities().find(_ship_entity))
        {
            ImGui::Separator();
            ImGui::TextUnformatted("Ship (anchor)");
            const WorldVec3 p_world = ship->position_world();
            ImGui::Text("p_world (m): %.3f, %.3f, %.3f", p_world.x, p_world.y, p_world.z);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics && ship->has_physics())
            {
                const Physics::BodyId body_id{ship->physics_body_value()};
                if (_physics->is_body_valid(body_id))
                {
                    const glm::dvec3 p_local = _physics->get_position(body_id);
                    const glm::vec3 v_local_f = _physics->get_linear_velocity(body_id);
                    const glm::dvec3 v_world = vel_origin + glm::dvec3(v_local_f);

                    ImGui::Text("p_local (m): %.3f, %.3f, %.3f", p_local.x, p_local.y, p_local.z);
                    ImGui::Text("v_local (m/s): %.3f, %.3f, %.3f (|v|=%.3f)",
                                v_local_f.x, v_local_f.y, v_local_f.z, glm::length(v_local_f));
                    ImGui::Text("v_world (m/s): %.3f, %.3f, %.3f (|v|=%.3f)",
                                v_world.x, v_world.y, v_world.z, static_cast<float>(safe_length(v_world)));
                }
            }
#endif
        }

        ImGui::Separator();
        if (ImGui::Button("Reset simulation"))
        {
            _reset_requested = true;
        }

        ImGui::End();
    }

    void RebasingTestGame::draw_debug()
    {
        if (!_runtime || !_debug_draw_enabled)
        {
            return;
        }

        auto &api = _runtime->api();
        const float ttl_s = static_cast<float>(std::max(0.0, _debug_draw_ttl_s));

        Entity *ship = _world.entities().find(_ship_entity);
        if (!ship)
        {
            return;
        }

        const WorldVec3 ship_pos_world = ship->get_render_position_world(_runtime->interpolation_alpha());

        api.debug_draw_line(glm::dvec3(_planet_center_world), glm::dvec3(ship_pos_world), COLOR_PLANET_TO_SHIP, ttl_s,
                            true);

        if (Entity *moon = _world.entities().find(_moon_entity))
        {
            const WorldVec3 moon_pos_world = moon->get_render_position_world(_runtime->interpolation_alpha());
            api.debug_draw_line(glm::dvec3(_planet_center_world), glm::dvec3(moon_pos_world), COLOR_PLANET_TO_MOON,
                                ttl_s, true);
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics && ship->has_physics() && _velocity_vector_seconds > 0.0)
        {
            const Physics::BodyId body_id{ship->physics_body_value()};
            if (_physics->is_body_valid(body_id))
            {
                const glm::vec3 v_local_f = _physics->get_linear_velocity(body_id);
                const glm::dvec3 v_origin = _physics_context ? _physics_context->velocity_origin_world() : glm::dvec3(0.0);
                const glm::dvec3 v_world = v_origin + glm::dvec3(v_local_f);

                const WorldVec3 end_world =
                        ship_pos_world + WorldVec3(v_world * _velocity_vector_seconds);

                api.debug_draw_line(glm::dvec3(ship_pos_world), glm::dvec3(end_world), COLOR_VELOCITY, ttl_s, false);
            }
        }
#endif

        if (_draw_orbit_circle)
        {
            const double orbit_radius_m = _planet_radius_m + _orbit_altitude_m;
            if (std::isfinite(orbit_radius_m) && orbit_radius_m > 0.0 && orbit_radius_m < static_cast<double>(
                    std::numeric_limits<float>::max()))
            {
                api.debug_draw_circle(glm::dvec3(_planet_center_world),
                                      glm::dvec3(0.0, 1.0, 0.0),
                                      static_cast<float>(orbit_radius_m),
                                      COLOR_ORBIT,
                                      ttl_s,
                                      true);
            }
        }

        if (_draw_trail && _ship_trail_world.size() >= 2)
        {
            for (size_t i = 1; i < _ship_trail_world.size(); ++i)
            {
                api.debug_draw_line(glm::dvec3(_ship_trail_world[i - 1]),
                                    glm::dvec3(_ship_trail_world[i]),
                                    COLOR_TRAIL,
                                    ttl_s,
                                    true);
            }
        }
    }
} // namespace Game
