#include "gameplay_state.h"
#include "pause_state.h"
#include "runtime/game_runtime.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/input/input_system.h"
#include "core/util/logger.h"

#include "imgui.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <string_view>

#include "orbitsim/game_sim.hpp"
#include "orbitsim/orbit_utils.hpp"
#include "orbitsim/trajectories.hpp"
#include "orbitsim/trajectory_transforms.hpp"

namespace Game
{
    struct OrbitsimDemo
    {
        orbitsim::GameSimulation sim{};
        orbitsim::BodyId earth_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId moon_id{orbitsim::kInvalidBodyId};

        double earth_mass_kg{0.0};
    };

    GameplayState::GameplayState() = default;

    GameplayState::~GameplayState() = default;

    namespace
    {
        bool finite_vec3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        const char *contact_event_type_name(Physics::ContactEventType type)
        {
            switch (type)
            {
                case Physics::ContactEventType::Begin:
                    return "Begin";
                case Physics::ContactEventType::Stay:
                    return "Stay";
                case Physics::ContactEventType::End:
                    return "End";
            }
            return "Unknown";
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

        glm::dvec3 point_mass_accel(const double gravitational_constant,
                                    const double mass_kg,
                                    const glm::dvec3 &r_m,
                                    const double softening_length2_m2)
        {
            if (!(gravitational_constant > 0.0) || !(mass_kg > 0.0))
            {
                return glm::dvec3(0.0);
            }

            const double r2 = glm::dot(r_m, r_m) + softening_length2_m2;
            if (!std::isfinite(r2) || r2 <= 0.0)
            {
                return glm::dvec3(0.0);
            }

            const double inv_r = 1.0 / std::sqrt(r2);
            const double inv_r3 = inv_r * inv_r * inv_r;
            const glm::dvec3 a = (-gravitational_constant * mass_kg) * r_m * inv_r3;

            if (!finite_vec3(a))
            {
                return glm::dvec3(0.0);
            }

            return a;
        }

        // Acceleration in a translating Earth-centered frame:
        //   a_rel = a_sc_bary - a_earth_bary
        // where barycentric acceleration is computed from all massive bodies.
        glm::dvec3 orbitsim_nbody_accel_earth_fixed(const OrbitsimDemo &demo, const glm::dvec3 &p_rel_m)
        {
            const orbitsim::MassiveBody *earth = demo.sim.body_by_id(demo.earth_id);
            if (!earth)
            {
                return glm::dvec3(0.0);
            }

            const double G = demo.sim.config().gravitational_constant;
            const double eps_m = demo.sim.config().softening_length_m;
            const double eps2 = eps_m * eps_m;

            const glm::dvec3 p_earth_bary = earth->state.position_m;
            const glm::dvec3 p_sc_bary = p_earth_bary + p_rel_m;

            glm::dvec3 a_sc_bary(0.0);
            glm::dvec3 a_earth_bary(0.0);

            // Spacecraft acceleration from central Earth.
            a_sc_bary += point_mass_accel(G, earth->mass_kg, p_rel_m, eps2);

            // Differential acceleration from other bodies (e.g., Moon).
            for (const orbitsim::MassiveBody &body: demo.sim.massive_bodies())
            {
                if (body.id == demo.earth_id)
                {
                    continue;
                }

                a_sc_bary += point_mass_accel(G, body.mass_kg, p_sc_bary - glm::dvec3(body.state.position_m), eps2);
                a_earth_bary += point_mass_accel(G, body.mass_kg, p_earth_bary - glm::dvec3(body.state.position_m),
                                                 eps2);
            }

            return a_sc_bary - a_earth_bary;
        }
    } // namespace

    void GameplayState::on_enter(GameStateContext &ctx)
    {
        _world.set_api(ctx.api);
        _elapsed = 0.0f;
        _fixed_time_s = 0.0;
        _reset_requested = false;

        setup_scene(ctx);
    }

    void GameplayState::on_exit(GameStateContext &ctx)
    {
        _world.clear_rebase_anchor();
        _world.clear();
        _world.set_physics(nullptr);
        _world.set_physics_context(nullptr);
        _world.set_api(nullptr);
        _orbitsim.reset();
        _contact_log.clear();
        _prediction_update_accum_s = 0.0f;
        _prediction_altitude_km.clear();
        _prediction_speed_kmps.clear();
        _prediction_points_world.clear();
        _ship_entity = EntityId{};
        _probe_entity = EntityId{};
        _moon_entity = EntityId{};

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (ctx.renderer && ctx.renderer->_context)
        {
            if (ctx.renderer->_context->physics_context == _physics_context.get())
            {
                ctx.renderer->_context->physics_context = nullptr;
            }
        }
        _physics_context.reset();
        _physics.reset();
#endif
    }

    void GameplayState::on_update(GameStateContext &ctx, float dt)
    {
        if (_reset_requested)
        {
            _reset_requested = false;
            setup_scene(ctx);
            return;
        }

        _elapsed += dt;

        // ESC to pause
        if (ctx.input && ctx.input->key_pressed(Key::Escape))
        {
            _pending = StateTransition::push<PauseState>();
            return;
        }

        // Update components (variable timestep)
        const float alpha = ctx.interpolation_alpha();
        ComponentContext comp_ctx = build_component_context(ctx, alpha);
        _world.entities().update_components(comp_ctx, dt);

        // Sync entities to render
        _world.entities().sync_to_render(*ctx.api, alpha);
    }

    void GameplayState::on_fixed_update(GameStateContext &ctx, float fixed_dt)
    {
        if (_reset_requested)
        {
            _reset_requested = false;
            setup_scene(ctx);
            return;
        }

        _fixed_time_s += static_cast<double>(fixed_dt);

        // Fixed update components (input → forces, game logic)
        ComponentContext comp_ctx = build_component_context(ctx);
        _world.entities().fixed_update_components(comp_ctx, fixed_dt);

        bool prediction_cache_refreshed = false;
        if (_prediction_enabled)
        {
            const float default_interval_s = 0.25f;
            const float update_interval_s =
                    (std::isfinite(_prediction_update_interval_s) && _prediction_update_interval_s > 0.0f)
                        ? _prediction_update_interval_s
                        : default_interval_s;

            _prediction_update_accum_s += fixed_dt;
            if (_prediction_points_world.empty() || _prediction_update_accum_s >= update_interval_s)
            {
                WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
                glm::dvec3 ship_vel_world(0.0);
                glm::vec3 ship_vel_local_f(0.0f);
                if (get_ship_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
                {
                    update_orbit_prediction_cache(ship_pos_world, ship_vel_world);
                    prediction_cache_refreshed = true;
                }
                else
                {
                    _prediction_altitude_km.clear();
                    _prediction_speed_kmps.clear();
                    _prediction_points_world.clear();
                }

                _prediction_update_accum_s = std::fmod(_prediction_update_accum_s, update_interval_s);
            }
        }
        else if (!_prediction_altitude_km.empty() || !_prediction_speed_kmps.empty() || !_prediction_points_world.
                 empty())
        {
            _prediction_update_accum_s = 0.0f;
            _prediction_altitude_km.clear();
            _prediction_speed_kmps.clear();
            _prediction_points_world.clear();
        }

        if (prediction_cache_refreshed)
        {
            emit_orbit_prediction_debug(ctx);
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_physics)
        {
            return;
        }

        _world.pre_physics_step();

        const bool use_orbitsim = _orbitsim && _orbitsim->earth_id != orbitsim::kInvalidBodyId;
        if (use_orbitsim)
        {
            const double dt_s = static_cast<double>(fixed_dt);
            _orbitsim->sim.step(dt_s);

            // Visualize Moon's orbit around Earth (Earth-centered).
            const orbitsim::MassiveBody *earth = _orbitsim->sim.body_by_id(_orbitsim->earth_id);
            const orbitsim::MassiveBody *moon = _orbitsim->sim.body_by_id(_orbitsim->moon_id);
            if (earth && moon)
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

        auto gravity_accel_world_at = [&](const WorldVec3 &p_world) -> glm::dvec3 {
            if (!_orbitsim)
            {
                return glm::dvec3(0.0);
            }

            const glm::dvec3 p_rel = glm::dvec3(p_world - _planet_center_world);
            if (use_orbitsim)
            {
                return orbitsim_nbody_accel_earth_fixed(*_orbitsim, p_rel);
            }

            if (!(_orbitsim->earth_mass_kg > 0.0))
            {
                return glm::dvec3(0.0);
            }

            const double G = orbitsim::kGravitationalConstant_SI;
            return point_mass_accel(G, _orbitsim->earth_mass_kg, p_rel, 0.0);
        };

        // Velocity-origin integration.
        //
        // We integrate v_origin using the anchor's world acceleration and apply gravity in the anchor's free-fall frame:
        //   a_local = a_world - a_anchor_world.
        // This keeps the anchor's v_local near 0 without calling shift_velocity_origin every physics step.
        glm::dvec3 anchor_accel_world(0.0);
        bool have_anchor_accel = false;
        const WorldVec3 physics_origin_world = _physics_context->origin_world();

        if (_ship_entity.is_valid())
        {
            Entity *anchor = _world.entities().find(_ship_entity);
            if (anchor && anchor->has_physics())
            {
                const Physics::BodyId anchor_body{anchor->physics_body_value()};
                if (_physics->is_body_valid(anchor_body))
                {
                    const glm::dvec3 p_local_anchor = _physics->get_position(anchor_body);
                    const WorldVec3 p_world_anchor = physics_origin_world + WorldVec3(p_local_anchor);
                    anchor_accel_world = gravity_accel_world_at(p_world_anchor);
                    have_anchor_accel = true;

                    const double dt = static_cast<double>(fixed_dt);
                    const glm::dvec3 v_origin_next =
                            _physics_context->velocity_origin_world() + anchor_accel_world * dt;
                    (void) _physics_context->set_velocity_origin_world(v_origin_next);
                }
            }
        }

        const glm::dvec3 frame_accel_world = have_anchor_accel ? anchor_accel_world : glm::dvec3(0.0);

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

            _physics->activate(body_id);
        };

        apply_gravity_accel(_ship_entity);
        apply_gravity_accel(_probe_entity);

        _physics->step(fixed_dt);

        // Advance the moving frame after stepping local physics for this dt.
        {
            const glm::dvec3 v_origin = _physics_context->velocity_origin_world();
            if (finite_vec3(v_origin))
            {
                const double dt = static_cast<double>(fixed_dt);
                const WorldVec3 new_origin = _physics_context->origin_world() + WorldVec3(v_origin * dt);
                (void) _physics_context->set_origin_world(new_origin);
            }
        }

        _world.post_physics_step();
#endif
    }

    void GameplayState::on_draw_ui(GameStateContext &ctx)
    {
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 10, viewport->WorkPos.y + 10));
        ImGui::SetNextWindowBgAlpha(0.4f);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize;

        if (ImGui::Begin("##GameplayHUD", nullptr, flags))
        {
            ImGui::Text("Time: %.1f s (fixed %.2f)", _elapsed, _fixed_time_s);
            ImGui::Text("[ESC] Pause");

#if !(defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT)
            ImGui::Separator();
            ImGui::TextUnformatted(
                "WARNING: Built without Jolt physics (collision test requires VULKAN_ENGINE_USE_JOLT=1).");
#endif

            if (ImGui::Button("Reset scenario"))
            {
                _reset_requested = true;
            }
            ImGui::SameLine();
            if (ImGui::Button("Replay collision"))
            {
                _reset_requested = true;
            }

            ImGui::Checkbox("Contact log", &_contact_log_enabled);
            ImGui::SameLine();
            ImGui::Checkbox("Print console", &_contact_log_print_console);

            if (ctx.api)
            {
                if (ImGui::Checkbox("Debug draw", &_debug_draw_enabled))
                {
                    ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
                }
            }

            ImGui::Separator();
            ImGui::Text("Contacts: %zu", _contact_log.size());

            const int max_lines = 6;
            const int n = static_cast<int>(std::min(_contact_log.size(), static_cast<size_t>(max_lines)));
            for (int i = 0; i < n; ++i)
            {
                const ContactLogEntry &e = _contact_log[_contact_log.size() - 1 - static_cast<size_t>(i)];
                ImGui::Text("[%s][%.2fs] self=%u other=%u depth=%.3f p=(%.2f,%.2f,%.2f)",
                            contact_event_type_name(e.type),
                            e.time_s,
                            e.self_body,
                            e.other_body,
                            e.penetration_depth,
                            e.point.x, e.point.y, e.point.z);
            }

            ImGui::Separator();
            if (ImGui::CollapsingHeader("Orbit", ImGuiTreeNodeFlags_DefaultOpen))
            {
                WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
                glm::dvec3 ship_vel_world(0.0);
                glm::vec3 ship_vel_local_f(0.0f);

                const bool have_ship = get_ship_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f);
                if (!have_ship)
                {
                    ImGui::TextUnformatted("Ship state unavailable.");
                }
                else
                {
                    const glm::dvec3 p_rel = glm::dvec3(ship_pos_world - _planet_center_world);
                    const double r_m = glm::length(p_rel);
                    const double alt_m = r_m - _planet_radius_m;
                    const double speed_mps = glm::length(ship_vel_world);

                    const double speed_scale = std::max(0.0, _orbit_speed_scale);
                    const double mu = _mu_base_m3ps2 * speed_scale * speed_scale;
                    const double v_circ_est = (r_m > 1.0) ? std::sqrt(mu / r_m) : 0.0;

                    ImGui::Text("Altitude: %.0f m", alt_m);
                    ImGui::Text("Speed:    %.3f km/s (v_circ est %.3f km/s)", speed_mps * 1.0e-3, v_circ_est * 1.0e-3);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                    if (_physics && _physics_context && _ship_entity.is_valid())
                    {
                        const glm::dvec3 v_origin_world = _physics_context->velocity_origin_world();
                        ImGui::Text("v_origin: %.1f, %.1f, %.1f m/s", v_origin_world.x, v_origin_world.y,
                                    v_origin_world.z);
                        ImGui::Text("v_local:  %.2f, %.2f, %.2f m/s", ship_vel_local_f.x, ship_vel_local_f.y,
                                    ship_vel_local_f.z);

                        const Entity *ship = _world.entities().find(_ship_entity);
                        if (ship && ship->has_physics())
                        {
                            const Physics::BodyId body_id{ship->physics_body_value()};
                            if (_physics->is_body_valid(body_id))
                            {
                                const glm::vec3 w_local_f = _physics->get_angular_velocity(body_id);
                                ImGui::Text("w_local:  %.3f, %.3f, %.3f rad/s (|w|=%.3f)",
                                            w_local_f.x, w_local_f.y, w_local_f.z, glm::length(w_local_f));
                            }
                        }
                    }
#endif
                }
            }
        }
        ImGui::End();
    }

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
        // If we're resetting inside GameplayState, detach the previous context from the renderer first.
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

        // Expose to EngineContext for debug draw
        if (ctx.renderer && ctx.renderer->_context)
        {
            ctx.renderer->_context->physics_context = _physics_context.get();
        }
#endif

        if (!ctx.api)
        {
            return;
        }

        // IBL / background
        if (ctx.renderer && ctx.renderer->_assetManager)
        {
            GameAPI::IBLPaths ibl{};
            ibl.specularCube = ctx.renderer->_assetManager->assetPath("ibl/darkstar.ktx2");
            ibl.diffuseCube = ctx.renderer->_assetManager->assetPath("ibl/darkstar.ktx2");
            ibl.brdfLut = ctx.renderer->_assetManager->assetPath("ibl/brdf_lut.ktx2");
            ibl.background = ctx.renderer->_assetManager->assetPath("ibl/darkstar.ktx2");
            ctx.api->load_global_ibl(ibl);
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

            (void) ctx.api->add_planet_terrain(earth);
            ctx.api->set_planet_system_enabled(true);
            ctx.api->set_atmosphere_enabled(true);
            ctx.api->reset_atmosphere_to_earth();

            auto atmo = ctx.api->get_atmosphere_settings();
            atmo.bodyName = _planet_name;
            ctx.api->set_atmosphere_settings(atmo);
        }

        const double orbit_radius_m = _planet_radius_m + _orbit_altitude_m;
        const double speed_scale = std::max(0.0, _orbit_speed_scale);
        const double mu = _mu_base_m3ps2 * speed_scale * speed_scale;
        const double v_circ = (orbit_radius_m > 0.0) ? std::sqrt(mu / orbit_radius_m) : 0.0;

        WorldVec3 ship_pos_world = _planet_center_world + WorldVec3(orbit_radius_m, 0.0, 0.0);
        glm::dvec3 ship_vel_world_d(0.0, 0.0, v_circ);

        const glm::dvec3 probe_rel_vel_world_d(0.0, 0.0, -10.0);
        glm::dvec3 probe_vel_world_d = ship_vel_world_d + probe_rel_vel_world_d;
        WorldVec3 probe_pos_world = ship_pos_world + WorldVec3(_probe_offset_world);

        WorldVec3 moon_pos_world(0.0);
        bool have_moon = false; {
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
                demo->earth_mass_kg = earth.mass_kg;

                const OrbitRelativeState ship_rel =
                        circular_orbit_relative_state_xz(cfg.gravitational_constant,
                                                         earth.mass_kg,
                                                         std::max(1.0, orbit_radius_m),
                                                         0.0);

                ship_pos_world = _planet_center_world + WorldVec3(ship_rel.position_m);
                ship_vel_world_d = glm::dvec3(ship_rel.velocity_mps);

                probe_vel_world_d = ship_vel_world_d + probe_rel_vel_world_d;
                probe_pos_world = ship_pos_world + WorldVec3(_probe_offset_world);

                moon_pos_world = _planet_center_world + WorldVec3(moon.state.position_m - earth.state.position_m);
                have_moon = true;

                _orbitsim = std::move(demo);
            }
            else
            {
                _orbitsim = std::move(demo);
            }
        }

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

        // Optional automatic position rebasing: disabled for the early orbit collision test.
        _world.set_rebase_anchor(_ship_entity);
        _world.set_rebase_settings(GameWorld::RebaseSettings{});

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        // Install contact callbacks on the ship only to keep logs easy to read.
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

    bool GameplayState::get_ship_world_state(WorldVec3 &out_pos_world,
                                             glm::dvec3 &out_vel_world,
                                             glm::vec3 &out_vel_local) const
    {
        const Entity *ship = _world.entities().find(_ship_entity);
        if (!ship)
        {
            return false;
        }

        out_pos_world = ship->position_world();
        out_vel_world = glm::dvec3(0.0);
        out_vel_local = glm::vec3(0.0f);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics && _physics_context && ship->has_physics())
        {
            const Physics::BodyId body_id{ship->physics_body_value()};
            if (_physics->is_body_valid(body_id))
            {
                out_vel_local = _physics->get_linear_velocity(body_id);
                out_vel_world = _physics_context->velocity_origin_world() + glm::dvec3(out_vel_local);
            }
        }
#endif

        return true;
    }

    void GameplayState::update_orbit_prediction_cache(const WorldVec3 &ship_pos_world, const glm::dvec3 &ship_vel_world)
    {
        _prediction_altitude_km.clear();
        _prediction_speed_kmps.clear();
        _prediction_points_world.clear();

        if (!_orbitsim)
        {
            return;
        }

        if (!std::isfinite(_prediction_dt_s) || _prediction_dt_s <= 0.0)
        {
            return;
        }

        if (!std::isfinite(_prediction_horizon_s) || _prediction_horizon_s <= 0.0)
        {
            return;
        }

        const double dt_s = std::clamp(_prediction_dt_s, 0.01, 60.0);
        const double horizon_s = std::clamp(_prediction_horizon_s, dt_s, 36'000.0);

        constexpr int max_steps = 512;
        const int steps = std::clamp(static_cast<int>(std::ceil(horizon_s / dt_s)), 2, max_steps);

        _prediction_altitude_km.reserve(static_cast<size_t>(steps) + 1);
        _prediction_speed_kmps.reserve(static_cast<size_t>(steps) + 1);
        _prediction_points_world.reserve(static_cast<size_t>(steps) + 1);

        OrbitsimDemo demo_pred = *_orbitsim; // copy sim so prediction setup doesn't perturb live sim

        auto safe_length = [](const glm::dvec3 &v) -> double {
            const double len2 = glm::dot(v, v);
            if (!std::isfinite(len2) || len2 <= 0.0)
            {
                return 0.0;
            }
            return std::sqrt(len2);
        };

        const bool can_use_orbitsim_predict = demo_pred.earth_id != orbitsim::kInvalidBodyId;
        if (can_use_orbitsim_predict)
        {
            const orbitsim::MassiveBody *earth = demo_pred.sim.body_by_id(demo_pred.earth_id);
            if (earth)
            {
                const glm::dvec3 ship_rel_pos_m = glm::dvec3(ship_pos_world - _planet_center_world);
                const glm::dvec3 ship_bary_pos_m = earth->state.position_m + ship_rel_pos_m;
                const glm::dvec3 ship_bary_vel_mps = earth->state.velocity_mps + ship_vel_world;

                orbitsim::Spacecraft ship_sc{};
                ship_sc.state = orbitsim::make_state(ship_bary_pos_m, ship_bary_vel_mps);
                ship_sc.dry_mass_kg = 1.0;

                const auto ship_h = demo_pred.sim.create_spacecraft(ship_sc);
                if (ship_h.valid())
                {
                    orbitsim::TrajectoryOptions opt{};
                    opt.duration_s = horizon_s;
                    opt.sample_dt_s = dt_s;
                    opt.spacecraft_sample_dt_s = dt_s;
                    opt.spacecraft_lookup_dt_s = dt_s;
                    opt.celestial_dt_s = dt_s;
                    opt.max_samples = static_cast<size_t>(steps) + 1;
                    opt.include_start = true;
                    opt.include_end = true;
                    opt.stop_on_impact = false;

                    const orbitsim::CelestialEphemeris eph = orbitsim::build_celestial_ephemeris(demo_pred.sim, opt);
                    const std::vector<orbitsim::TrajectorySample> traj_inertial =
                            orbitsim::predict_spacecraft_trajectory(demo_pred.sim, eph, ship_h.id, opt);

                    if (!traj_inertial.empty())
                    {
                        const std::vector<orbitsim::TrajectorySample> traj_earth_centered =
                                orbitsim::trajectory_to_body_centered_inertial(traj_inertial, eph, *earth);

                        if (!traj_earth_centered.empty())
                        {
                            _prediction_altitude_km.clear();
                            _prediction_speed_kmps.clear();
                            _prediction_points_world.clear();
                            _prediction_altitude_km.reserve(traj_earth_centered.size());
                            _prediction_speed_kmps.reserve(traj_earth_centered.size());
                            _prediction_points_world.reserve(traj_earth_centered.size());

                            for (const orbitsim::TrajectorySample &sample: traj_earth_centered)
                            {
                                const double r_m = safe_length(sample.position_m);
                                const double alt_km = (r_m - _planet_radius_m) * 1.0e-3;
                                const double spd_kmps = safe_length(sample.velocity_mps) * 1.0e-3;

                                _prediction_altitude_km.push_back(static_cast<float>(alt_km));
                                _prediction_speed_kmps.push_back(static_cast<float>(spd_kmps));
                                _prediction_points_world.push_back(_planet_center_world + WorldVec3(sample.position_m));
                            }
                            return;
                        }
                    }
                }
            }
        }

        // Fallback: legacy local integration when orbitsim prediction is unavailable.
        glm::dvec3 p_rel_m = glm::dvec3(ship_pos_world - _planet_center_world);
        glm::dvec3 v_rel_mps = ship_vel_world;
        const bool use_orbitsim = demo_pred.earth_id != orbitsim::kInvalidBodyId;

        for (int i = 0; i <= steps; ++i)
        {
            const double r_m = safe_length(p_rel_m);
            const double alt_km = (r_m - _planet_radius_m) * 1.0e-3;
            const double spd_kmps = safe_length(v_rel_mps) * 1.0e-3;

            _prediction_altitude_km.push_back(static_cast<float>(alt_km));
            _prediction_speed_kmps.push_back(static_cast<float>(spd_kmps));
            _prediction_points_world.push_back(_planet_center_world + WorldVec3(p_rel_m));

            if (i == steps)
            {
                break;
            }

            glm::dvec3 a_rel(0.0);
            if (use_orbitsim)
            {
                a_rel = orbitsim_nbody_accel_earth_fixed(demo_pred, p_rel_m);
                demo_pred.sim.step(dt_s);
            }
            else if (demo_pred.earth_mass_kg > 0.0)
            {
                a_rel = point_mass_accel(orbitsim::kGravitationalConstant_SI, demo_pred.earth_mass_kg, p_rel_m, 0.0);
            }

            if (!finite_vec3(a_rel))
            {
                a_rel = glm::dvec3(0.0);
            }

            v_rel_mps += a_rel * dt_s;
            p_rel_m += v_rel_mps * dt_s;
        }
    }

    void GameplayState::emit_orbit_prediction_debug(GameStateContext &ctx)
    {
        if (!_prediction_enabled || !_debug_draw_enabled)
        {
            return;
        }

        if (!ctx.api)
        {
            return;
        }

        if (_prediction_points_world.size() < 2)
        {
            return;
        }

        const float default_interval_s = 0.25f;
        const float update_interval_s =
                (std::isfinite(_prediction_update_interval_s) && _prediction_update_interval_s > 0.0f)
                    ? _prediction_update_interval_s
                    : default_interval_s;
        const float ttl_s = std::max(std::max(_prediction_debug_ttl_s, update_interval_s + 0.05f), 0.11f);

        constexpr glm::vec4 color_orbit{0.2f, 0.9f, 0.2f, 0.65f};
        constexpr glm::vec4 color_velocity{1.0f, 0.35f, 0.1f, 1.0f};

        for (size_t i = 1; i < _prediction_points_world.size(); ++i)
        {
            ctx.api->debug_draw_line(glm::dvec3(_prediction_points_world[i - 1]),
                                     glm::dvec3(_prediction_points_world[i]),
                                     color_orbit,
                                     ttl_s,
                                     true);
        }

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (get_ship_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
        {
            const double speed_mps = glm::length(ship_vel_world);
            double len_m = 40.0;
            if (std::isfinite(speed_mps) && speed_mps > 1.0)
            {
                len_m = std::clamp(speed_mps * 0.002, 10.0, 250.0);
            }

            ctx.api->debug_draw_ray(glm::dvec3(ship_pos_world), ship_vel_world, len_m, color_velocity, ttl_s, true);
        }
    }

    ComponentContext GameplayState::build_component_context(GameStateContext &ctx, float alpha)
    {
        ComponentContext comp_ctx{};
        comp_ctx.world = &_world;
        comp_ctx.api = ctx.api;
        comp_ctx.input = ctx.input;
        comp_ctx.physics = _physics.get();
        comp_ctx.interpolation_alpha = alpha;
        return comp_ctx;
    }
} // namespace Game
