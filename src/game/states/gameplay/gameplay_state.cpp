#include "gameplay_state.h"
#include "orbit_helpers.h"
#include "scenario_loader.h"
#include "game/states/pause_state.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/input/input_system.h"
#include "core/util/logger.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <filesystem>

namespace Game
{
    using detail::contact_event_type_name;

    GameplayState::GameplayState()
        : _scenario_config(default_earth_moon_config())
    {
    }

    GameplayState::~GameplayState() = default;

    void GameplayState::on_enter(GameStateContext &ctx)
    {
        _world.set_api(ctx.api);
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
        _reset_requested = false;
        _scenario_io_status.clear();
        _scenario_io_status_ok = true;

        // Try loading scenario from JSON; fall back to compiled default.
        // NOTE: JSON data is authoritative when present, including orbiter body mass.
        if (ctx.renderer && ctx.renderer->_assetManager)
        {
            const std::string scenario_path =
                    ctx.renderer->_assetManager->assetPath("scenarios/default_gameplay.json");
            if (auto loaded = load_scenario_config(scenario_path))
            {
                _scenario_config = std::move(*loaded);
            }
            else
            {
                Logger::warn("Falling back to compiled default scenario config.");
                _scenario_config = default_earth_moon_config();
            }
        }

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
        _orbiters.clear();
        _contact_log.clear();
        _prediction_cache.clear();
        _prediction_dirty = true;
        reset_time_warp_state();
        _warp_to_time_active = false;
        _warp_to_time_target_s = 0.0;
        _warp_to_time_restore_level = 0;
        _execute_node_armed = false;
        _execute_node_id = -1;
        _maneuver_state.nodes.clear();
        _maneuver_state.selected_node_id = -1;
        _maneuver_state.next_node_id = 0;

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

        handle_time_warp_input(ctx);

        if (ctx.input && ctx.input->key_pressed(Key::Escape))
        {
            _pending = StateTransition::push<PauseState>();
            return;
        }

        const float alpha = ctx.interpolation_alpha();
        ComponentContext comp_ctx = build_component_context(ctx, alpha);
        _world.entities().update_components(comp_ctx, dt);
        _world.entities().sync_to_render(*ctx.api, alpha);

        // Draw orbit debug using the same interpolation alpha as rendering to avoid visual offset.
        emit_orbit_prediction_debug(ctx);
        emit_maneuver_node_debug_overlay(ctx);
    }

    void GameplayState::on_fixed_update(GameStateContext &ctx, float fixed_dt)
    {
        if (_reset_requested)
        {
            _reset_requested = false;
            setup_scene(ctx);
            return;
        }

        update_maneuver_nodes_time_warp(ctx, fixed_dt);

        const TimeWarpState::Mode desired_mode = _time_warp.mode_for_level(_time_warp.warp_level);
        _time_warp.mode = desired_mode;

        const double warp_factor = _time_warp.factor();

        if (desired_mode == TimeWarpState::Mode::RailsWarp)
        {
            if (!_rails_warp_active)
            {
                enter_rails_warp(ctx);
            }

            if (_rails_warp_active)
            {
                double dt_s = static_cast<double>(fixed_dt) * warp_factor;
                if (_warp_to_time_active)
                {
                    const double now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
                    const double remaining_s = _warp_to_time_target_s - now_s;
                    if (std::isfinite(remaining_s) && remaining_s > 0.0)
                    {
                        dt_s = std::min(dt_s, remaining_s);
                    }
                }
                _fixed_time_s += dt_s;
                _last_sim_step_dt_s = dt_s;

                rails_warp_step(ctx, dt_s);
                update_maneuver_nodes_execution(ctx);
                update_prediction(ctx, static_cast<float>(dt_s));
                return;
            }
        }

        const int physics_steps =
                (desired_mode == TimeWarpState::Mode::PhysicsWarp)
                    ? std::max(1, static_cast<int>(warp_factor))
                    : 1;

        const double effective_dt_s = static_cast<double>(fixed_dt) * static_cast<double>(physics_steps);
        _last_sim_step_dt_s = static_cast<double>(fixed_dt);

        ComponentContext comp_ctx = build_component_context(ctx);

        for (int i = 0; i < physics_steps; ++i)
        {
            _fixed_time_s += static_cast<double>(fixed_dt);
            _world.entities().fixed_update_components(comp_ctx, fixed_dt);
            step_physics(ctx, fixed_dt);
        }

        update_maneuver_nodes_execution(ctx);
        update_prediction(ctx, static_cast<float>(effective_dt_s));
    }

    void GameplayState::on_draw_ui(GameStateContext &ctx)
    {
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 10, viewport->WorkPos.y + 10));
        ImGui::SetNextWindowBgAlpha(0.4f);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize;

        if (ImGui::Begin("##GameplayHUD", nullptr, flags))
        {
            const double sim_time_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
            const int sim_hours = static_cast<int>(std::floor(sim_time_s / 3600.0));
            const int sim_minutes = static_cast<int>(std::floor(std::fmod(sim_time_s, 3600.0) / 60.0));
            const double sim_seconds = std::fmod(sim_time_s, 60.0);

            const char *warp_mode = "Realtime";
            switch (_time_warp.mode)
            {
                case TimeWarpState::Mode::Realtime:
                    warp_mode = "Realtime";
                    break;
                case TimeWarpState::Mode::PhysicsWarp:
                    warp_mode = "Physics";
                    break;
                case TimeWarpState::Mode::RailsWarp:
                    warp_mode = "Rails";
                    break;
            }

            ImGui::Text("Sim: %dh %dm %.1fs", sim_hours, sim_minutes, sim_seconds);
            ImGui::Text("Warp: x%.0f (%s)  [.,] change  [/]/[Backspace] x1", _time_warp.factor(), warp_mode);
            ImGui::Text("Real: %.1f s", _elapsed);
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

            const auto resolve_slot_path = [&ctx, this]() -> std::string {
                const std::filesystem::path slot_rel(_scenario_slot_rel_path);
                if (slot_rel.is_absolute())
                {
                    return slot_rel.string();
                }
                if (ctx.renderer && ctx.renderer->_assetManager)
                {
                    const AssetPaths &paths = ctx.renderer->_assetManager->paths();
                    if (!paths.assets.empty())
                    {
                        return (paths.assets / slot_rel).string();
                    }
                }
                return slot_rel.string();
            };
            const std::string scenario_slot_path = resolve_slot_path();

            ImGui::SameLine();
            if (ImGui::Button("Save scenario slot"))
            {
                if (save_scenario_config(scenario_slot_path, _scenario_config))
                {
                    _scenario_io_status = "Saved scenario: " + scenario_slot_path;
                    _scenario_io_status_ok = true;
                }
                else
                {
                    _scenario_io_status = "Save failed: " + scenario_slot_path;
                    _scenario_io_status_ok = false;
                }
            }

            ImGui::SameLine();
            if (ImGui::Button("Load scenario slot"))
            {
                if (auto loaded = load_scenario_config(scenario_slot_path))
                {
                    _scenario_config = std::move(*loaded);
                    _scenario_io_status = "Loaded scenario: " + scenario_slot_path;
                    _scenario_io_status_ok = true;
                    _reset_requested = true;
                }
                else
                {
                    _scenario_io_status = "Load failed: " + scenario_slot_path;
                    _scenario_io_status_ok = false;
                }
            }

            ImGui::Text("Scenario slot: %s", scenario_slot_path.c_str());
            if (!_scenario_io_status.empty())
            {
                if (_scenario_io_status_ok)
                {
                    ImGui::TextUnformatted(_scenario_io_status.c_str());
                }
                else
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", _scenario_io_status.c_str());
                }
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

            // Ship controller HUD
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            {
                const EntityId player_eid = player_entity();
                if (player_eid.is_valid())
                {
                    Entity *player = _world.entities().find(player_eid);
                    if (player)
                    {
                        auto *sc = player->get_component<ShipController>();
                        if (sc)
                        {
                            ImGui::Separator();
                            const bool rails_warp = _rails_warp_active && _time_warp.mode == TimeWarpState::Mode::RailsWarp;
                            const glm::vec3 td = rails_warp ? _rails_last_thrust_dir_local : sc->last_thrust_dir();
                            ImGui::Text("SAS: %s  [T] toggle", sc->sas_enabled() ? "ON " : "OFF");
                            ImGui::Text("Thrust input: (%.1f, %.1f, %.1f)%s",
                                        td.x, td.y, td.z,
                                        (rails_warp && _rails_thrust_applied_this_tick) ? " [applied]" : "");

                            if (ctx.renderer && ctx.renderer->ui())
                            {
                                ImGui::Text("UI capture keyboard: %s",
                                            ctx.renderer->ui()->wantCaptureKeyboard() ? "YES" : "NO");
                            }

                            if (rails_warp)
                            {
                                WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
                                glm::dvec3 ship_vel_world(0.0);
                                glm::vec3 ship_vel_local_f(0.0f);
                                if (get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
                                {
                                    ImGui::Text("Speed(world): %.2f m/s", glm::length(ship_vel_world));
                                }
                            }
                            else if (player->has_physics() && _physics)
                            {
                                const Physics::BodyId body_id{player->physics_body_value()};
                                if (_physics->is_body_valid(body_id))
                                {
                                    const Physics::MotionType motion = _physics->get_motion_type(body_id);
                                    const char *motion_str =
                                            (motion == Physics::MotionType::Dynamic)
                                                ? "Dynamic"
                                                : (motion == Physics::MotionType::Kinematic) ? "Kinematic (forces ignored)" : "Static";
                                    ImGui::Text("Motion: %s", motion_str);

                                    float thrust = sc->thrust_force();
                                    if (ImGui::DragFloat("Thrust force (N)", &thrust, 1000.0f, 0.0f, 1.0e9f, "%.1f"))
                                    {
                                        sc->set_thrust_force(thrust);
                                    }

                                    float torque = sc->torque_strength();
                                    if (ImGui::DragFloat("Torque strength (N*m)", &torque, 1000.0f, 0.0f, 1.0e9f, "%.1f"))
                                    {
                                        sc->set_torque_strength(torque);
                                    }

                                    float sas = sc->sas_damping();
                                    if (ImGui::DragFloat("SAS damping", &sas, 0.1f, 0.0f, 1.0e4f, "%.2f"))
                                    {
                                        sc->set_sas_damping(sas);
                                    }

                                    const glm::vec3 vel = _physics->get_linear_velocity(body_id);
                                    ImGui::Text("Speed(local): %.2f m/s", glm::length(vel));
                                    if (_physics_context)
                                    {
                                        const glm::dvec3 v_world = _physics_context->velocity_origin_world() + glm::dvec3(vel);
                                        ImGui::Text("Speed(world): %.2f m/s", glm::length(v_world));
                                    }
                                }
                            }
                        }
                    }
                }
            }
#endif

            ImGui::Separator();
            if (ImGui::CollapsingHeader("Orbit", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Checkbox("Prediction full orbit", &_prediction_draw_full_orbit);
                ImGui::Checkbox("Prediction future segment", &_prediction_draw_future_segment);

                float future_window_s = static_cast<float>(_prediction_future_window_s);
                if (ImGui::DragFloat("Prediction future window (s)", &future_window_s, 10.0f, 0.0f, 36000.0f, "%.0f"))
                {
                    _prediction_future_window_s = static_cast<double>(std::max(0.0f, future_window_s));
                }

                float refresh_s = static_cast<float>(_prediction_periodic_refresh_s);
                if (ImGui::DragFloat("Prediction refresh (s)", &refresh_s, 1.0f, 0.0f, 36000.0f, "%.1f"))
                {
                    _prediction_periodic_refresh_s = static_cast<double>(std::max(0.0f, refresh_s));
                }
                ImGui::SameLine();
                ImGui::TextUnformatted("(0 = never)");

                float thrust_refresh_s = static_cast<float>(_prediction_thrust_refresh_s);
                if (ImGui::DragFloat("Prediction thrust refresh (s)",
                                     &thrust_refresh_s,
                                     0.01f,
                                     0.0f,
                                     2.0f,
                                     "%.2f"))
                {
                    _prediction_thrust_refresh_s = static_cast<double>(std::max(0.0f, thrust_refresh_s));
                }
                ImGui::SameLine();
                ImGui::TextUnformatted("(0 = every fixed tick)");

                ImGui::Checkbox("Prediction velocity ray", &_prediction_draw_velocity_ray);

                float prediction_alpha_scale = _prediction_line_alpha_scale;
                if (ImGui::DragFloat("Prediction line alpha scale",
                                     &prediction_alpha_scale,
                                     0.05f,
                                     0.1f,
                                     8.0f,
                                     "%.2f"))
                {
                    _prediction_line_alpha_scale = std::clamp(prediction_alpha_scale, 0.1f, 8.0f);
                }

                float prediction_overlay_boost = _prediction_line_overlay_boost;
                if (ImGui::DragFloat("Prediction line overlay boost",
                                     &prediction_overlay_boost,
                                     0.01f,
                                     0.0f,
                                     1.0f,
                                     "%.2f"))
                {
                    _prediction_line_overlay_boost = std::clamp(prediction_overlay_boost, 0.0f, 1.0f);
                }
                ImGui::SameLine();
                ImGui::TextUnformatted("(0 = depth-only)");

                WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
                glm::dvec3 ship_vel_world(0.0);
                glm::vec3 ship_vel_local_f(0.0f);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                if (_physics && _physics_context)
                {
                    int mode_idx = (_velocity_origin_mode == VelocityOriginMode::PerStepAnchorSync) ? 0 : 1;
                    const char *modes[] = {"Per-step anchor sync", "Free-fall anchor frame"};
                    if (ImGui::Combo("Velocity origin mode", &mode_idx, modes, IM_ARRAYSIZE(modes)))
                    {
                        _velocity_origin_mode =
                                (mode_idx == 0)
                                    ? VelocityOriginMode::PerStepAnchorSync
                                    : VelocityOriginMode::FreeFallAnchorFrame;
                        _prediction_dirty = true;
                    }

                    // Keep local velocities bounded (Jolt stores them as float).
                    GameWorld::RebaseSettings rs = _world.rebase_settings();
                    float v_rebase = static_cast<float>(rs.velocity_threshold_mps);
                    if (ImGui::DragFloat("Velocity rebase threshold (m/s)", &v_rebase, 50.0f, 0.0f, 100000.0f, "%.1f"))
                    {
                        rs.velocity_threshold_mps = static_cast<double>(std::max(0.0f, v_rebase));
                        _world.set_rebase_settings(rs);
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("(0 = off)");

                }

                const EntityId player_eid = player_entity();
                if (_physics && player_eid.is_valid())
                {
                    const Entity *player = _world.entities().find(player_eid);
                    if (player && player->has_physics())
                    {
                        const Physics::BodyId body_id{player->physics_body_value()};
                        if (_physics->is_body_valid(body_id))
                        {
                            const Physics::MotionType motion = _physics->get_motion_type(body_id);
                            bool kinematic = motion == Physics::MotionType::Kinematic;
                            if (ImGui::Checkbox("Primary player kinematic", &kinematic))
                            {
                                const Physics::MotionType target =
                                        kinematic ? Physics::MotionType::Kinematic : Physics::MotionType::Dynamic;
                                (void) _physics->set_motion_type(body_id, target);
                            }
                            ImGui::SameLine();
                            ImGui::TextUnformatted("Anchor source: orbiter config (is_rebase_anchor).");
                        }
                    }
                }
#endif

                const bool have_player = get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f);
                if (!have_player)
                {
                    ImGui::TextUnformatted("Ship state unavailable.");
                }
                else
                {
                    if (_prediction_cache.valid && _orbitsim)
                    {
                        const double age_s = _orbitsim->sim.time_s() - _prediction_cache.build_time_s;
                        ImGui::Text("Prediction: %zu pts, age %.1f s", _prediction_cache.points_world.size(), age_s);
                    }

                    const auto &cfg = _scenario_config;
                    const double planet_radius_m =
                            (_orbitsim && _orbitsim->reference_body())
                                ? _orbitsim->reference_body()->radius_m
                                : (cfg.celestials.empty() ? 0.0 : cfg.celestials[0].radius_m);

                    const glm::dvec3 p_rel = glm::dvec3(ship_pos_world - cfg.system_center);
                    const double r_m = glm::length(p_rel);
                    const double alt_m = r_m - planet_radius_m;
                    const double speed_mps = glm::length(ship_vel_world);

                    const double speed_scale = std::max(0.0, cfg.speed_scale);
                    const double mu = cfg.mu_base * speed_scale * speed_scale;
                    const double v_circ_est = (r_m > 1.0) ? std::sqrt(mu / r_m) : 0.0;

                    ImGui::Text("Altitude: %.0f m", alt_m);
                    ImGui::Text("Speed:    %.3f km/s (v_circ est %.3f km/s)", speed_mps * 1.0e-3, v_circ_est * 1.0e-3);
                    if (_prediction_cache.valid)
                    {
                        ImGui::Text("Predicted Pe: %.1f km", _prediction_cache.periapsis_alt_km);
                        if (std::isfinite(_prediction_cache.apoapsis_alt_km))
                        {
                            ImGui::Text("Predicted Ap: %.1f km", _prediction_cache.apoapsis_alt_km);
                        }
                        else
                        {
                            ImGui::TextUnformatted("Predicted Ap: escape");
                        }

                        if (_prediction_cache.orbital_period_s > 0.0 && std::isfinite(_prediction_cache.orbital_period_s))
                        {
                            ImGui::Text("Predicted Period: %.2f min", _prediction_cache.orbital_period_s / 60.0);
                        }
                        else
                        {
                            ImGui::TextUnformatted("Predicted Period: N/A");
                        }
                    }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                    const EntityId player_eid = player_entity();
                    if (_physics && _physics_context && player_eid.is_valid())
                    {
                        const glm::dvec3 v_origin_world = _physics_context->velocity_origin_world();
                        ImGui::Text("v_origin: %.1f, %.1f, %.1f m/s", v_origin_world.x, v_origin_world.y,
                                    v_origin_world.z);
                        ImGui::Text("v_local:  %.2f, %.2f, %.2f m/s", ship_vel_local_f.x, ship_vel_local_f.y,
                                    ship_vel_local_f.z);

                        const Entity *player = _world.entities().find(player_eid);
                        if (player && player->has_physics())
                        {
                            const Physics::BodyId body_id{player->physics_body_value()};
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

        draw_maneuver_nodes_panel(ctx);
    }

    void GameplayState::reset_time_warp_state()
    {
        _time_warp.warp_level = 0;
        _time_warp.mode = TimeWarpState::Mode::Realtime;
        _rails_warp_active = false;
        _last_sim_step_dt_s = 0.0;
        _rails_thrust_applied_this_tick = false;
        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);
    }

    void GameplayState::handle_time_warp_input(GameStateContext &ctx)
    {
        if (!ctx.input)
        {
            return;
        }

        if (_warp_to_time_active)
        {
            return;
        }

        const bool ui_capture_keyboard = ctx.renderer && ctx.renderer->ui() && ctx.renderer->ui()->wantCaptureKeyboard();
        if (ui_capture_keyboard)
        {
            return;
        }

        int level = _time_warp.warp_level;
        bool changed = false;

        if (ctx.input->key_pressed(Key::Period))
        {
            level += 1;
            changed = true;
        }
        if (ctx.input->key_pressed(Key::Comma))
        {
            level -= 1;
            changed = true;
        }
        if (ctx.input->key_pressed(Key::Slash) || ctx.input->key_pressed(Key::Backspace))
        {
            level = 0;
            changed = true;
        }

        if (changed)
        {
            set_time_warp_level(ctx, level);
        }
    }

    ComponentContext GameplayState::build_component_context(GameStateContext &ctx, float alpha)
    {
        ComponentContext comp_ctx{};
        comp_ctx.world = &_world;
        comp_ctx.api = ctx.api;
        comp_ctx.input = ctx.input;
        comp_ctx.physics = _physics.get();
        comp_ctx.ui_capture_keyboard = ctx.renderer && ctx.renderer->ui() && ctx.renderer->ui()->wantCaptureKeyboard();
        comp_ctx.interpolation_alpha = alpha;
        return comp_ctx;
    }
} // namespace Game
