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
        _prediction_altitude_km.clear();
        _prediction_speed_kmps.clear();
        _prediction_points_world.clear();
        _prediction_dirty = true;

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

        if (ctx.input && ctx.input->key_pressed(Key::Escape))
        {
            _pending = StateTransition::push<PauseState>();
            return;
        }

        const float alpha = ctx.interpolation_alpha();
        ComponentContext comp_ctx = build_component_context(ctx, alpha);
        _world.entities().update_components(comp_ctx, dt);
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

        ComponentContext comp_ctx = build_component_context(ctx);
        _world.entities().fixed_update_components(comp_ctx, fixed_dt);

        update_prediction(ctx, fixed_dt);
        step_physics(ctx, fixed_dt);
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
                            const glm::vec3 td = sc->last_thrust_dir();
                            ImGui::Text("SAS: %s  [T] toggle", sc->sas_enabled() ? "ON " : "OFF");
                            ImGui::Text("Thrust: (%.1f, %.1f, %.1f)", td.x, td.y, td.z);

                            if (ctx.renderer && ctx.renderer->ui())
                            {
                                ImGui::Text("UI capture keyboard: %s",
                                            ctx.renderer->ui()->wantCaptureKeyboard() ? "YES" : "NO");
                            }

                            if (player->has_physics() && _physics)
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
                ImGui::Checkbox("Prediction velocity ray", &_prediction_draw_velocity_ray);

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

                    if (ImGui::Checkbox("Legacy prediction fallback (Euler)", &_prediction_allow_legacy_fallback))
                    {
                        _prediction_dirty = true;
                    }
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
