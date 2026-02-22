#include "gameplay_state.h"
#include "orbit_helpers.h"
#include "game/states/pause_state.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/input/input_system.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    using detail::contact_event_type_name;

    GameplayState::GameplayState() = default;

    GameplayState::~GameplayState() = default;

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
                    }
                }
#endif

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
