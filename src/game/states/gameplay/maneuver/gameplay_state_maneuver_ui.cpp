#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_colors.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"

#include "core/game_api.h"
#include "core/engine.h"
#include "core/input/input_system.h"

#include "game/orbit/orbit_prediction_math.h"

#include "imgui.h"

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace
    {
        using OrbitPredictionMath::safe_length;
        using namespace ManeuverUtil;
    } // namespace

    void GameplayState::update_maneuver_ui_config(GameStateContext &ctx)
    {
        if (!ctx.renderer || !ctx.renderer->_window)
        {
            _maneuver_ui_config.effective_scale = _maneuver_ui_config.ui_scale;
            return;
        }

        int win_w = 0, win_h = 0, draw_w = 0, draw_h = 0;
        SDL_GetWindowSize(ctx.renderer->_window, &win_w, &win_h);
        SDL_Vulkan_GetDrawableSize(ctx.renderer->_window, &draw_w, &draw_h);

        float dpi_scale = 1.0f;
        if (_maneuver_ui_config.auto_dpi_scale && win_w > 0 && draw_w > 0)
        {
            dpi_scale = static_cast<float>(draw_w) / static_cast<float>(win_w);
        }

        float display_dpi = _maneuver_ui_config.base_dpi;
        const int display_idx = SDL_GetWindowDisplayIndex(ctx.renderer->_window);
        if (display_idx >= 0)
        {
            float reported_dpi = 0.0f;
            if (SDL_GetDisplayDPI(display_idx, nullptr, &reported_dpi, nullptr) == 0 && reported_dpi > 0.0f)
            {
                display_dpi = reported_dpi;
            }
        }

        _maneuver_ui_config.dpi = display_dpi;
        _maneuver_ui_config.effective_scale = _maneuver_ui_config.ui_scale * std::max(dpi_scale, display_dpi / _maneuver_ui_config.base_dpi);
        _maneuver_ui_config.effective_scale = std::clamp(_maneuver_ui_config.effective_scale, 0.5f, 4.0f);
    }

    const char *GameplayState::maneuver_axis_label(const ManeuverHandleAxis axis) const
    {
        return ManeuverUtil::axis_short_label(_maneuver_gizmo_basis_mode, axis);
    }

    uint32_t GameplayState::maneuver_axis_color(const ManeuverHandleAxis axis) const
    {
        switch (axis)
        {
            case ManeuverHandleAxis::TangentialPos:
            case ManeuverHandleAxis::TangentialNeg:
                return ManeuverColors::kAxisTangential;
            case ManeuverHandleAxis::RadialPos:
            case ManeuverHandleAxis::RadialNeg:
                return ManeuverColors::kAxisRadial;
            case ManeuverHandleAxis::NormalPos:
            case ManeuverHandleAxis::NormalNeg:
                return ManeuverColors::kAxisNormal;
            default:
                return ManeuverColors::kAxisDefault;
        }
    }

    void GameplayState::draw_maneuver_gizmo_markers(ImDrawList *draw_list,
                                                    const std::vector<ManeuverHubMarker> &hubs,
                                                    const std::vector<ManeuverAxisMarker> &handles,
                                                    const int hovered_hub_idx,
                                                    const int hovered_handle_idx,
                                                    const float hub_hit_px,
                                                    const float axis_hit_px) const
    {
        // Visual state is entirely screen-space here; the world-space solve already happened during marker generation.
        if (!draw_list)
        {
            return;
        }

        auto to_imvec2 = [](const glm::vec2 &p) -> ImVec2 {
            return ImVec2(p.x, p.y);
        };

        auto clamped_style_color = [&](const float boost, const float alpha_scale) -> ImVec4 {
            const glm::vec4 base = _maneuver_gizmo_style.icon_color;
            return ImVec4(std::clamp(base.x + boost, 0.0f, 1.0f),
                          std::clamp(base.y + boost, 0.0f, 1.0f),
                          std::clamp(base.z + boost, 0.0f, 1.0f),
                          std::clamp(base.w * alpha_scale, 0.1f, 1.0f));
        };

        for (const ManeuverHubMarker &hub : hubs)
        {
            const bool is_selected = (hub.node_id == _maneuver_state.selected_node_id);
            const bool is_hovered = (hovered_hub_idx >= 0 && hubs[hovered_hub_idx].node_id == hub.node_id);
            const float radius = is_selected ? hub_hit_px * 0.58f : hub_hit_px * 0.50f;

            const ImVec4 fill_rgba = is_selected
                                         ? ManeuverColors::kHubSelectedFill
                                         : (is_hovered
                                                ? clamped_style_color(0.18f, 1.0f)
                                                : clamped_style_color(0.0f, 0.95f));
            const ImVec4 ring_rgba = is_selected
                                         ? ManeuverColors::kHubSelectedRing
                                         : clamped_style_color(0.25f, 0.72f);

            draw_list->AddCircleFilled(to_imvec2(hub.screen), radius, ImGui::ColorConvertFloat4ToU32(fill_rgba), 24);
            draw_list->AddCircle(to_imvec2(hub.screen), radius + _maneuver_ui_config.scaled(2.5f), ImGui::ColorConvertFloat4ToU32(ring_rgba), 24, _maneuver_ui_config.scaled(1.8f));
        }

        for (const ManeuverAxisMarker &h : handles)
        {
            const bool is_hovered = (hovered_handle_idx >= 0 &&
                                     handles[hovered_handle_idx].node_id == h.node_id &&
                                     handles[hovered_handle_idx].axis == h.axis);
            const bool is_active = (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis) &&
                                   (_maneuver_gizmo_interaction.node_id == h.node_id) &&
                                   (_maneuver_gizmo_interaction.axis == h.axis);

            const ImU32 col = is_active
                                  ? ManeuverColors::kDragAxisActive
                                  : (is_hovered ? ManeuverColors::kDragAxisHovered : static_cast<ImU32>(h.base_color));
            const float thickness = is_active ? 3.2f : (is_hovered ? 2.8f : 2.2f);
            const float handle_r = is_active
                                       ? axis_hit_px * 0.95f
                                       : (is_hovered ? axis_hit_px * 0.88f : axis_hit_px * 0.78f);

            draw_list->AddLine(to_imvec2(h.hub_screen), to_imvec2(h.handle_screen), col, thickness);
            draw_list->AddCircleFilled(to_imvec2(h.handle_screen), handle_r, col, 24);
            if (_maneuver_gizmo_style.show_axis_labels)
            {
                draw_list->AddText(ImVec2(h.handle_screen.x + handle_r + _maneuver_ui_config.scaled(2.0f), h.handle_screen.y - handle_r), col, h.label);
            }
        }
    }

    void GameplayState::draw_maneuver_gizmo_hover_tooltip(const std::vector<ManeuverAxisMarker> &handles,
                                                          const int hovered_handle_idx) const
    {
        if (hovered_handle_idx < 0 || hovered_handle_idx >= static_cast<int>(handles.size()))
        {
            return;
        }

        const ManeuverAxisMarker &hover_h = handles[hovered_handle_idx];
        if (const ManeuverNode *node = _maneuver_state.find_node(hover_h.node_id))
        {
            ImGui::BeginTooltip();
            ImGui::Text("Node %d", node->id);
            ImGui::Text("Axis: %s", hover_h.label);
            ImGui::Text("Basis: %s", ManeuverUtil::basis_mode_label(_maneuver_gizmo_basis_mode));
            const char *dv_fmt = (node->total_dv_mps < 1.0) ? "DV RTN: (%.4f, %.4f, %.4f) m/s" : "DV RTN: (%.2f, %.2f, %.2f) m/s";
            ImGui::Text(dv_fmt, node->dv_rtn_mps.x, node->dv_rtn_mps.y, node->dv_rtn_mps.z);
            if (_maneuver_gizmo_basis_mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal)
            {
                const glm::dvec3 dv_display = dv_rtn_to_display_basis(*node);
                const char *pon_fmt = (node->total_dv_mps < 1.0) ? "DV PON: (%.4f, %.4f, %.4f) m/s" : "DV PON: (%.2f, %.2f, %.2f) m/s";
                ImGui::Text(pon_fmt, dv_display.y, dv_display.x, dv_display.z);
            }
            ImGui::Text("Total: %.3f m/s", node->total_dv_mps);
            ImGui::TextUnformatted("Drag LMB (Shift x0.1 / Ctrl x10)");
            ImGui::EndTooltip();
        }
    }

    void GameplayState::draw_maneuver_imgui_gizmo(GameStateContext &ctx)
    {
        // Per-frame gizmo loop: refresh runtime cache, rebuild markers, resolve hover/drag state, then draw overlay UI.
        if (!_maneuver_nodes_enabled || _maneuver_state.nodes.empty())
        {
            return;
        }
        if (!ctx.input || !ctx.renderer || !ctx.renderer->_sceneManager || !ctx.renderer->_swapchainManager ||
            !ctx.renderer->_window)
        {
            return;
        }

        update_maneuver_ui_config(ctx);
        refresh_maneuver_node_runtime_cache(ctx);

        ManeuverGizmoViewContext view{};
        if (!build_maneuver_gizmo_view_context(ctx, view))
        {
            return;
        }

        const float overlay_scale = std::max(0.25f, _maneuver_gizmo_style.overlay_scale);
        const float overlay_size_px = std::max(6.0f, _maneuver_ui_config.scaled(_maneuver_gizmo_style.icon_size_px * overlay_scale));

        std::vector<ManeuverHubMarker> hubs{};
        std::vector<ManeuverAxisMarker> handles{};
        build_maneuver_gizmo_markers(view, overlay_size_px, hubs, handles);

        const ImGuiIO &io = ImGui::GetIO();
        const glm::vec2 mouse_pos(io.MousePos.x, io.MousePos.y);

        const float hub_hit_px = std::max(_maneuver_ui_config.scaled(10.0f), overlay_size_px * 0.45f);
        const float axis_hit_px = std::max(_maneuver_ui_config.scaled(9.0f), overlay_size_px * 0.36f);
        const float hub_hit_px2 = hub_hit_px * hub_hit_px;
        const float axis_hit_px2 = axis_hit_px * axis_hit_px;

        int hovered_handle_idx = -1;
        int hovered_hub_idx = -1;
        find_maneuver_gizmo_hover(hubs, handles, mouse_pos, hub_hit_px2, axis_hit_px2, hovered_hub_idx, hovered_handle_idx);

        const bool ui_item_active = ImGui::IsAnyItemActive();
        const bool ui_item_hovered = ImGui::IsAnyItemHovered();
        const bool can_capture_click = !(ui_item_active || ui_item_hovered);

        if (_maneuver_gizmo_interaction.state != ManeuverGizmoInteraction::State::DragAxis)
        {
            if (hovered_handle_idx >= 0)
            {
                _maneuver_gizmo_interaction.state = ManeuverGizmoInteraction::State::HoverAxis;
                _maneuver_gizmo_interaction.node_id = handles[hovered_handle_idx].node_id;
                _maneuver_gizmo_interaction.axis = handles[hovered_handle_idx].axis;
            }
            else
            {
                _maneuver_gizmo_interaction = {};
            }
        }

        if (ctx.input->mouse_pressed(MouseButton::Left) && can_capture_click)
        {
            if (hovered_handle_idx >= 0)
            {
                _maneuver_state.selected_node_id = handles[hovered_handle_idx].node_id;
                (void) begin_maneuver_axis_drag(ctx, handles[hovered_handle_idx].node_id, handles[hovered_handle_idx].axis);
            }
            else if (hovered_hub_idx >= 0)
            {
                _maneuver_state.selected_node_id = hubs[hovered_hub_idx].node_id;
            }
        }

        if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            ManeuverNode *node = _maneuver_state.find_node(_maneuver_gizmo_interaction.node_id);
            if (!node || !node->gizmo_valid)
            {
                if (PredictionTrackState *track = active_prediction_track())
                {
                    PredictionDragDebugTelemetry &debug = track->drag_debug;
                    debug.drag_active = false;
                    debug.last_drag_end_tp = PredictionDragDebugTelemetry::Clock::now();
                }
                _maneuver_gizmo_interaction = {};
            }
            else if (!ctx.input->mouse_down(MouseButton::Left))
            {
                const bool changed = _maneuver_gizmo_interaction.applied_delta;
                if (PredictionTrackState *track = active_prediction_track())
                {
                    PredictionDragDebugTelemetry &debug = track->drag_debug;
                    debug.drag_active = false;
                    debug.last_drag_end_tp = PredictionDragDebugTelemetry::Clock::now();
                }
                if (hovered_handle_idx >= 0)
                {
                    _maneuver_gizmo_interaction.state = ManeuverGizmoInteraction::State::HoverAxis;
                    _maneuver_gizmo_interaction.node_id = handles[hovered_handle_idx].node_id;
                    _maneuver_gizmo_interaction.axis = handles[hovered_handle_idx].axis;
                    _maneuver_gizmo_interaction.applied_delta = false;
                }
                else
                {
                    _maneuver_gizmo_interaction = {};
                }

                if (changed)
                {
                    mark_maneuver_plan_dirty();
                }
            }
            else
            {
                apply_maneuver_axis_drag(ctx, *node, mouse_pos);
            }
        }

        ImDrawList *dl = ImGui::GetForegroundDrawList();
        if (!dl)
        {
            return;
        }

        draw_maneuver_gizmo_markers(dl, hubs, handles, hovered_hub_idx, hovered_handle_idx, hub_hit_px, axis_hit_px);

        if (hovered_handle_idx >= 0 && _maneuver_gizmo_interaction.state != ManeuverGizmoInteraction::State::DragAxis)
        {
            draw_maneuver_gizmo_hover_tooltip(handles, hovered_handle_idx);
        }
    }

    void GameplayState::emit_maneuver_node_debug_overlay(GameStateContext &ctx)
    {
        // World-space debug overlay mirrors the screen-space gizmo state for inspection and tuning.
        if (!_maneuver_nodes_enabled || !_maneuver_nodes_debug_draw)
        {
            return;
        }
        if (!_debug_draw_enabled || !ctx.api)
        {
            return;
        }
        refresh_maneuver_node_runtime_cache(ctx);

        const float ttl_s = std::clamp(ctx.delta_time(), 0.0f, 0.1f) + 0.002f;

        for (auto &node : _maneuver_state.nodes)
        {
            if (!node.gizmo_valid)
            {
                continue;
            }

            const bool selected = node.id == _maneuver_state.selected_node_id;
            const glm::vec4 c_node = selected ? ManeuverColors::kDebugNodeSelected : ManeuverColors::kDebugNode;

            const glm::dvec3 p = glm::dvec3(node.position_world);
            const double base_radius_m = std::clamp(static_cast<double>(node.gizmo_scale_m) * 0.35, 100.0, 50'000.0);
            ctx.api->debug_draw_sphere(p, base_radius_m, c_node, ttl_s, true);

            if (node.total_dv_mps > 0.05)
            {
                const double arrow_len_m = std::clamp(node.total_dv_mps * static_cast<double>(node.gizmo_scale_m) * 0.6,
                                                      static_cast<double>(node.gizmo_scale_m) * 0.5,
                                                      static_cast<double>(node.gizmo_scale_m) * 8.0);
                ctx.api->debug_draw_ray(p, node.burn_direction_world, arrow_len_m, ManeuverColors::kDebugDv, ttl_s, true);
            }

            if (selected || _maneuver_gizmo_style.show_axis_labels)
            {
                const double axis_len_m = static_cast<double>(node.gizmo_scale_m) * (selected ? 2.8 : 1.8);
                ctx.api->debug_draw_ray(p, node.basis_r_world, axis_len_m, ManeuverColors::kDebugRadial, ttl_s, true);
                ctx.api->debug_draw_ray(p, node.basis_t_world, axis_len_m, ManeuverColors::kDebugTangential, ttl_s, true);
                ctx.api->debug_draw_ray(p, node.basis_n_world, axis_len_m, ManeuverColors::kDebugNormal, ttl_s, true);
            }
        }
    }
} // namespace Game
