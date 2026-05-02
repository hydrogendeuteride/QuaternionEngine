#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_colors.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_gizmo_helpers.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"
#include "game/states/gameplay/maneuver/maneuver_commands.h"
#include "game/states/gameplay/maneuver/maneuver_prediction_bridge.h"
#include "game/states/gameplay/maneuver/maneuver_ui_controller.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"

#include "core/engine.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

namespace Game
{
    namespace Gizmo = ManeuverGizmoHelpers;

    namespace
    {
        using namespace ManeuverUtil;

        const char *prediction_derived_status_label(const PredictionDerivedStatus status)
        {
            switch (status)
            {
                case PredictionDerivedStatus::Success:
                    return "Ready";
                case PredictionDerivedStatus::MissingSolverData:
                    return "Waiting for solver data";
                case PredictionDerivedStatus::MissingEphemeris:
                    return "Missing ephemeris";
                case PredictionDerivedStatus::FrameTransformFailed:
                    return "Frame transform failed";
                case PredictionDerivedStatus::FrameSamplesUnavailable:
                    return "Frame samples unavailable";
                case PredictionDerivedStatus::ContinuityFailed:
                    return "Continuity failed";
                case PredictionDerivedStatus::Cancelled:
                    return "Cancelled";
                case PredictionDerivedStatus::None:
                default:
                    return "Idle";
            }
        }

        void draw_diamond(ImDrawList *dl, const ImVec2 &p, float r_px, ImU32 col)
        {
            const ImVec2 pts[4]{
                ImVec2(p.x, p.y - r_px),
                ImVec2(p.x + r_px, p.y),
                ImVec2(p.x, p.y + r_px),
                ImVec2(p.x - r_px, p.y),
            };
            dl->AddConvexPolyFilled(pts, 4, col);
        }

        std::string format_t_plus_label(const double delta_s)
        {
            if (!std::isfinite(delta_s))
            {
                return "T+n/a";
            }

            const long long rounded_seconds = static_cast<long long>(std::llround(delta_s));
            const bool negative = rounded_seconds < 0;
            const long long total_seconds = negative ? -rounded_seconds : rounded_seconds;
            const long long days = total_seconds / 86'400;
            const long long hours = (total_seconds / 3'600) % 24;
            const long long minutes = (total_seconds / 60) % 60;
            const long long seconds = total_seconds % 60;

            char buffer[64];
            if (days > 0)
            {
                std::snprintf(buffer,
                              sizeof(buffer),
                              "T%c%lldd %02lld:%02lld:%02lld",
                              negative ? '-' : '+',
                              days,
                              hours,
                              minutes,
                              seconds);
            }
            else
            {
                std::snprintf(buffer,
                              sizeof(buffer),
                              "T%c%02lld:%02lld:%02lld",
                              negative ? '-' : '+',
                              hours,
                              minutes,
                              seconds);
            }
            return buffer;
        }
    } // namespace

    void ManeuverUiController::draw_nodes_panel(GameplayState &state, GameStateContext &ctx)
    {
        auto &_maneuver = state._maneuver;
        auto &_prediction = state._prediction;
        auto &_orbit = state._orbit;
        auto &_show_maneuver_nodes_panel = state._show_maneuver_nodes_panel;
        auto current_sim_time_s = [&]() {
            return state.current_sim_time_s();
        };
        auto clear_gizmo_interaction = [&]() {
            _maneuver.clear_gizmo_interaction();
        };
        auto cancel_edit_preview = [&]() {
            _maneuver.cancel_edit_preview();
        };
        auto apply_maneuver_command = [&](const ManeuverCommand &command) {
            return state.apply_maneuver_command(command);
        };
        auto compute_maneuver_align_delta = [&](GameStateContext &align_ctx,
                                                 const OrbitPredictionCache &cache,
                                                 const std::vector<orbitsim::TrajectorySample> &traj_base) {
            return ManeuverPredictionBridge::compute_align_delta(state, align_ctx, cache, traj_base);
        };
        auto update_maneuver_node_time_edit_preview = [&](const int node_id, const double previous_time_s) {
            state.update_maneuver_node_time_edit_preview(node_id, previous_time_s);
        };
        auto finish_maneuver_node_time_edit_preview = [&](const bool changed) {
            state.finish_maneuver_node_time_edit_preview(changed);
        };
        auto update_maneuver_node_dv_edit_preview = [&](const int node_id) {
            state.update_maneuver_node_dv_edit_preview(node_id);
        };
        auto finish_maneuver_node_dv_edit_preview = [&](const bool changed) {
            state.finish_maneuver_node_dv_edit_preview(changed);
        };
        auto resolve_maneuver_node_primary_body_id = [&](const ManeuverNode &node, const double query_time_s) {
            return ManeuverPredictionBridge::resolve_node_primary_body_id(state, node, query_time_s);
        };
        auto remove_node_suffix = [&](const int node_id, const int hint_index) {
            (void) state.apply_maneuver_command(ManeuverCommand::remove_node_suffix(node_id, hint_index));
        };

        // Main editor window for the maneuver plan: creation, selection, timeline editing, and execution controls.
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        if (!viewport)
        {
            return;
        }

        const double now_s = current_sim_time_s();
        GameplayPredictionAdapter prediction(state);
        const PredictionTrackState *player_track = prediction.player_prediction_track();
        const PredictionSubjectKey time_key = player_track ? player_track->key : PredictionSubjectKey{};
        const PredictionTimeContext time_ctx = prediction.build_prediction_time_context(time_key, now_s);
        const auto to_t_plus_s = [&time_ctx](const double absolute_time_s) {
            return absolute_time_s - time_ctx.sim_now_s;
        };
        const auto from_t_plus_s = [&time_ctx](const double t_plus_s) {
            return time_ctx.sim_now_s + std::max(0.0, t_plus_s);
        };
        const auto format_t_plus_from_time_s = [&to_t_plus_s](const double absolute_time_s) {
            return format_t_plus_label(to_t_plus_s(absolute_time_s));
        };
        const auto default_node_primary_body_id = [&]() -> orbitsim::BodyId {
            if (_prediction->state().analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
                _prediction->state().analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
            {
                return _prediction->state().analysis_selection.spec.fixed_body_id;
            }

            if (const OrbitPredictionCache *player_cache = prediction.effective_prediction_cache(player_track))
            {
                if (!player_cache->solver.resolved_trajectory_inertial().empty())
                {
                    return prediction.resolve_prediction_analysis_body_id(*player_cache,
                                                                          player_track->key,
                                                                          now_s);
                }
            }

            return (_orbit.scenario_owner() && _orbit.scenario_owner()->world_reference_body())
                           ? _orbit.scenario_owner()->world_reference_body()->sim_id
                           : orbitsim::kInvalidBodyId;
        };

        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 28.0f,
                                       viewport->WorkPos.y + viewport->WorkSize.y - 28.0f),
                                ImGuiCond_FirstUseEver,
                                ImVec2(1.0f, 1.0f));
        ImGui::SetNextWindowSize(ImVec2(std::clamp(viewport->WorkSize.x * 0.42f, 420.0f, 560.0f),
                                        std::clamp(viewport->WorkSize.y * 0.42f, 300.0f, 460.0f)),
                                 ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.92f);

        bool panel_open = _show_maneuver_nodes_panel;
        if (!ImGui::Begin("Maneuver Nodes", &panel_open))
        {
            _show_maneuver_nodes_panel = panel_open;
            ImGui::End();
            return;
        }
        _show_maneuver_nodes_panel = panel_open;

        if (ImGui::Checkbox("Maneuver Nodes", &_maneuver.settings().nodes_enabled))
        {
            if (!_maneuver.settings().nodes_enabled)
            {
                clear_gizmo_interaction();
                _maneuver.clear_gizmo_interaction();
                cancel_edit_preview();
                prediction.clear_maneuver_prediction_artifacts();
            }
            (void) apply_maneuver_command(ManeuverCommand::mark_plan_dirty());
        }

        ImGui::SameLine();
        if (ImGui::Checkbox("Debug", &_maneuver.settings().nodes_debug_draw))
        {
            // no-op
        }

        auto add_maneuver_node_at_time = [&](const double time_s) {
            ManeuverNode n{};
            n.id = -1;
            n.time_s = time_s;
            n.dv_rtn_mps = glm::dvec3(0.0, 0.0, 0.0);
            n.primary_body_id = default_node_primary_body_id();
            n.primary_body_auto = true;
            n.total_dv_mps = 0.0;
            (void) apply_maneuver_command(ManeuverCommand::add_node(n));
        };

        ImGui::SameLine();
        if (ImGui::Button("+Node"))
        {
            add_maneuver_node_at_time(from_t_plus_s(60.0));
        }

        ImGui::SameLine();
        if (ImGui::Button("Clear") && !_maneuver.plan().nodes.empty())
        {
            _maneuver.runtime().disarm_execute_node();
            _maneuver.clear_gizmo_interaction();
            cancel_edit_preview();
            clear_gizmo_interaction();
            (void) apply_maneuver_command(ManeuverCommand::clear_plan());
        }

        ImGui::SameLine();
        float window_s = static_cast<float>(_maneuver.settings().timeline_window_s);
        if (ImGui::DragFloat("Timeline Window (s)", &window_s, 10.0f, 60.0f, 15'552'000.0f, "%.0f"))
        {
            _maneuver.settings().timeline_window_s = static_cast<double>(std::max(60.0f, window_s));
        }

        ImGui::SeparatorText("Planner Preview");
        float plan_horizon_s = static_cast<float>(_maneuver.settings().plan_horizon.horizon_s);
        if (ImGui::DragFloat("Plan Horizon (s)", &plan_horizon_s, 10.0f, 0.0f, 15'552'000.0f, "%.0f"))
        {
            _maneuver.settings().plan_horizon.horizon_s = static_cast<double>(std::max(0.0f, plan_horizon_s));
            (void) apply_maneuver_command(ManeuverCommand::mark_plan_dirty());
        }

        if (ImGui::Checkbox("Live Preview", &_maneuver.settings().live_preview_active))
        {
            if (!_maneuver.settings().live_preview_active)
            {
                _prediction->clear_maneuver_live_preview_state();
                cancel_edit_preview();
            }
            (void) apply_maneuver_command(ManeuverCommand::mark_plan_dirty());
        }

        if (player_track)
        {
            const bool has_plan = _maneuver.settings().nodes_enabled && !_maneuver.plan().nodes.empty();
            const uint64_t current_plan_signature = has_plan ? prediction.current_maneuver_plan_signature() : 0u;
            const auto cache_has_ready_current_plan = [&](const OrbitPredictionCache &cache) {
                return has_plan &&
                       cache.identity.valid &&
                       cache.has_planned_frame_draw_data() &&
                       cache.identity.maneuver_plan_signature_valid &&
                       cache.identity.maneuver_plan_signature == current_plan_signature;
            };
            const bool has_ready_plan =
                    cache_has_ready_current_plan(player_track->cache) ||
                    cache_has_ready_current_plan(player_track->authoritative_cache);
            const bool solver_pending =
                    has_plan &&
                    !has_ready_plan &&
                    player_track->request_pending &&
                    player_track->pending_solver_has_maneuver_plan &&
                    player_track->pending_solver_plan_signature == current_plan_signature;
            const bool derived_pending =
                    has_plan &&
                    !has_ready_plan &&
                    player_track->derived_request_pending &&
                    player_track->pending_derived_has_maneuver_plan &&
                    player_track->pending_derived_plan_signature == current_plan_signature;
            const bool update_pending = solver_pending || derived_pending;
            const bool queued_update = has_plan && player_track->dirty && !update_pending && !has_ready_plan;

            const char *status_label = "Idle";
            const char *status_detail = "Add a maneuver node to start a planned solve.";
            ImVec4 status_color = ImVec4(0.70f, 0.72f, 0.76f, 1.0f);

            if (!has_plan)
            {
                status_label = "Idle";
                status_detail = "Add a maneuver node to start a planned solve.";
            }
            else if (queued_update)
            {
                status_label = "Queued";
                status_detail = "Plan preview is marked dirty and will rebuild on the next prediction tick.";
                status_color = ImVec4(0.95f, 0.78f, 0.24f, 1.0f);
            }
            else if (solver_pending)
            {
                status_label = "Solving";
                status_detail = "The solver is still building the planned trajectory.";
                status_color = ImVec4(0.95f, 0.78f, 0.24f, 1.0f);
            }
            else if (derived_pending)
            {
                status_label = "Preparing";
                status_detail = "Solver output is being converted into frame-space render data.";
                status_color = ImVec4(0.98f, 0.66f, 0.26f, 1.0f);
            }
            else if (has_ready_plan)
            {
                status_label = "Ready";
                status_detail = "Planned trajectory solve is complete.";
                status_color = ImVec4(0.33f, 0.86f, 0.46f, 1.0f);
            }
            else
            {
                status_label = prediction_derived_status_label(player_track->derived_diagnostics.status);
                status_detail = "The last planned solve did not publish final draw data yet.";
                status_color = ImVec4(0.95f, 0.48f, 0.34f, 1.0f);
            }

            ImGui::TextColored(status_color, "Planner Status: %s", status_label);
            ImGui::TextWrapped("%s", status_detail);

            if (has_plan)
            {
                float progress = has_ready_plan ? 1.0f : 0.0f;
                const bool plan_progress_active = update_pending;
                if (plan_progress_active)
                {
                    const double pulse = 0.5 + 0.5 * std::sin(ImGui::GetTime() * 3.0);
                    progress = static_cast<float>(0.20 + 0.60 * pulse);
                }

                char progress_label[128];
                if (plan_progress_active)
                {
                    std::snprintf(progress_label,
                                  sizeof(progress_label),
                                  "Calculating plan...");
                }
                else if (queued_update)
                {
                    std::snprintf(progress_label, sizeof(progress_label), "Preview queued");
                }
                else if (has_ready_plan)
                {
                    std::snprintf(progress_label, sizeof(progress_label), "Preview ready");
                }
                else
                {
                    std::snprintf(progress_label, sizeof(progress_label), "Preview unavailable");
                }
                ImGui::ProgressBar(progress, ImVec2(ImGui::GetContentRegionAvail().x, 0.0f), progress_label);

                ImGui::Text("Solver: %.1f ms", player_track->solver_ms_last);
            }
            else
            {
                ImGui::TextDisabled("Planner preview is idle until at least one node exists.");
            }
        }
        else
        {
            ImGui::TextDisabled("Planner status unavailable without a player prediction track.");
        }

        // Gizmo basis toggle: large colored tab-style buttons.
        {
            const float avail_w = ImGui::GetContentRegionAvail().x;
            const float btn_w = std::max(80.0f, (avail_w - ImGui::GetStyle().ItemSpacing.x) * 0.5f);
            const float btn_h = ImGui::GetFrameHeight() + 4.0f;

            for (int bi = 0; bi < 2; ++bi)
            {
                const bool is_active = (_maneuver.settings().gizmo_basis_mode == ManeuverColors::kBasisModes[bi]);
                const char *label = ManeuverUtil::basis_mode_label(ManeuverColors::kBasisModes[bi]);

                if (is_active)
                {
                    ImGui::PushStyleColor(ImGuiCol_Button, ManeuverColors::kBasisButtonActive[bi]);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ManeuverColors::kBasisButtonActive[bi]);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ManeuverColors::kBasisButtonActive[bi]);
                }
                else
                {
                    ImGui::PushStyleColor(ImGuiCol_Button, ManeuverColors::kBasisButtonInactive);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ManeuverColors::kBasisButtonHovered[bi]);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ManeuverColors::kBasisButtonActive[bi]);
                }

                if (ImGui::Button(label, ImVec2(btn_w, btn_h)))
                {
                    _maneuver.settings().gizmo_basis_mode = ManeuverColors::kBasisModes[bi];
                    _maneuver.clear_gizmo_interaction();
                }
                ImGui::PopStyleColor(3);

                if (bi == 0)
                {
                    ImGui::SameLine();
                }
            }
        }

        if (_maneuver.gizmo_interaction().state == ManeuverGizmoInteraction::State::DragAxis)
        {
            ImGui::TextUnformatted("Gizmo: DragAxis  (Shift x0.1 / Ctrl x10)");
        }

        if (_maneuver.settings().nodes_debug_draw)
        {
            bool style_changed = false;
            if (ImGui::CollapsingHeader("Gizmo Style"))
            {
                style_changed |= ImGui::ColorEdit4("Icon Color", &_maneuver.settings().gizmo_style.icon_color.x,
                                                   ImGuiColorEditFlags_NoInputs);
                style_changed |= ImGui::DragFloat("Overlay Size (px)", &_maneuver.settings().gizmo_style.icon_size_px, 0.25f, 8.0f,
                                                  160.0f, "%.1f");
                style_changed |= ImGui::DragFloat("Overlay Scale", &_maneuver.settings().gizmo_style.overlay_scale, 0.01f, 0.25f,
                                                  6.0f, "%.2f");
                style_changed |= ImGui::DragFloat("Drag Sensitivity", &_maneuver.settings().gizmo_style.drag_sensitivity_mps_per_m,
                                                  0.0001f, 0.00001f, 1.0f, "%.5f");
                style_changed |= ImGui::Checkbox("Show Axis Overlay", &_maneuver.settings().gizmo_style.show_axis_labels);
            }

            if (style_changed)
            {
                _maneuver.settings().gizmo_style.icon_size_px = std::clamp(_maneuver.settings().gizmo_style.icon_size_px, 8.0f, 160.0f);
                _maneuver.settings().gizmo_style.overlay_scale = std::clamp(_maneuver.settings().gizmo_style.overlay_scale, 0.25f, 6.0f);
                _maneuver.settings().gizmo_style.drag_sensitivity_mps_per_m =
                        std::clamp(_maneuver.settings().gizmo_style.drag_sensitivity_mps_per_m, 0.00001f, 1.0f);
            }
        }

        if (_maneuver.runtime().warp_to_time_active)
        {
            const std::string warp_label = format_t_plus_from_time_s(_maneuver.runtime().warp_to_time_target_s);
            ImGui::SameLine();
            ImGui::Text("Warping: %s", warp_label.c_str());
        }

        const double t_start_s = time_ctx.sim_now_s;
        const double t_end_s = time_ctx.sim_now_s + std::max(60.0, _maneuver.settings().timeline_window_s);
        const double span_s = t_end_s - t_start_s;

        const PickingSystem *picking = nullptr;
        const PickingSystem::PickInfo *orbit_pick = nullptr;
        const bool allow_base_pick = _maneuver.plan().nodes.empty();
        const bool allow_planned_pick = !_maneuver.plan().nodes.empty();
        // First node is placed on the base orbit; once a plan exists, additional nodes snap to the planned orbit instead.
        auto pick_allowed_for_node_add = [&](const PickingSystem::PickInfo &pick) {
            if (!pick.valid || pick.kind != PickingSystem::PickInfo::Kind::Line)
            {
                return false;
            }
            if (allow_base_pick && pick.ownerName == "OrbitPlot/Base")
            {
                return true;
            }
            if (allow_planned_pick && pick.ownerName == "OrbitPlot/Planned")
            {
                return true;
            }
            return false;
        };
        if (ctx.renderer)
        {
            picking = ctx.renderer->picking();
            if (picking)
            {
                const auto &pick = picking->last_pick();
                if (pick.ownerName.rfind("OrbitPlot/", 0) == 0 &&
                    pick_allowed_for_node_add(pick))
                {
                    orbit_pick = &pick;
                }
            }
        }

        // Resample a pick position from the current prediction trajectory so the overlay
        // stays consistent when the reference frame or plan changes after the pick was recorded.
        auto resample_pick_world_pos = [&](const PickingSystem::PickInfo &pick) -> WorldVec3 {
            if (!std::isfinite(pick.time_s))
            {
                return pick.worldPos;
            }

            const OrbitPredictionCache *cache = prediction.player_prediction_cache();
            if (!cache || !cache->identity.valid)
            {
                return pick.worldPos;
            }
            const OrbitPredictionCache *stable_cache =
                    (player_track && player_track->cache.identity.valid) ? &player_track->cache : cache;

            const bool is_planned = (pick.ownerName == "OrbitPlot/Planned");
            const double display_time_s = current_sim_time_s();
            const auto &traj = is_planned
                                   ? (cache->display.trajectory_frame_planned.size() >= 2
                                              ? cache->display.trajectory_frame_planned
                                              : (stable_cache && stable_cache->display.trajectory_frame_planned.size() >= 2
                                                         ? stable_cache->display.trajectory_frame_planned
                                                         : cache->display.trajectory_frame))
                                   : cache->display.trajectory_frame;
            if (traj.size() < 2)
            {
                return pick.worldPos;
            }

            const double traj_t0 = traj.front().t_s;
            const double traj_t1 = traj.back().t_s;
            if (pick.time_s < traj_t0 || pick.time_s > traj_t1)
            {
                return pick.worldPos;
            }

            auto it_hi = std::lower_bound(traj.cbegin(), traj.cend(), pick.time_s,
                                          [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
            const size_t i_hi = static_cast<size_t>(std::distance(traj.cbegin(), it_hi));
            if (i_hi >= traj.size())
            {
                return pick.worldPos;
            }

            WorldVec3 pos = (i_hi > 0)
                                ? prediction.prediction_sample_hermite_world(*cache, traj[i_hi - 1], traj[i_hi],
                                                                             pick.time_s, display_time_s)
                                : prediction.prediction_sample_position_world(*cache, traj.front(), display_time_s);

            // Apply the same ship-to-prediction alignment delta used by the gizmo runtime cache.
            pos += compute_maneuver_align_delta(ctx, *cache, cache->display.trajectory_frame);

            return pos;
        };

        auto draw_orbit_pick_overlay = [&](const PickingSystem::PickInfo &pick,
                                           const float inner_radius_px,
                                           const float outer_radius_px,
                                           const ImU32 ring_color,
                                           const ImU32 outer_color) {
            // Resample position from current trajectory so overlay stays correct across frame/plan changes.
            const WorldVec3 overlay_world = resample_pick_world_pos(pick);

            ManeuverGizmoViewContext overlay_view{};
            glm::vec2 overlay_screen{0.0f, 0.0f};
            double overlay_depth_m = 0.0;
            if (!build_gizmo_view_context(state, ctx, overlay_view) ||
                !Gizmo::project_maneuver_gizmo_point(overlay_view, overlay_world, overlay_screen, overlay_depth_m))
            {
                return;
            }

            if (ImDrawList *overlay_dl = ImGui::GetForegroundDrawList())
            {
                const ImVec2 overlay_pos{overlay_screen.x, overlay_screen.y};
                overlay_dl->AddCircle(overlay_pos, inner_radius_px, ring_color, 24, 2.0f);
                overlay_dl->AddCircle(overlay_pos, outer_radius_px, outer_color, 24, 1.0f);
            }
        };

        if (picking)
        {
            const PickingSystem::PickInfo &hover_pick = picking->hover_pick();
            if (hover_pick.ownerName.rfind("OrbitPlot/", 0) == 0 &&
                pick_allowed_for_node_add(hover_pick))
            {
                const ImU32 ring_color = (hover_pick.ownerName == "OrbitPlot/Planned")
                                             ? ManeuverColors::kOrbitPickPlannedHover
                                             : ManeuverColors::kOrbitPickBaseHover;
                draw_orbit_pick_overlay(hover_pick,
                                        10.0f,
                                        16.0f,
                                        ring_color,
                                        ManeuverColors::kOrbitPickOuterHover);
            }
        }

        if (orbit_pick)
        {
            const ImU32 ring_color = (orbit_pick->ownerName == "OrbitPlot/Planned")
                                         ? ManeuverColors::kOrbitPickPlannedSelected
                                         : ManeuverColors::kOrbitPickBaseSelected;
            draw_orbit_pick_overlay(*orbit_pick,
                                    10.0f,
                                    16.0f,
                                    ring_color,
                                    ManeuverColors::kOrbitPickOuterSelected);

            const double dt_s = to_t_plus_s(orbit_pick->time_s);
            if (std::isfinite(dt_s))
            {
                const std::string pick_label = format_t_plus_label(dt_s);
                ImGui::Text("Orbit pick: %s  %s", orbit_pick->ownerName.c_str(), pick_label.c_str());
            }
            else
            {
                ImGui::Text("Orbit pick: %s", orbit_pick->ownerName.c_str());
            }

            const bool can_add = std::isfinite(orbit_pick->time_s);
            ImGui::BeginDisabled(!can_add);
            if (ImGui::Button("Add Node @ Pick"))
            {
                add_maneuver_node_at_time(orbit_pick->time_s);
            }
            ImGui::EndDisabled();
        }

        ImGui::Separator();

        // --- Timeline bar ---
        // This is both a readout and a direct-manipulation editor: marker drag writes back to node.time_s.
        ImDrawList *dl = ImGui::GetWindowDrawList();
        const float bar_h = _maneuver.settings().ui_config.scaled(28.0f);
        const float bar_w = std::max(200.0f, ImGui::GetContentRegionAvail().x);

        ImGui::InvisibleButton("##mn_timeline", ImVec2(bar_w, bar_h));
        const ImVec2 p0 = ImGui::GetItemRectMin();
        const ImVec2 p1 = ImGui::GetItemRectMax();
        const ImVec2 pc = ImVec2(0.5f * (p0.x + p1.x), 0.5f * (p0.y + p1.y));

        dl->AddRectFilled(p0, p1, ManeuverColors::kTimelineBackground, 6.0f);
        dl->AddRect(p0, p1, ManeuverColors::kTimelineBorder, 6.0f);

        // Ticks (simple 0/25/50/75/100%)
        for (int i = 0; i <= 4; ++i)
        {
            const float u = static_cast<float>(i) / 4.0f;
            const float x = p0.x + u * (p1.x - p0.x);
            dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), ManeuverColors::kTimelineTick);

            const double t_tick = span_s * static_cast<double>(u);
            const std::string tick_label = format_t_plus_label(t_tick);
            dl->AddText(ImVec2(x + 3.0f, p0.y - 16.0f), ManeuverColors::kTimelineTickText, tick_label.c_str());
        }

        const ImVec2 cursor_after_bar = ImGui::GetCursorScreenPos();

        // Node markers overlay
        bool needs_sort = false;
        for (auto &node : _maneuver.plan().nodes)
        {
            const double u = (span_s > 0.0) ? (node.time_s - t_start_s) / span_s : 0.0;
            if (!std::isfinite(u))
            {
                continue;
            }

            const float uf = static_cast<float>(std::clamp(u, 0.0, 1.0));
            const float x = p0.x + uf * (p1.x - p0.x);
            const float y = pc.y;

            const bool selected = node.id == _maneuver.plan().selected_node_id;
            const float r_px = _maneuver.settings().ui_config.scaled(selected ? 7.0f : 6.0f);
            const float hit = _maneuver.settings().ui_config.scaled(12.0f);

            ImGui::SetCursorScreenPos(ImVec2(x - hit, y - hit));
            ImGui::PushID(node.id);
            ImGui::InvisibleButton("##mn_node_marker", ImVec2(hit * 2.0f, hit * 2.0f));

            if (ImGui::IsItemClicked(ImGuiMouseButton_Left))
            {
                (void) apply_maneuver_command(ManeuverCommand::select_node(node.id));
            }

            if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
            {
                const ImVec2 mp = ImGui::GetIO().MousePos;
                const double u_drag = (p1.x > p0.x)
                                          ? (static_cast<double>(mp.x - p0.x) / static_cast<double>(p1.x - p0.x))
                                          : 0.0;
                // Keep dragged nodes inside the visible timeline window and never allow them into the past.
                const double u_clamped = std::clamp(u_drag, 0.0, 1.0);
                const double t_new = t_start_s + u_clamped * span_s;
                const double previous_time_s = node.time_s;
                const double new_time_s = std::max(time_ctx.sim_now_s, t_new);
                if (std::isfinite(new_time_s) && std::abs(new_time_s - previous_time_s) > 1.0e-9)
                {
                    (void) apply_maneuver_command(ManeuverCommand::set_node_time(node.id, new_time_s, true));
                    needs_sort = true;
                    update_maneuver_node_time_edit_preview(node.id, previous_time_s);
                }
            }
            if (ImGui::IsItemDeactivated())
            {
                finish_maneuver_node_time_edit_preview(false);
            }

            if (ImGui::IsItemHovered())
            {
                const std::string t_plus_label = format_t_plus_from_time_s(node.time_s);
                ImGui::BeginTooltip();
                ImGui::Text("Node %d", node.id);
                ImGui::TextUnformatted(t_plus_label.c_str());
                ImGui::Text("DV RTN: (%.1f, %.1f, %.1f) m/s",
                            node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z);
                ImGui::EndTooltip();
            }
            ImGui::PopID();

            const ImU32 col = selected ? ManeuverColors::kTimelineNodeSelected : ManeuverColors::kTimelineNodeUnselected;
            draw_diamond(dl, ImVec2(x, y), r_px, col);
            if (selected)
            {
                dl->AddCircle(ImVec2(x, y), 11.0f, ManeuverColors::kTimelineNodeSelectedRing, 0, 2.0f);
            }
        }

        ImGui::SetCursorScreenPos(cursor_after_bar);

        ImGui::Separator();

        double total_plan_dv_mps = 0.0;
        for (const ManeuverNode &node : _maneuver.plan().nodes)
        {
            total_plan_dv_mps += safe_length(node.dv_rtn_mps);
        }

        ImGui::Text("%zu nodes  |  Total DV %.2f m/s", _maneuver.plan().nodes.size(), total_plan_dv_mps);
        if (_maneuver.runtime().execute_node_armed && _maneuver.runtime().execute_node_id >= 0)
        {
            ImGui::SameLine();
            ImGui::TextDisabled("Armed: Node %d", _maneuver.runtime().execute_node_id);
        }

        if (_maneuver.plan().selected_node_id < 0 && !_maneuver.plan().nodes.empty())
        {
            (void) apply_maneuver_command(ManeuverCommand::ensure_selection());
        }

        if (_maneuver.plan().nodes.empty())
        {
            ImGui::TextUnformatted("No maneuver nodes. Add one with +Node or Add Node @ Pick.");
            if (needs_sort)
            {
                (void) apply_maneuver_command(ManeuverCommand::sort_by_time());
            }
            ImGui::End();
            return;
        }

        auto find_node_index = [&](const int node_id) -> int {
            for (size_t i = 0; i < _maneuver.plan().nodes.size(); ++i)
            {
                if (_maneuver.plan().nodes[i].id == node_id)
                {
                    return static_cast<int>(i);
                }
            }
            return -1;
        };

        const float panel_h = std::max(120.0f, ImGui::GetContentRegionAvail().y);
        const float list_w = std::min(320.0f, ImGui::GetContentRegionAvail().x * 0.42f);
        const int selected_idx = find_node_index(_maneuver.plan().selected_node_id);

        ImGui::BeginChild("##mn_node_list", ImVec2(list_w, panel_h), true);
        ImGui::TextUnformatted("Nodes");
        ImGui::Separator();

        for (size_t i = 0; i < _maneuver.plan().nodes.size(); ++i)
        {
            ManeuverNode &node = _maneuver.plan().nodes[i];
            node.total_dv_mps = safe_length(node.dv_rtn_mps);

            char label[96];
            const std::string t_plus_label = format_t_plus_from_time_s(node.time_s);
            std::snprintf(label,
                          sizeof(label),
                          "Node %d   %s   DV %.1f",
                          node.id,
                          t_plus_label.c_str(),
                          node.total_dv_mps);

            ImGui::PushID(node.id);
            if (ImGui::Selectable(label, node.id == _maneuver.plan().selected_node_id))
            {
                (void) apply_maneuver_command(ManeuverCommand::select_node(node.id));
            }
            if (_maneuver.settings().gizmo_basis_mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal && node.gizmo_valid)
            {
                const glm::dvec3 dv_d = ManeuverUtil::dv_rtn_to_display_basis(node);
                ImGui::TextDisabled("PON  %.1f / %.1f / %.1f", dv_d.y, dv_d.x, dv_d.z);
            }
            else
            {
                ImGui::TextDisabled("RTN  %.1f / %.1f / %.1f", node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z);
            }
            if (i + 1 < _maneuver.plan().nodes.size())
            {
                ImGui::Separator();
            }
            ImGui::PopID();
        }

        ImGui::EndChild();
        ImGui::SameLine();

        ImGui::BeginChild("##mn_node_detail", ImVec2(0.0f, panel_h), true);

        ManeuverNode *sel = _maneuver.plan().find_node(_maneuver.plan().selected_node_id);
        if (!sel)
        {
            ImGui::TextUnformatted("Select a node to edit.");
            ImGui::EndChild();
            if (needs_sort)
            {
                (void) apply_maneuver_command(ManeuverCommand::sort_by_time());
            }
            ImGui::End();
            return;
        }

        ImGui::Text("Selected Node %d", sel->id);
        const std::string selected_t_plus_label = format_t_plus_from_time_s(sel->time_s);
        ImGui::TextDisabled("%s  |  DV %.2f m/s", selected_t_plus_label.c_str(), safe_length(sel->dv_rtn_mps));

        // Navigation
        ImGui::BeginDisabled(selected_idx <= 0);
        if (ImGui::Button("Prev"))
        {
            (void) apply_maneuver_command(ManeuverCommand::select_node_by_index(selected_idx - 1));
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::BeginDisabled(selected_idx < 0 || selected_idx + 1 >= static_cast<int>(_maneuver.plan().nodes.size()));
        if (ImGui::Button("Next"))
        {
            (void) apply_maneuver_command(ManeuverCommand::select_node_by_index(selected_idx + 1));
        }
        ImGui::EndDisabled();

        // Actions
        if (ImGui::Button("Warp to Node"))
        {
            _maneuver.runtime().warp_to_time_active = true;
            _maneuver.runtime().warp_to_time_target_s = sel->time_s;
            _maneuver.runtime().warp_to_time_restore_level = 0;
        }

        ImGui::SameLine();
        if (ImGui::Button("Execute"))
        {
            _maneuver.runtime().execute_node_armed = true;
            _maneuver.runtime().execute_node_id = sel->id;

            if (sel->time_s > time_ctx.sim_now_s + 0.01)
            {
                _maneuver.runtime().warp_to_time_active = true;
                _maneuver.runtime().warp_to_time_target_s = sel->time_s;
                _maneuver.runtime().warp_to_time_restore_level = 0;
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("Delete"))
        {
            const int del_id = sel->id;
            if (needs_sort)
            {
                (void) apply_maneuver_command(ManeuverCommand::sort_by_time());
                needs_sort = false;
            }
            const int del_idx = find_node_index(del_id);
            remove_node_suffix(del_id, del_idx);
            ImGui::EndChild();
            ImGui::End();
            return;
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Deletes the selected node and all later nodes.");
        }

        ImGui::TextDisabled("%s from now", selected_t_plus_label.c_str());

        double node_time_input = sel->time_s;
        const double previous_node_time_s = sel->time_s;
        const double min_node_time_s = time_ctx.sim_now_s;
        if (ImGui::DragScalar("Sim Time (s)",
                              ImGuiDataType_Double,
                              &node_time_input,
                              1.0f,
                              &min_node_time_s,
                              nullptr,
                              "%.3f"))
        {
            const double new_time_s = std::max(min_node_time_s, node_time_input);
            if (std::isfinite(new_time_s) && std::abs(new_time_s - previous_node_time_s) > 1.0e-9)
            {
                (void) apply_maneuver_command(ManeuverCommand::set_node_time(sel->id, new_time_s, true));
                needs_sort = true;
                update_maneuver_node_time_edit_preview(sel->id, previous_node_time_s);
            }
        }
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            finish_maneuver_node_time_edit_preview(true);
        }

        float dv[3]{
            static_cast<float>(sel->dv_rtn_mps.x),
            static_cast<float>(sel->dv_rtn_mps.y),
            static_cast<float>(sel->dv_rtn_mps.z),
        };
        // DV is always edited in node-local RTN order so the panel and gizmo write the same underlying data.
        if (ImGui::DragFloat3("DV RTN (m/s)", dv, 0.1f, -50'000.0f, 50'000.0f, "%.2f"))
        {
            (void) apply_maneuver_command(ManeuverCommand::set_node_dv(sel->id, glm::dvec3(dv[0], dv[1], dv[2]), true));
            sel->total_dv_mps = safe_length(sel->dv_rtn_mps);
            update_maneuver_node_dv_edit_preview(sel->id);
        }
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            finish_maneuver_node_dv_edit_preview(true);
        }

        ImGui::Text("DV total: %.2f m/s", safe_length(sel->dv_rtn_mps));

        if (sel->gizmo_valid && _maneuver.settings().gizmo_basis_mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal)
        {
            const glm::dvec3 dv_display = ManeuverUtil::dv_rtn_to_display_basis(*sel);
            ImGui::Text("Gizmo PON: %.2f / %.2f / %.2f m/s",
                        dv_display.y,
                        dv_display.x,
                        dv_display.z);
        }

        const OrbitPredictionCache *player_cache = prediction.effective_prediction_cache(player_track);
        const orbitsim::BodyId analysis_body_id = player_cache
                                                      ? prediction.resolve_prediction_analysis_body_id(*player_cache,
                                                                                                      player_track->key,
                                                                                                      sel->time_s)
                                                      : default_node_primary_body_id();
        const orbitsim::BodyId effective_primary_body_id =
                resolve_maneuver_node_primary_body_id(*sel, sel->time_s);
        if (const CelestialBodyInfo *primary_body = prediction.find_celestial_body_info(effective_primary_body_id))
        {
            ImGui::Text("Primary body: %s%s",
                        primary_body->name.c_str(),
                        sel->primary_body_auto ? " (auto)" : "");
        }

        bool auto_primary = sel->primary_body_auto;
        if (ImGui::Checkbox("Auto Primary", &auto_primary))
        {
            const orbitsim::BodyId primary_body_id = auto_primary ? effective_primary_body_id : sel->primary_body_id;
            (void) apply_maneuver_command(
                    ManeuverCommand::set_node_primary_body(sel->id, auto_primary, primary_body_id));
        }

        if (const CelestialBodyInfo *analysis_body = prediction.find_celestial_body_info(analysis_body_id))
        {
            ImGui::Text("Analysis body: %s", analysis_body->name.c_str());
            if (ImGui::Button("Use Analysis Body"))
            {
                (void) apply_maneuver_command(
                        ManeuverCommand::set_node_primary_body(sel->id, false, analysis_body_id));
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Analysis Body To All"))
            {
                (void) apply_maneuver_command(
                        ManeuverCommand::apply_primary_body_to_all(analysis_body_id, false));
            }
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Tip: select from the list, drag the timeline marker, or use the gizmo in-world.");
        ImGui::EndChild();

        if (needs_sort)
        {
            (void) apply_maneuver_command(ManeuverCommand::sort_by_time());
        }

        ImGui::End();
    }
} // namespace Game
