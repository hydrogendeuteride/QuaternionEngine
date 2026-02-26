#include "gameplay_state.h"

#include "core/game_api.h"

#include "orbitsim/coordinate_frames.hpp"

#include "imgui.h"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <limits>
#include <string>

namespace Game
{
    namespace
    {
        bool finite3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        double safe_length(const glm::dvec3 &v)
        {
            const double len2 = glm::dot(v, v);
            if (!std::isfinite(len2) || len2 <= 0.0)
            {
                return 0.0;
            }
            return std::sqrt(len2);
        }

        glm::dvec3 normalized_or(const glm::dvec3 &v, const glm::dvec3 &fallback)
        {
            const double len = safe_length(v);
            if (!(len > 0.0) || !std::isfinite(len))
            {
                return fallback;
            }
            return v / len;
        }

        double clamp_sane(double x, double lo, double hi, double fallback = 0.0)
        {
            if (!std::isfinite(x))
            {
                return fallback;
            }
            return std::clamp(x, lo, hi);
        }

        WorldVec3 hermite_position_world(const WorldVec3 &ref_body_world,
                                         const orbitsim::TrajectorySample &a,
                                         const orbitsim::TrajectorySample &b,
                                         const double t_s)
        {
            const double ta = a.t_s;
            const double tb = b.t_s;
            const double h = tb - ta;
            if (!std::isfinite(h) || !(h > 0.0))
            {
                return ref_body_world + WorldVec3(a.position_m);
            }

            double u = (t_s - ta) / h;
            if (!std::isfinite(u))
            {
                u = 0.0;
            }
            u = std::clamp(u, 0.0, 1.0);

            const double u2 = u * u;
            const double u3 = u2 * u;

            const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
            const double h10 = u3 - (2.0 * u2) + u;
            const double h01 = (-2.0 * u3) + (3.0 * u2);
            const double h11 = u3 - u2;

            const glm::dvec3 p0 = glm::dvec3(a.position_m);
            const glm::dvec3 p1 = glm::dvec3(b.position_m);
            const glm::dvec3 m0 = glm::dvec3(a.velocity_mps) * h;
            const glm::dvec3 m1 = glm::dvec3(b.velocity_mps) * h;

            const glm::dvec3 p = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
            return ref_body_world + WorldVec3(p);
        }

        struct TrajectorySampledState
        {
            bool valid{false};
            glm::dvec3 r_rel_m{0.0, 0.0, 0.0};
            glm::dvec3 v_rel_mps{0.0, 0.0, 0.0};
            WorldVec3 position_world{0.0, 0.0, 0.0};
        };

        TrajectorySampledState sample_trajectory_state(const std::vector<orbitsim::TrajectorySample> &traj_bci,
                                                       const WorldVec3 &ref_body_world,
                                                       const double t_s)
        {
            TrajectorySampledState out{};
            if (traj_bci.size() < 2)
            {
                return out;
            }

            const double t0 = traj_bci.front().t_s;
            const double t1 = traj_bci.back().t_s;
            if (!(t1 > t0) || !std::isfinite(t_s))
            {
                return out;
            }

            const double t_clamped = std::clamp(t_s, t0, t1);

            auto it_hi = std::lower_bound(traj_bci.cbegin(),
                                          traj_bci.cend(),
                                          t_clamped,
                                          [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
            size_t i_hi = static_cast<size_t>(std::distance(traj_bci.cbegin(), it_hi));
            if (i_hi >= traj_bci.size())
            {
                return out;
            }

            size_t i_lo = (i_hi == 0) ? 0 : (i_hi - 1);
            const orbitsim::TrajectorySample &a = traj_bci[i_lo];
            const orbitsim::TrajectorySample &b = traj_bci[i_hi];

            const double ta = a.t_s;
            const double tb = b.t_s;
            const double h = tb - ta;

            double u = 0.0;
            if (std::isfinite(h) && h > 1e-9)
            {
                u = (t_clamped - ta) / h;
            }
            u = clamp_sane(u, 0.0, 1.0, 0.0);

            // Position: hermite for smoother marker placement.
            out.position_world = hermite_position_world(ref_body_world, a, b, t_clamped);

            // RTN basis: linear interpolation is sufficient.
            out.r_rel_m = glm::mix(glm::dvec3(a.position_m), glm::dvec3(b.position_m), u);
            out.v_rel_mps = glm::mix(glm::dvec3(a.velocity_mps), glm::dvec3(b.velocity_mps), u);
            out.valid = finite3(out.r_rel_m) && finite3(out.v_rel_mps);
            return out;
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

    } // namespace

    void GameplayState::draw_maneuver_nodes_panel(GameStateContext &ctx)
    {
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        if (!viewport)
        {
            return;
        }

        (void) ctx;

        const double now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;

        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
                                       viewport->WorkPos.y + viewport->WorkSize.y - 10.0f),
                                ImGuiCond_Always,
                                ImVec2(0.5f, 1.0f));
        ImGui::SetNextWindowSize(ImVec2(std::max(560.0f, viewport->WorkSize.x * 0.70f), 190.0f), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.45f);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoSavedSettings;

        if (!ImGui::Begin("##ManeuverNodes", nullptr, flags))
        {
            ImGui::End();
            return;
        }

        if (ImGui::Checkbox("Maneuver Nodes", &_maneuver_nodes_enabled))
        {
            mark_prediction_dirty();
        }

        ImGui::SameLine();
        if (ImGui::Checkbox("Debug", &_maneuver_nodes_debug_draw))
        {
            // no-op
        }

        ImGui::SameLine();
        if (ImGui::Button("+Node"))
        {
            ManeuverNode n{};
            n.id = _maneuver_state.next_node_id++;
            n.time_s = now_s + 60.0;
            n.dv_rtn_mps = glm::dvec3(0.0, 0.0, 0.0);
            n.primary_body_id = (_orbitsim && _orbitsim->reference_body())
                                    ? _orbitsim->reference_body()->sim_id
                                    : orbitsim::kInvalidBodyId;
            n.total_dv_mps = 0.0;
            _maneuver_state.nodes.push_back(n);
            _maneuver_state.selected_node_id = n.id;
            _maneuver_state.sort_by_time();
            mark_prediction_dirty();
        }

        ImGui::SameLine();
        if (ImGui::Button("Clear") && !_maneuver_state.nodes.empty())
        {
            _maneuver_state.nodes.clear();
            _maneuver_state.selected_node_id = -1;
            _execute_node_armed = false;
            _execute_node_id = -1;
            mark_prediction_dirty();
        }

        ImGui::SameLine();
        float window_s = static_cast<float>(_maneuver_timeline_window_s);
        if (ImGui::DragFloat("Window (s)", &window_s, 10.0f, 60.0f, 36'000.0f, "%.0f"))
        {
            _maneuver_timeline_window_s = static_cast<double>(std::max(60.0f, window_s));
        }

        if (_warp_to_time_active)
        {
            const double remain_s = _warp_to_time_target_s - now_s;
            ImGui::SameLine();
            ImGui::Text("Warping: T%+.0fs", remain_s);
        }

        const double t_start_s = now_s;
        const double t_end_s = now_s + std::max(60.0, _maneuver_timeline_window_s);
        const double span_s = t_end_s - t_start_s;

        ImGui::Separator();

        // --- Timeline bar ---
        ImDrawList *dl = ImGui::GetWindowDrawList();
        const float bar_h = 28.0f;
        const float bar_w = std::max(200.0f, ImGui::GetContentRegionAvail().x);

        ImGui::InvisibleButton("##mn_timeline", ImVec2(bar_w, bar_h));
        const ImVec2 p0 = ImGui::GetItemRectMin();
        const ImVec2 p1 = ImGui::GetItemRectMax();
        const ImVec2 pc = ImVec2(0.5f * (p0.x + p1.x), 0.5f * (p0.y + p1.y));

        dl->AddRectFilled(p0, p1, IM_COL32(0, 0, 0, 90), 6.0f);
        dl->AddRect(p0, p1, IM_COL32(255, 255, 255, 32), 6.0f);

        // Ticks (simple 0/25/50/75/100%)
        for (int i = 0; i <= 4; ++i)
        {
            const float u = static_cast<float>(i) / 4.0f;
            const float x = p0.x + u * (p1.x - p0.x);
            dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), IM_COL32(255, 255, 255, 18));

            const double t_tick = span_s * static_cast<double>(u);
            char buf[32];
            std::snprintf(buf, sizeof(buf), "+%.0fs", t_tick);
            dl->AddText(ImVec2(x + 3.0f, p0.y - 16.0f), IM_COL32(255, 255, 255, 110), buf);
        }

        const ImVec2 cursor_after_bar = ImGui::GetCursorScreenPos();

        // Node markers overlay
        bool needs_sort = false;
        for (auto &node : _maneuver_state.nodes)
        {
            const double u = (span_s > 0.0) ? (node.time_s - t_start_s) / span_s : 0.0;
            if (!std::isfinite(u))
            {
                continue;
            }

            const float uf = static_cast<float>(std::clamp(u, 0.0, 1.0));
            const float x = p0.x + uf * (p1.x - p0.x);
            const float y = pc.y;

            const bool selected = node.id == _maneuver_state.selected_node_id;
            const float r_px = selected ? 7.0f : 6.0f;
            const float hit = 12.0f;

            ImGui::SetCursorScreenPos(ImVec2(x - hit, y - hit));
            ImGui::PushID(node.id);
            ImGui::InvisibleButton("##mn_node_marker", ImVec2(hit * 2.0f, hit * 2.0f));

            if (ImGui::IsItemClicked(ImGuiMouseButton_Left))
            {
                _maneuver_state.selected_node_id = node.id;
            }

            if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
            {
                const ImVec2 mp = ImGui::GetIO().MousePos;
                const double u_drag = (p1.x > p0.x) ? (static_cast<double>(mp.x - p0.x) / static_cast<double>(p1.x - p0.x)) : 0.0;
                const double u_clamped = std::clamp(u_drag, 0.0, 1.0);
                const double t_new = t_start_s + u_clamped * span_s;
                node.time_s = std::max(now_s, t_new);
                needs_sort = true;
                mark_prediction_dirty();
            }

            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Node %d", node.id);
                ImGui::Text("T%+.1fs", node.time_s - now_s);
                ImGui::Text("DV RTN: (%.1f, %.1f, %.1f) m/s",
                            node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z);
                ImGui::EndTooltip();
            }
            ImGui::PopID();

            const ImU32 col = selected ? IM_COL32(255, 210, 80, 220) : IM_COL32(80, 200, 255, 190);
            draw_diamond(dl, ImVec2(x, y), r_px, col);
            if (selected)
            {
                dl->AddCircle(ImVec2(x, y), 11.0f, IM_COL32(255, 210, 80, 110), 0, 2.0f);
            }
        }

        ImGui::SetCursorScreenPos(cursor_after_bar);

        ImGui::Separator();

        ManeuverNode *sel = _maneuver_state.find_node(_maneuver_state.selected_node_id);
        if (!sel)
        {
            ImGui::TextUnformatted("Select a node to edit.");
            if (needs_sort)
            {
                _maneuver_state.sort_by_time();
            }
            ImGui::End();
            return;
        }

        ImGui::Text("Selected: Node %d", sel->id);

        float t_from_now = static_cast<float>(sel->time_s - now_s);
        if (ImGui::DragFloat("T+ (s)", &t_from_now, 1.0f, 0.0f, 36'000.0f, "%.1f"))
        {
            sel->time_s = now_s + static_cast<double>(std::max(0.0f, t_from_now));
            needs_sort = true;
            mark_prediction_dirty();
        }

        float dv[3]{static_cast<float>(sel->dv_rtn_mps.x),
                    static_cast<float>(sel->dv_rtn_mps.y),
                    static_cast<float>(sel->dv_rtn_mps.z)};
        if (ImGui::DragFloat3("DV RTN (m/s)", dv, 0.1f, -50'000.0f, 50'000.0f, "%.2f"))
        {
            sel->dv_rtn_mps = glm::dvec3(dv[0], dv[1], dv[2]);
            sel->total_dv_mps = safe_length(sel->dv_rtn_mps);
            mark_prediction_dirty();
        }

        sel->total_dv_mps = safe_length(sel->dv_rtn_mps);
        ImGui::Text("DV total: %.2f m/s", sel->total_dv_mps);

        if (ImGui::Button("Warp to Node"))
        {
            _warp_to_time_active = true;
            _warp_to_time_target_s = sel->time_s;
            _warp_to_time_restore_level = 0;
        }

        ImGui::SameLine();
        if (ImGui::Button("Execute (Impulse)"))
        {
            _execute_node_armed = true;
            _execute_node_id = sel->id;

            if (sel->time_s > now_s + 0.01)
            {
                _warp_to_time_active = true;
                _warp_to_time_target_s = sel->time_s;
                _warp_to_time_restore_level = 0;
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("Delete"))
        {
            const int del_id = sel->id;
            _maneuver_state.nodes.erase(
                    std::remove_if(_maneuver_state.nodes.begin(),
                                   _maneuver_state.nodes.end(),
                                   [&](const ManeuverNode &n) { return n.id == del_id; }),
                    _maneuver_state.nodes.end());

            if (_maneuver_state.selected_node_id == del_id)
            {
                _maneuver_state.selected_node_id = _maneuver_state.nodes.empty() ? -1 : _maneuver_state.nodes.front().id;
            }

            if (_execute_node_armed && _execute_node_id == del_id)
            {
                _execute_node_armed = false;
                _execute_node_id = -1;
            }

            mark_prediction_dirty();
        }

        if (needs_sort)
        {
            _maneuver_state.sort_by_time();
        }

        ImGui::End();
    }

    void GameplayState::update_maneuver_nodes_time_warp(GameStateContext &ctx, const float fixed_dt)
    {
        if (!_warp_to_time_active)
        {
            return;
        }

        if (!_orbitsim)
        {
            _warp_to_time_active = false;
            set_time_warp_level(ctx, _warp_to_time_restore_level);
            return;
        }

        const double now_s = _orbitsim->sim.time_s();
        const double remaining_s = _warp_to_time_target_s - now_s;
        if (!std::isfinite(remaining_s) || remaining_s <= 0.0)
        {
            _warp_to_time_active = false;
            set_time_warp_level(ctx, _warp_to_time_restore_level);
            return;
        }

        const double dt_base = static_cast<double>(fixed_dt);
        if (!std::isfinite(dt_base) || !(dt_base > 0.0))
        {
            return;
        }

        // Warp-to-target uses rails warp exclusively to allow clamping dt to avoid overshoot.
        int desired_level = 4; // x50

        for (int level = TimeWarpState::kMaxWarpLevel; level >= 4; --level)
        {
            const double step_s = dt_base * TimeWarpState::kWarpFactors[static_cast<size_t>(level)];
            if (std::isfinite(step_s) && step_s <= remaining_s)
            {
                desired_level = level;
                break;
            }
        }

        if (_time_warp.warp_level != desired_level)
        {
            set_time_warp_level(ctx, desired_level);
        }
    }

    void GameplayState::update_maneuver_nodes_execution(GameStateContext &ctx)
    {
        (void) ctx;

        if (!_execute_node_armed || _execute_node_id < 0)
        {
            return;
        }

        ManeuverNode *node = _maneuver_state.find_node(_execute_node_id);
        if (!node)
        {
            _execute_node_armed = false;
            _execute_node_id = -1;
            return;
        }

        const double now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
        if (!std::isfinite(now_s) || now_s + 1e-4 < node->time_s)
        {
            return;
        }

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (!get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
        {
            return;
        }

        const WorldVec3 ref_body_world = prediction_reference_body_world();
        const glm::dvec3 r_rel_m = glm::dvec3(ship_pos_world - ref_body_world);
        const glm::dvec3 v_rel_mps = ship_vel_world;

        const orbitsim::RtnFrame f = orbitsim::compute_rtn_frame(orbitsim::Vec3{r_rel_m.x, r_rel_m.y, r_rel_m.z},
                                                                 orbitsim::Vec3{v_rel_mps.x, v_rel_mps.y, v_rel_mps.z});

        const glm::dvec3 dv_rtn = node->dv_rtn_mps;
        glm::dvec3 dv_world =
                glm::dvec3(f.R.x, f.R.y, f.R.z) * dv_rtn.x +
                glm::dvec3(f.T.x, f.T.y, f.T.z) * dv_rtn.y +
                glm::dvec3(f.N.x, f.N.y, f.N.z) * dv_rtn.z;

        if (!finite3(dv_world))
        {
            return;
        }

        const bool rails = _rails_warp_active && _time_warp.mode == TimeWarpState::Mode::RailsWarp;
        bool applied = false;

        if (rails && _orbitsim)
        {
            const OrbiterInfo *p = find_player_orbiter();
            if (p && p->rails.active())
            {
                if (orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(p->rails.sc_id))
                {
                    sc->state.velocity_mps += orbitsim::Vec3{dv_world.x, dv_world.y, dv_world.z};
                    applied = true;
                }
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!applied)
        {
            const EntityId player_eid = player_entity();
            if (_physics && _physics_context && player_eid.is_valid())
            {
                const Entity *player = _world.entities().find(player_eid);
                if (player && player->has_physics())
                {
                    const Physics::BodyId body_id{player->physics_body_value()};
                    if (_physics->is_body_valid(body_id))
                    {
                        const glm::vec3 v_local_f = _physics->get_linear_velocity(body_id);
                        const glm::dvec3 v_origin_world = _physics_context->velocity_origin_world();
                        glm::dvec3 v_world = v_origin_world + glm::dvec3(v_local_f);
                        v_world += dv_world;
                        const glm::dvec3 v_local_d = v_world - v_origin_world;

                        _physics->set_linear_velocity(body_id, glm::vec3(v_local_d));
                        _physics->activate(body_id);
                        applied = true;
                    }
                }
            }
        }
#endif

        if (!applied)
        {
            return;
        }

        // Consume the node after execution (impulse mode).
        const int executed_id = node->id;
        _maneuver_state.nodes.erase(
                std::remove_if(_maneuver_state.nodes.begin(),
                               _maneuver_state.nodes.end(),
                               [&](const ManeuverNode &n) { return n.id == executed_id; }),
                _maneuver_state.nodes.end());

        if (_maneuver_state.selected_node_id == executed_id)
        {
            _maneuver_state.selected_node_id = _maneuver_state.nodes.empty() ? -1 : _maneuver_state.nodes.front().id;
        }

        _execute_node_armed = false;
        _execute_node_id = -1;
        mark_prediction_dirty();
    }

    void GameplayState::emit_maneuver_node_debug_overlay(GameStateContext &ctx)
    {
        if (!_maneuver_nodes_enabled || !_maneuver_nodes_debug_draw)
        {
            return;
        }
        if (!_debug_draw_enabled || !ctx.api)
        {
            return;
        }
        if (!_orbitsim || !_prediction_cache.valid || _prediction_cache.trajectory_bci.size() < 2)
        {
            return;
        }

        // Match orbit plot alignment logic so node markers sit on the displayed curve.
        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double interp_dt_s =
                (_last_sim_step_dt_s > 0.0) ? _last_sim_step_dt_s : static_cast<double>(ctx.fixed_delta_time());
        double now_s = _orbitsim->sim.time_s();

        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }

        const double t0 = _prediction_cache.trajectory_bci.front().t_s;
        const double t1 = _prediction_cache.trajectory_bci.back().t_s;
        if (!std::isfinite(now_s) || !(t1 > t0))
        {
            return;
        }
        now_s = std::clamp(now_s, t0, t1);

        WorldVec3 ship_pos_world_state{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (!get_player_world_state(ship_pos_world_state, ship_vel_world, ship_vel_local_f))
        {
            return;
        }

        WorldVec3 ship_pos_world = ship_pos_world_state;
        const EntityId player_eid = player_entity();
        if (const Entity *player = _world.entities().find(player_eid))
        {
            ship_pos_world = player->get_render_position_world(alpha_f);
        }

        const WorldVec3 ref_body_world = prediction_reference_body_world();

        const auto &traj = _prediction_cache.trajectory_bci;
        auto it_hi = std::lower_bound(traj.cbegin(),
                                      traj.cend(),
                                      now_s,
                                      [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
        size_t i_hi = static_cast<size_t>(std::distance(traj.cbegin(), it_hi));
        if (i_hi >= traj.size())
        {
            return;
        }

        WorldVec3 predicted_now_world = ref_body_world + WorldVec3(traj[i_hi].position_m);
        if (i_hi > 0)
        {
            predicted_now_world = hermite_position_world(ref_body_world, traj[i_hi - 1], traj[i_hi], now_s);
        }

        WorldVec3 align_delta = ship_pos_world - predicted_now_world;
        const double align_len = safe_length(glm::dvec3(align_delta));
        if (!std::isfinite(align_len) || align_len > 10'000.0)
        {
            align_delta = WorldVec3(0.0, 0.0, 0.0);
        }

        const float ttl_s = std::clamp(ctx.delta_time(), 0.0f, 0.1f) + 0.002f;

        constexpr glm::vec4 color_node{0.3f, 0.8f, 1.0f, 0.85f};
        constexpr glm::vec4 color_node_selected{1.0f, 0.82f, 0.25f, 0.95f};
        constexpr glm::vec4 color_dv{0.2f, 0.7f, 1.0f, 0.9f};
        constexpr glm::vec4 color_r{1.0f, 0.25f, 0.25f, 0.75f};
        constexpr glm::vec4 color_t{0.25f, 1.0f, 0.25f, 0.75f};
        constexpr glm::vec4 color_n{0.25f, 0.6f, 1.0f, 0.75f};

        const float base_radius_m = 9'000.0f;

        for (auto &node : _maneuver_state.nodes)
        {
            if (!std::isfinite(node.time_s) || node.time_s < t0 || node.time_s > t1)
            {
                continue;
            }

            // Sample a state slightly before the impulse time for the RTN frame.
            double t_basis = node.time_s;
            if (t_basis > t0)
            {
                t_basis = std::max(t0, t_basis - 1e-3);
            }

            const TrajectorySampledState s = sample_trajectory_state(traj, ref_body_world, t_basis);
            if (!s.valid)
            {
                continue;
            }

            const orbitsim::RtnFrame f = orbitsim::compute_rtn_frame(orbitsim::Vec3{s.r_rel_m.x, s.r_rel_m.y, s.r_rel_m.z},
                                                                     orbitsim::Vec3{s.v_rel_mps.x, s.v_rel_mps.y, s.v_rel_mps.z});

            const glm::dvec3 dv_rtn = node.dv_rtn_mps;
            const glm::dvec3 dv_world =
                    glm::dvec3(f.R.x, f.R.y, f.R.z) * dv_rtn.x +
                    glm::dvec3(f.T.x, f.T.y, f.T.z) * dv_rtn.y +
                    glm::dvec3(f.N.x, f.N.y, f.N.z) * dv_rtn.z;

            node.total_dv_mps = safe_length(dv_rtn);
            node.burn_direction_world = normalized_or(dv_world, glm::dvec3(0.0, 1.0, 0.0));
            node.position_world = s.position_world;

            const bool selected = node.id == _maneuver_state.selected_node_id;
            const glm::vec4 c_node = selected ? color_node_selected : color_node;

            const glm::dvec3 p = glm::dvec3(node.position_world + align_delta);
            ctx.api->debug_draw_sphere(p, base_radius_m, c_node, ttl_s, true);

            const double dv_mag = safe_length(dv_world);
            if (dv_mag > 0.05)
            {
                const double arrow_len_m = std::clamp(dv_mag * 100.0, 1'000.0, 100'000.0);
                ctx.api->debug_draw_ray(p, node.burn_direction_world, arrow_len_m, color_dv, ttl_s, true);
            }

            if (selected)
            {
                const double axis_len_m = 30'000.0;
                const glm::dvec3 R = glm::dvec3(f.R.x, f.R.y, f.R.z);
                const glm::dvec3 T = glm::dvec3(f.T.x, f.T.y, f.T.z);
                const glm::dvec3 N = glm::dvec3(f.N.x, f.N.y, f.N.z);

                ctx.api->debug_draw_ray(p, R, axis_len_m, color_r, ttl_s, true);
                ctx.api->debug_draw_ray(p, T, axis_len_m, color_t, ttl_s, true);
                ctx.api->debug_draw_ray(p, N, axis_len_m, color_n, ttl_s, true);
            }
        }
    }
} // namespace Game
