void GameplayState::draw_maneuver_nodes_panel(GameStateContext &ctx)
{
    // Main editor window for the maneuver plan: creation, selection, timeline editing, and execution controls.
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    if (!viewport)
    {
        return;
    }

    const double now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
    const PredictionTrackState *player_track = player_prediction_track();
    const auto default_node_primary_body_id = [&]() -> orbitsim::BodyId {
        if (_prediction_analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            _prediction_analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            return _prediction_analysis_selection.spec.fixed_body_id;
        }

        if (player_track)
        {
            if (player_track->cache.valid && !player_track->cache.trajectory_inertial.empty())
            {
                return resolve_prediction_analysis_body_id(player_track->cache,
                                                           player_track->key,
                                                           now_s);
            }
        }

        return (_orbitsim && _orbitsim->world_reference_body())
                       ? _orbitsim->world_reference_body()->sim_id
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

    if (!ImGui::Begin("Maneuver Nodes"))
    {
        ImGui::End();
        return;
    }

    if (ImGui::Checkbox("Maneuver Nodes", &_maneuver_nodes_enabled))
    {
        if (!_maneuver_nodes_enabled)
        {
            clear_maneuver_gizmo_instances(ctx);
            _maneuver_gizmo_interaction = {};
        }
        mark_prediction_dirty();
    }

    ImGui::SameLine();
    if (ImGui::Checkbox("Debug", &_maneuver_nodes_debug_draw))
    {
        // no-op
    }

    auto add_maneuver_node_at_time = [&](const double time_s) {
        ManeuverNode n{};
        n.id = _maneuver_state.next_node_id++;
        n.time_s = time_s;
        n.dv_rtn_mps = glm::dvec3(0.0, 0.0, 0.0);
        n.primary_body_id = default_node_primary_body_id();
        n.primary_body_auto = true;
        n.total_dv_mps = 0.0;
        _maneuver_state.nodes.push_back(n);
        _maneuver_state.selected_node_id = n.id;
        _maneuver_state.sort_by_time();
        mark_prediction_dirty();
    };

    ImGui::SameLine();
    if (ImGui::Button("+Node"))
    {
        add_maneuver_node_at_time(now_s + 60.0);
    }

    ImGui::SameLine();
    if (ImGui::Button("Clear") && !_maneuver_state.nodes.empty())
    {
        _maneuver_state.nodes.clear();
        _maneuver_state.selected_node_id = -1;
        _execute_node_armed = false;
        _execute_node_id = -1;
        _maneuver_gizmo_interaction = {};
        clear_maneuver_gizmo_instances(ctx);
        mark_prediction_dirty();
    }

    ImGui::SameLine();
    float window_s = static_cast<float>(_maneuver_timeline_window_s);
    if (ImGui::DragFloat("Window (s)", &window_s, 10.0f, 60.0f, 36'000.0f, "%.0f"))
    {
        _maneuver_timeline_window_s = static_cast<double>(std::max(60.0f, window_s));
    }

    // Gizmo basis toggle: large colored tab-style buttons
    {
        constexpr ManeuverGizmoBasisMode k_basis_modes[]{
            ManeuverGizmoBasisMode::ProgradeOutwardNormal,
            ManeuverGizmoBasisMode::RTN
        };
        constexpr ImVec4 k_active_colors[]{
            ImVec4(0.20f, 0.65f, 0.30f, 1.0f),  // PON green
            ImVec4(0.70f, 0.50f, 0.15f, 1.0f),   // RTN amber
        };
        constexpr ImVec4 k_hovered_colors[]{
            ImVec4(0.25f, 0.55f, 0.30f, 0.70f),
            ImVec4(0.55f, 0.40f, 0.15f, 0.70f),
        };

        const float avail_w = ImGui::GetContentRegionAvail().x;
        const float btn_w = std::max(80.0f, (avail_w - ImGui::GetStyle().ItemSpacing.x) * 0.5f);
        const float btn_h = ImGui::GetFrameHeight() + 4.0f;

        for (int bi = 0; bi < 2; ++bi)
        {
            const bool is_active = (_maneuver_gizmo_basis_mode == k_basis_modes[bi]);
            const char *label = maneuver_gizmo_basis_mode_label(k_basis_modes[bi]);

            if (is_active)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, k_active_colors[bi]);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, k_active_colors[bi]);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, k_active_colors[bi]);
            }
            else
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.20f, 0.22f, 0.60f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, k_hovered_colors[bi]);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, k_active_colors[bi]);
            }

            if (ImGui::Button(label, ImVec2(btn_w, btn_h)))
            {
                _maneuver_gizmo_basis_mode = k_basis_modes[bi];
                _maneuver_gizmo_interaction = {};
            }
            ImGui::PopStyleColor(3);

            if (bi == 0)
            {
                ImGui::SameLine();
            }
        }
    }

    if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
    {
        ImGui::TextUnformatted("Gizmo: DragAxis  (Shift x0.1 / Ctrl x10)");
    }

    if (_maneuver_nodes_debug_draw)
    {
        bool style_changed = false;
        if (ImGui::CollapsingHeader("Gizmo Style"))
        {
            style_changed |= ImGui::ColorEdit4("Icon Color", &_maneuver_gizmo_style.icon_color.x,
                                               ImGuiColorEditFlags_NoInputs);
            style_changed |= ImGui::DragFloat("Overlay Size (px)", &_maneuver_gizmo_style.icon_size_px, 0.25f, 8.0f,
                                              160.0f, "%.1f");
            style_changed |= ImGui::DragFloat("Overlay Scale", &_maneuver_gizmo_style.overlay_scale, 0.01f, 0.25f,
                                              6.0f, "%.2f");
            style_changed |= ImGui::DragFloat("Drag Sensitivity", &_maneuver_gizmo_style.drag_sensitivity_mps_per_m,
                                              0.0001f, 0.00001f, 1.0f, "%.5f");
            style_changed |= ImGui::Checkbox("Show Axis Overlay", &_maneuver_gizmo_style.show_axis_labels);
        }

        if (style_changed)
        {
            _maneuver_gizmo_style.icon_size_px = std::clamp(_maneuver_gizmo_style.icon_size_px, 8.0f, 160.0f);
            _maneuver_gizmo_style.overlay_scale = std::clamp(_maneuver_gizmo_style.overlay_scale, 0.25f, 6.0f);
            _maneuver_gizmo_style.drag_sensitivity_mps_per_m =
                    std::clamp(_maneuver_gizmo_style.drag_sensitivity_mps_per_m, 0.00001f, 1.0f);
        }
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

    const PickingSystem *picking = nullptr;
    const PickingSystem::PickInfo *orbit_pick = nullptr;
    const bool allow_base_pick = _maneuver_state.nodes.empty();
    const bool allow_planned_pick = !_maneuver_state.nodes.empty();
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

        const OrbitPredictionCache *cache = player_prediction_cache();
        if (!cache || !cache->valid)
        {
            return pick.worldPos;
        }

        const bool is_planned = (pick.ownerName == "OrbitPlot/Planned");
        const auto &traj = is_planned
                               ? (cache->trajectory_frame_planned.size() >= 2
                                      ? cache->trajectory_frame_planned
                                      : cache->trajectory_frame)
                               : cache->trajectory_frame;
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

        const double display_time_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
        WorldVec3 pos = (i_hi > 0)
                            ? prediction_sample_hermite_world(*cache, traj[i_hi - 1], traj[i_hi],
                                                              pick.time_s, display_time_s)
                            : prediction_sample_position_world(*cache, traj.front(), display_time_s);

        // Apply the same ship-to-prediction alignment delta used by the gizmo runtime cache.
        const auto &traj_base = cache->trajectory_frame;
        if (traj_base.size() >= 2)
        {
            const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
            const double interp_dt_s =
                    (_last_sim_step_dt_s > 0.0) ? _last_sim_step_dt_s : static_cast<double>(ctx.fixed_delta_time());
            double align_now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
            if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
            {
                align_now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
            }
            align_now_s = std::clamp(align_now_s, traj_base.front().t_s, traj_base.back().t_s);

            auto it_align = std::lower_bound(traj_base.cbegin(), traj_base.cend(), align_now_s,
                                              [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
            const size_t align_hi = static_cast<size_t>(std::distance(traj_base.cbegin(), it_align));
            if (align_hi < traj_base.size())
            {
                WorldVec3 predicted_now = (align_hi > 0)
                                              ? prediction_sample_hermite_world(*cache, traj_base[align_hi - 1],
                                                                                traj_base[align_hi], align_now_s, align_now_s)
                                              : prediction_sample_position_world(*cache, traj_base.front(), align_now_s);

                WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
                glm::dvec3 ship_vel_world(0.0);
                glm::vec3 ship_vel_local_f(0.0f);
                if (get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
                {
                    const EntityId player_eid = player_entity();
                    if (const Entity *player = _world.entities().find(player_eid))
                    {
                        ship_pos_world = player->get_render_position_world(alpha_f);
                    }

                    WorldVec3 align_delta = ship_pos_world - predicted_now;
                    const double align_len = safe_length(glm::dvec3(align_delta));
                    if (!std::isfinite(align_len) || align_len > 10'000.0)
                    {
                        align_delta = WorldVec3(0.0, 0.0, 0.0);
                    }
                    pos += align_delta;
                }
            }
        }

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
        if (!build_maneuver_gizmo_view_context(ctx, overlay_view) ||
            !project_maneuver_gizmo_point(overlay_view, overlay_world, overlay_screen, overlay_depth_m))
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
                                         ? IM_COL32(255, 190, 80, 230)
                                         : IM_COL32(170, 120, 255, 220);
            draw_orbit_pick_overlay(hover_pick, 10.0f, 16.0f, ring_color, IM_COL32(255, 255, 255, 80));
        }
    }

    if (orbit_pick)
    {
        const ImU32 ring_color = (orbit_pick->ownerName == "OrbitPlot/Planned")
                                     ? IM_COL32(255, 190, 80, 255)
                                     : IM_COL32(170, 120, 255, 245);
        draw_orbit_pick_overlay(*orbit_pick, 10.0f, 16.0f, ring_color, IM_COL32(255, 255, 255, 120));

        const double dt_s = orbit_pick->time_s - now_s;
        if (std::isfinite(dt_s))
        {
            ImGui::Text("Orbit pick: %s  T%+.0fs", orbit_pick->ownerName.c_str(), dt_s);
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
    for (auto &node: _maneuver_state.nodes)
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
            const double u_drag = (p1.x > p0.x)
                                      ? (static_cast<double>(mp.x - p0.x) / static_cast<double>(p1.x - p0.x))
                                      : 0.0;
            // Keep dragged nodes inside the visible timeline window and never allow them into the past.
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

    double total_plan_dv_mps = 0.0;
    for (const ManeuverNode &node: _maneuver_state.nodes)
    {
        total_plan_dv_mps += safe_length(node.dv_rtn_mps);
    }

    ImGui::Text("%zu nodes  |  Total DV %.2f m/s", _maneuver_state.nodes.size(), total_plan_dv_mps);
    if (_execute_node_armed && _execute_node_id >= 0)
    {
        ImGui::SameLine();
        ImGui::TextDisabled("Armed: Node %d", _execute_node_id);
    }

    if (_maneuver_state.selected_node_id < 0 && !_maneuver_state.nodes.empty())
    {
        _maneuver_state.selected_node_id = _maneuver_state.nodes.front().id;
    }

    if (_maneuver_state.nodes.empty())
    {
        ImGui::TextUnformatted("No maneuver nodes. Add one with +Node or Add Node @ Pick.");
        if (needs_sort)
        {
            _maneuver_state.sort_by_time();
        }
        ImGui::End();
        return;
    }

    auto find_node_index = [&](const int node_id) -> int {
        for (size_t i = 0; i < _maneuver_state.nodes.size(); ++i)
        {
            if (_maneuver_state.nodes[i].id == node_id)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    };

    const float panel_h = std::max(120.0f, ImGui::GetContentRegionAvail().y);
    const float list_w = std::min(320.0f, ImGui::GetContentRegionAvail().x * 0.42f);
    const int selected_idx = find_node_index(_maneuver_state.selected_node_id);

    ImGui::BeginChild("##mn_node_list", ImVec2(list_w, panel_h), true);
    ImGui::TextUnformatted("Nodes");
    ImGui::Separator();

    for (size_t i = 0; i < _maneuver_state.nodes.size(); ++i)
    {
        ManeuverNode &node = _maneuver_state.nodes[i];
        node.total_dv_mps = safe_length(node.dv_rtn_mps);

        char label[96];
        std::snprintf(label,
                      sizeof(label),
                      "Node %d   T%+.0fs   DV %.1f",
                      node.id,
                      node.time_s - now_s,
                      node.total_dv_mps);

        ImGui::PushID(node.id);
        if (ImGui::Selectable(label, node.id == _maneuver_state.selected_node_id))
        {
            _maneuver_state.selected_node_id = node.id;
        }
        if (_maneuver_gizmo_basis_mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal && node.gizmo_valid)
        {
            const glm::dvec3 dv_w = compose_basis_vector(node.dv_rtn_mps,
                                                          node.maneuver_basis_r_world,
                                                          node.maneuver_basis_t_world,
                                                          node.maneuver_basis_n_world);
            const glm::dvec3 dv_d = project_basis_vector(dv_w,
                                                          node.basis_r_world,
                                                          node.basis_t_world,
                                                          node.basis_n_world);
            ImGui::TextDisabled("PON  %.1f / %.1f / %.1f", dv_d.y, dv_d.x, dv_d.z);
        }
        else
        {
            ImGui::TextDisabled("RTN  %.1f / %.1f / %.1f", node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z);
        }
        if (i + 1 < _maneuver_state.nodes.size())
        {
            ImGui::Separator();
        }
        ImGui::PopID();
    }

    ImGui::EndChild();
    ImGui::SameLine();

    ImGui::BeginChild("##mn_node_detail", ImVec2(0.0f, panel_h), true);

    ManeuverNode *sel = _maneuver_state.find_node(_maneuver_state.selected_node_id);
    if (!sel)
    {
        ImGui::TextUnformatted("Select a node to edit.");
        ImGui::EndChild();
        if (needs_sort)
        {
            _maneuver_state.sort_by_time();
        }
        ImGui::End();
        return;
    }

    ImGui::Text("Selected Node %d", sel->id);
    ImGui::TextDisabled("T%+.1fs  |  DV %.2f m/s", sel->time_s - now_s, safe_length(sel->dv_rtn_mps));

    // Navigation
    ImGui::BeginDisabled(selected_idx <= 0);
    if (ImGui::Button("Prev"))
    {
        _maneuver_state.selected_node_id = _maneuver_state.nodes[static_cast<size_t>(selected_idx - 1)].id;
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::BeginDisabled(selected_idx < 0 || selected_idx + 1 >= static_cast<int>(_maneuver_state.nodes.size()));
    if (ImGui::Button("Next"))
    {
        _maneuver_state.selected_node_id = _maneuver_state.nodes[static_cast<size_t>(selected_idx + 1)].id;
    }
    ImGui::EndDisabled();

    // Actions
    if (ImGui::Button("Warp to Node"))
    {
        _warp_to_time_active = true;
        _warp_to_time_target_s = sel->time_s;
        _warp_to_time_restore_level = 0;
    }

    ImGui::SameLine();
    if (ImGui::Button("Execute"))
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
        const int del_idx = find_node_index(del_id);
        _maneuver_state.nodes.erase(
            std::remove_if(_maneuver_state.nodes.begin(),
                           _maneuver_state.nodes.end(),
                           [&](const ManeuverNode &n) { return n.id == del_id; }),
            _maneuver_state.nodes.end());

        if (_maneuver_state.selected_node_id == del_id)
        {
            if (_maneuver_state.nodes.empty())
            {
                _maneuver_state.selected_node_id = -1;
            }
            else
            {
                const int new_idx = std::clamp(del_idx, 0, static_cast<int>(_maneuver_state.nodes.size()) - 1);
                _maneuver_state.selected_node_id = _maneuver_state.nodes[static_cast<size_t>(new_idx)].id;
            }
        }

        if (_execute_node_armed && _execute_node_id == del_id)
        {
            _execute_node_armed = false;
            _execute_node_id = -1;
        }
        if (_maneuver_gizmo_interaction.node_id == del_id)
        {
            _maneuver_gizmo_interaction = {};
        }

        mark_prediction_dirty();
        ImGui::EndChild();
        if (needs_sort)
        {
            _maneuver_state.sort_by_time();
        }
        ImGui::End();
        return;
    }

    float t_from_now = static_cast<float>(sel->time_s - now_s);
    if (ImGui::DragFloat("T+ (s)", &t_from_now, 1.0f, 0.0f, 36'000.0f, "%.1f"))
    {
        sel->time_s = now_s + static_cast<double>(std::max(0.0f, t_from_now));
        needs_sort = true;
        mark_prediction_dirty();
    }

    float dv[3]{
        static_cast<float>(sel->dv_rtn_mps.x),
        static_cast<float>(sel->dv_rtn_mps.y),
        static_cast<float>(sel->dv_rtn_mps.z)
    };
    // DV is always edited in node-local RTN order so the panel and gizmo write the same underlying data.
    if (ImGui::DragFloat3("DV RTN (m/s)", dv, 0.1f, -50'000.0f, 50'000.0f, "%.2f"))
    {
        sel->dv_rtn_mps = glm::dvec3(dv[0], dv[1], dv[2]);
        sel->total_dv_mps = safe_length(sel->dv_rtn_mps);
        mark_prediction_dirty();
    }

    ImGui::Text("DV total: %.2f m/s", safe_length(sel->dv_rtn_mps));

    if (sel->gizmo_valid && _maneuver_gizmo_basis_mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal)
    {
        const glm::dvec3 dv_world = compose_basis_vector(sel->dv_rtn_mps,
                                                         sel->maneuver_basis_r_world,
                                                         sel->maneuver_basis_t_world,
                                                         sel->maneuver_basis_n_world);
        const glm::dvec3 dv_display = project_basis_vector(dv_world,
                                                           sel->basis_r_world,
                                                           sel->basis_t_world,
                                                           sel->basis_n_world);
        ImGui::Text("Gizmo PON: %.2f / %.2f / %.2f m/s",
                    dv_display.y,
                    dv_display.x,
                    dv_display.z);
    }

    const orbitsim::BodyId analysis_body_id = (player_track && player_track->cache.valid)
                                                  ? resolve_prediction_analysis_body_id(player_track->cache,
                                                                                        player_track->key,
                                                                                        sel->time_s)
                                                  : default_node_primary_body_id();
    const orbitsim::BodyId effective_primary_body_id =
            resolve_maneuver_node_primary_body_id(*sel, sel->time_s);
    if (const CelestialBodyInfo *primary_body = find_celestial_body_info(effective_primary_body_id))
    {
        ImGui::Text("Primary body: %s%s",
                    primary_body->name.c_str(),
                    sel->primary_body_auto ? " (auto)" : "");
    }

    bool auto_primary = sel->primary_body_auto;
    if (ImGui::Checkbox("Auto Primary", &auto_primary))
    {
        sel->primary_body_auto = auto_primary;
        if (sel->primary_body_auto)
        {
            sel->primary_body_id = effective_primary_body_id;
        }
        mark_prediction_dirty();
    }

    if (const CelestialBodyInfo *analysis_body = find_celestial_body_info(analysis_body_id))
    {
        ImGui::Text("Analysis body: %s", analysis_body->name.c_str());
        if (ImGui::Button("Use Analysis Body"))
        {
            sel->primary_body_id = analysis_body_id;
            sel->primary_body_auto = false;
            mark_prediction_dirty();
        }
        ImGui::SameLine();
        if (ImGui::Button("Apply Analysis Body To All"))
        {
            for (ManeuverNode &node : _maneuver_state.nodes)
            {
                node.primary_body_id = analysis_body_id;
                node.primary_body_auto = false;
            }
            mark_prediction_dirty();
        }
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Tip: select from the list, drag the timeline marker, or use the gizmo in-world.");
    ImGui::EndChild();

    if (needs_sort)
    {
        _maneuver_state.sort_by_time();
    }

    ImGui::End();
}
