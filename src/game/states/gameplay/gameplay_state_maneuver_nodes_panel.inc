    void GameplayState::draw_maneuver_nodes_panel(GameStateContext &ctx)
    {
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        if (!viewport)
        {
            return;
        }

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

        bool style_changed = false;
        if (ImGui::CollapsingHeader("Gizmo Style", ImGuiTreeNodeFlags_DefaultOpen))
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

        if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            ImGui::TextUnformatted("Gizmo: DragAxis  (Shift x0.1 / Ctrl x10)");
        }
        else if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::HoverAxis)
        {
            ImGui::TextUnformatted("Gizmo: HoverAxis");
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

        const PickingSystem::PickInfo *orbit_pick = nullptr;
        if (ctx.renderer)
        {
            if (const PickingSystem *picking = ctx.renderer->picking())
            {
                const auto &pick = picking->last_pick();
                if (pick.valid &&
                    pick.kind == PickingSystem::PickInfo::Kind::Line &&
                    pick.ownerName.rfind("OrbitPlot/", 0) == 0)
                {
                    orbit_pick = &pick;
                }
            }
        }

        if (orbit_pick)
        {
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
                ManeuverNode n{};
                n.id = _maneuver_state.next_node_id++;
                n.time_s = orbit_pick->time_s;
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
            ImGui::EndDisabled();
        }

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
            if (_maneuver_gizmo_interaction.node_id == del_id)
            {
                _maneuver_gizmo_interaction = {};
            }

            mark_prediction_dirty();
        }

        if (needs_sort)
        {
            _maneuver_state.sort_by_time();
        }

        ImGui::End();
    }
