    void GameplayState::clear_maneuver_gizmo_instances(GameStateContext &ctx)
    {
        (void) ctx;
        _maneuver_gizmo_interaction = {};
    }

    void GameplayState::draw_maneuver_imgui_gizmo(GameStateContext &ctx)
    {
        if (!_maneuver_nodes_enabled || _maneuver_state.nodes.empty())
        {
            return;
        }
        if (!ctx.input || !ctx.renderer || !ctx.renderer->_sceneManager || !ctx.renderer->_swapchainManager ||
            !ctx.renderer->_window)
        {
            return;
        }

        refresh_maneuver_node_runtime_cache(ctx);

        int win_w = 0;
        int win_h = 0;
        int draw_w = 0;
        int draw_h = 0;
        SDL_GetWindowSize(ctx.renderer->_window, &win_w, &win_h);
        SDL_Vulkan_GetDrawableSize(ctx.renderer->_window, &draw_w, &draw_h);
        if (win_w <= 0 || win_h <= 0 || draw_w <= 0 || draw_h <= 0)
        {
            return;
        }

        const VkExtent2D swap_extent = ctx.renderer->_swapchainManager->swapchainExtent();
        if (swap_extent.width == 0 || swap_extent.height == 0)
        {
            return;
        }

        VkExtent2D logical_extent = ctx.renderer->_logicalRenderExtent;
        if (logical_extent.width == 0 || logical_extent.height == 0)
        {
            logical_extent = VkExtent2D{1280, 720};
        }
        if (logical_extent.width == 0 || logical_extent.height == 0)
        {
            return;
        }

        const VkRect2D letterbox_rect = vkutil::compute_letterbox_rect(logical_extent, swap_extent);
        if (letterbox_rect.extent.width == 0 || letterbox_rect.extent.height == 0)
        {
            return;
        }

        const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
        const glm::dvec3 cam_world = cam.position_world;
        const glm::dmat3 world_to_cam = glm::transpose(glm::dmat3(cam.getRotationMatrix()));

        const double logical_w = static_cast<double>(logical_extent.width);
        const double logical_h = static_cast<double>(logical_extent.height);
        if (!(logical_w > 0.0) || !(logical_h > 0.0))
        {
            return;
        }

        const double fov_rad = glm::radians(static_cast<double>(cam.fovDegrees));
        const double tan_half_fov = std::tan(fov_rad * 0.5);
        const double aspect = logical_w / logical_h;
        if (!std::isfinite(tan_half_fov) || tan_half_fov <= 1.0e-8 || !std::isfinite(aspect) || aspect <= 0.0)
        {
            return;
        }

        const double draw_from_swap_x = static_cast<double>(draw_w) / static_cast<double>(swap_extent.width);
        const double draw_from_swap_y = static_cast<double>(draw_h) / static_cast<double>(swap_extent.height);
        const double window_from_draw_x = static_cast<double>(win_w) / static_cast<double>(draw_w);
        const double window_from_draw_y = static_cast<double>(win_h) / static_cast<double>(draw_h);
        if (!std::isfinite(draw_from_swap_x) || !std::isfinite(draw_from_swap_y) ||
            !std::isfinite(window_from_draw_x) || !std::isfinite(window_from_draw_y))
        {
            return;
        }

        const CelestialBodyInfo *ref_info = _orbitsim ? _orbitsim->reference_body() : nullptr;
        const bool depth_occluder_valid =
                ref_info && std::isfinite(ref_info->radius_m) && ref_info->radius_m > 0.0;
        const WorldVec3 depth_occluder_center = depth_occluder_valid
                                                    ? prediction_reference_body_world()
                                                    : WorldVec3(0.0, 0.0, 0.0);
        const double depth_occluder_radius = depth_occluder_valid ? ref_info->radius_m : 0.0;

        auto is_occluded_by_reference_body = [&](const WorldVec3 &point_world) -> bool {
            if (!depth_occluder_valid)
            {
                return false;
            }

            const glm::dvec3 center_to_cam = cam_world - glm::dvec3(depth_occluder_center);
            const double center_to_cam_len2 = glm::dot(center_to_cam, center_to_cam);
            const double radius2 = depth_occluder_radius * depth_occluder_radius;
            if (!std::isfinite(center_to_cam_len2) || center_to_cam_len2 <= radius2)
            {
                return false;
            }

            const glm::dvec3 cam_to_point = glm::dvec3(point_world) - cam_world;
            const double point_dist = safe_length(cam_to_point);
            if (!std::isfinite(point_dist) || point_dist <= 1.0e-3)
            {
                return false;
            }

            const glm::dvec3 ray_dir = cam_to_point / point_dist;
            const glm::dvec3 oc = cam_world - glm::dvec3(depth_occluder_center);
            const double b = glm::dot(oc, ray_dir);
            const double c = glm::dot(oc, oc) - radius2;
            const double disc = b * b - c;
            if (!std::isfinite(disc) || disc <= 0.0)
            {
                return false;
            }

            const double t_near = -b - std::sqrt(disc);
            const double eps = std::max(1.0, depth_occluder_radius * 1.0e-6);
            return std::isfinite(t_near) && t_near > eps && t_near < (point_dist - eps);
        };

        auto project_world_to_screen = [&](const WorldVec3 &point_world, ImVec2 &out_screen, double &out_depth_m) -> bool {
            out_screen = ImVec2(0.0f, 0.0f);
            out_depth_m = 0.0;

            const glm::dvec3 rel_world = glm::dvec3(point_world) - cam_world;
            const glm::dvec3 cam_space = world_to_cam * rel_world;
            const double depth_m = -cam_space.z;
            if (!std::isfinite(depth_m) || depth_m <= 1.0e-4)
            {
                return false;
            }

            const double ndc_x = cam_space.x / (depth_m * aspect * tan_half_fov);
            const double ndc_y = cam_space.y / (depth_m * tan_half_fov);
            if (!std::isfinite(ndc_x) || !std::isfinite(ndc_y))
            {
                return false;
            }
            if (ndc_x < -1.2 || ndc_x > 1.2 || ndc_y < -1.2 || ndc_y > 1.2)
            {
                return false;
            }

            const double logical_x = ((ndc_x + 1.0) * 0.5) * logical_w;
            const double logical_y = ((1.0 - ndc_y) * 0.5) * logical_h;
            if (!std::isfinite(logical_x) || !std::isfinite(logical_y))
            {
                return false;
            }

            const double u = logical_x / logical_w;
            const double v = logical_y / logical_h;
            const double swap_x = static_cast<double>(letterbox_rect.offset.x) +
                                  u * static_cast<double>(letterbox_rect.extent.width);
            const double swap_y = static_cast<double>(letterbox_rect.offset.y) +
                                  v * static_cast<double>(letterbox_rect.extent.height);

            const double draw_x = swap_x * draw_from_swap_x;
            const double draw_y = swap_y * draw_from_swap_y;
            const double win_x = draw_x * window_from_draw_x;
            const double win_y = draw_y * window_from_draw_y;
            if (!std::isfinite(win_x) || !std::isfinite(win_y))
            {
                return false;
            }

            out_screen = ImVec2(static_cast<float>(win_x), static_cast<float>(win_y));
            out_depth_m = depth_m;
            return true;
        };

        auto resolve_axis = [](const ManeuverNode &node,
                               const ManeuverHandleAxis axis,
                               glm::dvec3 &out_axis_dir_world,
                               int &out_component,
                               double &out_sign) -> bool {
            out_axis_dir_world = glm::dvec3(0.0, 1.0, 0.0);
            out_component = 1;
            out_sign = 1.0;

            switch (axis)
            {
                case ManeuverHandleAxis::ProgradePos:
                    out_axis_dir_world = node.basis_t_world;
                    out_component = 1;
                    out_sign = +1.0;
                    return true;
                case ManeuverHandleAxis::ProgradeNeg:
                    out_axis_dir_world = -node.basis_t_world;
                    out_component = 1;
                    out_sign = -1.0;
                    return true;
                case ManeuverHandleAxis::RadialPos:
                    out_axis_dir_world = node.basis_r_world;
                    out_component = 0;
                    out_sign = +1.0;
                    return true;
                case ManeuverHandleAxis::RadialNeg:
                    out_axis_dir_world = -node.basis_r_world;
                    out_component = 0;
                    out_sign = -1.0;
                    return true;
                case ManeuverHandleAxis::NormalPos:
                    out_axis_dir_world = node.basis_n_world;
                    out_component = 2;
                    out_sign = +1.0;
                    return true;
                case ManeuverHandleAxis::NormalNeg:
                    out_axis_dir_world = -node.basis_n_world;
                    out_component = 2;
                    out_sign = -1.0;
                    return true;
                default:
                    return false;
            }
        };

        auto axis_label = [](const ManeuverHandleAxis axis) -> const char * {
            switch (axis)
            {
                case ManeuverHandleAxis::ProgradePos: return "+T";
                case ManeuverHandleAxis::ProgradeNeg: return "-T";
                case ManeuverHandleAxis::RadialPos: return "+R";
                case ManeuverHandleAxis::RadialNeg: return "-R";
                case ManeuverHandleAxis::NormalPos: return "+N";
                case ManeuverHandleAxis::NormalNeg: return "-N";
                default: return "";
            }
        };

        auto axis_color = [](const ManeuverHandleAxis axis) -> ImU32 {
            switch (axis)
            {
                case ManeuverHandleAxis::ProgradePos:
                case ManeuverHandleAxis::ProgradeNeg:
                    return IM_COL32(95, 235, 105, 220);
                case ManeuverHandleAxis::RadialPos:
                case ManeuverHandleAxis::RadialNeg:
                    return IM_COL32(240, 100, 100, 220);
                case ManeuverHandleAxis::NormalPos:
                case ManeuverHandleAxis::NormalNeg:
                    return IM_COL32(95, 160, 255, 220);
                default:
                    return IM_COL32(170, 170, 170, 180);
            }
        };

        auto begin_drag = [&](const int node_id, const ManeuverHandleAxis axis) -> bool {
            if (axis == ManeuverHandleAxis::None || axis == ManeuverHandleAxis::Hub)
            {
                return false;
            }

            ManeuverNode *node = _maneuver_state.find_node(node_id);
            if (!node || !node->gizmo_valid)
            {
                return false;
            }

            glm::dvec3 axis_dir_world(0.0, 1.0, 0.0);
            int component = 1;
            double sign = 1.0;
            if (!resolve_axis(*node, axis, axis_dir_world, component, sign))
            {
                return false;
            }

            const ImVec2 mp = ImGui::GetIO().MousePos;
            CameraRay ray{};
            if (!compute_camera_ray(ctx, glm::vec2(mp.x, mp.y), ray))
            {
                return false;
            }

            double start_t = 0.0;
            const glm::dvec3 axis_origin_local = glm::dvec3(node->position_world - ray.camera_world);
            if (!closest_param_ray_line(ray.ray_origin_local, ray.ray_dir_local, axis_origin_local, axis_dir_world, start_t))
            {
                return false;
            }

            _maneuver_gizmo_interaction.state = ManeuverGizmoInteraction::State::DragAxis;
            _maneuver_gizmo_interaction.node_id = node_id;
            _maneuver_gizmo_interaction.axis = axis;
            _maneuver_gizmo_interaction.start_dv_rtn_mps = node->dv_rtn_mps;
            _maneuver_gizmo_interaction.start_axis_t_m = start_t;
            _maneuver_gizmo_interaction.applied_delta = false;
            return true;
        };

        struct HubMarker
        {
            int node_id{-1};
            ImVec2 screen{0.0f, 0.0f};
            bool selected{false};
            double depth_m{0.0};
        };

        struct AxisMarker
        {
            int node_id{-1};
            ManeuverHandleAxis axis{ManeuverHandleAxis::None};
            ImVec2 hub_screen{0.0f, 0.0f};
            ImVec2 handle_screen{0.0f, 0.0f};
            ImU32 base_color{IM_COL32(255, 255, 255, 255)};
            const char *label{""};
            double depth_m{0.0};
        };

        const float overlay_scale = std::max(0.25f, _maneuver_gizmo_style.overlay_scale);
        const float overlay_size_px = std::max(6.0f, _maneuver_gizmo_style.icon_size_px * overlay_scale);

        std::vector<HubMarker> hubs{};
        hubs.reserve(_maneuver_state.nodes.size());
        for (const ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!node.gizmo_valid)
            {
                continue;
            }
            if (is_occluded_by_reference_body(node.position_world))
            {
                continue;
            }

            HubMarker hub{};
            hub.node_id = node.id;
            hub.selected = (node.id == _maneuver_state.selected_node_id);
            if (!project_world_to_screen(node.position_world, hub.screen, hub.depth_m))
            {
                continue;
            }

            hubs.push_back(hub);
        }

        std::vector<AxisMarker> handles{};
        handles.reserve(6);
        if (ManeuverNode *selected = _maneuver_state.find_node(_maneuver_state.selected_node_id))
        {
            if (selected->gizmo_valid && !is_occluded_by_reference_body(selected->position_world))
            {
                ImVec2 hub_screen{};
                double hub_depth_m = 0.0;
                if (project_world_to_screen(selected->position_world, hub_screen, hub_depth_m))
                {
                    double dist_m = safe_length(glm::dvec3(selected->position_world) - cam_world);
                    if (!std::isfinite(dist_m) || dist_m <= 1.0)
                    {
                        dist_m = std::max(1.0, hub_depth_m);
                    }

                    const double meters_per_px = (2.0 * tan_half_fov * dist_m) / logical_h;
                    const float axis_len_px = std::max(24.0f, overlay_size_px * 1.9f);
                    const double axis_offset_m = meters_per_px * static_cast<double>(axis_len_px);

                    struct AxisDesc
                    {
                        ManeuverHandleAxis axis{ManeuverHandleAxis::None};
                        glm::dvec3 dir_world{0.0, 0.0, 0.0};
                    };

                    const AxisDesc axis_descs[6]{
                        {ManeuverHandleAxis::ProgradePos, selected->basis_t_world},
                        {ManeuverHandleAxis::ProgradeNeg, -selected->basis_t_world},
                        {ManeuverHandleAxis::RadialPos, selected->basis_r_world},
                        {ManeuverHandleAxis::RadialNeg, -selected->basis_r_world},
                        {ManeuverHandleAxis::NormalPos, selected->basis_n_world},
                        {ManeuverHandleAxis::NormalNeg, -selected->basis_n_world}
                    };

                    for (const AxisDesc &desc : axis_descs)
                    {
                        const glm::dvec3 dir = normalized_or(desc.dir_world, glm::dvec3(0.0, 1.0, 0.0));
                        const bool is_active_axis =
                                (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis) &&
                                (_maneuver_gizmo_interaction.node_id == selected->id) &&
                                (_maneuver_gizmo_interaction.axis == desc.axis);
                        const double axis_offset_active_m = axis_offset_m * (is_active_axis ? 1.14 : 1.0);
                        const WorldVec3 handle_world = selected->position_world + dir * axis_offset_active_m;
                        if (is_occluded_by_reference_body(handle_world))
                        {
                            continue;
                        }

                        AxisMarker marker{};
                        marker.node_id = selected->id;
                        marker.axis = desc.axis;
                        marker.hub_screen = hub_screen;
                        marker.base_color = axis_color(desc.axis);
                        marker.label = axis_label(desc.axis);
                        if (!project_world_to_screen(handle_world, marker.handle_screen, marker.depth_m))
                        {
                            continue;
                        }

                        handles.push_back(marker);
                    }
                }
            }
        }

        const ImGuiIO &io = ImGui::GetIO();
        const ImVec2 mouse_pos = io.MousePos;

        const float hub_hit_px = std::max(10.0f, overlay_size_px * 0.45f);
        const float axis_hit_px = std::max(9.0f, overlay_size_px * 0.36f);
        const float hub_hit_px2 = hub_hit_px * hub_hit_px;
        const float axis_hit_px2 = axis_hit_px * axis_hit_px;

        int hovered_handle_idx = -1;
        float best_handle_d2 = std::numeric_limits<float>::max();
        for (size_t i = 0; i < handles.size(); ++i)
        {
            const ImVec2 d = ImVec2(handles[i].handle_screen.x - mouse_pos.x, handles[i].handle_screen.y - mouse_pos.y);
            const float d2 = d.x * d.x + d.y * d.y;
            if (d2 <= axis_hit_px2 && d2 < best_handle_d2)
            {
                best_handle_d2 = d2;
                hovered_handle_idx = static_cast<int>(i);
            }
        }

        int hovered_hub_idx = -1;
        float best_hub_d2 = std::numeric_limits<float>::max();
        if (hovered_handle_idx < 0)
        {
            for (size_t i = 0; i < hubs.size(); ++i)
            {
                const ImVec2 d = ImVec2(hubs[i].screen.x - mouse_pos.x, hubs[i].screen.y - mouse_pos.y);
                const float d2 = d.x * d.x + d.y * d.y;
                if (d2 <= hub_hit_px2 && d2 < best_hub_d2)
                {
                    best_hub_d2 = d2;
                    hovered_hub_idx = static_cast<int>(i);
                }
            }
        }

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
                (void) begin_drag(handles[hovered_handle_idx].node_id, handles[hovered_handle_idx].axis);
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
                _maneuver_gizmo_interaction = {};
            }
            else if (!ctx.input->mouse_down(MouseButton::Left))
            {
                const bool changed = _maneuver_gizmo_interaction.applied_delta;
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
                    mark_prediction_dirty();
                }
            }
            else
            {
                glm::dvec3 axis_dir_world(0.0, 1.0, 0.0);
                int component = 1;
                double sign = 1.0;
                if (!resolve_axis(*node, _maneuver_gizmo_interaction.axis, axis_dir_world, component, sign))
                {
                    _maneuver_gizmo_interaction = {};
                }
                else
                {
                    CameraRay ray{};
                    if (compute_camera_ray(ctx, glm::vec2(mouse_pos.x, mouse_pos.y), ray))
                    {
                        double current_t = 0.0;
                        const glm::dvec3 axis_origin_local = glm::dvec3(node->position_world - ray.camera_world);
                        if (closest_param_ray_line(ray.ray_origin_local, ray.ray_dir_local,
                                                   axis_origin_local, axis_dir_world, current_t))
                        {
                            const double delta_axis_m = current_t - _maneuver_gizmo_interaction.start_axis_t_m;
                            double scale = static_cast<double>(
                                    std::max(0.00001f, _maneuver_gizmo_style.drag_sensitivity_mps_per_m));
                            const InputModifiers mods = ctx.input->modifiers();
                            if (mods.ctrl && !mods.shift)
                            {
                                scale *= 10.0;
                            }
                            else if (mods.shift && !mods.ctrl)
                            {
                                scale *= 0.1;
                            }

                            const double delta_mps = delta_axis_m * scale * sign;
                            glm::dvec3 dv_new = _maneuver_gizmo_interaction.start_dv_rtn_mps;
                            if (component == 0)
                            {
                                dv_new.x = _maneuver_gizmo_interaction.start_dv_rtn_mps.x + delta_mps;
                            }
                            else if (component == 1)
                            {
                                dv_new.y = _maneuver_gizmo_interaction.start_dv_rtn_mps.y + delta_mps;
                            }
                            else
                            {
                                dv_new.z = _maneuver_gizmo_interaction.start_dv_rtn_mps.z + delta_mps;
                            }

                            if (finite3(dv_new) && safe_length(dv_new - node->dv_rtn_mps) > 1.0e-7)
                            {
                                node->dv_rtn_mps = dv_new;
                                node->total_dv_mps = safe_length(node->dv_rtn_mps);
                                _maneuver_gizmo_interaction.applied_delta = true;
                                mark_prediction_dirty();
                            }
                        }
                    }
                }
            }
        }

        ImDrawList *dl = ImGui::GetForegroundDrawList();
        if (!dl)
        {
            return;
        }

        auto clamped_style_color = [&](float boost, float alpha_scale) -> ImVec4 {
            const glm::vec4 base = _maneuver_gizmo_style.icon_color;
            return ImVec4(std::clamp(base.x + boost, 0.0f, 1.0f),
                          std::clamp(base.y + boost, 0.0f, 1.0f),
                          std::clamp(base.z + boost, 0.0f, 1.0f),
                          std::clamp(base.w * alpha_scale, 0.1f, 1.0f));
        };

        for (const HubMarker &hub : hubs)
        {
            const bool is_selected = (hub.node_id == _maneuver_state.selected_node_id);
            const bool is_hovered = (hovered_hub_idx >= 0 && hubs[hovered_hub_idx].node_id == hub.node_id);
            const float radius = is_selected ? hub_hit_px * 0.58f : hub_hit_px * 0.50f;

            const ImVec4 fill_rgba = is_selected
                                         ? ImVec4(1.0f, 0.80f, 0.35f, 0.92f)
                                         : (is_hovered ? clamped_style_color(0.18f, 1.0f) : clamped_style_color(0.0f, 0.95f));
            const ImVec4 ring_rgba = is_selected
                                         ? ImVec4(1.0f, 0.90f, 0.55f, 0.86f)
                                         : clamped_style_color(0.25f, 0.72f);

            dl->AddCircleFilled(hub.screen, radius, ImGui::ColorConvertFloat4ToU32(fill_rgba), 24);
            dl->AddCircle(hub.screen, radius + 2.5f, ImGui::ColorConvertFloat4ToU32(ring_rgba), 24, 1.8f);
        }

        for (const AxisMarker &h : handles)
        {
            const bool is_hovered = (hovered_handle_idx >= 0 &&
                                     handles[hovered_handle_idx].node_id == h.node_id &&
                                     handles[hovered_handle_idx].axis == h.axis);
            const bool is_active = (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis) &&
                                   (_maneuver_gizmo_interaction.node_id == h.node_id) &&
                                   (_maneuver_gizmo_interaction.axis == h.axis);

            const ImU32 col = is_active
                                  ? IM_COL32(255, 190, 70, 240)
                                  : (is_hovered ? IM_COL32(255, 240, 140, 230) : h.base_color);
            const float thickness = is_active ? 3.2f : (is_hovered ? 2.8f : 2.2f);
            const float handle_r = is_active ? axis_hit_px * 0.95f : (is_hovered ? axis_hit_px * 0.88f : axis_hit_px * 0.78f);

            dl->AddLine(h.hub_screen, h.handle_screen, col, thickness);
            dl->AddCircleFilled(h.handle_screen, handle_r, col, 24);
            if (_maneuver_gizmo_style.show_axis_labels)
            {
                dl->AddText(ImVec2(h.handle_screen.x + 6.0f, h.handle_screen.y - 7.0f), col, h.label);
            }
        }

        if (hovered_handle_idx >= 0 && _maneuver_gizmo_interaction.state != ManeuverGizmoInteraction::State::DragAxis)
        {
            const AxisMarker &hover_h = handles[hovered_handle_idx];
            if (const ManeuverNode *node = _maneuver_state.find_node(hover_h.node_id))
            {
                ImGui::BeginTooltip();
                ImGui::Text("Node %d", node->id);
                ImGui::Text("Axis: %s", hover_h.label);
                ImGui::Text("DV RTN: (%.2f, %.2f, %.2f) m/s",
                            node->dv_rtn_mps.x, node->dv_rtn_mps.y, node->dv_rtn_mps.z);
                ImGui::TextUnformatted("Drag LMB (Shift x0.1 / Ctrl x10)");
                ImGui::EndTooltip();
            }
        }
    }

    void GameplayState::refresh_maneuver_node_runtime_cache(GameStateContext &ctx)
    {
        for (ManeuverNode &node : _maneuver_state.nodes)
        {
            node.total_dv_mps = safe_length(node.dv_rtn_mps);
            node.gizmo_valid = false;
        }

        if (!_orbitsim || !_prediction_cache.valid || _prediction_cache.trajectory_bci.size() < 2)
        {
            return;
        }

        // Match orbit plot alignment logic so maneuver gizmos sit on the displayed curve.
        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double interp_dt_s =
                (_last_sim_step_dt_s > 0.0) ? _last_sim_step_dt_s : static_cast<double>(ctx.fixed_delta_time());
        double now_s = _orbitsim->sim.time_s();

        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }

        const auto &traj = _prediction_cache.trajectory_bci;
        const double t0 = traj.front().t_s;
        const double t1 = traj.back().t_s;
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

        auto it_hi = std::lower_bound(traj.cbegin(),
                                      traj.cend(),
                                      now_s,
                                      [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
        const size_t i_hi = static_cast<size_t>(std::distance(traj.cbegin(), it_hi));
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

        for (ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!std::isfinite(node.time_s) || node.time_s < t0 || node.time_s > t1)
            {
                continue;
            }

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

            node.basis_r_world = normalized_or(glm::dvec3(f.R.x, f.R.y, f.R.z), glm::dvec3(1.0, 0.0, 0.0));
            node.basis_t_world = normalized_or(glm::dvec3(f.T.x, f.T.y, f.T.z), glm::dvec3(0.0, 1.0, 0.0));
            node.basis_n_world = normalized_or(glm::dvec3(f.N.x, f.N.y, f.N.z), glm::dvec3(0.0, 0.0, 1.0));

            const glm::dvec3 dv_world =
                    node.basis_r_world * node.dv_rtn_mps.x +
                    node.basis_t_world * node.dv_rtn_mps.y +
                    node.basis_n_world * node.dv_rtn_mps.z;

            node.burn_direction_world = normalized_or(dv_world, node.basis_t_world);
            node.position_world = s.position_world + align_delta;

            if (!std::isfinite(node.gizmo_scale_m) || node.gizmo_scale_m <= 0.0f)
            {
                node.gizmo_scale_m = 1.0f;
            }

            node.gizmo_valid = finite3(glm::dvec3(node.position_world)) &&
                               finite3(node.basis_r_world) &&
                               finite3(node.basis_t_world) &&
                               finite3(node.basis_n_world);
        }
    }
