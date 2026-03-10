void GameplayState::clear_maneuver_gizmo_instances(GameStateContext &ctx)
{
    (void) ctx;
    _maneuver_gizmo_interaction = {};
}

bool GameplayState::build_maneuver_gizmo_view_context(const GameStateContext &ctx,
                                                      ManeuverGizmoViewContext &out_view) const
{
    // Gather the camera and render-space transforms needed to project world-space node markers into ImGui space.
    out_view = {};

    if (!ctx.renderer || !ctx.renderer->_sceneManager || !ctx.renderer->_swapchainManager || !ctx.renderer->_window)
    {
        return false;
    }

    int win_w = 0;
    int win_h = 0;
    int draw_w = 0;
    int draw_h = 0;
    SDL_GetWindowSize(ctx.renderer->_window, &win_w, &win_h);
    SDL_Vulkan_GetDrawableSize(ctx.renderer->_window, &draw_w, &draw_h);
    if (win_w <= 0 || win_h <= 0 || draw_w <= 0 || draw_h <= 0)
    {
        return false;
    }

    const VkExtent2D swap_extent = ctx.renderer->_swapchainManager->swapchainExtent();
    if (swap_extent.width == 0 || swap_extent.height == 0)
    {
        return false;
    }

    VkExtent2D logical_extent = ctx.renderer->_logicalRenderExtent;
    if (logical_extent.width == 0 || logical_extent.height == 0)
    {
        logical_extent = VkExtent2D{1280, 720};
    }
    if (logical_extent.width == 0 || logical_extent.height == 0)
    {
        return false;
    }

    const VkRect2D letterbox_rect = vkutil::compute_letterbox_rect(logical_extent, swap_extent);
    if (letterbox_rect.extent.width == 0 || letterbox_rect.extent.height == 0)
    {
        return false;
    }

    const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
    out_view.camera_world = cam.position_world;
    out_view.world_to_cam = glm::transpose(glm::dmat3(cam.getRotationMatrix()));
    out_view.letterbox_rect = letterbox_rect;

    out_view.logical_w = static_cast<double>(logical_extent.width);
    out_view.logical_h = static_cast<double>(logical_extent.height);
    if (!(out_view.logical_w > 0.0) || !(out_view.logical_h > 0.0))
    {
        return false;
    }

    const double fov_rad = glm::radians(static_cast<double>(cam.fovDegrees));
    out_view.tan_half_fov = std::tan(fov_rad * 0.5);
    out_view.aspect = out_view.logical_w / out_view.logical_h;
    if (!std::isfinite(out_view.tan_half_fov) || out_view.tan_half_fov <= 1.0e-8 ||
        !std::isfinite(out_view.aspect) || out_view.aspect <= 0.0)
    {
        return false;
    }

    out_view.draw_from_swap_x = static_cast<double>(draw_w) / static_cast<double>(swap_extent.width);
    out_view.draw_from_swap_y = static_cast<double>(draw_h) / static_cast<double>(swap_extent.height);
    out_view.window_from_draw_x = static_cast<double>(win_w) / static_cast<double>(draw_w);
    out_view.window_from_draw_y = static_cast<double>(win_h) / static_cast<double>(draw_h);
    // ImGui mouse coordinates live in window pixels, while projection starts from logical render space.
    // Cache the conversion chain once so projection and picking use exactly the same path.
    if (!std::isfinite(out_view.draw_from_swap_x) || !std::isfinite(out_view.draw_from_swap_y) ||
        !std::isfinite(out_view.window_from_draw_x) || !std::isfinite(out_view.window_from_draw_y))
    {
        return false;
    }

    const CelestialBodyInfo *ref_info =
            (_prediction_frame_selection.spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial)
                ? find_celestial_body_info(_prediction_frame_selection.spec.primary_body_id)
                : nullptr;
    out_view.depth_occluder_valid = ref_info && std::isfinite(ref_info->radius_m) && ref_info->radius_m > 0.0;
    out_view.depth_occluder_center = out_view.depth_occluder_valid
                                         ? prediction_frame_origin_world(player_prediction_cache() ? *player_prediction_cache() : OrbitPredictionCache{})
                                         : WorldVec3(0.0, 0.0, 0.0);
    out_view.depth_occluder_radius = out_view.depth_occluder_valid ? ref_info->radius_m : 0.0;
    return true;
}

bool GameplayState::maneuver_gizmo_is_occluded(const ManeuverGizmoViewContext &view, const WorldVec3 &point_world) const
{
    // Approximate occlusion against the current reference body as a sphere so hidden markers do not bleed through.
    if (!view.depth_occluder_valid)
    {
        return false;
    }

    const glm::dvec3 center_to_cam = view.camera_world - glm::dvec3(view.depth_occluder_center);
    const double center_to_cam_len2 = glm::dot(center_to_cam, center_to_cam);
    const double radius2 = view.depth_occluder_radius * view.depth_occluder_radius;
    if (!std::isfinite(center_to_cam_len2) || center_to_cam_len2 <= radius2)
    {
        return false;
    }

    const glm::dvec3 cam_to_point = glm::dvec3(point_world) - view.camera_world;
    const double point_dist = safe_length(cam_to_point);
    if (!std::isfinite(point_dist) || point_dist <= 1.0e-3)
    {
        return false;
    }

    const glm::dvec3 ray_dir = cam_to_point / point_dist;
    const glm::dvec3 oc = view.camera_world - glm::dvec3(view.depth_occluder_center);
    const double b = glm::dot(oc, ray_dir);
    const double c = glm::dot(oc, oc) - radius2;
    const double disc = b * b - c;
    if (!std::isfinite(disc) || disc <= 0.0)
    {
        return false;
    }

    const double t_near = -b - std::sqrt(disc);
    const double eps = std::max(1.0, view.depth_occluder_radius * 1.0e-6);
    return std::isfinite(t_near) && t_near > eps && t_near < (point_dist - eps);
}

bool GameplayState::project_maneuver_gizmo_point(const ManeuverGizmoViewContext &view,
                                                 const WorldVec3 &point_world,
                                                 glm::vec2 &out_screen,
                                                 double &out_depth_m) const
{
    // Project through camera space into logical render space, then map back out to the actual window pixel location.
    out_screen = glm::vec2(0.0f, 0.0f);
    out_depth_m = 0.0;

    const glm::dvec3 rel_world = glm::dvec3(point_world) - view.camera_world;
    const glm::dvec3 cam_space = view.world_to_cam * rel_world;
    const double depth_m = -cam_space.z;
    if (!std::isfinite(depth_m) || depth_m <= 1.0e-4)
    {
        return false;
    }

    const double ndc_x = cam_space.x / (depth_m * view.aspect * view.tan_half_fov);
    const double ndc_y = cam_space.y / (depth_m * view.tan_half_fov);
    if (!std::isfinite(ndc_x) || !std::isfinite(ndc_y))
    {
        return false;
    }
    if (ndc_x < -1.2 || ndc_x > 1.2 || ndc_y < -1.2 || ndc_y > 1.2)
    {
        return false;
    }

    const double logical_x = ((ndc_x + 1.0) * 0.5) * view.logical_w;
    const double logical_y = ((1.0 - ndc_y) * 0.5) * view.logical_h;
    if (!std::isfinite(logical_x) || !std::isfinite(logical_y))
    {
        return false;
    }

    const double u = logical_x / view.logical_w;
    const double v = logical_y / view.logical_h;
    const double swap_x =
            static_cast<double>(view.letterbox_rect.offset.x) + u * static_cast<double>(view.letterbox_rect.extent.
                width);
    const double swap_y =
            static_cast<double>(view.letterbox_rect.offset.y) + v * static_cast<double>(view.letterbox_rect.extent.
                height);

    const double draw_x = swap_x * view.draw_from_swap_x;
    const double draw_y = swap_y * view.draw_from_swap_y;
    const double win_x = draw_x * view.window_from_draw_x;
    const double win_y = draw_y * view.window_from_draw_y;
    if (!std::isfinite(win_x) || !std::isfinite(win_y))
    {
        return false;
    }

    out_screen = glm::vec2(static_cast<float>(win_x), static_cast<float>(win_y));
    out_depth_m = depth_m;
    return true;
}

bool GameplayState::resolve_maneuver_axis(const ManeuverNode &node,
                                          const ManeuverHandleAxis axis,
                                          glm::dvec3 &out_axis_dir_world,
                                          int &out_component,
                                          double &out_sign) const
{
    // Map a visible handle to both its world-space drag axis and the RTN component it edits.
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
}

const char *GameplayState::maneuver_axis_label(const ManeuverHandleAxis axis) const
{
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
}

uint32_t GameplayState::maneuver_axis_color(const ManeuverHandleAxis axis) const
{
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
}

bool GameplayState::begin_maneuver_axis_drag(GameStateContext &ctx,
                                             const int node_id,
                                             const ManeuverHandleAxis axis)
{
    // Capture the initial closest-point parameter so drag updates can be expressed as delta-along-axis.
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
    if (!resolve_maneuver_axis(*node, axis, axis_dir_world, component, sign))
    {
        return false;
    }

    const ImVec2 mouse_pos = ImGui::GetIO().MousePos;
    CameraRay ray{};
    if (!compute_camera_ray(ctx, glm::vec2(mouse_pos.x, mouse_pos.y), ray))
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
}

void GameplayState::apply_maneuver_axis_drag(GameStateContext &ctx,
                                             ManeuverNode &node,
                                             const glm::vec2 &mouse_pos_window)
{
    glm::dvec3 axis_dir_world(0.0, 1.0, 0.0);
    int component = 1;
    double sign = 1.0;
    if (!resolve_maneuver_axis(node, _maneuver_gizmo_interaction.axis, axis_dir_world, component, sign))
    {
        _maneuver_gizmo_interaction = {};
        return;
    }

    CameraRay ray{};
    if (!compute_camera_ray(ctx, mouse_pos_window, ray))
    {
        return;
    }

    double current_t = 0.0;
    const glm::dvec3 axis_origin_local = glm::dvec3(node.position_world - ray.camera_world);
    if (!closest_param_ray_line(ray.ray_origin_local, ray.ray_dir_local, axis_origin_local, axis_dir_world, current_t))
    {
        return;
    }

    const double delta_axis_m = current_t - _maneuver_gizmo_interaction.start_axis_t_m;
    double scale = static_cast<double>(std::max(0.00001f, _maneuver_gizmo_style.drag_sensitivity_mps_per_m));
    const InputModifiers mods = ctx.input->modifiers();
    if (mods.ctrl && !mods.shift)
    {
        scale *= 10.0;
    }
    else if (mods.shift && !mods.ctrl)
    {
        scale *= 0.1;
    }

    // The drag solver works in world meters along the selected axis; convert that signed motion into DV.
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

    if (finite3(dv_new) && safe_length(dv_new - node.dv_rtn_mps) > 1.0e-7)
    {
        node.dv_rtn_mps = dv_new;
        node.total_dv_mps = safe_length(node.dv_rtn_mps);
        _maneuver_gizmo_interaction.applied_delta = true;
        mark_prediction_dirty();
    }
}

void GameplayState::build_maneuver_gizmo_markers(const ManeuverGizmoViewContext &view,
                                                 const float overlay_size_px,
                                                 std::vector<ManeuverHubMarker> &out_hubs,
                                                 std::vector<ManeuverAxisMarker> &out_handles) const
{
    // Draw hubs for every visible node, but only expose draggable axis handles for the selected node.
    out_hubs.clear();
    out_handles.clear();

    out_hubs.reserve(_maneuver_state.nodes.size());
    for (const ManeuverNode &node: _maneuver_state.nodes)
    {
        if (!node.gizmo_valid)
        {
            continue;
        }
        if (maneuver_gizmo_is_occluded(view, node.position_world))
        {
            continue;
        }

        ManeuverHubMarker hub{};
        hub.node_id = node.id;
        if (!project_maneuver_gizmo_point(view, node.position_world, hub.screen, hub.depth_m))
        {
            continue;
        }

        out_hubs.push_back(hub);
    }

    out_handles.reserve(6);
    const ManeuverNode *selected = _maneuver_state.find_node(_maneuver_state.selected_node_id);
    if (!selected || !selected->gizmo_valid || maneuver_gizmo_is_occluded(view, selected->position_world))
    {
        return;
    }

    glm::vec2 hub_screen{};
    double hub_depth_m = 0.0;
    if (!project_maneuver_gizmo_point(view, selected->position_world, hub_screen, hub_depth_m))
    {
        return;
    }

    double dist_m = safe_length(glm::dvec3(selected->position_world) - view.camera_world);
    if (!std::isfinite(dist_m) || dist_m <= 1.0)
    {
        dist_m = std::max(1.0, hub_depth_m);
    }

    const double meters_per_px = (2.0 * view.tan_half_fov * dist_m) / view.logical_h;
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

    for (const AxisDesc &desc: axis_descs)
    {
        const glm::dvec3 dir = normalized_or(desc.dir_world, glm::dvec3(0.0, 1.0, 0.0));
        const bool is_active_axis =
                (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis) &&
                (_maneuver_gizmo_interaction.node_id == selected->id) &&
                (_maneuver_gizmo_interaction.axis == desc.axis);
        const double axis_offset_active_m = axis_offset_m * (is_active_axis ? 1.14 : 1.0);
        const WorldVec3 handle_world = selected->position_world + dir * axis_offset_active_m;
        if (maneuver_gizmo_is_occluded(view, handle_world))
        {
            continue;
        }

        ManeuverAxisMarker marker{};
        marker.node_id = selected->id;
        marker.axis = desc.axis;
        marker.hub_screen = hub_screen;
        marker.base_color = maneuver_axis_color(desc.axis);
        marker.label = maneuver_axis_label(desc.axis);
        if (!project_maneuver_gizmo_point(view, handle_world, marker.handle_screen, marker.depth_m))
        {
            continue;
        }

        out_handles.push_back(marker);
    }
}

void GameplayState::find_maneuver_gizmo_hover(const std::vector<ManeuverHubMarker> &hubs,
                                              const std::vector<ManeuverAxisMarker> &handles,
                                              const glm::vec2 &mouse_pos,
                                              const float hub_hit_px2,
                                              const float axis_hit_px2,
                                              int &out_hovered_hub_idx,
                                              int &out_hovered_handle_idx) const
{
    // Axis handles win over hubs so editing takes priority over node selection when both overlap on screen.
    out_hovered_hub_idx = -1;
    out_hovered_handle_idx = -1;

    float best_handle_d2 = std::numeric_limits<float>::max();
    for (size_t i = 0; i < handles.size(); ++i)
    {
        const glm::vec2 d = handles[i].handle_screen - mouse_pos;
        const float d2 = glm::dot(d, d);
        if (d2 <= axis_hit_px2 && d2 < best_handle_d2)
        {
            best_handle_d2 = d2;
            out_hovered_handle_idx = static_cast<int>(i);
        }
    }

    if (out_hovered_handle_idx >= 0)
    {
        return;
    }

    float best_hub_d2 = std::numeric_limits<float>::max();
    for (size_t i = 0; i < hubs.size(); ++i)
    {
        const glm::vec2 d = hubs[i].screen - mouse_pos;
        const float d2 = glm::dot(d, d);
        if (d2 <= hub_hit_px2 && d2 < best_hub_d2)
        {
            best_hub_d2 = d2;
            out_hovered_hub_idx = static_cast<int>(i);
        }
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

    for (const ManeuverHubMarker &hub: hubs)
    {
        const bool is_selected = (hub.node_id == _maneuver_state.selected_node_id);
        const bool is_hovered = (hovered_hub_idx >= 0 && hubs[hovered_hub_idx].node_id == hub.node_id);
        const float radius = is_selected ? hub_hit_px * 0.58f : hub_hit_px * 0.50f;

        const ImVec4 fill_rgba = is_selected
                                     ? ImVec4(1.0f, 0.80f, 0.35f, 0.92f)
                                     : (is_hovered
                                            ? clamped_style_color(0.18f, 1.0f)
                                            : clamped_style_color(0.0f, 0.95f));
        const ImVec4 ring_rgba = is_selected
                                     ? ImVec4(1.0f, 0.90f, 0.55f, 0.86f)
                                     : clamped_style_color(0.25f, 0.72f);

        draw_list->AddCircleFilled(to_imvec2(hub.screen), radius, ImGui::ColorConvertFloat4ToU32(fill_rgba), 24);
        draw_list->AddCircle(to_imvec2(hub.screen), radius + 2.5f, ImGui::ColorConvertFloat4ToU32(ring_rgba), 24, 1.8f);
    }

    for (const ManeuverAxisMarker &h: handles)
    {
        const bool is_hovered = (hovered_handle_idx >= 0 &&
                                 handles[hovered_handle_idx].node_id == h.node_id &&
                                 handles[hovered_handle_idx].axis == h.axis);
        const bool is_active = (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis) &&
                               (_maneuver_gizmo_interaction.node_id == h.node_id) &&
                               (_maneuver_gizmo_interaction.axis == h.axis);

        const ImU32 col = is_active
                              ? IM_COL32(255, 190, 70, 240)
                              : (is_hovered ? IM_COL32(255, 240, 140, 230) : static_cast<ImU32>(h.base_color));
        const float thickness = is_active ? 3.2f : (is_hovered ? 2.8f : 2.2f);
        const float handle_r = is_active
                                   ? axis_hit_px * 0.95f
                                   : (is_hovered ? axis_hit_px * 0.88f : axis_hit_px * 0.78f);

        draw_list->AddLine(to_imvec2(h.hub_screen), to_imvec2(h.handle_screen), col, thickness);
        draw_list->AddCircleFilled(to_imvec2(h.handle_screen), handle_r, col, 24);
        if (_maneuver_gizmo_style.show_axis_labels)
        {
            draw_list->AddText(ImVec2(h.handle_screen.x + 6.0f, h.handle_screen.y - 7.0f), col, h.label);
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
        ImGui::Text("DV RTN: (%.2f, %.2f, %.2f) m/s",
                    node->dv_rtn_mps.x, node->dv_rtn_mps.y, node->dv_rtn_mps.z);
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

    refresh_maneuver_node_runtime_cache(ctx);

    ManeuverGizmoViewContext view{};
    if (!build_maneuver_gizmo_view_context(ctx, view))
    {
        return;
    }

    const float overlay_scale = std::max(0.25f, _maneuver_gizmo_style.overlay_scale);
    const float overlay_size_px = std::max(6.0f, _maneuver_gizmo_style.icon_size_px * overlay_scale);

    std::vector<ManeuverHubMarker> hubs{};
    std::vector<ManeuverAxisMarker> handles{};
    build_maneuver_gizmo_markers(view, overlay_size_px, hubs, handles);

    const ImGuiIO &io = ImGui::GetIO();
    const glm::vec2 mouse_pos(io.MousePos.x, io.MousePos.y);

    const float hub_hit_px = std::max(10.0f, overlay_size_px * 0.45f);
    const float axis_hit_px = std::max(9.0f, overlay_size_px * 0.36f);
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

void GameplayState::refresh_maneuver_node_runtime_cache(GameStateContext &ctx)
{
    // Rebuild all derived node state so UI/editor code can work from the latest prediction and render interpolation.
    for (ManeuverNode &node: _maneuver_state.nodes)
    {
        node.total_dv_mps = safe_length(node.dv_rtn_mps);
        node.gizmo_valid = false;
    }

    if (!_orbitsim || !_prediction_selection.active_subject.valid() ||
        !prediction_subject_is_player(_prediction_selection.active_subject))
    {
        return;
    }

    const OrbitPredictionCache *prediction_cache = player_prediction_cache();
    if (!prediction_cache || !prediction_cache->valid || prediction_cache->trajectory_frame.size() < 2)
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

    const auto &traj_base = prediction_cache->trajectory_frame;
    const auto &traj_planned = prediction_cache->trajectory_frame_planned;
    const auto &traj_now = traj_base;
    const double t0 = traj_now.front().t_s;
    const double t1 = traj_now.back().t_s;
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

    const WorldVec3 ref_body_world = prediction_frame_origin_world(*prediction_cache);

    auto it_hi = std::lower_bound(traj_now.cbegin(),
                                  traj_now.cend(),
                                  now_s,
                                  [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
    const size_t i_hi = static_cast<size_t>(std::distance(traj_now.cbegin(), it_hi));
    if (i_hi >= traj_now.size())
    {
        return;
    }

    WorldVec3 predicted_now_world = ref_body_world + WorldVec3(traj_now[i_hi].position_m);
    if (i_hi > 0)
    {
        predicted_now_world = hermite_position_world(ref_body_world, traj_now[i_hi - 1], traj_now[i_hi], now_s);
    }

    WorldVec3 align_delta = ship_pos_world - predicted_now_world;
    const double align_len = safe_length(glm::dvec3(align_delta));
    if (!std::isfinite(align_len) || align_len > 10'000.0)
    {
        // If prediction/render diverge too much, stop trying to align rather than placing markers far off-screen.
        align_delta = WorldVec3(0.0, 0.0, 0.0);
    }

    for (ManeuverNode &node: _maneuver_state.nodes)
    {
        if (!std::isfinite(node.time_s) || node.time_s < t0 || node.time_s > t1)
        {
            continue;
        }

        const OrbitPredictionCache::ManeuverNodePreview *preview = nullptr;
        for (const OrbitPredictionCache::ManeuverNodePreview &candidate: prediction_cache->maneuver_previews)
        {
            if (candidate.node_id == node.id)
            {
                preview = &candidate;
                break;
            }
        }

        glm::dvec3 r_rel_m{0.0, 0.0, 0.0};
        glm::dvec3 v_rel_mps{0.0, 0.0, 0.0};
        WorldVec3 node_position_world{0.0, 0.0, 0.0};
        bool have_runtime_state = false;

        if (preview && preview->valid)
        {
            const orbitsim::BodyId analysis_body_id =
                    resolve_prediction_analysis_body_id(*prediction_cache, _prediction_selection.active_subject, node.time_s);
            const orbitsim::MassiveBody *analysis_body = find_massive_body(prediction_cache->massive_bodies, analysis_body_id);
            if (analysis_body)
            {
                const orbitsim::State primary_state =
                        (prediction_cache->shared_ephemeris && !prediction_cache->shared_ephemeris->empty())
                            ? prediction_cache->shared_ephemeris->body_state_at_by_id(analysis_body_id, node.time_s)
                            : analysis_body->state;
                r_rel_m = preview->inertial_position_m - primary_state.position_m;
                v_rel_mps = preview->inertial_velocity_mps - primary_state.velocity_mps;
            }

            std::vector<orbitsim::TrajectorySample> preview_frame =
                    (prediction_cache->shared_ephemeris && !prediction_cache->shared_ephemeris->empty())
                        ? orbitsim::trajectory_to_frame_spec(
                                  {orbitsim::TrajectorySample{
                                          .t_s = node.time_s,
                                          .position_m = preview->inertial_position_m,
                                          .velocity_mps = preview->inertial_velocity_mps,
                                  }},
                                  *prediction_cache->shared_ephemeris,
                                  prediction_cache->massive_bodies,
                                  _prediction_frame_selection.spec)
                        : std::vector<orbitsim::TrajectorySample>{};
            if (_prediction_frame_selection.spec.type == orbitsim::TrajectoryFrameType::Inertial)
            {
                preview_frame = {orbitsim::TrajectorySample{
                        .t_s = node.time_s,
                        .position_m = preview->inertial_position_m,
                        .velocity_mps = preview->inertial_velocity_mps,
                }};
            }

            if (!preview_frame.empty())
            {
                node_position_world = ref_body_world + WorldVec3(preview_frame.front().position_m) + align_delta;
                have_runtime_state = finite3(r_rel_m) && finite3(v_rel_mps);
            }
        }
        else
        {
            // Fallback when previews are unavailable: sample the plotted trajectory directly.
            const auto &traj_node_world = traj_planned.size() >= 2 ? traj_planned : traj_base;
            const auto &traj_node_inertial =
                    prediction_cache->trajectory_inertial_planned.size() >= 2
                        ? prediction_cache->trajectory_inertial_planned
                        : prediction_cache->trajectory_inertial;
            const double traj_node_t0 = traj_node_world.front().t_s;
            const double traj_node_t1 = traj_node_world.back().t_s;
            if (node.time_s < traj_node_t0 || node.time_s > traj_node_t1 || traj_node_inertial.size() < 2)
            {
                continue;
            }

            double t_basis = node.time_s;
            if (t_basis > traj_node_t0)
            {
                t_basis = std::max(traj_node_t0, t_basis - 1e-3);
            }

            const TrajectorySampledState world_state = sample_trajectory_state(traj_node_world, ref_body_world, t_basis);
            if (!world_state.valid)
            {
                continue;
            }

            const orbitsim::BodyId analysis_body_id =
                    resolve_prediction_analysis_body_id(*prediction_cache, _prediction_selection.active_subject, t_basis);
            const orbitsim::MassiveBody *analysis_body = find_massive_body(prediction_cache->massive_bodies, analysis_body_id);
            if (!analysis_body)
            {
                continue;
            }

            const TrajectorySampledState inertial_state =
                    sample_trajectory_state(traj_node_inertial, WorldVec3(0.0), t_basis);
            if (!inertial_state.valid)
            {
                continue;
            }

            const orbitsim::State primary_state =
                    (prediction_cache->shared_ephemeris && !prediction_cache->shared_ephemeris->empty())
                        ? prediction_cache->shared_ephemeris->body_state_at_by_id(analysis_body_id, t_basis)
                        : analysis_body->state;

            r_rel_m = inertial_state.r_rel_m - primary_state.position_m;
            v_rel_mps = inertial_state.v_rel_mps - primary_state.velocity_mps;
            node_position_world = world_state.position_world + align_delta;
            have_runtime_state = finite3(r_rel_m) && finite3(v_rel_mps);
        }

        if (!have_runtime_state)
        {
            continue;
        }

        const orbitsim::RtnFrame f = compute_maneuver_rtf_frame(r_rel_m, v_rel_mps);

        node.basis_r_world = normalized_or(glm::dvec3(f.R.x, f.R.y, f.R.z), glm::dvec3(1.0, 0.0, 0.0));
        node.basis_t_world = normalized_or(glm::dvec3(f.T.x, f.T.y, f.T.z), glm::dvec3(0.0, 1.0, 0.0));
        node.basis_n_world = normalized_or(glm::dvec3(f.N.x, f.N.y, f.N.z), glm::dvec3(0.0, 0.0, 1.0));

        // Burn direction is used purely for debug visualization; zero-DV nodes fall back to the prograde axis.
        const glm::dvec3 dv_world =
                node.basis_r_world * node.dv_rtn_mps.x +
                node.basis_t_world * node.dv_rtn_mps.y +
                node.basis_n_world * node.dv_rtn_mps.z;

        node.burn_direction_world = normalized_or(dv_world, node.basis_t_world);
        node.position_world = node_position_world;

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
