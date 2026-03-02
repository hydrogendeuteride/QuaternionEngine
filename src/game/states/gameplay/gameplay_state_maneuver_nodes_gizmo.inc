    void GameplayState::clear_maneuver_gizmo_instances(GameStateContext &ctx)
    {
        PickingSystem *picking = (ctx.renderer != nullptr) ? ctx.renderer->picking() : nullptr;
        for (const std::string &name : _maneuver_gizmo_instances)
        {
            if (ctx.api)
            {
                (void) ctx.api->remove_mesh_instance(name);
            }
            if (picking)
            {
                picking->clear_owner_picks(RenderObject::OwnerType::MeshInstance, name);
            }
        }
        _maneuver_gizmo_instances.clear();
        _maneuver_gizmo_interaction = {};
    }

    void GameplayState::update_maneuver_gizmo_materials(GameStateContext &ctx)
    {
        if (!ctx.api)
        {
            return;
        }
        if (!_maneuver_gizmo_materials_dirty)
        {
            return;
        }

        const glm::vec3 icon_tint = glm::clamp(glm::vec3(_maneuver_gizmo_style.icon_color), glm::vec3(0.0f), glm::vec3(1.0f));
        const float icon_alpha = std::clamp(_maneuver_gizmo_style.icon_color.a, 0.05f, 1.0f);

        float preset_fresnel = 3.0f;
        float preset_emission = 2.0f;
        switch (_maneuver_gizmo_style.preset)
        {
            case ManeuverGizmoStyle::Preset::Classic:
                preset_fresnel = 3.0f;
                preset_emission = 2.2f;
                break;
            case ManeuverGizmoStyle::Preset::Minimal:
                preset_fresnel = 1.2f;
                preset_emission = 1.1f;
                break;
            case ManeuverGizmoStyle::Preset::Flat:
                preset_fresnel = 0.4f;
                preset_emission = 0.8f;
                break;
        }

        auto make_vfx = [&](const char *name, const glm::vec3 &tint, const float opacity,
                            const float fresnel_strength, const float emission) {
            GameAPI::MeshVfxMaterialSettings m{};
            m.tint = glm::clamp(tint, glm::vec3(0.0f), glm::vec3(1.0f));
            m.opacity = std::clamp(opacity, 0.01f, 1.0f);
            m.fresnelPower = preset_fresnel;
            m.fresnelStrength = std::max(0.0f, fresnel_strength);
            m.coreColor = m.tint;
            m.edgeColor = m.tint;
            m.emissionStrength = std::max(0.0f, emission);
            (void) ctx.api->create_or_update_mesh_vfx_material(name, m);
        };

        make_vfx(kMatHub, icon_tint, icon_alpha, 1.0f, preset_emission);
        make_vfx(kMatHubSelected, glm::mix(icon_tint, glm::vec3(1.0f, 0.85f, 0.30f), 0.55f),
                 std::min(1.0f, icon_alpha + 0.10f), 1.3f, preset_emission * 1.2f);

        make_vfx(kMatAxisRadial, glm::vec3(1.0f, 0.35f, 0.35f), std::max(0.20f, icon_alpha * 0.80f), 1.0f, 1.3f);
        make_vfx(kMatAxisPrograde, glm::vec3(0.35f, 1.0f, 0.35f), std::max(0.20f, icon_alpha * 0.80f), 1.0f, 1.3f);
        make_vfx(kMatAxisNormal, glm::vec3(0.35f, 0.65f, 1.0f), std::max(0.20f, icon_alpha * 0.80f), 1.0f, 1.3f);
        make_vfx(kMatAxisHover, glm::vec3(1.0f, 1.0f, 0.65f), std::min(1.0f, icon_alpha + 0.15f), 1.2f, 2.0f);
        make_vfx(kMatAxisActive, glm::vec3(1.0f, 0.78f, 0.20f), std::min(1.0f, icon_alpha + 0.25f), 1.4f, 2.3f);

        _maneuver_gizmo_materials_dirty = false;
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

    void GameplayState::sync_maneuver_gizmo_instances(GameStateContext &ctx)
    {
        refresh_maneuver_node_runtime_cache(ctx);

        if (!ctx.api || !_maneuver_nodes_enabled || _maneuver_state.nodes.empty())
        {
            clear_maneuver_gizmo_instances(ctx);
            return;
        }

        update_maneuver_gizmo_materials(ctx);

        PickingSystem *picking = (ctx.renderer != nullptr) ? ctx.renderer->picking() : nullptr;
        ParsedManeuverInstance hover{};
        if (picking)
        {
            const auto &pick = picking->hover_pick();
            if (pick.valid && pick.kind == PickingSystem::PickInfo::Kind::SceneObject)
            {
                hover = parse_maneuver_instance_name(pick.ownerName);
            }
        }

        const auto suffix_for_axis = [](const ManeuverHandleAxis axis) -> const char * {
            switch (axis)
            {
                case ManeuverHandleAxis::Hub: return "hub";
                case ManeuverHandleAxis::ProgradePos: return "prograde_pos";
                case ManeuverHandleAxis::ProgradeNeg: return "prograde_neg";
                case ManeuverHandleAxis::RadialPos: return "radial_pos";
                case ManeuverHandleAxis::RadialNeg: return "radial_neg";
                case ManeuverHandleAxis::NormalPos: return "normal_pos";
                case ManeuverHandleAxis::NormalNeg: return "normal_neg";
                default: return "";
            }
        };

        const char *active_suffix = "";
        if (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            active_suffix = suffix_for_axis(_maneuver_gizmo_interaction.axis);
        }

        const glm::dvec3 cam_world = (ctx.renderer && ctx.renderer->_sceneManager)
                                          ? ctx.renderer->_sceneManager->getMainCamera().position_world
                                          : ctx.api->get_camera_position_d();

        float viewport_h_px = 720.0f;
        float fov_deg = 70.0f;
        if (ctx.renderer && ctx.renderer->_sceneManager)
        {
            const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
            fov_deg = cam.fovDegrees;
            if (ctx.renderer->_logicalRenderExtent.height > 0)
            {
                viewport_h_px = static_cast<float>(ctx.renderer->_logicalRenderExtent.height);
            }
        }

        const double tan_half_fov = std::tan(glm::radians(static_cast<double>(fov_deg)) * 0.5);
        const float min_scale = std::max(1.0f, _maneuver_gizmo_style.min_scale_m);
        const float max_scale = std::max(min_scale, _maneuver_gizmo_style.max_scale_m);

        auto ensure_instance = [&](const std::string &name, const GameAPI::PrimitiveType primitive) {
            if (std::find(_maneuver_gizmo_instances.cbegin(), _maneuver_gizmo_instances.cend(), name) !=
                _maneuver_gizmo_instances.cend())
            {
                return;
            }
            GameAPI::TransformD tr{};
            if (ctx.api->add_primitive_instance(name, primitive, tr))
            {
                _maneuver_gizmo_instances.push_back(name);
            }
        };

        auto handle_material_for_suffix = [](const std::string &suffix) -> const char * {
            if (suffix.rfind("radial_", 0) == 0) return kMatAxisRadial;
            if (suffix.rfind("prograde_", 0) == 0) return kMatAxisPrograde;
            if (suffix.rfind("normal_", 0) == 0) return kMatAxisNormal;
            return kMatAxisPrograde;
        };

        struct AxisHandleDesc
        {
            const char *suffix{""};
            glm::dvec3 dir{0.0, 1.0, 0.0};
        };

        std::unordered_set<std::string> desired{};
        desired.reserve(_maneuver_state.nodes.size() * 8);

        for (ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!node.gizmo_valid)
            {
                continue;
            }

            const glm::dvec3 node_pos = glm::dvec3(node.position_world);
            double distance_m = safe_length(node_pos - cam_world);
            if (!std::isfinite(distance_m) || distance_m < 1.0)
            {
                distance_m = 1.0;
            }

            const double meters_per_px = (2.0 * tan_half_fov * distance_m) / std::max(1.0f, viewport_h_px);
            float raw_scale = static_cast<float>(meters_per_px * std::max(1.0f, _maneuver_gizmo_style.icon_size_px));
            raw_scale = std::clamp(raw_scale, min_scale, max_scale);

            float quantized = quantize_scale(raw_scale, std::max(1.01f, _maneuver_gizmo_style.quantize_ratio));
            quantized = std::clamp(quantized, min_scale, max_scale);

            if (std::isfinite(node.gizmo_scale_m) && node.gizmo_scale_m > 0.0f)
            {
                const float h = std::clamp(_maneuver_gizmo_style.hysteresis, 0.0f, 0.45f);
                const float lo = node.gizmo_scale_m * (1.0f - h);
                const float hi = node.gizmo_scale_m * (1.0f + h);
                if (quantized >= lo && quantized <= hi)
                {
                    quantized = node.gizmo_scale_m;
                }
            }
            node.gizmo_scale_m = quantized;

            WorldVec3 center_world = node.position_world;
            const glm::dvec3 to_node = glm::dvec3(center_world) - cam_world;
            const double to_node_len = safe_length(to_node);
            const double near_safe_dist = std::max(2.0, static_cast<double>(node.gizmo_scale_m) * 1.30);
            if (to_node_len > 1e-6 && to_node_len < near_safe_dist)
            {
                center_world = WorldVec3(cam_world + (to_node / to_node_len) * near_safe_dist);
            }

            std::string prefix = "mn_" + std::to_string(node.id) + "_";

            const char *hub_suffix = "hub";
            const std::string hub_name = prefix + hub_suffix;
            ensure_instance(hub_name, GameAPI::PrimitiveType::Sphere);
            desired.insert(hub_name);

            GameAPI::TransformD hub_tr{};
            hub_tr.position = glm::dvec3(center_world);
            float hub_scale = node.gizmo_scale_m * 0.45f;
            switch (_maneuver_gizmo_style.preset)
            {
                case ManeuverGizmoStyle::Preset::Classic: hub_scale = node.gizmo_scale_m * 0.45f; break;
                case ManeuverGizmoStyle::Preset::Minimal: hub_scale = node.gizmo_scale_m * 0.32f; break;
                case ManeuverGizmoStyle::Preset::Flat: hub_scale = node.gizmo_scale_m * 0.38f; break;
            }
            hub_tr.scale = glm::vec3(std::max(0.1f, hub_scale));
            if (_maneuver_gizmo_style.icon_billboard)
            {
                const glm::dvec3 view_dir = cam_world - glm::dvec3(center_world);
                hub_tr.rotation = quat_from_y_to_dir(view_dir);
            }
            (void) ctx.api->set_mesh_instance_transform(hub_name, hub_tr);

            const bool selected = (node.id == _maneuver_state.selected_node_id);
            const bool hub_hovered = hover.valid && hover.node_id == node.id && hover.suffix == hub_suffix;
            if (selected || hub_hovered)
            {
                (void) ctx.api->apply_mesh_vfx_material_to_primitive(hub_name, kMatHubSelected);
            }
            else
            {
                (void) ctx.api->apply_mesh_vfx_material_to_primitive(hub_name, kMatHub);
            }

            const AxisHandleDesc handles[6]{
                {"prograde_pos", node.basis_t_world},
                {"prograde_neg", -node.basis_t_world},
                {"radial_pos", node.basis_r_world},
                {"radial_neg", -node.basis_r_world},
                {"normal_pos", node.basis_n_world},
                {"normal_neg", -node.basis_n_world}
            };

            const float arm_offset = std::max(0.2f, node.gizmo_scale_m * 1.20f);
            const float half_len = std::max(0.1f, node.gizmo_scale_m * 0.42f);
            const float radius = std::max(0.05f, node.gizmo_scale_m * 0.09f);
            const float active_scale = (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis) ? 1.20f : 1.10f;

            for (const AxisHandleDesc &h : handles)
            {
                const std::string handle_name = prefix + h.suffix;
                ensure_instance(handle_name, GameAPI::PrimitiveType::Capsule);
                desired.insert(handle_name);

                const bool is_hover = hover.valid && hover.node_id == node.id && hover.suffix == h.suffix;
                const bool is_active = (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis) &&
                                       (_maneuver_gizmo_interaction.node_id == node.id) &&
                                       (std::string_view(active_suffix) == std::string_view(h.suffix));

                const float emphasize = is_active ? active_scale : (is_hover ? 1.10f : 1.0f);
                const glm::dvec3 dir = normalized_or(h.dir, glm::dvec3(0.0, 1.0, 0.0));

                GameAPI::TransformD tr{};
                tr.position = glm::dvec3(center_world) + dir * static_cast<double>(arm_offset * emphasize);
                tr.rotation = quat_from_y_to_dir(dir);
                tr.scale = glm::vec3(radius * emphasize, half_len * emphasize, radius * emphasize);
                (void) ctx.api->set_mesh_instance_transform(handle_name, tr);

                const char *mat = handle_material_for_suffix(h.suffix);
                if (is_active)
                {
                    mat = kMatAxisActive;
                }
                else if (is_hover)
                {
                    mat = kMatAxisHover;
                }
                (void) ctx.api->apply_mesh_vfx_material_to_primitive(handle_name, mat);
            }
        }

        for (auto it = _maneuver_gizmo_instances.begin(); it != _maneuver_gizmo_instances.end();)
        {
            if (desired.find(*it) != desired.end())
            {
                ++it;
                continue;
            }

            if (ctx.api)
            {
                (void) ctx.api->remove_mesh_instance(*it);
            }
            if (picking)
            {
                picking->clear_owner_picks(RenderObject::OwnerType::MeshInstance, *it);
            }
            it = _maneuver_gizmo_instances.erase(it);
        }
    }

    void GameplayState::update_maneuver_gizmo_interaction(GameStateContext &ctx)
    {
        if (!_maneuver_nodes_enabled || _maneuver_state.nodes.empty())
        {
            _maneuver_gizmo_interaction = {};
            return;
        }

        if (!ctx.input)
        {
            return;
        }

        auto axis_from_suffix = [](const std::string &suffix) -> ManeuverHandleAxis {
            if (suffix == "hub") return ManeuverHandleAxis::Hub;
            if (suffix == "prograde_pos") return ManeuverHandleAxis::ProgradePos;
            if (suffix == "prograde_neg") return ManeuverHandleAxis::ProgradeNeg;
            if (suffix == "radial_pos") return ManeuverHandleAxis::RadialPos;
            if (suffix == "radial_neg") return ManeuverHandleAxis::RadialNeg;
            if (suffix == "normal_pos") return ManeuverHandleAxis::NormalPos;
            if (suffix == "normal_neg") return ManeuverHandleAxis::NormalNeg;
            return ManeuverHandleAxis::None;
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

        ParsedManeuverInstance hover{};
        if (ctx.renderer)
        {
            if (const PickingSystem *picking = ctx.renderer->picking())
            {
                const auto &pick = picking->hover_pick();
                if (pick.valid && pick.kind == PickingSystem::PickInfo::Kind::SceneObject)
                {
                    hover = parse_maneuver_instance_name(pick.ownerName);
                }
            }
        }

        const ManeuverHandleAxis hover_axis = hover.valid ? axis_from_suffix(hover.suffix) : ManeuverHandleAxis::None;
        if (ctx.input->mouse_pressed(MouseButton::Left) && hover.valid)
        {
            _maneuver_state.selected_node_id = hover.node_id;
        }

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

            CameraRay ray{};
            if (!compute_camera_ray(ctx, ctx.input->mouse_position(), ray))
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

        switch (_maneuver_gizmo_interaction.state)
        {
            case ManeuverGizmoInteraction::State::Idle:
            {
                if (hover_axis != ManeuverHandleAxis::None)
                {
                    _maneuver_gizmo_interaction.state = ManeuverGizmoInteraction::State::HoverAxis;
                    _maneuver_gizmo_interaction.node_id = hover.node_id;
                    _maneuver_gizmo_interaction.axis = hover_axis;
                }

                if (ctx.input->mouse_pressed(MouseButton::Left) &&
                    hover_axis != ManeuverHandleAxis::None &&
                    hover_axis != ManeuverHandleAxis::Hub)
                {
                    (void) begin_drag(hover.node_id, hover_axis);
                }
                break;
            }

            case ManeuverGizmoInteraction::State::HoverAxis:
            {
                if (hover_axis == ManeuverHandleAxis::None)
                {
                    _maneuver_gizmo_interaction = {};
                    break;
                }

                _maneuver_gizmo_interaction.node_id = hover.node_id;
                _maneuver_gizmo_interaction.axis = hover_axis;

                if (ctx.input->mouse_pressed(MouseButton::Left) &&
                    hover_axis != ManeuverHandleAxis::Hub)
                {
                    (void) begin_drag(hover.node_id, hover_axis);
                }
                break;
            }

            case ManeuverGizmoInteraction::State::DragAxis:
            {
                ManeuverNode *node = _maneuver_state.find_node(_maneuver_gizmo_interaction.node_id);
                if (!node || !node->gizmo_valid)
                {
                    _maneuver_gizmo_interaction = {};
                    break;
                }

                if (!ctx.input->mouse_down(MouseButton::Left))
                {
                    const bool changed = _maneuver_gizmo_interaction.applied_delta;
                    if (hover_axis != ManeuverHandleAxis::None)
                    {
                        _maneuver_gizmo_interaction.state = ManeuverGizmoInteraction::State::HoverAxis;
                        _maneuver_gizmo_interaction.node_id = hover.node_id;
                        _maneuver_gizmo_interaction.axis = hover_axis;
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
                    break;
                }

                glm::dvec3 axis_dir_world(0.0, 1.0, 0.0);
                int component = 1;
                double sign = 1.0;
                if (!resolve_axis(*node, _maneuver_gizmo_interaction.axis, axis_dir_world, component, sign))
                {
                    _maneuver_gizmo_interaction = {};
                    break;
                }

                CameraRay ray{};
                if (!compute_camera_ray(ctx, ctx.input->mouse_position(), ray))
                {
                    break;
                }

                double current_t = 0.0;
                const glm::dvec3 axis_origin_local = glm::dvec3(node->position_world - ray.camera_world);
                if (!closest_param_ray_line(ray.ray_origin_local, ray.ray_dir_local,
                                            axis_origin_local, axis_dir_world, current_t))
                {
                    break;
                }

                const double delta_axis_m = current_t - _maneuver_gizmo_interaction.start_axis_t_m;
                double scale = static_cast<double>(std::max(0.0001f, _maneuver_gizmo_style.drag_sensitivity_mps_per_m));
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

                if (!finite3(dv_new))
                {
                    break;
                }

                if (safe_length(dv_new - node->dv_rtn_mps) > 1.0e-7)
                {
                    node->dv_rtn_mps = dv_new;
                    node->total_dv_mps = safe_length(node->dv_rtn_mps);
                    _maneuver_gizmo_interaction.applied_delta = true;
                }
                break;
            }
        }
    }
