#include "game/states/gameplay/maneuver/maneuver_system.h"

#include "game/states/gameplay/maneuver/maneuver_controller.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace Game
{
    namespace
    {
        bool contains_node_id(const std::vector<int> &ids, const int node_id)
        {
            return std::find(ids.begin(), ids.end(), node_id) != ids.end();
        }
    } // namespace

    void ManeuverRuntimeState::reset()
    {
        warp_to_time_active = false;
        warp_to_time_target_s = 0.0;
        warp_to_time_restore_level = 0;
        execute_node_armed = false;
        execute_node_id = -1;
    }

    void ManeuverRuntimeState::arm_warp_to_time(const double target_s, const int restore_level)
    {
        warp_to_time_active = true;
        warp_to_time_target_s = target_s;
        warp_to_time_restore_level = restore_level;
    }

    void ManeuverRuntimeState::disarm_warp_to_time()
    {
        warp_to_time_active = false;
        warp_to_time_target_s = 0.0;
        warp_to_time_restore_level = 0;
    }

    void ManeuverRuntimeState::arm_execute_node(const int node_id)
    {
        execute_node_armed = true;
        execute_node_id = node_id;
    }

    void ManeuverRuntimeState::disarm_execute_node()
    {
        execute_node_armed = false;
        execute_node_id = -1;
    }

    ManeuverSystemSettings &ManeuverSystem::settings()
    {
        return _settings;
    }

    const ManeuverSystemSettings &ManeuverSystem::settings() const
    {
        return _settings;
    }

    ManeuverRuntimeState &ManeuverSystem::runtime()
    {
        return _runtime;
    }

    const ManeuverRuntimeState &ManeuverSystem::runtime() const
    {
        return _runtime;
    }

    ManeuverPlanState &ManeuverSystem::plan()
    {
        return _plan;
    }

    const ManeuverPlanState &ManeuverSystem::plan() const
    {
        return _plan;
    }

    ManeuverGizmoInteraction &ManeuverSystem::gizmo_interaction()
    {
        return _gizmo_interaction;
    }

    const ManeuverGizmoInteraction &ManeuverSystem::gizmo_interaction() const
    {
        return _gizmo_interaction;
    }

    ManeuverNodeEditPreview &ManeuverSystem::edit_preview()
    {
        return _edit_preview;
    }

    const ManeuverNodeEditPreview &ManeuverSystem::edit_preview() const
    {
        return _edit_preview;
    }

    uint64_t &ManeuverSystem::revision()
    {
        return _revision;
    }

    uint64_t ManeuverSystem::revision() const
    {
        return _revision;
    }

    void ManeuverSystem::reset_session()
    {
        _runtime.reset();
        _plan.nodes.clear();
        _plan.selected_node_id = -1;
        _plan.next_node_id = 0;
        _gizmo_interaction = {};
        _edit_preview = {};
    }

    uint64_t ManeuverSystem::increment_revision()
    {
        return ++_revision;
    }

    void ManeuverSystem::set_revision(const uint64_t revision)
    {
        _revision = revision;
    }

    ManeuverCommandResult ManeuverSystem::apply_command(const ManeuverCommand &command)
    {
        ManeuverCommandResult result = ManeuverPlanController::apply(_plan, command);
        if (!result.applied)
        {
            return result;
        }

        if (result.nodes_removed)
        {
            if (contains_node_id(result.removed_node_ids, _gizmo_interaction.node_id))
            {
                _gizmo_interaction = {};
            }
            if (_runtime.execute_node_armed && contains_node_id(result.removed_node_ids, _runtime.execute_node_id))
            {
                _runtime.disarm_execute_node();
            }
            if (contains_node_id(result.removed_node_ids, _edit_preview.node_id))
            {
                _edit_preview = {};
            }
        }

        return result;
    }

    bool ManeuverSystem::live_preview_active(const bool with_maneuvers) const
    {
        if (!with_maneuvers || !_settings.live_preview_active)
        {
            return false;
        }

        if (_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            return _plan.find_node(active_preview_anchor_node_id()) != nullptr;
        }

        const bool node_edit_preview_active =
                _edit_preview.state == ManeuverNodeEditPreview::State::EditingDv ||
                _edit_preview.state == ManeuverNodeEditPreview::State::EditingTime;
        return node_edit_preview_active && _plan.find_node(_edit_preview.node_id) != nullptr;
    }

    int ManeuverSystem::active_preview_anchor_node_id() const
    {
        if (_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            if (_plan.find_node(_gizmo_interaction.node_id))
            {
                return _gizmo_interaction.node_id;
            }
            return _plan.selected_node_id;
        }

        if (_edit_preview.state == ManeuverNodeEditPreview::State::EditingDv ||
            _edit_preview.state == ManeuverNodeEditPreview::State::EditingTime)
        {
            return _edit_preview.node_id;
        }

        return -1;
    }

    uint64_t ManeuverSystem::plan_signature(const double plan_horizon_s) const
    {
        uint64_t seed = 0xcbf29ce484222325ULL;
        const auto mix = [&seed](const uint64_t value) {
            seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6u) + (seed >> 2u);
        };
        const auto quantized = [](const double value, const double scale) -> uint64_t {
            if (!std::isfinite(value))
            {
                return 0u;
            }
            return static_cast<uint64_t>(static_cast<int64_t>(std::llround(value * scale)));
        };

        mix(_settings.nodes_enabled ? 1u : 0u);
        mix(static_cast<uint64_t>(_plan.nodes.size()));
        mix(quantized(plan_horizon_s, 1000.0));
        mix(quantized(_settings.plan_windows.solve_margin_s, 1000.0));
        mix(quantized(_settings.plan_windows.preview_window_s, 1000.0));

        for (const ManeuverNode &node : _plan.nodes)
        {
            mix(static_cast<uint64_t>(node.id));
            mix(quantized(node.time_s, 1000.0));
            mix(quantized(node.dv_rtn_mps.x, 1000.0));
            mix(quantized(node.dv_rtn_mps.y, 1000.0));
            mix(quantized(node.dv_rtn_mps.z, 1000.0));
            mix(node.primary_body_auto ? 1u : 0u);
            mix(static_cast<uint64_t>(node.primary_body_auto ? orbitsim::kInvalidBodyId : node.primary_body_id));
        }

        return seed;
    }
} // namespace Game
