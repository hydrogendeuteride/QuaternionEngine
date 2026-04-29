#include "game/states/gameplay/maneuver/maneuver_commands.h"

namespace Game
{
    ManeuverCommand ManeuverCommand::add_node(ManeuverNode node, const bool select_added)
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::AddNode;
        command.node = node;
        command.select_added = select_added;
        return command;
    }

    ManeuverCommand ManeuverCommand::clear_plan()
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::ClearPlan;
        return command;
    }

    ManeuverCommand ManeuverCommand::select_node(const int node_id)
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::SelectNode;
        command.node_id = node_id;
        return command;
    }

    ManeuverCommand ManeuverCommand::select_node_by_index(const int node_index)
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::SelectNodeByIndex;
        command.node_index = node_index;
        return command;
    }

    ManeuverCommand ManeuverCommand::ensure_selection()
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::EnsureSelection;
        return command;
    }

    ManeuverCommand ManeuverCommand::remove_node(const int node_id, const int hint_index)
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::RemoveNode;
        command.node_id = node_id;
        command.hint_index = hint_index;
        return command;
    }

    ManeuverCommand ManeuverCommand::remove_node_suffix(const int node_id, const int hint_index)
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::RemoveNodeSuffix;
        command.node_id = node_id;
        command.hint_index = hint_index;
        return command;
    }

    ManeuverCommand ManeuverCommand::set_node_time(const int node_id,
                                                   const double time_s,
                                                   const bool defer_prediction_dirty)
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::SetNodeTime;
        command.node_id = node_id;
        command.time_s = time_s;
        command.defer_prediction_dirty = defer_prediction_dirty;
        return command;
    }

    ManeuverCommand ManeuverCommand::set_node_dv(const int node_id,
                                                 const glm::dvec3 &dv_rtn_mps,
                                                 const bool defer_prediction_dirty)
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::SetNodeDv;
        command.node_id = node_id;
        command.dv_rtn_mps = dv_rtn_mps;
        command.defer_prediction_dirty = defer_prediction_dirty;
        return command;
    }

    ManeuverCommand ManeuverCommand::set_node_primary_body(const int node_id,
                                                           const bool primary_body_auto,
                                                           const orbitsim::BodyId primary_body_id)
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::SetNodePrimaryBody;
        command.node_id = node_id;
        command.primary_body_auto = primary_body_auto;
        command.primary_body_id = primary_body_id;
        return command;
    }

    ManeuverCommand ManeuverCommand::apply_primary_body_to_all(const orbitsim::BodyId primary_body_id,
                                                               const bool primary_body_auto)
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::ApplyPrimaryBodyToAll;
        command.primary_body_auto = primary_body_auto;
        command.primary_body_id = primary_body_id;
        return command;
    }

    ManeuverCommand ManeuverCommand::sort_by_time()
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::SortByTime;
        return command;
    }

    ManeuverCommand ManeuverCommand::mark_plan_dirty()
    {
        ManeuverCommand command{};
        command.kind = ManeuverCommandKind::MarkPlanDirty;
        return command;
    }
} // namespace Game
