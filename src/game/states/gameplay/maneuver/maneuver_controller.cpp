#include "game/states/gameplay/maneuver/maneuver_controller.h"

#include <utility>

namespace Game
{
    namespace
    {
        void mark_plan_changed(ManeuverCommandResult &result)
        {
            result.plan_changed = true;
            result.prediction_dirty = true;
        }
    } // namespace

    ManeuverCommandResult ManeuverPlanController::apply(ManeuverPlanState &state,
                                                        const ManeuverCommand &command)
    {
        ManeuverPlanModel model(state);
        ManeuverCommandResult result{};
        result.previous_selected_node_id = model.selected_node_id();
        result.selected_node_id = model.selected_node_id();

        switch (command.kind)
        {
            case ManeuverCommandKind::None:
                break;

            case ManeuverCommandKind::AddNode:
            {
                result.added_node_id = model.add_node(command.node, command.select_added);
                result.applied = true;
                mark_plan_changed(result);
                break;
            }

            case ManeuverCommandKind::ClearPlan:
                for (const ManeuverNode &node : model.nodes())
                {
                    result.removed_node_ids.push_back(node.id);
                }
                result.applied = model.clear();
                if (result.applied)
                {
                    result.nodes_removed = !result.removed_node_ids.empty();
                    result.plan_empty = true;
                    result.clear_prediction_artifacts = true;
                    mark_plan_changed(result);
                }
                break;

            case ManeuverCommandKind::SelectNode:
                result.applied = model.select_node(command.node_id);
                result.selection_changed = result.applied;
                break;

            case ManeuverCommandKind::SelectNodeByIndex:
                result.applied = model.select_index(command.node_index);
                result.selection_changed = result.applied;
                break;

            case ManeuverCommandKind::EnsureSelection:
                result.applied = model.ensure_valid_selection();
                result.selection_changed = result.applied;
                break;

            case ManeuverCommandKind::RemoveNode:
            {
                ManeuverNodeRemovalResult removal = model.remove_node(command.node_id, command.hint_index);
                result.applied = removal.removed;
                if (result.applied)
                {
                    result.nodes_removed = true;
                    result.plan_empty = removal.plan_empty;
                    result.clear_prediction_artifacts = removal.plan_empty;
                    result.removed_node_ids = std::move(removal.removed_node_ids);
                    result.selection_changed = removal.removed_selected;
                    mark_plan_changed(result);
                }
                break;
            }

            case ManeuverCommandKind::RemoveNodeSuffix:
            {
                ManeuverNodeRemovalResult removal = model.remove_suffix(command.node_id, command.hint_index);
                result.applied = removal.removed;
                if (result.applied)
                {
                    result.nodes_removed = true;
                    result.plan_empty = removal.plan_empty;
                    result.clear_prediction_artifacts = removal.plan_empty;
                    result.removed_node_ids = std::move(removal.removed_node_ids);
                    result.selection_changed = removal.removed_selected;
                    mark_plan_changed(result);
                }
                break;
            }

            case ManeuverCommandKind::SetNodeTime:
                result.applied = model.set_node_time(command.node_id, command.time_s);
                if (result.applied)
                {
                    mark_plan_changed(result);
                    result.prediction_dirty = !command.defer_prediction_dirty;
                }
                break;

            case ManeuverCommandKind::SetNodeDv:
                result.applied = model.set_node_dv(command.node_id, command.dv_rtn_mps);
                if (result.applied)
                {
                    mark_plan_changed(result);
                    result.prediction_dirty = !command.defer_prediction_dirty;
                }
                break;

            case ManeuverCommandKind::SetNodePrimaryBody:
                result.applied = model.set_node_primary_body(command.node_id,
                                                             command.primary_body_auto,
                                                             command.primary_body_id);
                if (result.applied)
                {
                    mark_plan_changed(result);
                }
                break;

            case ManeuverCommandKind::ApplyPrimaryBodyToAll:
                result.applied = model.apply_primary_body_to_all(command.primary_body_id,
                                                                 command.primary_body_auto);
                if (result.applied)
                {
                    mark_plan_changed(result);
                }
                break;

            case ManeuverCommandKind::SortByTime:
                model.sort_by_time();
                result.applied = true;
                break;

            case ManeuverCommandKind::MarkPlanDirty:
                result.applied = true;
                mark_plan_changed(result);
                break;
        }

        result.selected_node_id = model.selected_node_id();
        if (result.selected_node_id != result.previous_selected_node_id)
        {
            result.selection_changed = true;
        }
        return result;
    }
} // namespace Game
