#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"

#include <vector>

namespace Game
{
    enum class ManeuverCommandKind
    {
        None = 0,
        AddNode,
        ClearPlan,
        SelectNode,
        SelectNodeByIndex,
        EnsureSelection,
        RemoveNode,
        RemoveNodeSuffix,
        SetNodeTime,
        SetNodeDv,
        SetNodePrimaryBody,
        ApplyPrimaryBodyToAll,
        SortByTime,
        MarkPlanDirty,
    };

    struct ManeuverCommand
    {
        ManeuverCommandKind kind{ManeuverCommandKind::None};
        ManeuverNode node{};
        int node_id{-1};
        int node_index{-1};
        int hint_index{-1};
        double time_s{0.0};
        glm::dvec3 dv_rtn_mps{0.0, 0.0, 0.0};
        bool primary_body_auto{true};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        bool select_added{true};
        bool defer_prediction_dirty{false};

        static ManeuverCommand add_node(ManeuverNode node, bool select_added = true);
        static ManeuverCommand clear_plan();
        static ManeuverCommand select_node(int node_id);
        static ManeuverCommand select_node_by_index(int node_index);
        static ManeuverCommand ensure_selection();
        static ManeuverCommand remove_node(int node_id, int hint_index = -1);
        static ManeuverCommand remove_node_suffix(int node_id, int hint_index = -1);
        static ManeuverCommand set_node_time(int node_id, double time_s, bool defer_prediction_dirty = false);
        static ManeuverCommand set_node_dv(int node_id,
                                           const glm::dvec3 &dv_rtn_mps,
                                           bool defer_prediction_dirty = false);
        static ManeuverCommand set_node_primary_body(int node_id,
                                                     bool primary_body_auto,
                                                     orbitsim::BodyId primary_body_id);
        static ManeuverCommand apply_primary_body_to_all(orbitsim::BodyId primary_body_id,
                                                         bool primary_body_auto);
        static ManeuverCommand sort_by_time();
        static ManeuverCommand mark_plan_dirty();
    };

    struct ManeuverCommandResult
    {
        bool applied{false};
        bool plan_changed{false};
        bool selection_changed{false};
        bool nodes_removed{false};
        bool plan_empty{false};
        bool clear_prediction_artifacts{false};
        bool prediction_dirty{false};
        int previous_selected_node_id{-1};
        int selected_node_id{-1};
        int added_node_id{-1};
        std::vector<int> removed_node_ids{};
    };
} // namespace Game
