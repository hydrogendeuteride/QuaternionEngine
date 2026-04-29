#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"

#include <cstddef>
#include <vector>

namespace Game
{
    struct ManeuverNodeRemovalResult
    {
        bool removed{false};
        bool removed_selected{false};
        bool plan_empty{false};
        std::vector<int> removed_node_ids{};
    };

    class ManeuverPlanModel
    {
    public:
        explicit ManeuverPlanModel(ManeuverPlanState &state);

        [[nodiscard]] ManeuverPlanState &state();
        [[nodiscard]] const ManeuverPlanState &state() const;
        [[nodiscard]] std::vector<ManeuverNode> &nodes();
        [[nodiscard]] const std::vector<ManeuverNode> &nodes() const;
        [[nodiscard]] int selected_node_id() const;
        [[nodiscard]] int next_node_id() const;
        [[nodiscard]] bool empty() const;
        [[nodiscard]] std::size_t size() const;

        [[nodiscard]] ManeuverNode *find_node(int node_id);
        [[nodiscard]] const ManeuverNode *find_node(int node_id) const;
        [[nodiscard]] int find_node_index(int node_id) const;

        int add_node(ManeuverNode node, bool select_added);
        bool clear();
        bool select_node(int node_id);
        bool select_index(int node_index);
        bool ensure_valid_selection();
        bool set_node_time(int node_id, double time_s);
        bool set_node_dv(int node_id, const glm::dvec3 &dv_rtn_mps);
        bool set_node_primary_body(int node_id, bool primary_body_auto, orbitsim::BodyId primary_body_id);
        bool apply_primary_body_to_all(orbitsim::BodyId primary_body_id, bool primary_body_auto);
        void sort_by_time();
        ManeuverNodeRemovalResult remove_node(int node_id, int hint_index);
        ManeuverNodeRemovalResult remove_suffix(int node_id, int hint_index);

    private:
        ManeuverPlanState &_state;
    };
} // namespace Game
