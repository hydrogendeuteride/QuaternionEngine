#include "game/states/gameplay/maneuver/maneuver_plan_model.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace
    {
        bool same_dv(const glm::dvec3 &a, const glm::dvec3 &b)
        {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }

        bool contains_id(const std::vector<int> &ids, const int candidate)
        {
            return std::find(ids.begin(), ids.end(), candidate) != ids.end();
        }
    } // namespace

    ManeuverPlanModel::ManeuverPlanModel(ManeuverPlanState &state)
        : _state(state)
    {
    }

    ManeuverPlanState &ManeuverPlanModel::state()
    {
        return _state;
    }

    const ManeuverPlanState &ManeuverPlanModel::state() const
    {
        return _state;
    }

    std::vector<ManeuverNode> &ManeuverPlanModel::nodes()
    {
        return _state.nodes;
    }

    const std::vector<ManeuverNode> &ManeuverPlanModel::nodes() const
    {
        return _state.nodes;
    }

    int ManeuverPlanModel::selected_node_id() const
    {
        return _state.selected_node_id;
    }

    int ManeuverPlanModel::next_node_id() const
    {
        return _state.next_node_id;
    }

    bool ManeuverPlanModel::empty() const
    {
        return _state.nodes.empty();
    }

    std::size_t ManeuverPlanModel::size() const
    {
        return _state.nodes.size();
    }

    ManeuverNode *ManeuverPlanModel::find_node(const int node_id)
    {
        return _state.find_node(node_id);
    }

    const ManeuverNode *ManeuverPlanModel::find_node(const int node_id) const
    {
        return _state.find_node(node_id);
    }

    int ManeuverPlanModel::find_node_index(const int node_id) const
    {
        for (std::size_t i = 0; i < _state.nodes.size(); ++i)
        {
            if (_state.nodes[i].id == node_id)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    int ManeuverPlanModel::add_node(ManeuverNode node, const bool select_added)
    {
        if (node.id < 0)
        {
            node.id = _state.next_node_id++;
        }
        else
        {
            _state.next_node_id = std::max(_state.next_node_id, node.id + 1);
        }

        const int node_id = node.id;
        _state.nodes.push_back(node);
        if (select_added)
        {
            _state.selected_node_id = node_id;
        }
        sort_by_time();
        return node_id;
    }

    bool ManeuverPlanModel::clear()
    {
        if (_state.nodes.empty() && _state.selected_node_id < 0)
        {
            return false;
        }

        _state.nodes.clear();
        _state.selected_node_id = -1;
        return true;
    }

    bool ManeuverPlanModel::select_node(const int node_id)
    {
        if (_state.selected_node_id == node_id)
        {
            return false;
        }
        if (node_id >= 0 && !_state.find_node(node_id))
        {
            return false;
        }

        _state.selected_node_id = node_id;
        return true;
    }

    bool ManeuverPlanModel::select_index(const int node_index)
    {
        if (node_index < 0 || node_index >= static_cast<int>(_state.nodes.size()))
        {
            return false;
        }
        return select_node(_state.nodes[static_cast<std::size_t>(node_index)].id);
    }

    bool ManeuverPlanModel::ensure_valid_selection()
    {
        if (_state.selected_node_id >= 0 && _state.find_node(_state.selected_node_id))
        {
            return false;
        }

        const int next_selection = _state.nodes.empty() ? -1 : _state.nodes.front().id;
        if (_state.selected_node_id == next_selection)
        {
            return false;
        }

        _state.selected_node_id = next_selection;
        return true;
    }

    bool ManeuverPlanModel::set_node_time(const int node_id, const double time_s)
    {
        ManeuverNode *node = _state.find_node(node_id);
        if (!node || !std::isfinite(time_s) || node->time_s == time_s)
        {
            return false;
        }

        node->time_s = time_s;
        return true;
    }

    bool ManeuverPlanModel::set_node_dv(const int node_id, const glm::dvec3 &dv_rtn_mps)
    {
        ManeuverNode *node = _state.find_node(node_id);
        if (!node || same_dv(node->dv_rtn_mps, dv_rtn_mps))
        {
            return false;
        }

        node->dv_rtn_mps = dv_rtn_mps;
        return true;
    }

    bool ManeuverPlanModel::set_node_primary_body(const int node_id,
                                                  const bool primary_body_auto,
                                                  const orbitsim::BodyId primary_body_id)
    {
        ManeuverNode *node = _state.find_node(node_id);
        if (!node)
        {
            return false;
        }

        if (node->primary_body_auto == primary_body_auto && node->primary_body_id == primary_body_id)
        {
            return false;
        }

        node->primary_body_auto = primary_body_auto;
        node->primary_body_id = primary_body_id;
        return true;
    }

    bool ManeuverPlanModel::apply_primary_body_to_all(const orbitsim::BodyId primary_body_id,
                                                      const bool primary_body_auto)
    {
        bool changed = false;
        for (ManeuverNode &node : _state.nodes)
        {
            if (node.primary_body_id == primary_body_id && node.primary_body_auto == primary_body_auto)
            {
                continue;
            }

            node.primary_body_id = primary_body_id;
            node.primary_body_auto = primary_body_auto;
            changed = true;
        }
        return changed;
    }

    void ManeuverPlanModel::sort_by_time()
    {
        _state.sort_by_time();
    }

    ManeuverNodeRemovalResult ManeuverPlanModel::remove_node(const int node_id, const int hint_index)
    {
        ManeuverNodeRemovalResult result{};
        const auto before_size = _state.nodes.size();
        const bool removed_selected = _state.selected_node_id == node_id;

        _state.nodes.erase(
                std::remove_if(_state.nodes.begin(),
                               _state.nodes.end(),
                               [&](const ManeuverNode &node) { return node.id == node_id; }),
                _state.nodes.end());
        result.removed = _state.nodes.size() != before_size;
        if (!result.removed)
        {
            return result;
        }

        result.removed_selected = removed_selected;
        result.removed_node_ids.push_back(node_id);
        if (removed_selected)
        {
            if (_state.nodes.empty())
            {
                _state.selected_node_id = -1;
            }
            else if (hint_index >= 0)
            {
                const int new_index = std::clamp(hint_index, 0, static_cast<int>(_state.nodes.size()) - 1);
                _state.selected_node_id = _state.nodes[static_cast<std::size_t>(new_index)].id;
            }
            else
            {
                _state.selected_node_id = _state.nodes.front().id;
            }
        }
        result.plan_empty = _state.nodes.empty();
        return result;
    }

    ManeuverNodeRemovalResult ManeuverPlanModel::remove_suffix(const int node_id, const int hint_index)
    {
        ManeuverNodeRemovalResult result{};
        const auto erase_begin = std::find_if(_state.nodes.begin(),
                                              _state.nodes.end(),
                                              [node_id](const ManeuverNode &node) { return node.id == node_id; });
        if (erase_begin == _state.nodes.end())
        {
            return result;
        }

        for (auto it = erase_begin; it != _state.nodes.end(); ++it)
        {
            result.removed_node_ids.push_back(it->id);
        }
        result.removed = !result.removed_node_ids.empty();
        result.removed_selected = contains_id(result.removed_node_ids, _state.selected_node_id);

        _state.nodes.erase(erase_begin, _state.nodes.end());
        if (result.removed_selected)
        {
            if (_state.nodes.empty())
            {
                _state.selected_node_id = -1;
            }
            else if (hint_index >= 0)
            {
                const int new_index = std::clamp(hint_index, 0, static_cast<int>(_state.nodes.size()) - 1);
                _state.selected_node_id = _state.nodes[static_cast<std::size_t>(new_index)].id;
            }
            else
            {
                _state.selected_node_id = _state.nodes.front().id;
            }
        }
        result.plan_empty = _state.nodes.empty();
        return result;
    }
} // namespace Game
