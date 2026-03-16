#pragma once

#include "core/world.h"
#include "orbitsim/trajectory_segments.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace Game
{
    class OrbitRenderCurve
    {
    public:
        static constexpr uint32_t kInvalidNodeIndex = 0xffffffffu;

        struct Node
        {
            orbitsim::TrajectorySegment segment{};
            uint32_t left_child{kInvalidNodeIndex};
            uint32_t right_child{kInvalidNodeIndex};
            double max_error_m{0.0};

            [[nodiscard]] bool is_leaf() const
            {
                return left_child == kInvalidNodeIndex || right_child == kInvalidNodeIndex;
            }
        };

        struct SelectionContext
        {
            WorldVec3 reference_body_world{0.0, 0.0, 0.0};
            WorldVec3 align_delta_world{0.0, 0.0, 0.0};
            glm::dmat3 frame_to_world{1.0};
            glm::dvec3 camera_world{0.0, 0.0, 0.0};
            double tan_half_fov{0.0};
            double viewport_height_px{1.0};
            double error_px{0.75};
        };

        [[nodiscard]] bool empty() const
        {
            return _root_index == kInvalidNodeIndex || _nodes.empty();
        }

        [[nodiscard]] uint32_t root_index() const
        {
            return _root_index;
        }

        [[nodiscard]] std::span<const Node> nodes() const
        {
            return std::span<const Node>(_nodes.data(), _nodes.size());
        }

        void clear();

        static OrbitRenderCurve build(std::span<const orbitsim::TrajectorySegment> source_segments);

        static void select_segments(const OrbitRenderCurve &curve,
                                    const SelectionContext &ctx,
                                    double t_start_s,
                                    double t_end_s,
                                    std::vector<orbitsim::TrajectorySegment> &out_segments);

    private:
        std::vector<Node> _nodes{};
        uint32_t _root_index{kInvalidNodeIndex};
    };
} // namespace Game
