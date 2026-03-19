#pragma once

#include "core/world.h"
#include "orbitsim/trajectory_segments.hpp"

#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>

#include <cstdint>
#include <cstddef>
#include <span>
#include <vector>

namespace Game
{
    class OrbitRenderCurve
    {
    public:
        static constexpr uint32_t kInvalidNodeIndex = 0xffffffffu;

        struct CameraContext
        {
            glm::dvec3 camera_world{0.0, 0.0, 0.0};
            double tan_half_fov{0.0};
            double viewport_height_px{1.0};
        };

        struct RenderSettings
        {
            double error_px{0.75};
            std::size_t max_segments{4000};
        };

        struct RenderSegment
        {
            WorldVec3 a_world{0.0, 0.0, 0.0};
            WorldVec3 b_world{0.0, 0.0, 0.0};
            double t0_s{0.0};
            double t1_s{0.0};
        };

        struct RenderResult
        {
            std::vector<RenderSegment> segments{};
            bool cap_hit{false};
        };

        struct FrustumContext
        {
            bool valid{false};
            glm::mat4 viewproj{1.0f};
            WorldVec3 origin_world{0.0, 0.0, 0.0};
        };

        struct PickSettings
        {
            std::size_t max_segments{8000};
            double frustum_margin_ratio{0.05};
        };

        struct PickSegment
        {
            WorldVec3 a_world{0.0, 0.0, 0.0};
            WorldVec3 b_world{0.0, 0.0, 0.0};
            double t0_s{0.0};
            double t1_s{0.0};
        };

        struct PickResult
        {
            std::vector<PickSegment> segments{};
            std::size_t segments_before_cull{0};
            std::size_t segments_after_cull{0};
            bool cap_hit{false};
        };

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
            std::span<const double> anchor_times_s{};
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

        static RenderResult build_render_lod(
                std::span<const orbitsim::TrajectorySegment> segments_bci,
                const WorldVec3 &reference_body_world,
                const WorldVec3 &align_delta_world,
                const CameraContext &camera,
                const RenderSettings &settings,
                double t_start_s,
                double t_end_s,
                const FrustumContext &frustum);

        static RenderResult build_render_lod(
                const OrbitRenderCurve &curve,
                const SelectionContext &ctx,
                const FrustumContext &frustum,
                std::size_t max_segments,
                double t_start_s,
                double t_end_s);

        static PickResult build_pick_lod(
                std::span<const orbitsim::TrajectorySegment> segments_bci,
                const WorldVec3 &reference_body_world,
                const WorldVec3 &align_delta_world,
                const FrustumContext &frustum,
                const PickSettings &settings,
                double t_start_s,
                double t_end_s,
                std::span<const double> anchor_times_s = {});

        static PickResult build_pick_lod(
                const OrbitRenderCurve &curve,
                const SelectionContext &ctx,
                const FrustumContext &frustum,
                const PickSettings &settings,
                double t_start_s,
                double t_end_s);

    private:
        static void select_segments(const OrbitRenderCurve &curve,
                                    const SelectionContext &ctx,
                                    double t_start_s,
                                    double t_end_s,
                                    std::vector<orbitsim::TrajectorySegment> &out_segments);

        std::vector<Node> _nodes{};
        uint32_t _root_index{kInvalidNodeIndex};
    };
} // namespace Game
