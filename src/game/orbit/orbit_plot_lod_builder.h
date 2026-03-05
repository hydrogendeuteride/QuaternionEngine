#pragma once

#include "core/world.h"
#include "orbitsim/trajectory_segments.hpp"

#include <glm/mat4x4.hpp>

#include <cstddef>
#include <span>
#include <vector>

namespace Game
{
    class OrbitPlotLodBuilder
    {
    public:
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

        static RenderResult build_render_lod(std::span<const orbitsim::TrajectorySegment> segments_bci,
                                             const WorldVec3 &reference_body_world,
                                             const WorldVec3 &align_delta_world,
                                             const CameraContext &camera,
                                             const RenderSettings &settings,
                                             double t_start_s,
                                             double t_end_s);

        static PickResult build_pick_lod(std::span<const orbitsim::TrajectorySegment> segments_bci,
                                         const WorldVec3 &reference_body_world,
                                         const WorldVec3 &align_delta_world,
                                         const FrustumContext &frustum,
                                         const PickSettings &settings,
                                         double t_start_s,
                                         double t_end_s,
                                         std::span<const double> anchor_times_s = {});
    };
} // namespace Game
