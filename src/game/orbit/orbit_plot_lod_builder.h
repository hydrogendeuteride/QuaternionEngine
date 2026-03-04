#pragma once

#include "core/world.h"
#include "orbitsim/trajectory_segments.hpp"

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

        static RenderResult build_render_lod(std::span<const orbitsim::TrajectorySegment> segments_bci,
                                             const WorldVec3 &reference_body_world,
                                             const WorldVec3 &align_delta_world,
                                             const CameraContext &camera,
                                             const RenderSettings &settings,
                                             double t_start_s,
                                             double t_end_s);
    };
} // namespace Game
