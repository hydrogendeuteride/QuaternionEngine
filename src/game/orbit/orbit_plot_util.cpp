#include "game/orbit/orbit_plot_util.h"

#include <glm/geometric.hpp>

#include <cmath>

namespace Game::OrbitPlotUtil
{
    glm::dvec3 eval_segment_local_position(const orbitsim::TrajectorySegment &segment, const double t_s)
    {
        return orbitsim::trajectory_segment_position_at(segment, t_s);
    }

    double meters_per_px_at_world(const glm::dvec3 &camera_world,
                                   const double tan_half_fov,
                                   const double viewport_height_px,
                                   const WorldVec3 &point_world)
    {
        const double dist_m = glm::length(glm::dvec3(point_world) - camera_world);
        if (!std::isfinite(dist_m) || dist_m <= 1.0e-3 ||
            !std::isfinite(tan_half_fov) || tan_half_fov <= 1.0e-8 ||
            !std::isfinite(viewport_height_px) || viewport_height_px <= 1.0)
        {
            return 1.0;
        }

        const double mpp = (2.0 * tan_half_fov * dist_m) / viewport_height_px;
        if (!std::isfinite(mpp) || mpp <= 1.0e-6)
        {
            return 1.0;
        }
        return mpp;
    }
} // namespace Game::OrbitPlotUtil
