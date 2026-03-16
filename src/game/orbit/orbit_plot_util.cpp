#include "game/orbit/orbit_plot_util.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>

namespace Game::OrbitPlotUtil
{
    glm::dvec3 eval_segment_local_position(const orbitsim::TrajectorySegment &segment, const double t_s)
    {
        if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
        {
            return glm::dvec3(segment.start.position_m);
        }

        double u = (t_s - segment.t0_s) / segment.dt_s;
        if (!std::isfinite(u))
        {
            u = 0.0;
        }
        u = std::clamp(u, 0.0, 1.0);

        const double u2 = u * u;
        const double u3 = u2 * u;
        const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
        const double h10 = u3 - (2.0 * u2) + u;
        const double h01 = (-2.0 * u3) + (3.0 * u2);
        const double h11 = u3 - u2;

        const glm::dvec3 p0 = glm::dvec3(segment.start.position_m);
        const glm::dvec3 p1 = glm::dvec3(segment.end.position_m);
        const glm::dvec3 m0 = glm::dvec3(segment.start.velocity_mps) * segment.dt_s;
        const glm::dvec3 m1 = glm::dvec3(segment.end.velocity_mps) * segment.dt_s;
        return (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
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
