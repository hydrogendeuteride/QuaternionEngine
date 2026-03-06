#pragma once

#include "core/world.h"
#include "game/orbit/orbit_prediction_service.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <limits>
#include <vector>

namespace Game
{
    struct OrbitPredictionCache
    {
        using ManeuverNodePreview = OrbitPredictionService::ManeuverNodePreview;

        bool valid{false};
        double build_time_s{0.0};
        WorldVec3 build_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 build_vel_world{0.0, 0.0, 0.0};

        // BCI = body-centered inertial (relative to the current reference body)
        std::vector<orbitsim::TrajectorySample> trajectory_bci;
        std::vector<orbitsim::TrajectorySample> trajectory_bci_planned;
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_bci;
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_bci_planned;
        std::vector<ManeuverNodePreview> maneuver_previews;

        // Cached world-space polyline refreshed from trajectory_bci each frame.
        std::vector<WorldVec3> points_world;
        std::vector<WorldVec3> points_world_planned;
        std::vector<float> altitude_km;
        std::vector<float> speed_kmps;

        // Classical orbital-element-like values for HUD.
        double semi_major_axis_m{0.0};
        double eccentricity{0.0};
        double orbital_period_s{0.0};
        double periapsis_alt_km{0.0};
        double apoapsis_alt_km{std::numeric_limits<double>::infinity()};

        void clear()
        {
            valid = false;
            build_time_s = 0.0;
            build_pos_world = WorldVec3(0.0, 0.0, 0.0);
            build_vel_world = glm::dvec3(0.0, 0.0, 0.0);
            trajectory_bci.clear();
            trajectory_bci_planned.clear();
            trajectory_segments_bci.clear();
            trajectory_segments_bci_planned.clear();
            maneuver_previews.clear();
            points_world.clear();
            points_world_planned.clear();
            altitude_km.clear();
            speed_kmps.clear();
            semi_major_axis_m = 0.0;
            eccentricity = 0.0;
            orbital_period_s = 0.0;
            periapsis_alt_km = 0.0;
            apoapsis_alt_km = std::numeric_limits<double>::infinity();
        }
    };

    struct OrbitPlotPerfStats
    {
        double solver_ms_last{0.0};
        double render_lod_ms_last{0.0};
        double pick_lod_ms_last{0.0};

        uint32_t solver_segments_base{0};
        uint32_t solver_segments_planned{0};
        uint32_t pick_segments_before_cull{0};
        uint32_t pick_segments{0};

        bool render_cap_hit_last_frame{false};
        bool pick_cap_hit_last_frame{false};
        uint64_t render_cap_hits_total{0};
        uint64_t pick_cap_hits_total{0};
    };
} // namespace Game
