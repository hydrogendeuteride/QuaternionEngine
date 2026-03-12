#pragma once

#include "core/world.h"
#include "game/orbit/orbit_prediction_service.h"
#include "orbitsim/frame_spec.hpp"

#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace Game
{
    enum class PredictionSubjectKind : uint8_t
    {
        None = 0,
        Orbiter,
        Celestial
    };

    struct PredictionSubjectKey
    {
        PredictionSubjectKind kind{PredictionSubjectKind::None};
        uint32_t value{0};

        [[nodiscard]] bool valid() const
        {
            return kind != PredictionSubjectKind::None && value != 0;
        }

        [[nodiscard]] uint64_t track_id() const
        {
            return (static_cast<uint64_t>(kind) << 32u) | static_cast<uint64_t>(value);
        }

        friend bool operator==(const PredictionSubjectKey &a, const PredictionSubjectKey &b)
        {
            return a.kind == b.kind && a.value == b.value;
        }

        friend bool operator!=(const PredictionSubjectKey &a, const PredictionSubjectKey &b)
        {
            return !(a == b);
        }
    };

    // Centralized style and tuning knobs for orbit prediction drawing.
    struct OrbitPredictionDrawPalette
    {
        glm::vec4 orbit_full{0.75f, 0.20f, 0.92f, 0.22f};
        glm::vec4 orbit_future{0.75f, 0.20f, 0.92f, 0.80f};
        glm::vec4 orbit_planned{1.00f, 0.62f, 0.10f, 0.90f};
        glm::vec4 velocity_ray{1.0f, 0.35f, 0.1f, 1.0f};
    };

    inline glm::vec4 prediction_overlay_seed_color(const std::size_t index)
    {
        static constexpr std::array<glm::vec4, 16> kOverlayColors{{
                {0.20f, 0.72f, 1.00f, 1.0f},
                {1.00f, 0.42f, 0.20f, 1.0f},
                {0.40f, 0.92f, 0.46f, 1.0f},
                {1.00f, 0.86f, 0.22f, 1.0f},
                {0.96f, 0.32f, 0.70f, 1.0f},
                {0.72f, 0.58f, 1.00f, 1.0f},
                {0.18f, 0.90f, 0.82f, 1.0f},
                {1.00f, 0.60f, 0.84f, 1.0f},
                {0.58f, 0.96f, 0.24f, 1.0f},
                {1.00f, 0.56f, 0.12f, 1.0f},
                {0.34f, 0.82f, 0.78f, 1.0f},
                {0.86f, 0.44f, 1.00f, 1.0f},
                {0.98f, 0.74f, 0.30f, 1.0f},
                {0.30f, 0.62f, 1.00f, 1.0f},
                {1.00f, 0.36f, 0.46f, 1.0f},
                {0.52f, 0.88f, 0.98f, 1.0f},
        }};
        return kOverlayColors[index % kOverlayColors.size()];
    }

    // Draw-time tuning shared by orbit plotting, dashed plans, and picking.
    struct OrbitPredictionDrawConfig
    {
        OrbitPredictionDrawPalette palette{};
        std::size_t pick_planned_reserve_segments{2'000};
        double pick_planned_reserve_ratio{0.25};
        double node_time_tolerance_s{1.0e-3};
        double dashed_segment_on_px{14.0};
        double dashed_segment_off_px{9.0};
        int dash_max_chunks_per_segment{4096};
        bool draw_planned_as_dashed{true};
    };

    struct OrbitPredictionCache
    {
        using ManeuverNodePreview = OrbitPredictionService::ManeuverNodePreview;

        bool valid{false};
        double build_time_s{0.0};
        WorldVec3 build_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 build_vel_world{0.0, 0.0, 0.0};
        OrbitPredictionService::SharedCelestialEphemeris shared_ephemeris{};
        std::vector<orbitsim::MassiveBody> massive_bodies;

        // Canonical prediction output always stays in orbitsim's inertial frame.
        std::vector<orbitsim::TrajectorySample> trajectory_inertial;
        std::vector<orbitsim::TrajectorySample> trajectory_inertial_planned;
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial;
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial_planned;

        // Derived display-frame data rebuilt locally when the selected display frame changes.
        std::vector<orbitsim::TrajectorySample> trajectory_frame;
        std::vector<orbitsim::TrajectorySample> trajectory_frame_planned;
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_frame;
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_frame_planned;
        orbitsim::TrajectoryFrameSpec resolved_frame_spec{};
        bool resolved_frame_spec_valid{false};
        std::vector<ManeuverNodePreview> maneuver_previews;

        std::vector<float> altitude_km;
        std::vector<float> speed_kmps;

        // Classical orbital-element-like values for HUD.
        double semi_major_axis_m{0.0};
        double eccentricity{0.0};
        double orbital_period_s{0.0};
        double periapsis_alt_km{0.0};
        double apoapsis_alt_km{std::numeric_limits<double>::infinity()};
        orbitsim::BodyId metrics_body_id{orbitsim::kInvalidBodyId};
        bool metrics_valid{false};

        // Reset every cached prediction artifact so the next update rebuilds from scratch.
        void clear()
        {
            valid = false;
            build_time_s = 0.0;
            build_pos_world = WorldVec3(0.0, 0.0, 0.0);
            build_vel_world = glm::dvec3(0.0, 0.0, 0.0);
            shared_ephemeris.reset();
            massive_bodies.clear();
            trajectory_inertial.clear();
            trajectory_inertial_planned.clear();
            trajectory_segments_inertial.clear();
            trajectory_segments_inertial_planned.clear();
            trajectory_frame.clear();
            trajectory_frame_planned.clear();
            trajectory_segments_frame.clear();
            trajectory_segments_frame_planned.clear();
            resolved_frame_spec = {};
            resolved_frame_spec_valid = false;
            maneuver_previews.clear();
            altitude_km.clear();
            speed_kmps.clear();
            semi_major_axis_m = 0.0;
            eccentricity = 0.0;
            orbital_period_s = 0.0;
            periapsis_alt_km = 0.0;
            apoapsis_alt_km = std::numeric_limits<double>::infinity();
            metrics_body_id = orbitsim::kInvalidBodyId;
            metrics_valid = false;
        }
    };

    struct PredictionFrameOption
    {
        orbitsim::TrajectoryFrameSpec spec{};
        std::string label{};
    };

    struct PredictionFrameSelectionState
    {
        orbitsim::TrajectoryFrameSpec spec = orbitsim::TrajectoryFrameSpec::inertial();
        std::vector<PredictionFrameOption> options{};
        int selected_index{-1};

        void clear()
        {
            spec = orbitsim::TrajectoryFrameSpec::inertial();
            options.clear();
            selected_index = -1;
        }
    };

    enum class PredictionAnalysisMode : uint8_t
    {
        AutoPrimaryBCI = 0,
        FixedBodyBCI,
    };

    struct PredictionAnalysisSpec
    {
        PredictionAnalysisMode mode{PredictionAnalysisMode::AutoPrimaryBCI};
        orbitsim::BodyId fixed_body_id{orbitsim::kInvalidBodyId};
    };

    struct PredictionAnalysisOption
    {
        PredictionAnalysisSpec spec{};
        std::string label{};
    };

    struct PredictionAnalysisSelectionState
    {
        PredictionAnalysisSpec spec{};
        std::vector<PredictionAnalysisOption> options{};
        int selected_index{-1};

        void clear()
        {
            spec = {};
            options.clear();
            selected_index = -1;
        }
    };

    struct PredictionTrackState
    {
        PredictionSubjectKey key{};
        std::string label{};
        OrbitPredictionCache cache{};
        bool dirty{true};
        bool request_pending{false};
        bool invalidated_while_pending{false};
        bool supports_maneuvers{false};
        bool is_celestial{false};
        double solver_ms_last{0.0};

        void clear_runtime()
        {
            cache.clear();
            dirty = true;
            request_pending = false;
            invalidated_while_pending = false;
            solver_ms_last = 0.0;
        }
    };

    struct PredictionGroup
    {
        std::string name{};
        PredictionSubjectKey primary_subject{};
        std::vector<PredictionSubjectKey> members{};
    };

    struct PredictionSelectionState
    {
        PredictionSubjectKey active_subject{};
        std::vector<PredictionSubjectKey> overlay_subjects{};
        int selected_group_index{-1};

        void clear()
        {
            active_subject = {};
            overlay_subjects.clear();
            selected_group_index = -1;
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
