#pragma once

#include "core/world.h"
#include "game/states/gameplay/gameplay_settings.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace Game
{
    struct OrbitalScenario;

    struct PredictionSubjectDescriptor
    {
        PredictionSubjectKey key{};
        std::string label{};
        bool supports_maneuvers{false};
        bool is_celestial{false};
        glm::vec3 orbit_rgb{1.0f};
    };

    struct PredictionManeuverView
    {
        const ManeuverPlanState *plan{nullptr};
        const ManeuverNodeEditPreview *edit_preview{nullptr};
        const ManeuverGizmoInteraction *gizmo_interaction{nullptr};
        ManeuverPlanHorizonSettings plan_horizon{};
        ManeuverPlanWindowSettings plan_windows{};
        bool nodes_enabled{false};
        bool live_preview_active{false};
        bool edit_in_progress{false};
        uint64_t revision{0};
        uint64_t signature{0};
        int active_preview_anchor_node_id{-1};
    };

    struct PredictionHostContext
    {
        const OrbitalScenario *orbital_scenario{nullptr};
        std::vector<PredictionSubjectDescriptor> subjects{};
        PredictionSubjectKey player_subject{};
        PredictionManeuverView maneuver{};

        double current_sim_time_s{0.0};
        double last_sim_step_dt_s{0.0};
        float fixed_delta_time_s{0.0f};
        float interpolation_alpha{0.0f};
        float frame_delta_time_s{0.0f};
        bool debug_draw_enabled{false};

        std::function<WorldVec3()> reference_body_world;
        std::function<bool(PredictionSubjectKey, WorldVec3 &, glm::dvec3 &, glm::vec3 &)> get_subject_world_state;
        std::function<WorldVec3(PredictionSubjectKey, float)> render_subject_position_world;
        std::function<double(PredictionSubjectKey)> future_window_s;
        std::function<double(const PredictionTrackState &, double, bool)> required_window_s;
        std::function<double(const PredictionTrackState &, double, bool)> preview_exact_window_s;
        std::function<void(PredictionTrackState &, double, bool)> refresh_preview_anchor;
        std::function<bool(PredictionSubjectKey)> subject_is_player;
        std::function<bool(PredictionSubjectKey)> subject_thrust_applied_this_tick;
        std::function<orbitsim::BodyId(const ManeuverNode &, double)> resolve_maneuver_node_primary_body_id;
        std::function<orbitsim::TrajectoryFrameSpec(const OrbitPredictionCache &, double)> resolve_display_frame_spec;
        std::function<orbitsim::BodyId(const OrbitPredictionCache &,
                                       PredictionSubjectKey,
                                       double,
                                       orbitsim::BodyId)> resolve_analysis_body_id;
    };
} // namespace Game
