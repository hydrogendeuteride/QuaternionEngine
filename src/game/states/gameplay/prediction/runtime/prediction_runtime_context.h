#pragma once

#include "game/states/gameplay/gameplay_settings.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"
#include "game/states/gameplay/orbit_helpers.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <functional>

namespace Game
{
    struct PredictionRuntimeContext
    {
        const OrbitalScenario *orbital_scenario{nullptr};
        PredictionSelectionState selection{};
        PredictionFrameSelectionState frame_selection{};
        PredictionAnalysisSelectionState analysis_selection{};
        PredictionSamplingPolicy sampling_policy{};
        ManeuverPlanWindowSettings maneuver_plan_windows{};
        const ManeuverPlanState *maneuver_plan{nullptr};
        const ManeuverNodeEditPreview *maneuver_edit_preview{nullptr};

        bool maneuver_nodes_enabled{false};
        bool maneuver_edit_in_progress{false};
        bool maneuver_live_preview_available{false};
        uint64_t maneuver_plan_revision{0};
        uint64_t maneuver_plan_signature{0};
        uint64_t display_frame_revision{1};
        int active_maneuver_preview_anchor_node_id{-1};
        double current_sim_time_s{0.0};

        std::function<WorldVec3()> reference_body_world;
        std::function<bool(PredictionSubjectKey, WorldVec3 &, glm::dvec3 &, glm::vec3 &)> get_subject_world_state;
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
        std::function<const OrbitPredictionCache *()> player_effective_cache;
    };
} // namespace Game
