#pragma once

#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include "orbitsim/game_sim.hpp"

#include <functional>
#include <limits>

namespace Game
{
    struct PredictionFrameControllerContext
    {
        OrbitPredictionDerivedService *derived_service{nullptr};
        PredictionSelectionState selection{};
        uint64_t display_frame_revision{1};
        orbitsim::GameSimulation::Config sim_config{};

        std::function<orbitsim::TrajectoryFrameSpec(const OrbitPredictionCache &, double)> resolve_display_frame_spec;
        std::function<orbitsim::BodyId(const OrbitPredictionCache &,
                                       PredictionSubjectKey,
                                       double,
                                       orbitsim::BodyId)> resolve_analysis_body_id;
        std::function<bool(PredictionSubjectKey, WorldVec3 &, glm::dvec3 &, glm::vec3 &)> get_subject_world_state;
        std::function<bool(PredictionSubjectKey)> subject_is_player;
        std::function<const PredictionTrackState *()> player_track;
        std::function<const OrbitPredictionCache *(const PredictionTrackState *)> effective_cache;
        std::function<double()> current_sim_time_s;
    };

    class PredictionFrameController
    {
    public:
        static void mark_derived_request_submitted(
                PredictionTrackState &track,
                const OrbitPredictionDerivedService::Request &request);

        static bool request_derived_refresh(
                const PredictionFrameControllerContext &context,
                PredictionTrackState &track,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());

        static bool has_current_derived_cache(
                const PredictionFrameControllerContext &context,
                const PredictionTrackState &track,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());

        static void refresh_derived_cache(
                const PredictionFrameControllerContext &context,
                PredictionTrackState &track,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());

        static void reset_track_derived_state(PredictionTrackState &track);
    };
} // namespace Game
