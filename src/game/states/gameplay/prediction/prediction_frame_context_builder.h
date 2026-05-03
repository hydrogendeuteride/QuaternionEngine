#pragma once

#include "game/states/gameplay/prediction/gameplay_prediction_context.h"
#include "game/states/gameplay/prediction/prediction_frame_resolver.h"
#include "game/states/gameplay/prediction/prediction_host_context.h"
#include "orbitsim/spacecraft_lookup.hpp"

#include <limits>
#include <vector>

namespace Game
{
    class PredictionFrameContextBuilder
    {
    public:
        explicit PredictionFrameContextBuilder(GameplayPredictionContext context);

        [[nodiscard]] const PredictionTrackState *player_prediction_track() const;
        [[nodiscard]] const OrbitPredictionCache *effective_prediction_cache(const PredictionTrackState *track) const;
        [[nodiscard]] const CelestialBodyInfo *find_celestial_body_info(orbitsim::BodyId body_id) const;
        [[nodiscard]] WorldVec3 prediction_body_world_position(
                orbitsim::BodyId body_id,
                const OrbitPredictionCache *cache = nullptr,
                double query_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        [[nodiscard]] WorldVec3 prediction_world_reference_body_world() const;
        [[nodiscard]] bool sample_prediction_inertial_state(
                const std::vector<orbitsim::TrajectorySample> &trajectory,
                double query_time_s,
                orbitsim::State &out_state) const;
        [[nodiscard]] orbitsim::SpacecraftStateLookup build_prediction_player_lookup() const;
        [[nodiscard]] PredictionFrameResolverContext build_prediction_frame_resolver_context() const;
        [[nodiscard]] orbitsim::TrajectoryFrameSpec resolve_prediction_display_frame_spec(
                const OrbitPredictionCache &cache,
                double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        [[nodiscard]] orbitsim::BodyId select_prediction_primary_body_id(
                const std::vector<orbitsim::MassiveBody> &bodies,
                const OrbitPredictionCache *cache,
                const orbitsim::Vec3 &query_pos_m,
                double query_time_s,
                orbitsim::BodyId preferred_body_id = orbitsim::kInvalidBodyId) const;
        [[nodiscard]] orbitsim::BodyId resolve_prediction_analysis_body_id(
                const OrbitPredictionCache &cache,
                PredictionSubjectKey key,
                double query_time_s,
                orbitsim::BodyId preferred_body_id = orbitsim::kInvalidBodyId) const;

    private:
        [[nodiscard]] PredictionSubjectKey player_subject_key() const;
        [[nodiscard]] const PredictionTrackState *find_track(PredictionSubjectKey key) const;
        [[nodiscard]] const PredictionTrackState *player_track() const;
        [[nodiscard]] const OrbitPredictionCache *effective_cache(const PredictionTrackState *track) const;

        GameplayPredictionContext _context;
    };
} // namespace Game
