#pragma once

#include "game/orbit/orbit_prediction_service.h"
#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"
#include "game/states/gameplay/prediction/runtime/prediction_track_lifecycle.h"

#include <cstdint>

namespace Game
{
    enum class PredictionRuntimeResetMode : uint8_t
    {
        Dirty,
        Clean,
    };

    struct PredictionSolverRequestSubmittedEvent
    {
        uint64_t generation_id{0};
        OrbitPredictionService::SolveQuality solve_quality{OrbitPredictionService::SolveQuality::Full};
        bool request_has_maneuver_plan{false};
        uint64_t maneuver_plan_signature{0};
        bool preview_request_active{false};
        double now_s{0.0};
    };

    class PredictionLifecycleReducer
    {
    public:
        static void reset_runtime(PredictionTrackState &track, PredictionRuntimeResetMode mode);

        static void mark_dirty(PredictionTrackState &track);
        static void mark_dirty_for_preview(PredictionTrackState &track);

        static void mark_solver_request_submitted(PredictionTrackState &track,
                                                  const PredictionSolverRequestSubmittedEvent &event);
        static void mark_rebuild_attempt_finished(PredictionTrackState &track, bool requested);

        static void mark_derived_request_submitted(
                PredictionTrackState &track,
                const OrbitPredictionDerivedService::Request &request);

        static void mark_solver_result_accepted(
                PredictionTrackState &track,
                OrbitPredictionService::SolveQuality solve_quality,
                OrbitPredictionService::PublishStage publish_stage,
                const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot &lifecycle_before);

        static void mark_solver_result_rejected_for_rebuild(PredictionTrackState &track,
                                                            bool reset_derived_request = false,
                                                            bool clear_invalidated = true);

        static void mark_derived_result_completed(
                PredictionTrackState &track,
                bool completes_latest,
                const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot &lifecycle_before);

        static void mark_derived_result_rejected_for_rebuild(PredictionTrackState &track,
                                                             bool complete_derived_request = false);

        static void enter_preview_drag(PredictionTrackState &track, double now_s);
        static void mark_preview_request_submitted(PredictionTrackState &track, double now_s);
        static void mark_preview_streaming(PredictionTrackState &track);
        static void mark_preview_publish_accepted(
                PredictionTrackState &track,
                const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot &lifecycle_before,
                OrbitPredictionService::PublishStage publish_stage,
                bool live_preview_active_now);
        static void mark_preview_publish_rejected(PredictionTrackState &track);
        static void mark_final_publish_completed(PredictionTrackState &track, bool live_preview_active_now);
        static void await_full_refine(PredictionTrackState &track, double now_s);
        static void reset_preview(PredictionTrackState &track);

        static void reset_derived_request_state(PredictionTrackState &track);
        static void clear_pending_maneuver_requests(PredictionTrackState &track, bool mark_dirty);
        static void clear_maneuver_scoped_pending_requests(PredictionTrackState &track);
    };
} // namespace Game
