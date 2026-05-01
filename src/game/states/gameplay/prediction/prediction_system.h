#pragma once

#include <cstdint>
#include <vector>

namespace Game
{
    class OrbitPredictionDerivedService;
    struct GameplayPredictionState;
    struct PredictionSubjectKey;
    struct PredictionTrackState;

    class PredictionSystem
    {
    public:
        explicit PredictionSystem(GameplayPredictionState &prediction)
            : _prediction(prediction)
        {
        }

        OrbitPredictionDerivedService &derived_service();
        const OrbitPredictionDerivedService &derived_service() const;

        void reset_solver_service();
        void reset_derived_service();
        void reset_services();

        bool any_visible_track_dirty(const std::vector<PredictionSubjectKey> &visible_subjects) const;
        void sync_visible_dirty_flag(const std::vector<PredictionSubjectKey> &visible_subjects);
        void mark_visible_tracks_dirty(const std::vector<PredictionSubjectKey> &visible_subjects);
        void invalidate_maneuver_plan_revision(uint64_t maneuver_plan_revision);
        void clear_maneuver_prediction_artifacts();
        void mark_maneuver_preview_dirty(PredictionTrackState &track);
        void await_maneuver_preview_full_refine(PredictionTrackState &track, double now_s);
        void clear_unapplied_maneuver_drag_preview(PredictionTrackState &track);
        void clear_maneuver_live_preview_state();

    private:
        GameplayPredictionState &_prediction;
    };
} // namespace Game
