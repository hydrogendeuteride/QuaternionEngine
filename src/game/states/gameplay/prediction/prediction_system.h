#pragma once

#include "game/states/gameplay/gameplay_settings.h"
#include "game/states/gameplay/prediction/gameplay_prediction_state.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"
#include "game/states/gameplay/prediction/prediction_host_context.h"
#include "game/states/gameplay/prediction/runtime/prediction_runtime_context.h"

#include <cstdint>
#include <vector>

namespace Game
{
    class PredictionSystem
    {
    public:
        PredictionSystem() = default;

        [[nodiscard]] GameplayPredictionState &state();
        [[nodiscard]] const GameplayPredictionState &state() const;
        [[nodiscard]] OrbitPlotBudgetSettings &budget();
        [[nodiscard]] const OrbitPlotBudgetSettings &budget() const;

        OrbitPredictionDerivedService &derived_service();
        const OrbitPredictionDerivedService &derived_service() const;

        void reset_solver_service();
        void reset_derived_service();
        void reset_services();
        void reset_session_state();

        void sync_subjects(const std::vector<PredictionSubjectDescriptor> &subjects,
                           PredictionSubjectKey preferred_active_subject);
        void sync_subjects(const PredictionHostContext &host);
        [[nodiscard]] std::vector<PredictionSubjectKey> collect_visible_subjects() const;
        [[nodiscard]] PredictionTrackState *find_track(PredictionSubjectKey key);
        [[nodiscard]] const PredictionTrackState *find_track(PredictionSubjectKey key) const;
        [[nodiscard]] PredictionTrackState *active_track();
        [[nodiscard]] const PredictionTrackState *active_track() const;
        [[nodiscard]] PredictionTrackState *player_track(PredictionSubjectKey player_subject);
        [[nodiscard]] const PredictionTrackState *player_track(PredictionSubjectKey player_subject) const;
        [[nodiscard]] OrbitPredictionCache *effective_cache(PredictionTrackState *track);
        [[nodiscard]] const OrbitPredictionCache *effective_cache(const PredictionTrackState *track) const;
        [[nodiscard]] OrbitPredictionCache *player_cache(PredictionSubjectKey player_subject);
        [[nodiscard]] const OrbitPredictionCache *player_cache(PredictionSubjectKey player_subject) const;
        [[nodiscard]] OrbitPlotPerfStats &perf_stats();
        [[nodiscard]] const OrbitPlotPerfStats &perf_stats() const;

        bool any_visible_track_dirty(const std::vector<PredictionSubjectKey> &visible_subjects) const;
        void sync_visible_dirty_flag(const std::vector<PredictionSubjectKey> &visible_subjects);
        void mark_visible_tracks_dirty(const std::vector<PredictionSubjectKey> &visible_subjects);
        void invalidate_maneuver_plan_revision(uint64_t maneuver_plan_revision);
        void clear_maneuver_prediction_artifacts();
        void mark_maneuver_preview_dirty(PredictionTrackState &track);
        void await_maneuver_preview_full_refine(PredictionTrackState &track, double now_s);
        void clear_unapplied_maneuver_drag_preview(PredictionTrackState &track);
        void clear_maneuver_live_preview_state();

        [[nodiscard]] PredictionRuntimeContext build_runtime_context(const PredictionHostContext &host) const;
        void clear_runtime();
        void clear_visible_runtime(const std::vector<PredictionSubjectKey> &visible_subjects);
        [[nodiscard]] bool poll_completed_results(const PredictionHostContext &host);
        [[nodiscard]] bool apply_completed_solver_result(const PredictionHostContext &host,
                                                         OrbitPredictionService::Result result);
        [[nodiscard]] bool apply_completed_derived_result(const PredictionHostContext &host,
                                                          OrbitPredictionDerivedService::Result result);
        [[nodiscard]] bool should_rebuild_track(const PredictionHostContext &host,
                                                const PredictionTrackState &track,
                                                double now_s,
                                                float fixed_dt,
                                                bool thrusting,
                                                bool with_maneuvers) const;
        [[nodiscard]] bool request_orbiter_prediction_async(const PredictionHostContext &host,
                                                            PredictionTrackState &track,
                                                            const WorldVec3 &subject_pos_world,
                                                            const glm::dvec3 &subject_vel_world,
                                                            double now_s,
                                                            bool thrusting,
                                                            bool with_maneuvers,
                                                            bool *out_throttled = nullptr);
        [[nodiscard]] bool request_celestial_prediction_async(const PredictionHostContext &host,
                                                              PredictionTrackState &track,
                                                              double now_s);
        void update_orbiter_prediction_track(const PredictionHostContext &host,
                                             PredictionTrackState &track,
                                             double now_s,
                                             bool thrusting,
                                             bool with_maneuvers);
        void update_celestial_prediction_track(const PredictionHostContext &host,
                                               PredictionTrackState &track,
                                               double now_s);
        void update(const PredictionHostContext &host, float fixed_dt);

    private:
        GameplayPredictionState _state{};
        OrbitPlotBudgetSettings _budget{};
    };
} // namespace Game
