#pragma once

#include "game/states/gameplay/prediction/gameplay_prediction_context.h"
#include "game/states/gameplay/prediction/prediction_host_context.h"
#include "game/states/gameplay/prediction/runtime/prediction_window_policy.h"

#include <cstdint>
#include <limits>

namespace Game
{
    class PredictionWindowContextBuilder
    {
    public:
        explicit PredictionWindowContextBuilder(GameplayPredictionContext context);

        [[nodiscard]] double future_window_s(PredictionSubjectKey key) const;
        [[nodiscard]] double maneuver_plan_horizon_s() const;
        [[nodiscard]] uint64_t current_maneuver_plan_signature() const;
        [[nodiscard]] double authored_plan_request_window_s(double now_s) const;
        [[nodiscard]] PredictionTimeContext build_time_context(
                PredictionSubjectKey key,
                double sim_now_s,
                double trajectory_t0_s = std::numeric_limits<double>::quiet_NaN(),
                double trajectory_t1_s = std::numeric_limits<double>::quiet_NaN()) const;
        [[nodiscard]] PredictionWindowPolicyResult resolve_policy(
                const PredictionTrackState *track,
                const PredictionTimeContext &time_ctx,
                bool with_maneuvers) const;
        [[nodiscard]] double required_window_s(const PredictionTrackState &track,
                                               double now_s,
                                               bool with_maneuvers) const;
        [[nodiscard]] double required_window_s(PredictionSubjectKey key,
                                               double now_s,
                                               bool with_maneuvers) const;
        [[nodiscard]] double display_window_s(PredictionSubjectKey key,
                                              double now_s,
                                              bool with_maneuvers) const;
        [[nodiscard]] double preview_exact_window_s(const PredictionTrackState &track,
                                                    double now_s,
                                                    bool with_maneuvers) const;
        [[nodiscard]] double planned_exact_window_s(const PredictionTrackState &track,
                                                    double now_s,
                                                    bool with_maneuvers) const;
        void refresh_preview_anchor(PredictionTrackState &track, double now_s, bool with_maneuvers) const;

    private:
        [[nodiscard]] const PredictionTrackState *find_track(PredictionSubjectKey key) const;

        GameplayPredictionContext _context;
    };
} // namespace Game
