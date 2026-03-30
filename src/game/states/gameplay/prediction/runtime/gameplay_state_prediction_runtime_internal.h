#pragma once

#include "game/states/gameplay/gameplay_state.h"

#include <algorithm>
#include <chrono>

namespace Game::PredictionRuntimeDetail
{
    template<typename T>
    bool contains_key(const std::vector<T> &items, const T &needle)
    {
        return std::find(items.begin(), items.end(), needle) != items.end();
    }

    inline bool maneuver_drag_active(const ManeuverGizmoInteraction::State state)
    {
        return state == ManeuverGizmoInteraction::State::DragAxis;
    }

    inline double elapsed_ms(const PredictionDragDebugTelemetry::TimePoint &start_tp,
                             const PredictionDragDebugTelemetry::TimePoint &end_tp)
    {
        return std::chrono::duration<double, std::milli>(end_tp - start_tp).count();
    }

    inline void update_last_and_peak(double &last_value, double &peak_value, const double sample)
    {
        last_value = std::max(0.0, sample);
        peak_value = std::max(peak_value, last_value);
    }

    inline uint64_t visible_generation_id(const PredictionTrackState &track)
    {
        return track.cache.valid ? track.cache.generation_id : 0u;
    }

    inline bool latest_solver_generation_published(const PredictionTrackState &track)
    {
        return track.latest_requested_generation_id <= visible_generation_id(track);
    }
} // namespace Game::PredictionRuntimeDetail
