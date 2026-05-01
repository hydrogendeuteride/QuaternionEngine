#pragma once

#include "game/states/gameplay/gameplay_settings.h"
#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <cstdint>
#include <vector>

namespace Game
{
    struct GameplayPredictionState
    {
        bool enabled{true};
        bool dirty{true};
        bool draw_full_orbit{true};
        bool draw_future_segment{true};
        bool draw_velocity_ray{false};
        float line_alpha_scale{1.0f};
        float line_overlay_boost{0.0f};
        double periodic_refresh_s{0.0};
        double thrust_refresh_s{0.1};
        PredictionSamplingPolicy sampling_policy{};

        OrbitPlotPerfStats orbit_plot_perf{};
        OrbitPredictionDrawConfig draw_config{};
        OrbitPredictionService service{};
        OrbitPredictionDerivedService derived_service{};
        std::vector<PredictionTrackState> tracks{};
        std::vector<PredictionGroup> groups{};
        PredictionSelectionState selection{};
        PredictionFrameSelectionState frame_selection{};
        uint64_t display_frame_revision{1};
        PredictionAnalysisSelectionState analysis_selection{};
    };
} // namespace Game
