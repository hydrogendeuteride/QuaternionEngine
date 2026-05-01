#pragma once

#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

namespace Game::PredictionRuntimeDetail
{
    struct PredictionTrackLifecycleSnapshot
    {
        PredictionTrackLifecycleState state{PredictionTrackLifecycleState::Idle};
        PredictionPreviewRuntimeState preview_state{PredictionPreviewRuntimeState::Idle};
        bool visible_cache_valid{false};
        bool authoritative_cache_valid{false};
        bool preview_overlay_active{false};
        bool full_stream_overlay_active{false};
        bool dirty{false};
        bool invalidated_while_pending{false};
        bool request_pending{false};
        bool derived_request_pending{false};
        bool awaiting_authoritative_publish{false};
        bool needs_rebuild{false};
    };
} // namespace Game::PredictionRuntimeDetail
