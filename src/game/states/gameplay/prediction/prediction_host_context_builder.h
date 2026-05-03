#pragma once

#include "game/states/gameplay/prediction/gameplay_prediction_context.h"
#include "game/states/gameplay/prediction/prediction_host_context.h"
#include "game/states/gameplay/prediction/prediction_subject_state_provider.h"

namespace Game
{
    struct GameStateContext;

    class PredictionHostContextBuilder
    {
    public:
        explicit PredictionHostContextBuilder(GameplayPredictionContext context);

        [[nodiscard]] PredictionSubjectStateProvider make_subject_state_provider() const;
        [[nodiscard]] PredictionHostContext build(const GameStateContext *ctx = nullptr) const;

    private:
        GameplayPredictionContext _context;
    };
} // namespace Game
