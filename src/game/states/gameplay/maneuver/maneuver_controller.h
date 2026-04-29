#pragma once

#include "game/states/gameplay/maneuver/maneuver_commands.h"
#include "game/states/gameplay/maneuver/maneuver_plan_model.h"

namespace Game
{
    class ManeuverPlanController
    {
    public:
        static ManeuverCommandResult apply(ManeuverPlanState &state, const ManeuverCommand &command);
    };
} // namespace Game
