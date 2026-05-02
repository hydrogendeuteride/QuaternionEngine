#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"

namespace Game
{
    double GameplayState::current_sim_time_s() const
    {
        return _orbit.scenario_owner() ? _orbit.scenario_owner()->sim.time_s() : _fixed_time_s;
    }

    ManeuverCommandResult GameplayState::apply_maneuver_command(const ManeuverCommand &command)
    {
        ManeuverCommandResult result = _maneuver.apply_command(command);
        if (!result.applied)
        {
            return result;
        }

        if (result.clear_prediction_artifacts)
        {
            GameplayPredictionAdapter(*this).clear_maneuver_prediction_artifacts();
        }
        if (result.prediction_dirty)
        {
            GameplayPredictionAdapter(*this).mark_maneuver_plan_dirty();
        }

        return result;
    }
} // namespace Game
