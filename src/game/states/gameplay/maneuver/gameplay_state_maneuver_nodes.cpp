#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"

namespace Game
{
    double GameplayState::current_sim_time_s() const
    {
        return _orbit.scenario_owner() ? _orbit.scenario_owner()->sim.time_s() : _fixed_time_s;
    }

    void GameplayState::begin_maneuver_node_dv_edit_preview(const int node_id)
    {
        if (_maneuver.begin_dv_edit_preview(node_id))
        {
            GameplayPredictionAdapter prediction(*this);
            if (PredictionTrackState *track = prediction.active_prediction_track())
            {
                prediction.refresh_prediction_preview_anchor(*track, current_sim_time_s(), true);
            }
        }
    }

    void GameplayState::update_maneuver_node_dv_edit_preview(const int node_id)
    {
        begin_maneuver_node_dv_edit_preview(node_id);
        if (!_maneuver.mark_edit_preview_changed(ManeuverNodeEditPreview::State::EditingDv, node_id))
        {
            return;
        }

        GameplayPredictionAdapter prediction(*this);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            _prediction->mark_maneuver_preview_dirty(*track);
            prediction.sync_prediction_dirty_flag();
        }
    }

    void GameplayState::finish_maneuver_node_dv_edit_preview(const bool changed)
    {
        const bool preview_changed =
                _maneuver.finish_edit_preview(ManeuverNodeEditPreview::State::EditingDv, changed);

        if (!preview_changed)
        {
            return;
        }

        GameplayPredictionAdapter prediction(*this);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            _prediction->await_maneuver_preview_full_refine(*track, current_sim_time_s());
        }
        (void) apply_maneuver_command(ManeuverCommand::mark_plan_dirty());
    }

    void GameplayState::begin_maneuver_node_time_edit_preview(const int node_id,
                                                              const double previous_time_s)
    {
        if (_maneuver.begin_time_edit_preview(node_id, previous_time_s))
        {
            GameplayPredictionAdapter prediction(*this);
            if (PredictionTrackState *track = prediction.active_prediction_track())
            {
                prediction.refresh_prediction_preview_anchor(*track, current_sim_time_s(), true);
            }
        }
    }

    void GameplayState::update_maneuver_node_time_edit_preview(const int node_id,
                                                               const double previous_time_s)
    {
        begin_maneuver_node_time_edit_preview(node_id, previous_time_s);
        if (!_maneuver.mark_edit_preview_changed(ManeuverNodeEditPreview::State::EditingTime, node_id))
        {
            return;
        }

        GameplayPredictionAdapter prediction(*this);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            _prediction->mark_maneuver_preview_dirty(*track);
            prediction.sync_prediction_dirty_flag();
        }
    }

    void GameplayState::finish_maneuver_node_time_edit_preview(const bool changed)
    {
        const bool preview_changed =
                _maneuver.finish_edit_preview(ManeuverNodeEditPreview::State::EditingTime, changed);

        if (!preview_changed)
        {
            return;
        }

        GameplayPredictionAdapter prediction(*this);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            _prediction->await_maneuver_preview_full_refine(*track, current_sim_time_s());
        }
        (void) apply_maneuver_command(ManeuverCommand::mark_plan_dirty());
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
