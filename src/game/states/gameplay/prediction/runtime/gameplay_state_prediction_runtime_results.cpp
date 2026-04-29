#include "game/states/gameplay/gameplay_state.h"

#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/states/gameplay/prediction/runtime/prediction_result_applier.h"

namespace Game
{
    namespace
    {
        PredictionTrackState *find_track_by_id(std::vector<PredictionTrackState> &tracks, const uint64_t track_id)
        {
            for (PredictionTrackState &track : tracks)
            {
                if (track.key.track_id() == track_id)
                {
                    return &track;
                }
            }
            return nullptr;
        }
    } // namespace

    void GameplayState::apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result)
    {
        PredictionTrackState *track = find_track_by_id(_prediction.tracks, result.track_id);
        if (!track)
        {
            return;
        }

        const bool current_plan_active = track->supports_maneuvers &&
                                         _maneuver_nodes_enabled &&
                                         !_maneuver_state.nodes.empty();
        const bool active_maneuver_edit =
                current_plan_active &&
                (PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state) ||
                 _maneuver_node_edit_preview.state != ManeuverNodeEditPreview::State::Idle) &&
                track->key == _prediction.selection.active_subject &&
                maneuver_live_preview_active(true);

        PredictionResultApplier::apply_derived_result(
                *track,
                std::move(result),
                PredictionDerivedResultApplyContext{
                        .current_maneuver_plan_revision = _maneuver_plan_revision,
                        .live_preview_active = maneuver_live_preview_active(true),
                        .current_plan_active = current_plan_active,
                        .current_plan_signature = current_plan_active ? current_maneuver_plan_signature() : 0u,
                        .active_maneuver_edit = active_maneuver_edit,
                });
    }
} // namespace Game
