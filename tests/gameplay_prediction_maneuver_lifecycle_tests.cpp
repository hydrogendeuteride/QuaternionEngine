#include "gameplay_prediction_maneuver_test_common.h"

#include "game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.h"

TEST(GameplayPredictionManeuverTests, LifecycleSnapshotClassifiesIdleTrack)
{
    Game::PredictionTrackState track{};
    track.dirty = false;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::Idle);
    EXPECT_FALSE(snapshot.visible_cache_valid);
    EXPECT_FALSE(snapshot.authoritative_cache_valid);
    EXPECT_TRUE(snapshot.needs_rebuild);
}

TEST(GameplayPredictionManeuverTests, LifecycleSnapshotClassifiesNeedsRebuildTrack)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(3u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.dirty = true;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::NeedsRebuild);
    EXPECT_TRUE(snapshot.visible_cache_valid);
    EXPECT_TRUE(snapshot.needs_rebuild);
}

TEST(GameplayPredictionManeuverTests, LifecycleSnapshotClassifiesPreviewDragStatesAsPending)
{
    Game::PredictionTrackState track{};
    track.preview_state = Game::PredictionPreviewRuntimeState::EnterDrag;

    auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::DragPreviewPending);

    track.preview_state = Game::PredictionPreviewRuntimeState::DragPreviewPending;
    snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::DragPreviewPending);
}

TEST(GameplayPredictionManeuverTests, LifecycleSnapshotClassifiesAwaitFullRefineTrack)
{
    Game::PredictionTrackState track{};
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.cache = make_prediction_cache(5u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = false;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::AwaitFullRefine);
    EXPECT_FALSE(snapshot.request_pending);
    EXPECT_FALSE(snapshot.derived_request_pending);
}

TEST(GameplayPredictionManeuverTests, LifecycleSnapshotClassifiesFullStreamingTrack)
{
    Game::PredictionTrackState track{};
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.cache = make_prediction_cache(9u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.dirty = false;
    track.request_pending = true;
    track.full_stream_overlay.chunk_assembly.valid = true;
    track.full_stream_overlay.chunk_assembly.generation_id = 9u;
    track.full_stream_overlay.chunk_assembly.chunks.push_back(make_chunk(1u, 9u, 5.0, 15.0, 7'050'000.0, 7'250'000.0));

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::FullStreaming);
    EXPECT_TRUE(snapshot.full_stream_overlay_active);
}

TEST(GameplayPredictionManeuverTests, LifecycleSnapshotClassifiesFinalDerivedPendingTrack)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(8u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = make_prediction_cache(7u, 0.0, 20.0, 7'000'000.0, 7'150'000.0);
    track.latest_requested_generation_id = 8u;
    track.latest_requested_authoritative_generation_id = 8u;
    track.dirty = false;
    track.derived_request_pending = true;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::FinalDerivedPending);
    EXPECT_TRUE(snapshot.awaiting_authoritative_publish);
    EXPECT_TRUE(snapshot.derived_request_pending);
}

TEST(GameplayPredictionManeuverTests, LifecycleSnapshotClassifiesStableTrack)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(4u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = false;
    track.latest_requested_generation_id = 4u;
    track.latest_requested_authoritative_generation_id = 4u;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::Stable);
    EXPECT_FALSE(snapshot.needs_rebuild);
    EXPECT_TRUE(snapshot.authoritative_cache_valid);
}

TEST(GameplayPredictionManeuverTests, LifecycleHelperDefersWhileFullSolveIsPending)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(6u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.request_pending = true;
    track.dirty = false;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::FullSolvePending);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_defer_solver_request(snapshot));
}

TEST(GameplayPredictionManeuverTests, LifecycleHelperStillDefersDuringPreviewDragWhileRequestIsPending)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(6u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.preview_state = Game::PredictionPreviewRuntimeState::DragPreviewPending;
    track.request_pending = true;
    track.dirty = false;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::DragPreviewPending);
    EXPECT_TRUE(snapshot.request_pending);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_defer_solver_request(snapshot));
}

TEST(GameplayPredictionManeuverTests, LifecycleHelperStillDefersDuringPreviewStreamingWhileDerivedPublishPending)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(8u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = make_prediction_cache(7u, 0.0, 20.0, 7'000'000.0, 7'150'000.0);
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    track.derived_request_pending = true;
    track.latest_requested_generation_id = 8u;
    track.latest_requested_authoritative_generation_id = 8u;
    track.dirty = false;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::PreviewStreaming);
    EXPECT_TRUE(snapshot.derived_request_pending);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_defer_solver_request(snapshot));
}

TEST(GameplayPredictionManeuverTests, LifecycleHelperDefersWhileFinalPublishIsPendingAndTrackIsClean)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(8u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = make_prediction_cache(7u, 0.0, 20.0, 7'000'000.0, 7'150'000.0);
    track.latest_requested_generation_id = 8u;
    track.latest_requested_authoritative_generation_id = 8u;
    track.derived_request_pending = true;
    track.dirty = false;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::FinalDerivedPending);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_defer_solver_request(snapshot));
}

TEST(GameplayPredictionManeuverTests, LifecycleHelperAllowsRebuildWhenFinalPublishIsPendingButTrackIsDirty)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(8u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = make_prediction_cache(7u, 0.0, 20.0, 7'000'000.0, 7'150'000.0);
    track.latest_requested_generation_id = 8u;
    track.latest_requested_authoritative_generation_id = 8u;
    track.derived_request_pending = true;
    track.dirty = true;

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::FinalDerivedPending);
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_should_defer_solver_request(snapshot));
}

TEST(GameplayPredictionManeuverTests, LifecycleHelperRecognizesPreviewTransitionStates)
{
    Game::PredictionTrackState track{};
    track.dirty = false;

    auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_can_enter_preview_drag(snapshot));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_can_transition_to_await_full_refine(snapshot));

    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_can_enter_preview_drag(snapshot));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_preserves_await_full_refine(snapshot));

    track = {};
    track.cache = make_prediction_cache(4u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = false;
    track.preview_state = Game::PredictionPreviewRuntimeState::Idle;
    snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::Stable);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_can_enter_preview_drag(snapshot));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_preview_fallback_active(snapshot));

    track.preview_state = Game::PredictionPreviewRuntimeState::DragPreviewPending;
    snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_can_enter_preview_drag(snapshot));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_can_transition_to_await_full_refine(snapshot));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_preview_fallback_active(snapshot));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_preview_pick_clamp_active(snapshot));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_preview_overlay_draw_active(snapshot, true));

    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_overlay.chunk_assembly.valid = true;
    track.preview_overlay.chunk_assembly.chunks = {make_chunk(0u, 4u, 10.0, 20.0, 7'100'000.0, 7'200'000.0)};
    track.derived_request_pending = true;
    track.latest_requested_generation_id = 4u;
    track.latest_requested_authoritative_generation_id = 4u;
    snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_EQ(snapshot.state, Game::PredictionTrackLifecycleState::FinalDerivedPending);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_preserves_await_full_refine(snapshot));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_preview_pick_clamp_active(snapshot));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_preview_overlay_draw_active(snapshot, true));
}

TEST(GameplayPredictionManeuverTests, LifecycleHelperDescribesVisibleOverlayLayers)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(11u, 0.0, 40.0, 7'000'000.0, 7'400'000.0);
    track.authoritative_cache = track.cache;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.dirty = true;
    track.full_stream_overlay.chunk_assembly.valid = true;
    track.full_stream_overlay.chunk_assembly.generation_id = track.cache.identity.generation_id;
    track.full_stream_overlay.chunk_assembly.chunks = {
            make_chunk(0u, track.cache.identity.generation_id, 10.0, 20.0, 7'100'000.0, 7'200'000.0),
    };

    const auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    const auto layers = Game::PredictionRuntimeDetail::describe_prediction_overlay_layers(
            snapshot,
            true,
            false,
            track.preview_anchor.valid);
    const Game::PredictionChunkAssembly preview_snapshot =
            Game::PredictionRuntimeDetail::prediction_preview_overlay_snapshot_for_draw(track, layers);
    const Game::PredictionChunkAssembly full_stream_snapshot =
            Game::PredictionRuntimeDetail::prediction_full_stream_overlay_snapshot_for_draw(
                    track,
                    track.cache.identity,
                    track.cache.display,
                    layers);

    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_planned_preview_like(snapshot));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_keep_stale_planned_visible(
            snapshot,
            true,
            false));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_hold_maneuver_node_cache(snapshot, true));
    EXPECT_FALSE(layers.preview_fallback_active);
    EXPECT_FALSE(layers.preview_overlay_draw_active);
    EXPECT_TRUE(layers.full_stream_overlay_draw_active);
    EXPECT_FALSE(preview_snapshot.valid);
    EXPECT_TRUE(full_stream_snapshot.valid);
    EXPECT_EQ(full_stream_snapshot.chunks.size(), 1u);
}

TEST(GameplayPredictionManeuverTests, LifecycleHelperCoversPendingAndDirtyPromotionPolicies)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(6u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.dirty = false;

    auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_should_mark_invalidated_while_pending(snapshot));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_live_preview_drag_pending_override(snapshot));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_should_keep_dirty_for_followup(snapshot));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_should_promote_dirty_after_solver_publish(snapshot));

    track.request_pending = true;
    track.invalidated_while_pending = true;
    snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_mark_invalidated_while_pending(snapshot));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_live_preview_drag_pending_override(snapshot));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_keep_dirty_for_followup(snapshot));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_promote_dirty_after_solver_publish(snapshot));
}

TEST(GameplayPredictionManeuverTests, LifecycleHelperCoversPublishPoliciesAndNames)
{
    Game::PredictionTrackState track{};
    track.cache = make_prediction_cache(4u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.dirty = false;

    auto snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_rebuild_from_await_full_refine(snapshot));
    EXPECT_EQ(Game::PredictionRuntimeDetail::prediction_track_preview_state_after_preview_publish(
                      snapshot,
                      Game::OrbitPredictionService::PublishStage::PreviewFinalizing,
                      false),
              Game::PredictionPreviewRuntimeState::AwaitFullRefine);

    track.derived_request_pending = true;
    snapshot = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_should_rebuild_from_await_full_refine(snapshot));

    Game::PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot idle_snapshot{};
    idle_snapshot.preview_state = Game::PredictionPreviewRuntimeState::Idle;
    EXPECT_EQ(Game::PredictionRuntimeDetail::prediction_track_preview_state_after_preview_publish(
                      idle_snapshot,
                      Game::OrbitPredictionService::PublishStage::PreviewFinalizing,
                      true),
              Game::PredictionPreviewRuntimeState::PreviewStreaming);
    EXPECT_EQ(Game::PredictionRuntimeDetail::prediction_track_preview_state_after_preview_publish(
                      idle_snapshot,
                      Game::OrbitPredictionService::PublishStage::PreviewFinalizing,
                      false),
              Game::PredictionPreviewRuntimeState::Idle);

    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_is_preview_streaming_publish(
            Game::OrbitPredictionService::SolveQuality::FastPreview,
            Game::OrbitPredictionService::PublishStage::PreviewStreaming));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_is_preview_streaming_publish(
            Game::OrbitPredictionService::SolveQuality::Full,
            Game::OrbitPredictionService::PublishStage::PreviewStreaming));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_is_full_streaming_publish(
            Game::OrbitPredictionService::SolveQuality::Full,
            Game::OrbitPredictionService::PublishStage::FullStreaming));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_is_full_streaming_publish(
            Game::OrbitPredictionService::SolveQuality::FastPreview,
            Game::OrbitPredictionService::PublishStage::FullStreaming));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_keep_request_pending_after_solver_publish(
            Game::OrbitPredictionService::SolveQuality::Full,
            Game::OrbitPredictionService::PublishStage::FullStreaming));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_should_keep_request_pending_after_solver_publish(
            Game::OrbitPredictionService::SolveQuality::Full,
            Game::OrbitPredictionService::PublishStage::Final));
    EXPECT_TRUE(Game::PredictionRuntimeDetail::prediction_track_should_clear_preview_anchor_after_final_publish(false));
    EXPECT_FALSE(Game::PredictionRuntimeDetail::prediction_track_should_clear_preview_anchor_after_final_publish(true));
    EXPECT_STREQ(Game::PredictionRuntimeDetail::prediction_track_lifecycle_name(
                         Game::PredictionTrackLifecycleState::FullStreaming),
                 "FullStreaming");
}

TEST(GameplayPredictionManeuverTests, LifecycleReducerMarksCleanAndPendingTracksDirty)
{
    Game::PredictionTrackState track{};
    track.dirty = false;

    Game::PredictionLifecycleReducer::mark_dirty(track);

    EXPECT_TRUE(track.dirty);
    EXPECT_FALSE(track.invalidated_while_pending);

    track.dirty = false;
    track.request_pending = true;

    Game::PredictionLifecycleReducer::mark_dirty(track);

    EXPECT_FALSE(track.dirty);
    EXPECT_TRUE(track.invalidated_while_pending);
}

TEST(GameplayPredictionManeuverTests, LifecycleReducerRecordsSolverRequestMetadata)
{
    Game::PredictionTrackState track{};
    track.derived_request_pending = true;
    track.pending_derived_has_maneuver_plan = true;
    track.pending_derived_plan_signature = 11u;

    Game::PredictionLifecycleReducer::mark_solver_request_submitted(
            track,
            Game::PredictionSolverRequestSubmittedEvent{
                    42u,
                    Game::OrbitPredictionService::SolveQuality::FastPreview,
                    true,
                    99u,
                    true,
                    123.0,
            });

    EXPECT_EQ(track.latest_requested_generation_id, 42u);
    EXPECT_EQ(track.latest_requested_authoritative_generation_id, 0u);
    EXPECT_TRUE(track.request_pending);
    EXPECT_FALSE(track.derived_request_pending);
    EXPECT_EQ(track.pending_solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    EXPECT_TRUE(track.pending_solver_has_maneuver_plan);
    EXPECT_EQ(track.pending_solver_plan_signature, 99u);
    EXPECT_FALSE(track.pending_derived_has_maneuver_plan);
    EXPECT_EQ(track.pending_derived_plan_signature, 0u);
    EXPECT_FALSE(track.invalidated_while_pending);
    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::DragPreviewPending);
    EXPECT_DOUBLE_EQ(track.preview_last_request_at_s, 123.0);
}

TEST(GameplayPredictionManeuverTests, LifecycleReducerRecordsAndResetsDerivedRequestMetadata)
{
    Game::PredictionTrackState track{};
    Game::OrbitPredictionDerivedService::Request request{};
    request.generation_id = 7u;
    request.display_frame_key = 3u;
    request.display_frame_revision = 5u;
    request.analysis_body_id = 13;
    request.maneuver_plan_signature_valid = true;
    request.maneuver_plan_signature = 77u;
    request.solver_result.publish_stage = Game::OrbitPredictionService::PublishStage::FullStreaming;

    Game::PredictionLifecycleReducer::mark_derived_request_submitted(track, request);

    EXPECT_TRUE(track.derived_request_pending);
    EXPECT_EQ(track.latest_requested_derived_generation_id, 7u);
    EXPECT_EQ(track.latest_requested_derived_display_frame_key, 3u);
    EXPECT_EQ(track.latest_requested_derived_display_frame_revision, 5u);
    EXPECT_EQ(track.latest_requested_derived_analysis_body_id, 13);
    EXPECT_EQ(track.latest_requested_derived_publish_stage,
              Game::OrbitPredictionService::PublishStage::FullStreaming);
    EXPECT_TRUE(track.pending_derived_has_maneuver_plan);
    EXPECT_EQ(track.pending_derived_plan_signature, 77u);

    Game::PredictionLifecycleReducer::reset_derived_request_state(track);

    EXPECT_FALSE(track.derived_request_pending);
    EXPECT_EQ(track.latest_requested_derived_generation_id, 0u);
    EXPECT_EQ(track.latest_requested_derived_display_frame_key, 0u);
    EXPECT_EQ(track.latest_requested_derived_display_frame_revision, 0u);
    EXPECT_EQ(track.latest_requested_derived_analysis_body_id, orbitsim::kInvalidBodyId);
    EXPECT_EQ(track.latest_requested_derived_publish_stage,
              Game::OrbitPredictionService::PublishStage::Final);
}

TEST(GameplayPredictionManeuverTests, LifecycleReducerClearsPendingManeuverRequestsWithoutResettingDerivedIdentity)
{
    Game::PredictionTrackState track{};
    track.request_pending = true;
    track.derived_request_pending = true;
    track.pending_solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    track.pending_solver_has_maneuver_plan = true;
    track.pending_solver_plan_signature = 17u;
    track.pending_derived_has_maneuver_plan = true;
    track.pending_derived_plan_signature = 19u;
    track.invalidated_while_pending = true;
    track.latest_requested_derived_generation_id = 31u;
    track.latest_requested_derived_display_frame_key = 37u;
    track.latest_requested_derived_display_frame_revision = 41u;

    Game::PredictionLifecycleReducer::clear_pending_maneuver_requests(track, true);

    EXPECT_FALSE(track.request_pending);
    EXPECT_FALSE(track.derived_request_pending);
    EXPECT_EQ(track.pending_solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_FALSE(track.pending_solver_has_maneuver_plan);
    EXPECT_EQ(track.pending_solver_plan_signature, 0u);
    EXPECT_FALSE(track.pending_derived_has_maneuver_plan);
    EXPECT_EQ(track.pending_derived_plan_signature, 0u);
    EXPECT_FALSE(track.invalidated_while_pending);
    EXPECT_TRUE(track.dirty);
    EXPECT_EQ(track.latest_requested_derived_generation_id, 31u);
    EXPECT_EQ(track.latest_requested_derived_display_frame_key, 37u);
    EXPECT_EQ(track.latest_requested_derived_display_frame_revision, 41u);
}

TEST(GameplayPredictionManeuverTests, LifecycleReducerAppliesSolverPublishPendingPolicy)
{
    Game::PredictionTrackState track{};
    track.request_pending = true;
    track.pending_solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    track.dirty = false;
    const auto lifecycle = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    Game::PredictionLifecycleReducer::mark_solver_result_accepted(
            track,
            Game::OrbitPredictionService::SolveQuality::Full,
            Game::OrbitPredictionService::PublishStage::FullStreaming,
            lifecycle);

    EXPECT_TRUE(track.request_pending);
    EXPECT_EQ(track.pending_solve_quality, Game::OrbitPredictionService::SolveQuality::Full);

    Game::PredictionLifecycleReducer::mark_solver_result_accepted(
            track,
            Game::OrbitPredictionService::SolveQuality::Full,
            Game::OrbitPredictionService::PublishStage::Final,
            lifecycle);

    EXPECT_FALSE(track.request_pending);
    EXPECT_EQ(track.pending_solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
}

TEST(GameplayPredictionManeuverTests, LifecycleReducerRequeuesDirtyAfterRejectedResults)
{
    Game::PredictionTrackState track{};
    track.request_pending = true;
    track.derived_request_pending = true;
    track.invalidated_while_pending = true;
    track.dirty = false;

    Game::PredictionLifecycleReducer::mark_solver_result_rejected_for_rebuild(track, true);

    EXPECT_FALSE(track.request_pending);
    EXPECT_FALSE(track.derived_request_pending);
    EXPECT_FALSE(track.invalidated_while_pending);
    EXPECT_TRUE(track.dirty);
    EXPECT_EQ(track.pending_solve_quality, Game::OrbitPredictionService::SolveQuality::Full);

    track.dirty = false;
    track.derived_request_pending = true;

    Game::PredictionLifecycleReducer::mark_derived_result_rejected_for_rebuild(track, true);

    EXPECT_FALSE(track.derived_request_pending);
    EXPECT_FALSE(track.invalidated_while_pending);
    EXPECT_TRUE(track.dirty);
}

TEST(GameplayPredictionManeuverTests, LifecycleReducerCompletesDerivedResultWithFollowupDirty)
{
    Game::PredictionTrackState track{};
    track.dirty = false;
    track.invalidated_while_pending = true;
    track.derived_request_pending = true;
    const auto lifecycle = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);

    Game::PredictionLifecycleReducer::mark_derived_result_completed(track, true, lifecycle);

    EXPECT_FALSE(track.derived_request_pending);
    EXPECT_FALSE(track.invalidated_while_pending);
    EXPECT_TRUE(track.dirty);
}

TEST(GameplayPredictionManeuverTests, LifecycleReducerTransitionsPreviewStates)
{
    Game::PredictionTrackState track{};

    Game::PredictionLifecycleReducer::enter_preview_drag(track, 10.0);

    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::EnterDrag);
    EXPECT_DOUBLE_EQ(track.preview_entered_at_s, 10.0);

    Game::PredictionLifecycleReducer::mark_preview_request_submitted(track, 11.0);

    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::DragPreviewPending);
    EXPECT_DOUBLE_EQ(track.preview_last_request_at_s, 11.0);

    Game::PredictionLifecycleReducer::await_full_refine(track, 12.0);

    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::AwaitFullRefine);
    EXPECT_DOUBLE_EQ(track.preview_last_anchor_refresh_at_s, 12.0);

    track.preview_anchor.valid = true;
    Game::PredictionLifecycleReducer::reset_preview(track);

    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::Idle);
    EXPECT_FALSE(track.preview_anchor.valid);
}
