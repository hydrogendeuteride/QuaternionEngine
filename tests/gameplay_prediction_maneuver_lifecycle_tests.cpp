#include "gameplay_prediction_maneuver_test_common.h"

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
