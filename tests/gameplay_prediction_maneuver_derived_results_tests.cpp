#include "gameplay_prediction_maneuver_test_common.h"

namespace
{
    void activate_live_maneuver_preview(Game::GameplayState &state,
                                        Game::PredictionTrackState &track,
                                        const int node_id,
                                        const double anchor_time_s)
    {
        track.supports_maneuvers = true;
        track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
        track.preview_anchor.valid = true;
        track.preview_anchor.anchor_node_id = node_id;
        track.preview_anchor.anchor_time_s = anchor_time_s;
        track.preview_anchor.visual_window_s = 60.0;
        track.preview_anchor.exact_window_s = 60.0;

        state._maneuver_node_edit_preview.state = Game::ManeuverNodeEditPreview::State::EditingDv;
        state._maneuver_node_edit_preview.node_id = node_id;
    }
} // namespace

TEST(GameplayPredictionManeuverTests, StaleDerivedResultDoesNotReplaceNewerDisplayedGeneration)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.identity.valid = true;
    track.cache.identity.generation_id = 3;
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    track.cache.display.trajectory_frame = track.cache.solver.trajectory_inertial;
    track.cache.display.trajectory_segments_frame = track.cache.solver.trajectory_segments_inertial;
    track.latest_requested_generation_id = 4;
    track.derived_request_pending = true;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 2;
    result.valid = true;
    result.cache.identity.valid = true;
    result.cache.identity.generation_id = 2;
    result.cache.solver.trajectory_inertial = {make_sample(0.0, 8'000'000.0), make_sample(60.0, 8'100'000.0)};
    result.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 60.0, 8'000'000.0, 8'100'000.0)};
    result.cache.display.trajectory_frame = result.cache.solver.trajectory_inertial;
    result.cache.display.trajectory_segments_frame = result.cache.solver.trajectory_segments_inertial;

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    EXPECT_EQ(state._prediction_tracks.front().cache.identity.generation_id, 3u);
    EXPECT_DOUBLE_EQ(state._prediction_tracks.front().cache.display.trajectory_frame.front().position_m.x, 7'000'000.0);
    EXPECT_TRUE(state._prediction_tracks.front().derived_request_pending);
}

TEST(GameplayPredictionManeuverTests, StaleManeuverPlanDerivedResultIsDroppedAndRebuildQueued)
{
    Game::GameplayState state{};
    state._maneuver_plan_revision = 3u;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.supports_maneuvers = true;
    track.cache = make_prediction_cache(7u, 0.0, 60.0, 7'000'000.0, 7'100'000.0);
    track.dirty = false;
    track.derived_request_pending = true;
    track.latest_requested_derived_generation_id = 8u;
    track.latest_requested_derived_display_frame_key = 1u;
    track.latest_requested_derived_display_frame_revision = 2u;
    track.latest_requested_derived_analysis_body_id = 1;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 8u;
    result.maneuver_plan_revision = 2u;
    result.display_frame_key = 1u;
    result.display_frame_revision = 2u;
    result.analysis_body_id = 1;
    result.valid = true;
    result.cache = make_prediction_cache(8u, 0.0, 60.0, 8'000'000.0, 8'100'000.0);

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    const Game::PredictionTrackState &updated_track = state._prediction_tracks.front();
    EXPECT_FALSE(updated_track.derived_request_pending);
    EXPECT_TRUE(updated_track.dirty);
    EXPECT_EQ(updated_track.cache.identity.generation_id, 7u);
    EXPECT_DOUBLE_EQ(updated_track.cache.display.trajectory_frame.front().position_m.x, 7'000'000.0);
}

TEST(GameplayPredictionManeuverTests, DerivedServiceInvalidatesStaleManeuverRevision)
{
    Game::OrbitPredictionDerivedService service{};
    constexpr uint64_t track_id = 77u;

    Game::OrbitPredictionDerivedService::Result completed{};
    completed.track_id = track_id;
    completed.generation_id = 12u;
    completed.maneuver_plan_revision = 2u;
    service._completed.push_back(completed);

    service._latest_requested_generation_by_track[track_id] = 12u;
    service.invalidate_maneuver_plan_revision(track_id, 3u);

    EXPECT_TRUE(service._completed.empty());
    EXPECT_FALSE(service.should_continue_job(track_id,
                                             12u,
                                             service._request_epoch,
                                             2u,
                                             Game::OrbitPredictionService::SolveQuality::Full));
    EXPECT_TRUE(service.should_continue_job(track_id,
                                            12u,
                                            service._request_epoch,
                                            3u,
                                            Game::OrbitPredictionService::SolveQuality::Full));
}

TEST(GameplayPredictionManeuverTests, ReusedBaseFrameDerivedResultPreservesBaseDiagnostics)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.identity.valid = true;
    track.cache.display.resolved_frame_spec_valid = true;
    track.cache.display.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::body_centered_inertial(1);
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    track.cache.display.trajectory_frame = track.cache.solver.trajectory_inertial;
    track.cache.display.trajectory_segments_frame = track.cache.solver.trajectory_segments_inertial;
    track.dirty = false;
    track.derived_diagnostics.frame_base.accepted_segments = 11;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 1;
    result.valid = true;
    result.base_frame_reused = true;
    result.diagnostics.frame_base.accepted_segments = 11;
    result.cache.identity.valid = true;
    result.cache.display.resolved_frame_spec_valid = true;
    result.cache.display.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::body_centered_inertial(1);
    result.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    result.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    result.cache.solver.trajectory_inertial_planned = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'200'000.0)};
    result.cache.solver.trajectory_segments_inertial_planned = {make_segment(0.0, 60.0, 7'000'000.0, 7'200'000.0)};

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    EXPECT_EQ(state._prediction_tracks.front().derived_diagnostics.frame_base.accepted_segments, 11u);
    EXPECT_EQ(state._prediction_tracks.front().derived_diagnostics.status, Game::PredictionDerivedStatus::Success);
    const auto lifecycle =
            Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(state._prediction_tracks.front());
    EXPECT_EQ(lifecycle.state, Game::PredictionTrackLifecycleState::Stable);
}

TEST(GameplayPredictionManeuverTests, FullStreamingDerivedResultKeepsFinalPending)
{
    Game::GameplayState state{};
    state._prediction_sampling_policy.orbiter_min_window_s = 5.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.cache = make_prediction_cache(8u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.dirty = false;
    track.latest_requested_generation_id = 9u;
    track.latest_requested_authoritative_generation_id = 9u;
    track.derived_request_pending = true;
    track.latest_requested_derived_generation_id = 9u;
    track.latest_requested_derived_display_frame_key = 1u;
    track.latest_requested_derived_display_frame_revision = 1u;
    track.latest_requested_derived_analysis_body_id = orbitsim::kInvalidBodyId;
    track.latest_requested_derived_publish_stage = Game::OrbitPredictionService::PublishStage::Final;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 9u;
    result.display_frame_key = 1u;
    result.display_frame_revision = 1u;
    result.analysis_body_id = orbitsim::kInvalidBodyId;
    result.valid = true;
    result.solve_quality = Game::OrbitPredictionService::SolveQuality::Full;
    result.publish_stage = Game::OrbitPredictionService::PublishStage::FullStreaming;
    result.cache = make_prediction_cache(9u, 0.0, 20.0, 7'050'000.0, 7'250'000.0);
    result.chunk_assembly.valid = true;
    result.chunk_assembly.generation_id = 9u;
    result.chunk_assembly.chunks = {
            make_chunk(0u, 9u, 0.0, 20.0, 7'050'000.0, 7'250'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    const Game::PredictionTrackState &updated_track = state._prediction_tracks.front();
    EXPECT_TRUE(updated_track.derived_request_pending);
    EXPECT_EQ(updated_track.cache.identity.generation_id, 9u);
    EXPECT_FALSE(state.should_rebuild_prediction_track(updated_track, 10.0, 0.016f, false, false));
    const auto lifecycle = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(updated_track);
    EXPECT_EQ(lifecycle.state, Game::PredictionTrackLifecycleState::FullStreaming);
}

TEST(GameplayPredictionManeuverTests, RefreshAllDerivedCachesClearsStaleChunkOverlaysBeforeRequeue)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.cache = make_prediction_cache(9u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = false;
    track.latest_requested_generation_id = 9u;
    track.latest_requested_authoritative_generation_id = 9u;
    track.preview_overlay.chunk_assembly.valid = true;
    track.preview_overlay.chunk_assembly.generation_id = 9u;
    track.preview_overlay.chunk_assembly.chunks = {
            make_chunk(0u, 9u, 10.0, 15.0, 7'050'000.0, 7'100'000.0),
    };
    track.full_stream_overlay.chunk_assembly.valid = true;
    track.full_stream_overlay.chunk_assembly.generation_id = 9u;
    track.full_stream_overlay.display_frame_key = 77u;
    track.full_stream_overlay.display_frame_revision = 3u;
    track.full_stream_overlay.chunk_assembly.chunks = {
            make_chunk(1u, 9u, 0.0, 20.0, 7'000'000.0, 7'250'000.0),
    };
    state._prediction_tracks.push_back(track);

    state.refresh_all_prediction_derived_caches();

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    const Game::PredictionTrackState &updated_track = state._prediction_tracks.front();
    EXPECT_FALSE(updated_track.preview_overlay.chunk_assembly.valid);
    EXPECT_TRUE(updated_track.preview_overlay.chunk_assembly.chunks.empty());
    EXPECT_FALSE(updated_track.full_stream_overlay.chunk_assembly.valid);
    EXPECT_TRUE(updated_track.full_stream_overlay.chunk_assembly.chunks.empty());
    EXPECT_EQ(updated_track.full_stream_overlay.display_frame_key, 0u);
    EXPECT_EQ(updated_track.full_stream_overlay.display_frame_revision, 0u);
    EXPECT_TRUE(updated_track.derived_request_pending);

    const auto lifecycle = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(updated_track);
    EXPECT_NE(lifecycle.state, Game::PredictionTrackLifecycleState::FullStreaming);
}

TEST(GameplayPredictionManeuverTests, DerivedRefreshWaitsForAuthoritativePublish)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache = make_prediction_cache(9u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = make_prediction_cache(8u, 0.0, 20.0, 7'000'000.0, 7'150'000.0);
    track.latest_requested_generation_id = 9u;
    track.latest_requested_authoritative_generation_id = 9u;

    EXPECT_FALSE(state.request_prediction_derived_refresh(track, 10.0));
    EXPECT_FALSE(track.derived_request_pending);
    EXPECT_EQ(track.latest_requested_derived_generation_id, 0u);
}

TEST(GameplayPredictionManeuverTests, PreviewDerivedResultsAccumulatePlannedChunkAssemblyAcrossStages)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.identity.valid = true;
    track.cache.identity.generation_id = 4;
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    track.cache.display.trajectory_frame = track.cache.solver.trajectory_inertial;
    track.cache.display.trajectory_segments_frame = track.cache.solver.trajectory_segments_inertial;
    track.cache.solver.trajectory_inertial_planned = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10.0, 7'100'000.0),
            make_sample(20.0, 7'200'000.0),
            make_sample(30.0, 7'300'000.0),
    };
    track.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(0.0, 10.0, 7'000'000.0, 7'100'000.0),
            make_segment(10.0, 20.0, 7'100'000.0, 7'200'000.0),
            make_segment(20.0, 30.0, 7'200'000.0, 7'300'000.0),
    };
    track.cache.display.trajectory_frame_planned = track.cache.solver.trajectory_inertial_planned;
    track.cache.display.trajectory_segments_frame_planned = track.cache.solver.trajectory_segments_inertial_planned;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result streaming_result{};
    streaming_result.track_id = state._prediction_tracks.front().key.track_id();
    streaming_result.generation_id = 5;
    streaming_result.valid = true;
    streaming_result.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    streaming_result.cache.identity.valid = true;
    streaming_result.cache.identity.generation_id = 5;
    streaming_result.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    streaming_result.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    streaming_result.cache.display.trajectory_frame = streaming_result.cache.solver.trajectory_inertial;
    streaming_result.cache.display.trajectory_segments_frame = streaming_result.cache.solver.trajectory_segments_inertial;
    streaming_result.cache.solver.trajectory_inertial_planned = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10.0, 7'100'000.0),
            make_sample(20.0, 7'200'000.0),
    };
    streaming_result.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(0.0, 10.0, 7'000'000.0, 7'100'000.0),
            make_segment(10.0, 20.0, 7'100'000.0, 7'200'000.0),
    };
    streaming_result.cache.display.trajectory_frame_planned = streaming_result.cache.solver.trajectory_inertial_planned;
    streaming_result.cache.display.trajectory_segments_frame_planned = streaming_result.cache.solver.trajectory_segments_inertial_planned;
    streaming_result.chunk_assembly.valid = true;
    streaming_result.chunk_assembly.generation_id = 5;
    streaming_result.chunk_assembly.chunks = {
            make_chunk(0u, 5u, 0.0, 10.0, 7'000'000.0, 7'100'000.0),
            make_chunk(1u, 5u, 10.0, 20.0, 7'100'000.0, 7'200'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(streaming_result));

    ASSERT_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.chunks.size(), 2u);
    EXPECT_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.chunks[0].chunk_id, 0u);
    EXPECT_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.chunks[1].chunk_id, 1u);

    Game::OrbitPredictionDerivedService::Result finalizing_result{};
    finalizing_result.track_id = state._prediction_tracks.front().key.track_id();
    finalizing_result.generation_id = 5;
    finalizing_result.valid = true;
    finalizing_result.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    finalizing_result.cache.identity.valid = true;
    finalizing_result.cache.identity.generation_id = 5;
    finalizing_result.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    finalizing_result.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    finalizing_result.cache.display.trajectory_frame = finalizing_result.cache.solver.trajectory_inertial;
    finalizing_result.cache.display.trajectory_segments_frame = finalizing_result.cache.solver.trajectory_segments_inertial;
    finalizing_result.cache.solver.trajectory_inertial_planned = {
            make_sample(20.0, 7'200'000.0),
            make_sample(30.0, 7'300'000.0),
    };
    finalizing_result.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(20.0, 30.0, 7'200'000.0, 7'300'000.0),
    };
    finalizing_result.cache.display.trajectory_frame_planned = finalizing_result.cache.solver.trajectory_inertial_planned;
    finalizing_result.cache.display.trajectory_segments_frame_planned = finalizing_result.cache.solver.trajectory_segments_inertial_planned;
    finalizing_result.chunk_assembly.valid = true;
    finalizing_result.chunk_assembly.generation_id = 5;
    finalizing_result.chunk_assembly.chunks = {
            make_chunk(2u, 5u, 20.0, 30.0, 7'200'000.0, 7'300'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(finalizing_result));

    ASSERT_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.chunks.size(), 3u);
    EXPECT_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.generation_id, 5u);
    EXPECT_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.chunks[0].chunk_id, 0u);
    EXPECT_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.chunks[1].chunk_id, 1u);
    EXPECT_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.chunks[2].chunk_id, 2u);
    EXPECT_DOUBLE_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.chunks[0].t0_s, 0.0);
    EXPECT_DOUBLE_EQ(state._prediction_tracks.front().preview_overlay.chunk_assembly.chunks[2].t1_s, 30.0);
}

TEST(GameplayPredictionManeuverTests, PreviewDerivedResultRestoresExistingPlannedCacheWhenAuthoritativeSeedIsMissing)
{
    Game::GameplayState state{};

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 10.0;
    state._maneuver_state.nodes.push_back(node);
    const uint64_t plan_signature = state.current_maneuver_plan_signature();

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    activate_live_maneuver_preview(state, track, node.id, node.time_s);
    track.cache.identity.valid = true;
    track.cache.identity.generation_id = 4;
    track.cache.identity.maneuver_plan_signature_valid = true;
    track.cache.identity.maneuver_plan_signature = plan_signature;
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    track.cache.display.trajectory_frame = track.cache.solver.trajectory_inertial;
    track.cache.display.trajectory_segments_frame = track.cache.solver.trajectory_segments_inertial;
    track.cache.solver.trajectory_inertial_planned = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10.0, 7'100'000.0),
            make_sample(20.0, 7'200'000.0),
            make_sample(30.0, 7'300'000.0),
    };
    track.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(0.0, 10.0, 7'000'000.0, 7'100'000.0),
            make_segment(10.0, 20.0, 7'100'000.0, 7'200'000.0),
            make_segment(20.0, 30.0, 7'200'000.0, 7'300'000.0),
    };
    track.cache.display.trajectory_frame_planned = track.cache.solver.trajectory_inertial_planned;
    track.cache.display.trajectory_segments_frame_planned = track.cache.solver.trajectory_segments_inertial_planned;
    track.cache.solver.maneuver_previews = {
            Game::OrbitPredictionService::ManeuverNodePreview{
                    .node_id = 1,
                    .t_s = 10.0,
                    .valid = true,
                    .inertial_position_m = glm::dvec3(7'100'000.0, 1.0, 0.0),
                    .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
            },
    };
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 5;
    result.valid = true;
    result.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    result.publish_stage = Game::OrbitPredictionService::PublishStage::PreviewStreaming;
    result.cache.identity.valid = true;
    result.cache.identity.generation_id = 5;
    result.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    result.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    result.cache.display.trajectory_frame = result.cache.solver.trajectory_inertial;
    result.cache.display.trajectory_segments_frame = result.cache.solver.trajectory_segments_inertial;
    result.cache.solver.trajectory_inertial_planned = {
            make_sample(10.0, 7'150'000.0),
            make_sample(20.0, 7'250'000.0),
    };
    result.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(10.0, 20.0, 7'150'000.0, 7'250'000.0),
    };
    result.cache.display.trajectory_frame_planned = result.cache.solver.trajectory_inertial_planned;
    result.cache.display.trajectory_segments_frame_planned = result.cache.solver.trajectory_segments_inertial_planned;
    result.cache.solver.maneuver_previews = {
            Game::OrbitPredictionService::ManeuverNodePreview{
                    .node_id = 1,
                    .t_s = 10.0,
                    .valid = true,
                    .inertial_position_m = glm::dvec3(7'150'000.0, 2.0, 0.0),
                    .inertial_velocity_mps = glm::dvec3(0.0, 7'600.0, 0.0),
            },
    };
    result.chunk_assembly.valid = true;
    result.chunk_assembly.generation_id = 5;
    result.chunk_assembly.chunks = {
            make_chunk(1u, 5u, 10.0, 20.0, 7'150'000.0, 7'250'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    const Game::PredictionTrackState &updated_track = state._prediction_tracks.front();
    EXPECT_TRUE(updated_track.authoritative_cache.identity.valid);
    ASSERT_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.size(), 3u);
    EXPECT_DOUBLE_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.front().t0_s, 0.0);
    EXPECT_DOUBLE_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.back().t0_s, 20.0);
    ASSERT_EQ(updated_track.cache.solver.maneuver_previews.size(), 1u);
    EXPECT_DOUBLE_EQ(updated_track.cache.solver.maneuver_previews.front().inertial_position_m.x, 7'100'000.0);
    ASSERT_EQ(updated_track.preview_overlay.chunk_assembly.chunks.size(), 1u);
    EXPECT_DOUBLE_EQ(updated_track.preview_overlay.chunk_assembly.chunks.front().t0_s, 10.0);
}

TEST(GameplayPredictionManeuverTests, PreviewDerivedResultDoesNotRestorePlannedCacheForMovedNodeTime)
{
    Game::GameplayState state{};

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 10.0;
    state._maneuver_state.nodes.push_back(node);
    const uint64_t old_plan_signature = state.current_maneuver_plan_signature();
    state._maneuver_state.nodes.front().time_s = 20.0;
    node.time_s = 20.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    activate_live_maneuver_preview(state, track, node.id, node.time_s);
    track.cache.identity.valid = true;
    track.cache.identity.generation_id = 4;
    track.cache.identity.maneuver_plan_signature_valid = true;
    track.cache.identity.maneuver_plan_signature = old_plan_signature;
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    track.cache.display.trajectory_frame = track.cache.solver.trajectory_inertial;
    track.cache.display.trajectory_segments_frame = track.cache.solver.trajectory_segments_inertial;
    track.cache.solver.trajectory_inertial_planned = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10.0, 7'100'000.0),
            make_sample(30.0, 7'300'000.0),
    };
    track.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(0.0, 10.0, 7'000'000.0, 7'100'000.0),
            make_segment(10.0, 30.0, 7'100'000.0, 7'300'000.0),
    };
    track.cache.display.trajectory_frame_planned = track.cache.solver.trajectory_inertial_planned;
    track.cache.display.trajectory_segments_frame_planned = track.cache.solver.trajectory_segments_inertial_planned;
    track.cache.solver.maneuver_previews = {
            Game::OrbitPredictionService::ManeuverNodePreview{
                    .node_id = node.id,
                    .t_s = 10.0,
                    .valid = true,
                    .inertial_position_m = glm::dvec3(7'100'000.0, 0.0, 0.0),
                    .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
            },
    };
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 5;
    result.valid = true;
    result.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    result.publish_stage = Game::OrbitPredictionService::PublishStage::PreviewStreaming;
    result.cache.identity.valid = true;
    result.cache.identity.generation_id = 5;
    result.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    result.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    result.cache.display.trajectory_frame = result.cache.solver.trajectory_inertial;
    result.cache.display.trajectory_segments_frame = result.cache.solver.trajectory_segments_inertial;
    result.cache.solver.trajectory_inertial_planned = {
            make_sample(20.0, 7'200'000.0),
            make_sample(30.0, 7'350'000.0),
    };
    result.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(20.0, 30.0, 7'200'000.0, 7'350'000.0),
    };
    result.cache.display.trajectory_frame_planned = result.cache.solver.trajectory_inertial_planned;
    result.cache.display.trajectory_segments_frame_planned = result.cache.solver.trajectory_segments_inertial_planned;
    result.cache.solver.maneuver_previews = {
            Game::OrbitPredictionService::ManeuverNodePreview{
                    .node_id = node.id,
                    .t_s = node.time_s,
                    .valid = true,
                    .inertial_position_m = glm::dvec3(7'200'000.0, 0.0, 0.0),
                    .inertial_velocity_mps = glm::dvec3(0.0, 7'600.0, 0.0),
            },
    };
    result.chunk_assembly.valid = true;
    result.chunk_assembly.generation_id = 5;
    result.chunk_assembly.chunks = {
            make_chunk(2u, 5u, 20.0, 30.0, 7'200'000.0, 7'350'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    const Game::PredictionTrackState &updated_track = state._prediction_tracks.front();
    ASSERT_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.size(), 1u);
    EXPECT_DOUBLE_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.front().t0_s, 20.0);
    ASSERT_EQ(updated_track.cache.solver.maneuver_previews.size(), 1u);
    EXPECT_DOUBLE_EQ(updated_track.cache.solver.maneuver_previews.front().t_s, 20.0);
}

TEST(GameplayPredictionManeuverTests, PreviewDerivedResultDoesNotRestorePlannedCacheForChangedNodeDeltaV)
{
    Game::GameplayState state{};

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 10.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver_state.nodes.push_back(node);
    const uint64_t old_plan_signature = state.current_maneuver_plan_signature();
    state._maneuver_state.nodes.front().dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    activate_live_maneuver_preview(state, track, node.id, node.time_s);
    track.cache.identity.valid = true;
    track.cache.identity.generation_id = 4;
    track.cache.identity.maneuver_plan_signature_valid = true;
    track.cache.identity.maneuver_plan_signature = old_plan_signature;
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    track.cache.display.trajectory_frame = track.cache.solver.trajectory_inertial;
    track.cache.display.trajectory_segments_frame = track.cache.solver.trajectory_segments_inertial;
    track.cache.solver.trajectory_inertial_planned = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10.0, 7'100'000.0),
            make_sample(30.0, 7'300'000.0),
    };
    track.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(0.0, 10.0, 7'000'000.0, 7'100'000.0),
            make_segment(10.0, 30.0, 7'100'000.0, 7'300'000.0),
    };
    track.cache.display.trajectory_frame_planned = track.cache.solver.trajectory_inertial_planned;
    track.cache.display.trajectory_segments_frame_planned = track.cache.solver.trajectory_segments_inertial_planned;
    track.cache.solver.maneuver_previews = {
            Game::OrbitPredictionService::ManeuverNodePreview{
                    .node_id = node.id,
                    .t_s = node.time_s,
                    .valid = true,
                    .inertial_position_m = glm::dvec3(7'100'000.0, 0.0, 0.0),
                    .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
            },
    };
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 5;
    result.valid = true;
    result.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    result.publish_stage = Game::OrbitPredictionService::PublishStage::PreviewStreaming;
    result.cache.identity.valid = true;
    result.cache.identity.generation_id = 5;
    result.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    result.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    result.cache.display.trajectory_frame = result.cache.solver.trajectory_inertial;
    result.cache.display.trajectory_segments_frame = result.cache.solver.trajectory_segments_inertial;
    result.cache.solver.trajectory_inertial_planned = {
            make_sample(10.0, 7'250'000.0),
            make_sample(30.0, 7'350'000.0),
    };
    result.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(10.0, 30.0, 7'250'000.0, 7'350'000.0),
    };
    result.cache.display.trajectory_frame_planned = result.cache.solver.trajectory_inertial_planned;
    result.cache.display.trajectory_segments_frame_planned = result.cache.solver.trajectory_segments_inertial_planned;
    result.cache.solver.maneuver_previews = {
            Game::OrbitPredictionService::ManeuverNodePreview{
                    .node_id = node.id,
                    .t_s = node.time_s,
                    .valid = true,
                    .inertial_position_m = glm::dvec3(7'250'000.0, 0.0, 0.0),
                    .inertial_velocity_mps = glm::dvec3(0.0, 7'700.0, 0.0),
            },
    };
    result.chunk_assembly.valid = true;
    result.chunk_assembly.generation_id = 5;
    result.chunk_assembly.chunks = {
            make_chunk(2u, 5u, 10.0, 30.0, 7'250'000.0, 7'350'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    const Game::PredictionTrackState &updated_track = state._prediction_tracks.front();
    ASSERT_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.size(), 1u);
    EXPECT_DOUBLE_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.front().t0_s, 10.0);
    EXPECT_DOUBLE_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.front().start.position_m.x, 7'250'000.0);
    ASSERT_EQ(updated_track.cache.solver.maneuver_previews.size(), 1u);
    EXPECT_DOUBLE_EQ(updated_track.cache.solver.maneuver_previews.front().inertial_position_m.x, 7'250'000.0);
}

TEST(GameplayPredictionManeuverTests, LateFastPreviewDerivedResultAfterPreviewEndsIsIgnored)
{
    Game::GameplayState state{};

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 10.0;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.cache = make_prediction_cache(4u, 0.0, 30.0, 7'000'000.0, 7'300'000.0);
    track.preview_overlay.chunk_assembly.valid = true;
    track.preview_overlay.chunk_assembly.generation_id = 5u;
    track.preview_overlay.chunk_assembly.chunks = {
            make_chunk(0u, 5u, 10.0, 20.0, 7'050'000.0, 7'150'000.0),
    };
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 5u;
    result.valid = true;
    result.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    result.publish_stage = Game::OrbitPredictionService::PublishStage::PreviewStreaming;
    result.cache.identity.valid = true;
    result.cache.identity.generation_id = 5u;
    result.cache.solver.trajectory_inertial = {make_sample(0.0, 8'000'000.0), make_sample(30.0, 8'300'000.0)};
    result.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 8'000'000.0, 8'300'000.0)};
    result.cache.display.trajectory_frame = result.cache.solver.trajectory_inertial;
    result.cache.display.trajectory_segments_frame = result.cache.solver.trajectory_segments_inertial;
    result.cache.solver.trajectory_inertial_planned = {make_sample(10.0, 8'100'000.0), make_sample(20.0, 8'200'000.0)};
    result.cache.solver.trajectory_segments_inertial_planned = {make_segment(10.0, 20.0, 8'100'000.0, 8'200'000.0)};
    result.cache.display.trajectory_frame_planned = result.cache.solver.trajectory_inertial_planned;
    result.cache.display.trajectory_segments_frame_planned = result.cache.solver.trajectory_segments_inertial_planned;
    result.chunk_assembly.valid = true;
    result.chunk_assembly.generation_id = 5u;
    result.chunk_assembly.chunks = {
            make_chunk(1u, 5u, 20.0, 30.0, 8'200'000.0, 8'300'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(result));

    const Game::PredictionTrackState &updated_track = state._prediction_tracks.front();
    EXPECT_EQ(updated_track.cache.identity.generation_id, 4u);
    EXPECT_FALSE(updated_track.preview_overlay.chunk_assembly.valid);
    EXPECT_TRUE(updated_track.preview_overlay.chunk_assembly.chunks.empty());
    EXPECT_EQ(updated_track.preview_state, Game::PredictionPreviewRuntimeState::AwaitFullRefine);
    EXPECT_TRUE(updated_track.preview_anchor.valid);
}

TEST(GameplayPredictionManeuverTests, PreviewFinalizingDerivedResultKeepsAnchorStateAvailableForNextPreview)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = 1;
    track.preview_anchor.anchor_time_s = 10.0;
    track.cache.identity.valid = true;
    track.cache.identity.generation_id = 4;
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    track.cache.display.trajectory_frame = track.cache.solver.trajectory_inertial;
    track.cache.display.trajectory_segments_frame = track.cache.solver.trajectory_segments_inertial;
    track.preview_overlay.chunk_assembly.valid = true;
    track.preview_overlay.chunk_assembly.generation_id = 5;
    track.preview_overlay.chunk_assembly.chunks = {
            make_chunk(0u, 5u, 10.0, 20.0, 7'050'000.0, 7'150'000.0),
    };
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 5;
    result.valid = true;
    result.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    result.publish_stage = Game::OrbitPredictionService::PublishStage::PreviewFinalizing;
    result.cache.identity.valid = true;
    result.cache.identity.generation_id = 5;
    result.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    result.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    result.cache.display.trajectory_frame = result.cache.solver.trajectory_inertial;
    result.cache.display.trajectory_segments_frame = result.cache.solver.trajectory_segments_inertial;
    result.cache.solver.trajectory_inertial_planned = {
            make_sample(10.0, 7'050'000.0),
            make_sample(20.0, 7'150'000.0),
            make_sample(30.0, 7'300'000.0),
    };
    result.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(10.0, 20.0, 7'050'000.0, 7'150'000.0),
            make_segment(20.0, 30.0, 7'150'000.0, 7'300'000.0),
    };
    result.cache.display.trajectory_frame_planned = result.cache.solver.trajectory_inertial_planned;
    result.cache.display.trajectory_segments_frame_planned = result.cache.solver.trajectory_segments_inertial_planned;
    result.cache.solver.maneuver_previews = {
            Game::OrbitPredictionService::ManeuverNodePreview{
                    .node_id = 1,
                    .t_s = 10.0,
                    .valid = true,
                    .inertial_position_m = glm::dvec3(7'050'000.0, 11.0, 0.0),
                    .inertial_velocity_mps = glm::dvec3(0.0, 7'550.0, 1.0),
            },
            Game::OrbitPredictionService::ManeuverNodePreview{
                    .node_id = 2,
                    .t_s = 25.0,
                    .valid = true,
                    .inertial_position_m = glm::dvec3(7'220'000.0, 22.0, 0.0),
                    .inertial_velocity_mps = glm::dvec3(0.0, 7'650.0, 2.0),
            },
    };
    result.chunk_assembly.valid = true;
    result.chunk_assembly.generation_id = 5;
    result.chunk_assembly.chunks = {
            make_chunk(1u, 5u, 20.0, 30.0, 7'150'000.0, 7'300'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    const Game::PredictionTrackState &updated_track = state._prediction_tracks.front();
    ASSERT_EQ(updated_track.cache.solver.maneuver_previews.size(), 2u);
    ASSERT_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.size(), 2u);
    EXPECT_DOUBLE_EQ(updated_track.cache.solver.trajectory_segments_inertial_planned.front().t0_s, 10.0);

    orbitsim::State anchor_state{};
    ASSERT_TRUE(state.resolve_prediction_preview_anchor_state(updated_track, anchor_state));
    EXPECT_DOUBLE_EQ(anchor_state.position_m.x, 7'050'000.0);
    EXPECT_DOUBLE_EQ(anchor_state.position_m.y, 11.0);
    EXPECT_DOUBLE_EQ(anchor_state.velocity_mps.y, 7'550.0);
}

TEST(GameplayPredictionManeuverTests, FullStreamingSequenceConvergesAfterFinalDerivedPublish)
{
    Game::GameplayState state{};
    state._prediction_sampling_policy.orbiter_min_window_s = 5.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.cache = make_prediction_cache(8u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = false;
    track.latest_requested_generation_id = 9u;
    track.latest_requested_authoritative_generation_id = 9u;
    track.derived_request_pending = true;
    track.latest_requested_derived_generation_id = 9u;
    track.latest_requested_derived_display_frame_key = 1u;
    track.latest_requested_derived_display_frame_revision = 1u;
    track.latest_requested_derived_analysis_body_id = orbitsim::kInvalidBodyId;
    track.latest_requested_derived_publish_stage = Game::OrbitPredictionService::PublishStage::Final;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result streamed_result{};
    streamed_result.track_id = state._prediction_tracks.front().key.track_id();
    streamed_result.generation_id = 9u;
    streamed_result.display_frame_key = 1u;
    streamed_result.display_frame_revision = 1u;
    streamed_result.analysis_body_id = orbitsim::kInvalidBodyId;
    streamed_result.valid = true;
    streamed_result.solve_quality = Game::OrbitPredictionService::SolveQuality::Full;
    streamed_result.publish_stage = Game::OrbitPredictionService::PublishStage::FullStreaming;
    streamed_result.cache = make_prediction_cache(9u, 0.0, 20.0, 7'050'000.0, 7'250'000.0);
    streamed_result.chunk_assembly.valid = true;
    streamed_result.chunk_assembly.generation_id = 9u;
    streamed_result.chunk_assembly.chunks = {
            make_chunk(0u, 9u, 0.0, 10.0, 7'050'000.0, 7'150'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(streamed_result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    auto lifecycle = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(state._prediction_tracks.front());
    EXPECT_EQ(lifecycle.state, Game::PredictionTrackLifecycleState::FullStreaming);
    EXPECT_TRUE(state._prediction_tracks.front().derived_request_pending);

    Game::OrbitPredictionService::Result solver_result{};
    solver_result.track_id = state._prediction_tracks.front().key.track_id();
    solver_result.generation_id = 9u;
    solver_result.valid = true;
    solver_result.solve_quality = Game::OrbitPredictionService::SolveQuality::Full;
    solver_result.publish_stage = Game::OrbitPredictionService::PublishStage::Final;
    solver_result.build_time_s = 0.0;
    solver_result.trajectory_inertial = {
            make_sample(0.0, 7'000'000.0),
            make_sample(20.0, 7'250'000.0),
    };
    solver_result.trajectory_segments_inertial = {
            make_segment(0.0, 20.0, 7'000'000.0, 7'250'000.0),
    };
    solver_result.trajectory_inertial_planned = {
            make_sample(0.0, 7'000'000.0),
            make_sample(20.0, 7'300'000.0),
    };
    solver_result.trajectory_segments_inertial_planned = {
            make_segment(0.0, 20.0, 7'000'000.0, 7'300'000.0),
    };

    orbitsim::MassiveBody ref{};
    ref.id = 1;
    ref.mass_kg = 5.972e24;
    ref.radius_m = 6'371'000.0;
    ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
    solver_result.massive_bodies.push_back(ref);

    state.apply_completed_prediction_result(std::move(solver_result));

    const Game::PredictionTrackState &after_solver = state._prediction_tracks.front();
    ASSERT_TRUE(after_solver.derived_request_pending);
    EXPECT_EQ(after_solver.latest_requested_derived_publish_stage, Game::OrbitPredictionService::PublishStage::Final);
    EXPECT_FALSE(after_solver.request_pending);

    Game::OrbitPredictionDerivedService::Result late_streamed_result{};
    late_streamed_result.track_id = after_solver.key.track_id();
    late_streamed_result.generation_id = after_solver.latest_requested_derived_generation_id;
    late_streamed_result.display_frame_key = after_solver.latest_requested_derived_display_frame_key;
    late_streamed_result.display_frame_revision = after_solver.latest_requested_derived_display_frame_revision;
    late_streamed_result.analysis_body_id = after_solver.latest_requested_derived_analysis_body_id;
    late_streamed_result.valid = true;
    late_streamed_result.solve_quality = Game::OrbitPredictionService::SolveQuality::Full;
    late_streamed_result.publish_stage = Game::OrbitPredictionService::PublishStage::FullStreaming;
    late_streamed_result.cache = make_prediction_cache(9u, 0.0, 20.0, 7'050'000.0, 7'250'000.0);
    late_streamed_result.chunk_assembly.valid = true;
    late_streamed_result.chunk_assembly.generation_id = 9u;
    late_streamed_result.chunk_assembly.chunks = {
            make_chunk(1u, 9u, 10.0, 20.0, 7'150'000.0, 7'300'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(late_streamed_result));

    const Game::PredictionTrackState &after_late_stream = state._prediction_tracks.front();
    EXPECT_TRUE(after_late_stream.derived_request_pending);
    EXPECT_FALSE(state.should_rebuild_prediction_track(after_late_stream, 10.0, 0.016f, false, false));

    Game::OrbitPredictionDerivedService::Result final_result{};
    final_result.track_id = after_late_stream.key.track_id();
    final_result.generation_id = after_late_stream.latest_requested_derived_generation_id;
    final_result.display_frame_key = after_late_stream.latest_requested_derived_display_frame_key;
    final_result.display_frame_revision = after_late_stream.latest_requested_derived_display_frame_revision;
    final_result.analysis_body_id = after_late_stream.latest_requested_derived_analysis_body_id;
    final_result.valid = true;
    final_result.solve_quality = Game::OrbitPredictionService::SolveQuality::Full;
    final_result.publish_stage = Game::OrbitPredictionService::PublishStage::Final;
    final_result.cache = make_prediction_cache(9u, 0.0, 20.0, 7'050'000.0, 7'300'000.0);

    state.apply_completed_prediction_derived_result(std::move(final_result));

    const Game::PredictionTrackState &final_track = state._prediction_tracks.front();
    lifecycle = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(final_track);
    EXPECT_FALSE(final_track.derived_request_pending);
    EXPECT_EQ(final_track.authoritative_cache.identity.generation_id, 9u);
    EXPECT_FALSE(final_track.full_stream_overlay.chunk_assembly.valid);
    EXPECT_TRUE(final_track.full_stream_overlay.chunk_assembly.chunks.empty());
    EXPECT_EQ(lifecycle.state, Game::PredictionTrackLifecycleState::Stable);
    EXPECT_FALSE(state.should_rebuild_prediction_track(final_track, 10.0, 0.016f, false, false));
}

TEST(GameplayPredictionManeuverTests, LateFullStreamingDerivedResultAfterFinalPublishIsIgnored)
{
    Game::GameplayState state{};
    state._prediction_sampling_policy.orbiter_min_window_s = 5.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.supports_maneuvers = true;
    track.cache = make_prediction_cache(9u, 0.0, 20.0, 7'050'000.0, 7'300'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = false;
    track.latest_requested_generation_id = 9u;
    track.latest_requested_authoritative_generation_id = 9u;
    track.derived_request_pending = false;
    track.latest_requested_derived_generation_id = 9u;
    track.latest_requested_derived_display_frame_key = 1u;
    track.latest_requested_derived_display_frame_revision = 1u;
    track.latest_requested_derived_analysis_body_id = orbitsim::kInvalidBodyId;
    track.latest_requested_derived_publish_stage = Game::OrbitPredictionService::PublishStage::Final;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result late_streamed_result{};
    late_streamed_result.track_id = state._prediction_tracks.front().key.track_id();
    late_streamed_result.generation_id = 9u;
    late_streamed_result.display_frame_key = 1u;
    late_streamed_result.display_frame_revision = 1u;
    late_streamed_result.analysis_body_id = orbitsim::kInvalidBodyId;
    late_streamed_result.valid = true;
    late_streamed_result.solve_quality = Game::OrbitPredictionService::SolveQuality::Full;
    late_streamed_result.publish_stage = Game::OrbitPredictionService::PublishStage::FullStreaming;
    late_streamed_result.cache = make_prediction_cache(9u, 0.0, 20.0, 8'050'000.0, 8'300'000.0);
    late_streamed_result.chunk_assembly.valid = true;
    late_streamed_result.chunk_assembly.generation_id = 9u;
    late_streamed_result.chunk_assembly.chunks = {
            make_chunk(1u, 9u, 10.0, 20.0, 8'150'000.0, 8'300'000.0),
    };

    state.apply_completed_prediction_derived_result(std::move(late_streamed_result));

    const Game::PredictionTrackState &updated_track = state._prediction_tracks.front();
    const auto lifecycle = Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(updated_track);
    EXPECT_EQ(lifecycle.state, Game::PredictionTrackLifecycleState::Stable);
    EXPECT_FALSE(updated_track.derived_request_pending);
    EXPECT_FALSE(updated_track.full_stream_overlay.chunk_assembly.valid);
    EXPECT_TRUE(updated_track.full_stream_overlay.chunk_assembly.chunks.empty());
    EXPECT_DOUBLE_EQ(updated_track.cache.display.trajectory_frame.front().position_m.x, 7'050'000.0);
    EXPECT_DOUBLE_EQ(updated_track.cache.display.trajectory_frame.back().position_m.x, 7'300'000.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
