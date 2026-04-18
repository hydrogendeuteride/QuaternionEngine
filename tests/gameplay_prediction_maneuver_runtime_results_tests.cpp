#include "gameplay_prediction_maneuver_test_common.h"

TEST(GameplayPredictionManeuverTests, CompletedSolverResultClearsSolverPendingAndQueuesDerivedWork)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.request_pending = true;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 1;
    result.valid = true;
    result.build_time_s = 0.0;
    result.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    result.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};

    orbitsim::MassiveBody ref{};
    ref.id = 1;
    ref.mass_kg = 5.972e24;
    ref.radius_m = 6'371'000.0;
    ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
    result.massive_bodies.push_back(ref);

    state.apply_completed_prediction_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    EXPECT_FALSE(state._prediction_tracks.front().request_pending);
    EXPECT_TRUE(state._prediction_tracks.front().derived_request_pending);
    const auto lifecycle =
            Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(state._prediction_tracks.front());
    EXPECT_EQ(lifecycle.state, Game::PredictionTrackLifecycleState::FinalDerivedPending);
}

TEST(GameplayPredictionManeuverTests, OlderSolverResultCanStillQueueDerivedWorkWhileNewerRequestIsPending)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.request_pending = true;
    track.latest_requested_generation_id = 2;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 1;
    result.valid = true;
    result.build_time_s = 0.0;
    result.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    result.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};

    orbitsim::MassiveBody ref{};
    ref.id = 1;
    ref.mass_kg = 5.972e24;
    ref.radius_m = 6'371'000.0;
    ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
    result.massive_bodies.push_back(ref);

    state.apply_completed_prediction_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    EXPECT_TRUE(state._prediction_tracks.front().derived_request_pending);
}

TEST(GameplayPredictionManeuverTests, PreviewStreamingSolverResultMarksPreviewStateAndQueuesDerivedWork)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.supports_maneuvers = true;
    track.request_pending = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::DragPreviewPending;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 5;
    result.valid = true;
    result.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    result.publish_stage = Game::OrbitPredictionService::PublishStage::PreviewStreaming;
    result.build_time_s = 0.0;
    result.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    result.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    result.trajectory_inertial_planned = {make_sample(0.0, 7'000'000.0), make_sample(20.0, 7'050'000.0)};
    result.trajectory_segments_inertial_planned = {make_segment(0.0, 20.0, 7'000'000.0, 7'050'000.0)};

    orbitsim::MassiveBody ref{};
    ref.id = 1;
    ref.mass_kg = 5.972e24;
    ref.radius_m = 6'371'000.0;
    ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
    result.massive_bodies.push_back(ref);

    state.apply_completed_prediction_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    EXPECT_EQ(state._prediction_tracks.front().preview_state,
              Game::PredictionPreviewRuntimeState::PreviewStreaming);
    EXPECT_FALSE(state._prediction_tracks.front().request_pending);
    EXPECT_TRUE(state._prediction_tracks.front().derived_request_pending);
    EXPECT_EQ(state._prediction_tracks.front().latest_requested_derived_generation_id, 5u);
    const auto lifecycle =
            Game::PredictionRuntimeDetail::describe_prediction_track_lifecycle(state._prediction_tracks.front());
    EXPECT_EQ(lifecycle.state, Game::PredictionTrackLifecycleState::PreviewStreaming);
}

TEST(GameplayPredictionManeuverTests, DerivedPreviewStreamingBuildSkipsPlannedRenderCurve)
{
    Game::OrbitPredictionDerivedService service{};

    Game::OrbitPredictionDerivedService::Result result = service.build_cache(make_prediction_derived_job(
            Game::OrbitPredictionService::SolveQuality::FastPreview,
            Game::OrbitPredictionService::PublishStage::PreviewStreaming));

    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.cache.valid);
    EXPECT_FALSE(result.cache.trajectory_segments_frame_planned.empty());
    EXPECT_TRUE(result.cache.render_curve_frame_planned.empty());
}

TEST(GameplayPredictionManeuverTests, DerivedPreviewFinalizingBuildRestoresPlannedRenderCurve)
{
    Game::OrbitPredictionDerivedService service{};

    Game::OrbitPredictionDerivedService::Result result = service.build_cache(make_prediction_derived_job(
            Game::OrbitPredictionService::SolveQuality::FastPreview,
            Game::OrbitPredictionService::PublishStage::PreviewFinalizing));

    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.cache.valid);
    EXPECT_FALSE(result.cache.trajectory_segments_frame_planned.empty());
    EXPECT_FALSE(result.cache.render_curve_frame_planned.empty());
}

TEST(GameplayPredictionManeuverTests, DerivedFullStreamingBuildUsesStreamedChunkPayloadWithoutFlatteningPlannedCache)
{
    Game::OrbitPredictionDerivedService service{};
    Game::OrbitPredictionDerivedService::PendingJob job = make_prediction_derived_job(
            Game::OrbitPredictionService::SolveQuality::Full,
            Game::OrbitPredictionService::PublishStage::FullStreaming);

    Game::OrbitPredictionService::Result &solver = job.request.solver_result;
    solver.trajectory_inertial_planned.clear();
    solver.trajectory_segments_inertial_planned.clear();
    solver.maneuver_previews.clear();
    solver.published_chunks = {
            Game::OrbitPredictionService::PublishedChunk{
                    .chunk_id = 3u,
                    .quality_state = Game::OrbitPredictionService::ChunkQualityState::Final,
                    .t0_s = 0.0,
                    .t1_s = 10.0,
                    .includes_planned_path = true,
            },
    };

    Game::OrbitPredictionService::StreamedPlannedChunk streamed_chunk{};
    streamed_chunk.published_chunk = solver.published_chunks.front();
    streamed_chunk.trajectory_inertial = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10.0, 7'150'000.0),
    };
    streamed_chunk.trajectory_segments_inertial = {
            make_segment(0.0, 10.0, 7'000'000.0, 7'150'000.0),
    };
    solver.streamed_planned_chunks = {std::move(streamed_chunk)};

    Game::OrbitPredictionDerivedService::Result result = service.build_cache(std::move(job));

    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.cache.valid);
    EXPECT_TRUE(result.cache.trajectory_segments_frame_planned.empty());
    EXPECT_TRUE(result.cache.render_curve_frame_planned.empty());
    ASSERT_TRUE(result.chunk_assembly.valid);
    ASSERT_EQ(result.chunk_assembly.chunks.size(), 1u);
    EXPECT_EQ(result.chunk_assembly.chunks.front().chunk_id, 3u);
    EXPECT_DOUBLE_EQ(result.chunk_assembly.chunks.front().t0_s, 0.0);
    EXPECT_DOUBLE_EQ(result.chunk_assembly.chunks.front().t1_s, 10.0);
    EXPECT_FALSE(result.chunk_assembly.chunks.front().render_curve.empty());
    EXPECT_EQ(result.diagnostics.status, Game::PredictionDerivedStatus::Success);
}

TEST(GameplayPredictionManeuverTests, DerivedFullStreamingRequestMergesPendingChunksForSameGeneration)
{
    Game::OrbitPredictionDerivedService service{};
    const uint64_t track_id = 17u;
    {
        std::lock_guard<std::mutex> lock(service._mutex);
        service._tracks_in_flight.insert(track_id);
    }

    const auto make_request = [track_id](const uint32_t chunk_id,
                                         const double t0_s,
                                         const double t1_s,
                                         const double x0_m,
                                         const double x1_m) {
        Game::OrbitPredictionDerivedService::Request request{};
        request.track_id = track_id;
        request.generation_id = 9u;
        request.display_frame_key = 1u;
        request.display_frame_revision = 1u;
        request.analysis_body_id = 1;
        request.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::inertial();

        Game::OrbitPredictionService::Result &solver = request.solver_result;
        solver.valid = true;
        solver.solve_quality = Game::OrbitPredictionService::SolveQuality::Full;
        solver.publish_stage = Game::OrbitPredictionService::PublishStage::FullStreaming;
        solver.trajectory_inertial = {
                make_sample(0.0, 7'000'000.0),
                make_sample(20.0, 7'200'000.0),
        };
        solver.trajectory_segments_inertial = {
                make_segment(0.0, 20.0, 7'000'000.0, 7'200'000.0),
        };
        solver.published_chunks = {
                Game::OrbitPredictionService::PublishedChunk{
                        .chunk_id = chunk_id,
                        .quality_state = Game::OrbitPredictionService::ChunkQualityState::Final,
                        .t0_s = t0_s,
                        .t1_s = t1_s,
                        .includes_planned_path = true,
                },
        };

        Game::OrbitPredictionService::StreamedPlannedChunk streamed_chunk{};
        streamed_chunk.published_chunk = solver.published_chunks.front();
        streamed_chunk.trajectory_inertial = {
                make_sample(t0_s, x0_m),
                make_sample(t1_s, x1_m),
        };
        streamed_chunk.trajectory_segments_inertial = {
                make_segment(t0_s, t1_s, x0_m, x1_m),
        };
        solver.streamed_planned_chunks = {std::move(streamed_chunk)};
        return request;
    };

    service.request(make_request(0u, 0.0, 10.0, 7'000'000.0, 7'100'000.0));
    service.request(make_request(1u, 10.0, 20.0, 7'100'000.0, 7'200'000.0));

    std::lock_guard<std::mutex> lock(service._mutex);
    ASSERT_EQ(service._pending_jobs.size(), 1u);
    const Game::OrbitPredictionService::Result &merged_solver = service._pending_jobs.front().request.solver_result;
    ASSERT_EQ(merged_solver.published_chunks.size(), 2u);
    EXPECT_EQ(merged_solver.published_chunks[0].chunk_id, 0u);
    EXPECT_EQ(merged_solver.published_chunks[1].chunk_id, 1u);
    ASSERT_EQ(merged_solver.streamed_planned_chunks.size(), 2u);
    EXPECT_EQ(merged_solver.streamed_planned_chunks[0].published_chunk.chunk_id, 0u);
    EXPECT_EQ(merged_solver.streamed_planned_chunks[1].published_chunk.chunk_id, 1u);
}

TEST(GameplayPredictionManeuverTests, GameplayDefaultsEnableLivePreview)
{
    Game::GameplayState state{};
    const Game::GameplaySettings settings{};

    EXPECT_TRUE(state._maneuver_plan_live_preview_active);
    EXPECT_TRUE(settings.maneuver_plan_live_preview_active);
}

