#include "gameplay_prediction_maneuver_test_common.h"

TEST(GameplayPredictionManeuverTests, BuildEphemerisSamplingSpecAllowsFutureWindowBeyondLegacyCap)
{
    Game::OrbitPredictionService::Request request = make_prediction_request(0.0, 946'080'000.0); // 30 years

    const Game::OrbitPredictionService::EphemerisSamplingSpec spec =
            Game::OrbitPredictionService::build_ephemeris_sampling_spec(request);

    ASSERT_TRUE(spec.valid);
    EXPECT_GE(spec.horizon_s, request.future_window_s);
}

TEST(GameplayPredictionManeuverTests, PredictionServiceBuildsSecondNodePreviewFromPrefixPlan)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request = make_prediction_request(0.0, 120.0);

    Game::OrbitPredictionService::ManeuverImpulse first{};
    first.node_id = 1;
    first.t_s = 10.0;
    first.primary_body_id = 1;
    first.dv_rtn_mps = glm::dvec3(0.0, 0.0, 50.0);
    request.maneuver_impulses.push_back(first);

    Game::OrbitPredictionService::ManeuverImpulse second{};
    second.node_id = 2;
    second.t_s = 20.0;
    second.primary_body_id = 1;
    second.dv_rtn_mps = glm::dvec3(0.0, 0.0, 0.0);
    request.maneuver_impulses.push_back(second);

    std::vector<Game::OrbitPredictionService::Result> results = run_prediction_results(service, 1, std::move(request));
    ASSERT_EQ(results.size(), 1u);
    const Game::OrbitPredictionService::Result &result = results.front();

    ASSERT_TRUE(result.valid) << "status=" << static_cast<int>(result.diagnostics.status)
                              << " ephemeris_accepted=" << result.diagnostics.ephemeris.accepted_segments
                              << " base_accepted=" << result.diagnostics.trajectory_base.accepted_segments
                              << " planned_accepted=" << result.diagnostics.trajectory_planned.accepted_segments;
    ASSERT_GE(result.maneuver_previews.size(), 2u);

    const auto find_preview = [&](const int node_id) -> const Game::OrbitPredictionService::ManeuverNodePreview * {
        for (const auto &preview : result.maneuver_previews)
        {
            if (preview.node_id == node_id)
            {
                return &preview;
            }
        }
        return nullptr;
    };

    const auto *first_preview = find_preview(1);
    const auto *second_preview = find_preview(2);
    ASSERT_NE(first_preview, nullptr);
    ASSERT_NE(second_preview, nullptr);
    EXPECT_TRUE(first_preview->valid);
    EXPECT_TRUE(second_preview->valid);
    EXPECT_NEAR(first_preview->inertial_velocity_mps.z, 0.0, 1.0e-3);
    EXPECT_GT(std::abs(second_preview->inertial_velocity_mps.z), 10.0);
}

TEST(GameplayPredictionManeuverTests, PredictionServiceSupportsFarFutureManeuverBeyondTenHours)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request = make_prediction_request(0.0, 50'400.0);

    Game::OrbitPredictionService::ManeuverImpulse impulse{};
    impulse.node_id = 1;
    impulse.t_s = 50'000.0;
    impulse.primary_body_id = 1;
    impulse.dv_rtn_mps = glm::dvec3(0.0, 0.0, 25.0);
    request.maneuver_impulses.push_back(impulse);

    std::vector<Game::OrbitPredictionService::Result> results = run_prediction_results(service, 1, std::move(request));
    ASSERT_EQ(results.size(), 1u);
    const Game::OrbitPredictionService::Result &result = results.front();

    ASSERT_TRUE(result.valid) << "status=" << static_cast<int>(result.diagnostics.status)
                              << " ephemeris_accepted=" << result.diagnostics.ephemeris.accepted_segments
                              << " base_accepted=" << result.diagnostics.trajectory_base.accepted_segments
                              << " planned_accepted=" << result.diagnostics.trajectory_planned.accepted_segments;
    ASSERT_FALSE(result.trajectory_segments_inertial_planned.empty());
    ASSERT_FALSE(result.maneuver_previews.empty());

    const auto *preview = [&]() -> const Game::OrbitPredictionService::ManeuverNodePreview * {
        for (const auto &item : result.maneuver_previews)
        {
            if (item.node_id == impulse.node_id)
            {
                return &item;
            }
        }
        return nullptr;
    }();
    ASSERT_NE(preview, nullptr);
    EXPECT_TRUE(preview->valid);

    const orbitsim::TrajectorySegment &last_planned = result.trajectory_segments_inertial_planned.back();
    const double planned_end_s = last_planned.t0_s + last_planned.dt_s;
    EXPECT_GE(planned_end_s, impulse.t_s);
}

TEST(GameplayPredictionManeuverTests, PredictionServiceReusesBaselineForFastManeuverPreview)
{
    Game::OrbitPredictionService service{};

    Game::OrbitPredictionService::Request baseline_request = make_prediction_request(0.0, 120.0);
    std::vector<Game::OrbitPredictionService::Result> baseline_results =
            run_prediction_results(service, 1, std::move(baseline_request));
    ASSERT_EQ(baseline_results.size(), 1u);
    const Game::OrbitPredictionService::Result &baseline_result = baseline_results.front();

    ASSERT_TRUE(baseline_result.valid);
    ASSERT_FALSE(baseline_result.trajectory_inertial.empty());
    ASSERT_FALSE(baseline_result.trajectory_segments_inertial.empty());
    ASSERT_TRUE(baseline_result.shared_ephemeris);
    EXPECT_FALSE(baseline_result.diagnostics.trajectory_base.cache_reused);

    Game::OrbitPredictionService::Request preview_request = make_prediction_request(5.0, 120.0);
    preview_request.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    preview_request.shared_ephemeris = baseline_result.shared_ephemeris;

    Game::OrbitPredictionService::ManeuverImpulse impulse{};
    impulse.node_id = 1;
    impulse.t_s = 20.0;
    impulse.primary_body_id = 1;
    impulse.dv_rtn_mps = glm::dvec3(0.0, 0.0, 10.0);
    preview_request.maneuver_impulses.push_back(impulse);

    std::vector<Game::OrbitPredictionService::Result> preview_results =
            run_prediction_results(service, 2, std::move(preview_request));
    ASSERT_EQ(preview_results.size(), 1u);
    const Game::OrbitPredictionService::Result &preview_result = preview_results.front();

    ASSERT_TRUE(preview_result.valid);
    EXPECT_TRUE(preview_result.diagnostics.trajectory_base.cache_reused);
    EXPECT_EQ(preview_result.diagnostics.trajectory_sample_count, baseline_result.diagnostics.trajectory_sample_count);
    ASSERT_FALSE(preview_result.trajectory_segments_inertial_planned.empty());
    EXPECT_GE(preview_result.trajectory_segments_inertial.back().t0_s +
                      preview_result.trajectory_segments_inertial.back().dt_s,
              preview_request.sim_time_s + preview_request.future_window_s);
}

TEST(GameplayPredictionManeuverTests, PredictionServicePublishesChunkMetadataForBaselineOnlyFullRequest)
{
    Game::OrbitPredictionService service{};
    const Game::OrbitPredictionService::Request request = make_prediction_request(5.0, 90.0);

    const std::vector<Game::OrbitPredictionService::Result> results =
            run_prediction_results(service, 1, request);
    ASSERT_EQ(results.size(), 1u);
    const Game::OrbitPredictionService::Result &result = results.front();

    ASSERT_TRUE(result.valid);
    ASSERT_EQ(result.publish_stage, Game::OrbitPredictionService::PublishStage::PreviewFinalizing);
    ASSERT_EQ(result.published_chunks.size(), 1u);
    const Game::OrbitPredictionService::PublishedChunk &chunk = result.published_chunks.front();
    EXPECT_EQ(chunk.chunk_id, 0u);
    EXPECT_EQ(chunk.quality_state, Game::OrbitPredictionService::ChunkQualityState::Final);
    EXPECT_FALSE(chunk.includes_planned_path);
    EXPECT_FALSE(chunk.reused_from_cache);
    EXPECT_DOUBLE_EQ(chunk.t0_s, request.sim_time_s);
    EXPECT_GE(chunk.t1_s, request.sim_time_s + request.future_window_s);
}

TEST(GameplayPredictionManeuverTests, PredictionServicePublishesChunkMetadataForCelestialFullRequest)
{
    Game::OrbitPredictionService service{};
    const Game::OrbitPredictionService::Request request = make_celestial_prediction_request(0.0, 180.0);

    const std::vector<Game::OrbitPredictionService::Result> results =
            run_prediction_results(service, 1, request);
    ASSERT_EQ(results.size(), 1u);
    const Game::OrbitPredictionService::Result &result = results.front();

    ASSERT_TRUE(result.valid);
    ASSERT_EQ(result.publish_stage, Game::OrbitPredictionService::PublishStage::PreviewFinalizing);
    ASSERT_EQ(result.published_chunks.size(), 1u);
    const Game::OrbitPredictionService::PublishedChunk &chunk = result.published_chunks.front();
    EXPECT_EQ(chunk.chunk_id, 0u);
    EXPECT_EQ(chunk.quality_state, Game::OrbitPredictionService::ChunkQualityState::Final);
    EXPECT_FALSE(chunk.includes_planned_path);
    EXPECT_FALSE(chunk.reused_from_cache);
    EXPECT_DOUBLE_EQ(chunk.t0_s, request.sim_time_s);
    EXPECT_GE(chunk.t1_s, request.sim_time_s + request.future_window_s);
}

TEST(GameplayPredictionManeuverTests, PredictionServicePublishesPreviewPatchAndTailAsSeparateChunkStages)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request = make_prediction_request(0.0, 60.0);
    request.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    request.preview_patch.active = true;
    request.preview_patch.anchor_state_valid = true;
    request.preview_patch.anchor_time_s = 20.0;
    request.preview_patch.visual_window_s = 10.0;
    request.preview_patch.exact_window_s = request.preview_patch.visual_window_s;
    request.preview_patch.anchor_state_inertial =
            orbitsim::make_state(request.ship_bary_position_m, request.ship_bary_velocity_mps);

    Game::OrbitPredictionService::ManeuverImpulse stream_impulse{};
    stream_impulse.node_id = 1;
    stream_impulse.t_s = 25.0;
    stream_impulse.primary_body_id = 1;
    stream_impulse.dv_rtn_mps = glm::dvec3(0.0, 0.0, 10.0);
    request.maneuver_impulses.push_back(stream_impulse);

    Game::OrbitPredictionService::ManeuverImpulse finalizing_impulse{};
    finalizing_impulse.node_id = 2;
    finalizing_impulse.t_s = 45.0;
    finalizing_impulse.primary_body_id = 1;
    finalizing_impulse.dv_rtn_mps = glm::dvec3(0.0, 0.0, 15.0);
    request.maneuver_impulses.push_back(finalizing_impulse);

    std::vector<Game::OrbitPredictionService::Result> results = run_prediction_results(service, 1, std::move(request));
    ASSERT_GE(results.size(), 2u);

    std::vector<const Game::OrbitPredictionService::Result *> streaming_results;
    std::vector<const Game::OrbitPredictionService::Result *> finalizing_results;
    bool saw_finalizing_stage = false;
    for (const Game::OrbitPredictionService::Result &result : results)
    {
        ASSERT_TRUE(result.valid);
        if (result.publish_stage == Game::OrbitPredictionService::PublishStage::PreviewStreaming)
        {
            EXPECT_FALSE(saw_finalizing_stage);
            streaming_results.push_back(&result);
            continue;
        }

        EXPECT_EQ(result.publish_stage, Game::OrbitPredictionService::PublishStage::PreviewFinalizing);
        saw_finalizing_stage = true;
        finalizing_results.push_back(&result);
    }

    ASSERT_FALSE(streaming_results.empty());
    ASSERT_FALSE(finalizing_results.empty());

    const Game::OrbitPredictionService::Result &first_streaming_result = *streaming_results.front();
    const Game::OrbitPredictionService::Result &last_streaming_result = *streaming_results.back();
    const Game::OrbitPredictionService::Result &first_finalizing_result = *finalizing_results.front();
    const Game::OrbitPredictionService::Result &last_finalizing_result = *finalizing_results.back();

    ASSERT_FALSE(first_streaming_result.published_chunks.empty());
    EXPECT_EQ(first_streaming_result.published_chunks.front().chunk_id, 0u);
    EXPECT_DOUBLE_EQ(first_streaming_result.published_chunks.front().t0_s, 20.0);
    EXPECT_LE(last_streaming_result.published_chunks.back().t1_s, 40.0 + 1.0e-6);
    for (const Game::OrbitPredictionService::Result *result : streaming_results)
    {
        for (const Game::OrbitPredictionService::PublishedChunk &chunk : result->published_chunks)
        {
            EXPECT_EQ(chunk.quality_state, Game::OrbitPredictionService::ChunkQualityState::PreviewPatch);
        }
    }
    ASSERT_FALSE(last_streaming_result.trajectory_segments_inertial_planned.empty());
    EXPECT_DOUBLE_EQ(last_streaming_result.trajectory_segments_inertial_planned.front().t0_s, 20.0);
    EXPECT_LE(last_streaming_result.trajectory_segments_inertial_planned.back().t0_s +
                      last_streaming_result.trajectory_segments_inertial_planned.back().dt_s,
              40.0 + 1.0e-6);
    bool saw_stream_impulse_preview = false;
    for (const Game::OrbitPredictionService::Result *result : streaming_results)
    {
        for (const Game::OrbitPredictionService::ManeuverNodePreview &preview : result->maneuver_previews)
        {
            saw_stream_impulse_preview |= preview.node_id == 1;
        }
    }
    EXPECT_TRUE(saw_stream_impulse_preview);

    ASSERT_FALSE(first_finalizing_result.published_chunks.empty());
    EXPECT_GE(first_finalizing_result.published_chunks.front().chunk_id,
              last_streaming_result.published_chunks.back().chunk_id + 1u);
    EXPECT_GE(first_finalizing_result.published_chunks.front().t0_s, 40.0 - 1.0e-6);
    for (const Game::OrbitPredictionService::Result *result : finalizing_results)
    {
        for (const Game::OrbitPredictionService::PublishedChunk &chunk : result->published_chunks)
        {
            EXPECT_EQ(chunk.quality_state, Game::OrbitPredictionService::ChunkQualityState::Final);
        }
    }
    ASSERT_FALSE(first_finalizing_result.trajectory_segments_inertial_planned.empty());
    ASSERT_FALSE(last_finalizing_result.trajectory_segments_inertial_planned.empty());
    EXPECT_LE(first_finalizing_result.trajectory_segments_inertial_planned.front().t0_s, 20.0 + 1.0e-6);
    EXPECT_GE(last_finalizing_result.trajectory_segments_inertial_planned.back().t0_s +
                      last_finalizing_result.trajectory_segments_inertial_planned.back().dt_s,
              60.0);
    EXPECT_GE(first_finalizing_result.trajectory_segments_inertial_planned.size(),
              last_streaming_result.trajectory_segments_inertial_planned.size());
    bool saw_finalizing_impulse_preview = false;
    bool saw_preview_impulse_in_finalizing = false;
    for (const Game::OrbitPredictionService::Result *result : finalizing_results)
    {
        for (const Game::OrbitPredictionService::ManeuverNodePreview &preview : result->maneuver_previews)
        {
            saw_finalizing_impulse_preview |= preview.node_id == 2;
            saw_preview_impulse_in_finalizing |= preview.node_id == 1;
        }
    }
    EXPECT_TRUE(saw_finalizing_impulse_preview);
    EXPECT_TRUE(saw_preview_impulse_in_finalizing);
}

TEST(GameplayPredictionManeuverTests, PredictionServiceFinalizingStageRetainsPreviewPrefixForAnchorReuse)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request = make_prediction_request(0.0, 60.0);
    request.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    request.preview_patch.active = true;
    request.preview_patch.anchor_state_valid = true;
    request.preview_patch.anchor_time_s = 20.0;
    request.preview_patch.visual_window_s = 10.0;
    request.preview_patch.exact_window_s = 10.0;
    request.preview_patch.anchor_state_inertial =
            orbitsim::make_state(request.ship_bary_position_m, request.ship_bary_velocity_mps);

    Game::OrbitPredictionService::ManeuverImpulse preview_impulse{};
    preview_impulse.node_id = 1;
    preview_impulse.t_s = 25.0;
    preview_impulse.primary_body_id = 1;
    preview_impulse.dv_rtn_mps = glm::dvec3(0.0, 0.0, 10.0);
    request.maneuver_impulses.push_back(preview_impulse);

    Game::OrbitPredictionService::ManeuverImpulse tail_impulse{};
    tail_impulse.node_id = 2;
    tail_impulse.t_s = 45.0;
    tail_impulse.primary_body_id = 1;
    tail_impulse.dv_rtn_mps = glm::dvec3(0.0, 0.0, 15.0);
    request.maneuver_impulses.push_back(tail_impulse);

    const std::vector<Game::OrbitPredictionService::Result> results =
            run_prediction_results(service, 1, std::move(request));

    const auto finalizing_it = std::find_if(results.begin(),
                                            results.end(),
                                            [](const Game::OrbitPredictionService::Result &result) {
                                                return result.publish_stage ==
                                                       Game::OrbitPredictionService::PublishStage::PreviewFinalizing;
                                            });
    ASSERT_NE(finalizing_it, results.end());
    ASSERT_TRUE(finalizing_it->valid);
    ASSERT_GE(finalizing_it->trajectory_inertial_planned.size(), 3u);
    ASSERT_FALSE(finalizing_it->trajectory_segments_inertial_planned.empty());
    EXPECT_LE(finalizing_it->trajectory_segments_inertial_planned.front().t0_s, 20.0);

    bool saw_preview_impulse = false;
    bool saw_tail_impulse = false;
    for (const Game::OrbitPredictionService::ManeuverNodePreview &preview : finalizing_it->maneuver_previews)
    {
        saw_preview_impulse = saw_preview_impulse || preview.node_id == preview_impulse.node_id;
        saw_tail_impulse = saw_tail_impulse || preview.node_id == tail_impulse.node_id;
    }
    EXPECT_TRUE(saw_preview_impulse);
    EXPECT_TRUE(saw_tail_impulse);
}

TEST(GameplayPredictionManeuverTests, PredictionServiceReusesPlannedChunksWhenRequestIsUnchanged)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request =
            make_prediction_request(0.0, 2.0 * Game::OrbitPredictionTuning::kSecondsPerDay);

    Game::OrbitPredictionService::ManeuverImpulse first{};
    first.node_id = 1;
    first.t_s = 6.0 * Game::OrbitPredictionTuning::kSecondsPerHour;
    first.primary_body_id = 1;
    first.dv_rtn_mps = glm::dvec3(0.0, 0.0, 5.0);
    request.maneuver_impulses.push_back(first);

    Game::OrbitPredictionService::ManeuverImpulse second{};
    second.node_id = 2;
    second.t_s = 30.0 * Game::OrbitPredictionTuning::kSecondsPerHour;
    second.primary_body_id = 1;
    second.dv_rtn_mps = glm::dvec3(0.0, 2.0, 0.0);
    request.maneuver_impulses.push_back(second);

    const std::vector<Game::OrbitPredictionService::Result> first_results =
            run_prediction_results(service, 1, request, 1);
    ASSERT_EQ(first_results.size(), 1u);
    ASSERT_TRUE(first_results.front().valid);
    ASSERT_FALSE(first_results.front().published_chunks.empty());
    for (const Game::OrbitPredictionService::PublishedChunk &chunk : first_results.front().published_chunks)
    {
        EXPECT_FALSE(chunk.reused_from_cache);
    }

    const std::vector<Game::OrbitPredictionService::Result> second_results =
            run_prediction_results(service, 2, request, 1);
    ASSERT_EQ(second_results.size(), 1u);
    ASSERT_TRUE(second_results.front().valid);
    ASSERT_FALSE(second_results.front().published_chunks.empty());
    EXPECT_TRUE(second_results.front().diagnostics.trajectory_planned.cache_reused);
    for (const Game::OrbitPredictionService::PublishedChunk &chunk : second_results.front().published_chunks)
    {
        EXPECT_TRUE(chunk.reused_from_cache);
    }
}

TEST(GameplayPredictionManeuverTests, PredictionServiceInvalidatesOnlyChunksDownstreamOfChangedManeuver)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request =
            make_prediction_request(0.0, 3.0 * Game::OrbitPredictionTuning::kSecondsPerDay);

    Game::OrbitPredictionService::ManeuverImpulse first{};
    first.node_id = 1;
    first.t_s = 6.0 * Game::OrbitPredictionTuning::kSecondsPerHour;
    first.primary_body_id = 1;
    first.dv_rtn_mps = glm::dvec3(0.0, 0.0, 5.0);
    request.maneuver_impulses.push_back(first);

    Game::OrbitPredictionService::ManeuverImpulse second{};
    second.node_id = 2;
    second.t_s = 36.0 * Game::OrbitPredictionTuning::kSecondsPerHour;
    second.primary_body_id = 1;
    second.dv_rtn_mps = glm::dvec3(0.0, 2.0, 0.0);
    request.maneuver_impulses.push_back(second);

    const std::vector<Game::OrbitPredictionService::Result> first_results =
            run_prediction_results(service, 1, request, 1);
    ASSERT_EQ(first_results.size(), 1u);
    ASSERT_TRUE(first_results.front().valid);

    Game::OrbitPredictionService::Request changed_request = request;
    changed_request.maneuver_impulses.back().dv_rtn_mps = glm::dvec3(0.0, 3.0, 0.0);

    const std::vector<Game::OrbitPredictionService::Result> second_results =
            run_prediction_results(service, 2, changed_request, 1);
    ASSERT_EQ(second_results.size(), 1u);
    ASSERT_TRUE(second_results.front().valid);
    ASSERT_FALSE(second_results.front().published_chunks.empty());
    EXPECT_TRUE(second_results.front().diagnostics.trajectory_planned.cache_reused);

    const double changed_time_s = changed_request.maneuver_impulses.back().t_s;
    bool saw_reused_upstream = false;
    bool saw_resolved_downstream = false;
    for (const Game::OrbitPredictionService::PublishedChunk &chunk : second_results.front().published_chunks)
    {
        if (chunk.t1_s <= changed_time_s + 1.0e-6)
        {
            EXPECT_TRUE(chunk.reused_from_cache);
            saw_reused_upstream = true;
        }
        if (chunk.t0_s >= changed_time_s - 1.0e-6)
        {
            EXPECT_FALSE(chunk.reused_from_cache);
            saw_resolved_downstream = true;
        }
    }

    EXPECT_TRUE(saw_reused_upstream);
    EXPECT_TRUE(saw_resolved_downstream);
}

TEST(GameplayPredictionManeuverTests, PredictionServiceBuildsChunkScopedEphemeridesForPlannedSolve)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request =
            make_prediction_request(0.0, 5.0 * Game::OrbitPredictionTuning::kSecondsPerDay);

    Game::OrbitPredictionService::ManeuverImpulse maneuver{};
    maneuver.node_id = 1;
    maneuver.t_s = 2.0 * Game::OrbitPredictionTuning::kSecondsPerHour;
    maneuver.primary_body_id = 1;
    maneuver.dv_rtn_mps = glm::dvec3(0.0, 0.0, 5.0);
    request.maneuver_impulses.push_back(maneuver);

    const std::vector<Game::OrbitPredictionService::Result> results =
            run_prediction_results(service, 1, request, 1);
    ASSERT_EQ(results.size(), 1u);
    ASSERT_TRUE(results.front().valid);

    std::lock_guard<std::mutex> lock(service._ephemeris_mutex);
    ASSERT_FALSE(service._ephemeris_cache.empty());

    bool saw_full_window_ephemeris = false;
    bool saw_chunk_scoped_ephemeris = false;
    for (const Game::OrbitPredictionService::CachedEphemerisEntry &entry : service._ephemeris_cache)
    {
        if (std::abs(entry.sim_time_s - request.sim_time_s) <= 1.0e-9 &&
            std::abs(entry.duration_s - request.future_window_s) <= 1.0e-6)
        {
            saw_full_window_ephemeris = true;
        }
        if (entry.sim_time_s > (request.sim_time_s + 1.0e-6) &&
            entry.duration_s < (request.future_window_s - 1.0e-6))
        {
            saw_chunk_scoped_ephemeris = true;
        }
    }

    EXPECT_TRUE(saw_full_window_ephemeris);
    EXPECT_TRUE(saw_chunk_scoped_ephemeris);
}

TEST(GameplayPredictionManeuverTests, PredictionServiceRejectsResultsFromPreviousResetEpoch)
{
    Game::OrbitPredictionService::PendingJob stale_job{};
    stale_job.track_id = 77;
    stale_job.request_epoch = 3;
    stale_job.generation_id = 12;

    const std::unordered_map<uint64_t, uint64_t> latest_requested_generation_by_track{};
    EXPECT_FALSE(Game::OrbitPredictionService::should_publish_result(stale_job, 4, latest_requested_generation_by_track));

    stale_job.request_epoch = 4;
    EXPECT_TRUE(Game::OrbitPredictionService::should_publish_result(stale_job, 4, latest_requested_generation_by_track));

    const std::unordered_map<uint64_t, uint64_t> superseded_generation{{stale_job.track_id, stale_job.generation_id + 1}};
    EXPECT_FALSE(Game::OrbitPredictionService::should_publish_result(stale_job, 4, superseded_generation));
}

