#include "gameplay_prediction_maneuver_test_common.h"
#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <algorithm>
#include <functional>
#include <utility>

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

    add_maneuver_impulse(request, 1, 10.0, glm::dvec3(0.0, 0.0, 50.0));
    add_maneuver_impulse(request, 2, 20.0, glm::dvec3(0.0, 0.0, 0.0));

    std::vector<Game::OrbitPredictionService::Result> results = run_prediction_results(service, 1, std::move(request));
    ASSERT_EQ(results.size(), 1u);
    const Game::OrbitPredictionService::Result &result = results.front();

    ASSERT_TRUE(result.valid) << "status=" << static_cast<int>(result.diagnostics.status)
                              << " ephemeris_accepted=" << result.diagnostics.ephemeris.accepted_segments
                              << " base_accepted=" << result.diagnostics.trajectory_base.accepted_segments
                              << " planned_accepted=" << result.diagnostics.trajectory_planned.accepted_segments;
    ASSERT_GE(result.maneuver_previews.size(), 2u);

    const auto *first_preview = find_maneuver_preview(result, 1);
    const auto *second_preview = find_maneuver_preview(result, 2);
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

    constexpr int impulse_node_id = 1;
    constexpr double impulse_time_s = 50'000.0;
    add_maneuver_impulse(request, impulse_node_id, impulse_time_s, glm::dvec3(0.0, 0.0, 25.0));

    std::vector<Game::OrbitPredictionService::Result> results = run_prediction_results(service, 1, std::move(request));
    ASSERT_EQ(results.size(), 1u);
    const Game::OrbitPredictionService::Result &result = results.front();

    ASSERT_TRUE(result.valid) << "status=" << static_cast<int>(result.diagnostics.status)
                              << " ephemeris_accepted=" << result.diagnostics.ephemeris.accepted_segments
                              << " base_accepted=" << result.diagnostics.trajectory_base.accepted_segments
                              << " planned_accepted=" << result.diagnostics.trajectory_planned.accepted_segments;
    ASSERT_FALSE(result.trajectory_segments_inertial_planned.empty());
    ASSERT_FALSE(result.maneuver_previews.empty());

    const auto *preview = find_maneuver_preview(result, impulse_node_id);
    ASSERT_NE(preview, nullptr);
    EXPECT_TRUE(preview->valid);

    const orbitsim::TrajectorySegment &last_planned = result.trajectory_segments_inertial_planned.back();
    const double planned_end_s = last_planned.t0_s + last_planned.dt_s;
    EXPECT_GE(planned_end_s, impulse_time_s);
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

    add_maneuver_impulse(preview_request, 1, 20.0, glm::dvec3(0.0, 0.0, 10.0));

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

TEST(GameplayPredictionManeuverTests, PredictionServicePublishesPreviewPatchThenFullFinalStage)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request = make_prediction_request(0.0, 60.0);
    request.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    request.preview_patch.active = true;
    request.preview_patch.anchor_state_valid = true;
    request.preview_patch.anchor_state_trusted = true;
    request.preview_patch.anchor_time_s = 20.0;
    request.preview_patch.visual_window_s = 10.0;
    request.preview_patch.exact_window_s = request.preview_patch.visual_window_s;
    request.preview_patch.anchor_state_inertial =
            orbitsim::make_state(request.ship_bary_position_m, request.ship_bary_velocity_mps);

    add_maneuver_impulse(request, 1, 25.0, glm::dvec3(0.0, 0.0, 10.0));
    add_maneuver_impulse(request, 2, 45.0, glm::dvec3(0.0, 0.0, 15.0));

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
    EXPECT_EQ(first_finalizing_result.published_chunks.front().chunk_id, 0u);
    EXPECT_LE(first_finalizing_result.published_chunks.front().t0_s, 1.0e-6);
    EXPECT_GE(first_finalizing_result.published_chunks.back().t1_s, 60.0);
    EXPECT_GT(first_finalizing_result.published_chunks.size(),
              last_streaming_result.published_chunks.size());
    bool saw_final_prefix_chunk = false;
    bool saw_final_preview_chunk = false;
    bool saw_final_tail_chunk = false;
    for (const Game::OrbitPredictionService::Result *result : finalizing_results)
    {
        for (const Game::OrbitPredictionService::PublishedChunk &chunk : result->published_chunks)
        {
            EXPECT_EQ(chunk.quality_state, Game::OrbitPredictionService::ChunkQualityState::Final);
            saw_final_prefix_chunk |= chunk.t0_s < (20.0 - 1.0e-6);
            saw_final_preview_chunk |=
                    chunk.t0_s >= (20.0 - 1.0e-6) &&
                    chunk.t1_s <= (40.0 + 1.0e-6);
            saw_final_tail_chunk |= chunk.t0_s >= (40.0 - 1.0e-6);
        }
    }
    EXPECT_TRUE(saw_final_prefix_chunk);
    EXPECT_TRUE(saw_final_preview_chunk);
    EXPECT_TRUE(saw_final_tail_chunk);
    ASSERT_FALSE(first_finalizing_result.trajectory_segments_inertial_planned.empty());
    ASSERT_FALSE(last_finalizing_result.trajectory_segments_inertial_planned.empty());
    EXPECT_LE(first_finalizing_result.trajectory_segments_inertial_planned.front().t0_s, 1.0e-6);
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
    request.preview_patch.anchor_state_trusted = true;
    request.preview_patch.anchor_time_s = 20.0;
    request.preview_patch.visual_window_s = 10.0;
    request.preview_patch.exact_window_s = 10.0;
    request.preview_patch.anchor_state_inertial =
            orbitsim::make_state(request.ship_bary_position_m, request.ship_bary_velocity_mps);

    const auto preview_impulse = make_maneuver_impulse(1, 25.0, glm::dvec3(0.0, 0.0, 10.0));
    const auto tail_impulse = make_maneuver_impulse(2, 45.0, glm::dvec3(0.0, 0.0, 15.0));
    request.maneuver_impulses.push_back(preview_impulse);
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

TEST(GameplayPredictionManeuverTests, PredictionServiceObservationPinsPlannedChunkReuseShape)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request =
            make_prediction_request(0.0, 2.0 * Game::OrbitPredictionTuning::kSecondsPerDay);

    add_maneuver_impulse(request,
                         1,
                         6.0 * Game::OrbitPredictionTuning::kSecondsPerHour,
                         glm::dvec3(0.0, 0.0, 5.0));
    add_maneuver_impulse(request,
                         2,
                         30.0 * Game::OrbitPredictionTuning::kSecondsPerHour,
                         glm::dvec3(0.0, 2.0, 0.0));

    const std::vector<Game::OrbitPredictionService::Result> first_results =
            run_prediction_results(service, 1, request, 1);
    ASSERT_EQ(first_results.size(), 1u);
    const Game::OrbitPredictionService::Result &first_result = first_results.front();
    ASSERT_TRUE(first_result.valid);
    ASSERT_GT(first_result.published_chunks.size(), 1u);
    ASSERT_FALSE(first_result.trajectory_segments_inertial_planned.empty());

    const std::vector<Game::OrbitPredictionService::Result> second_results =
            run_prediction_results(service, 2, request, 1);
    ASSERT_EQ(second_results.size(), 1u);
    const Game::OrbitPredictionService::Result &second_result = second_results.front();
    ASSERT_TRUE(second_result.valid);
    ASSERT_EQ(second_result.published_chunks.size(), first_result.published_chunks.size());

    EXPECT_TRUE(second_result.diagnostics.trajectory_planned.cache_reused);
    EXPECT_EQ(second_result.diagnostics.trajectory_segment_count_planned,
              first_result.diagnostics.trajectory_segment_count_planned);
    EXPECT_EQ(second_result.diagnostics.trajectory_sample_count_planned,
              first_result.diagnostics.trajectory_sample_count_planned);
    EXPECT_EQ(second_result.trajectory_segments_inertial_planned.size(),
              first_result.trajectory_segments_inertial_planned.size());
    EXPECT_EQ(second_result.maneuver_previews.size(), first_result.maneuver_previews.size());

    for (std::size_t i = 0; i < first_result.published_chunks.size(); ++i)
    {
        const Game::OrbitPredictionService::PublishedChunk &fresh_chunk = first_result.published_chunks[i];
        const Game::OrbitPredictionService::PublishedChunk &reused_chunk = second_result.published_chunks[i];

        EXPECT_EQ(reused_chunk.chunk_id, fresh_chunk.chunk_id);
        EXPECT_EQ(reused_chunk.quality_state, Game::OrbitPredictionService::ChunkQualityState::Final);
        EXPECT_EQ(reused_chunk.quality_state, fresh_chunk.quality_state);
        EXPECT_DOUBLE_EQ(reused_chunk.t0_s, fresh_chunk.t0_s);
        EXPECT_DOUBLE_EQ(reused_chunk.t1_s, fresh_chunk.t1_s);
        EXPECT_TRUE(reused_chunk.includes_planned_path);
        EXPECT_EQ(reused_chunk.includes_planned_path, fresh_chunk.includes_planned_path);
        EXPECT_FALSE(fresh_chunk.reused_from_cache);
        EXPECT_TRUE(reused_chunk.reused_from_cache);
    }
}

TEST(GameplayPredictionManeuverTests, PredictionServicePlannedSuffixRefineStartsAtSelectedNode)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request baseline_request = make_prediction_request(0.0, 120.0);

    const auto selected = make_maneuver_impulse(2, 20.0, glm::dvec3(0.0, 2.0, 0.0));
    const auto downstream = make_maneuver_impulse(3, 45.0, glm::dvec3(0.0, 0.0, 0.0));
    add_maneuver_impulse(baseline_request, 1, 10.0, glm::dvec3(0.0, 0.0, 5.0));
    baseline_request.maneuver_impulses.push_back(selected);
    baseline_request.maneuver_impulses.push_back(downstream);

    const std::vector<Game::OrbitPredictionService::Result> baseline_results =
            run_prediction_results(service, 1, baseline_request, 1);
    ASSERT_EQ(baseline_results.size(), 1u);
    const Game::OrbitPredictionService::Result &baseline_result = baseline_results.front();
    ASSERT_TRUE(baseline_result.valid);
    ASSERT_FALSE(baseline_result.trajectory_segments_inertial_planned.empty());

    const Game::OrbitPredictionService::ManeuverNodePreview *baseline_selected_preview =
            find_maneuver_preview(baseline_result, selected.node_id);
    const Game::OrbitPredictionService::ManeuverNodePreview *baseline_downstream_preview =
            find_maneuver_preview(baseline_result, downstream.node_id);
    ASSERT_NE(baseline_selected_preview, nullptr);
    ASSERT_NE(baseline_downstream_preview, nullptr);
    ASSERT_TRUE(baseline_selected_preview->valid);

    Game::OrbitPredictionService::Request refined_request = baseline_request;
    refined_request.maneuver_impulses[1].dv_rtn_mps = glm::dvec3(0.0, 6.0, 0.0);
    refined_request.planned_suffix_refine.active = true;
    refined_request.planned_suffix_refine.anchor_node_id = selected.node_id;
    refined_request.planned_suffix_refine.anchor_time_s = selected.t_s;
    refined_request.planned_suffix_refine.anchor_state_inertial =
            orbitsim::make_state(baseline_selected_preview->inertial_position_m,
                                 baseline_selected_preview->inertial_velocity_mps);
    refined_request.planned_suffix_refine.prefix_segments_inertial =
            Game::slice_trajectory_segments(baseline_result.trajectory_segments_inertial_planned,
                                            refined_request.sim_time_s,
                                            selected.t_s);
    for (const Game::OrbitPredictionService::ManeuverNodePreview &preview : baseline_result.maneuver_previews)
    {
        if (preview.t_s < (selected.t_s - 1.0e-6))
        {
            refined_request.planned_suffix_refine.prefix_previews.push_back(preview);
        }
    }

    const std::vector<Game::OrbitPredictionService::Result> refined_results =
            run_prediction_results(service, 2, refined_request, 1);
    ASSERT_EQ(refined_results.size(), 1u);
    const Game::OrbitPredictionService::Result &refined_result = refined_results.front();
    ASSERT_TRUE(refined_result.valid) << "status=" << static_cast<int>(refined_result.diagnostics.status);
    ASSERT_FALSE(refined_result.trajectory_segments_inertial_planned.empty());
    EXPECT_NEAR(refined_result.trajectory_segments_inertial_planned.front().t0_s,
                refined_request.sim_time_s,
                1.0e-6);
    EXPECT_GE(refined_result.trajectory_segments_inertial_planned.back().t0_s +
                      refined_result.trajectory_segments_inertial_planned.back().dt_s,
              refined_request.sim_time_s + refined_request.future_window_s);

    const auto first_resolved_chunk =
            std::find_if(refined_result.published_chunks.begin(),
                         refined_result.published_chunks.end(),
                         [](const Game::OrbitPredictionService::PublishedChunk &chunk) {
                             return !chunk.reused_from_cache;
                         });
    ASSERT_NE(first_resolved_chunk, refined_result.published_chunks.end());
    EXPECT_NEAR(first_resolved_chunk->t0_s, selected.t_s, 1.0e-6);

    const Game::OrbitPredictionService::ManeuverNodePreview *refined_downstream_preview =
            find_maneuver_preview(refined_result, downstream.node_id);
    ASSERT_NE(refined_downstream_preview, nullptr);
    const double downstream_velocity_delta_mps =
            glm::length(glm::dvec3(refined_downstream_preview->inertial_velocity_mps -
                                   baseline_downstream_preview->inertial_velocity_mps));
    EXPECT_GT(downstream_velocity_delta_mps, 0.1);
}

TEST(GameplayPredictionManeuverTests, PredictionServicePlannedSuffixRefineStreamsCachedPrefixFirst)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request baseline_request = make_prediction_request(0.0, 90.0);

    const auto selected = make_maneuver_impulse(2, 20.0, glm::dvec3(0.0, 2.0, 0.0));
    add_maneuver_impulse(baseline_request, 1, 10.0, glm::dvec3(0.0, 0.0, 1.0));
    baseline_request.maneuver_impulses.push_back(selected);

    const std::vector<Game::OrbitPredictionService::Result> baseline_results =
            run_prediction_results(service, 1, baseline_request, 1);
    ASSERT_EQ(baseline_results.size(), 1u);
    const Game::OrbitPredictionService::Result &baseline_result = baseline_results.front();
    ASSERT_TRUE(baseline_result.valid);
    ASSERT_FALSE(baseline_result.trajectory_segments_inertial_planned.empty());

    const Game::OrbitPredictionService::ManeuverNodePreview *selected_preview =
            find_valid_maneuver_preview(baseline_result, selected.node_id);
    ASSERT_NE(selected_preview, nullptr);

    Game::OrbitPredictionService::Request refined_request = baseline_request;
    refined_request.maneuver_impulses[1].dv_rtn_mps = glm::dvec3(0.0, 6.0, 0.0);
    refined_request.full_stream_publish.active = true;
    refined_request.planned_suffix_refine.active = true;
    refined_request.planned_suffix_refine.anchor_node_id = selected.node_id;
    refined_request.planned_suffix_refine.anchor_time_s = selected.t_s;
    refined_request.planned_suffix_refine.anchor_state_inertial =
            orbitsim::make_state(selected_preview->inertial_position_m,
                                 selected_preview->inertial_velocity_mps);
    refined_request.planned_suffix_refine.prefix_segments_inertial =
            Game::slice_trajectory_segments(baseline_result.trajectory_segments_inertial_planned,
                                            refined_request.sim_time_s,
                                            selected.t_s);
    ASSERT_FALSE(refined_request.planned_suffix_refine.prefix_segments_inertial.empty());
    for (const Game::OrbitPredictionService::ManeuverNodePreview &preview : baseline_result.maneuver_previews)
    {
        if (preview.t_s < (selected.t_s - 1.0e-6))
        {
            refined_request.planned_suffix_refine.prefix_previews.push_back(preview);
        }
    }

    const std::vector<Game::OrbitPredictionService::Result> refined_results =
            run_prediction_results(service, 2, refined_request, 1);
    ASSERT_GE(refined_results.size(), 2u);

    const Game::OrbitPredictionService::Result &prefix_result = refined_results.front();
    ASSERT_TRUE(prefix_result.valid);
    EXPECT_EQ(prefix_result.publish_stage, Game::OrbitPredictionService::PublishStage::FullStreaming);
    ASSERT_FALSE(prefix_result.published_chunks.empty());
    ASSERT_FALSE(prefix_result.streamed_planned_chunks.empty());
    EXPECT_TRUE(prefix_result.trajectory_segments_inertial_planned.empty());

    for (const Game::OrbitPredictionService::PublishedChunk &chunk : prefix_result.published_chunks)
    {
        EXPECT_TRUE(chunk.reused_from_cache);
        EXPECT_LE(chunk.t1_s, selected.t_s + 1.0e-6);
    }
    EXPECT_NEAR(prefix_result.published_chunks.front().t0_s, refined_request.sim_time_s, 1.0e-6);
    EXPECT_NEAR(prefix_result.published_chunks.back().t1_s, selected.t_s, 1.0e-6);

    const auto final_it =
            std::find_if(refined_results.begin(),
                         refined_results.end(),
                         [](const Game::OrbitPredictionService::Result &result) {
                             return result.publish_stage == Game::OrbitPredictionService::PublishStage::Final;
                         });
    ASSERT_NE(final_it, refined_results.end());
    ASSERT_TRUE(final_it->valid);
    ASSERT_FALSE(final_it->published_chunks.empty());
    const auto first_resolved_chunk =
            std::find_if(final_it->published_chunks.begin(),
                         final_it->published_chunks.end(),
                         [](const Game::OrbitPredictionService::PublishedChunk &chunk) {
                             return !chunk.reused_from_cache;
                         });
    ASSERT_NE(first_resolved_chunk, final_it->published_chunks.end());
    EXPECT_NEAR(first_resolved_chunk->t0_s, selected.t_s, 1.0e-6);
}

TEST(GameplayPredictionManeuverTests, PredictionServicePlannedSuffixRefineFallsBackOnBadPrefix)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request baseline_request = make_prediction_request(0.0, 90.0);

    const auto selected = make_maneuver_impulse(7, 20.0, glm::dvec3(0.0, 2.0, 0.0));
    baseline_request.maneuver_impulses.push_back(selected);

    const std::vector<Game::OrbitPredictionService::Result> baseline_results =
            run_prediction_results(service, 1, baseline_request, 1);
    ASSERT_EQ(baseline_results.size(), 1u);
    const Game::OrbitPredictionService::Result &baseline_result = baseline_results.front();
    ASSERT_TRUE(baseline_result.valid);
    ASSERT_FALSE(baseline_result.trajectory_segments_inertial_planned.empty());

    Game::OrbitPredictionService::Request refined_request = baseline_request;
    refined_request.maneuver_impulses.front().dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    refined_request.planned_suffix_refine.active = true;
    refined_request.planned_suffix_refine.anchor_node_id = selected.node_id;
    refined_request.planned_suffix_refine.anchor_time_s = selected.t_s;
    refined_request.planned_suffix_refine.prefix_segments_inertial =
            Game::slice_trajectory_segments(baseline_result.trajectory_segments_inertial_planned,
                                            refined_request.sim_time_s,
                                            selected.t_s);
    ASSERT_FALSE(refined_request.planned_suffix_refine.prefix_segments_inertial.empty());
    refined_request.planned_suffix_refine.anchor_state_inertial =
            refined_request.planned_suffix_refine.prefix_segments_inertial.back().end;
    refined_request.planned_suffix_refine.anchor_state_inertial.position_m.x += 1'000'000.0;

    const std::vector<Game::OrbitPredictionService::Result> refined_results =
            run_prediction_results(service, 2, refined_request, 1);
    ASSERT_EQ(refined_results.size(), 1u);
    const Game::OrbitPredictionService::Result &refined_result = refined_results.front();
    ASSERT_TRUE(refined_result.valid) << "status=" << static_cast<int>(refined_result.diagnostics.status);
    EXPECT_EQ(refined_result.diagnostics.status, Game::OrbitPredictionService::Status::Success);
    ASSERT_FALSE(refined_result.trajectory_segments_inertial_planned.empty());
    EXPECT_NEAR(refined_result.trajectory_segments_inertial_planned.front().t0_s,
                refined_request.sim_time_s,
                1.0e-6);
}

TEST(GameplayPredictionManeuverTests, PredictionServiceInvalidatesOnlyChunksDownstreamOfChangedManeuver)
{
    Game::OrbitPredictionService service{};
    Game::OrbitPredictionService::Request request =
            make_prediction_request(0.0, 3.0 * Game::OrbitPredictionTuning::kSecondsPerDay);

    add_maneuver_impulse(request,
                         1,
                         6.0 * Game::OrbitPredictionTuning::kSecondsPerHour,
                         glm::dvec3(0.0, 0.0, 5.0));
    add_maneuver_impulse(request,
                         2,
                         36.0 * Game::OrbitPredictionTuning::kSecondsPerHour,
                         glm::dvec3(0.0, 2.0, 0.0));

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

    add_maneuver_impulse(request,
                         1,
                         2.0 * Game::OrbitPredictionTuning::kSecondsPerHour,
                         glm::dvec3(0.0, 0.0, 5.0));

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

TEST(GameplayPredictionManeuverTests, PredictionServiceReusesCoveredEphemerisForLaterWindow)
{
    Game::OrbitPredictionService service{};
    const Game::OrbitPredictionService::Request first_request =
            make_celestial_prediction_request(0.0, 600.0);
    const Game::OrbitPredictionService::EphemerisSamplingSpec first_sampling_spec =
            Game::OrbitPredictionService::build_ephemeris_sampling_spec(first_request);
    ASSERT_TRUE(first_sampling_spec.valid);

    const Game::OrbitPredictionService::EphemerisBuildRequest first_build_request =
            Game::build_ephemeris_build_request(first_request, first_sampling_spec);

    bool first_reused = true;
    const Game::OrbitPredictionService::SharedCelestialEphemeris first_ephemeris =
            service.get_or_build_ephemeris(first_build_request,
                                           std::function<bool()>{},
                                           nullptr,
                                           &first_reused);
    ASSERT_TRUE(first_ephemeris);
    ASSERT_FALSE(first_ephemeris->empty());
    EXPECT_FALSE(first_reused);

    Game::OrbitPredictionService::Request later_request =
            make_celestial_prediction_request(30.0, 120.0);
    for (orbitsim::MassiveBody &body : later_request.massive_bodies)
    {
        body.state = first_ephemeris->body_state_at_by_id(body.id, later_request.sim_time_s);
    }

    const Game::OrbitPredictionService::EphemerisSamplingSpec later_sampling_spec =
            Game::OrbitPredictionService::build_ephemeris_sampling_spec(later_request);
    ASSERT_TRUE(later_sampling_spec.valid);
    const Game::OrbitPredictionService::EphemerisBuildRequest later_build_request =
            Game::build_ephemeris_build_request(later_request, later_sampling_spec);

    Game::OrbitPredictionService::AdaptiveStageDiagnostics later_diagnostics{};
    bool later_reused = false;
    const Game::OrbitPredictionService::SharedCelestialEphemeris later_ephemeris =
            service.get_or_build_ephemeris(later_build_request,
                                           std::function<bool()>{},
                                           &later_diagnostics,
                                           &later_reused);

    EXPECT_TRUE(later_reused);
    EXPECT_TRUE(later_diagnostics.cache_reused);
    EXPECT_DOUBLE_EQ(later_diagnostics.requested_duration_s, later_build_request.duration_s);
    EXPECT_EQ(later_ephemeris, first_ephemeris);

    std::lock_guard<std::mutex> lock(service._ephemeris_mutex);
    EXPECT_EQ(service._ephemeris_cache.size(), 1u);
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

TEST(GameplayPredictionManeuverTests, PredictionServiceInvalidatesStaleManeuverRevision)
{
    Game::OrbitPredictionService service{};
    constexpr uint64_t track_id = 77u;

    Game::OrbitPredictionService::Result completed{};
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

    Game::OrbitPredictionService::PendingJob stale_job{};
    stale_job.track_id = track_id;
    stale_job.request_epoch = service._request_epoch;
    stale_job.generation_id = 12u;
    stale_job.request.maneuver_plan_revision = 2u;

    Game::OrbitPredictionService::Result stale_result{};
    stale_result.track_id = track_id;
    stale_result.generation_id = 12u;
    stale_result.maneuver_plan_revision = 2u;
    EXPECT_FALSE(service.publish_completed_result(stale_job, std::move(stale_result)));
    EXPECT_TRUE(service._completed.empty());
}
