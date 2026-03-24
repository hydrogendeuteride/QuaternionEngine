#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

namespace
{
    constexpr double kEpsilonS = 1.0e-6;

    Game::OrbitPredictionService::Request make_request(const double sim_time_s, const double horizon_s)
    {
        Game::OrbitPredictionService::Request request{};
        request.sim_time_s = sim_time_s;
        request.future_window_s = horizon_s;
        return request;
    }

    bool has_boundary(const Game::OrbitPredictionService::PredictionSolvePlan &plan, const double time_s)
    {
        for (const Game::OrbitPredictionService::PredictionChunkPlan &chunk : plan.chunks)
        {
            if (std::abs(chunk.t0_s - time_s) <= kEpsilonS || std::abs(chunk.t1_s - time_s) <= kEpsilonS)
            {
                return true;
            }
        }
        return false;
    }

    std::optional<Game::OrbitPredictionService::PredictionChunkPlan> find_first_chunk_with_profile(
            const Game::OrbitPredictionService::PredictionSolvePlan &plan,
            const Game::OrbitPredictionService::PredictionProfileId profile_id)
    {
        for (const Game::OrbitPredictionService::PredictionChunkPlan &chunk : plan.chunks)
        {
            if (chunk.profile_id == profile_id)
            {
                return chunk;
            }
        }
        return std::nullopt;
    }

    std::optional<Game::OrbitPredictionService::PredictionChunkPlan> find_first_chunk_with_flags(
            const Game::OrbitPredictionService::PredictionSolvePlan &plan,
            const uint32_t flags)
    {
        for (const Game::OrbitPredictionService::PredictionChunkPlan &chunk : plan.chunks)
        {
            if ((chunk.boundary_flags & flags) == flags)
            {
                return chunk;
            }
        }
        return std::nullopt;
    }
} // namespace

TEST(OrbitPredictionPlannerTests, BuildsStableChunksForReferenceHorizons)
{
    const Game::OrbitPredictionService::PredictionSolvePlan plan_180d =
            Game::build_prediction_solve_plan(
                    make_request(10.0, 180.0 * Game::OrbitPredictionTuning::kSecondsPerDay));
    ASSERT_TRUE(plan_180d.valid);
    EXPECT_EQ(plan_180d.chunks.size(), 69u);
    EXPECT_DOUBLE_EQ(plan_180d.t0_s, 10.0);
    EXPECT_DOUBLE_EQ(plan_180d.t1_s, 10.0 + 180.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
    EXPECT_EQ(plan_180d.chunks.front().profile_id, Game::OrbitPredictionService::PredictionProfileId::NearBody);
    EXPECT_EQ(plan_180d.chunks.back().profile_id, Game::OrbitPredictionService::PredictionProfileId::Cruise);

    const Game::OrbitPredictionService::PredictionSolvePlan plan_5y =
            Game::build_prediction_solve_plan(
                    make_request(20.0, 5.0 * Game::OrbitPredictionTuning::kSecondsPerYear));
    ASSERT_TRUE(plan_5y.valid);
    EXPECT_EQ(plan_5y.chunks.size(), 101u);
    EXPECT_EQ(plan_5y.chunks.back().profile_id, Game::OrbitPredictionService::PredictionProfileId::DeepTail);

    const Game::OrbitPredictionService::PredictionSolvePlan plan_20y =
            Game::build_prediction_solve_plan(
                    make_request(30.0, 20.0 * Game::OrbitPredictionTuning::kSecondsPerYear));
    ASSERT_TRUE(plan_20y.valid);
    EXPECT_EQ(plan_20y.chunks.size(), 161u);
    EXPECT_EQ(plan_20y.chunks.back().profile_id, Game::OrbitPredictionService::PredictionProfileId::DeepTail);
}

TEST(OrbitPredictionPlannerTests, InsertsManeuverAndPreviewBoundaries)
{
    using Flags = Game::OrbitPredictionService::PredictionChunkBoundaryFlags;
    using Profile = Game::OrbitPredictionService::PredictionProfileId;

    Game::OrbitPredictionService::Request request =
            make_request(1'000.0, 10.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
    request.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    request.preview_patch.active = true;
    request.preview_patch.anchor_state_valid = true;
    request.preview_patch.anchor_time_s = request.sim_time_s + 2.0 * Game::OrbitPredictionTuning::kSecondsPerHour;
    request.preview_patch.patch_window_s = 0.5 * Game::OrbitPredictionTuning::kSecondsPerHour;

    Game::OrbitPredictionService::ManeuverImpulse maneuver_a{};
    maneuver_a.node_id = 1;
    maneuver_a.t_s = request.sim_time_s + 1.0 * Game::OrbitPredictionTuning::kSecondsPerHour;

    Game::OrbitPredictionService::ManeuverImpulse maneuver_b{};
    maneuver_b.node_id = 2;
    maneuver_b.t_s = request.sim_time_s + 5.0 * Game::OrbitPredictionTuning::kSecondsPerDay;

    request.maneuver_impulses.push_back(maneuver_a);
    request.maneuver_impulses.push_back(maneuver_b);

    const Game::OrbitPredictionService::PredictionSolvePlan plan = Game::build_prediction_solve_plan(request);
    ASSERT_TRUE(plan.valid);

    EXPECT_TRUE(has_boundary(plan, request.sim_time_s));
    EXPECT_TRUE(has_boundary(plan, request.preview_patch.anchor_time_s));
    EXPECT_TRUE(has_boundary(plan, request.preview_patch.anchor_time_s + request.preview_patch.patch_window_s));
    EXPECT_TRUE(has_boundary(plan, request.preview_patch.anchor_time_s + request.preview_patch.patch_window_s * 2.0));
    EXPECT_TRUE(has_boundary(plan, request.maneuver_impulses[0].t_s));
    EXPECT_TRUE(has_boundary(plan, request.maneuver_impulses[1].t_s));

    std::vector<Game::OrbitPredictionService::PredictionChunkPlan> interactive_chunks;
    for (const Game::OrbitPredictionService::PredictionChunkPlan &chunk : plan.chunks)
    {
        if (chunk.profile_id == Profile::InteractiveExact)
        {
            interactive_chunks.push_back(chunk);
        }
    }

    ASSERT_EQ(interactive_chunks.size(), 2u);
    EXPECT_NE((interactive_chunks[0].boundary_flags & static_cast<uint32_t>(Flags::PreviewAnchor)), 0u);
    EXPECT_NE((interactive_chunks[0].boundary_flags & static_cast<uint32_t>(Flags::PreviewChunk)), 0u);
    EXPECT_NE((interactive_chunks[1].boundary_flags & static_cast<uint32_t>(Flags::PreviewChunk)), 0u);
    EXPECT_FALSE(interactive_chunks[0].allow_reuse);
    EXPECT_FALSE(interactive_chunks[1].allow_reuse);
}

TEST(OrbitPredictionPlannerTests, ResolvesChunkAdaptiveOptionsPerProfile)
{
    using Profile = Game::OrbitPredictionService::PredictionProfileId;

    Game::OrbitPredictionService::Request request =
            make_request(0.0, 6.0 * Game::OrbitPredictionTuning::kSecondsPerYear);

    const Game::OrbitPredictionService::PredictionSolvePlan plan = Game::build_prediction_solve_plan(request);
    ASSERT_TRUE(plan.valid);

    const auto near_chunk = find_first_chunk_with_profile(plan, Profile::NearBody);
    const auto transfer_chunk = find_first_chunk_with_profile(plan, Profile::Transfer);
    const auto cruise_chunk = find_first_chunk_with_profile(plan, Profile::Cruise);
    const auto deep_tail_chunk = find_first_chunk_with_profile(plan, Profile::DeepTail);
    ASSERT_TRUE(near_chunk.has_value());
    ASSERT_TRUE(transfer_chunk.has_value());
    ASSERT_TRUE(cruise_chunk.has_value());
    ASSERT_TRUE(deep_tail_chunk.has_value());

    const auto near_def = Game::resolve_prediction_profile_definition(request, *near_chunk);
    const auto transfer_def = Game::resolve_prediction_profile_definition(request, *transfer_chunk);
    const auto cruise_def = Game::resolve_prediction_profile_definition(request, *cruise_chunk);
    const auto deep_tail_def = Game::resolve_prediction_profile_definition(request, *deep_tail_chunk);

    EXPECT_LT(near_def.max_dt_s, transfer_def.max_dt_s);
    EXPECT_LT(transfer_def.max_dt_s, cruise_def.max_dt_s);
    EXPECT_LT(cruise_def.max_dt_s, deep_tail_def.max_dt_s);
    EXPECT_GT(near_def.soft_max_segments, transfer_def.soft_max_segments);
    EXPECT_GT(transfer_def.soft_max_segments, cruise_def.soft_max_segments);
    EXPECT_GT(cruise_def.soft_max_segments, deep_tail_def.soft_max_segments);
    EXPECT_GT(near_def.output_sample_density_scale, transfer_def.output_sample_density_scale);
    EXPECT_GT(transfer_def.output_sample_density_scale, cruise_def.output_sample_density_scale);
    EXPECT_GT(cruise_def.output_sample_density_scale, deep_tail_def.output_sample_density_scale);

    const auto near_segment_opt =
            Game::build_spacecraft_adaptive_segment_options_for_chunk(request, *near_chunk);
    const auto deep_tail_segment_opt =
            Game::build_spacecraft_adaptive_segment_options_for_chunk(request, *deep_tail_chunk);
    const auto near_ephemeris_opt =
            Game::build_adaptive_ephemeris_options_for_chunk(request, *near_chunk);
    const auto deep_tail_ephemeris_opt =
            Game::build_adaptive_ephemeris_options_for_chunk(request, *deep_tail_chunk);

    EXPECT_DOUBLE_EQ(near_segment_opt.duration_s, near_chunk->t1_s - near_chunk->t0_s);
    EXPECT_DOUBLE_EQ(deep_tail_segment_opt.duration_s, deep_tail_chunk->t1_s - deep_tail_chunk->t0_s);
    EXPECT_LT(near_segment_opt.max_dt_s, deep_tail_segment_opt.max_dt_s);
    EXPECT_GT(near_segment_opt.soft_max_segments, deep_tail_segment_opt.soft_max_segments);
    EXPECT_LT(near_ephemeris_opt.max_dt_s, deep_tail_ephemeris_opt.max_dt_s);
    EXPECT_GT(near_ephemeris_opt.soft_max_segments, deep_tail_ephemeris_opt.soft_max_segments);

    const std::size_t near_sample_budget =
            Game::prediction_sample_budget_for_chunk(request, *near_chunk, 256u);
    const std::size_t deep_tail_sample_budget =
            Game::prediction_sample_budget_for_chunk(request, *deep_tail_chunk, 256u);
    EXPECT_GT(near_sample_budget, deep_tail_sample_budget);
}

TEST(OrbitPredictionPlannerTests, PreviewAndManeuverChunksTightenProfiles)
{
    using Flags = Game::OrbitPredictionService::PredictionChunkBoundaryFlags;
    using Profile = Game::OrbitPredictionService::PredictionProfileId;

    Game::OrbitPredictionService::Request request =
            make_request(500.0, 40.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
    request.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    request.preview_patch.active = true;
    request.preview_patch.anchor_state_valid = true;
    request.preview_patch.anchor_time_s = request.sim_time_s + 2.0 * Game::OrbitPredictionTuning::kSecondsPerHour;
    request.preview_patch.patch_window_s = 0.5 * Game::OrbitPredictionTuning::kSecondsPerHour;

    Game::OrbitPredictionService::ManeuverImpulse maneuver{};
    maneuver.node_id = 7;
    maneuver.t_s = request.sim_time_s + 5.0 * Game::OrbitPredictionTuning::kSecondsPerDay;
    request.maneuver_impulses.push_back(maneuver);

    const Game::OrbitPredictionService::PredictionSolvePlan plan = Game::build_prediction_solve_plan(request);
    ASSERT_TRUE(plan.valid);

    const auto interactive_chunk = find_first_chunk_with_profile(plan, Profile::InteractiveExact);
    ASSERT_TRUE(interactive_chunk.has_value());

    const auto maneuver_chunk = find_first_chunk_with_flags(plan, static_cast<uint32_t>(Flags::Maneuver));
    ASSERT_TRUE(maneuver_chunk.has_value());

    const auto interactive_def = Game::resolve_prediction_profile_definition(request, *interactive_chunk);
    const auto maneuver_def = Game::resolve_prediction_profile_definition(request, *maneuver_chunk);

    EXPECT_LT(interactive_def.integrator_tolerance_multiplier, 1.0);
    EXPECT_LT(interactive_def.max_dt_s, maneuver_def.max_dt_s);
    EXPECT_GT(interactive_def.output_sample_density_scale, maneuver_def.output_sample_density_scale);

    const auto interactive_opt =
            Game::build_spacecraft_adaptive_segment_options_for_chunk(request, *interactive_chunk);
    const auto maneuver_opt =
            Game::build_spacecraft_adaptive_segment_options_for_chunk(request, *maneuver_chunk);
    EXPECT_LT(interactive_opt.max_dt_s, maneuver_opt.max_dt_s);
    EXPECT_LE(interactive_opt.lookup_max_dt_s, maneuver_opt.lookup_max_dt_s);
    EXPECT_GT(interactive_opt.soft_max_segments, maneuver_opt.soft_max_segments);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
