#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <gtest/gtest.h>

#include <cmath>
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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
