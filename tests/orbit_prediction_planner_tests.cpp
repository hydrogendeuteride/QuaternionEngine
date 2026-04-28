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

    orbitsim::TrajectorySegment make_activity_segment(const double t0_s,
                                                      const double t1_s,
                                                      const orbitsim::Vec3 &p0_m,
                                                      const orbitsim::Vec3 &v0_mps,
                                                      const orbitsim::Vec3 &p1_m,
                                                      const orbitsim::Vec3 &v1_mps)
    {
        orbitsim::TrajectorySegment segment{};
        segment.t0_s = t0_s;
        segment.dt_s = t1_s - t0_s;
        segment.start = orbitsim::make_state(p0_m, v0_mps);
        segment.end = orbitsim::make_state(p1_m, v1_mps);
        return segment;
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
    EXPECT_EQ(plan_180d.chunks.size(), 141u);
    EXPECT_DOUBLE_EQ(plan_180d.t0_s, 10.0);
    EXPECT_DOUBLE_EQ(plan_180d.t1_s, 10.0 + 180.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
    EXPECT_EQ(plan_180d.chunks.front().profile_id, Game::OrbitPredictionService::PredictionProfileId::Near);
    EXPECT_EQ(plan_180d.chunks.back().profile_id, Game::OrbitPredictionService::PredictionProfileId::Tail);

    const Game::OrbitPredictionService::PredictionSolvePlan plan_5y =
            Game::build_prediction_solve_plan(
                    make_request(20.0, 5.0 * Game::OrbitPredictionTuning::kSecondsPerYear));
    ASSERT_TRUE(plan_5y.valid);
    EXPECT_EQ(plan_5y.chunks.size(), 233u);
    EXPECT_EQ(plan_5y.chunks.back().profile_id, Game::OrbitPredictionService::PredictionProfileId::Tail);

    const Game::OrbitPredictionService::PredictionSolvePlan plan_20y =
            Game::build_prediction_solve_plan(
                    make_request(30.0, 20.0 * Game::OrbitPredictionTuning::kSecondsPerYear));
    ASSERT_TRUE(plan_20y.valid);
    EXPECT_EQ(plan_20y.chunks.size(), 415u);
    EXPECT_EQ(plan_20y.chunks.back().profile_id, Game::OrbitPredictionService::PredictionProfileId::Tail);
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
    request.preview_patch.visual_window_s = 0.5 * Game::OrbitPredictionTuning::kSecondsPerHour;
    request.preview_patch.exact_window_s = request.preview_patch.visual_window_s;

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
    EXPECT_TRUE(has_boundary(plan, request.preview_patch.anchor_time_s + request.preview_patch.exact_window_s));
    EXPECT_TRUE(has_boundary(plan, request.preview_patch.anchor_time_s + request.preview_patch.exact_window_s * 2.0));
    EXPECT_TRUE(has_boundary(plan, request.maneuver_impulses[0].t_s));
    EXPECT_TRUE(has_boundary(plan, request.maneuver_impulses[1].t_s));

    std::vector<Game::OrbitPredictionService::PredictionChunkPlan> exact_chunks;
    for (const Game::OrbitPredictionService::PredictionChunkPlan &chunk : plan.chunks)
    {
        if (chunk.profile_id == Profile::Exact)
        {
            exact_chunks.push_back(chunk);
        }
    }

    ASSERT_EQ(exact_chunks.size(), 2u);
    EXPECT_NE((exact_chunks[0].boundary_flags & static_cast<uint32_t>(Flags::PreviewAnchor)), 0u);
    EXPECT_NE((exact_chunks[0].boundary_flags & static_cast<uint32_t>(Flags::PreviewChunk)), 0u);
    EXPECT_NE((exact_chunks[1].boundary_flags & static_cast<uint32_t>(Flags::PreviewChunk)), 0u);
    EXPECT_TRUE(exact_chunks[0].allow_reuse);
    EXPECT_TRUE(exact_chunks[1].allow_reuse);

    std::size_t preview_chunk_count = 0u;
    std::size_t preview_anchor_count = 0u;
    for (const Game::OrbitPredictionService::PredictionChunkPlan &chunk : plan.chunks)
    {
        const bool has_preview_chunk_flag =
                (chunk.boundary_flags & static_cast<uint32_t>(Flags::PreviewChunk)) != 0u;
        const bool has_preview_anchor_flag =
                (chunk.boundary_flags & static_cast<uint32_t>(Flags::PreviewAnchor)) != 0u;

        preview_chunk_count += has_preview_chunk_flag ? 1u : 0u;
        preview_anchor_count += has_preview_anchor_flag ? 1u : 0u;

        if (has_preview_chunk_flag)
        {
            EXPECT_EQ(chunk.profile_id, Profile::Exact);
            EXPECT_GE(chunk.t0_s + kEpsilonS, request.preview_patch.anchor_time_s);
            EXPECT_LE(chunk.t1_s,
                      request.preview_patch.anchor_time_s + (2.0 * request.preview_patch.exact_window_s) + kEpsilonS);
        }

        if (std::abs(chunk.t1_s - request.preview_patch.anchor_time_s) <= kEpsilonS)
        {
            EXPECT_FALSE(has_preview_chunk_flag);
            EXPECT_FALSE(has_preview_anchor_flag);
        }
    }

    EXPECT_EQ(preview_chunk_count, exact_chunks.size());
    EXPECT_EQ(preview_anchor_count, 1u);
}

TEST(OrbitPredictionPlannerTests, ResolvesChunkAdaptiveOptionsPerChunkClass)
{
    using Profile = Game::OrbitPredictionService::PredictionProfileId;

    Game::OrbitPredictionService::Request request =
            make_request(0.0, 6.0 * Game::OrbitPredictionTuning::kSecondsPerYear);

    const Game::OrbitPredictionService::PredictionSolvePlan plan = Game::build_prediction_solve_plan(request);
    ASSERT_TRUE(plan.valid);

    const auto near_chunk = find_first_chunk_with_profile(plan, Profile::Near);
    const auto tail_chunk = find_first_chunk_with_profile(plan, Profile::Tail);
    ASSERT_TRUE(near_chunk.has_value());
    ASSERT_TRUE(tail_chunk.has_value());

    const auto near_def = Game::resolve_prediction_profile_definition(request, *near_chunk);
    const auto tail_def = Game::resolve_prediction_profile_definition(request, *tail_chunk);

    EXPECT_LT(near_def.max_dt_s, tail_def.max_dt_s);
    EXPECT_GT(near_def.soft_max_segments, tail_def.soft_max_segments);
    EXPECT_GT(near_def.output_sample_density_scale, tail_def.output_sample_density_scale);

    const auto near_segment_opt =
            Game::build_spacecraft_adaptive_segment_options_for_chunk(request, *near_chunk);
    const auto tail_segment_opt =
            Game::build_spacecraft_adaptive_segment_options_for_chunk(request, *tail_chunk);
    const auto near_ephemeris_opt =
            Game::build_adaptive_ephemeris_options_for_chunk(request, *near_chunk);
    const auto tail_ephemeris_opt =
            Game::build_adaptive_ephemeris_options_for_chunk(request, *tail_chunk);

    EXPECT_DOUBLE_EQ(near_segment_opt.duration_s, near_chunk->t1_s - near_chunk->t0_s);
    EXPECT_DOUBLE_EQ(tail_segment_opt.duration_s, tail_chunk->t1_s - tail_chunk->t0_s);
    EXPECT_LT(near_segment_opt.max_dt_s, tail_segment_opt.max_dt_s);
    EXPECT_GT(near_segment_opt.soft_max_segments, tail_segment_opt.soft_max_segments);
    EXPECT_LT(near_ephemeris_opt.max_dt_s, tail_ephemeris_opt.max_dt_s);
    EXPECT_GT(near_ephemeris_opt.soft_max_segments, tail_ephemeris_opt.soft_max_segments);

    const std::size_t near_sample_budget =
            Game::prediction_sample_budget_for_chunk(request, *near_chunk, 256u);
    const std::size_t tail_sample_budget =
            Game::prediction_sample_budget_for_chunk(request, *tail_chunk, 256u);
    EXPECT_GT(near_sample_budget, tail_sample_budget);
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
    request.preview_patch.visual_window_s = 0.5 * Game::OrbitPredictionTuning::kSecondsPerHour;
    request.preview_patch.exact_window_s = request.preview_patch.visual_window_s;

    Game::OrbitPredictionService::ManeuverImpulse maneuver{};
    maneuver.node_id = 7;
    maneuver.t_s = request.sim_time_s + 5.0 * Game::OrbitPredictionTuning::kSecondsPerDay;
    request.maneuver_impulses.push_back(maneuver);

    const Game::OrbitPredictionService::PredictionSolvePlan plan = Game::build_prediction_solve_plan(request);
    ASSERT_TRUE(plan.valid);

    const auto exact_chunk = find_first_chunk_with_profile(plan, Profile::Exact);
    ASSERT_TRUE(exact_chunk.has_value());

    const auto maneuver_chunk = find_first_chunk_with_flags(plan, static_cast<uint32_t>(Flags::Maneuver));
    ASSERT_TRUE(maneuver_chunk.has_value());

    const auto exact_def = Game::resolve_prediction_profile_definition(request, *exact_chunk);
    const auto maneuver_def = Game::resolve_prediction_profile_definition(request, *maneuver_chunk);

    EXPECT_LT(exact_def.integrator_tolerance_multiplier, 1.0);
    EXPECT_LT(exact_def.max_dt_s, maneuver_def.max_dt_s);
    EXPECT_GT(exact_def.output_sample_density_scale, maneuver_def.output_sample_density_scale);

    const auto exact_opt =
            Game::build_spacecraft_adaptive_segment_options_for_chunk(request, *exact_chunk);
    const auto maneuver_opt =
            Game::build_spacecraft_adaptive_segment_options_for_chunk(request, *maneuver_chunk);
    EXPECT_LT(exact_opt.max_dt_s, maneuver_opt.max_dt_s);
    EXPECT_LE(exact_opt.lookup_max_dt_s, maneuver_opt.lookup_max_dt_s);
    EXPECT_GT(exact_opt.soft_max_segments, maneuver_opt.soft_max_segments);
}

TEST(OrbitPredictionPlannerTests, ActivityClassifierKeepsCalmChunkStable)
{
    using Profile = Game::OrbitPredictionService::PredictionProfileId;

    Game::OrbitPredictionService::Request request =
            make_request(0.0, 20.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
    request.preferred_primary_body_id = 1;

    orbitsim::MassiveBody primary{};
    primary.id = 1;
    primary.mass_kg = 5.972e24;
    primary.state = orbitsim::make_state(orbitsim::Vec3(0.0), orbitsim::Vec3(0.0));
    request.massive_bodies.push_back(primary);

    const Game::OrbitPredictionService::PredictionChunkPlan chunk{
            .chunk_id = 0u,
            .t0_s = 0.0,
            .t1_s = 10.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
            .profile_id = Profile::Tail,
    };
    const std::vector<orbitsim::TrajectorySegment> baseline{
            make_activity_segment(0.0,
                                  5.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                                  orbitsim::Vec3(10'000'000.0, 0.0, 0.0),
                                  orbitsim::Vec3(0.0, 2'500.0, 0.0),
                                  orbitsim::Vec3(10'000'000.0, 1.08e9, 0.0),
                                  orbitsim::Vec3(0.0, 2'500.0, 0.0)),
            make_activity_segment(5.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                                  10.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                                  orbitsim::Vec3(10'000'000.0, 1.08e9, 0.0),
                                  orbitsim::Vec3(0.0, 2'500.0, 0.0),
                                  orbitsim::Vec3(10'000'000.0, 2.16e9, 0.0),
                                  orbitsim::Vec3(0.0, 2'500.0, 0.0)),
    };

    const auto probe = Game::classify_chunk_activity(request, chunk, &baseline);
    ASSERT_TRUE(probe.valid);
    EXPECT_FALSE(probe.should_split);
    EXPECT_EQ(probe.recommended_profile_id, Profile::Tail);
    EXPECT_GT(probe.dominant_gravity_ratio, 0.99);
}

TEST(OrbitPredictionPlannerTests, ActivityClassifierPromotesAndSplitsHighCurvatureChunk)
{
    using Profile = Game::OrbitPredictionService::PredictionProfileId;

    Game::OrbitPredictionService::Request request =
            make_request(0.0, 20.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
    request.preferred_primary_body_id = 1;

    orbitsim::MassiveBody primary_a{};
    primary_a.id = 1;
    primary_a.mass_kg = 5.0e24;
    primary_a.state = orbitsim::make_state(orbitsim::Vec3(-10'000'000.0, 0.0, 0.0), orbitsim::Vec3(0.0));
    request.massive_bodies.push_back(primary_a);

    orbitsim::MassiveBody primary_b{};
    primary_b.id = 2;
    primary_b.mass_kg = 5.0e24;
    primary_b.state = orbitsim::make_state(orbitsim::Vec3(10'000'000.0, 0.0, 0.0), orbitsim::Vec3(0.0));
    request.massive_bodies.push_back(primary_b);

    const Game::OrbitPredictionService::PredictionChunkPlan chunk{
            .chunk_id = 0u,
            .t0_s = 0.0,
            .t1_s = 10.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
            .profile_id = Profile::Tail,
    };
    const std::vector<orbitsim::TrajectorySegment> baseline{
            make_activity_segment(0.0,
                                  5.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                                  orbitsim::Vec3(0.0, 0.0, 0.0),
                                  orbitsim::Vec3(2'000.0, 0.0, 0.0),
                                  orbitsim::Vec3(2.0e8, 0.0, 0.0),
                                  orbitsim::Vec3(0.0, 2'000.0, 0.0)),
            make_activity_segment(5.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                                  10.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                                  orbitsim::Vec3(2.0e8, 0.0, 0.0),
                                  orbitsim::Vec3(0.0, 2'000.0, 0.0),
                                  orbitsim::Vec3(2.0e8, 2.0e8, 0.0),
                                  orbitsim::Vec3(-2'000.0, 0.0, 0.0)),
    };

    const auto probe = Game::classify_chunk_activity(request, chunk, &baseline);
    ASSERT_TRUE(probe.valid);
    EXPECT_TRUE(probe.should_split);
    EXPECT_LT(static_cast<int>(probe.recommended_profile_id), static_cast<int>(Profile::Tail));
    EXPECT_LT(probe.dominant_gravity_ratio, 0.8);
}

TEST(OrbitPredictionPlannerTests, ActivityClassifierSplitsNearAndExactAtDominantGravityTransition)
{
    using Profile = Game::OrbitPredictionService::PredictionProfileId;

    Game::OrbitPredictionService::Request request =
            make_request(0.0, 20.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
    request.preferred_primary_body_id = 1;

    orbitsim::MassiveBody primary_a{};
    primary_a.id = 1;
    primary_a.mass_kg = 5.0e24;
    primary_a.state = orbitsim::make_state(orbitsim::Vec3(-100'000'000.0, 0.0, 0.0), orbitsim::Vec3(0.0));
    request.massive_bodies.push_back(primary_a);

    orbitsim::MassiveBody primary_b{};
    primary_b.id = 2;
    primary_b.mass_kg = 5.0e24;
    primary_b.state = orbitsim::make_state(orbitsim::Vec3(100'000'000.0, 0.0, 0.0), orbitsim::Vec3(0.0));
    request.massive_bodies.push_back(primary_b);

    const std::vector<orbitsim::TrajectorySegment> baseline{
            make_activity_segment(0.0,
                                  5.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                                  orbitsim::Vec3(-90'000'000.0, 0.0, 0.0),
                                  orbitsim::Vec3(200.0, 0.0, 0.0),
                                  orbitsim::Vec3(0.0, 0.0, 0.0),
                                  orbitsim::Vec3(200.0, 0.0, 0.0)),
            make_activity_segment(5.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                                  10.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                                  orbitsim::Vec3(0.0, 0.0, 0.0),
                                  orbitsim::Vec3(200.0, 0.0, 0.0),
                                  orbitsim::Vec3(90'000'000.0, 0.0, 0.0),
                                  orbitsim::Vec3(200.0, 0.0, 0.0)),
    };

    const Profile profile_ids[] = {Profile::Exact, Profile::Near};
    for (const Profile profile_id : profile_ids)
    {
        SCOPED_TRACE(static_cast<int>(profile_id));

        const Game::OrbitPredictionService::PredictionChunkPlan chunk{
                .chunk_id = 0u,
                .t0_s = 0.0,
                .t1_s = 10.0 * Game::OrbitPredictionTuning::kSecondsPerDay,
                .profile_id = profile_id,
        };

        const auto probe = Game::classify_chunk_activity(request, chunk, &baseline);
        ASSERT_TRUE(probe.valid);
        EXPECT_TRUE(probe.should_split);
        EXPECT_EQ(probe.primary_body_id_start, 1);
        EXPECT_EQ(probe.primary_body_id_mid, 1);
        EXPECT_EQ(probe.primary_body_id_end, 2);
        EXPECT_LT(probe.dominant_gravity_ratio,
                  Game::OrbitPredictionTuning::kPredictionActivityProbeDominantGravitySplitRatio);
        EXPECT_EQ(probe.recommended_profile_id, Profile::Exact);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
