#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

#include "orbitsim/trajectories.hpp"

#include <gtest/gtest.h>

namespace
{
    struct LinearPredictionFixture
    {
        orbitsim::GameSimulation sim{};
        orbitsim::BodyId body_id{orbitsim::kInvalidBodyId};
        Game::OrbitPredictionCache cache{};
    };

    LinearPredictionFixture make_linear_cache()
    {
        orbitsim::GameSimulation::Config cfg{};
        cfg.gravitational_constant = 1.0e-6;
        cfg.enable_events = false;
        cfg.spacecraft_integrator.adaptive = true;
        cfg.spacecraft_integrator.max_substeps = 256;

        LinearPredictionFixture fixture{.sim = orbitsim::GameSimulation(cfg)};
        (void) fixture.sim.set_time_s(0.0);

        const auto body_h = fixture.sim.create_body(orbitsim::MassiveBody{
                .mass_kg = 1.0,
                .radius_m = 1.0,
                .state = orbitsim::make_state({100.0, 0.0, 0.0}, {0.5, 0.0, 0.0}),
        });
        if (!body_h.valid())
        {
            return fixture;
        }
        fixture.body_id = body_h.id;

        const auto sc_h = fixture.sim.create_spacecraft(orbitsim::Spacecraft{
                .state = orbitsim::make_state({110.0, 0.0, 0.0}, {1.0, 0.0, 0.0}),
                .dry_mass_kg = 1.0,
                .prop_mass_kg = 0.0,
        });
        if (!sc_h.valid())
        {
            return fixture;
        }

        orbitsim::AdaptiveEphemerisOptions eph_opt{};
        eph_opt.duration_s = 10.0;
        eph_opt.min_dt_s = 0.25;
        eph_opt.max_dt_s = 10.0;
        eph_opt.soft_max_segments = 8;
        eph_opt.hard_max_segments = 32;
        const auto ephemeris = std::make_shared<orbitsim::CelestialEphemeris>(
                orbitsim::build_celestial_ephemeris_adaptive(fixture.sim, eph_opt));
        if (ephemeris->empty())
        {
            return fixture;
        }

        orbitsim::AdaptiveSegmentOptions seg_opt{};
        seg_opt.duration_s = 10.0;
        seg_opt.min_dt_s = 0.25;
        seg_opt.max_dt_s = 10.0;
        seg_opt.lookup_max_dt_s = 1.0;
        seg_opt.soft_max_segments = 8;
        seg_opt.hard_max_segments = 32;
        const std::vector<orbitsim::TrajectorySegment> root_segments =
                orbitsim::predict_spacecraft_trajectory_segments_adaptive(fixture.sim, *ephemeris, sc_h.id, seg_opt);
        if (root_segments.empty())
        {
            return fixture;
        }

        fixture.cache.valid = true;
        fixture.cache.shared_ephemeris = ephemeris;
        fixture.cache.massive_bodies = fixture.sim.massive_bodies();
        fixture.cache.trajectory_segments_inertial = root_segments;
        fixture.cache.trajectory_inertial =
                orbitsim::sample_trajectory_segments_uniform_dt(root_segments, 1.0, 64, true, true);
        return fixture;
    }
} // namespace

TEST(PredictionCacheInternalTests, RebuildFrameCacheProducesBodyCenteredSegments)
{
    LinearPredictionFixture fixture = make_linear_cache();
    ASSERT_TRUE(fixture.cache.valid);
    ASSERT_NE(fixture.body_id, orbitsim::kInvalidBodyId);
    ASSERT_GE(fixture.cache.trajectory_inertial.size(), 2u);

    Game::OrbitPredictionDerivedDiagnostics diagnostics{};
    const bool ok = Game::PredictionCacheInternal::rebuild_prediction_frame_cache(
            fixture.cache,
            orbitsim::TrajectoryFrameSpec::body_centered_inertial(fixture.body_id),
            fixture.cache.trajectory_segments_inertial,
            {},
            &diagnostics);

    ASSERT_TRUE(ok);
    ASSERT_TRUE(fixture.cache.resolved_frame_spec_valid);
    ASSERT_FALSE(fixture.cache.trajectory_segments_frame.empty());
    ASSERT_GE(fixture.cache.trajectory_frame.size(), 2u);
    EXPECT_EQ(diagnostics.status, Game::PredictionDerivedStatus::Success);
    EXPECT_EQ(diagnostics.frame_segment_count, fixture.cache.trajectory_segments_frame.size());
    EXPECT_EQ(diagnostics.frame_sample_count, fixture.cache.trajectory_frame.size());
    EXPECT_EQ(diagnostics.frame_base.accepted_segments, fixture.cache.trajectory_segments_frame.size());
    EXPECT_GT(diagnostics.frame_base.covered_duration_s, 0.0);

    EXPECT_NEAR(fixture.cache.trajectory_segments_frame.front().start.position_m.x, 10.0, 1.0e-6);
    EXPECT_NEAR(fixture.cache.trajectory_segments_frame.front().start.velocity_mps.x, 0.5, 1.0e-6);
    EXPECT_NEAR(fixture.cache.trajectory_frame.back().position_m.x, 15.0, 1.0e-4);
}

TEST(PredictionCacheInternalTests, RebuildPredictionMetricsUsesSegmentDerivedSamples)
{
    LinearPredictionFixture fixture = make_linear_cache();
    ASSERT_TRUE(fixture.cache.valid);
    ASSERT_NE(fixture.body_id, orbitsim::kInvalidBodyId);
    ASSERT_GE(fixture.cache.trajectory_inertial.size(), 2u);

    ASSERT_TRUE(Game::PredictionCacheInternal::rebuild_prediction_frame_cache(
            fixture.cache,
            orbitsim::TrajectoryFrameSpec::body_centered_inertial(fixture.body_id),
            fixture.cache.trajectory_segments_inertial));

    Game::PredictionCacheInternal::rebuild_prediction_metrics(fixture.cache, fixture.sim.config(), fixture.body_id);

    EXPECT_TRUE(fixture.cache.metrics_valid);
    EXPECT_EQ(fixture.cache.metrics_body_id, fixture.body_id);
    ASSERT_EQ(fixture.cache.altitude_km.size(), fixture.cache.trajectory_frame.size());
    ASSERT_EQ(fixture.cache.speed_kmps.size(), fixture.cache.trajectory_frame.size());
    EXPECT_NEAR(fixture.cache.altitude_km.front(), 0.009f, 1.0e-4f);
    EXPECT_NEAR(fixture.cache.speed_kmps.front(), 0.0005f, 1.0e-5f);
}

TEST(PredictionCacheInternalTests, RebuildPredictionMetricsFindsPeriapsisFromSegments)
{
    constexpr orbitsim::BodyId body_id = 1;

    Game::OrbitPredictionCache cache{};
    cache.valid = true;
    cache.massive_bodies = {
            orbitsim::MassiveBody{
                    .mass_kg = 1.0,
                    .radius_m = 1.0,
                    .id = body_id,
            },
    };
    cache.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::body_centered_inertial(body_id);
    cache.resolved_frame_spec_valid = true;
    cache.trajectory_segments_frame = {
            orbitsim::TrajectorySegment{
                    .t0_s = 0.0,
                    .dt_s = 10.0,
                    .start = orbitsim::make_state(orbitsim::Vec3{10.0, 0.0, 0.0},
                                                  orbitsim::Vec3{-2.0, 0.0, 0.0}),
                    .end = orbitsim::make_state(orbitsim::Vec3{10.0, 0.0, 0.0},
                                                orbitsim::Vec3{2.0, 0.0, 0.0}),
            },
    };
    cache.trajectory_frame = {
            orbitsim::TrajectorySample{
                    .t_s = 0.0,
                    .position_m = cache.trajectory_segments_frame.front().start.position_m,
                    .velocity_mps = cache.trajectory_segments_frame.front().start.velocity_mps,
            },
            orbitsim::TrajectorySample{
                    .t_s = 10.0,
                    .position_m = cache.trajectory_segments_frame.front().end.position_m,
                    .velocity_mps = cache.trajectory_segments_frame.front().end.velocity_mps,
            },
    };

    orbitsim::GameSimulation::Config cfg{};
    cfg.gravitational_constant = 1.0;

    Game::PredictionCacheInternal::rebuild_prediction_metrics(cache, cfg, body_id);

    ASSERT_TRUE(cache.metrics_valid);
    ASSERT_EQ(cache.altitude_km.size(), 2u);
    EXPECT_NEAR(cache.altitude_km.front(), 0.009f, 1.0e-6f);
    EXPECT_NEAR(cache.periapsis_alt_km, 0.004, 1.0e-6);
}

TEST(PredictionCacheInternalTests, RebuildFrameCacheCanSkipPreviewPlannedRenderCurve)
{
    LinearPredictionFixture fixture = make_linear_cache();
    ASSERT_TRUE(fixture.cache.valid);
    ASSERT_NE(fixture.body_id, orbitsim::kInvalidBodyId);
    ASSERT_FALSE(fixture.cache.trajectory_segments_inertial.empty());

    fixture.cache.trajectory_inertial_planned = fixture.cache.trajectory_inertial;
    fixture.cache.trajectory_segments_inertial_planned = fixture.cache.trajectory_segments_inertial;

    ASSERT_TRUE(Game::PredictionCacheInternal::rebuild_prediction_frame_cache(
            fixture.cache,
            orbitsim::TrajectoryFrameSpec::body_centered_inertial(fixture.body_id),
            fixture.cache.trajectory_segments_inertial,
            {},
            nullptr,
            false));

    EXPECT_FALSE(fixture.cache.render_curve_frame.empty());
    EXPECT_FALSE(fixture.cache.trajectory_segments_frame_planned.empty());
    EXPECT_TRUE(fixture.cache.render_curve_frame_planned.empty());
}

TEST(PredictionCacheInternalTests, RebuildPlannedFrameCacheCanSkipPreviewPlannedRenderCurve)
{
    LinearPredictionFixture fixture = make_linear_cache();
    ASSERT_TRUE(fixture.cache.valid);
    ASSERT_NE(fixture.body_id, orbitsim::kInvalidBodyId);
    ASSERT_FALSE(fixture.cache.trajectory_segments_inertial.empty());

    fixture.cache.trajectory_inertial_planned = fixture.cache.trajectory_inertial;
    fixture.cache.trajectory_segments_inertial_planned = fixture.cache.trajectory_segments_inertial;

    ASSERT_TRUE(Game::PredictionCacheInternal::rebuild_prediction_planned_frame_cache(
            fixture.cache,
            orbitsim::TrajectoryFrameSpec::body_centered_inertial(fixture.body_id),
            fixture.cache.trajectory_segments_inertial,
            {},
            nullptr,
            false));

    EXPECT_FALSE(fixture.cache.trajectory_segments_frame_planned.empty());
    EXPECT_GE(fixture.cache.trajectory_frame_planned.size(), 2u);
    EXPECT_TRUE(fixture.cache.render_curve_frame_planned.empty());
}

TEST(PredictionCacheInternalTests, RebuildPredictionMetricsCachesAnalysisTransformResults)
{
    LinearPredictionFixture fixture = make_linear_cache();
    ASSERT_TRUE(fixture.cache.valid);
    ASSERT_NE(fixture.body_id, orbitsim::kInvalidBodyId);

    Game::PredictionCacheInternal::rebuild_prediction_metrics(fixture.cache, fixture.sim.config(), fixture.body_id);

    EXPECT_TRUE(fixture.cache.metrics_valid);
    EXPECT_TRUE(fixture.cache.analysis_cache_valid);
    EXPECT_EQ(fixture.cache.analysis_cache_body_id, fixture.body_id);
    EXPECT_FALSE(fixture.cache.trajectory_segments_analysis_bci.empty());
    EXPECT_FALSE(fixture.cache.trajectory_analysis_bci.empty());
}

TEST(PredictionCacheInternalTests, RebuildFrameCacheRejectsDiscontinuousSegments)
{
    LinearPredictionFixture fixture = make_linear_cache();
    ASSERT_TRUE(fixture.cache.valid);
    ASSERT_FALSE(fixture.cache.trajectory_segments_inertial.empty());

    fixture.cache.trajectory_segments_inertial.push_back(orbitsim::TrajectorySegment{
            .t0_s = 20.0,
            .dt_s = 5.0,
            .start = orbitsim::make_state(orbitsim::Vec3{500.0, 0.0, 0.0}, orbitsim::Vec3{0.0, 1.0, 0.0}),
            .end = orbitsim::make_state(orbitsim::Vec3{505.0, 0.0, 0.0}, orbitsim::Vec3{0.0, 1.0, 0.0}),
            .flags = 0u,
    });

    Game::OrbitPredictionDerivedDiagnostics diagnostics{};
    const bool ok = Game::PredictionCacheInternal::rebuild_prediction_frame_cache(
            fixture.cache,
            orbitsim::TrajectoryFrameSpec::inertial(),
            fixture.cache.trajectory_segments_inertial,
            {},
            &diagnostics);

    EXPECT_FALSE(ok);
    EXPECT_EQ(diagnostics.status, Game::PredictionDerivedStatus::ContinuityFailed);
}

TEST(PredictionCacheInternalTests, RebuildPredictionPatchChunksClipsStraddlingSegmentsAtChunkBoundaries)
{
    Game::OrbitPredictionCache cache{};
    cache.valid = true;
    cache.trajectory_segments_inertial_planned = {
            orbitsim::TrajectorySegment{
                    .t0_s = 0.0,
                    .dt_s = 20.0,
                    .start = orbitsim::make_state(orbitsim::Vec3{0.0, 0.0, 0.0}, orbitsim::Vec3{1.0, 0.0, 0.0}),
                    .end = orbitsim::make_state(orbitsim::Vec3{20.0, 0.0, 0.0}, orbitsim::Vec3{1.0, 0.0, 0.0}),
                    .flags = 0u,
            },
    };
    cache.trajectory_segments_frame_planned = cache.trajectory_segments_inertial_planned;

    const std::vector<Game::OrbitPredictionService::PublishedChunk> published_chunks = {
            Game::OrbitPredictionService::PublishedChunk{
                    .chunk_id = 0u,
                    .quality_state = Game::OrbitPredictionService::ChunkQualityState::PreviewPatch,
                    .t0_s = 0.0,
                    .t1_s = 10.0,
                    .includes_planned_path = true,
            },
            Game::OrbitPredictionService::PublishedChunk{
                    .chunk_id = 1u,
                    .quality_state = Game::OrbitPredictionService::ChunkQualityState::Final,
                    .t0_s = 10.0,
                    .t1_s = 20.0,
                    .includes_planned_path = true,
            },
    };

    Game::PredictionChunkAssembly assembly{};
    Game::OrbitPredictionDerivedDiagnostics diagnostics{};
    ASSERT_TRUE(Game::PredictionCacheInternal::rebuild_prediction_patch_chunks(
            assembly,
            cache,
            published_chunks,
            7u,
            orbitsim::TrajectoryFrameSpec::inertial(),
            0u,
            0u,
            {},
            {},
            &diagnostics));

    ASSERT_TRUE(assembly.valid);
    ASSERT_EQ(assembly.chunks.size(), 2u);
    EXPECT_DOUBLE_EQ(assembly.chunks[0].frame_segments.front().t0_s, 0.0);
    EXPECT_DOUBLE_EQ(assembly.chunks[0].frame_segments.front().dt_s, 10.0);
    EXPECT_DOUBLE_EQ(assembly.chunks[1].frame_segments.front().t0_s, 10.0);
    EXPECT_DOUBLE_EQ(assembly.chunks[1].frame_segments.front().dt_s, 10.0);

    Game::PredictionCacheInternal::flatten_chunk_assembly_to_cache(cache, assembly);
    ASSERT_EQ(cache.trajectory_segments_frame_planned.size(), 2u);
    EXPECT_DOUBLE_EQ(cache.trajectory_segments_frame_planned[0].t0_s, 0.0);
    EXPECT_DOUBLE_EQ(cache.trajectory_segments_frame_planned[0].dt_s, 10.0);
    EXPECT_DOUBLE_EQ(cache.trajectory_segments_frame_planned[1].t0_s, 10.0);
    EXPECT_DOUBLE_EQ(cache.trajectory_segments_frame_planned[1].dt_s, 10.0);
    EXPECT_EQ(diagnostics.status, Game::PredictionDerivedStatus::Success);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
