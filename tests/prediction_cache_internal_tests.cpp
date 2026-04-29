#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"
#include "game/states/gameplay/prediction/prediction_frame_cache_builder.h"
#include "game/states/gameplay/prediction/prediction_metrics_builder.h"
#include "game/states/gameplay/prediction/streamed_chunk_assembly_builder.h"

#include "orbitsim/trajectories.hpp"

#include <gtest/gtest.h>

namespace
{
    orbitsim::TrajectorySegment make_test_segment(const double t0_s,
                                                  const double dt_s,
                                                  const orbitsim::Vec3 &p0,
                                                  const orbitsim::Vec3 &v0,
                                                  const orbitsim::Vec3 &p1,
                                                  const orbitsim::Vec3 &v1,
                                                  const std::uint32_t flags = 0u)
    {
        return orbitsim::TrajectorySegment{
                .t0_s = t0_s,
                .dt_s = dt_s,
                .start = orbitsim::make_state(p0, v0),
                .end = orbitsim::make_state(p1, v1),
                .flags = flags,
        };
    }

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

        fixture.cache.identity.valid = true;
        fixture.cache.solver.shared_ephemeris = ephemeris;
        fixture.cache.solver.massive_bodies = fixture.sim.massive_bodies();
        fixture.cache.solver.trajectory_segments_inertial = root_segments;
        fixture.cache.solver.trajectory_inertial =
                orbitsim::sample_trajectory_segments_uniform_dt(root_segments, 1.0, 64, true, true);
        return fixture;
    }

    class PredictionCacheInternalFixture : public ::testing::Test
    {
    protected:
        LinearPredictionFixture fixture = make_linear_cache();

        void assert_ready() const
        {
            ASSERT_TRUE(fixture.cache.identity.valid);
            ASSERT_NE(fixture.body_id, orbitsim::kInvalidBodyId);
            ASSERT_GE(fixture.cache.solver.trajectory_inertial.size(), 2u);
            ASSERT_FALSE(fixture.cache.solver.trajectory_segments_inertial.empty());
        }

        orbitsim::TrajectoryFrameSpec body_centered_frame() const
        {
            return orbitsim::TrajectoryFrameSpec::body_centered_inertial(fixture.body_id);
        }

        bool rebuild_body_centered_frame_cache(
                Game::OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                const bool build_planned_render_curve = true)
        {
            return Game::PredictionFrameCacheBuilder::rebuild(
                    fixture.cache.solver,
                    fixture.cache.display,
                    fixture.cache.analysis,
                    body_centered_frame(),
                    fixture.cache.solver.trajectory_segments_inertial,
                    {},
                    diagnostics,
                    build_planned_render_curve);
        }

        bool rebuild_body_centered_planned_frame_cache(const bool build_planned_render_curve = true)
        {
            return Game::PredictionFrameCacheBuilder::rebuild_planned(
                    fixture.cache.solver,
                    fixture.cache.display,
                    body_centered_frame(),
                    fixture.cache.solver.trajectory_segments_inertial,
                    {},
                    nullptr,
                    build_planned_render_curve);
        }

        void seed_planned_from_base()
        {
            fixture.cache.solver.trajectory_inertial_planned = fixture.cache.solver.trajectory_inertial;
            fixture.cache.solver.trajectory_segments_inertial_planned = fixture.cache.solver.trajectory_segments_inertial;
        }
    };
} // namespace

TEST(PredictionTrajectorySamplingTests, BoundarySideSelectsBeforeOrAfterImpulseVelocity)
{
    std::vector<orbitsim::TrajectorySegment> segments{
            make_test_segment(0.0,
                              10.0,
                              orbitsim::Vec3{0.0, 0.0, 0.0},
                              orbitsim::Vec3{1.0, 0.0, 0.0},
                              orbitsim::Vec3{10.0, 0.0, 0.0},
                              orbitsim::Vec3{1.0, 0.0, 0.0}),
            make_test_segment(10.0,
                              10.0,
                              orbitsim::Vec3{10.0, 0.0, 0.0},
                              orbitsim::Vec3{3.0, 0.0, 0.0},
                              orbitsim::Vec3{40.0, 0.0, 0.0},
                              orbitsim::Vec3{3.0, 0.0, 0.0},
                              orbitsim::kTrajectorySegmentFlagImpulseBoundary),
    };
    ASSERT_TRUE(Game::validate_trajectory_segment_continuity(segments));

    orbitsim::State default_state{};
    ASSERT_TRUE(Game::sample_trajectory_segment_state(segments, 10.0, default_state));
    EXPECT_DOUBLE_EQ(default_state.position_m.x, 10.0);
    EXPECT_DOUBLE_EQ(default_state.velocity_mps.x, 1.0);

    orbitsim::State before_state{};
    ASSERT_TRUE(Game::sample_trajectory_segment_state(segments,
                                                      10.0,
                                                      before_state,
                                                      Game::TrajectoryBoundarySide::Before));
    EXPECT_DOUBLE_EQ(before_state.position_m.x, 10.0);
    EXPECT_DOUBLE_EQ(before_state.velocity_mps.x, 1.0);

    orbitsim::State after_state{};
    ASSERT_TRUE(Game::sample_trajectory_segment_state(segments,
                                                      10.0,
                                                      after_state,
                                                      Game::TrajectoryBoundarySide::After));
    EXPECT_DOUBLE_EQ(after_state.position_m.x, 10.0);
    EXPECT_DOUBLE_EQ(after_state.velocity_mps.x, 3.0);

    orbitsim::State position_only_state{};
    ASSERT_TRUE(Game::sample_trajectory_segment_state(segments,
                                                      10.0,
                                                      position_only_state,
                                                      Game::TrajectoryBoundarySide::ContinuousPositionOnly));
    EXPECT_DOUBLE_EQ(position_only_state.position_m.x, 10.0);
}

TEST(PredictionTrajectorySamplingTests, ContinuityRequiresImpulseFlagForVelocityDiscontinuity)
{
    std::vector<orbitsim::TrajectorySegment> segments{
            make_test_segment(0.0,
                              10.0,
                              orbitsim::Vec3{0.0, 0.0, 0.0},
                              orbitsim::Vec3{1.0, 0.0, 0.0},
                              orbitsim::Vec3{10.0, 0.0, 0.0},
                              orbitsim::Vec3{1.0, 0.0, 0.0}),
            make_test_segment(10.0,
                              10.0,
                              orbitsim::Vec3{10.0, 0.0, 0.0},
                              orbitsim::Vec3{3.0, 0.0, 0.0},
                              orbitsim::Vec3{40.0, 0.0, 0.0},
                              orbitsim::Vec3{3.0, 0.0, 0.0}),
    };
    EXPECT_FALSE(Game::validate_trajectory_segment_continuity(segments));

    segments[1].flags = orbitsim::kTrajectorySegmentFlagImpulseBoundary;
    EXPECT_TRUE(Game::validate_trajectory_segment_continuity(segments));
}

TEST_F(PredictionCacheInternalFixture, RebuildFrameCacheProducesBodyCenteredSegments)
{
    assert_ready();

    Game::OrbitPredictionDerivedDiagnostics diagnostics{};
    const bool ok = rebuild_body_centered_frame_cache(&diagnostics);

    ASSERT_TRUE(ok);
    ASSERT_TRUE(fixture.cache.display.resolved_frame_spec_valid);
    ASSERT_FALSE(fixture.cache.display.trajectory_segments_frame.empty());
    ASSERT_GE(fixture.cache.display.trajectory_frame.size(), 2u);
    EXPECT_EQ(diagnostics.status, Game::PredictionDerivedStatus::Success);
    EXPECT_EQ(diagnostics.frame_segment_count, fixture.cache.display.trajectory_segments_frame.size());
    EXPECT_EQ(diagnostics.frame_sample_count, fixture.cache.display.trajectory_frame.size());
    EXPECT_EQ(diagnostics.frame_base.accepted_segments, fixture.cache.display.trajectory_segments_frame.size());
    EXPECT_GT(diagnostics.frame_base.covered_duration_s, 0.0);

    EXPECT_NEAR(fixture.cache.display.trajectory_segments_frame.front().start.position_m.x, 10.0, 1.0e-6);
    EXPECT_NEAR(fixture.cache.display.trajectory_segments_frame.front().start.velocity_mps.x, 0.5, 1.0e-6);
    EXPECT_NEAR(fixture.cache.display.trajectory_frame.back().position_m.x, 15.0, 1.0e-4);
}

TEST_F(PredictionCacheInternalFixture, RebuildPredictionMetricsUsesSegmentDerivedSamples)
{
    assert_ready();

    ASSERT_TRUE(rebuild_body_centered_frame_cache());

    Game::PredictionMetricsBuilder::rebuild(fixture.cache.solver,
                                            fixture.cache.display,
                                            fixture.cache.analysis,
                                            fixture.sim.config(),
                                            fixture.body_id);

    EXPECT_TRUE(fixture.cache.analysis.metrics_valid);
    EXPECT_EQ(fixture.cache.analysis.metrics_body_id, fixture.body_id);
    ASSERT_EQ(fixture.cache.analysis.altitude_km.size(), fixture.cache.display.trajectory_frame.size());
    ASSERT_EQ(fixture.cache.analysis.speed_kmps.size(), fixture.cache.display.trajectory_frame.size());
    EXPECT_TRUE(fixture.cache.analysis.analysis_cache_valid);
    EXPECT_EQ(fixture.cache.analysis.analysis_cache_body_id, fixture.body_id);
    EXPECT_FALSE(fixture.cache.analysis.trajectory_segments_analysis_bci.empty());
    EXPECT_FALSE(fixture.cache.analysis.trajectory_analysis_bci.empty());
    EXPECT_NEAR(fixture.cache.analysis.altitude_km.front(), 0.009f, 1.0e-4f);
    EXPECT_NEAR(fixture.cache.analysis.speed_kmps.front(), 0.0005f, 1.0e-5f);
}

TEST(PredictionCacheInternalTests, RebuildPredictionMetricsFindsPeriapsisFromSegments)
{
    constexpr orbitsim::BodyId body_id = 1;

    Game::OrbitPredictionCache cache{};
    cache.identity.valid = true;
    cache.solver.massive_bodies = {
            orbitsim::MassiveBody{
                    .mass_kg = 1.0,
                    .radius_m = 1.0,
                    .id = body_id,
            },
    };
    cache.display.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::body_centered_inertial(body_id);
    cache.display.resolved_frame_spec_valid = true;
    cache.display.trajectory_segments_frame = {
            orbitsim::TrajectorySegment{
                    .t0_s = 0.0,
                    .dt_s = 10.0,
                    .start = orbitsim::make_state(orbitsim::Vec3{10.0, 0.0, 0.0},
                                                  orbitsim::Vec3{-2.0, 0.0, 0.0}),
                    .end = orbitsim::make_state(orbitsim::Vec3{10.0, 0.0, 0.0},
                                                orbitsim::Vec3{2.0, 0.0, 0.0}),
            },
    };
    cache.display.trajectory_frame = {
            orbitsim::TrajectorySample{
                    .t_s = 0.0,
                    .position_m = cache.display.trajectory_segments_frame.front().start.position_m,
                    .velocity_mps = cache.display.trajectory_segments_frame.front().start.velocity_mps,
            },
            orbitsim::TrajectorySample{
                    .t_s = 10.0,
                    .position_m = cache.display.trajectory_segments_frame.front().end.position_m,
                    .velocity_mps = cache.display.trajectory_segments_frame.front().end.velocity_mps,
            },
    };

    orbitsim::GameSimulation::Config cfg{};
    cfg.gravitational_constant = 1.0;

    Game::PredictionMetricsBuilder::rebuild(cache.solver,
                                            cache.display,
                                            cache.analysis,
                                            cfg,
                                            body_id);

    ASSERT_TRUE(cache.analysis.metrics_valid);
    ASSERT_EQ(cache.analysis.altitude_km.size(), 2u);
    EXPECT_NEAR(cache.analysis.altitude_km.front(), 0.009f, 1.0e-6f);
    EXPECT_NEAR(cache.analysis.periapsis_alt_km, 0.004, 1.0e-6);
}

TEST_F(PredictionCacheInternalFixture, RebuildFrameCacheCanSkipPreviewPlannedRenderCurve)
{
    assert_ready();
    seed_planned_from_base();

    ASSERT_TRUE(rebuild_body_centered_frame_cache(nullptr, false));

    EXPECT_FALSE(fixture.cache.display.render_curve_frame.empty());
    EXPECT_FALSE(fixture.cache.display.trajectory_segments_frame_planned.empty());
    EXPECT_TRUE(fixture.cache.display.render_curve_frame_planned.empty());
}

TEST_F(PredictionCacheInternalFixture, RebuildPlannedFrameCacheCanSkipPreviewPlannedRenderCurve)
{
    assert_ready();
    seed_planned_from_base();

    ASSERT_TRUE(rebuild_body_centered_planned_frame_cache(false));

    EXPECT_FALSE(fixture.cache.display.trajectory_segments_frame_planned.empty());
    EXPECT_GE(fixture.cache.display.trajectory_frame_planned.size(), 2u);
    EXPECT_TRUE(fixture.cache.display.render_curve_frame_planned.empty());
}

TEST_F(PredictionCacheInternalFixture, RebuildFrameCacheRejectsDiscontinuousSegments)
{
    assert_ready();

    fixture.cache.solver.trajectory_segments_inertial.push_back(orbitsim::TrajectorySegment{
            .t0_s = 20.0,
            .dt_s = 5.0,
            .start = orbitsim::make_state(orbitsim::Vec3{500.0, 0.0, 0.0}, orbitsim::Vec3{0.0, 1.0, 0.0}),
            .end = orbitsim::make_state(orbitsim::Vec3{505.0, 0.0, 0.0}, orbitsim::Vec3{0.0, 1.0, 0.0}),
            .flags = 0u,
    });

    Game::OrbitPredictionDerivedDiagnostics diagnostics{};
    const bool ok = Game::PredictionFrameCacheBuilder::rebuild(
            fixture.cache.solver,
            fixture.cache.display,
            fixture.cache.analysis,
            orbitsim::TrajectoryFrameSpec::inertial(),
            fixture.cache.solver.trajectory_segments_inertial,
            {},
            &diagnostics);

    EXPECT_FALSE(ok);
    EXPECT_EQ(diagnostics.status, Game::PredictionDerivedStatus::ContinuityFailed);
}

TEST(PredictionCacheInternalTests, RebuildPredictionPatchChunksClipsStraddlingSegmentsAtChunkBoundaries)
{
    Game::OrbitPredictionCache cache{};
    cache.identity.valid = true;
    cache.solver.trajectory_segments_inertial_planned = {
            orbitsim::TrajectorySegment{
                    .t0_s = 0.0,
                    .dt_s = 20.0,
                    .start = orbitsim::make_state(orbitsim::Vec3{0.0, 0.0, 0.0}, orbitsim::Vec3{1.0, 0.0, 0.0}),
                    .end = orbitsim::make_state(orbitsim::Vec3{20.0, 0.0, 0.0}, orbitsim::Vec3{1.0, 0.0, 0.0}),
                    .flags = 0u,
            },
    };
    cache.display.trajectory_segments_frame_planned = cache.solver.trajectory_segments_inertial_planned;

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
    ASSERT_TRUE(Game::StreamedChunkAssemblyBuilder::rebuild_from_published(
            assembly,
            cache.display,
            published_chunks,
            7u,
            orbitsim::TrajectoryFrameSpec::inertial(),
            0u,
            0u,
            {},
            {},
            &diagnostics));

    ASSERT_TRUE(assembly.valid);
    EXPECT_EQ(assembly.generation_id, 7u);
    ASSERT_EQ(assembly.chunks.size(), 2u);
    EXPECT_TRUE(assembly.chunks[0].valid);
    EXPECT_TRUE(assembly.chunks[1].valid);
    EXPECT_EQ(assembly.chunks[0].chunk_id, published_chunks[0].chunk_id);
    EXPECT_EQ(assembly.chunks[1].chunk_id, published_chunks[1].chunk_id);
    EXPECT_EQ(assembly.chunks[0].generation_id, 7u);
    EXPECT_EQ(assembly.chunks[1].generation_id, 7u);
    EXPECT_EQ(assembly.chunks[0].quality_state, Game::OrbitPredictionService::ChunkQualityState::PreviewPatch);
    EXPECT_EQ(assembly.chunks[1].quality_state, Game::OrbitPredictionService::ChunkQualityState::Final);
    EXPECT_DOUBLE_EQ(assembly.chunks[0].t0_s, published_chunks[0].t0_s);
    EXPECT_DOUBLE_EQ(assembly.chunks[0].t1_s, published_chunks[0].t1_s);
    EXPECT_DOUBLE_EQ(assembly.chunks[1].t0_s, published_chunks[1].t0_s);
    EXPECT_DOUBLE_EQ(assembly.chunks[1].t1_s, published_chunks[1].t1_s);
    ASSERT_FALSE(assembly.chunks[0].frame_samples.empty());
    ASSERT_FALSE(assembly.chunks[1].frame_samples.empty());
    EXPECT_DOUBLE_EQ(assembly.chunks[0].frame_segments.front().t0_s, 0.0);
    EXPECT_DOUBLE_EQ(assembly.chunks[0].frame_segments.front().dt_s, 10.0);
    EXPECT_DOUBLE_EQ(assembly.chunks[1].frame_segments.front().t0_s, 10.0);
    EXPECT_DOUBLE_EQ(assembly.chunks[1].frame_segments.front().dt_s, 10.0);
    EXPECT_DOUBLE_EQ(assembly.chunks[0].frame_samples.front().t_s, published_chunks[0].t0_s);
    EXPECT_DOUBLE_EQ(assembly.chunks[0].frame_samples.back().t_s, published_chunks[0].t1_s);
    EXPECT_DOUBLE_EQ(assembly.chunks[1].frame_samples.front().t_s, published_chunks[1].t0_s);
    EXPECT_DOUBLE_EQ(assembly.chunks[1].frame_samples.back().t_s, published_chunks[1].t1_s);

    Game::StreamedChunkAssemblyBuilder::flatten(cache.display, assembly);
    ASSERT_EQ(cache.display.trajectory_segments_frame_planned.size(), 2u);
    ASSERT_EQ(cache.display.trajectory_frame_planned.size(), 3u);
    EXPECT_DOUBLE_EQ(cache.display.trajectory_segments_frame_planned[0].t0_s, 0.0);
    EXPECT_DOUBLE_EQ(cache.display.trajectory_segments_frame_planned[0].dt_s, 10.0);
    EXPECT_DOUBLE_EQ(cache.display.trajectory_segments_frame_planned[1].t0_s, 10.0);
    EXPECT_DOUBLE_EQ(cache.display.trajectory_segments_frame_planned[1].dt_s, 10.0);
    EXPECT_DOUBLE_EQ(cache.display.trajectory_frame_planned.front().t_s, 0.0);
    EXPECT_DOUBLE_EQ(cache.display.trajectory_frame_planned.back().t_s, 20.0);
    EXPECT_EQ(diagnostics.status, Game::PredictionDerivedStatus::Success);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
