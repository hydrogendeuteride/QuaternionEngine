#include <sstream>

#define private public
#include "game/states/gameplay/gameplay_state.h"
#undef private

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <unordered_map>

namespace
{
    std::unique_ptr<Game::OrbitalScenario> make_reference_orbitsim(const double time_s,
                                                                   const double reference_mass_kg = 5.972e24)
    {
        auto scenario = std::make_unique<Game::OrbitalScenario>();

        orbitsim::GameSimulation::Config cfg{};
        cfg.enable_events = false;
        cfg.spacecraft_integrator.adaptive = true;
        cfg.spacecraft_integrator.max_substeps = 256;
        scenario->sim = orbitsim::GameSimulation(cfg);
        (void) scenario->sim.set_time_s(time_s);

        orbitsim::MassiveBody ref{};
        ref.id = 1;
        ref.mass_kg = reference_mass_kg;
        ref.radius_m = 6'371'000.0;
        ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));

        const auto handle = scenario->sim.create_body_with_id(ref.id, ref);
        if (!handle.valid())
        {
            return nullptr;
        }

        Game::CelestialBodyInfo ref_info{};
        ref_info.sim_id = handle.id;
        ref_info.name = "earth";
        ref_info.radius_m = ref.radius_m;
        ref_info.mass_kg = ref.mass_kg;

        scenario->bodies.push_back(ref_info);
        scenario->world_reference_body_index = 0;
        return scenario;
    }

    orbitsim::TrajectorySample make_sample(const double t_s, const double x_m)
    {
        orbitsim::TrajectorySample sample{};
        sample.t_s = t_s;
        sample.position_m = orbitsim::Vec3{x_m, 0.0, 0.0};
        sample.velocity_mps = orbitsim::Vec3{0.0, 7'500.0, 0.0};
        return sample;
    }

    orbitsim::TrajectorySegment make_segment(const double t0_s, const double t1_s, const double x0_m, const double x1_m)
    {
        orbitsim::TrajectorySegment segment{};
        segment.t0_s = t0_s;
        segment.dt_s = t1_s - t0_s;
        segment.start = orbitsim::make_state(orbitsim::Vec3{x0_m, 0.0, 0.0}, orbitsim::Vec3{0.0, 7'500.0, 0.0});
        segment.end = orbitsim::make_state(orbitsim::Vec3{x1_m, 0.0, 0.0}, orbitsim::Vec3{0.0, 7'500.0, 0.0});
        return segment;
    }

    Game::OrbitPredictionService::Request make_prediction_request(const double time_s,
                                                                  const double future_window_s = 120.0)
    {
        Game::OrbitPredictionService::Request request{};
        request.track_id = 7;
        request.sim_time_s = time_s;
        request.sim_config.enable_events = false;
        request.sim_config.spacecraft_integrator.adaptive = true;
        request.sim_config.spacecraft_integrator.max_substeps = 256;
        request.ship_bary_position_m = orbitsim::Vec3{7'000'000.0, 0.0, 0.0};
        request.ship_bary_velocity_mps = orbitsim::Vec3{0.0, 7'500.0, 0.0};
        request.future_window_s = future_window_s;
        request.preferred_primary_body_id = 1;

        orbitsim::MassiveBody ref{};
        ref.id = 1;
        ref.mass_kg = 5.972e24;
        ref.radius_m = 6'371'000.0;
        ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
        request.massive_bodies.push_back(ref);
        return request;
    }
} // namespace

TEST(GameplayPredictionManeuverTests, ClearPredictionRuntimeResetsTrackState)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.valid = true;
    track.cache.build_time_s = 42.0;
    track.cache.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    track.cache.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    track.request_pending = true;
    track.derived_request_pending = true;
    track.dirty = true;
    track.invalidated_while_pending = true;
    track.solver_ms_last = 12.0;
    track.solver_diagnostics.status = Game::OrbitPredictionService::Status::Success;
    track.derived_diagnostics.status = Game::PredictionDerivedStatus::Success;
    state._prediction_tracks.push_back(track);
    state._prediction_dirty = true;

    state.clear_prediction_runtime();

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    const Game::PredictionTrackState &cleared = state._prediction_tracks.front();
    EXPECT_FALSE(cleared.cache.valid);
    EXPECT_TRUE(cleared.cache.trajectory_inertial.empty());
    EXPECT_TRUE(cleared.cache.trajectory_segments_inertial.empty());
    EXPECT_FALSE(cleared.request_pending);
    EXPECT_FALSE(cleared.derived_request_pending);
    EXPECT_FALSE(cleared.dirty);
    EXPECT_FALSE(cleared.invalidated_while_pending);
    EXPECT_DOUBLE_EQ(cleared.solver_ms_last, 0.0);
    EXPECT_EQ(cleared.solver_diagnostics.status, Game::OrbitPredictionService::Status::None);
    EXPECT_EQ(cleared.derived_diagnostics.status, Game::PredictionDerivedStatus::None);
    EXPECT_FALSE(state._prediction_dirty);
}

TEST(GameplayPredictionManeuverTests, PredictionFutureWindowClampsNegativeValues)
{
    Game::GameplayState state{};
    state._prediction_sampling_policy.orbiter_min_window_s = -5.0;
    state._prediction_sampling_policy.celestial_min_window_s = -10.0;

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const Game::PredictionSubjectKey celestial_key{Game::PredictionSubjectKind::Celestial, 2};

    EXPECT_DOUBLE_EQ(state.prediction_future_window_s(orbiter_key), 0.0);
    EXPECT_DOUBLE_EQ(state.prediction_future_window_s(celestial_key), 0.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowExtendsPastLastManeuverNode)
{
    Game::GameplayState state{};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;

    Game::GameplayState::ManeuverNode first{};
    first.id = 1;
    first.time_s = 150.0;
    state._maneuver_state.nodes.push_back(first);

    Game::GameplayState::ManeuverNode second{};
    second.id = 2;
    second.time_s = 240.0;
    state._maneuver_state.nodes.push_back(second);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = state.prediction_required_window_s(orbiter_key, 100.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 440.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowSupportsFarFutureManeuverNodes)
{
    Game::GameplayState state{};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;

    Game::GameplayState::ManeuverNode far_node{};
    far_node.id = 1;
    far_node.time_s = 50'000.0;
    state._maneuver_state.nodes.push_back(far_node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = state.prediction_required_window_s(orbiter_key, 100.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 50'200.0);
}

TEST(GameplayPredictionManeuverTests, ShouldRebuildPredictionTrackWhenCoverageFallsShort)
{
    Game::GameplayState state{};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 120.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.valid = true;
    track.cache.build_time_s = 0.0;
    track.cache.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'050'000.0)};
    track.cache.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'050'000.0)};

    EXPECT_TRUE(state.should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));
}

TEST(GameplayPredictionManeuverTests, ShouldRebuildPredictionTrackWhenManeuverCoverageFallsShortOutsideLivePreview)
{
    Game::GameplayState state{};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = false;

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 400.0;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.valid = true;
    track.cache.build_time_s = 0.0;
    track.cache.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'050'000.0)};
    track.cache.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'050'000.0)};

    EXPECT_TRUE(state.should_rebuild_prediction_track(track, 10.0, 0.016f, false, true));
}

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

    const Game::OrbitPredictionService::Result result = service.compute_prediction(1, request, 1);

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

    const Game::OrbitPredictionService::Result result = service.compute_prediction(1, request, 1);

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
    const Game::OrbitPredictionService::Result baseline_result = service.compute_prediction(1, baseline_request, 1);

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

    const Game::OrbitPredictionService::Result preview_result = service.compute_prediction(2, preview_request, 1);

    ASSERT_TRUE(preview_result.valid);
    EXPECT_TRUE(preview_result.diagnostics.trajectory_base.cache_reused);
    EXPECT_EQ(preview_result.diagnostics.trajectory_sample_count, baseline_result.diagnostics.trajectory_sample_count);
    ASSERT_FALSE(preview_result.trajectory_segments_inertial_planned.empty());
    EXPECT_GE(preview_result.trajectory_segments_inertial.back().t0_s +
                      preview_result.trajectory_segments_inertial.back().dt_s,
              preview_request.sim_time_s + preview_request.future_window_s);
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
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
