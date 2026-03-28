#include <sstream>

#define private public
#include "game/states/gameplay/gameplay_state.h"
#undef private
#include "game/orbit/orbit_prediction_tuning.h"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <mutex>
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

    Game::OrbitChunk make_chunk(const uint32_t chunk_id,
                                const uint64_t generation_id,
                                const double t0_s,
                                const double t1_s,
                                const double x0_m,
                                const double x1_m)
    {
        Game::OrbitChunk chunk{};
        chunk.chunk_id = chunk_id;
        chunk.generation_id = generation_id;
        chunk.quality_state = Game::OrbitPredictionService::ChunkQualityState::Final;
        chunk.t0_s = t0_s;
        chunk.t1_s = t1_s;
        chunk.frame_samples = {make_sample(t0_s, x0_m), make_sample(t1_s, x1_m)};
        chunk.frame_segments = {make_segment(t0_s, t1_s, x0_m, x1_m)};
        chunk.render_curve = Game::OrbitRenderCurve::build(chunk.frame_segments);
        chunk.valid = true;
        return chunk;
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

    std::vector<Game::OrbitPredictionService::Result> run_prediction_results(
            Game::OrbitPredictionService &service,
            const uint64_t generation_id,
            Game::OrbitPredictionService::Request request,
            const uint64_t request_epoch = 1)
    {
        service._completed.clear();

        Game::OrbitPredictionService::PendingJob job{};
        job.track_id = request.track_id;
        job.request_epoch = request_epoch;
        job.generation_id = generation_id;
        job.request = std::move(request);

        service.compute_prediction(job);

        std::vector<Game::OrbitPredictionService::Result> results;
        while (auto completed = service.poll_completed())
        {
            results.push_back(std::move(*completed));
        }
        return results;
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
    EXPECT_EQ(cleared.latest_requested_generation_id, 0u);
    EXPECT_FALSE(cleared.dirty);
    EXPECT_FALSE(cleared.invalidated_while_pending);
    EXPECT_EQ(cleared.preview_state, Game::PredictionPreviewRuntimeState::Idle);
    EXPECT_FALSE(cleared.preview_anchor.valid);
    EXPECT_TRUE(std::isnan(cleared.preview_entered_at_s));
    EXPECT_TRUE(std::isnan(cleared.preview_last_anchor_refresh_at_s));
    EXPECT_TRUE(std::isnan(cleared.preview_last_request_at_s));
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

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowKeepsDisplayedHorizonDuringLivePreview)
{
    Game::GameplayState state{};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 50'000.0;
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;

    Game::GameplayState::ManeuverNode selected{};
    selected.id = 7;
    selected.time_s = 240.0;
    state._maneuver_state.selected_node_id = selected.id;
    state._maneuver_state.nodes.push_back(selected);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = state.prediction_required_window_s(orbiter_key, 100.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 50'000.0);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheSeparatesPatchWindowFromDisplayWindow)
{
    Game::GameplayState state{};
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode far_node{};
    far_node.id = 7;
    far_node.time_s = 50'000.0;
    state._maneuver_state.selected_node_id = far_node.id;
    state._maneuver_state.nodes.push_back(far_node);

    Game::PredictionTrackState track{};
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.generation_id = 42;

    state.refresh_prediction_preview_anchor(track, 100.0, true);

    ASSERT_TRUE(track.preview_anchor.valid);
    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::EnterDrag);
    EXPECT_EQ(track.preview_anchor.anchor_node_id, far_node.id);
    EXPECT_DOUBLE_EQ(track.preview_anchor.anchor_time_s, 50'000.0);
    EXPECT_DOUBLE_EQ(state.prediction_display_window_s(track.key, 100.0, true), 50'200.0);
    EXPECT_DOUBLE_EQ(state.prediction_preview_exact_window_s(track, 100.0, true), 300.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.request_window_s, 50'200.0);
    EXPECT_DOUBLE_EQ(track.preview_last_anchor_refresh_at_s, 100.0);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheExtendsVisualWindowBeyondExactWindowDuringDrag)
{
    Game::GameplayState state{};
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 2.0 * Game::OrbitPredictionTuning::kSecondsPerDay;
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode node{};
    node.id = 9;
    node.time_s = 240.0;
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;

    state.refresh_prediction_preview_anchor(track, 100.0, true);

    ASSERT_TRUE(track.preview_anchor.valid);
    EXPECT_DOUBLE_EQ(track.preview_anchor.visual_window_s,
                     (2.0 * Game::OrbitPredictionTuning::kSecondsPerDay) - 140.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.exact_window_s,
                     Game::OrbitPredictionTuning::kDragInteractivePreviewWindowMaxS);
    EXPECT_GT(track.preview_anchor.visual_window_s, track.preview_anchor.exact_window_s);
    EXPECT_DOUBLE_EQ(track.preview_anchor.request_window_s,
                     2.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheTransitionsToAwaitFullRefineAfterDragEnds)
{
    Game::GameplayState state{};
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode node{};
    node.id = 3;
    node.time_s = 240.0;
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;

    state.refresh_prediction_preview_anchor(track, 100.0, true);
    ASSERT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::EnterDrag);

    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    state._maneuver_plan_live_preview_active = false;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::Idle;
    state.refresh_prediction_preview_anchor(track, 101.0, true);

    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::AwaitFullRefine);
    EXPECT_TRUE(track.preview_anchor.valid);
    EXPECT_EQ(track.preview_anchor.anchor_node_id, node.id);
}

TEST(GameplayPredictionManeuverTests, RequestOrbiterPredictionTracksPreviewRequestTimestamp)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode node{};
    node.id = 9;
    node.time_s = 240.0;
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;
    state.refresh_prediction_preview_anchor(track, 100.0, true);

    const bool requested = state.request_orbiter_prediction_async(track,
                                                                  WorldVec3(7'000'000.0, 0.0, 0.0),
                                                                  glm::dvec3(0.0, 7'500.0, 0.0),
                                                                  100.0,
                                                                  false,
                                                                  true);

    ASSERT_TRUE(requested);
    EXPECT_TRUE(track.request_pending);
    EXPECT_EQ(track.pending_solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::DragPreviewPending);
    EXPECT_DOUBLE_EQ(track.preview_last_request_at_s, 100.0);
    EXPECT_GT(track.latest_requested_generation_id, 0u);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestKeepsUpstreamManeuversBeforeAnchor)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode first{};
    first.id = 1;
    first.time_s = 150.0;
    state._maneuver_state.nodes.push_back(first);

    Game::GameplayState::ManeuverNode second{};
    second.id = 2;
    second.time_s = 240.0;
    state._maneuver_state.selected_node_id = second.id;
    state._maneuver_state.nodes.push_back(second);

    Game::PredictionTrackState track{};
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;

    const bool requested = state.request_orbiter_prediction_async(track,
                                                                  WorldVec3(7'000'000.0, 0.0, 0.0),
                                                                  glm::dvec3(0.0, 7'500.0, 0.0),
                                                                  100.0,
                                                                  false,
                                                                  true);

    ASSERT_TRUE(requested);
    ASSERT_EQ(state._prediction_service._pending_jobs.size(), 1u);
    const auto &queued = state._prediction_service._pending_jobs.front().request;
    ASSERT_EQ(queued.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_EQ(queued.maneuver_impulses.size(), 2u);
    EXPECT_EQ(queued.maneuver_impulses[0].node_id, first.id);
    EXPECT_EQ(queued.maneuver_impulses[1].node_id, second.id);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestKeepsDownstreamManeuversWithinDisplayHorizon)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode anchor{};
    anchor.id = 1;
    anchor.time_s = 240.0;
    state._maneuver_state.selected_node_id = anchor.id;
    state._maneuver_state.nodes.push_back(anchor);

    Game::GameplayState::ManeuverNode downstream{};
    downstream.id = 2;
    downstream.time_s = 700.0; // Outside the local patch window, still inside the full request horizon.
    state._maneuver_state.nodes.push_back(downstream);

    Game::PredictionTrackState track{};
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;

    const bool requested = state.request_orbiter_prediction_async(track,
                                                                  WorldVec3(7'000'000.0, 0.0, 0.0),
                                                                  glm::dvec3(0.0, 7'500.0, 0.0),
                                                                  100.0,
                                                                  false,
                                                                  true);

    ASSERT_TRUE(requested);
    ASSERT_EQ(state._prediction_service._pending_jobs.size(), 1u);
    const auto &queued = state._prediction_service._pending_jobs.front().request;
    ASSERT_EQ(queued.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_EQ(queued.maneuver_impulses.size(), 2u);
    EXPECT_EQ(queued.maneuver_impulses[0].node_id, anchor.id);
    EXPECT_EQ(queued.maneuver_impulses[1].node_id, downstream.id);
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
    EXPECT_GE(first_finalizing_result.trajectory_segments_inertial_planned.front().t0_s, 40.0 - 1.0e-6);
    EXPECT_GE(last_finalizing_result.trajectory_segments_inertial_planned.back().t0_s +
                      last_finalizing_result.trajectory_segments_inertial_planned.back().dt_s,
              60.0);
    const orbitsim::State &streaming_end_state = last_streaming_result.trajectory_segments_inertial_planned.back().end;
    const orbitsim::State &finalizing_start_state =
            first_finalizing_result.trajectory_segments_inertial_planned.front().start;
    EXPECT_LT(glm::length(glm::dvec3(streaming_end_state.position_m - finalizing_start_state.position_m)), 1.0e-3);
    EXPECT_LT(glm::length(glm::dvec3(streaming_end_state.velocity_mps - finalizing_start_state.velocity_mps)),
              1.0e-6);
    bool saw_finalizing_impulse_preview = false;
    for (const Game::OrbitPredictionService::Result *result : finalizing_results)
    {
        for (const Game::OrbitPredictionService::ManeuverNodePreview &preview : result->maneuver_previews)
        {
            saw_finalizing_impulse_preview |= preview.node_id == 2;
        }
    }
    EXPECT_TRUE(saw_finalizing_impulse_preview);
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

TEST(GameplayPredictionManeuverTests, StaleDerivedResultDoesNotReplaceNewerDisplayedGeneration)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.valid = true;
    track.cache.generation_id = 3;
    track.cache.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    track.cache.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    track.cache.trajectory_frame = track.cache.trajectory_inertial;
    track.cache.trajectory_segments_frame = track.cache.trajectory_segments_inertial;
    track.latest_requested_generation_id = 4;
    track.derived_request_pending = true;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 2;
    result.valid = true;
    result.cache.valid = true;
    result.cache.generation_id = 2;
    result.cache.trajectory_inertial = {make_sample(0.0, 8'000'000.0), make_sample(60.0, 8'100'000.0)};
    result.cache.trajectory_segments_inertial = {make_segment(0.0, 60.0, 8'000'000.0, 8'100'000.0)};
    result.cache.trajectory_frame = result.cache.trajectory_inertial;
    result.cache.trajectory_segments_frame = result.cache.trajectory_segments_inertial;

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    EXPECT_EQ(state._prediction_tracks.front().cache.generation_id, 3u);
    EXPECT_DOUBLE_EQ(state._prediction_tracks.front().cache.trajectory_frame.front().position_m.x, 7'000'000.0);
    EXPECT_TRUE(state._prediction_tracks.front().derived_request_pending);
}

TEST(GameplayPredictionManeuverTests, ReusedBaseFrameDerivedResultPreservesBaseDiagnostics)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.valid = true;
    track.cache.resolved_frame_spec_valid = true;
    track.cache.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::body_centered_inertial(1);
    track.cache.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    track.cache.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    track.cache.trajectory_frame = track.cache.trajectory_inertial;
    track.cache.trajectory_segments_frame = track.cache.trajectory_segments_inertial;
    track.derived_diagnostics.frame_base.accepted_segments = 11;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result result{};
    result.track_id = state._prediction_tracks.front().key.track_id();
    result.generation_id = 1;
    result.valid = true;
    result.base_frame_reused = true;
    result.diagnostics.frame_base.accepted_segments = 11;
    result.cache.valid = true;
    result.cache.resolved_frame_spec_valid = true;
    result.cache.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::body_centered_inertial(1);
    result.cache.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    result.cache.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    result.cache.trajectory_inertial_planned = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'200'000.0)};
    result.cache.trajectory_segments_inertial_planned = {make_segment(0.0, 60.0, 7'000'000.0, 7'200'000.0)};

    state.apply_completed_prediction_derived_result(std::move(result));

    ASSERT_EQ(state._prediction_tracks.size(), 1u);
    EXPECT_EQ(state._prediction_tracks.front().derived_diagnostics.frame_base.accepted_segments, 11u);
    EXPECT_EQ(state._prediction_tracks.front().derived_diagnostics.status, Game::PredictionDerivedStatus::Success);
}

TEST(GameplayPredictionManeuverTests, PreviewDerivedResultsAccumulatePlannedChunkAssemblyAcrossStages)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.valid = true;
    track.cache.generation_id = 4;
    track.cache.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    track.cache.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    track.cache.trajectory_frame = track.cache.trajectory_inertial;
    track.cache.trajectory_segments_frame = track.cache.trajectory_segments_inertial;
    track.cache.trajectory_inertial_planned = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10.0, 7'100'000.0),
            make_sample(20.0, 7'200'000.0),
            make_sample(30.0, 7'300'000.0),
    };
    track.cache.trajectory_segments_inertial_planned = {
            make_segment(0.0, 10.0, 7'000'000.0, 7'100'000.0),
            make_segment(10.0, 20.0, 7'100'000.0, 7'200'000.0),
            make_segment(20.0, 30.0, 7'200'000.0, 7'300'000.0),
    };
    track.cache.trajectory_frame_planned = track.cache.trajectory_inertial_planned;
    track.cache.trajectory_segments_frame_planned = track.cache.trajectory_segments_inertial_planned;
    state._prediction_tracks.push_back(track);

    Game::OrbitPredictionDerivedService::Result streaming_result{};
    streaming_result.track_id = state._prediction_tracks.front().key.track_id();
    streaming_result.generation_id = 5;
    streaming_result.valid = true;
    streaming_result.solve_quality = Game::OrbitPredictionService::SolveQuality::FastPreview;
    streaming_result.cache.valid = true;
    streaming_result.cache.generation_id = 5;
    streaming_result.cache.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    streaming_result.cache.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    streaming_result.cache.trajectory_frame = streaming_result.cache.trajectory_inertial;
    streaming_result.cache.trajectory_segments_frame = streaming_result.cache.trajectory_segments_inertial;
    streaming_result.cache.trajectory_inertial_planned = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10.0, 7'100'000.0),
            make_sample(20.0, 7'200'000.0),
    };
    streaming_result.cache.trajectory_segments_inertial_planned = {
            make_segment(0.0, 10.0, 7'000'000.0, 7'100'000.0),
            make_segment(10.0, 20.0, 7'100'000.0, 7'200'000.0),
    };
    streaming_result.cache.trajectory_frame_planned = streaming_result.cache.trajectory_inertial_planned;
    streaming_result.cache.trajectory_segments_frame_planned = streaming_result.cache.trajectory_segments_inertial_planned;
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
    finalizing_result.cache.valid = true;
    finalizing_result.cache.generation_id = 5;
    finalizing_result.cache.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(30.0, 7'300'000.0)};
    finalizing_result.cache.trajectory_segments_inertial = {make_segment(0.0, 30.0, 7'000'000.0, 7'300'000.0)};
    finalizing_result.cache.trajectory_frame = finalizing_result.cache.trajectory_inertial;
    finalizing_result.cache.trajectory_segments_frame = finalizing_result.cache.trajectory_segments_inertial;
    finalizing_result.cache.trajectory_inertial_planned = {
            make_sample(20.0, 7'200'000.0),
            make_sample(30.0, 7'300'000.0),
    };
    finalizing_result.cache.trajectory_segments_inertial_planned = {
            make_segment(20.0, 30.0, 7'200'000.0, 7'300'000.0),
    };
    finalizing_result.cache.trajectory_frame_planned = finalizing_result.cache.trajectory_inertial_planned;
    finalizing_result.cache.trajectory_segments_frame_planned = finalizing_result.cache.trajectory_segments_inertial_planned;
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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
