#include "gameplay_prediction_maneuver_test_common.h"
#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"

namespace
{
    void register_player_draw_subject(Game::GameplayState &state, Game::Entity &entity)
    {
        entity.set_position_world(WorldVec3(7'000'000.0, 0.0, 0.0));
        GameplayTestHooks::register_entity(&entity);

        Game::OrbiterInfo player{};
        player.entity = entity.id();
        player.is_player = true;
        state._orbiters.push_back(player);
        state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, entity.id().value};
    }

    void mark_cache_current_display_frame(Game::GameplayState &state,
                                          Game::OrbitPredictionCache &cache)
    {
        cache.display.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::inertial();
        cache.display.resolved_frame_spec_valid = true;
        cache.display.display_frame_key = Game::prediction_display_frame_key(cache.display.resolved_frame_spec);
        cache.display.display_frame_revision = state._prediction.display_frame_revision;
    }

    Game::OrbitPredictionCache make_draw_ready_cache(Game::GameplayState &state,
                                                     const uint64_t generation_id,
                                                     const double t0_s,
                                                     const double t1_s)
    {
        Game::OrbitPredictionCache cache =
                make_prediction_cache(generation_id, t0_s, t1_s, 7'000'000.0, 7'400'000.0);
        mark_cache_current_display_frame(state, cache);
        return cache;
    }

    void set_planned_path(Game::OrbitPredictionCache &cache,
                          const double t0_s,
                          const double t_mid_s,
                          const double t1_s)
    {
        cache.solver.trajectory_inertial_planned = {
                make_sample(t0_s, 7'000'000.0),
                make_sample(t_mid_s, 7'200'000.0),
                make_sample(t1_s, 7'400'000.0),
        };
        cache.solver.trajectory_segments_inertial_planned = {
                make_segment(t0_s, t_mid_s, 7'000'000.0, 7'200'000.0),
                make_segment(t_mid_s, t1_s, 7'200'000.0, 7'400'000.0),
        };
        cache.display.trajectory_frame_planned = cache.solver.trajectory_inertial_planned;
        cache.display.trajectory_segments_frame_planned = cache.solver.trajectory_segments_inertial_planned;
    }

    Game::PredictionDrawDetail::PredictionGlobalDrawContext make_draw_global_context(const double display_time_s)
    {
        Game::PredictionDrawDetail::PredictionGlobalDrawContext global_ctx{};
        global_ctx.display_time_s = display_time_s;
        global_ctx.alpha_f = 1.0f;
        global_ctx.viewport_height_px = 720.0f;
        global_ctx.tan_half_fov = 1.0;
        global_ctx.color_orbit_plan = glm::vec4(1.0f);
        return global_ctx;
    }
} // namespace

TEST(GameplayPredictionManeuverTests, ClearPredictionRuntimeResetsTrackState)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.identity.valid = true;
    track.cache.identity.build_time_s = 42.0;
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    track.request_pending = true;
    track.derived_request_pending = true;
    track.dirty = true;
    track.invalidated_while_pending = true;
    track.solver_ms_last = 12.0;
    track.solver_diagnostics.status = Game::OrbitPredictionService::Status::Success;
    track.derived_diagnostics.status = Game::PredictionDerivedStatus::Success;
    state._prediction.tracks.push_back(track);
    state._prediction.dirty = true;

    state.clear_prediction_runtime();

    ASSERT_EQ(state._prediction.tracks.size(), 1u);
    const Game::PredictionTrackState &cleared = state._prediction.tracks.front();
    EXPECT_FALSE(cleared.cache.identity.valid);
    EXPECT_TRUE(cleared.cache.solver.trajectory_inertial.empty());
    EXPECT_TRUE(cleared.cache.solver.trajectory_segments_inertial.empty());
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
    EXPECT_FALSE(state._prediction.dirty);
}

TEST(GameplayPredictionManeuverTests, PredictionFutureWindowClampsNegativeValues)
{
    Game::GameplayState state{};
    state._prediction.sampling_policy.orbiter_min_window_s = -5.0;
    state._prediction.sampling_policy.celestial_min_window_s = -10.0;

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const Game::PredictionSubjectKey celestial_key{Game::PredictionSubjectKind::Celestial, 2};

    EXPECT_DOUBLE_EQ(state.prediction_future_window_s(orbiter_key), 0.0);
    EXPECT_DOUBLE_EQ(state.prediction_future_window_s(celestial_key), 0.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowAnchorsPlanHorizonAtFirstFutureNode)
{
    Game::GameplayState state{};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 120.0;
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
    // The active solve anchors at the first future node, then adds the plan horizon and solve margin.
    EXPECT_DOUBLE_EQ(required_window_s, 650.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowDoesNotAddLargeSolveMarginToNodeAnchoredPlanHorizon)
{
    Game::GameplayState state{};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_horizon.horizon_s = 600.0;
    state._maneuver_plan_windows.solve_margin_s = 600.0;

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 150.0;
    state._maneuver_state.nodes.push_back(node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = state.prediction_required_window_s(orbiter_key, 100.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 650.0);
}

TEST(GameplayPredictionManeuverTests, FullRequestAnchorsPlanHorizonAfterNearbyNode)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_horizon.horizon_s = 600.0;
    state._maneuver_plan_windows.solve_margin_s = 600.0;

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 150.0;
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_DOUBLE_EQ(request.future_window_s, 650.0);
    ASSERT_EQ(request.maneuver_impulses.size(), 1u);
    EXPECT_EQ(request.maneuver_impulses.front().node_id, node.id);
    EXPECT_DOUBLE_EQ(request.maneuver_impulses.front().t_s, node.time_s);
}

TEST(GameplayPredictionManeuverTests, FullRequestKeepsFarFutureNodeWhenPlanHorizonIsShorterThanNodeTime)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_horizon.horizon_s = 600.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 50'000.0;
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_DOUBLE_EQ(request.future_window_s, 50'500.0);
    ASSERT_EQ(request.maneuver_impulses.size(), 1u);
    EXPECT_EQ(request.maneuver_impulses.front().node_id, node.id);
    EXPECT_DOUBLE_EQ(request.maneuver_impulses.front().t_s, node.time_s);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowExtendsPastFarFutureManeuverNode)
{
    Game::GameplayState state{};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;

    Game::GameplayState::ManeuverNode far_node{};
    far_node.id = 1;
    far_node.time_s = 50'000.0;
    state._maneuver_state.nodes.push_back(far_node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = state.prediction_required_window_s(orbiter_key, 100.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 50'500.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowUsesNodeAnchoredPlanHorizonBeyondBaseSamplingWindow)
{
    Game::GameplayState state{};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 60'000.0;
    state._maneuver_plan_horizon.horizon_s = 120'000.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 1'000.0;
    state._maneuver_state.nodes.push_back(node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = state.prediction_required_window_s(orbiter_key, 0.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 121'000.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowKeepsDisplayedHorizonDuringLivePreview)
{
    Game::GameplayState state{};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 50'000.0;
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
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 120.0;
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
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.identity.generation_id = 42;

    state.refresh_prediction_preview_anchor(track, 100.0, true);

    ASSERT_TRUE(track.preview_anchor.valid);
    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::EnterDrag);
    EXPECT_EQ(track.preview_anchor.anchor_node_id, far_node.id);
    EXPECT_DOUBLE_EQ(track.preview_anchor.anchor_time_s, 50'000.0);
    EXPECT_DOUBLE_EQ(state.prediction_display_window_s(track.key, 100.0, true), 180.0);
    EXPECT_DOUBLE_EQ(state.prediction_preview_exact_window_s(track, 100.0, true), 180.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.request_window_s, 50'500.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.visual_window_s, 180.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.exact_window_s, 300.0);
    EXPECT_DOUBLE_EQ(track.preview_last_anchor_refresh_at_s, 100.0);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheClampsVisualWindowToPreviewWindowDuringDrag)
{
    Game::GameplayState state{};
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 2.0 * Game::OrbitPredictionTuning::kSecondsPerDay;
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
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;

    state.refresh_prediction_preview_anchor(track, 100.0, true);

    ASSERT_TRUE(track.preview_anchor.valid);
    EXPECT_DOUBLE_EQ(track.preview_anchor.visual_window_s, 180.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.exact_window_s, 300.0);
    EXPECT_LT(track.preview_anchor.visual_window_s, track.preview_anchor.exact_window_s);
    EXPECT_DOUBLE_EQ(track.preview_anchor.request_window_s,
                     2.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheCanReenterDragFromStableIdleTrack)
{
    Game::GameplayState state{};
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode node{};
    node.id = 11;
    node.time_s = 240.0;
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.cache = make_prediction_cache(4u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = false;
    track.preview_state = Game::PredictionPreviewRuntimeState::Idle;

    state.refresh_prediction_preview_anchor(track, 100.0, true);

    EXPECT_TRUE(track.preview_anchor.valid);
    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::EnterDrag);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheTransitionsToAwaitFullRefineAfterDragEnds)
{
    Game::GameplayState state{};
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
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
    track.key = state._prediction.selection.active_subject;
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
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
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
    track.key = state._prediction.selection.active_subject;
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
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
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
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request queued{};
    const bool requested = build_orbiter_prediction_request(state, track,
                                                                  WorldVec3(7'000'000.0, 0.0, 0.0),
                                                                  glm::dvec3(0.0, 7'500.0, 0.0),
                                                                  100.0,
                                                                  false,
                                                                  true,
                                                                  queued);

    ASSERT_TRUE(requested);
    ASSERT_EQ(queued.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_EQ(queued.maneuver_impulses.size(), 2u);
    EXPECT_EQ(queued.maneuver_impulses[0].node_id, first.id);
    EXPECT_EQ(queued.maneuver_impulses[1].node_id, second.id);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestKeepsDownstreamManeuversBeyondPreviewDisplayWindow)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 120.0;
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
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request queued{};
    const bool requested = build_orbiter_prediction_request(state, track,
                                                                  WorldVec3(7'000'000.0, 0.0, 0.0),
                                                                  glm::dvec3(0.0, 7'500.0, 0.0),
                                                                  100.0,
                                                                  false,
                                                                  true,
                                                                  queued);

    ASSERT_TRUE(requested);
    ASSERT_EQ(queued.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_EQ(queued.maneuver_impulses.size(), 2u);
    EXPECT_EQ(queued.maneuver_impulses[0].node_id, anchor.id);
    EXPECT_EQ(queued.maneuver_impulses[1].node_id, downstream.id);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestCapsHorizonToExactPatchWindow)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s =
            2.0 * Game::OrbitPredictionTuning::kSecondsPerDay;
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode selected{};
    selected.id = 4;
    selected.time_s = 240.0;
    state._maneuver_state.selected_node_id = selected.id;
    state._maneuver_state.nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.preview_patch.active);
    EXPECT_DOUBLE_EQ(request.future_window_s, 500.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_time_s, selected.time_s);
    EXPECT_DOUBLE_EQ(request.preview_patch.visual_window_s, 180.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.exact_window_s, 180.0);
    EXPECT_GT(request.future_window_s, request.preview_patch.visual_window_s);
    EXPECT_LT(request.future_window_s, 2.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestUsesSelectedNodePreviewForAnchorState)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode upstream{};
    upstream.id = 1;
    upstream.time_s = 150.0;
    state._maneuver_state.nodes.push_back(upstream);

    Game::GameplayState::ManeuverNode selected{};
    selected.id = 2;
    selected.time_s = 240.0;
    state._maneuver_state.selected_node_id = selected.id;
    state._maneuver_state.nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.solver.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(240.0, 7'100'000.0),
    };
    track.cache.solver.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = selected.id,
            .t_s = selected.time_s,
            .valid = true,
            .inertial_position_m = glm::dvec3(7'250'000.0, 123.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'650.0, 5.0),
    });

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.preview_patch.active);
    EXPECT_FALSE(request.planned_suffix_refine.active);
    ASSERT_TRUE(request.preview_patch.anchor_state_valid);
    EXPECT_TRUE(request.preview_patch.anchor_state_trusted);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.position_m.x, 7'250'000.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.position_m.y, 123.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.velocity_mps.y, 7'650.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.velocity_mps.z, 5.0);
}

TEST(GameplayPredictionManeuverTests, FullRequestEnablesPlannedSuffixRefineForPostDragAnchor)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::GameplayState::ManeuverNode upstream{};
    upstream.id = 1;
    upstream.time_s = 150.0;
    state._maneuver_state.nodes.push_back(upstream);

    Game::GameplayState::ManeuverNode selected{};
    selected.id = 2;
    selected.time_s = 240.0;
    state._maneuver_state.selected_node_id = selected.id;
    state._maneuver_state.nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.cache = make_prediction_cache(4u, 100.0, 360.0, 7'000'000.0, 7'260'000.0);
    track.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(100.0, 150.0, 7'000'000.0, 7'050'000.0),
            make_segment(150.0, 240.0, 7'050'000.0, 7'140'000.0),
    };
    track.cache.solver.trajectory_inertial_planned = {
            make_sample(100.0, 7'000'000.0),
            make_sample(150.0, 7'050'000.0),
            make_sample(240.0, 7'140'000.0),
    };
    track.cache.solver.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = upstream.id,
            .t_s = upstream.time_s,
            .valid = true,
            .inertial_position_m = glm::dvec3(7'050'000.0, 0.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
    });
    track.cache.solver.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = selected.id,
            .t_s = selected.time_s,
            .valid = true,
            .inertial_position_m = glm::dvec3(7'140'000.0, 0.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
    });
    track.authoritative_cache = track.cache;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = selected.id;
    track.preview_anchor.anchor_time_s = selected.time_s;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    ASSERT_TRUE(request.planned_suffix_refine.active);
    EXPECT_EQ(request.planned_suffix_refine.anchor_node_id, selected.id);
    EXPECT_DOUBLE_EQ(request.planned_suffix_refine.anchor_time_s, selected.time_s);
    ASSERT_EQ(request.planned_suffix_refine.prefix_segments_inertial.size(), 2u);
    EXPECT_DOUBLE_EQ(request.planned_suffix_refine.prefix_segments_inertial.front().t0_s, 100.0);
    EXPECT_DOUBLE_EQ(request.planned_suffix_refine.prefix_segments_inertial.back().t0_s +
                             request.planned_suffix_refine.prefix_segments_inertial.back().dt_s,
                     selected.time_s);
    ASSERT_EQ(request.planned_suffix_refine.prefix_previews.size(), 1u);
    EXPECT_EQ(request.planned_suffix_refine.prefix_previews.front().node_id, upstream.id);
    EXPECT_FALSE(request.preview_patch.active);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestFallsBackToInertialCacheForAnchorState)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;

    Game::GameplayState::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 240.0;
    state._maneuver_state.selected_node_id = selected.id;
    state._maneuver_state.nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.solver.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(240.0, 7'100'000.0),
    };

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.preview_patch.active);
    ASSERT_TRUE(request.preview_patch.anchor_state_valid);
    EXPECT_TRUE(request.preview_patch.anchor_state_trusted);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.position_m.x, 7'100'000.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.velocity_mps.y, 7'500.0);
}

TEST(GameplayPredictionManeuverTests, TimeEditActivatesFastPreviewRequest)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;

    Game::GameplayState::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 360.0;
    selected.dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);
    state._maneuver_state.selected_node_id = selected.id;
    state._maneuver_state.nodes.push_back(selected);
    state._maneuver_node_edit_preview.state = Game::GameplayState::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver_node_edit_preview.node_id = selected.id;
    state._maneuver_node_edit_preview.changed = true;
    state._maneuver_node_edit_preview.start_time_s = 240.0;

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.solver.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 7'200'000.0),
    };

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.preview_patch.active);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_time_s, selected.time_s);
    ASSERT_EQ(request.maneuver_impulses.size(), 1u);
    EXPECT_DOUBLE_EQ(request.maneuver_impulses.front().t_s, selected.time_s);
}

TEST(GameplayPredictionManeuverTests, TimeEditUsesBaselineAnchorStateForMovedFirstNode)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;

    Game::GameplayState::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 300.0;
    selected.dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);
    state._maneuver_state.selected_node_id = selected.id;
    state._maneuver_state.nodes.push_back(selected);
    state._maneuver_node_edit_preview.state = Game::GameplayState::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver_node_edit_preview.node_id = selected.id;
    state._maneuver_node_edit_preview.changed = true;
    state._maneuver_node_edit_preview.start_time_s = 240.0;

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.identity.valid = true;
    track.cache.solver.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 7'200'000.0),
    };
    track.cache.solver.trajectory_inertial_planned = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 9'000'000.0),
    };
    track.cache.solver.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = selected.id,
            .t_s = selected.time_s,
            .valid = true,
            .inertial_position_m = glm::dvec3(9'000'000.0, 0.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
    });

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.preview_patch.active);
    ASSERT_TRUE(request.preview_patch.anchor_state_valid);
    EXPECT_TRUE(request.preview_patch.anchor_state_trusted);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.position_m.x, 7'200'000.0);
}

TEST(GameplayPredictionManeuverTests, TimeEditLeavesAnchorStateUnseededWhenPriorManeuverExists)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver_plan_windows.preview_window_s = 180.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = true;

    Game::GameplayState::ManeuverNode upstream{};
    upstream.id = 4;
    upstream.time_s = 180.0;
    upstream.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);

    Game::GameplayState::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 300.0;
    selected.dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);
    state._maneuver_state.selected_node_id = selected.id;
    state._maneuver_state.nodes.push_back(upstream);
    state._maneuver_state.nodes.push_back(selected);
    state._maneuver_node_edit_preview.state = Game::GameplayState::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver_node_edit_preview.node_id = selected.id;
    state._maneuver_node_edit_preview.changed = true;
    state._maneuver_node_edit_preview.start_time_s = 240.0;

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.solver.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 7'200'000.0),
    };

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.preview_patch.active);
    EXPECT_FALSE(request.preview_patch.anchor_state_valid);
    ASSERT_EQ(request.maneuver_impulses.size(), 2u);
}

TEST(GameplayPredictionManeuverTests, TimeEditFinishRequestsFullRefine)
{
    Game::GameplayState state{};
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction.dirty = false;

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    state._prediction.tracks.push_back(track);

    state._maneuver_node_edit_preview.state = Game::GameplayState::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver_node_edit_preview.node_id = 5;
    state._maneuver_node_edit_preview.changed = true;
    state._maneuver_node_edit_preview.start_time_s = 240.0;

    state.finish_maneuver_node_time_edit_preview(false);

    ASSERT_EQ(state._prediction.tracks.size(), 1u);
    EXPECT_EQ(state._prediction.tracks.front().preview_state, Game::PredictionPreviewRuntimeState::AwaitFullRefine);
    EXPECT_EQ(state._maneuver_node_edit_preview.state, Game::GameplayState::ManeuverNodeEditPreview::State::Idle);
    EXPECT_TRUE(state._prediction.dirty);
}

TEST(GameplayPredictionManeuverTests, FinishedTimeEditDoesNotSeedAnchorFromStalePlannedPath)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::GameplayState::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 300.0;
    selected.dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);
    state._maneuver_state.selected_node_id = selected.id;
    state._maneuver_state.nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = selected.id;
    track.preview_anchor.anchor_time_s = selected.time_s;
    track.cache.identity.valid = true;
    track.cache.solver.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 7'200'000.0),
    };
    track.cache.solver.trajectory_segments_inertial = {
            make_segment(100.0, selected.time_s, 7'000'000.0, 7'200'000.0),
    };
    track.cache.solver.trajectory_inertial_planned = {
            make_sample(100.0, 7'000'000.0),
            make_sample(240.0, 9'000'000.0),
            make_sample(selected.time_s, 9'500'000.0),
    };
    track.cache.solver.trajectory_segments_inertial_planned = {
            make_segment(100.0, 240.0, 7'000'000.0, 9'000'000.0),
            make_segment(240.0, selected.time_s, 9'000'000.0, 9'500'000.0),
    };
    track.cache.solver.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = selected.id,
            .t_s = 240.0,
            .valid = true,
            .inertial_position_m = glm::dvec3(9'000'000.0, 0.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
    });
    track.authoritative_cache = track.cache;

    orbitsim::State anchor_state{};
    ASSERT_TRUE(resolve_prediction_preview_anchor_state(state, track, anchor_state));
    EXPECT_DOUBLE_EQ(anchor_state.position_m.x, 7'200'000.0);

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_FALSE(request.planned_suffix_refine.active);
}

TEST(GameplayPredictionManeuverTests, DrawContextUsesAuthoritativePlannedPrefixDuringDeltaVEdit)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);
    const uint64_t old_plan_signature = state.current_maneuver_plan_signature();
    state._maneuver_state.nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    state._maneuver_node_edit_preview.state = Game::ManeuverNodeEditPreview::State::EditingDv;
    state._maneuver_node_edit_preview.node_id = node.id;

    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    set_planned_path(track.cache, node.time_s, 320.0, 500.0);
    track.cache.identity.maneuver_plan_signature_valid = true;
    track.cache.identity.maneuver_plan_signature = old_plan_signature;

    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(state.build_orbit_prediction_track_draw_context(
            track,
            make_draw_global_context(100.0),
            draw_ctx));

    EXPECT_FALSE(draw_ctx.planned_cache_current);
    EXPECT_FALSE(draw_ctx.planned_cache_drawable);
    EXPECT_EQ(draw_ctx.stale_planned_cache, &track.authoritative_cache);
    EXPECT_TRUE(draw_ctx.stale_planned_cache_drawable);
    EXPECT_DOUBLE_EQ(draw_ctx.stale_planned_cache_prefix_cutoff_s, node.time_s);
    EXPECT_EQ(draw_ctx.planned_window_segments,
              &track.authoritative_cache.display.trajectory_segments_frame_planned);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawContextLimitsTimeEditStalePrefixToEarlierNodeTime)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::GameplayState::ManeuverNode node{};
    node.id = 9;
    node.time_s = 240.0;
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);
    const uint64_t old_plan_signature = state.current_maneuver_plan_signature();
    state._maneuver_state.nodes.front().time_s = 300.0;

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = 300.0;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    state._maneuver_node_edit_preview.state = Game::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver_node_edit_preview.node_id = node.id;
    state._maneuver_node_edit_preview.changed = true;
    state._maneuver_node_edit_preview.start_time_s = 240.0;

    track.cache = make_draw_ready_cache(state, 6u, 100.0, 500.0);
    set_planned_path(track.cache, 300.0, 380.0, 500.0);
    track.cache.identity.maneuver_plan_signature_valid = true;
    track.cache.identity.maneuver_plan_signature = old_plan_signature;

    track.authoritative_cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, 240.0, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(state.build_orbit_prediction_track_draw_context(
            track,
            make_draw_global_context(100.0),
            draw_ctx));

    EXPECT_EQ(draw_ctx.stale_planned_cache, &track.authoritative_cache);
    EXPECT_TRUE(draw_ctx.stale_planned_cache_drawable);
    EXPECT_DOUBLE_EQ(draw_ctx.stale_planned_cache_prefix_cutoff_s, 240.0);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawContextKeepsAuthoritativePlannedPrefixAfterEditRelease)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);
    const uint64_t old_plan_signature = state.current_maneuver_plan_signature();
    state._maneuver_state.nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.dirty = true;

    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);

    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(state.build_orbit_prediction_track_draw_context(
            track,
            make_draw_global_context(100.0),
            draw_ctx));

    EXPECT_FALSE(draw_ctx.planned_cache_current);
    EXPECT_FALSE(draw_ctx.planned_cache_drawable);
    EXPECT_EQ(draw_ctx.stale_planned_cache, &track.authoritative_cache);
    EXPECT_TRUE(draw_ctx.stale_planned_cache_drawable);
    EXPECT_DOUBLE_EQ(draw_ctx.stale_planned_cache_prefix_cutoff_s, node.time_s);
    EXPECT_EQ(draw_ctx.planned_window_segments,
              &track.authoritative_cache.display.trajectory_segments_frame_planned);
    EXPECT_TRUE(draw_ctx.planned_window_policy.valid);
    EXPECT_DOUBLE_EQ(draw_ctx.planned_window_policy.visual_window_start_time_s, 100.0);
    EXPECT_DOUBLE_EQ(draw_ctx.planned_window_policy.pick_window_start_time_s, 100.0);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawTrackUsesFullStreamOverlayForActiveManeuverRefine)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);
    const uint64_t old_plan_signature = state.current_maneuver_plan_signature();
    state._maneuver_state.nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.dirty = true;

    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    track.full_stream_overlay.chunk_assembly.valid = true;
    track.full_stream_overlay.chunk_assembly.generation_id = track.cache.identity.generation_id;
    track.full_stream_overlay.display_frame_key = track.cache.display.display_frame_key;
    track.full_stream_overlay.display_frame_revision = track.cache.display.display_frame_revision;
    track.full_stream_overlay.chunk_assembly.chunks = {
            make_chunk(0u, track.cache.identity.generation_id, node.time_s, node.time_s + 60.0, 7'200'000.0, 7'280'000.0),
    };

    OrbitPlotSystem plot{};
    Game::PredictionDrawDetail::PredictionGlobalDrawContext global_ctx = make_draw_global_context(100.0);
    global_ctx.orbit_plot = &plot;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(state.build_orbit_prediction_track_draw_context(track, global_ctx, draw_ctx));
    state.draw_orbit_prediction_track_windows(draw_ctx);

    EXPECT_EQ(state._prediction.orbit_plot_perf.planned_chunk_count, 1u);
    EXPECT_EQ(state._prediction.orbit_plot_perf.planned_chunks_drawn, 1u);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawTrackUsesStalePrefixWhenAwaitingFullRefineHasNoOverlay)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction.draw_full_orbit = false;
    state._prediction.draw_future_segment = false;
    state._prediction.draw_config.draw_planned_as_dashed = false;
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);
    const uint64_t old_plan_signature = state.current_maneuver_plan_signature();
    state._maneuver_state.nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.dirty = true;

    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    OrbitPlotSystem plot{};
    Game::PredictionDrawDetail::PredictionGlobalDrawContext global_ctx = make_draw_global_context(100.0);
    global_ctx.orbit_plot = &plot;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(state.build_orbit_prediction_track_draw_context(track, global_ctx, draw_ctx));
    ASSERT_EQ(draw_ctx.stale_planned_cache, &track.authoritative_cache);
    ASSERT_TRUE(draw_ctx.stale_planned_cache_drawable);

    draw_ctx.direct_world_polyline = true;
    state.draw_orbit_prediction_track_windows(draw_ctx);

    EXPECT_GT(plot.stats().pending_line_count, 0u);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, RuntimeCacheKeepsCachedNodeGizmoAfterEditRelease)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);
    node.position_world = WorldVec3(7'240'000.0, 11.0, 0.0);
    node.basis_r_world = glm::dvec3(1.0, 0.0, 0.0);
    node.basis_t_world = glm::dvec3(0.0, 1.0, 0.0);
    node.basis_n_world = glm::dvec3(0.0, 0.0, 1.0);
    node.maneuver_basis_r_world = node.basis_r_world;
    node.maneuver_basis_t_world = node.basis_t_world;
    node.maneuver_basis_n_world = node.basis_n_world;
    node.gizmo_valid = true;
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);

    Game::GameplayState::ManeuverNode downstream{};
    downstream.id = 8;
    downstream.time_s = 300.0;
    downstream.dv_rtn_mps = glm::dvec3(0.0, 1.0, 0.0);
    state._maneuver_state.nodes.push_back(downstream);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.dirty = true;
    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    state._prediction.tracks.push_back(track);

    Game::GameStateContext ctx{};
    state.refresh_maneuver_node_runtime_cache(ctx);

    ASSERT_EQ(state._maneuver_state.nodes.size(), 2u);
    EXPECT_TRUE(state._maneuver_state.nodes.front().gizmo_valid);
    EXPECT_DOUBLE_EQ(state._maneuver_state.nodes.front().position_world.x, node.position_world.x);
    EXPECT_DOUBLE_EQ(state._maneuver_state.nodes.front().position_world.y, node.position_world.y);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, FullRequestKeepsFullStreamPublishDisabledForStableActivePlayerWithManeuvers)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);

    Game::OrbiterInfo player{};
    player.entity = Game::EntityId{1};
    player.is_player = true;
    state._orbiters.push_back(player);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_EQ(request.maneuver_impulses.size(), 1u);
    EXPECT_FALSE(request.full_stream_publish.active);
}

TEST(GameplayPredictionManeuverTests, FullRequestEnablesFullStreamPublishForPostPreviewRefineWithPendingDerivedWork)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);

    Game::OrbiterInfo player{};
    player.entity = Game::EntityId{1};
    player.is_player = true;
    state._orbiters.push_back(player);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction.selection.active_subject;
    track.supports_maneuvers = true;
    track.cache = make_prediction_cache(4u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = true;
    track.derived_request_pending = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_EQ(request.maneuver_impulses.size(), 1u);
    EXPECT_FALSE(request.planned_suffix_refine.active);
    EXPECT_TRUE(request.full_stream_publish.active);
}

TEST(GameplayPredictionManeuverTests, FullRequestKeepsFullStreamPublishDisabledOutsideActivePlayerManeuverRefine)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);

    Game::OrbiterInfo player{};
    player.entity = Game::EntityId{1};
    player.is_player = true;
    state._orbiters.push_back(player);
    state._prediction.selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::GameplayState::ManeuverNode node{};
    node.id = 9;
    node.time_s = 240.0;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState active_track{};
    active_track.key = state._prediction.selection.active_subject;
    active_track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request no_maneuver_request{};
    ASSERT_TRUE(build_orbiter_prediction_request(state, active_track,
                                                       WorldVec3(7'000'000.0, 0.0, 0.0),
                                                       glm::dvec3(0.0, 7'500.0, 0.0),
                                                       100.0,
                                                       false,
                                                       false,
                                                       no_maneuver_request));
    EXPECT_FALSE(no_maneuver_request.full_stream_publish.active);

    Game::PredictionTrackState overlay_track{};
    overlay_track.key = {Game::PredictionSubjectKind::Orbiter, 2};
    overlay_track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request overlay_request{};
    ASSERT_TRUE(build_orbiter_prediction_request(state, overlay_track,
                                                       WorldVec3(7'000'000.0, 0.0, 0.0),
                                                       glm::dvec3(0.0, 7'500.0, 0.0),
                                                       100.0,
                                                       false,
                                                       true,
                                                       overlay_request));
    EXPECT_EQ(overlay_request.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_EQ(overlay_request.maneuver_impulses.size(), 1u);
    EXPECT_FALSE(overlay_request.full_stream_publish.active);
}

TEST(GameplayPredictionManeuverTests, ShouldRebuildPredictionTrackWhenCoverageFallsShort)
{
    Game::GameplayState state{};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 120.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.identity.valid = true;
    track.cache.identity.build_time_s = 0.0;
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'050'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'050'000.0)};

    EXPECT_TRUE(state.should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));
}

TEST(GameplayPredictionManeuverTests, ShouldRebuildPredictionTrackWhenManeuverCoverageFallsShortOutsideLivePreview)
{
    Game::GameplayState state{};
    state._prediction.draw_future_segment = true;
    state._prediction.sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;
    state._maneuver_plan_live_preview_active = false;

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 400.0;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.identity.valid = true;
    track.cache.identity.build_time_s = 0.0;
    track.cache.solver.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'050'000.0)};
    track.cache.solver.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'050'000.0)};

    EXPECT_TRUE(state.should_rebuild_prediction_track(track, 10.0, 0.016f, false, true));
}

TEST(GameplayPredictionManeuverTests, AwaitFullRefineWaitsForPendingWork)
{
    Game::GameplayState state{};
    state._prediction.sampling_policy.orbiter_min_window_s = 5.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.cache = make_prediction_cache(5u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.dirty = false;
    track.derived_request_pending = true;

    EXPECT_FALSE(state.should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));

    track.derived_request_pending = false;
    EXPECT_TRUE(state.should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));
}

TEST(GameplayPredictionManeuverTests, PendingInvalidationAloneDoesNotForceImmediateRebuild)
{
    Game::GameplayState state{};
    state._prediction.sampling_policy.orbiter_min_window_s = 5.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache = make_prediction_cache(5u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.dirty = false;
    track.request_pending = true;
    track.invalidated_while_pending = true;

    EXPECT_FALSE(state.should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));
}
