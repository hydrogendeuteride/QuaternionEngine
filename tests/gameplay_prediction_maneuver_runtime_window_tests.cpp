#include "gameplay_prediction_maneuver_test_common.h"

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

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowKeepsPlanHorizonAnchoredToNowForNearbyNodes)
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
    EXPECT_DOUBLE_EQ(required_window_s, 600.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowDoesNotAddLargeSolveMarginToNearbyNodes)
{
    Game::GameplayState state{};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_horizon.horizon_s = 600.0;
    state._maneuver_plan_windows.solve_margin_s = 600.0;

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 150.0;
    state._maneuver_state.nodes.push_back(node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = state.prediction_required_window_s(orbiter_key, 100.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 600.0);
}

TEST(GameplayPredictionManeuverTests, FullRequestKeepsPlanHorizonAnchoredToNowWhenAddingNearbyNode)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver_plan_horizon.horizon_s = 600.0;
    state._maneuver_plan_windows.solve_margin_s = 600.0;

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 150.0;
    state._maneuver_state.selected_node_id = node.id;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = state.build_orbiter_prediction_request(track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_DOUBLE_EQ(request.future_window_s, 600.0);
    ASSERT_EQ(request.maneuver_impulses.size(), 1u);
    EXPECT_EQ(request.maneuver_impulses.front().node_id, node.id);
    EXPECT_DOUBLE_EQ(request.maneuver_impulses.front().t_s, node.time_s);
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
    EXPECT_DOUBLE_EQ(required_window_s, 50'020.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowUsesPlanHorizonBeyondBaseSamplingWindow)
{
    Game::GameplayState state{};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s = 60'000.0;
    state._maneuver_plan_horizon.horizon_s = 120'000.0;
    state._maneuver_plan_windows.solve_margin_s = 300.0;

    Game::GameplayState::ManeuverNode node{};
    node.id = 1;
    node.time_s = 1'000.0;
    state._maneuver_state.nodes.push_back(node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = state.prediction_required_window_s(orbiter_key, 0.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 120'000.0);
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
    EXPECT_DOUBLE_EQ(state.prediction_display_window_s(track.key, 100.0, true), 180.0);
    EXPECT_DOUBLE_EQ(state.prediction_preview_exact_window_s(track, 100.0, true), 180.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.request_window_s, 50'260.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.visual_window_s, 180.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.exact_window_s, 300.0);
    EXPECT_DOUBLE_EQ(track.preview_last_anchor_refresh_at_s, 100.0);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheClampsVisualWindowToPreviewWindowDuringDrag)
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
    EXPECT_DOUBLE_EQ(track.preview_anchor.visual_window_s, 180.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.exact_window_s, 300.0);
    EXPECT_LT(track.preview_anchor.visual_window_s, track.preview_anchor.exact_window_s);
    EXPECT_DOUBLE_EQ(track.preview_anchor.request_window_s,
                     2.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheCanReenterDragFromStableIdleTrack)
{
    Game::GameplayState state{};
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
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
    track.key = state._prediction_selection.active_subject;
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

    Game::OrbitPredictionService::Request queued{};
    const bool requested = state.build_orbiter_prediction_request(track,
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

    Game::OrbitPredictionService::Request queued{};
    const bool requested = state.build_orbiter_prediction_request(track,
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
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._prediction_draw_future_segment = true;
    state._prediction_sampling_policy.orbiter_min_window_s =
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
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = state.build_orbiter_prediction_request(track,
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
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
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
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(240.0, 7'100'000.0),
    };
    track.cache.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = selected.id,
            .t_s = selected.time_s,
            .valid = true,
            .inertial_position_m = glm::dvec3(7'250'000.0, 123.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'650.0, 5.0),
    });

    Game::OrbitPredictionService::Request request{};
    const bool built = state.build_orbiter_prediction_request(track,
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
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.position_m.x, 7'250'000.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.position_m.y, 123.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.velocity_mps.y, 7'650.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.velocity_mps.z, 5.0);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestFallsBackToInertialCacheForAnchorState)
{
    Game::GameplayState state{};
    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbitsim);
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
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
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(240.0, 7'100'000.0),
    };

    Game::OrbitPredictionService::Request request{};
    const bool built = state.build_orbiter_prediction_request(track,
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
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.position_m.x, 7'100'000.0);
    EXPECT_DOUBLE_EQ(request.preview_patch.anchor_state_inertial.velocity_mps.y, 7'500.0);
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
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction_selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = state.build_orbiter_prediction_request(track,
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
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state._prediction_selection.active_subject;
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
    const bool built = state.build_orbiter_prediction_request(track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_EQ(request.maneuver_impulses.size(), 1u);
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
    state._prediction_selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::GameplayState::ManeuverNode node{};
    node.id = 9;
    node.time_s = 240.0;
    state._maneuver_state.nodes.push_back(node);

    Game::PredictionTrackState active_track{};
    active_track.key = state._prediction_selection.active_subject;
    active_track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request no_maneuver_request{};
    ASSERT_TRUE(state.build_orbiter_prediction_request(active_track,
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
    ASSERT_TRUE(state.build_orbiter_prediction_request(overlay_track,
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

TEST(GameplayPredictionManeuverTests, AwaitFullRefineWaitsForPendingWork)
{
    Game::GameplayState state{};
    state._prediction_sampling_policy.orbiter_min_window_s = 5.0;

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
    state._prediction_sampling_policy.orbiter_min_window_s = 5.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache = make_prediction_cache(5u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.dirty = false;
    track.request_pending = true;
    track.invalidated_while_pending = true;

    EXPECT_FALSE(state.should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));
}
