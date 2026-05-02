list(APPEND VULKAN_ENGINE_SOURCES
  # game
  game/entity.h
  game/entity.cpp
  game/entity_manager.h
  game/entity_manager.cpp
  game/game_world.h
  game/game_world.cpp

  # game/component
  game/component/component.h
  game/component/ship_controller.h
  game/component/ship_controller.cpp

  # game/input
  game/input/keybinds.h
  game/input/keybinds.cpp

  # game/orbit
  game/orbit/orbit_prediction_math.h
  game/orbit/orbit_prediction_math.cpp
  game/orbit/orbit_render_curve.h
  game/orbit/render_curve/orbit_render_curve_internal.h
  game/orbit/render_curve/orbit_render_curve.cpp
  game/orbit/render_curve/orbit_render_curve_render.cpp
  game/orbit/render_curve/orbit_render_curve_pick.cpp
  game/orbit/orbit_plot_util.h
  game/orbit/orbit_plot_util.cpp
  game/orbit/orbit_prediction_tuning.h
  game/orbit/trajectory/trajectory_utils.h
  game/orbit/trajectory/trajectory_utils.cpp
  game/orbit/orbit_prediction_service.h
  game/orbit/orbit_prediction_service.cpp
  game/orbit/prediction/prediction_diagnostics_util.h
  game/orbit/prediction/orbit_prediction_service_internal.h
  game/orbit/prediction/orbit_prediction_service_compute.cpp
  game/orbit/prediction/orbit_prediction_service_route_solvers.cpp
  game/orbit/prediction/orbit_prediction_service_spacecraft_route.cpp
  game/orbit/prediction/orbit_prediction_service_planned_route.cpp
  game/orbit/prediction/orbit_prediction_service_planned_stage_publisher.cpp
  game/orbit/prediction/orbit_prediction_service_planned.cpp
  game/orbit/prediction/orbit_prediction_service_trajectory.cpp
  game/orbit/prediction/orbit_prediction_service_sampling.cpp
  game/orbit/prediction/orbit_prediction_service_policy_integrator.cpp
  game/orbit/prediction/orbit_prediction_service_policy_chunking.cpp
  game/orbit/prediction/orbit_prediction_service_policy_profile.cpp
  game/orbit/prediction/orbit_prediction_service_policy_adaptive.cpp
  game/orbit/prediction/orbit_prediction_service_policy_ephemeris.cpp

  # game root
  game/main_game.h
  game/main_game.cpp

  # game/state
  game/state/game_state.h
  game/state/game_state.cpp
  game/state/game_state_manager.h
  game/state/game_state_manager.cpp

  # game/states
  game/states/title_screen_state.h
  game/states/title_screen_state.cpp
  game/states/gameplay_loading_state.h
  game/states/gameplay_loading_state.cpp
  game/states/pause_state.h
  game/states/pause_state.cpp
  game/states/settings_state.h
  game/states/settings_state.cpp

  # game/states/gameplay
  game/states/gameplay/gameplay_state.h
  game/states/gameplay/gameplay_state.cpp
  game/states/gameplay/gameplay_preload_cache.h
  game/states/gameplay/gameplay_preload_cache.cpp
  game/states/gameplay/gameplay_state_ui.cpp
  game/states/gameplay/gameplay_state_scene.cpp
  game/states/gameplay/gameplay_state_sim.cpp
  game/states/gameplay/gameplay_settings.h
  game/states/gameplay/gameplay_settings.cpp
  game/states/gameplay/formation_hold_system.h
  game/states/gameplay/formation_hold_system.cpp
  game/states/gameplay/frame_monitor.h
  game/states/gameplay/frame_monitor.cpp
  game/states/gameplay/orbit_helpers.h
  game/states/gameplay/orbit_runtime_types.h
  game/states/gameplay/orbiter_physics_bridge.h
  game/states/gameplay/orbiter_physics_bridge.cpp
  game/states/gameplay/orbital_runtime_system.h
  game/states/gameplay/orbital_runtime_system.cpp
  game/states/gameplay/orbital_physics_system.h
  game/states/gameplay/orbital_physics_system.cpp
  game/states/gameplay/time_warp_state.h
  game/states/gameplay/prediction/gameplay_state_prediction_types.h
  game/states/gameplay/prediction/gameplay_prediction_cache_internal.h
  game/states/gameplay/prediction/gameplay_prediction_cache_internal.cpp
  game/states/gameplay/prediction/prediction_trajectory_sampler.h
  game/states/gameplay/prediction/prediction_trajectory_sampler.cpp
  game/states/gameplay/prediction/prediction_frame_resolver.h
  game/states/gameplay/prediction/prediction_frame_resolver.cpp
  game/states/gameplay/prediction/prediction_frame_controller.h
  game/states/gameplay/prediction/prediction_frame_controller.cpp
  game/states/gameplay/prediction/prediction_frame_cache_builder.h
  game/states/gameplay/prediction/prediction_frame_cache_builder.cpp
  game/states/gameplay/prediction/prediction_metrics_builder.h
  game/states/gameplay/prediction/prediction_metrics_builder.cpp
  game/states/gameplay/prediction/streamed_chunk_assembly_builder.h
  game/states/gameplay/prediction/streamed_chunk_assembly_builder.cpp
  game/states/gameplay/prediction/gameplay_prediction_state.h
  game/states/gameplay/prediction/prediction_host_context.h
  game/states/gameplay/prediction/prediction_subject_state_provider.h
  game/states/gameplay/prediction/prediction_subject_state_provider.cpp
  game/states/gameplay/prediction/prediction_system.h
  game/states/gameplay/prediction/prediction_system.cpp
  game/states/gameplay/prediction/gameplay_prediction_derived_service.h
  game/states/gameplay/prediction/gameplay_prediction_derived_service.cpp
  game/states/gameplay/prediction/gameplay_state_prediction.cpp
  game/states/gameplay/prediction/gameplay_state_prediction_frames.cpp
  game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h
  game/states/gameplay/prediction/runtime/prediction_track_lifecycle.h
  game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.h
  game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.cpp
  game/states/gameplay/prediction/runtime/prediction_runtime_context.h
  game/states/gameplay/prediction/runtime/prediction_request_factory.h
  game/states/gameplay/prediction/runtime/prediction_request_factory.cpp
  game/states/gameplay/prediction/runtime/prediction_invalidation_controller.h
  game/states/gameplay/prediction/runtime/prediction_invalidation_controller.cpp
  game/states/gameplay/prediction/runtime/prediction_maneuver_invalidation.h
  game/states/gameplay/prediction/runtime/prediction_maneuver_invalidation.cpp
  game/states/gameplay/prediction/runtime/prediction_result_applier.h
  game/states/gameplay/prediction/runtime/prediction_result_applier.cpp
  game/states/gameplay/prediction/runtime/prediction_solver_result_applier.h
  game/states/gameplay/prediction/runtime/prediction_solver_result_applier.cpp
  game/states/gameplay/prediction/runtime/prediction_runtime_controller.h
  game/states/gameplay/prediction/runtime/prediction_runtime_controller.cpp
  game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime.cpp
  game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_solver.cpp
  game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_requests.cpp
  game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_results.cpp
  game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h
  game/states/gameplay/prediction/draw/gameplay_state_prediction_draw.cpp
  game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_helpers.cpp
  game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_prepare.cpp
  game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_render_helpers.cpp
  game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_shared.cpp
  game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_track.cpp
  game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_pick.cpp
  game/states/gameplay/maneuver/gameplay_state_maneuver_colors.h
  game/states/gameplay/maneuver/gameplay_state_maneuver_types.h
  game/states/gameplay/maneuver/gameplay_state_maneuver_util.h
  game/states/gameplay/maneuver/maneuver_plan_model.h
  game/states/gameplay/maneuver/maneuver_plan_model.cpp
  game/states/gameplay/maneuver/maneuver_commands.h
  game/states/gameplay/maneuver/maneuver_commands.cpp
  game/states/gameplay/maneuver/maneuver_controller.h
  game/states/gameplay/maneuver/maneuver_controller.cpp
  game/states/gameplay/maneuver/maneuver_gizmo_controller.h
  game/states/gameplay/maneuver/maneuver_gizmo_controller.cpp
  game/states/gameplay/maneuver/maneuver_prediction_bridge.h
  game/states/gameplay/maneuver/maneuver_prediction_bridge.cpp
  game/states/gameplay/maneuver/maneuver_ui_controller.h
  game/states/gameplay/maneuver/maneuver_system.h
  game/states/gameplay/maneuver/maneuver_system.cpp
  game/states/gameplay/maneuver/maneuver_runtime_cache_builder.h
  game/states/gameplay/maneuver/maneuver_runtime_cache_builder.cpp
  game/states/gameplay/maneuver/gameplay_state_maneuver_gizmo.cpp
  game/states/gameplay/maneuver/gameplay_state_maneuver_nodes.cpp
  game/states/gameplay/maneuver/gameplay_state_maneuver_nodes_cache.cpp
  game/states/gameplay/maneuver/gameplay_state_maneuver_panel.cpp
  game/states/gameplay/maneuver/gameplay_state_maneuver_runtime.cpp
  game/states/gameplay/maneuver/gameplay_state_maneuver_ui.cpp
  game/states/gameplay/scenario/scenario_config.h
  game/states/gameplay/scenario/scenario_loader.h
  game/states/gameplay/scenario/scenario_loader.cpp

  # game/legacy
  game/legacy/example_game.h
  game/legacy/example_game.cpp
  game/legacy/rebasing_test_game.h
  game/legacy/rebasing_test_game.cpp
)
