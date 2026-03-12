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

  # game/orbit
  game/orbit/orbit_prediction_math.h
  game/orbit/orbit_prediction_math.cpp
  game/orbit/orbit_render_curve.h
  game/orbit/orbit_render_curve.cpp
  game/orbit/orbit_plot_lod_builder.h
  game/orbit/orbit_plot_lod_builder.cpp
  game/orbit/orbit_prediction_tuning.h
  game/orbit/orbit_prediction_service.h
  game/orbit/orbit_prediction_service.cpp

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
  game/states/pause_state.h
  game/states/pause_state.cpp
  game/states/settings_state.h
  game/states/settings_state.cpp

  # game/states/gameplay
  game/states/gameplay/gameplay_state.h
  game/states/gameplay/gameplay_state.cpp
  game/states/gameplay/gameplay_state_scene.cpp
  game/states/gameplay/gameplay_state_sim.cpp
  game/states/gameplay/frame_monitor.h
  game/states/gameplay/frame_monitor.cpp
  game/states/gameplay/orbit_helpers.h
  game/states/gameplay/time_warp_state.h
  game/states/gameplay/prediction/gameplay_state_prediction_types.h
  game/states/gameplay/prediction/gameplay_prediction_derived_service.h
  game/states/gameplay/prediction/gameplay_prediction_derived_service.cpp
  game/states/gameplay/prediction/gameplay_state_prediction.cpp
  game/states/gameplay/prediction/gameplay_state_prediction_frames.cpp
  game/states/gameplay/prediction/gameplay_state_prediction_runtime.cpp
  game/states/gameplay/prediction/gameplay_state_prediction_draw_internal.h
  game/states/gameplay/prediction/gameplay_state_prediction_draw.cpp
  game/states/gameplay/prediction/gameplay_state_prediction_draw_helpers.cpp
  game/states/gameplay/maneuver/gameplay_state_maneuver_types.h
  game/states/gameplay/maneuver/gameplay_state_maneuver_nodes.cpp
  game/states/gameplay/scenario/scenario_config.h
  game/states/gameplay/scenario/scenario_loader.h
  game/states/gameplay/scenario/scenario_loader.cpp

  # game/legacy
  game/legacy/example_game.h
  game/legacy/example_game.cpp
  game/legacy/rebasing_test_game.h
  game/legacy/rebasing_test_game.cpp
)
