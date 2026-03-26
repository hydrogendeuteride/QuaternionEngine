if(NOT TARGET gtest)
  add_subdirectory(
    "${PROJECT_SOURCE_DIR}/third_party/fmt/test/gtest"
    "${CMAKE_BINARY_DIR}/third_party/fmt_gtest"
    EXCLUDE_FROM_ALL
  )
endif()

add_executable(scenario_loader_tests
  scenario_loader_tests.cpp
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/scenario/scenario_loader.cpp"
  "${PROJECT_SOURCE_DIR}/src/core/util/logger.cpp"
)

set_property(TARGET scenario_loader_tests PROPERTY CXX_STANDARD 20)

target_include_directories(scenario_loader_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

set(SCENARIO_TEST_ASSETS_DIR "${CMAKE_CURRENT_BINARY_DIR}/assets/scenarios")
file(MAKE_DIRECTORY "${SCENARIO_TEST_ASSETS_DIR}")
configure_file(
  "${PROJECT_SOURCE_DIR}/tests/fixtures/scenarios/default_gameplay.json"
  "${SCENARIO_TEST_ASSETS_DIR}/default_gameplay.json"
  COPYONLY
)

target_compile_definitions(scenario_loader_tests PRIVATE
  SCENARIO_TEST_ASSETS_DIR="${SCENARIO_TEST_ASSETS_DIR}"
)

target_link_libraries(scenario_loader_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME scenario_loader_tests COMMAND scenario_loader_tests)

add_executable(orbit_helpers_tests
  orbit_helpers_tests.cpp
)

set_property(TARGET orbit_helpers_tests PROPERTY CXX_STANDARD 20)

target_include_directories(orbit_helpers_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(orbit_helpers_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME orbit_helpers_tests COMMAND orbit_helpers_tests)

add_executable(orbit_prediction_planner_tests
  orbit_prediction_planner_tests.cpp
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/prediction/orbit_prediction_service_policy.cpp"
)

set_property(TARGET orbit_prediction_planner_tests PROPERTY CXX_STANDARD 20)

target_include_directories(orbit_prediction_planner_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(orbit_prediction_planner_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME orbit_prediction_planner_tests COMMAND orbit_prediction_planner_tests)

add_executable(orbit_render_curve_tests
  orbit_render_curve_tests.cpp
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_render.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_pick.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_plot_util.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
)

set_property(TARGET orbit_render_curve_tests PROPERTY CXX_STANDARD 20)

target_include_directories(orbit_render_curve_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(orbit_render_curve_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME orbit_render_curve_tests COMMAND orbit_render_curve_tests)

add_executable(time_warp_state_tests
  time_warp_state_tests.cpp
)

set_property(TARGET time_warp_state_tests PROPERTY CXX_STANDARD 20)

target_include_directories(time_warp_state_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(time_warp_state_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME time_warp_state_tests COMMAND time_warp_state_tests)

add_executable(gameplay_time_warp_transition_tests
  gameplay_time_warp_transition_tests.cpp
  gameplay_state_test_stubs.cpp
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/gameplay_state_sim.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_render.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_pick.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_plot_util.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_service.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/prediction/orbit_prediction_service_trajectory.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/prediction/orbit_prediction_service_sampling.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/prediction/orbit_prediction_service_policy.cpp"
)

set_property(TARGET gameplay_time_warp_transition_tests PROPERTY CXX_STANDARD 20)

target_compile_definitions(gameplay_time_warp_transition_tests PRIVATE
  VULKAN_ENGINE_USE_JOLT=0
)

target_include_directories(gameplay_time_warp_transition_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(gameplay_time_warp_transition_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME gameplay_time_warp_transition_tests COMMAND gameplay_time_warp_transition_tests)

add_executable(gameplay_prediction_maneuver_tests
  gameplay_prediction_maneuver_tests.cpp
  gameplay_prediction_maneuver_test_stubs.cpp
  "${PROJECT_SOURCE_DIR}/src/game/entity.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_render.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_pick.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_plot_util.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_service.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/prediction/orbit_prediction_service_trajectory.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/prediction/orbit_prediction_service_sampling.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/prediction/orbit_prediction_service_policy.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/gameplay_state_sim.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction/gameplay_state_prediction.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_solver.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_requests.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_results.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction/gameplay_state_prediction_frames.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction/gameplay_prediction_derived_service.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver/gameplay_state_maneuver_nodes.cpp"
  "${PROJECT_SOURCE_DIR}/third_party/imgui/imgui.cpp"
  "${PROJECT_SOURCE_DIR}/third_party/imgui/imgui_draw.cpp"
  "${PROJECT_SOURCE_DIR}/third_party/imgui/imgui_tables.cpp"
  "${PROJECT_SOURCE_DIR}/third_party/imgui/imgui_widgets.cpp"
)

set_property(TARGET gameplay_prediction_maneuver_tests PROPERTY CXX_STANDARD 20)

target_compile_definitions(gameplay_prediction_maneuver_tests PRIVATE
  VULKAN_ENGINE_USE_JOLT=0
)

target_include_directories(gameplay_prediction_maneuver_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/src/core"
  "${PROJECT_SOURCE_DIR}/third_party/imgui"
  "${PROJECT_SOURCE_DIR}/third_party/vkbootstrap"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(gameplay_prediction_maneuver_tests PRIVATE
  gtest
  fmt::fmt
  glm
  SDL2::SDL2
  orbitsim
)

add_test(NAME gameplay_prediction_maneuver_tests COMMAND gameplay_prediction_maneuver_tests)

add_executable(prediction_cache_internal_tests
  prediction_cache_internal_tests.cpp
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_plot_util.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_render.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_pick.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/prediction/orbit_prediction_service_trajectory.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/prediction/orbit_prediction_service_sampling.cpp"
)

set_property(TARGET prediction_cache_internal_tests PROPERTY CXX_STANDARD 20)

target_include_directories(prediction_cache_internal_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(prediction_cache_internal_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME prediction_cache_internal_tests COMMAND prediction_cache_internal_tests)
