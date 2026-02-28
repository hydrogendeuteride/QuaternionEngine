if(NOT TARGET gtest)
  add_subdirectory(
    "${PROJECT_SOURCE_DIR}/third_party/fmt/test/gtest"
    "${CMAKE_BINARY_DIR}/third_party/fmt_gtest"
    EXCLUDE_FROM_ALL
  )
endif()

add_executable(scenario_loader_tests
  scenario_loader_tests.cpp
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/scenario_loader.cpp"
  "${PROJECT_SOURCE_DIR}/src/core/util/logger.cpp"
)

set_property(TARGET scenario_loader_tests PROPERTY CXX_STANDARD 20)

target_include_directories(scenario_loader_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
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
