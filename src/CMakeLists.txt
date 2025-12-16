
# Add source to this project's executable.
add_executable (vulkan_engine
  main.cpp
  # core root
  core/types.h
  core/world.h
  core/config.h
  core/context.h
  core/context.cpp
  core/engine.h
  core/engine.cpp
  core/engine_ui.cpp
  core/input/input_system.h
  core/input/input_system.cpp
  core/ui/imgui_system.h
  core/ui/imgui_system.cpp
  core/picking/picking_system.h
  core/picking/picking_system.cpp
  core/game_api.h
  core/game_api.cpp
  # core/device
  core/device/device.h
  core/device/device.cpp
  core/device/swapchain.h
  core/device/swapchain.cpp
  core/device/resource.h
  core/device/resource.cpp
  core/device/images.h
  core/device/images.cpp
  # core/descriptor
  core/descriptor/descriptors.h
  core/descriptor/descriptors.cpp
  core/descriptor/manager.h
  core/descriptor/manager.cpp
  # core/pipeline
  core/pipeline/manager.h
  core/pipeline/manager.cpp
  core/pipeline/sampler.h
  core/pipeline/sampler.cpp
  # core/assets
  core/assets/locator.h
  core/assets/locator.cpp
  core/assets/manager.h
  core/assets/manager.cpp
  core/assets/async_loader.h
  core/assets/async_loader.cpp
  core/assets/texture_cache.h
  core/assets/texture_cache.cpp
  core/assets/ktx_loader.h
  core/assets/ktx_loader.cpp
  core/assets/ibl_manager.h
  core/assets/ibl_manager.cpp
  # core/frame
  core/frame/resources.h
  core/frame/resources.cpp
  # core/util
  core/util/initializers.h
  core/util/initializers.cpp
  core/util/debug.h
  core/util/debug.cpp
  # core/raytracing
  core/raytracing/raytracing.h
  core/raytracing/raytracing.cpp
  # render
  render/pipelines.h
  render/pipelines.cpp
  render/renderpass.h
  render/renderpass.cpp
  render/materials.h
  render/materials.cpp
  render/primitives.h
  # render passes
  render/passes/background.h
  render/passes/background.cpp
  render/passes/geometry.h
  render/passes/geometry.cpp
  render/passes/lighting.h
  render/passes/lighting.cpp
  render/passes/shadow.h
  render/passes/shadow.cpp
  render/passes/fxaa.h
  render/passes/fxaa.cpp
  render/passes/ssr.h
  render/passes/ssr.cpp
  render/passes/transparent.h
  render/passes/transparent.cpp
  render/passes/imgui_pass.h
  render/passes/imgui_pass.cpp
  render/passes/tonemap.h
  render/passes/tonemap.cpp
  # render graph
  render/graph/types.h
  render/graph/graph.h
  render/graph/graph.cpp
  render/graph/builder.h
  render/graph/builder.cpp
  render/graph/resources.h
  render/graph/resources.cpp
  # scene
  scene/vk_scene.h
  scene/vk_scene.cpp
  scene/vk_scene_picking.cpp
  scene/mesh_bvh.h
  scene/mesh_bvh.cpp
  scene/vk_loader.h
  scene/vk_loader.cpp
  scene/tangent_space.h
  scene/tangent_space.cpp
  scene/camera.h
  scene/camera.cpp
  # compute
  compute/vk_compute.h
  compute/vk_compute.cpp
)

set_property(TARGET vulkan_engine PROPERTY CXX_STANDARD 20)
target_compile_definitions(vulkan_engine PUBLIC GLM_FORCE_DEPTH_ZERO_TO_ONE)
target_include_directories(vulkan_engine PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/core"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/device"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/descriptor"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/pipeline"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/assets"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/frame"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/util"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/raytracing"
  "${CMAKE_CURRENT_SOURCE_DIR}/render"
  "${CMAKE_CURRENT_SOURCE_DIR}/render/passes"
  "${CMAKE_CURRENT_SOURCE_DIR}/render/graph"
  "${CMAKE_CURRENT_SOURCE_DIR}/scene"
  "${CMAKE_CURRENT_SOURCE_DIR}/compute"
)

option(ENABLE_MIKKTS "Use MikkTSpace for tangent generation" ON)

target_link_libraries(vulkan_engine PUBLIC vma glm Vulkan::Vulkan fmt::fmt stb_image SDL2::SDL2 vkbootstrap imgui fastgltf::fastgltf ImGuizmo BVH2)
if (ENABLE_MIKKTS)
  target_link_libraries(vulkan_engine PUBLIC mikktspace)
  target_compile_definitions(vulkan_engine PUBLIC MIKKTS_ENABLE=1)
endif()

add_library(vma_impl OBJECT vma_impl.cpp)
target_include_directories(vma_impl PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/vma")
target_link_libraries(vma_impl PRIVATE Vulkan::Vulkan)
target_link_libraries(vulkan_engine PUBLIC $<TARGET_OBJECTS:vma_impl>)

target_precompile_headers(vulkan_engine PUBLIC <optional> <vector> <memory> <string> <vector> <unordered_map> <glm/mat4x4.hpp>  <glm/vec4.hpp> <vulkan/vulkan.h>)

add_custom_command(TARGET vulkan_engine POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:vulkan_engine> $<TARGET_FILE_DIR:vulkan_engine>
  COMMAND_EXPAND_LISTS
  )

find_package(ktx CONFIG REQUIRED)
set(_KTX_TARGET "")
if (TARGET ktx::ktx)
  set(_KTX_TARGET ktx::ktx)
elseif (TARGET KTX::ktx)
  set(_KTX_TARGET KTX::ktx)
elseif (TARGET ktx)
  set(_KTX_TARGET ktx)
endif()
if (_KTX_TARGET STREQUAL "")
  message(FATAL_ERROR "libktx not found; please install KTX v2 and expose its CMake package")
endif()
target_link_libraries(vulkan_engine PUBLIC ${_KTX_TARGET})
