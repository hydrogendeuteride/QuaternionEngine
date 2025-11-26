
# Add source to this project's executable.
add_executable (vulkan_engine 
  main.cpp
  # core
  core/vk_types.h
  core/vk_initializers.cpp
  core/vk_initializers.h
  core/vk_images.h
  core/vk_images.cpp 
  core/vk_debug.h
  core/vk_debug.cpp
  core/vk_descriptors.h
  core/vk_descriptors.cpp
  core/vk_device.h
  core/vk_device.cpp
  core/vk_swapchain.h
  core/vk_swapchain.cpp
  core/vk_resource.h
  core/vk_resource.cpp
  core/engine_context.h
  core/engine_context.cpp
  core/vk_descriptor_manager.h
  core/vk_descriptor_manager.cpp
  core/vk_sampler_manager.h
  core/vk_sampler_manager.cpp
  core/asset_locator.h
  core/asset_locator.cpp
  core/asset_manager.h
  core/asset_manager.cpp
  core/vk_pipeline_manager.h
  core/vk_pipeline_manager.cpp
  core/frame_resources.h
  core/frame_resources.cpp
  core/texture_cache.h
  core/texture_cache.cpp
  core/ktx_loader.h
  core/ktx_loader.cpp
  core/config.h
  core/vk_engine.h
  core/vk_engine.cpp
  core/vk_engine_ui.cpp
  core/vk_raytracing.h
  core/vk_raytracing.cpp
  core/ibl_manager.h
  core/ibl_manager.cpp
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
