
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
  core/ktx2_loader.h
  core/ktx2_loader.cpp
  core/config.h
  core/vk_engine.h
  core/vk_engine.cpp
  core/vk_raytracing.h
  core/vk_raytracing.cpp
  # render
  render/vk_pipelines.h
  render/vk_pipelines.cpp
  render/vk_renderpass.h
  render/vk_renderpass.cpp
  render/vk_renderpass_background.h
  render/vk_renderpass_background.cpp
  render/vk_renderpass_geometry.h
  render/vk_renderpass_geometry.cpp
  render/vk_renderpass_lighting.h
  render/vk_renderpass_lighting.cpp
  render/vk_renderpass_shadow.h
  render/vk_renderpass_shadow.cpp
  render/vk_renderpass_transparent.h
  render/vk_renderpass_transparent.cpp
  render/vk_renderpass_imgui.h
  render/vk_renderpass_imgui.cpp
  render/vk_renderpass_tonemap.h
  render/vk_renderpass_tonemap.cpp
  # render graph (initial skeleton)
  render/rg_types.h
  render/rg_graph.h
  render/rg_graph.cpp
  render/rg_builder.h
  render/rg_builder.cpp
  render/rg_resources.h
  render/rg_resources.cpp
  render/vk_materials.h
  render/vk_materials.cpp
  render/primitives.h
  # scene
  scene/vk_scene.h
  scene/vk_scene.cpp
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
  "${CMAKE_CURRENT_SOURCE_DIR}/scene"
  "${CMAKE_CURRENT_SOURCE_DIR}/compute"
)

option(ENABLE_MIKKTS "Use MikkTSpace for tangent generation" ON)

target_link_libraries(vulkan_engine PUBLIC vma glm Vulkan::Vulkan fmt::fmt stb_image SDL2::SDL2 vkbootstrap imgui fastgltf::fastgltf)
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

option(ENABLE_LIBKTX "Enable KTX2 loading via libktx" ON)
if (ENABLE_LIBKTX)
  find_package(ktx CONFIG QUIET)
  if (NOT ktx_FOUND)
    find_package(ktx QUIET)
  endif()

  set(_KTX_TARGET "")
  if (TARGET ktx::ktx)
    set(_KTX_TARGET ktx::ktx)
  elseif (TARGET KTX::ktx)
    set(_KTX_TARGET KTX::ktx)
  elseif (TARGET ktx)
    set(_KTX_TARGET ktx)
  endif()

  if (_KTX_TARGET STREQUAL "")
    message(STATUS "libktx not found via find_package; looking for in-tree build...")
  else()
    target_link_libraries(vulkan_engine PUBLIC ${_KTX_TARGET})
    target_compile_definitions(vulkan_engine PUBLIC VULKAN_ENGINE_HAS_KTX=1)
  endif()
endif()
