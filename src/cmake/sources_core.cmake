list(APPEND VULKAN_ENGINE_SOURCES
  # core root
  core/types.h
  core/world.h
  core/config.h
  core/context.h
  core/context.cpp
  core/engine.h
  core/engine.cpp
  core/engine_ui.cpp
  core/game_api.h

  # core/input
  core/input/input_system.h
  core/input/input_system.cpp

  # core/ui
  core/ui/imgui_system.h
  core/ui/imgui_system.cpp

  # core/picking
  core/picking/picking_system.h
  core/picking/picking_system.cpp

  # core/debug_draw
  core/debug_draw/debug_draw.h
  core/debug_draw/debug_draw.cpp
  core/debug_draw/engine_debug_draw.h
  core/debug_draw/engine_debug_draw.cpp

  # core/orbit_plot
  core/orbit_plot/orbit_plot.h
  core/orbit_plot/orbit_plot.cpp

  # core/game_api
  core/game_api/game_api.cpp
  core/game_api/game_api_textures.cpp
  core/game_api/game_api_scene.cpp
  core/game_api/game_api_planets.cpp
  core/game_api/game_api_lighting.cpp
  core/game_api/game_api_camera.cpp
  core/game_api/game_api_postfx.cpp
  core/game_api/game_api_particles.cpp
  core/game_api/game_api_mesh_vfx.cpp
  core/game_api/game_api_blackbody.cpp
  core/game_api/game_api_volumetrics.cpp
  core/game_api/game_api_rocket_plumes.cpp
  core/game_api/game_api_debug.cpp

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
  core/util/logger.h
  core/util/logger.cpp

  # core/raytracing
  core/raytracing/raytracing.h
  core/raytracing/raytracing.cpp
)
