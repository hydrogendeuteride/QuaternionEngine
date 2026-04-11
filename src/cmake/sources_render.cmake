list(APPEND VULKAN_ENGINE_SOURCES
  # render
  render/pipelines.h
  render/pipelines.cpp
  render/renderpass.h
  render/renderpass.cpp
  render/materials.h
  render/materials.cpp
  render/primitives.h

  # render/passes
  render/passes/background.h
  render/passes/background.cpp
  render/passes/sun_disk.h
  render/passes/sun_disk.cpp
  render/passes/geometry.h
  render/passes/geometry.cpp
  render/passes/decal.h
  render/passes/decal.cpp
  render/passes/lighting.h
  render/passes/lighting.cpp
  render/passes/ocean.h
  render/passes/ocean.cpp
  render/passes/shadow.h
  render/passes/shadow.cpp
  render/passes/punctual_shadow.h
  render/passes/punctual_shadow.cpp
  render/passes/fxaa.h
  render/passes/fxaa.cpp
  render/passes/ssr.h
  render/passes/ssr.cpp
  render/passes/clouds.h
  render/passes/clouds.cpp
  render/passes/rocket_plume.h
  render/passes/rocket_plume.cpp
  render/passes/atmosphere.h
  render/passes/atmosphere/atmosphere_internal.h
  render/passes/atmosphere/atmosphere_setup.cpp
  render/passes/atmosphere/atmosphere_textures.cpp
  render/passes/atmosphere/atmosphere_history.cpp
  render/passes/atmosphere/atmosphere_graph.cpp
  render/passes/atmosphere/atmosphere_draw.cpp
  render/passes/particles.h
  render/passes/particles.cpp
  render/passes/mesh_vfx.h
  render/passes/mesh_vfx.cpp
  render/passes/transparent.h
  render/passes/transparent.cpp
  render/passes/imgui_pass.h
  render/passes/imgui_pass.cpp
  render/passes/auto_exposure.h
  render/passes/auto_exposure.cpp
  render/passes/tonemap.h
  render/passes/tonemap.cpp
  render/passes/orbit_plot.h
  render/passes/orbit_plot.cpp
  render/passes/debug_draw.h
  render/passes/debug_draw.cpp

  # render/graph
  render/graph/types.h
  render/graph/graph.h
  render/graph/graph.cpp
  render/graph/builder.h
  render/graph/builder.cpp
  render/graph/resources.h
  render/graph/resources.cpp
)
