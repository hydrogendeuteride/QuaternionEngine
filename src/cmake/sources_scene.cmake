list(APPEND VULKAN_ENGINE_SOURCES
  # scene
  scene/vk_scene.h
  scene/vk_scene_internal.h
  scene/vk_scene.cpp
  scene/vk_scene_update.cpp
  scene/vk_scene_instances.cpp
  scene/vk_scene_physics.cpp
  scene/vk_scene_picking.cpp
  scene/mesh_bvh.h
  scene/mesh_bvh.cpp
  scene/vk_loader.h
  scene/vk_loader.cpp
  scene/vk_loader_load.cpp
  scene/tangent_space.h
  scene/tangent_space.cpp
  scene/camera.h
  scene/camera.cpp

  # scene/camera
  scene/camera/icamera_mode.h
  scene/camera/camera_rig.h
  scene/camera/camera_rig.cpp
  scene/camera/mode_free.h
  scene/camera/mode_free.cpp
  scene/camera/mode_orbit.h
  scene/camera/mode_orbit.cpp
  scene/camera/mode_follow.h
  scene/camera/mode_follow.cpp
  scene/camera/mode_chase.h
  scene/camera/mode_chase.cpp
  scene/camera/mode_fixed.h
  scene/camera/mode_fixed.cpp

  # scene/planet
  scene/planet/cubesphere.h
  scene/planet/cubesphere.cpp
  scene/planet/planet_heightmap.h
  scene/planet/planet_heightmap.cpp
  scene/planet/planet_quadtree.h
  scene/planet/planet_quadtree.cpp
  scene/planet/planet_patch_helpers.h
  scene/planet/planet_patch_helpers.cpp
  scene/planet/planet_system.h
  scene/planet/planet_system.cpp
  scene/planet/planet_terrain.cpp
)
