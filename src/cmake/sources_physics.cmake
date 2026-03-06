list(APPEND VULKAN_ENGINE_SOURCES
  # physics
  physics/collision_shape.h
  physics/collider_mesh_instance.h
  physics/body_settings.h
  physics/collider_asset.h
  physics/collider_asset.cpp
  physics/gltf_collider_parser.h
  physics/gltf_collider_parser.cpp
  physics/physics_body.h
  physics/physics_body.cpp
  physics/physics_world.h
  physics/physics_context.h
  physics/physics_context.cpp

  # physics/jolt
  physics/jolt/jolt_physics_world.h
  physics/jolt/jolt_physics_world.cpp
  physics/jolt/jolt_query_filters.h
)
