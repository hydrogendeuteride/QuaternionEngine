#pragma once

#include <core/world.h>

class DebugDrawSystem;

struct PhysicsDebugSettings;

namespace Physics
{
    class PhysicsWorld;
}

// Draws physics colliders into DebugDrawSystem (wireframe) using the Physics debug body iterator.
// This is intentionally decoupled from VulkanEngine; pass the current world origin from SceneManager.
void debug_draw_physics_colliders(DebugDrawSystem *dd,
                                 const WorldVec3 &origin_world,
                                 const Physics::PhysicsWorld *physics,
                                 const PhysicsDebugSettings &settings);

