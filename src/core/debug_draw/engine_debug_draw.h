#pragma once

#include <core/world.h>

class DebugDrawSystem;
class SceneManager;
class PickingSystem;
class RenderPassManager;
class EngineContext;

// Emits per-frame engine debug primitives (picking/lights/particles/volumetrics/physics) into DebugDrawSystem.
// Keeps VulkanEngine::draw() lean by moving debug visualization wiring into a dedicated module.
void debug_draw_engine_layers(DebugDrawSystem *dd,
                              const WorldVec3 &origin_world,
                              EngineContext *context,
                              SceneManager *scene,
                              PickingSystem *picking,
                              RenderPassManager *render_pass_manager);

