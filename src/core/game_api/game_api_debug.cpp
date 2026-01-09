#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "core/debug_draw/debug_draw.h"
#include "core/picking/picking_system.h"

namespace GameAPI
{

// ----------------------------------------------------------------------------
// Picking / Selection
// ----------------------------------------------------------------------------

Engine::PickResult Engine::get_last_pick() const
{
    PickResult r;
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (picking)
    {
        const auto &pick = picking->last_pick();
        r.valid = pick.valid;
        r.ownerName = pick.ownerName;
        r.worldPosition = glm::vec3(pick.worldPos);
    }
    return r;
}

Engine::PickResultD Engine::get_last_pick_d() const
{
    PickResultD r;
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (picking)
    {
        const auto &pick = picking->last_pick();
        r.valid = pick.valid;
        r.ownerName = pick.ownerName;
        r.worldPosition = pick.worldPos;
    }
    return r;
}

void Engine::set_use_id_buffer_picking(bool use)
{
    if (!_engine) return;
    PickingSystem *picking = _engine->picking();
    if (!picking) return;
    picking->set_use_id_buffer_picking(use);
}

bool Engine::get_use_id_buffer_picking() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (!picking) return false;
    return picking->use_id_buffer_picking();
}

// ----------------------------------------------------------------------------
// Debug Drawing
// ----------------------------------------------------------------------------

void Engine::set_debug_draw_enabled(bool enabled)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().enabled = enabled;
}

bool Engine::get_debug_draw_enabled() const
{
    if (!_engine || !_engine->_debugDraw) return false;
    return _engine->_debugDraw->settings().enabled;
}

void Engine::set_debug_layer_mask(uint32_t mask)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().layer_mask = mask;
}

uint32_t Engine::get_debug_layer_mask() const
{
    if (!_engine || !_engine->_debugDraw) return 0;
    return _engine->_debugDraw->settings().layer_mask;
}

void Engine::set_debug_show_depth_tested(bool show)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().show_depth_tested = show;
}

bool Engine::get_debug_show_depth_tested() const
{
    if (!_engine || !_engine->_debugDraw) return true;
    return _engine->_debugDraw->settings().show_depth_tested;
}

void Engine::set_debug_show_overlay(bool show)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().show_overlay = show;
}

bool Engine::get_debug_show_overlay() const
{
    if (!_engine || !_engine->_debugDraw) return true;
    return _engine->_debugDraw->settings().show_overlay;
}

void Engine::set_debug_segments(int segments)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().segments = segments;
}

int Engine::get_debug_segments() const
{
    if (!_engine || !_engine->_debugDraw) return 32;
    return _engine->_debugDraw->settings().segments;
}

void Engine::debug_draw_clear()
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->clear();
}

void Engine::debug_draw_line(const glm::vec3& a, const glm::vec3& b,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_line(WorldVec3(a), WorldVec3(b), color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_line(const glm::dvec3& a, const glm::dvec3& b,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_line(WorldVec3(a), WorldVec3(b), color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_ray(const glm::vec3& origin, const glm::vec3& direction, float length,
                             const glm::vec4& color,
                             float duration_seconds,
                             bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_ray(WorldVec3(origin), glm::dvec3(direction), length, color, duration_seconds,
                                  depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                  DebugDrawLayer::Misc);
}

void Engine::debug_draw_ray(const glm::dvec3& origin, const glm::dvec3& direction, double length,
                             const glm::vec4& color,
                             float duration_seconds,
                             bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_ray(WorldVec3(origin), direction, length, color, duration_seconds,
                                  depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                  DebugDrawLayer::Misc);
}

void Engine::debug_draw_aabb(const glm::vec3& center, const glm::vec3& half_extents,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_aabb(WorldVec3(center), half_extents, color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_aabb(const glm::dvec3& center, const glm::vec3& half_extents,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_aabb(WorldVec3(center), half_extents, color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_sphere(const glm::vec3& center, float radius,
                                const glm::vec4& color,
                                float duration_seconds,
                                bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_sphere(WorldVec3(center), radius, color, duration_seconds,
                                     depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Misc);
}

void Engine::debug_draw_sphere(const glm::dvec3& center, float radius,
                                const glm::vec4& color,
                                float duration_seconds,
                                bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_sphere(WorldVec3(center), radius, color, duration_seconds,
                                     depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Misc);
}

void Engine::debug_draw_capsule(const glm::vec3& p0, const glm::vec3& p1, float radius,
                                 const glm::vec4& color,
                                 float duration_seconds,
                                 bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_capsule(WorldVec3(p0), WorldVec3(p1), radius, color, duration_seconds,
                                      depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                      DebugDrawLayer::Misc);
}

void Engine::debug_draw_capsule(const glm::dvec3& p0, const glm::dvec3& p1, float radius,
                                 const glm::vec4& color,
                                 float duration_seconds,
                                 bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_capsule(WorldVec3(p0), WorldVec3(p1), radius, color, duration_seconds,
                                      depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                      DebugDrawLayer::Misc);
}

void Engine::debug_draw_circle(const glm::vec3& center, const glm::vec3& normal, float radius,
                                const glm::vec4& color,
                                float duration_seconds,
                                bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_circle(WorldVec3(center), glm::dvec3(normal), radius, color, duration_seconds,
                                     depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Misc);
}

void Engine::debug_draw_circle(const glm::dvec3& center, const glm::dvec3& normal, float radius,
                                const glm::vec4& color,
                                float duration_seconds,
                                bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_circle(WorldVec3(center), normal, radius, color, duration_seconds,
                                     depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Misc);
}

void Engine::debug_draw_cone(const glm::vec3& apex, const glm::vec3& direction,
                              float length, float angle_degrees,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_cone(WorldVec3(apex), glm::dvec3(direction), length, angle_degrees,
                                   color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_cone(const glm::dvec3& apex, const glm::dvec3& direction,
                              float length, float angle_degrees,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_cone(WorldVec3(apex), direction, length, angle_degrees,
                                   color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

} // namespace GameAPI
