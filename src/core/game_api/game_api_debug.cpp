#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "core/debug_draw/debug_draw.h"
#include "core/picking/picking_system.h"
#include "render/passes/hover_outline.h"
#include "scene/vk_scene.h"

#include <cmath>

namespace GameAPI
{

namespace
{
    Engine::PickOwnerType to_api_owner_type(RenderObject::OwnerType type)
    {
        switch (type)
        {
            case RenderObject::OwnerType::GLTFInstance:
                return Engine::PickOwnerType::GLTFInstance;
            case RenderObject::OwnerType::MeshInstance:
                return Engine::PickOwnerType::MeshInstance;
            case RenderObject::OwnerType::None:
            default:
                return Engine::PickOwnerType::None;
        }
    }

    Engine::SelectionLevel to_api_selection_level(PickingSystem::SelectionLevel level)
    {
        switch (level)
        {
            case PickingSystem::SelectionLevel::Object:
                return Engine::SelectionLevel::Object;
            case PickingSystem::SelectionLevel::Member:
                return Engine::SelectionLevel::Member;
            case PickingSystem::SelectionLevel::Node:
                return Engine::SelectionLevel::Node;
            case PickingSystem::SelectionLevel::Primitive:
                return Engine::SelectionLevel::Primitive;
            case PickingSystem::SelectionLevel::None:
            default:
                return Engine::SelectionLevel::None;
        }
    }

    PickingSystem::SelectionLevel to_internal_selection_level(Engine::SelectionLevel level)
    {
        switch (level)
        {
            case Engine::SelectionLevel::Object:
                return PickingSystem::SelectionLevel::Object;
            case Engine::SelectionLevel::Member:
                return PickingSystem::SelectionLevel::Member;
            case Engine::SelectionLevel::Node:
                return PickingSystem::SelectionLevel::Node;
            case Engine::SelectionLevel::Primitive:
                return PickingSystem::SelectionLevel::Primitive;
            case Engine::SelectionLevel::None:
            default:
                return PickingSystem::SelectionLevel::None;
        }
    }

    HoverOutlinePass::TargetMode to_outline_target_mode(Engine::SelectionLevel level)
    {
        switch (level)
        {
            case Engine::SelectionLevel::Object:
                return HoverOutlinePass::TargetMode::Object;
            case Engine::SelectionLevel::Member:
                return HoverOutlinePass::TargetMode::Member;
            case Engine::SelectionLevel::Node:
                return HoverOutlinePass::TargetMode::Node;
            case Engine::SelectionLevel::Primitive:
            case Engine::SelectionLevel::None:
            default:
                return HoverOutlinePass::TargetMode::Primitive;
        }
    }

    Engine::SelectionLevel from_outline_target_mode(HoverOutlinePass::TargetMode mode)
    {
        switch (mode)
        {
            case HoverOutlinePass::TargetMode::Object:
                return Engine::SelectionLevel::Object;
            case HoverOutlinePass::TargetMode::Member:
                return Engine::SelectionLevel::Member;
            case HoverOutlinePass::TargetMode::Node:
                return Engine::SelectionLevel::Node;
            case HoverOutlinePass::TargetMode::Primitive:
            default:
                return Engine::SelectionLevel::Primitive;
        }
    }

    template<typename ResultT, typename PositionT>
    ResultT make_pick_result(const PickingSystem::PickInfo &pick, const PositionT &position)
    {
        ResultT r{};
        r.valid = pick.valid;
        r.ownerType = to_api_owner_type(pick.ownerType);
        r.ownerName = pick.ownerName;
        r.objectName = pick.objectName;
        r.memberName = pick.memberName;
        r.nodeName = pick.nodeName;
        r.nodeParentName = pick.nodeParentName;
        r.nodeChildren = pick.nodeChildren;
        r.nodePath = pick.nodePath;
        r.surfaceIndex = pick.surfaceIndex;
        r.selectionLevel = to_api_selection_level(pick.selectionLevel);
        r.worldPosition = position;
        return r;
    }

    bool has_mesh_instance(const SceneManager *scene, const std::string &instance_name)
    {
        if (!scene || instance_name.empty())
        {
            return false;
        }

        glm::mat4 dummy{1.0f};
        return scene->getMeshInstanceTransformLocal(instance_name, dummy);
    }

    bool has_gltf_instance(const SceneManager *scene, const std::string &instance_name)
    {
        return scene && !instance_name.empty() && scene->getGLTFInstanceScene(instance_name) != nullptr;
    }
}

// ----------------------------------------------------------------------------
// Picking / Selection
// ----------------------------------------------------------------------------

Engine::PickResult Engine::get_last_pick() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    return picking ? make_pick_result<PickResult>(picking->last_pick(), glm::vec3(picking->last_pick().worldPos)) : PickResult{};
}

Engine::PickResultD Engine::get_last_pick_d() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    return picking ? make_pick_result<PickResultD>(picking->last_pick(), picking->last_pick().worldPos) : PickResultD{};
}

Engine::PickResult Engine::get_hover_pick() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    return picking ? make_pick_result<PickResult>(picking->hover_pick(), glm::vec3(picking->hover_pick().worldPos)) : PickResult{};
}

Engine::PickResultD Engine::get_hover_pick_d() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    return picking ? make_pick_result<PickResultD>(picking->hover_pick(), picking->hover_pick().worldPos) : PickResultD{};
}

bool Engine::set_last_pick_selection_level(SelectionLevel level)
{
    if (!_engine)
    {
        return false;
    }

    PickingSystem *picking = _engine->picking();
    return picking ? picking->set_last_pick_selection_level(to_internal_selection_level(level)) : false;
}

Engine::SelectionLevel Engine::get_last_pick_selection_level() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (!picking)
    {
        return SelectionLevel::None;
    }

    return to_api_selection_level(picking->last_pick().selectionLevel);
}

bool Engine::select_object_of_last_pick()
{
    if (!_engine)
    {
        return false;
    }

    PickingSystem *picking = _engine->picking();
    return picking ? picking->select_last_pick_object() : false;
}

bool Engine::select_member_of_last_pick()
{
    if (!_engine)
    {
        return false;
    }

    PickingSystem *picking = _engine->picking();
    return picking ? picking->select_last_pick_member() : false;
}

bool Engine::select_parent_of_last_pick()
{
    if (!_engine)
    {
        return false;
    }
    PickingSystem *picking = _engine->picking();
    return picking ? picking->move_last_pick_to_parent() : false;
}

bool Engine::select_child_of_last_pick(size_t childIndex)
{
    if (!_engine)
    {
        return false;
    }
    PickingSystem *picking = _engine->picking();
    return picking ? picking->move_last_pick_to_child(childIndex) : false;
}

bool Engine::select_child_of_last_pick(const std::string &childName)
{
    if (!_engine)
    {
        return false;
    }
    PickingSystem *picking = _engine->picking();
    return picking ? picking->move_last_pick_to_child(childName) : false;
}

bool Engine::bind_instance_to_selection_object(const std::string &instanceName,
                                               const std::string &objectName,
                                               const std::string &memberName)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    PickingSystem *picking = _engine->picking();
    if (!picking || instanceName.empty())
    {
        return false;
    }

    bool bound = false;
    if (has_mesh_instance(_engine->_sceneManager.get(), instanceName))
    {
        picking->set_owner_binding(RenderObject::OwnerType::MeshInstance, instanceName, objectName, memberName);
        bound = true;
    }
    if (has_gltf_instance(_engine->_sceneManager.get(), instanceName))
    {
        picking->set_owner_binding(RenderObject::OwnerType::GLTFInstance, instanceName, objectName, memberName);
        bound = true;
    }
    return bound;
}

void Engine::clear_instance_selection_object_binding(const std::string &instanceName)
{
    if (!_engine)
    {
        return;
    }

    PickingSystem *picking = _engine->picking();
    if (!picking || instanceName.empty())
    {
        return;
    }

    picking->clear_owner_binding(RenderObject::OwnerType::MeshInstance, instanceName);
    picking->clear_owner_binding(RenderObject::OwnerType::GLTFInstance, instanceName);
}

bool Engine::get_instance_selection_object_binding(const std::string &instanceName,
                                                   std::string &outObjectName,
                                                   std::string &outMemberName) const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (!picking || !_engine || !_engine->_sceneManager || instanceName.empty())
    {
        outObjectName.clear();
        outMemberName.clear();
        return false;
    }

    if (has_mesh_instance(_engine->_sceneManager.get(), instanceName) &&
        picking->get_owner_binding(RenderObject::OwnerType::MeshInstance, instanceName, outObjectName, outMemberName))
    {
        return true;
    }

    if (has_gltf_instance(_engine->_sceneManager.get(), instanceName) &&
        picking->get_owner_binding(RenderObject::OwnerType::GLTFInstance, instanceName, outObjectName, outMemberName))
    {
        return true;
    }

    outObjectName.clear();
    outMemberName.clear();
    return false;
}

void Engine::set_outline_settings(const OutlineSettings &settings)
{
    if (!_engine || !_engine->_renderPassManager)
    {
        return;
    }

    if (auto *outline = _engine->_renderPassManager->getPass<HoverOutlinePass>())
    {
        HoverOutlinePass::Settings pass_settings = outline->settings();
        pass_settings.enabled = settings.enabled;
        pass_settings.half_resolution_blur = settings.halfResolutionBlur;
        pass_settings.suppress_hover_when_selected = settings.suppressHoverWhenSelected;

        pass_settings.hover.enabled = settings.hover.enabled;
        pass_settings.hover.color = settings.hover.color;
        pass_settings.hover.intensity = settings.hover.intensity;
        pass_settings.hover.outline_width_px = settings.hover.outlineWidthPx;
        pass_settings.hover.blur_radius_px = settings.hover.blurRadiusPx;
        pass_settings.hover.target_mode = to_outline_target_mode(settings.hover.scope);
        pass_settings.hover.use_pick_selection_level = settings.hover.useSelectionLevel;

        pass_settings.selection.enabled = settings.selection.enabled;
        pass_settings.selection.color = settings.selection.color;
        pass_settings.selection.intensity = settings.selection.intensity;
        pass_settings.selection.outline_width_px = settings.selection.outlineWidthPx;
        pass_settings.selection.blur_radius_px = settings.selection.blurRadiusPx;
        pass_settings.selection.target_mode = to_outline_target_mode(settings.selection.scope);
        pass_settings.selection.use_pick_selection_level = settings.selection.useSelectionLevel;

        outline->settings() = pass_settings;
    }
}

Engine::OutlineSettings Engine::get_outline_settings() const
{
    OutlineSettings out{};
    if (!_engine || !_engine->_renderPassManager)
    {
        return out;
    }

    if (auto *outline = _engine->_renderPassManager->getPass<HoverOutlinePass>())
    {
        const HoverOutlinePass::Settings &pass_settings = outline->settings();
        out.enabled = pass_settings.enabled;
        out.halfResolutionBlur = pass_settings.half_resolution_blur;
        out.suppressHoverWhenSelected = pass_settings.suppress_hover_when_selected;

        out.hover.enabled = pass_settings.hover.enabled;
        out.hover.color = pass_settings.hover.color;
        out.hover.intensity = pass_settings.hover.intensity;
        out.hover.outlineWidthPx = pass_settings.hover.outline_width_px;
        out.hover.blurRadiusPx = pass_settings.hover.blur_radius_px;
        out.hover.scope = from_outline_target_mode(pass_settings.hover.target_mode);
        out.hover.useSelectionLevel = pass_settings.hover.use_pick_selection_level;

        out.selection.enabled = pass_settings.selection.enabled;
        out.selection.color = pass_settings.selection.color;
        out.selection.intensity = pass_settings.selection.intensity;
        out.selection.outlineWidthPx = pass_settings.selection.outline_width_px;
        out.selection.blurRadiusPx = pass_settings.selection.blur_radius_px;
        out.selection.scope = from_outline_target_mode(pass_settings.selection.target_mode);
        out.selection.useSelectionLevel = pass_settings.selection.use_pick_selection_level;
    }

    return out;
}

void Engine::set_outline_enabled(bool enabled)
{
    if (!_engine || !_engine->_renderPassManager)
    {
        return;
    }

    if (auto *outline = _engine->_renderPassManager->getPass<HoverOutlinePass>())
    {
        outline->set_enabled(enabled);
    }
}

bool Engine::get_outline_enabled() const
{
    if (!_engine || !_engine->_renderPassManager)
    {
        return false;
    }

    if (auto *outline = _engine->_renderPassManager->getPass<HoverOutlinePass>())
    {
        return outline->enabled();
    }
    return false;
}

void Engine::set_hover_outline_enabled(bool enabled)
{
    if (!_engine || !_engine->_renderPassManager) return;
    if (auto *outline = _engine->_renderPassManager->getPass<HoverOutlinePass>())
    {
        outline->settings().hover.enabled = enabled;
    }
}

void Engine::set_selection_outline_enabled(bool enabled)
{
    if (!_engine || !_engine->_renderPassManager) return;
    if (auto *outline = _engine->_renderPassManager->getPass<HoverOutlinePass>())
    {
        outline->settings().selection.enabled = enabled;
    }
}

void Engine::set_hover_outline_color(const glm::vec3 &color, float intensity)
{
    if (!_engine || !_engine->_renderPassManager) return;
    if (auto *outline = _engine->_renderPassManager->getPass<HoverOutlinePass>())
    {
        auto &channel = outline->settings().hover;
        channel.color = color;
        if (intensity >= 0.0f) channel.intensity = intensity;
    }
}

void Engine::set_selection_outline_color(const glm::vec3 &color, float intensity)
{
    if (!_engine || !_engine->_renderPassManager) return;
    if (auto *outline = _engine->_renderPassManager->getPass<HoverOutlinePass>())
    {
        auto &channel = outline->settings().selection;
        channel.color = color;
        if (intensity >= 0.0f) channel.intensity = intensity;
    }
}

void Engine::set_picking_enabled(bool enabled)
{
    if (!_engine) return;
    PickingSystem *picking = _engine->picking();
    if (!picking) return;
    picking->settings().enabled = enabled;
}

bool Engine::get_picking_enabled() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (!picking) return false;
    return picking->settings().enabled;
}

void Engine::set_picking_select_button_mask(uint32_t mask)
{
    if (!_engine) return;
    PickingSystem *picking = _engine->picking();
    if (!picking) return;

    constexpr uint32_t kAllButtonsMask = (1u << InputState::kMouseButtonCount) - 1u;
    picking->settings().select_button_mask = (mask & kAllButtonsMask);
}

uint32_t Engine::get_picking_select_button_mask() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (!picking) return 0u;
    return picking->settings().select_button_mask;
}

void Engine::set_picking_click_threshold_px(float px)
{
    if (!_engine) return;
    PickingSystem *picking = _engine->picking();
    if (!picking) return;

    if (!std::isfinite(px) || px < 0.0f)
    {
        px = 0.0f;
    }
    picking->settings().click_threshold_px = px;
}

float Engine::get_picking_click_threshold_px() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (!picking) return 0.0f;
    return picking->settings().click_threshold_px;
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
