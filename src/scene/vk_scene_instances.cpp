#include "vk_scene.h"

#include "core/context.h"
#include "core/frame/resources.h"
#include "core/util/logger.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"
#include "vk_scene_internal.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
    bool get_instance_node_world(const SceneManager::GLTFInstance &inst,
                                 const std::string &node_name,
                                 glm::mat4 &out_node_world)
    {
        if (!inst.scene)
        {
            return false;
        }

        auto node_ptr = inst.scene->getNode(node_name);
        if (!node_ptr)
        {
            return false;
        }

        out_node_world = inst.nodeLocalOverrides.empty()
                             ? node_ptr->worldTransform
                             : SceneInternal::build_node_world_with_overrides(node_ptr.get(), inst.nodeLocalOverrides);
        return true;
    }
}

void SceneManager::addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh,
                                   const glm::mat4 &transform, std::optional<BoundsType> boundsType)
{
    if (!mesh)
    {
        return;
    }

    MeshInstance inst{};
    inst.mesh = std::move(mesh);
    glm::vec3 t{};
    decompose_trs_matrix(transform, t, inst.rotation, inst.scale);
    inst.translation_world = WorldVec3(t);
    inst.boundsTypeOverride = boundsType;
    dynamicMeshInstances[name] = std::move(inst);
}

bool SceneManager::getMeshInstanceTransform(const std::string &name, glm::mat4 &outTransform)
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    const MeshInstance &inst = it->second;
    outTransform = make_trs_matrix(glm::vec3(inst.translation_world), inst.rotation, inst.scale);
    return true;
}

bool SceneManager::setMeshInstanceTransform(const std::string &name, const glm::mat4 &transform)
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    MeshInstance &inst = it->second;
    glm::vec3 t{};
    decompose_trs_matrix(transform, t, inst.rotation, inst.scale);
    inst.translation_world = WorldVec3(t);
    return true;
}

bool SceneManager::getMeshInstanceTransformLocal(const std::string &name, glm::mat4 &outTransformLocal) const
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    const MeshInstance &inst = it->second;
    const glm::vec3 t_local = world_to_local(inst.translation_world, get_world_origin());
    outTransformLocal = make_trs_matrix(t_local, inst.rotation, inst.scale);
    return true;
}

bool SceneManager::setMeshInstanceTransformLocal(const std::string &name, const glm::mat4 &transformLocal)
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    MeshInstance &inst = it->second;
    glm::vec3 t_local{};
    decompose_trs_matrix(transformLocal, t_local, inst.rotation, inst.scale);
    inst.translation_world = local_to_world(t_local, get_world_origin());
    return true;
}

bool SceneManager::getMeshInstanceTRSWorld(const std::string &name,
                                           WorldVec3 &outTranslationWorld,
                                           glm::quat &outRotation,
                                           glm::vec3 &outScale) const
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    const MeshInstance &inst = it->second;
    outTranslationWorld = inst.translation_world;
    outRotation = inst.rotation;
    outScale = inst.scale;
    return true;
}

bool SceneManager::setMeshInstanceTRSWorld(const std::string &name,
                                           const WorldVec3 &translationWorld,
                                           const glm::quat &rotation,
                                           const glm::vec3 &scale)
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    MeshInstance &inst = it->second;
    inst.translation_world = translationWorld;
    inst.rotation = rotation;
    inst.scale = scale;
    return true;
}

bool SceneManager::setMeshInstanceMaterial(const std::string &name, std::shared_ptr<GLTFMaterial> material)
{
    if (!material)
    {
        return false;
    }

    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    MeshInstance &inst = it->second;
    if (!inst.mesh || inst.mesh->surfaces.empty())
    {
        return false;
    }
    if (inst.mesh->name.rfind("Primitive.", 0) != 0)
    {
        return false;
    }

    auto unique_mesh = std::make_shared<MeshAsset>(*inst.mesh);
    for (auto &surf : unique_mesh->surfaces)
    {
        surf.material = material;
    }
    inst.mesh = std::move(unique_mesh);
    return true;
}

bool SceneManager::removeMeshInstance(const std::string &name)
{
    return dynamicMeshInstances.erase(name) > 0;
}

void SceneManager::clearMeshInstances()
{
    dynamicMeshInstances.clear();
}

bool SceneManager::setDynamicInstanceTransform(const std::string &name, const glm::mat4 &transform)
{
    return setMeshInstanceTransform(name, transform) || setGLTFInstanceTransform(name, transform);
}

bool SceneManager::setDynamicInstanceTRSWorld(const std::string &name,
                                              const WorldVec3 &translationWorld,
                                              const glm::quat &rotation,
                                              const glm::vec3 &scale)
{
    return setMeshInstanceTRSWorld(name, translationWorld, rotation, scale) ||
           setGLTFInstanceTRSWorld(name, translationWorld, rotation, scale);
}

bool SceneManager::removeDynamicInstance(const std::string &name)
{
    return removeMeshInstance(name) || removeGLTFInstance(name);
}

bool SceneManager::setDecal(const std::string &name, const DecalInstance &decal)
{
    if (name.empty())
    {
        return false;
    }

    const auto it = dynamicDecals.find(name);
    if (it == dynamicDecals.end() && dynamicDecals.size() >= kMaxDecals)
    {
        Logger::warn("[SceneManager] Decal limit ({}) reached -- '{}' dropped.", kMaxDecals, name);
        return false;
    }

    DecalInstance clamped = decal;
    const float qlen2 = glm::dot(clamped.rotation, clamped.rotation);
    if (qlen2 > 1.0e-8f)
    {
        clamped.rotation = glm::normalize(clamped.rotation);
    }
    else
    {
        clamped.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }

    clamped.half_extents = glm::max(glm::abs(clamped.half_extents), glm::vec3(1.0e-3f));
    if (clamped.shape == DecalShape::Sphere)
    {
        const float r = std::max(clamped.half_extents.x, std::max(clamped.half_extents.y, clamped.half_extents.z));
        clamped.half_extents = glm::vec3(r);
    }
    clamped.opacity = glm::clamp(clamped.opacity, 0.0f, 1.0f);
    clamped.normalStrength = std::max(0.0f, clamped.normalStrength);
    clamped.tint = glm::max(clamped.tint, glm::vec3(0.0f));

    dynamicDecals[name] = clamped;
    return true;
}

bool SceneManager::getDecal(const std::string &name, DecalInstance &outDecal) const
{
    const auto it = dynamicDecals.find(name);
    if (it == dynamicDecals.end())
    {
        return false;
    }

    outDecal = it->second;
    return true;
}

bool SceneManager::removeDecal(const std::string &name)
{
    return dynamicDecals.erase(name) > 0;
}

void SceneManager::clearDecals()
{
    dynamicDecals.clear();
}

void SceneManager::addGLTFInstance(const std::string &name, std::shared_ptr<LoadedGLTF> scene,
                                   const glm::mat4 &transform)
{
    if (!scene)
    {
        return;
    }

    Logger::info("[SceneManager] addGLTFInstance '{}' (scene='{}')",
                 name,
                 scene->debugName.empty() ? "<unnamed>" : scene->debugName.c_str());

    GLTFInstance inst{};
    inst.scene = std::move(scene);
    glm::vec3 t{};
    decompose_trs_matrix(transform, t, inst.rotation, inst.scale);
    inst.translation_world = WorldVec3(t);
    if (inst.scene && !inst.scene->animations.empty())
    {
        inst.animation.activeAnimation = 0;
        inst.animation.animationTime = 0.0f;
        inst.animation.animationLoop = true;
    }

    dynamicGLTFInstances[name] = std::move(inst);
}

bool SceneManager::removeGLTFInstance(const std::string &name)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    disableColliderSync(name);
    disableDynamicRootColliderBody(name);

    if (it->second.scene)
    {
        if (_context && _context->currentFrame)
        {
            auto keep_alive = it->second.scene;
            Logger::info("[SceneManager] removeGLTFInstance '{}' scheduling deferred destroy (scene='{}')",
                         name,
                         keep_alive && !keep_alive->debugName.empty() ? keep_alive->debugName.c_str() : "<unnamed>");
            _context->currentFrame->_deletionQueue.push_function([keep_alive]() mutable { keep_alive.reset(); });
        }
        else
        {
            pendingGLTFRelease.push_back(it->second.scene);
        }
    }

    dynamicGLTFInstances.erase(it);
    return true;
}

std::shared_ptr<LoadedGLTF> SceneManager::getGLTFInstanceScene(const std::string &instanceName) const
{
    auto it = dynamicGLTFInstances.find(instanceName);
    return it != dynamicGLTFInstances.end() ? it->second.scene : nullptr;
}

bool SceneManager::getGLTFInstanceNodeWorldTransform(const std::string &instanceName,
                                                     const std::string &nodeName,
                                                     glm::mat4 &outWorldTransform) const
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    glm::mat4 node_world(1.0f);
    if (!get_instance_node_world(it->second, nodeName, node_world))
    {
        return false;
    }

    const GLTFInstance &inst = it->second;
    const glm::mat4 inst_world = make_trs_matrix(glm::vec3(inst.translation_world), inst.rotation, inst.scale);
    outWorldTransform = inst_world * node_world;
    return true;
}

glm::mat4 SceneManager::getGLTFInstanceNodeWorldTransform(const std::string &instanceName,
                                                          const std::string &nodeName) const
{
    glm::mat4 world(1.0f);
    (void)getGLTFInstanceNodeWorldTransform(instanceName, nodeName, world);
    return world;
}

bool SceneManager::getGLTFInstanceNodeWorldTransformLocal(const std::string &instanceName,
                                                          const std::string &nodeName,
                                                          glm::mat4 &outWorldTransformLocal) const
{
    return getGLTFInstanceNodeWorldTransformLocal(instanceName, nodeName, get_world_origin(), outWorldTransformLocal);
}

bool SceneManager::getGLTFInstanceNodeWorldTransformLocal(const std::string &instanceName,
                                                          const std::string &nodeName,
                                                          const WorldVec3 &origin_world,
                                                          glm::mat4 &outWorldTransformLocal) const
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    glm::mat4 node_world(1.0f);
    if (!get_instance_node_world(it->second, nodeName, node_world))
    {
        return false;
    }

    const GLTFInstance &inst = it->second;
    const glm::vec3 t_local = world_to_local(inst.translation_world, origin_world);
    const glm::mat4 inst_local = make_trs_matrix(t_local, inst.rotation, inst.scale);
    outWorldTransformLocal = inst_local * node_world;
    return true;
}

glm::mat4 SceneManager::getGLTFInstanceNodeWorldTransformLocal(const std::string &instanceName,
                                                               const std::string &nodeName) const
{
    glm::mat4 world_local(1.0f);
    (void)getGLTFInstanceNodeWorldTransformLocal(instanceName, nodeName, world_local);
    return world_local;
}

glm::mat4 SceneManager::getGLTFInstanceNodeWorldTransformLocal(const std::string &instanceName,
                                                               const std::string &nodeName,
                                                               const WorldVec3 &origin_world) const
{
    glm::mat4 world_local(1.0f);
    (void)getGLTFInstanceNodeWorldTransformLocal(instanceName, nodeName, origin_world, world_local);
    return world_local;
}

bool SceneManager::getGLTFInstanceTransform(const std::string &name, glm::mat4 &outTransform)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    const GLTFInstance &inst = it->second;
    outTransform = make_trs_matrix(glm::vec3(inst.translation_world), inst.rotation, inst.scale);
    return true;
}

bool SceneManager::setGLTFInstanceTransform(const std::string &name, const glm::mat4 &transform)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    GLTFInstance &inst = it->second;
    glm::vec3 t{};
    decompose_trs_matrix(transform, t, inst.rotation, inst.scale);
    inst.translation_world = WorldVec3(t);

    if (auto body_it = _dynamicRootColliderBodies.find(name); body_it != _dynamicRootColliderBodies.end())
    {
        DynamicRootColliderBodyEntry &entry = body_it->second;
        constexpr float kScaleEps = 1.0e-3f;
        const bool scale_changed =
            std::abs(inst.scale.x - entry.scale.x) > kScaleEps ||
            std::abs(inst.scale.y - entry.scale.y) > kScaleEps ||
            std::abs(inst.scale.z - entry.scale.z) > kScaleEps;
        if (scale_changed)
        {
            Logger::info("[SceneManager] setGLTFInstanceTransform: rebuilding dynamic root collider body for '{}' after scale change",
                         name);
            return rebuildDynamicRootColliderBody(name, entry, true);
        }
        if (entry.world && entry.world->is_body_valid(entry.body))
        {
            const WorldVec3 physics_origin_world =
                (_context && _context->physics_context) ? _context->physics_context->origin_world() : WorldVec3{0.0, 0.0, 0.0};
            const WorldVec3 body_position_world =
                inst.translation_world + WorldVec3(inst.rotation * entry.center_of_mass_local);
            entry.world->set_transform(entry.body, world_to_local_d(body_position_world, physics_origin_world), inst.rotation);
        }
    }

    return true;
}

bool SceneManager::getGLTFInstanceTransformLocal(const std::string &name, glm::mat4 &outTransformLocal) const
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    const GLTFInstance &inst = it->second;
    const glm::vec3 t_local = world_to_local(inst.translation_world, get_world_origin());
    outTransformLocal = make_trs_matrix(t_local, inst.rotation, inst.scale);
    return true;
}

bool SceneManager::setGLTFInstanceTransformLocal(const std::string &name, const glm::mat4 &transformLocal)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    GLTFInstance &inst = it->second;
    glm::vec3 t_local{};
    decompose_trs_matrix(transformLocal, t_local, inst.rotation, inst.scale);
    inst.translation_world = local_to_world(t_local, get_world_origin());

    if (auto body_it = _dynamicRootColliderBodies.find(name); body_it != _dynamicRootColliderBodies.end())
    {
        DynamicRootColliderBodyEntry &entry = body_it->second;
        constexpr float kScaleEps = 1.0e-3f;
        const bool scale_changed =
            std::abs(inst.scale.x - entry.scale.x) > kScaleEps ||
            std::abs(inst.scale.y - entry.scale.y) > kScaleEps ||
            std::abs(inst.scale.z - entry.scale.z) > kScaleEps;
        if (scale_changed)
        {
            Logger::info("[SceneManager] setGLTFInstanceTransformLocal: rebuilding dynamic root collider body for '{}' after scale change",
                         name);
            return rebuildDynamicRootColliderBody(name, entry, true);
        }
        if (entry.world && entry.world->is_body_valid(entry.body))
        {
            const WorldVec3 physics_origin_world =
                (_context && _context->physics_context) ? _context->physics_context->origin_world() : WorldVec3{0.0, 0.0, 0.0};
            const WorldVec3 body_position_world =
                inst.translation_world + WorldVec3(inst.rotation * entry.center_of_mass_local);
            entry.world->set_transform(entry.body, world_to_local_d(body_position_world, physics_origin_world), inst.rotation);
        }
    }

    return true;
}

bool SceneManager::getGLTFInstanceTRSWorld(const std::string &name,
                                           WorldVec3 &outTranslationWorld,
                                           glm::quat &outRotation,
                                           glm::vec3 &outScale) const
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    const GLTFInstance &inst = it->second;
    outTranslationWorld = inst.translation_world;
    outRotation = inst.rotation;
    outScale = inst.scale;
    return true;
}

bool SceneManager::setGLTFInstanceTRSWorld(const std::string &name,
                                           const WorldVec3 &translationWorld,
                                           const glm::quat &rotation,
                                           const glm::vec3 &scale)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    GLTFInstance &inst = it->second;
    inst.translation_world = translationWorld;
    inst.rotation = rotation;
    inst.scale = scale;

    if (auto body_it = _dynamicRootColliderBodies.find(name); body_it != _dynamicRootColliderBodies.end())
    {
        DynamicRootColliderBodyEntry &entry = body_it->second;
        constexpr float kScaleEps = 1.0e-3f;
        const bool scale_changed =
            std::abs(scale.x - entry.scale.x) > kScaleEps ||
            std::abs(scale.y - entry.scale.y) > kScaleEps ||
            std::abs(scale.z - entry.scale.z) > kScaleEps;
        if (scale_changed)
        {
            Logger::info("[SceneManager] setGLTFInstanceTRSWorld: rebuilding dynamic root collider body for '{}' after scale change",
                         name);
            return rebuildDynamicRootColliderBody(name, entry, true);
        }

        if (entry.world && entry.world->is_body_valid(entry.body))
        {
            const WorldVec3 physics_origin_world =
                (_context && _context->physics_context) ? _context->physics_context->origin_world() : WorldVec3{0.0, 0.0, 0.0};
            const WorldVec3 body_position_world =
                inst.translation_world + WorldVec3(inst.rotation * entry.center_of_mass_local);
            entry.world->set_transform(entry.body, world_to_local_d(body_position_world, physics_origin_world), inst.rotation);
        }
    }

    return true;
}

void SceneManager::clearGLTFInstances()
{
    Logger::info("[SceneManager] clearGLTFInstances: dynamicGLTFInstances={} pendingBefore={}",
                 dynamicGLTFInstances.size(),
                 pendingGLTFRelease.size());

    for (auto &[entry_name, entry] : _colliderSyncEntries)
    {
        destroyColliderSyncEntry(entry);
    }
    _colliderSyncEntries.clear();

    for (auto &[entry_name, entry] : _dynamicRootColliderBodies)
    {
        destroyDynamicRootColliderBodyEntry(entry);
    }
    _dynamicRootColliderBodies.clear();

    for (auto &kv : dynamicGLTFInstances)
    {
        if (kv.second.scene)
        {
            pendingGLTFRelease.push_back(kv.second.scene);
        }
    }

    dynamicGLTFInstances.clear();
    Logger::info("[SceneManager] clearGLTFInstances: pendingAfter={}",
                 pendingGLTFRelease.size());
}

bool SceneManager::setGLTFInstanceNodeOffset(const std::string &instanceName,
                                             const std::string &nodeName,
                                             const glm::mat4 &offset)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    GLTFInstance &inst = it->second;
    if (!inst.scene)
    {
        return false;
    }

    auto node_ptr = inst.scene->getNode(nodeName);
    if (!node_ptr)
    {
        return false;
    }

    inst.nodeLocalOverrides[node_ptr.get()] = offset;
    return true;
}

bool SceneManager::clearGLTFInstanceNodeOffset(const std::string &instanceName,
                                               const std::string &nodeName)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    GLTFInstance &inst = it->second;
    if (!inst.scene)
    {
        return false;
    }

    auto node_ptr = inst.scene->getNode(nodeName);
    if (!node_ptr)
    {
        return false;
    }

    auto ov_it = inst.nodeLocalOverrides.find(node_ptr.get());
    if (ov_it == inst.nodeLocalOverrides.end())
    {
        return false;
    }

    inst.nodeLocalOverrides.erase(ov_it);
    return true;
}

void SceneManager::clearGLTFInstanceNodeOffsets(const std::string &instanceName)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end())
    {
        return;
    }

    it->second.nodeLocalOverrides.clear();
}

bool SceneManager::setGLTFInstanceAnimation(const std::string &instanceName, int animationIndex, bool resetTime)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    LoadedGLTF::AnimationState &anim_state = it->second.animation;
    it->second.scene->setActiveAnimation(anim_state, animationIndex, resetTime);
    return true;
}

bool SceneManager::setGLTFInstanceAnimation(const std::string &instanceName, const std::string &animationName, bool resetTime)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    LoadedGLTF::AnimationState &anim_state = it->second.animation;
    it->second.scene->setActiveAnimation(anim_state, animationName, resetTime);
    return true;
}

bool SceneManager::setGLTFInstanceAnimationLoop(const std::string &instanceName, bool loop)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    it->second.animation.animationLoop = loop;
    return true;
}

bool SceneManager::setGLTFInstanceAnimationSpeed(const std::string &instanceName, float speed)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    it->second.animation.playbackSpeed = speed;
    return true;
}

bool SceneManager::transitionGLTFInstanceAnimation(const std::string &instanceName,
                                                   int animationIndex,
                                                   float blendDurationSeconds,
                                                   bool resetTime)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    LoadedGLTF::AnimationState &anim_state = it->second.animation;
    it->second.scene->transitionAnimation(anim_state, animationIndex, blendDurationSeconds, resetTime);
    return true;
}

bool SceneManager::transitionGLTFInstanceAnimation(const std::string &instanceName,
                                                   const std::string &animationName,
                                                   float blendDurationSeconds,
                                                   bool resetTime)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    LoadedGLTF::AnimationState &anim_state = it->second.animation;
    it->second.scene->transitionAnimation(anim_state, animationName, blendDurationSeconds, resetTime);
    return true;
}
