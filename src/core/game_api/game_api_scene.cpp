#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "core/assets/manager.h"
#include "scene/vk_scene.h"
#include "physics/physics_body.h"
#include "physics/physics_world.h"

namespace GameAPI
{

glm::dvec3 Engine::get_world_origin() const
{
    if (!_engine || !_engine->_context)
    {
        return glm::dvec3(0.0);
    }
    return glm::dvec3(_engine->_context->origin_world);
}

glm::dvec3 Engine::get_physics_origin() const
{
    if (!_engine || !_engine->_context)
    {
        return glm::dvec3(0.0);
    }
    return glm::dvec3(_engine->_context->physics_origin_world);
}

glm::dvec3 Engine::get_physics_velocity_origin() const
{
    if (!_engine || !_engine->_context)
    {
        return glm::dvec3(0.0);
    }
    return _engine->_context->physics_velocity_origin_world;
}

void Engine::set_physics_origin_anchor(const glm::dvec3& anchor_world)
{
    if (!_engine || !_engine->_context)
    {
        return;
    }
    _engine->_context->set_physics_origin_anchor_world(WorldVec3(anchor_world));
}

void Engine::clear_physics_origin_anchor()
{
    if (!_engine || !_engine->_context)
    {
        return;
    }
    _engine->_context->clear_physics_origin_anchor_world();
}

bool Engine::maybe_rebase_physics_origin_to_body(uint32_t physics_body_value, double threshold_m, double snap_size_m)
{
    if (!_engine || !_engine->_context)
    {
        return false;
    }

    Physics::PhysicsWorld *physics = _engine->_context->physics;
    if (!physics)
    {
        return false;
    }

    Physics::BodyId body_id{physics_body_value};
    if (!physics->is_body_valid(body_id))
    {
        return false;
    }

    const glm::dvec3 p_local = physics->get_position(body_id);
    const double dist2 = glm::dot(p_local, p_local);
    const double threshold2 = (threshold_m <= 0.0) ? 0.0 : (threshold_m * threshold_m);
    if (dist2 <= threshold2)
    {
        return false;
    }

    const WorldVec3 origin_before = _engine->_context->physics_origin_world;
    const WorldVec3 anchor_world = origin_before + WorldVec3(p_local);
    const WorldVec3 new_origin =
        (snap_size_m > 0.0) ? snap_world(anchor_world, snap_size_m) : anchor_world;

    const glm::dvec3 delta_local = origin_before - new_origin;
    physics->shift_origin(delta_local);

    (void)_engine->_context->set_physics_origin_world(new_origin);
    return true;
}

bool Engine::maybe_rebase_physics_velocity_to_body(uint32_t physics_body_value, double threshold_mps)
{
    if (!_engine || !_engine->_context)
    {
        return false;
    }

    Physics::PhysicsWorld *physics = _engine->_context->physics;
    if (!physics)
    {
        return false;
    }

    Physics::BodyId body_id{physics_body_value};
    if (!physics->is_body_valid(body_id))
    {
        return false;
    }

    const glm::vec3 v_local_f = physics->get_linear_velocity(body_id);
    const double speed2 = static_cast<double>(glm::dot(v_local_f, v_local_f));
    const double threshold2 = (threshold_mps <= 0.0) ? 0.0 : (threshold_mps * threshold_mps);
    if (speed2 <= threshold2)
    {
        return false;
    }

    const glm::dvec3 delta_v_local(v_local_f);
    physics->shift_velocity_origin(delta_v_local);

    _engine->_context->physics_velocity_origin_world += delta_v_local;
    ++_engine->_context->physics_velocity_origin_revision;
    return true;
}

void Engine::set_floating_origin_anchor(const glm::dvec3& anchor_world)
{
    set_physics_origin_anchor(anchor_world);
}

void Engine::clear_floating_origin_anchor()
{
    clear_physics_origin_anchor();
}

// ----------------------------------------------------------------------------
// Objects / Instances
// ----------------------------------------------------------------------------

bool Engine::add_gltf_instance(const std::string& name,
                               const std::string& modelPath,
                               const Transform& transform,
                               bool preloadTextures)
{
    return _engine->addGLTFInstance(name, modelPath, transform.to_matrix(), preloadTextures);
}

bool Engine::add_gltf_instance(const std::string& name,
                               const std::string& modelPath,
                               const TransformD& transform,
                               bool preloadTextures)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    // Add the instance first (GPU resources), then apply the authoritative world transform in double.
    if (!_engine->addGLTFInstance(name, modelPath, glm::mat4(1.0f), preloadTextures))
    {
        return false;
    }

    return _engine->_sceneManager->setGLTFInstanceTRSWorld(name,
                                                          WorldVec3(transform.position),
                                                          transform.rotation,
                                                          transform.scale);
}

uint32_t Engine::add_gltf_instance_async(const std::string& name,
                                         const std::string& modelPath,
                                         const Transform& transform,
                                         bool preloadTextures)
{
    return _engine->loadGLTFAsync(name, modelPath, transform.to_matrix(), preloadTextures);
}

uint32_t Engine::add_gltf_instance_async(const std::string& name,
                                         const std::string& modelPath,
                                         const TransformD& transform,
                                         bool preloadTextures)
{
    return _engine->loadGLTFAsync(name,
                                 modelPath,
                                 WorldVec3(transform.position),
                                 transform.rotation,
                                 transform.scale,
                                 preloadTextures);
}

bool Engine::remove_gltf_instance(const std::string& name)
{
    return _engine->_sceneManager ? _engine->_sceneManager->removeGLTFInstance(name) : false;
}

bool Engine::get_gltf_instance_transform(const std::string& name, Transform& out) const
{
    if (!_engine->_sceneManager) return false;

    glm::mat4 m;
    if (_engine->_sceneManager->getGLTFInstanceTransform(name, m))
    {
        out = Transform::from_matrix(m);
        return true;
    }
    return false;
}

bool Engine::get_gltf_instance_transform(const std::string& name, TransformD& out) const
{
    if (!_engine->_sceneManager) return false;

    WorldVec3 t{};
    glm::quat r{};
    glm::vec3 s{};
    if (_engine->_sceneManager->getGLTFInstanceTRSWorld(name, t, r, s))
    {
        out.position = glm::dvec3(t);
        out.rotation = r;
        out.scale = s;
        return true;
    }
    return false;
}

bool Engine::set_gltf_instance_transform(const std::string& name, const Transform& transform)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceTransform(name, transform.to_matrix())
           : false;
}

bool Engine::set_gltf_instance_transform(const std::string& name, const TransformD& transform)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceTRSWorld(name,
                                                            WorldVec3(transform.position),
                                                            transform.rotation,
                                                            transform.scale)
           : false;
}

bool Engine::add_primitive_instance(const std::string& name,
                                    PrimitiveType type,
                                    const Transform& transform)
{
    AssetManager::MeshGeometryDesc::Type geomType;
    switch (type)
    {
    case PrimitiveType::Cube:    geomType = AssetManager::MeshGeometryDesc::Type::Cube; break;
    case PrimitiveType::Sphere:  geomType = AssetManager::MeshGeometryDesc::Type::Sphere; break;
    case PrimitiveType::Plane:   geomType = AssetManager::MeshGeometryDesc::Type::Plane; break;
    case PrimitiveType::Capsule: geomType = AssetManager::MeshGeometryDesc::Type::Capsule; break;
    default: return false;
    }

    return _engine->addPrimitiveInstance(name, geomType, transform.to_matrix());
}

bool Engine::add_primitive_instance(const std::string& name,
                                    PrimitiveType type,
                                    const TransformD& transform)
{
    AssetManager::MeshGeometryDesc::Type geomType;
    switch (type)
    {
    case PrimitiveType::Cube:    geomType = AssetManager::MeshGeometryDesc::Type::Cube; break;
    case PrimitiveType::Sphere:  geomType = AssetManager::MeshGeometryDesc::Type::Sphere; break;
    case PrimitiveType::Plane:   geomType = AssetManager::MeshGeometryDesc::Type::Plane; break;
    case PrimitiveType::Capsule: geomType = AssetManager::MeshGeometryDesc::Type::Capsule; break;
    default: return false;
    }

    if (!_engine->addPrimitiveInstance(name, geomType, glm::mat4(1.0f)))
    {
        return false;
    }
    return _engine->_sceneManager
           ? _engine->_sceneManager->setMeshInstanceTRSWorld(name,
                                                            WorldVec3(transform.position),
                                                            transform.rotation,
                                                            transform.scale)
           : false;
}

bool Engine::add_textured_primitive(const std::string& name,
                                    PrimitiveType type,
                                    const PrimitiveMaterial& material,
                                    const Transform& transform)
{
    AssetManager::MeshGeometryDesc::Type geomType;
    switch (type)
    {
    case PrimitiveType::Cube:    geomType = AssetManager::MeshGeometryDesc::Type::Cube; break;
    case PrimitiveType::Sphere:  geomType = AssetManager::MeshGeometryDesc::Type::Sphere; break;
    case PrimitiveType::Plane:   geomType = AssetManager::MeshGeometryDesc::Type::Plane; break;
    case PrimitiveType::Capsule: geomType = AssetManager::MeshGeometryDesc::Type::Capsule; break;
    default: return false;
    }

    AssetManager::MeshMaterialDesc matDesc;
    matDesc.kind = AssetManager::MeshMaterialDesc::Kind::Textured;
    matDesc.options.albedoPath = material.albedoPath;
    matDesc.options.metalRoughPath = material.metalRoughPath;
    matDesc.options.normalPath = material.normalPath;
    matDesc.options.occlusionPath = material.occlusionPath;
    matDesc.options.emissivePath = material.emissivePath;
    matDesc.options.constants.colorFactors = material.colorFactor;
    matDesc.options.constants.metal_rough_factors = glm::vec4(material.metallic, material.roughness, 0.0f, 0.0f);

    return _engine->addPrimitiveInstance(name, geomType, transform.to_matrix(), matDesc);
}

bool Engine::add_textured_primitive(const std::string& name,
                                    PrimitiveType type,
                                    const PrimitiveMaterial& material,
                                    const TransformD& transform)
{
    AssetManager::MeshGeometryDesc::Type geomType;
    switch (type)
    {
    case PrimitiveType::Cube:    geomType = AssetManager::MeshGeometryDesc::Type::Cube; break;
    case PrimitiveType::Sphere:  geomType = AssetManager::MeshGeometryDesc::Type::Sphere; break;
    case PrimitiveType::Plane:   geomType = AssetManager::MeshGeometryDesc::Type::Plane; break;
    case PrimitiveType::Capsule: geomType = AssetManager::MeshGeometryDesc::Type::Capsule; break;
    default: return false;
    }

    AssetManager::MeshMaterialDesc matDesc;
    matDesc.kind = AssetManager::MeshMaterialDesc::Kind::Textured;
    matDesc.options.albedoPath = material.albedoPath;
    matDesc.options.metalRoughPath = material.metalRoughPath;
    matDesc.options.normalPath = material.normalPath;
    matDesc.options.occlusionPath = material.occlusionPath;
    matDesc.options.emissivePath = material.emissivePath;
    matDesc.options.constants.colorFactors = material.colorFactor;
    matDesc.options.constants.metal_rough_factors = glm::vec4(material.metallic, material.roughness, 0.0f, 0.0f);

    if (!_engine->addPrimitiveInstance(name, geomType, glm::mat4(1.0f), matDesc))
    {
        return false;
    }
    return _engine->_sceneManager
           ? _engine->_sceneManager->setMeshInstanceTRSWorld(name,
                                                            WorldVec3(transform.position),
                                                            transform.rotation,
                                                            transform.scale)
           : false;
}

bool Engine::remove_mesh_instance(const std::string& name)
{
    return _engine->_sceneManager ? _engine->_sceneManager->removeMeshInstance(name) : false;
}

bool Engine::get_mesh_instance_transform(const std::string& name, Transform& out) const
{
    if (!_engine->_sceneManager) return false;

    glm::mat4 m;
    if (_engine->_sceneManager->getMeshInstanceTransform(name, m))
    {
        out = Transform::from_matrix(m);
        return true;
    }
    return false;
}

bool Engine::get_mesh_instance_transform(const std::string& name, TransformD& out) const
{
    if (!_engine->_sceneManager) return false;

    WorldVec3 t{};
    glm::quat r{};
    glm::vec3 s{};
    if (_engine->_sceneManager->getMeshInstanceTRSWorld(name, t, r, s))
    {
        out.position = glm::dvec3(t);
        out.rotation = r;
        out.scale = s;
        return true;
    }
    return false;
}

bool Engine::set_mesh_instance_transform(const std::string& name, const Transform& transform)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setMeshInstanceTransform(name, transform.to_matrix())
           : false;
}

bool Engine::set_mesh_instance_transform(const std::string& name, const TransformD& transform)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setMeshInstanceTRSWorld(name,
                                                            WorldVec3(transform.position),
                                                            transform.rotation,
                                                            transform.scale)
           : false;
}

void Engine::preload_instance_textures(const std::string& name)
{
    _engine->preloadInstanceTextures(name);
}

void Engine::clear_all_instances()
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->clearGLTFInstances();
        _engine->_sceneManager->clearMeshInstances();
    }
}

// ----------------------------------------------------------------------------
// Animation
// ----------------------------------------------------------------------------

bool Engine::set_instance_animation(const std::string& instanceName, int animationIndex, bool resetTime)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceAnimation(instanceName, animationIndex, resetTime)
           : false;
}

bool Engine::set_instance_animation(const std::string& instanceName, const std::string& animationName, bool resetTime)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceAnimation(instanceName, animationName, resetTime)
           : false;
}

bool Engine::set_instance_animation_loop(const std::string& instanceName, bool loop)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceAnimationLoop(instanceName, loop)
           : false;
}

bool Engine::set_instance_animation_speed(const std::string& instanceName, float speed)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceAnimationSpeed(instanceName, speed)
           : false;
}

bool Engine::transition_instance_animation(const std::string& instanceName, int animationIndex, float blendDurationSeconds, bool resetTime)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->transitionGLTFInstanceAnimation(instanceName, animationIndex, blendDurationSeconds, resetTime)
           : false;
}

bool Engine::transition_instance_animation(const std::string& instanceName, const std::string& animationName, float blendDurationSeconds, bool resetTime)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->transitionGLTFInstanceAnimation(instanceName, animationName, blendDurationSeconds, resetTime)
           : false;
}

bool Engine::set_instance_node_offset(const std::string& instanceName, const std::string& nodeName, const glm::mat4& offset)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceNodeOffset(instanceName, nodeName, offset)
           : false;
}

bool Engine::clear_instance_node_offset(const std::string& instanceName, const std::string& nodeName)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->clearGLTFInstanceNodeOffset(instanceName, nodeName)
           : false;
}

void Engine::clear_all_instance_node_offsets(const std::string& instanceName)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->clearGLTFInstanceNodeOffsets(instanceName);
    }
}

} // namespace GameAPI
