#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "core/assets/manager.h"
#include "scene/vk_scene.h"

// Ensure GameAPI::DecalShape and ::DecalShape stay in sync.
static_assert(static_cast<uint8_t>(GameAPI::DecalShape::Box) == static_cast<uint8_t>(::DecalShape::Box));
static_assert(static_cast<uint8_t>(GameAPI::DecalShape::Sphere) == static_cast<uint8_t>(::DecalShape::Sphere));

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

bool Engine::set_decal(const std::string& name, const Decal& decal)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    SceneManager::DecalInstance inst{};
    inst.shape = static_cast<::DecalShape>(decal.shape);
    inst.center_world = WorldVec3(decal.position);
    inst.rotation = decal.rotation;
    inst.half_extents = decal.halfExtents;
    inst.albedoHandle = static_cast<uint32_t>(decal.albedoTexture);
    inst.normalHandle = static_cast<uint32_t>(decal.normalTexture);
    inst.tint = decal.tint;
    inst.opacity = decal.opacity;
    inst.normalStrength = decal.normalStrength;
    inst.sort_order = decal.sortOrder;

    return _engine->_sceneManager->setDecal(name, inst);
}

bool Engine::get_decal(const std::string& name, Decal& out) const
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    SceneManager::DecalInstance inst{};
    if (!_engine->_sceneManager->getDecal(name, inst))
    {
        return false;
    }

    out.shape = static_cast<GameAPI::DecalShape>(inst.shape);
    out.position = glm::dvec3(inst.center_world);
    out.rotation = inst.rotation;
    out.halfExtents = inst.half_extents;
    out.albedoTexture = static_cast<TextureHandle>(inst.albedoHandle);
    out.normalTexture = static_cast<TextureHandle>(inst.normalHandle);
    out.tint = inst.tint;
    out.opacity = inst.opacity;
    out.normalStrength = inst.normalStrength;
    out.sortOrder = inst.sort_order;
    return true;
}

bool Engine::remove_decal(const std::string& name)
{
    return _engine && _engine->_sceneManager ? _engine->_sceneManager->removeDecal(name) : false;
}

size_t Engine::get_decal_count() const
{
    return _engine && _engine->_sceneManager ? _engine->_sceneManager->getDecalCount() : 0;
}

void Engine::clear_decals()
{
    if (_engine && _engine->_sceneManager)
    {
        _engine->_sceneManager->clearDecals();
    }
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
