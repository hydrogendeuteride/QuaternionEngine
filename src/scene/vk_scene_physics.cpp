#include "vk_scene.h"

#include "core/context.h"
#include "core/util/logger.h"
#include "physics/body_settings.h"
#include "physics/collider_asset.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"
#include "vk_scene_internal.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

namespace
{
    bool approx_equal_rel(float a, float b, float eps)
    {
        const float aa = std::abs(a);
        const float bb = std::abs(b);
        const float denom = std::max(1.0f, std::max(aa, bb));
        return std::abs(a - b) <= eps * denom;
    }

    std::optional<Physics::PrimitiveShapeVariant> scale_primitive_with_vec3(
        const Physics::PrimitiveShapeVariant &shape,
        const glm::vec3 &raw_scale,
        std::string_view scene_name,
        std::string_view owner_name)
    {
        const glm::vec3 scale = glm::abs(raw_scale);
        auto valid_scale = [](float v) { return std::isfinite(v) && v > 0.0f; };
        if (!valid_scale(scale.x) || !valid_scale(scale.y) || !valid_scale(scale.z))
        {
            Logger::warn("[SceneManager] dynamic root collider '{}': owner '{}' has invalid scale ({}, {}, {}); skipping child",
                         scene_name, owner_name, scale.x, scale.y, scale.z);
            return {};
        }

        constexpr float kUniformEps = 1.0e-3f;

        return std::visit([&](const auto &prim) -> std::optional<Physics::PrimitiveShapeVariant>
        {
            using T = std::decay_t<decltype(prim)>;

            if constexpr (std::is_same_v<T, Physics::BoxShape>)
            {
                return Physics::BoxShape{prim.half_extents * scale};
            }
            else if constexpr (std::is_same_v<T, Physics::SphereShape>)
            {
                if (!approx_equal_rel(scale.x, scale.y, kUniformEps) || !approx_equal_rel(scale.x, scale.z, kUniformEps))
                {
                    Logger::warn("[SceneManager] dynamic root collider '{}': owner '{}' sphere child has non-uniform scale ({}, {}, {}); using max radius",
                                 scene_name, owner_name, scale.x, scale.y, scale.z);
                }
                return Physics::SphereShape{prim.radius * std::max({scale.x, scale.y, scale.z})};
            }
            else if constexpr (std::is_same_v<T, Physics::CapsuleShape>)
            {
                if (!approx_equal_rel(scale.x, scale.z, kUniformEps))
                {
                    Logger::warn("[SceneManager] dynamic root collider '{}': owner '{}' capsule child has non-uniform X/Z scale ({}, {}); using max radius",
                                 scene_name, owner_name, scale.x, scale.z);
                }
                return Physics::CapsuleShape{
                    prim.radius * std::max(scale.x, scale.z),
                    prim.half_height * scale.y};
            }
            else if constexpr (std::is_same_v<T, Physics::CylinderShape>)
            {
                if (!approx_equal_rel(scale.x, scale.z, kUniformEps))
                {
                    Logger::warn("[SceneManager] dynamic root collider '{}': owner '{}' cylinder child has non-uniform X/Z scale ({}, {}); using max radius",
                                 scene_name, owner_name, scale.x, scale.z);
                }
                return Physics::CylinderShape{
                    prim.radius * std::max(scale.x, scale.z),
                    prim.half_height * scale.y};
            }
            else if constexpr (std::is_same_v<T, Physics::TaperedCylinderShape>)
            {
                return Physics::TaperedCylinderShape{
                    prim.half_height * scale.y,
                    prim.top_radius * scale.x,
                    prim.bottom_radius * scale.z};
            }
            else if constexpr (std::is_same_v<T, Physics::PlaneShape>)
            {
                return Physics::PlaneShape{prim.normal, prim.offset * scale.y};
            }
            else
            {
                return {};
            }
        }, shape);
    }

    Physics::CompoundShape aggregate_compounds_for_dynamic_root(const SceneManager::GLTFInstance &inst)
    {
        Physics::CompoundShape out{};
        if (!inst.scene)
        {
            return out;
        }

        const LoadedGLTF &scene = *inst.scene;
        const std::string_view scene_name =
            scene.debugName.empty() ? std::string_view("<unnamed>") : std::string_view(scene.debugName);

        for (const auto &[owner_name, compound] : scene.collider_compounds)
        {
            auto node_it = scene.nodes.find(owner_name);
            if (node_it == scene.nodes.end() || !node_it->second)
            {
                Logger::warn("[SceneManager] dynamic root collider '{}': owner node '{}' missing; skipping compound",
                             scene_name, owner_name);
                continue;
            }

            const glm::mat4 owner_world = inst.nodeLocalOverrides.empty()
                                              ? node_it->second->worldTransform
                                              : SceneInternal::build_node_world_with_overrides(node_it->second.get(), inst.nodeLocalOverrides);

            for (const Physics::CompoundShapeChild &child : compound.children)
            {
                const glm::mat4 child_local = make_trs_matrix(child.position, child.rotation, glm::vec3(1.0f));
                const glm::mat4 combined = owner_world * child_local;

                glm::vec3 t{0.0f};
                glm::quat r{1.0f, 0.0f, 0.0f, 0.0f};
                glm::vec3 s{1.0f};
                decompose_trs_matrix(combined, t, r, s);
                r = glm::normalize(r);

                std::optional<Physics::PrimitiveShapeVariant> scaled_shape =
                    scale_primitive_with_vec3(child.shape, s, scene_name, owner_name);
                if (!scaled_shape.has_value())
                {
                    continue;
                }

                float effective_mass = child.mass;
                if (!child.name.empty())
                {
                    auto it = inst.dynamicRootChildMassOverrides.find(child.name);
                    if (it != inst.dynamicRootChildMassOverrides.end())
                    {
                        effective_mass = it->second;
                    }
                }

                out.add_child(*scaled_shape, t, r, child.user_data, effective_mass, child.name);
            }
        }

        return out;
    }
}

size_t SceneManager::enableColliderSync(const std::string &instanceName,
                                        Physics::PhysicsWorld *world,
                                        uint32_t layer,
                                        uint64_t user_data)
{
    if (!world)
    {
        Logger::error("[SceneManager] enableColliderSync: null physics world");
        return 0;
    }

    if (_colliderSyncEntries.contains(instanceName))
    {
        return 0;
    }

    auto instIt = dynamicGLTFInstances.find(instanceName);
    if (instIt == dynamicGLTFInstances.end())
    {
        Logger::warn("[SceneManager] enableColliderSync: instance '{}' not found", instanceName);
        return 0;
    }

    const GLTFInstance &inst = instIt->second;
    if (!inst.scene)
    {
        Logger::warn("[SceneManager] enableColliderSync: instance '{}' has no scene", instanceName);
        return 0;
    }

    const auto &compounds = inst.scene->collider_compounds;
    const auto &mesh_instances = inst.scene->collider_mesh_instances;
    if (compounds.empty() && mesh_instances.empty())
    {
        Logger::warn("[SceneManager] enableColliderSync: instance '{}' has no colliders", instanceName);
        return 0;
    }

    const glm::vec3 &s = inst.scale;
    const float uniform_scale = s.x;
    constexpr float kScaleEps = 1.0e-3f;
    if (std::abs(s.y - uniform_scale) > kScaleEps || std::abs(s.z - uniform_scale) > kScaleEps)
    {
        Logger::warn("[SceneManager] enableColliderSync: instance '{}' has non-uniform scale ({}, {}, {}); "
                     "using x-component for collider scaling",
                     instanceName, s.x, s.y, s.z);
    }

    ColliderSyncEntry entry{};
    entry.world = world;
    entry.layer = layer;
    entry.user_data = user_data;

    const WorldVec3 physics_origin_world =
        (_context && _context->physics_context) ? _context->physics_context->origin_world() : WorldVec3{0.0, 0.0, 0.0};

    size_t created = 0;
    for (const auto &[nodeName, compound] : compounds)
    {
        glm::mat4 nodeWorld(1.0f);
        if (!getGLTFInstanceNodeWorldTransformLocal(instanceName, nodeName, physics_origin_world, nodeWorld))
        {
            continue;
        }

        glm::vec3 t{0.0f};
        glm::quat r{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 nodeScale{1.0f};
        decompose_trs_matrix(nodeWorld, t, r, nodeScale);

        Physics::CompoundShape scaledCompound = Physics::scale_compound_uniform(compound, uniform_scale);
        if (scaledCompound.children.empty())
        {
            continue;
        }

        Physics::BodySettings settings{};
        settings.shape = Physics::CollisionShape::Compound(std::move(scaledCompound));
        settings.position = glm::dvec3(t);
        settings.rotation = glm::normalize(r);
        settings.motion_type = Physics::MotionType::Kinematic;
        settings.layer = layer;
        settings.user_data = user_data;

        const Physics::BodyId bodyId = world->create_body(settings);
        if (bodyId.is_valid())
        {
            entry.node_bodies[nodeName] = bodyId;
            ++created;
        }
    }

    for (const auto &[ownerNodeName, instances] : mesh_instances)
    {
        if (instances.empty())
        {
            continue;
        }

        glm::mat4 ownerWorld(1.0f);
        if (!getGLTFInstanceNodeWorldTransformLocal(instanceName, ownerNodeName, physics_origin_world, ownerWorld))
        {
            continue;
        }

        for (const Physics::ColliderMeshInstance &mesh_inst : instances)
        {
            if (!mesh_inst.mesh || mesh_inst.mesh->triangles.empty())
            {
                continue;
            }

            const glm::mat4 colliderWorld = ownerWorld * mesh_inst.relative_transform;

            glm::vec3 t{0.0f};
            glm::quat r{1.0f, 0.0f, 0.0f, 0.0f};
            glm::vec3 mesh_scale{1.0f};
            decompose_trs_matrix(colliderWorld, t, r, mesh_scale);

            Physics::BodySettings settings{};
            settings.shape = Physics::CollisionShape::TriangleMesh(mesh_inst.mesh, mesh_scale);
            settings.position = glm::dvec3(t);
            settings.rotation = glm::normalize(r);
            settings.motion_type = Physics::MotionType::Static;
            settings.layer = layer;
            settings.user_data = user_data;

            const Physics::BodyId bodyId = world->create_body(settings);
            if (bodyId.is_valid())
            {
                entry.mesh_bodies.push_back(bodyId);
                ++created;
            }
        }
    }

    if (created > 0)
    {
        _colliderSyncEntries[instanceName] = std::move(entry);
        Logger::info("[SceneManager] enableColliderSync: created {} bodies for '{}'", created, instanceName);
    }

    return created;
}

bool SceneManager::disableColliderSync(const std::string &instanceName)
{
    auto it = _colliderSyncEntries.find(instanceName);
    if (it == _colliderSyncEntries.end())
    {
        return false;
    }

    destroyColliderSyncEntry(it->second);
    _colliderSyncEntries.erase(it);
    return true;
}

bool SceneManager::isColliderSyncEnabled(const std::string &instanceName) const
{
    return _colliderSyncEntries.contains(instanceName);
}

void SceneManager::syncColliders()
{
    for (auto &[instanceName, entry] : _colliderSyncEntries)
    {
        if (!entry.world)
        {
            continue;
        }

        const WorldVec3 physics_origin_world =
            (_context && _context->physics_context) ? _context->physics_context->origin_world() : WorldVec3{0.0, 0.0, 0.0};

        auto instIt = dynamicGLTFInstances.find(instanceName);
        if (instIt == dynamicGLTFInstances.end())
        {
            continue;
        }

        for (auto &[nodeName, bodyId] : entry.node_bodies)
        {
            if (!entry.world->is_body_valid(bodyId))
            {
                continue;
            }

            const glm::mat4 nodeWorld =
                getGLTFInstanceNodeWorldTransformLocal(instanceName, nodeName, physics_origin_world);

            glm::vec3 t{0.0f};
            glm::quat r{1.0f, 0.0f, 0.0f, 0.0f};
            glm::vec3 s{1.0f};
            decompose_trs_matrix(nodeWorld, t, r, s);

            entry.world->set_transform(bodyId, glm::dvec3(t), glm::normalize(r));
        }
    }
}

std::vector<Physics::BodyId> SceneManager::getColliderSyncBodies(const std::string &instanceName) const
{
    std::vector<Physics::BodyId> result;
    auto it = _colliderSyncEntries.find(instanceName);
    if (it != _colliderSyncEntries.end())
    {
        result.reserve(it->second.node_bodies.size() + it->second.mesh_bodies.size());
        for (const auto &[nodeName, bodyId] : it->second.node_bodies)
        {
            result.push_back(bodyId);
        }
        for (const Physics::BodyId bodyId : it->second.mesh_bodies)
        {
            result.push_back(bodyId);
        }
    }
    return result;
}

void SceneManager::destroyColliderSyncEntry(ColliderSyncEntry &entry)
{
    if (!entry.world)
    {
        return;
    }

    for (auto &[nodeName, bodyId] : entry.node_bodies)
    {
        if (entry.world->is_body_valid(bodyId))
        {
            entry.world->destroy_body(bodyId);
        }
    }
    entry.node_bodies.clear();

    for (const Physics::BodyId bodyId : entry.mesh_bodies)
    {
        if (entry.world->is_body_valid(bodyId))
        {
            entry.world->destroy_body(bodyId);
        }
    }
    entry.mesh_bodies.clear();
}

Physics::BodyId SceneManager::createDynamicRootColliderBodyForInstance(const std::string &instanceName,
                                                                       const GLTFInstance &inst,
                                                                       Physics::PhysicsWorld *world,
                                                                       uint32_t layer,
                                                                       uint64_t user_data,
                                                                       std::optional<float> mass_override) const
{
    if (!world || !inst.scene)
    {
        return {};
    }

    if (inst.scene->collider_compounds.empty())
    {
        Logger::warn("[SceneManager] enableDynamicRootColliderBody: instance '{}' has no primitive colliders", instanceName);
        return {};
    }

    if (!inst.scene->collider_mesh_instances.empty())
    {
        Logger::warn("[SceneManager] enableDynamicRootColliderBody: instance '{}' has mesh colliders; ignoring them for dynamic root body",
                     instanceName);
    }

    Physics::CompoundShape aggregate = aggregate_compounds_for_dynamic_root(inst);
    if (aggregate.children.empty())
    {
        Logger::warn("[SceneManager] enableDynamicRootColliderBody: instance '{}' has no usable aggregate collider shape",
                     instanceName);
        return {};
    }

    const glm::vec3 &scale = inst.scale;
    const float uniform_scale = scale.x;
    constexpr float kScaleEps = 1.0e-3f;
    if (std::abs(scale.y - uniform_scale) > kScaleEps || std::abs(scale.z - uniform_scale) > kScaleEps)
    {
        Logger::warn("[SceneManager] enableDynamicRootColliderBody: instance '{}' has non-uniform scale ({}, {}, {}); using x-component for collider scaling",
                     instanceName, scale.x, scale.y, scale.z);
    }

    Physics::CompoundShape scaled = Physics::scale_compound_uniform(aggregate, uniform_scale);
    if (scaled.children.empty())
    {
        Logger::warn("[SceneManager] enableDynamicRootColliderBody: instance '{}' produced an empty scaled collider shape",
                     instanceName);
        return {};
    }

    const WorldVec3 physics_origin_world =
        (_context && _context->physics_context) ? _context->physics_context->origin_world() : WorldVec3{0.0, 0.0, 0.0};

    Physics::BodySettings settings{};
    settings.shape = Physics::CollisionShape::Compound(std::move(scaled));
    settings.position = world_to_local_d(inst.translation_world, physics_origin_world);
    settings.rotation = glm::normalize(inst.rotation);
    settings.motion_type = Physics::MotionType::Dynamic;
    settings.layer = layer;
    settings.user_data = user_data;
    if (mass_override.has_value())
    {
        settings.mass = std::max(*mass_override, 0.001f);
        settings.has_explicit_mass = true;
    }

    return world->create_body(settings);
}

Physics::BodyId SceneManager::enableDynamicRootColliderBody(const std::string &instanceName,
                                                            Physics::PhysicsWorld *world,
                                                            uint32_t layer,
                                                            uint64_t user_data,
                                                            std::optional<float> mass_override)
{
    if (!world)
    {
        Logger::error("[SceneManager] enableDynamicRootColliderBody: null physics world");
        return {};
    }

    if (_dynamicRootColliderBodies.contains(instanceName))
    {
        return _dynamicRootColliderBodies[instanceName].body;
    }

    if (_colliderSyncEntries.contains(instanceName))
    {
        Logger::warn("[SceneManager] enableDynamicRootColliderBody: instance '{}' already uses node collider sync",
                     instanceName);
        return {};
    }

    auto inst_it = dynamicGLTFInstances.find(instanceName);
    if (inst_it == dynamicGLTFInstances.end())
    {
        Logger::warn("[SceneManager] enableDynamicRootColliderBody: instance '{}' not found", instanceName);
        return {};
    }

    GLTFInstance &inst = inst_it->second;
    if (!inst.scene)
    {
        Logger::warn("[SceneManager] enableDynamicRootColliderBody: instance '{}' has no scene", instanceName);
        return {};
    }

    const Physics::BodyId body = createDynamicRootColliderBodyForInstance(instanceName,
                                                                          inst,
                                                                          world,
                                                                          layer,
                                                                          user_data,
                                                                          mass_override);
    if (!body.is_valid())
    {
        Logger::warn("[SceneManager] enableDynamicRootColliderBody: failed to create body for '{}'", instanceName);
        return {};
    }

    DynamicRootColliderBodyEntry entry{};
    entry.world = world;
    entry.body = body;
    entry.scale = inst.scale;
    entry.layer = layer;
    entry.user_data = user_data;
    entry.mass_override = mass_override.has_value() ? std::optional<float>(std::max(*mass_override, 0.001f))
                                                    : std::nullopt;
    _dynamicRootColliderBodies[instanceName] = entry;

    Logger::info("[SceneManager] enableDynamicRootColliderBody: created dynamic root body {} for '{}'",
                 body.value, instanceName);
    return body;
}

bool SceneManager::disableDynamicRootColliderBody(const std::string &instanceName)
{
    auto it = _dynamicRootColliderBodies.find(instanceName);
    if (it == _dynamicRootColliderBodies.end())
    {
        return false;
    }

    destroyDynamicRootColliderBodyEntry(it->second);
    _dynamicRootColliderBodies.erase(it);
    return true;
}

bool SceneManager::isDynamicRootColliderBodyEnabled(const std::string &instanceName) const
{
    return _dynamicRootColliderBodies.contains(instanceName);
}

Physics::BodyId SceneManager::getDynamicRootColliderBody(const std::string &instanceName) const
{
    auto it = _dynamicRootColliderBodies.find(instanceName);
    return it != _dynamicRootColliderBodies.end() ? it->second.body : Physics::BodyId{};
}

bool SceneManager::setDynamicRootColliderMass(const std::string &instanceName, float mass)
{
    if (!std::isfinite(mass) || mass <= 0.0f)
    {
        Logger::warn("[SceneManager] setDynamicRootColliderMass: invalid mass {} for '{}'",
                     mass, instanceName);
        return false;
    }

    auto entry_it = _dynamicRootColliderBodies.find(instanceName);
    if (entry_it == _dynamicRootColliderBodies.end())
    {
        return false;
    }

    entry_it->second.mass_override = mass;
    return rebuildDynamicRootColliderBody(instanceName, entry_it->second);
}

bool SceneManager::clearDynamicRootColliderMassOverride(const std::string &instanceName)
{
    auto entry_it = _dynamicRootColliderBodies.find(instanceName);
    if (entry_it == _dynamicRootColliderBodies.end())
    {
        return false;
    }

    entry_it->second.mass_override.reset();
    return rebuildDynamicRootColliderBody(instanceName, entry_it->second);
}

bool SceneManager::setDynamicRootColliderChildMassOverride(const std::string &instanceName,
                                                           const std::string &colliderName,
                                                           float mass)
{
    if (!std::isfinite(mass) || mass <= 0.0f)
    {
        Logger::warn("[SceneManager] setDynamicRootColliderChildMassOverride: invalid mass {} for '{}'",
                     mass, colliderName);
        return false;
    }

    auto inst_it = dynamicGLTFInstances.find(instanceName);
    if (inst_it == dynamicGLTFInstances.end())
    {
        return false;
    }

    inst_it->second.dynamicRootChildMassOverrides[colliderName] = mass;

    auto entry_it = _dynamicRootColliderBodies.find(instanceName);
    return entry_it == _dynamicRootColliderBodies.end() || rebuildDynamicRootColliderBody(instanceName, entry_it->second);
}

bool SceneManager::setDynamicRootColliderChildMassOverrides(const std::string &instanceName,
                                                            const std::unordered_map<std::string, float> &overrides)
{
    auto inst_it = dynamicGLTFInstances.find(instanceName);
    if (inst_it == dynamicGLTFInstances.end())
    {
        return false;
    }

    for (const auto &[name, mass] : overrides)
    {
        if (!std::isfinite(mass) || mass <= 0.0f)
        {
            Logger::warn("[SceneManager] setDynamicRootColliderChildMassOverrides: invalid mass {} for '{}'",
                         mass, name);
            return false;
        }
    }

    inst_it->second.dynamicRootChildMassOverrides = overrides;

    auto entry_it = _dynamicRootColliderBodies.find(instanceName);
    return entry_it == _dynamicRootColliderBodies.end() || rebuildDynamicRootColliderBody(instanceName, entry_it->second);
}

bool SceneManager::clearDynamicRootColliderChildMassOverride(const std::string &instanceName,
                                                             const std::string &colliderName)
{
    auto inst_it = dynamicGLTFInstances.find(instanceName);
    if (inst_it == dynamicGLTFInstances.end())
    {
        return false;
    }

    inst_it->second.dynamicRootChildMassOverrides.erase(colliderName);

    auto entry_it = _dynamicRootColliderBodies.find(instanceName);
    return entry_it == _dynamicRootColliderBodies.end() || rebuildDynamicRootColliderBody(instanceName, entry_it->second);
}

bool SceneManager::clearDynamicRootColliderChildMassOverrides(const std::string &instanceName)
{
    auto inst_it = dynamicGLTFInstances.find(instanceName);
    if (inst_it == dynamicGLTFInstances.end())
    {
        return false;
    }

    inst_it->second.dynamicRootChildMassOverrides.clear();

    auto entry_it = _dynamicRootColliderBodies.find(instanceName);
    return entry_it == _dynamicRootColliderBodies.end() || rebuildDynamicRootColliderBody(instanceName, entry_it->second);
}

void SceneManager::destroyDynamicRootColliderBodyEntry(DynamicRootColliderBodyEntry &entry)
{
    if (!entry.world)
    {
        return;
    }

    if (entry.body.is_valid() && entry.world->is_body_valid(entry.body))
    {
        entry.world->destroy_body(entry.body);
    }
    entry.body = {};
}

bool SceneManager::rebuildDynamicRootColliderBody(const std::string &instanceName,
                                                  DynamicRootColliderBodyEntry &entry,
                                                  bool preserve_instance_transform)
{
    auto inst_it = dynamicGLTFInstances.find(instanceName);
    if (inst_it == dynamicGLTFInstances.end() || !entry.world)
    {
        return false;
    }

    GLTFInstance &inst = inst_it->second;
    glm::vec3 linear_velocity{0.0f};
    glm::vec3 angular_velocity{0.0f};
    bool was_active = false;

    if (entry.body.is_valid() && entry.world->is_body_valid(entry.body))
    {
        linear_velocity = entry.world->get_linear_velocity(entry.body);
        angular_velocity = entry.world->get_angular_velocity(entry.body);
        was_active = entry.world->is_active(entry.body);

        if (!preserve_instance_transform)
        {
            const WorldVec3 physics_origin_world =
                (_context && _context->physics_context) ? _context->physics_context->origin_world() : WorldVec3{0.0, 0.0, 0.0};

            inst.translation_world = local_to_world_d(entry.world->get_position(entry.body), physics_origin_world);
            inst.rotation = glm::normalize(entry.world->get_rotation(entry.body));
        }

        entry.world->destroy_body(entry.body);
        entry.body = {};
    }

    const Physics::BodyId new_body = createDynamicRootColliderBodyForInstance(instanceName,
                                                                              inst,
                                                                              entry.world,
                                                                              entry.layer,
                                                                              entry.user_data,
                                                                              entry.mass_override);
    if (!new_body.is_valid())
    {
        Logger::warn("[SceneManager] rebuildDynamicRootColliderBody: failed to recreate body for '{}'",
                     instanceName);
        return false;
    }

    entry.body = new_body;
    entry.scale = inst.scale;
    entry.world->set_linear_velocity(new_body, linear_velocity);
    entry.world->set_angular_velocity(new_body, angular_velocity);
    if (was_active)
    {
        entry.world->activate(new_body);
    }
    else
    {
        entry.world->deactivate(new_body);
    }

    return true;
}

void SceneManager::syncDynamicRootColliderBodies()
{
    static int debug_dynamic_root_logs = 0;
    for (auto &[instanceName, entry] : _dynamicRootColliderBodies)
    {
        if (!entry.world || !entry.body.is_valid() || !entry.world->is_body_valid(entry.body))
        {
            continue;
        }

        auto inst_it = dynamicGLTFInstances.find(instanceName);
        if (inst_it == dynamicGLTFInstances.end())
        {
            continue;
        }

        const WorldVec3 physics_origin_world =
            (_context && _context->physics_context) ? _context->physics_context->origin_world() : WorldVec3{0.0, 0.0, 0.0};

        inst_it->second.translation_world = local_to_world_d(entry.world->get_position(entry.body), physics_origin_world);
        inst_it->second.rotation = glm::normalize(entry.world->get_rotation(entry.body));
        inst_it->second.scale = entry.scale;

        if (debug_dynamic_root_logs < 12)
        {
            const glm::dvec3 pos_local = entry.world->get_position(entry.body);
            Logger::info("[SceneManager] syncDynamicRootColliderBodies '{}' body={} localPos=({}, {}, {}) worldPos=({}, {}, {}) scale=({}, {}, {})",
                         instanceName,
                         entry.body.value,
                         pos_local.x,
                         pos_local.y,
                         pos_local.z,
                         inst_it->second.translation_world.x,
                         inst_it->second.translation_world.y,
                         inst_it->second.translation_world.z,
                         inst_it->second.scale.x,
                         inst_it->second.scale.y,
                         inst_it->second.scale.z);
            ++debug_dynamic_root_logs;
        }
    }
}
