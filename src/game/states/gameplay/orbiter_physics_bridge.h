#pragma once

#include "core/world.h"
#include "game/entity.h"
#include "physics/physics_body.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class VulkanEngine;

namespace Physics
{
    class PhysicsContext;
    class PhysicsWorld;
    struct BodySettings;
} // namespace Physics

namespace Game
{
    class GameWorld;

    struct OrbiterPhysicsBridgeContext
    {
        VulkanEngine *renderer{nullptr};
        GameWorld *world{nullptr};
        Physics::PhysicsWorld *physics{nullptr};
        Physics::PhysicsContext *physics_context{nullptr};
    };

    struct OrbiterBodyState
    {
        WorldVec3 body_position_world{0.0, 0.0, 0.0};
        glm::dvec3 velocity_world{0.0, 0.0, 0.0};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 angular_velocity_world{0.0f, 0.0f, 0.0f};
    };

    class OrbiterPhysicsBridge
    {
    public:
        static WorldVec3 body_world_from_body_local(const glm::dvec3 &body_position_local,
                                                    const WorldVec3 &physics_origin_world);
        static glm::dvec3 body_local_from_body_world(const WorldVec3 &body_position_world,
                                                     const WorldVec3 &physics_origin_world);
        static WorldVec3 entity_world_from_body_world(const Entity &entity,
                                                      const WorldVec3 &body_position_world,
                                                      const glm::quat &rotation);
        static WorldVec3 body_world_from_entity_world(const Entity &entity,
                                                      const WorldVec3 &entity_position_world,
                                                      const glm::quat &rotation);

        static Physics::BodyId create_body(const OrbiterPhysicsBridgeContext &context,
                                           bool render_is_gltf,
                                           Entity &entity,
                                           const Physics::BodySettings &settings_template,
                                           const WorldVec3 &position_world,
                                           const glm::quat &rotation,
                                           glm::vec3 *out_origin_offset_local = nullptr);
        static bool destroy_body(const OrbiterPhysicsBridgeContext &context,
                                 bool render_is_gltf,
                                 Entity &entity);
        static bool read_body_state(const OrbiterPhysicsBridgeContext &context,
                                    const Entity &entity,
                                    OrbiterBodyState &out_state);
        static bool restore_body_state(const OrbiterPhysicsBridgeContext &context,
                                       bool render_is_gltf,
                                       Entity &entity,
                                       const Physics::BodySettings &settings,
                                       bool use_physics_interpolation,
                                       const OrbiterBodyState &state);
    };
} // namespace Game
