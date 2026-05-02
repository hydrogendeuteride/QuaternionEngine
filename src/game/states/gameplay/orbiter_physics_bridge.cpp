#include "game/states/gameplay/orbiter_physics_bridge.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "core/engine.h"
#include "core/util/logger.h"
#include "game/game_world.h"
#include "physics/body_settings.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"
#endif

#include <cmath>
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include <algorithm>
#include <optional>
#endif

namespace Game
{
    WorldVec3 OrbiterPhysicsBridge::body_world_from_body_local(const glm::dvec3 &body_position_local,
                                                               const WorldVec3 &physics_origin_world)
    {
        return local_to_world_d(body_position_local, physics_origin_world);
    }

    glm::dvec3 OrbiterPhysicsBridge::body_local_from_body_world(const WorldVec3 &body_position_world,
                                                                const WorldVec3 &physics_origin_world)
    {
        return world_to_local_d(body_position_world, physics_origin_world);
    }

    WorldVec3 OrbiterPhysicsBridge::entity_world_from_body_world(const Entity &entity,
                                                                 const WorldVec3 &body_position_world,
                                                                 const glm::quat &rotation)
    {
        return entity.entity_position_from_physics_center_of_mass_world(body_position_world, rotation);
    }

    WorldVec3 OrbiterPhysicsBridge::body_world_from_entity_world(const Entity &entity,
                                                                 const WorldVec3 &entity_position_world,
                                                                 const glm::quat &rotation)
    {
        return entity.physics_center_of_mass_world(entity_position_world, rotation);
    }

    Physics::BodyId OrbiterPhysicsBridge::create_body(const OrbiterPhysicsBridgeContext &context,
                                                      const bool render_is_gltf,
                                                      Entity &entity,
                                                      const Physics::BodySettings &settings_template,
                                                      const WorldVec3 &position_world,
                                                      const glm::quat &rotation,
                                                      glm::vec3 *out_origin_offset_local)
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!context.physics || !context.physics_context)
        {
            return {};
        }

        if (out_origin_offset_local)
        {
            *out_origin_offset_local = glm::vec3(0.0f);
        }

        const uint64_t user_data = static_cast<uint64_t>(entity.id().value);

        if (render_is_gltf)
        {
            if (!context.renderer || !context.renderer->_sceneManager || entity.render_name().empty())
            {
                Logger::error("[OrbiterPhysicsBridge] Cannot create glTF collider body for '{}'", entity.name());
                return {};
            }

            (void) context.renderer->_sceneManager->setGLTFInstanceTRSWorld(entity.render_name(),
                                                                            position_world,
                                                                            rotation,
                                                                            entity.scale());

            std::optional<float> mass_override;
            if (settings_template.has_explicit_mass &&
                std::isfinite(settings_template.mass) &&
                settings_template.mass > 0.0f)
            {
                mass_override = settings_template.mass;
            }

            Physics::BodyId body_id =
                    context.renderer->_sceneManager->enableDynamicRootColliderBody(entity.render_name(),
                                                                                   context.physics,
                                                                                   settings_template.layer,
                                                                                   user_data,
                                                                                   mass_override);
            if (!body_id.is_valid())
            {
                return {};
            }

            if (out_origin_offset_local)
            {
                glm::vec3 origin_offset_local{0.0f, 0.0f, 0.0f};
                if (context.renderer->_sceneManager->getDynamicRootColliderCenterOfMassLocal(entity.render_name(),
                                                                                             origin_offset_local))
                {
                    *out_origin_offset_local = origin_offset_local;
                }
            }

            context.physics->set_gravity_scale(body_id, settings_template.gravity_scale);
            if (settings_template.motion_type != Physics::MotionType::Dynamic)
            {
                (void) context.physics->set_motion_type(body_id, settings_template.motion_type);
            }
            if (!settings_template.start_active)
            {
                context.physics->deactivate(body_id);
            }
            entity.set_render_sync_mode(Entity::RenderSyncMode::Authoritative);
            return body_id;
        }

        Physics::BodySettings settings = settings_template;
        settings.position = body_local_from_body_world(position_world, context.physics_context->origin_world());
        settings.rotation = rotation;
        settings.user_data = user_data;
        return context.physics->create_body(settings);
#else
        (void) context;
        (void) render_is_gltf;
        (void) entity;
        (void) settings_template;
        (void) position_world;
        (void) rotation;
        (void) out_origin_offset_local;
        return {};
#endif
    }

    bool OrbiterPhysicsBridge::destroy_body(const OrbiterPhysicsBridgeContext &context,
                                            const bool render_is_gltf,
                                            Entity &entity)
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!context.physics || !entity.has_physics())
        {
            return false;
        }

        const Physics::BodyId body_id{entity.physics_body_value()};
        bool destroyed = false;

        if (render_is_gltf && context.renderer && context.renderer->_sceneManager && !entity.render_name().empty())
        {
            destroyed = context.renderer->_sceneManager->disableDynamicRootColliderBody(entity.render_name());
        }

        if (!destroyed && context.physics->is_body_valid(body_id))
        {
            context.physics->destroy_body(body_id);
            destroyed = true;
        }

        entity.clear_physics_body();
        if (render_is_gltf)
        {
            entity.set_render_sync_mode(Entity::RenderSyncMode::Interpolated);
        }
        return destroyed;
#else
        (void) context;
        (void) render_is_gltf;
        (void) entity;
        return false;
#endif
    }

    bool OrbiterPhysicsBridge::read_body_state(const OrbiterPhysicsBridgeContext &context,
                                               const Entity &entity,
                                               OrbiterBodyState &out_state)
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!context.physics || !context.physics_context || !entity.has_physics())
        {
            return false;
        }

        const Physics::BodyId body_id{entity.physics_body_value()};
        if (!context.physics->is_body_valid(body_id))
        {
            return false;
        }

        out_state.rotation = context.physics->get_rotation(body_id);
        const WorldVec3 entity_origin_world =
                body_world_from_body_local(context.physics->get_position(body_id),
                                           context.physics_context->origin_world());
        out_state.body_position_world =
                body_world_from_entity_world(entity, entity_origin_world, out_state.rotation);
        out_state.velocity_world =
                context.physics_context->velocity_origin_world() +
                glm::dvec3(context.physics->get_linear_velocity(body_id));
        out_state.angular_velocity_world = context.physics->get_angular_velocity(body_id);
        return true;
#else
        (void) context;
        (void) entity;
        (void) out_state;
        return false;
#endif
    }

    bool OrbiterPhysicsBridge::restore_body_state(const OrbiterPhysicsBridgeContext &context,
                                                  const bool render_is_gltf,
                                                  Entity &entity,
                                                  const Physics::BodySettings &settings,
                                                  const bool use_physics_interpolation,
                                                  const OrbiterBodyState &state)
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!context.physics || !context.physics_context || !context.world)
        {
            return false;
        }

        const WorldVec3 entity_pos_world =
                entity_world_from_body_world(entity, state.body_position_world, state.rotation);

        Physics::BodyId body_id{};
        if (entity.has_physics())
        {
            body_id = Physics::BodyId{entity.physics_body_value()};
        }

        if (!body_id.is_valid() || !context.physics->is_body_valid(body_id))
        {
            if (entity.has_physics())
            {
                entity.clear_physics_body();
            }

            glm::vec3 origin_offset_local{0.0f, 0.0f, 0.0f};
            body_id = create_body(context,
                                  render_is_gltf,
                                  entity,
                                  settings,
                                  entity_pos_world,
                                  state.rotation,
                                  &origin_offset_local);
            if (!body_id.is_valid() ||
                !context.world->bind_physics(entity.id(),
                                             body_id.value,
                                             use_physics_interpolation,
                                             false,
                                             origin_offset_local))
            {
                return false;
            }
        }

        context.physics->set_transform(body_id,
                                       body_local_from_body_world(entity_pos_world,
                                                                  context.physics_context->origin_world()),
                                       state.rotation);
        context.physics->set_linear_velocity(
                body_id,
                glm::vec3(state.velocity_world - context.physics_context->velocity_origin_world()));
        context.physics->set_angular_velocity(body_id, state.angular_velocity_world);
        context.physics->activate(body_id);

        entity.set_position_world(entity_pos_world);
        entity.set_rotation(state.rotation);
        if (entity.uses_interpolation())
        {
            entity.interpolation().set_immediate(entity_pos_world, state.rotation);
        }
        return true;
#else
        (void) context;
        (void) render_is_gltf;
        (void) entity;
        (void) settings;
        (void) use_physics_interpolation;
        (void) state;
        return false;
#endif
    }
} // namespace Game
