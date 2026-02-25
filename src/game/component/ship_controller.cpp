#include "ship_controller.h"
#include "game/entity.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/physics_world.h"
#include "physics/physics_body.h"
#endif

#include <glm/geometric.hpp>
#include <algorithm>
#include <cmath>

namespace Game
{
    void ShipController::on_fixed_update(ComponentContext &ctx, float fixed_dt)
    {
        _thrust_applied_this_tick = false;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!ctx.input || !ctx.physics || !entity() || !entity()->has_physics())
        {
            return;
        }

        const Physics::BodyId body_id{entity()->physics_body_value()};
        if (!ctx.physics->is_body_valid(body_id))
        {
            return;
        }

        const auto *input = ctx.input;

        // --- SAS toggle (T key, edge-triggered) ---
        // fixed_update may run multiple times per frame; use key_down edge detection.
        const bool sas_toggle_down = input->key_down(Key::T);
        if (!ctx.ui_capture_keyboard && sas_toggle_down && !_sas_toggle_prev_down)
        {
            _sas_enabled = !_sas_enabled;
        }
        _sas_toggle_prev_down = sas_toggle_down;

        const bool allow_keyboard = !ctx.ui_capture_keyboard;

        // --- Translation input (local frame) ---
        glm::vec3 local_thrust{0.0f};

        if (allow_keyboard)
        {
            if (input->key_down(Key::W)) local_thrust.z -= 1.0f; // forward = -Z
            if (input->key_down(Key::S)) local_thrust.z += 1.0f; // back    = +Z
            if (input->key_down(Key::A)) local_thrust.x -= 1.0f; // left    = -X
            if (input->key_down(Key::D)) local_thrust.x += 1.0f; // right   = +X
            if (input->key_down(Key::Space))    local_thrust.y += 1.0f; // up   = +Y
            if (input->key_down(Key::LeftCtrl)) local_thrust.y -= 1.0f; // down = -Y
        }

        // Normalize so diagonal thrust isn't stronger
        if (glm::length(local_thrust) > 0.0f)
        {
            local_thrust = glm::normalize(local_thrust);
        }

        _last_thrust_dir = local_thrust;

        // --- Rotation input (local frame) ---
        glm::vec3 local_torque{0.0f};

        if (allow_keyboard)
        {
            if (input->key_down(Key::ArrowUp))   local_torque.x -= 1.0f; // pitch up   = -X
            if (input->key_down(Key::ArrowDown)) local_torque.x += 1.0f; // pitch down = +X
            if (input->key_down(Key::ArrowLeft))  local_torque.y += 1.0f; // yaw left   = +Y
            if (input->key_down(Key::ArrowRight)) local_torque.y -= 1.0f; // yaw right  = -Y
            if (input->key_down(Key::Q)) local_torque.z -= 1.0f; // roll left  = -Z
            if (input->key_down(Key::E)) local_torque.z += 1.0f; // roll right = +Z
        }

        if (glm::length(local_torque) > 0.0f)
        {
            local_torque = glm::normalize(local_torque);
        }

        const bool has_rotation_input = glm::length(local_torque) > 0.0f;

        // --- Transform local -> world ---
        const glm::quat ship_rot = ctx.physics->get_rotation(body_id);

        const glm::vec3 world_force = ship_rot * (local_thrust * _thrust_force);
        const glm::vec3 world_torque = ship_rot * (local_torque * _torque_strength);

        // --- Apply force & torque ---
        if (glm::length(world_force) > 0.0f)
        {
            ctx.physics->add_force(body_id, world_force);
            _thrust_applied_this_tick = true;
        }

        if (glm::length(world_torque) > 0.0f)
        {
            ctx.physics->add_torque(body_id, world_torque);
        }

        // --- SAS: mass/inertia-independent angular damping when enabled ---
        if (_sas_enabled && !has_rotation_input)
        {
            const glm::vec3 angular_vel = ctx.physics->get_angular_velocity(body_id);
            const float ang_speed = glm::length(angular_vel);

            if (ang_speed > 1e-4f)
            {
                // Exponential decay is frame-rate independent and avoids mass-dependent SAS feel.
                const float damping = std::max(0.0f, _sas_damping);
                const float decay = std::exp(-damping * std::max(0.0f, fixed_dt));
                glm::vec3 damped_angular_vel = angular_vel * decay;

                // Snap tiny residual spin to zero to avoid endless micro-jitter.
                if (glm::length(damped_angular_vel) < 1e-3f)
                {
                    damped_angular_vel = glm::vec3(0.0f);
                }

                ctx.physics->set_angular_velocity(body_id, damped_angular_vel);
            }
        }
#endif // VULKAN_ENGINE_USE_JOLT
    }
} // namespace Game
