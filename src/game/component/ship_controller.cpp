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
    ThrustInput ShipController::read_input(const InputState *input, const ShipKeybinds *binds,
                                           const bool ui_capture_keyboard, bool &sas_toggle_prev_down)
    {
        ThrustInput out{};
        if (!input)
        {
            sas_toggle_prev_down = false;
            return out;
        }

        const bool allow_keyboard = !ui_capture_keyboard;

        // Fall back to defaults when no bindings are configured.
        ShipKeybinds default_binds{};
        const ShipKeybinds &kb = binds ? *binds : default_binds;

        auto pressed = [&](const Key k) {
            return k != Key::Unknown && input->key_down(k);
        };

        // --- SAS toggle (edge-triggered; fixed_update may run multiple times per frame) ---
        const bool sas_toggle_down = pressed(kb.sas_toggle);
        if (allow_keyboard && sas_toggle_down && !sas_toggle_prev_down)
        {
            out.sas_toggled = true;
        }
        sas_toggle_prev_down = sas_toggle_down;

        glm::vec3 local_thrust{0.0f};
        if (allow_keyboard)
        {
            if (pressed(kb.thrust_forward)) local_thrust.z -= 1.0f;
            if (pressed(kb.thrust_back))    local_thrust.z += 1.0f;
            if (pressed(kb.thrust_left))    local_thrust.x -= 1.0f;
            if (pressed(kb.thrust_right))   local_thrust.x += 1.0f;
            if (pressed(kb.thrust_up))      local_thrust.y += 1.0f;
            if (pressed(kb.thrust_down))    local_thrust.y -= 1.0f;
        }
        if (glm::length(local_thrust) > 0.0f)
        {
            local_thrust = glm::normalize(local_thrust);
        }
        out.local_thrust_dir = local_thrust;

        glm::vec3 local_torque{0.0f};
        if (allow_keyboard)
        {
            if (pressed(kb.pitch_up))   local_torque.x -= 1.0f;
            if (pressed(kb.pitch_down)) local_torque.x += 1.0f;
            if (pressed(kb.yaw_left))   local_torque.y += 1.0f;
            if (pressed(kb.yaw_right))  local_torque.y -= 1.0f;
            if (pressed(kb.roll_left))  local_torque.z -= 1.0f;
            if (pressed(kb.roll_right)) local_torque.z += 1.0f;
        }
        if (glm::length(local_torque) > 0.0f)
        {
            local_torque = glm::normalize(local_torque);
        }
        out.local_torque_dir = local_torque;

        return out;
    }

    void ShipController::on_fixed_update(ComponentContext &ctx, float fixed_dt)
    {
        _thrust_applied_this_tick = false;

        const ShipKeybinds *ship_binds = ctx.keybinds ? &ctx.keybinds->ship : nullptr;
        const ThrustInput input = read_input(ctx.input, ship_binds, ctx.ui_capture_keyboard, _sas_toggle_prev_down);
        if (input.sas_toggled)
        {
            _sas_enabled = !_sas_enabled;
        }
        _last_thrust_dir = input.local_thrust_dir;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!ctx.physics || !entity() || !entity()->has_physics())
        {
            return;
        }

        const Physics::BodyId body_id{entity()->physics_body_value()};
        if (!ctx.physics->is_body_valid(body_id))
        {
            return;
        }

        const bool has_rotation_input = glm::length(input.local_torque_dir) > 0.0f;

        // --- Transform local -> world ---
        const glm::quat ship_rot = ctx.physics->get_rotation(body_id);

        const glm::vec3 world_force = ship_rot * (input.local_thrust_dir * _thrust_force);
        const glm::vec3 world_torque = ship_rot * (input.local_torque_dir * _torque_strength);

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
