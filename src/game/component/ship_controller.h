#pragma once

#include "game/component/component.h"
#include "core/input/input_system.h"

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Game
{
    // ============================================================================
    // ShipController: Reads keyboard input and applies thrust/torque to a ship
    //
    // Translation (on/off thrust): W/S = forward/back, A/D = left/right,
    //                               Space/LCtrl = up/down
    // Rotation: ArrowUp/Down = pitch, ArrowLeft/Right = yaw, Q/E = roll
    // SAS toggle: T - kills angular velocity (stability assist)
    //
    // All forces applied in ship-local frame, transformed to world frame.
    // ============================================================================

    class ShipController : public Component<ShipController>
    {
    public:
        ShipController() = default;

        explicit ShipController(float thrust_force, float torque_strength)
            : _thrust_force(thrust_force), _torque_strength(torque_strength)
        {
        }

        void on_fixed_update(ComponentContext &ctx, float fixed_dt) override;

        // Config
        float thrust_force() const { return _thrust_force; }
        void set_thrust_force(float f) { _thrust_force = f; }

        float torque_strength() const { return _torque_strength; }
        void set_torque_strength(float t) { _torque_strength = t; }

        float sas_damping() const { return _sas_damping; }
        void set_sas_damping(float d) { _sas_damping = d; }

        // State
        bool sas_enabled() const { return _sas_enabled; }
        glm::vec3 last_thrust_dir() const { return _last_thrust_dir; }
        bool thrust_applied_this_tick() const { return _thrust_applied_this_tick; }

    private:
        float _thrust_force{500.0f};
        float _torque_strength{50.0f};
        float _sas_damping{5.0f};
        bool _sas_enabled{false};
        bool _sas_toggle_prev_down{false};

        // Last-frame state for HUD display
        glm::vec3 _last_thrust_dir{0.0f};
        bool _thrust_applied_this_tick{false};
    };
} // namespace Game
