#include "game/states/gameplay/gameplay_orbital_context.h"

#include "game/states/gameplay/orbiter_world_state_provider.h"

#include <utility>

namespace Game
{
    GameplayOrbitalContextBuilder::GameplayOrbitalContextBuilder(GameplayOrbitalContextInputs inputs)
        : _inputs(std::move(inputs))
    {
    }

    OrbitalPhysicsSystem::Context GameplayOrbitalContextBuilder::build() const
    {
        GameplayOrbitalContextInputs inputs = _inputs;
        return OrbitalPhysicsSystem::Context{
            .renderer = inputs.renderer,
            .world = inputs.world,
            .orbit = inputs.orbit,
            .physics = inputs.physics,
            .physics_context = inputs.physics_context,
            .scenario_config = inputs.scenario_config,
            .keybinds = inputs.keybinds,
            .orbiter_world_state_sampler =
                    [inputs](const OrbiterInfo &sample_orbiter,
                             WorldVec3 &out_pos_world,
                             glm::dvec3 &out_vel_world,
                             glm::vec3 &out_vel_local) {
                        return OrbiterWorldStateProvider(OrbiterWorldStateProvider::Context{
                                .orbit = inputs.orbit,
                                .world = inputs.world,
                                .physics = inputs.physics,
                                .physics_context = inputs.physics_context,
                                .scenario_config = inputs.scenario_config,
                        }).get_orbiter_world_state(
                                sample_orbiter,
                                out_pos_world,
                                out_vel_world,
                                out_vel_local);
                    },
            .ui_capture_keyboard = std::move(inputs.ui_capture_keyboard),
            .mark_prediction_dirty = std::move(inputs.mark_prediction_dirty),
        };
    }
} // namespace Game
