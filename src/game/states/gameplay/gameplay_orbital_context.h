#pragma once

#include "game/states/gameplay/orbital_physics_system.h"

#include <functional>

namespace Game
{
    struct GameplayOrbitalContextInputs
    {
        VulkanEngine *renderer{nullptr};
        GameWorld &world;
        OrbitalRuntimeSystem &orbit;
        Physics::PhysicsWorld *physics{nullptr};
        Physics::PhysicsContext *physics_context{nullptr};
        const ScenarioConfig &scenario_config;
        const Keybinds *keybinds{nullptr};
        std::function<bool(const GameStateContext &)> ui_capture_keyboard{};
        std::function<void()> mark_prediction_dirty{};
    };

    class GameplayOrbitalContextBuilder
    {
    public:
        explicit GameplayOrbitalContextBuilder(GameplayOrbitalContextInputs inputs);

        [[nodiscard]] OrbitalPhysicsSystem::Context build() const;

    private:
        GameplayOrbitalContextInputs _inputs;
    };
} // namespace Game
