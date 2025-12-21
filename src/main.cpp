// Main entry point for Vulkan Engine
//
// Two modes are available:
// 1. Legacy mode: Uses VulkanEngine::run() directly (simple, no game separation)
// 2. GameRuntime mode: Uses GameRuntime for clean game/engine separation
//
// Set USE_GAME_RUNTIME to 1 to enable GameRuntime with example callbacks.

#define USE_GAME_RUNTIME 1

#include "core/engine.h"

#if USE_GAME_RUNTIME
#include "runtime/game_runtime.h"
#include <glm/gtx/transform.hpp>

// Example game implementation using IGameCallbacks
class ExampleGame : public GameRuntime::IGameCallbacks
{
public:
    void on_init(GameRuntime::Runtime& runtime) override
    {
        // Example: Set up initial scene
        auto& api = runtime.api();

        // Load a glTF model asynchronously
        // api.load_gltf_async("example_model", "models/example.gltf",
        //     GameAPI::Transform{}.with_position({0, 0, 0}));

        // Spawn a primitive
        // api.spawn_mesh_instance("test_cube", api.primitive_cube(),
        //     GameAPI::Transform{}.with_position({2, 0, 0}));

        // Set up camera
        // api.set_camera_position({0, 5, -10});
        // api.set_camera_rotation({-20, 0, 0});
    }

    void on_update(float dt) override
    {
        // Called every frame with variable delta time
        // Use for rendering-dependent logic, input handling, camera control
        _elapsed += dt;
    }

    void on_fixed_update(float fixed_dt) override
    {
        // Called at fixed intervals (default: 60Hz)
        // Use for physics updates, AI tick, game state simulation
        // Example: Apply physics forces, update AI state machines
    }

    void on_shutdown() override
    {
        // Called before engine shutdown
        // Use for cleanup, saving game state, etc.
    }

private:
    float _elapsed{0.0f};
};
#endif // USE_GAME_RUNTIME

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    VulkanEngine engine;
    engine.init();

#if USE_GAME_RUNTIME
    // GameRuntime mode: clean separation between engine and game logic
    {
        GameRuntime::Runtime runtime(&engine);
        ExampleGame game;
        runtime.run(&game);
    }
#else
    // Legacy mode: simple direct engine loop
    engine.run();
#endif

    engine.cleanup();
    return 0;
}