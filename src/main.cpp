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
        auto& api = runtime.api();
        VulkanEngine* renderer = runtime.renderer();
        if (renderer && renderer->_assetManager)
        {
            GameAPI::IBLPaths ibl{};
            ibl.specularCube = renderer->_assetManager->assetPath("ibl/docklands.ktx2");
            ibl.diffuseCube = renderer->_assetManager->assetPath("ibl/docklands.ktx2"); // fallback: reuse specular for diffuse
            ibl.brdfLut = renderer->_assetManager->assetPath("ibl/brdf_lut.ktx2");
            // Optional dedicated background texture (2D equirect); if omitted, background falls back to specularCube.
            ibl.background = renderer->_assetManager->assetPath("ibl/sky.KTX2");
            api.load_global_ibl(ibl);
        }

        // Planet demo: Earth terrain + Moon sphere (real scale, meters).
        {
            constexpr double kEarthRadiusM = 6378137.0;
            constexpr double kMoonRadiusM = 1737400.0;
            constexpr double kMoonDistanceM = 384400000.0;

            GameAPI::PlanetTerrain earth{};
            earth.name = "Earth";
            earth.center = glm::dvec3(0.0, 0.0, 0.0);
            earth.radius_m = kEarthRadiusM;
            earth.visible = true;
            earth.base_color = glm::vec4(1.0f);
            earth.metallic = 0.0f;
            earth.roughness = 1.0f;
            earth.albedo_dir = "planets/earth/albedo/L0";
            earth.height_dir = "planets/earth/height/L0";
            earth.height_max_m = 6400.0;
            api.add_planet_terrain(earth);

            GameAPI::PlanetSphere moon{};
            moon.name = "Moon";
            moon.center = glm::dvec3(kMoonDistanceM, 0.0, 0.0);
            moon.radius_m = kMoonRadiusM;
            moon.visible = true;
            moon.base_color = glm::vec4(0.72f, 0.72f, 0.75f, 1.0f);
            moon.metallic = 0.0f;
            moon.roughness = 1.0f;
            api.add_planet_sphere(moon);

            GameAPI::FreeCameraSettings free = api.get_free_camera_settings();
            free.moveSpeed = 20000.0f;
            api.set_free_camera_settings(free);

            api.set_camera_position(glm::dvec3(0.0, 0.0, kEarthRadiusM + 1.0e6));
            api.camera_look_at(glm::dvec3(0.0, 0.0, 0.0));
        }

        // Load a glTF model asynchronously
        // api.add_gltf_instance_async("example_model", "models/example.gltf",
        //     GameAPI::Transform{.position = {0, 0, 0}});

        // Spawn a primitive
        // api.add_primitive_instance("test_cube", GameAPI::PrimitiveType::Cube,
        //     GameAPI::Transform{.position = {2, 0, 0}});

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
