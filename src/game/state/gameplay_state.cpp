#include "gameplay_state.h"
#include "pause_state.h"
#include "runtime/game_runtime.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/input/input_system.h"

#include "imgui.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

namespace Game
{

void GameplayState::on_enter(GameStateContext &ctx)
{
    _world.set_api(ctx.api);
    _elapsed = 0.0f;

    setup_scene(ctx);
}

void GameplayState::on_exit(GameStateContext &ctx)
{
    _world.clear_rebase_anchor();
    _world.clear();
    _world.set_physics(nullptr);
    _world.set_physics_context(nullptr);
    _world.set_api(nullptr);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
    if (ctx.renderer && ctx.renderer->_context)
    {
        if (ctx.renderer->_context->physics_context == _physics_context.get())
        {
            ctx.renderer->_context->physics_context = nullptr;
        }
    }
    _physics_context.reset();
    _physics.reset();
#endif
}

void GameplayState::on_update(GameStateContext &ctx, float dt)
{
    _elapsed += dt;

    // ESC to pause
    if (ctx.input && ctx.input->key_pressed(Key::Escape))
    {
        _pending = StateTransition::push<PauseState>();
        return;
    }

    // Update components (variable timestep)
    const float alpha = ctx.interpolation_alpha();
    ComponentContext comp_ctx = build_component_context(ctx, alpha);
    _world.entities().update_components(comp_ctx, dt);

    // Sync entities to render
    _world.entities().sync_to_render(*ctx.api, alpha);
}

void GameplayState::on_fixed_update(GameStateContext &ctx, float fixed_dt)
{
    // Fixed update components (input → forces, game logic)
    ComponentContext comp_ctx = build_component_context(ctx);
    _world.entities().fixed_update_components(comp_ctx, fixed_dt);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
    if (!_physics)
    {
        return;
    }

    _world.pre_physics_step();
    _physics->step(fixed_dt);
    _world.post_physics_step();
#endif
}

void GameplayState::on_draw_ui(GameStateContext &ctx)
{
    (void)ctx;

    // Minimal HUD placeholder
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 10, viewport->WorkPos.y + 10));
    ImGui::SetNextWindowBgAlpha(0.4f);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration
                           | ImGuiWindowFlags_NoInputs
                           | ImGuiWindowFlags_AlwaysAutoResize;

    if (ImGui::Begin("##GameHUD", nullptr, flags))
    {
        ImGui::Text("Time: %.1f s", _elapsed);
        ImGui::Text("[ESC] Pause");
    }
    ImGui::End();
}

void GameplayState::setup_scene(GameStateContext &ctx)
{
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
    _physics = std::make_unique<Physics::JoltPhysicsWorld>();
    _physics_context = std::make_unique<Physics::PhysicsContext>(*_physics);

    _world.set_physics(_physics.get());
    _world.set_physics_context(_physics_context.get());

    // Expose to EngineContext for debug draw
    if (ctx.renderer && ctx.renderer->_context)
    {
        ctx.renderer->_context->physics_context = _physics_context.get();
    }
#endif

    // Setup camera
    if (ctx.api)
    {
        ctx.api->set_camera_position(glm::vec3(0.0f, 5.0f, -20.0f));
        ctx.api->camera_look_at(glm::vec3(0.0f, 0.0f, 0.0f));
    }

    // TODO: Spawn initial game entities (ships, celestial bodies, etc.)
    // This will be filled in as we build the orbital mechanics and ship systems.
}

ComponentContext GameplayState::build_component_context(GameStateContext &ctx, float alpha)
{
    ComponentContext comp_ctx{};
    comp_ctx.world = &_world;
    comp_ctx.api = ctx.api;
    comp_ctx.input = ctx.input;
    comp_ctx.physics = _physics.get();
    comp_ctx.interpolation_alpha = alpha;
    return comp_ctx;
}

} // namespace Game
