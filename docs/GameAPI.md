## Game‑Facing API Overview

This document summarizes the main engine APIs that are directly useful when building a game (spawn actors, control lights/animations, and interact via picking).

For details on the underlying systems, see also:

- `docs/Scene.md` – cameras, draw context, instances, picking.
- `docs/EngineContext.md` – access to managers and per‑frame state.
- `docs/RenderGraph.md` – render‑graph API for custom passes.
- `docs/InputSystem.md` – keyboard, mouse, and cursor mode handling.
- `docs/Picking.md` – object selection, hover detection, and drag-box multi-select.
- `docs/ImGuiSystem.md` – immediate-mode UI integration and debug widgets.

---

## `GameAPI::Engine` (High‑Level Game Wrapper)

Header: `src/core/game_api.h`  
Implementation: `src/core/game_api.cpp`

`GameAPI::Engine` is a thin, game‑friendly wrapper around `VulkanEngine`. It exposes stable, snake_case methods grouped by responsibility:

- Texture streaming and VRAM budget.
- Shadows and reflections.
- IBL volumes.
- Instances and animation.
- Post‑processing (tonemap, bloom, FXAA).
- Camera control.
- Picking and render‑graph pass toggles.
- Input handling (keyboard, mouse, cursor modes).

Typical creation:

```cpp
#include "core/engine.h"
#include "core/game_api.h"

VulkanEngine engine;
engine.init();

GameAPI::Engine api(&engine); // non‑owning
```

You then call `api.*` from your game loop to spawn content and tweak settings.

### Texture Streaming & VRAM Budget

Relevant methods:

- `size_t get_texture_budget() const;`
- `void set_texture_loads_per_frame(int count);`
- `void set_texture_upload_budget(size_t bytes);`
- `void set_cpu_source_budget(size_t bytes);`
- `void set_max_upload_dimension(uint32_t dim);`
- `void set_keep_source_bytes(bool keep);`
- `void evict_textures_to_budget();`

At a lower level, `VulkanEngine::query_texture_budget_bytes()` computes a conservative per‑frame texture budget using VMA heap info and constants in `src/core/config.h`:

- `kTextureBudgetFraction` – fraction of total device‑local VRAM reserved for streamed textures (default `0.35`).
- `kTextureBudgetFallbackBytes` – fallback budget when memory properties are unavailable (default `512 MiB`).
- `kTextureBudgetMinBytes` – minimum budget clamp (default `128 MiB`).

To globally change how aggressive streaming can be, edit these constants in `config.h` and rebuild. Use the `GameAPI::Engine` setters for per‑scene tuning (e.g. reducing upload bandwidth on low‑end machines).

### Shadows: Resolution, Quality, and RT Modes

Shadows are controlled by a combination of:

- Global settings in `EngineContext::shadowSettings`.
- Config constants in `src/core/config.h`.
- The `ShadowPass` render pass and lighting shader (`shadow.vert`, `deferred_lighting.frag`).

High‑level game‑side controls:

- `void set_shadows_enabled(bool enabled);`
- `void set_shadow_mode(ShadowMode mode);`
  - `ClipmapOnly` – cascaded shadow maps only.
  - `ClipmapPlusRT` – cascades + optional ray‑query assist.
  - `RTOnly` – ray‑traced shadows only (no raster maps).
- `void set_hybrid_ray_cascade_mask(uint32_t mask);`
- `void set_hybrid_ray_threshold(float threshold);`

These map directly onto `EngineContext::shadowSettings` and are also visualized in the ImGui “Shadows / Ray Query” tab.

#### Shadow Map Resolution (`kShadowMapResolution`)

The shadow map resolution is driven by `kShadowMapResolution` in `src/core/config.h`:

- Used for:
  - The actual depth image size created for each cascaded shadow map in `VulkanEngine::draw()` (`shadowExtent`).
  - Texel snapping for cascade stabilization in `SceneManager::update_scene()`:
    - `texel = (2.0f * cover) / float(kShadowMapResolution);`
- Default: `2048.0f`, which gives a good compromise between quality and VRAM usage on mid‑range GPUs.

Increasing `kShadowMapResolution` has two important effects:

- **VRAM cost grows quadratically.**
  - Depth D32F, per cascade:
    - 2048 → ~16 MB.
    - 4096 → ~64 MB.
  - With 4 cascades, 4096×4096 can consume ~256 MB just for shadow depth, on top of swapchain, HDR, G‑buffers, IBL, and other images.
- **Allocation failures can effectively “kill” shadows.**
  - All shadow maps are created as transient RenderGraph images each frame run.
  - If VMA runs out of suitable device‑local memory, `vmaCreateImage` will fail, and the engine will assert via `VK_CHECK`. In practice (especially in a release build), this often manifests as:
    - No shadow rendering, or
    - The app aborting when the first frame tries to allocate these images.

Practical guidance:

- Prefer 2048 or 3072 on consumer hardware unless you have headroom and have profiled memory.
- If you push to 4096 and shadows “disappear”, suspect VRAM pressure:
  - Try reducing `kTextureBudgetFraction` so textures use less VRAM.
  - Or bring `kShadowMapResolution` back down and re‑test.

The following quality‑related shadow constants also live in `config.h`:

- `kShadowCascadeCount`, `kShadowCSMFar`, `kShadowCascadeRadiusScale`, `kShadowCascadeRadiusMargin`.
- `kShadowBorderSmoothNDC`, `kShadowPCFBaseRadius`, `kShadowPCFCascadeGain`.
- `kShadowDepthBiasConstant`, `kShadowDepthBiasSlope`.

These affect how cascades are distributed and how soft/filtered the resulting shadows are. Changing them is safe but should be tested against your content and FOV ranges.

### Reflections and Post‑Processing

Game‑side reflection controls:

- `void set_ssr_enabled(bool enabled);`
- `void set_reflection_mode(ReflectionMode mode);`  
  (`SSROnly`, `SSRPlusRT`, `RTOnly`)

Tone mapping and bloom:

- `void set_exposure(float exposure);`
- `void set_tonemap_operator(TonemapOperator op);` (`Reinhard`, `ACES`)
- `void set_bloom_enabled(bool enabled);`
- `void set_bloom_threshold(float threshold);`
- `void set_bloom_intensity(float intensity);`

These wrap `TonemapPass` parameters and are equivalent to flipping the corresponding ImGui controls at runtime.

FXAA:

- `void set_fxaa_enabled(bool enabled);`
- `void set_fxaa_edge_threshold(float threshold);`
- `void set_fxaa_edge_threshold_min(float threshold);`

### Camera and Render Scale

Camera:

- `void set_camera_position(const glm::vec3 &position);`
- `glm::vec3 get_camera_position() const;`
- `void set_camera_rotation(float pitchDeg, float yawDeg);`
- `void get_camera_rotation(float &pitchDeg, float &yawDeg) const;`
- `void set_camera_fov(float fovDegrees);`
- `float get_camera_fov() const;`
- `void camera_look_at(const glm::vec3 &target);`

These functions internally manipulate the quaternion‑based `Camera::orientation` and `position` in `SceneManager`. They respect the engine’s `-Z` forward convention.

Render resolution scaling:

- `void set_render_scale(float scale); // 0.3–1.0`
- `float get_render_scale() const;`

This scales the internal draw extent relative to the swapchain and main HDR image sizes, trading resolution for performance.

### Picking & Pass Toggles

Picking:

- `Engine::PickResult get_last_pick() const;`
- `void set_use_id_buffer_picking(bool use);`
- `bool get_use_id_buffer_picking() const;`

These mirror `VulkanEngine::get_last_pick()` and `_useIdBufferPicking`, letting you choose between:

- CPU raycast picking (immediate, cheaper VRAM).
- ID‑buffer based picking (async, 1‑frame latency, robust for dense scenes).

Render‑graph pass toggles:

- `void set_pass_enabled(const std::string &passName, bool enabled);`
- `bool get_pass_enabled(const std::string &passName) const;`

This writes into `VulkanEngine::_rgPassToggles` and is applied during RenderGraph compilation. It allows you to permanently disable or enable named passes (e.g. `"ShadowMap[0]"`, `"FXAA"`, `"SSR"`) from game code, not just via the debug UI.

---

## Input System

Header: `src/core/input/input_system.h`
Docs: `docs/InputSystem.md`

The engine provides a unified input abstraction layer that wraps SDL2 events. Access it via `VulkanEngine::input()` or store a reference during initialization.

### Polled State (Recommended for Games)

Query current keyboard/mouse state each frame:

```cpp
void Game::update(VulkanEngine& engine)
{
    const InputState& input = engine.input().state();

    // Movement (held keys)
    glm::vec3 move{0.0f};
    if (input.key_down(Key::W)) move.z -= 1.0f;
    if (input.key_down(Key::S)) move.z += 1.0f;
    if (input.key_down(Key::A)) move.x -= 1.0f;
    if (input.key_down(Key::D)) move.x += 1.0f;

    // Sprint modifier
    float speed = input.modifiers().shift ? 10.0f : 5.0f;
    player.move(move * speed * dt);

    // Fire on left click (just pressed this frame)
    if (input.mouse_pressed(MouseButton::Left))
    {
        player.fire();
    }

    // Toggle menu on Escape (just pressed)
    if (input.key_pressed(Key::Escape))
    {
        ui.toggle_menu();
    }

    // Mouse look (right button held)
    if (input.mouse_down(MouseButton::Right))
    {
        engine.input().set_cursor_mode(CursorMode::Relative);
        glm::vec2 delta = input.mouse_delta();
        camera.rotate(delta.x * 0.1f, delta.y * 0.1f);
    }
    else
    {
        engine.input().set_cursor_mode(CursorMode::Normal);
    }

    // Scroll wheel for zoom
    float scroll = input.wheel_delta().y;
    if (scroll != 0.0f)
    {
        camera.zoom(scroll * 0.5f);
    }
}
```

### Key State Types

- `key_down(Key)` / `mouse_down(MouseButton)` — true while held (continuous actions)
- `key_pressed(Key)` / `mouse_pressed(MouseButton)` — true only on the frame it was pressed (one-shot)
- `key_released(Key)` / `mouse_released(MouseButton)` — true only on the frame it was released

### Available Keys

Letters: `Key::A` through `Key::Z`
Numbers: `Key::Num0` through `Key::Num9`
Special: `Key::Enter`, `Key::Escape`, `Key::Space`, `Key::Tab`, `Key::Backspace`
Modifiers: `Key::LeftShift`, `Key::LeftCtrl`, `Key::LeftAlt`, `Key::LeftSuper` (and Right variants)

### Mouse Buttons

`MouseButton::Left`, `MouseButton::Middle`, `MouseButton::Right`, `MouseButton::X1`, `MouseButton::X2`

### Cursor Modes

```cpp
enum class CursorMode : uint8_t
{
    Normal = 0,   // Default visible cursor
    Hidden = 1,   // Hidden but not captured
    Relative = 2, // FPS-style: hidden + captured, only delta matters
};

// Set via:
engine.input().set_cursor_mode(CursorMode::Relative);
```

### Mouse Position and Motion

```cpp
glm::vec2 pos   = input.mouse_position(); // Screen coordinates (pixels)
glm::vec2 delta = input.mouse_delta();    // Frame motion delta
glm::vec2 wheel = input.wheel_delta();    // Scroll wheel delta
```

### Modifier Keys

```cpp
InputModifiers mods = input.modifiers();
if (mods.ctrl && input.key_pressed(Key::S))
{
    save_game();
}
```

### Event-Driven Access (For UI)

For text input or other event-driven needs:

```cpp
for (const InputEvent& ev : engine.input().events())
{
    if (ev.type == InputEvent::Type::KeyDown)
    {
        if (ev.key == Key::Backspace)
        {
            text_field.delete_char();
        }
    }
    else if (ev.type == InputEvent::Type::MouseWheel)
    {
        scroll_view.scroll(ev.wheel_delta.y);
    }
}
```

### Window State

```cpp
if (engine.input().quit_requested())
{
    // Handle window close
}

if (engine.input().window_minimized())
{
    // Skip heavy rendering
}
```

---

## VulkanEngine Helpers

Header: `src/core/engine.h`

- Lifecycle
  - `void init()` – initialize SDL, device, managers, scene, render graph.
  - `void run()` – main loop: handles events, updates scene, builds Render Graph, submits frames.
  - `void cleanup()` – destroy managers and GPU resources.

- GLTF spawn helper
  - `bool addGLTFInstance(const std::string &instanceName, const std::string &modelRelativePath, const glm::mat4 &transform = glm::mat4(1.f));`
    - Loads a glTF from `assets/models/...` (via `AssetManager`) and registers it as a runtime scene instance.
    - `instanceName` becomes the logical name used by `SceneManager` (e.g. for picking and animation control).

- Picking access
  - `struct PickInfo` (nested in `VulkanEngine`)
    - `MeshAsset *mesh`, `LoadedGLTF *scene`, `Node *node`
    - `RenderObject::OwnerType ownerType` (e.g. `StaticGLTF`, `GLTFInstance`, `MeshInstance`)
    - `std::string ownerName`
    - `glm::vec3 worldPos`
    - `glm::mat4 worldTransform`
    - `uint32_t indexCount`, `firstIndex`, `surfaceIndex`
    - `bool valid`
  - `const PickInfo &get_last_pick() const`
    - Returns the last click selection result.
    - Filled by the engine from either CPU ray picking or ID‑buffer picking depending on `_useIdBufferPicking`.
    - Typical usage:
      - On mouse‑up in your game layer, read `engine->get_last_pick()` and, if `valid`, use `ownerName`/`worldPos` to drive selection logic.

> Note: hover picks and drag selections are also available as internal fields on `VulkanEngine` (`_hoverPick`, `_dragSelection`) and are documented in `docs/Scene.md`. You can expose additional getters if you want to rely on them from game code.

---

## Scene & Instances (Actors, Lights, Animations)

Header: `src/scene/vk_scene.h`  
Docs: `docs/Scene.md`

### Dynamic Mesh/GLTF Instances

- Mesh instances
  - `void addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh, const glm::mat4 &transform = glm::mat4(1.f), std::optional<BoundsType> boundsType = {});`
  - `bool getMeshInstanceTransform(const std::string &name, glm::mat4 &outTransform);`
  - `bool setMeshInstanceTransform(const std::string &name, const glm::mat4 &transform);`
  - `bool removeMeshInstance(const std::string &name);`
  - `void clearMeshInstances();`
  - Typical usage:
    - Spawn primitives or dynamic meshes at runtime (e.g. projectiles, props).
    - Use `setMeshInstanceTransform` every frame to move them based on game logic.

- GLTF instances (actors)
  - `void addGLTFInstance(const std::string &name, std::shared_ptr<LoadedGLTF> scene, const glm::mat4 &transform = glm::mat4(1.f));`
  - `bool getGLTFInstanceTransform(const std::string &name, glm::mat4 &outTransform);`
  - `bool setGLTFInstanceTransform(const std::string &name, const glm::mat4 &transform);`
  - `bool removeGLTFInstance(const std::string &name);`
  - `void clearGLTFInstances();`
  - Usage pattern:
    - Treat each GLTF instance as an “actor” with a name; use transforms to place characters, doors, props, etc.

### Animations (GLTF)

- Scene‑level
  - `bool setSceneAnimation(const std::string &sceneName, int animationIndex, bool resetTime = true);`
  - `bool setSceneAnimation(const std::string &sceneName, const std::string &animationName, bool resetTime = true);`
  - `bool setSceneAnimationLoop(const std::string &sceneName, bool loop);`
- Instance‑level
  - `bool setGLTFInstanceAnimation(const std::string &instanceName, int animationIndex, bool resetTime = true);`
  - `bool setGLTFInstanceAnimation(const std::string &instanceName, const std::string &animationName, bool resetTime = true);`
  - `bool setGLTFInstanceAnimationLoop(const std::string &instanceName, bool loop);`
- Notes:
  - All functions return `bool` indicating whether the named scene/instance exists.
  - Animation state is **independent per scene and per instance**:
    - Each named scene has its own `AnimationState`.
    - Each glTF instance has its own `AnimationState`, even when sharing the same `LoadedGLTF`.
  - An index `< 0` (e.g. `-1`) disables animation for that scene/instance (pose is frozen at the last evaluated state).
  - `SceneManager::update_scene()` advances each active animation state every frame using engine delta time.

### Per‑Instance Node / Joint Control (Non‑Skinned)

For rigid models and simple “joints” (e.g. flaps, doors, turrets), you can apply local‑space pose offsets to individual glTF nodes per instance:

- `bool setGLTFInstanceNodeOffset(const std::string &instanceName, const std::string &nodeName, const glm::mat4 &offset);`
- `bool clearGLTFInstanceNodeOffset(const std::string &instanceName, const std::string &nodeName);`
- `void clearGLTFInstanceNodeOffsets(const std::string &instanceName);`

Typical usage:

- Use glTF animation for the base motion (e.g. gear deployment).
- Layer game‑driven offsets on top for per‑instance control:

  ```cpp
  // Rotate a control surface on one aircraft instance
  glm::mat4 offset =
      glm::rotate(glm::mat4(1.f),
                  glm::radians(aileronDegrees),
                  glm::vec3(1.f, 0.f, 0.f));
  sceneMgr->setGLTFInstanceNodeOffset("plane01", "LeftAileron", offset);
  ```

### Point Lights

- Struct:
  - `struct PointLight { glm::vec3 position; float radius; glm::vec3 color; float intensity; };`
- API:
  - `void addPointLight(const PointLight &light);`
  - `void clearPointLights();`
  - `size_t getPointLightCount() const;`
  - `bool getPointLight(size_t index, PointLight &outLight) const;`
  - `bool setPointLight(size_t index, const PointLight &light);`
  - `bool removePointLight(size_t index);`
- Typical usage:
  - On level load, add all static lights.
  - At runtime, animate or toggle lights based on gameplay events (e.g. explosions, flickering lamps).

---

## Picking System

Header: `src/core/picking/picking_system.h`
Docs: `docs/Picking.md`

The engine provides a unified picking system that handles click selection, hover detection, and drag-box multi-select. Access it via `VulkanEngine::picking()`.

### Accessing Pick Results

```cpp
void Game::handle_interaction(VulkanEngine& engine)
{
    PickingSystem& picking = engine.picking();

    // Last click selection
    const PickingSystem::PickInfo& pick = picking.last_pick();
    if (pick.valid)
    {
        fmt::println("Selected: {}", pick.ownerName);
        interact_with(pick.ownerName, pick.worldPos);
    }

    // Current hover (for tooltips)
    const PickingSystem::PickInfo& hover = picking.hover_pick();
    if (hover.valid)
    {
        show_tooltip(hover.ownerName);
    }

    // Drag-box multi-select
    for (const auto& sel : picking.drag_selection())
    {
        if (sel.valid)
        {
            add_to_selection(sel.ownerName);
        }
    }
}
```

### PickInfo Structure

```cpp
struct PickInfo
{
    MeshAsset *mesh;                     // Source mesh
    LoadedGLTF *scene;                   // Source glTF
    Node *node;                          // glTF node
    RenderObject::OwnerType ownerType;   // StaticGLTF, GLTFInstance, MeshInstance
    std::string ownerName;               // Logical name (e.g., "player")
    WorldVec3 worldPos;                  // Hit position (double-precision)
    glm::mat4 worldTransform;            // Object transform
    uint32_t indexCount, firstIndex;     // Picked surface indices
    uint32_t surfaceIndex;               // Surface index in mesh
    bool valid;                          // True if hit something
};
```

### Picking Modes

```cpp
// CPU ray picking (default) - immediate, BVH-accelerated
picking.set_use_id_buffer_picking(false);

// ID-buffer picking - pixel-perfect, 1-frame latency
picking.set_use_id_buffer_picking(true);
```

### Owner Types

```cpp
enum class OwnerType
{
    None,
    StaticGLTF,    // Loaded via loadScene()
    GLTFInstance,  // Runtime glTF instance
    MeshInstance,  // Runtime mesh instance
};
```

---

## ImGui System

Header: `src/core/ui/imgui_system.h`
Docs: `docs/ImGuiSystem.md`

The engine integrates Dear ImGui for debug UI and editor tools. Access it via `VulkanEngine::imgui()`.

### Adding Custom UI

```cpp
void Game::init(VulkanEngine& engine)
{
    // Register your UI callback
    engine.imgui().add_draw_callback([this]() {
        draw_game_ui();
    });
}

void Game::draw_game_ui()
{
    if (ImGui::Begin("Game HUD"))
    {
        ImGui::Text("Score: %d", _score);
        ImGui::Text("Health: %.0f%%", _health * 100.0f);

        if (ImGui::Button("Pause"))
        {
            toggle_pause();
        }
    }
    ImGui::End();
}
```

### Input Capture

Always check ImGui input capture before processing game input:

```cpp
void Game::update(VulkanEngine& engine)
{
    // Skip game mouse input when ImGui wants it
    if (!engine.imgui().want_capture_mouse())
    {
        handle_mouse_input();
    }

    // Skip game keyboard input when ImGui wants it
    if (!engine.imgui().want_capture_keyboard())
    {
        handle_keyboard_input();
    }
}
```

### Built-in Debug UI

The engine provides comprehensive debug widgets in `src/core/engine_ui.cpp`:

- **Window Tab**: Monitor selection, fullscreen modes, HiDPI info
- **Stats Tab**: Frame time, FPS, draw calls, triangle count
- **Scene Tab**: Instance spawning, point light editor, ImGuizmo gizmos
- **Render Graph Tab**: Pass toggles, resource tracking
- **Texture Streaming Tab**: VRAM budget, cache stats
- **Shadows Tab**: Shadow mode, cascade visualization
- **Post Processing Tab**: Tonemapping, bloom, FXAA, SSR settings

### ImGuizmo Integration

For 3D transform gizmos on selected objects:

```cpp
#include "ImGuizmo.h"

void draw_gizmo(const PickingSystem::PickInfo& pick,
                const glm::mat4& view, const glm::mat4& proj)
{
    if (!pick.valid) return;

    glm::mat4 transform = pick.worldTransform;

    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetDrawlist();

    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

    if (ImGuizmo::Manipulate(glm::value_ptr(view),
                             glm::value_ptr(proj),
                             ImGuizmo::TRANSLATE,
                             ImGuizmo::WORLD,
                             glm::value_ptr(transform)))
    {
        // Apply transform to the picked object
        scene->setGLTFInstanceTransform(pick.ownerName, transform);
    }
}
```
