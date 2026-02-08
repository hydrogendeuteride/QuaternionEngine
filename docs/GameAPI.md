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
- Lighting (directional, point, spot).
- Volumetrics (clouds, smoke, flame).
- Particle systems.
- Debug drawing.
- Picking and render‑graph pass toggles.
- Time and statistics.

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
- `int get_texture_loads_per_frame() const;`
- `void set_texture_upload_budget(size_t bytes);`
- `size_t get_texture_upload_budget() const;`
- `void set_cpu_source_budget(size_t bytes);`
- `size_t get_cpu_source_budget() const;`
- `void set_max_upload_dimension(uint32_t dim);`
- `uint32_t get_max_upload_dimension() const;`
- `void set_keep_source_bytes(bool keep);`
- `bool get_keep_source_bytes() const;`
- `void evict_textures_to_budget();`

At a lower level, `VulkanEngine::query_texture_budget_bytes()` computes a conservative per‑frame texture budget using VMA heap info and constants in `src/core/config.h`:

- `kTextureBudgetFraction` – fraction of total device‑local VRAM reserved for streamed textures (default `0.35`).
- `kTextureBudgetFallbackBytes` – fallback budget when memory properties are unavailable (default `512 MiB`).
- `kTextureBudgetMinBytes` – minimum budget clamp (default `128 MiB`).

To globally change how aggressive streaming can be, edit these constants in `config.h` and rebuild. Use the `GameAPI::Engine` setters for per‑scene tuning (e.g. reducing upload bandwidth on low‑end machines).

#### Texture Loading

```cpp
// Load from file (relative to assets/textures/ or absolute path)
TextureHandle load_texture(const std::string& path, const TextureLoadParams& params = {});

// Load from memory (compressed image data: PNG, JPG, KTX2, etc.)
TextureHandle load_texture_from_memory(const std::vector<uint8_t>& data, const TextureLoadParams& params = {});

// Check if texture is loaded and resident in VRAM
bool is_texture_loaded(TextureHandle handle) const;

// Get internal Vulkan image view (VkImageView) for advanced use
void* get_texture_image_view(TextureHandle handle) const;

// Pin texture to prevent automatic eviction (for UI, critical assets)
void pin_texture(TextureHandle handle);
void unpin_texture(TextureHandle handle);
bool is_texture_pinned(TextureHandle handle) const;

// Unload texture and free VRAM (optional - cache auto-manages)
void unload_texture(TextureHandle handle);

// Create ImGui descriptor set for use with ImGui::Image()
void* create_imgui_texture(TextureHandle handle, void* sampler = nullptr);
void free_imgui_texture(void* imgui_texture_id);
```

**TextureLoadParams:**
```cpp
struct TextureLoadParams
{
    bool srgb{false};                      // Use sRGB color space (true for albedo/emissive)
    bool mipmapped{true};                  // Generate mipmap chain
    TextureChannels channels{Auto};        // Channel hint (Auto, R, RG, RGBA)
    uint32_t mipLevels{0};                 // 0 = full chain, otherwise limit to N levels
};
```

**Usage Example:**
```cpp
GameAPI::Engine api(&engine);

// Load UI texture and pin it to prevent eviction
GameAPI::TextureLoadParams params;
params.srgb = true;
params.mipmapped = false;  // UI textures don't need mipmaps
TextureHandle uiTex = api.load_texture("ui/button.png", params);
api.pin_texture(uiTex);

// Create ImGui descriptor for rendering
void* imguiId = api.create_imgui_texture(uiTex);
ImGui::Image(imguiId, ImVec2(128, 64));

// Later: cleanup
api.free_imgui_texture(imguiId);
api.unpin_texture(uiTex);
```

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

### IBL (Image-Based Lighting)

**API:**
```cpp
// Load global IBL asynchronously
bool load_global_ibl(const IBLPaths& paths);

// Get/set global IBL paths (does not trigger reload)
IBLPaths get_global_ibl_paths() const;
void set_global_ibl_paths(const IBLPaths& paths);

// Add/remove local IBL volumes
size_t add_ibl_volume(const IBLVolume& volume);
size_t add_ibl_volume(const IBLVolumeD& volume);  // double-precision
bool remove_ibl_volume(size_t index);

// Get/set IBL volume properties
bool get_ibl_volume(size_t index, IBLVolume& out) const;
bool set_ibl_volume(size_t index, const IBLVolume& volume);
bool get_ibl_volume(size_t index, IBLVolumeD& out) const;  // double-precision
bool set_ibl_volume(size_t index, const IBLVolumeD& volume);

// Query active volume
int get_active_ibl_volume() const;  // -1 = global
size_t get_ibl_volume_count() const;
void clear_ibl_volumes();
```

**Structures:**
```cpp
struct IBLPaths
{
    std::string specularCube;  // .ktx2 specular cubemap
    std::string diffuseCube;   // .ktx2 diffuse cubemap
    std::string brdfLut;       // .ktx2 BRDF lookup table
    std::string background;    // .ktx2 background (optional, falls back to specular)
};

struct IBLVolume
{
    glm::vec3 center{0.0f};
    glm::vec3 halfExtents{10.0f};
    IBLPaths paths;
    bool enabled{true};
};

struct IBLVolumeD  // double-precision variant
{
    glm::dvec3 center{0.0};
    glm::vec3 halfExtents{10.0f};
    IBLPaths paths;
    bool enabled{true};
};
```

**Usage Example:**
```cpp
GameAPI::Engine api(&engine);

// Load global IBL (outdoor environment)
GameAPI::IBLPaths globalIBL;
globalIBL.specularCube = "ibl/outdoor_spec.ktx2";
globalIBL.diffuseCube = "ibl/outdoor_diff.ktx2";
globalIBL.brdfLut = "ibl/brdf_lut.ktx2";
api.load_global_ibl(globalIBL);

// Create local IBL volume for interior (overrides global when camera inside)
GameAPI::IBLVolume interior;
interior.center = glm::vec3(10.0f, 2.0f, -5.0f);
interior.halfExtents = glm::vec3(5.0f, 3.0f, 5.0f);
interior.paths.specularCube = "ibl/indoor_spec.ktx2";
interior.paths.diffuseCube = "ibl/indoor_diff.ktx2";
interior.paths.brdfLut = "ibl/brdf_lut.ktx2";
interior.enabled = true;

size_t idx = api.add_ibl_volume(interior);

// Query which IBL is active
int activeVol = api.get_active_ibl_volume();
if (activeVol == -1)
{
    // Using global IBL
}
else
{
    // Using local volume at index activeVol
}
```

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
- `void set_camera_position(const glm::dvec3 &position);`  // double-precision
- `glm::vec3 get_camera_position() const;`
- `glm::dvec3 get_camera_position_d() const;`  // double-precision
- `void set_camera_rotation(float pitchDeg, float yawDeg);`
- `void get_camera_rotation(float &pitchDeg, float &yawDeg) const;`
- `void set_camera_fov(float fovDegrees);`
- `float get_camera_fov() const;`
- `void camera_look_at(const glm::vec3 &target);`
- `void camera_look_at(const glm::dvec3 &target);`  // double-precision

These functions internally manipulate the quaternion‑based `Camera::orientation` and `position` in `SceneManager`. They respect the engine's `-Z` forward convention. Double-precision variants allow precise camera positioning in large worlds.

Render resolution scaling:

- `void set_render_scale(float scale); // 0.3–1.0`
- `float get_render_scale() const;`

This scales the internal draw extent relative to the swapchain and main HDR image sizes, trading resolution for performance.

### Picking & Pass Toggles

Picking:

- `Engine::PickResult get_last_pick() const;`
- `Engine::PickResultD get_last_pick_d() const;`  // double-precision
- `bool select_parent_of_last_pick();`
- `bool select_child_of_last_pick(size_t childIndex = 0);`
- `bool select_child_of_last_pick(const std::string &childName);`
- `void set_use_id_buffer_picking(bool use);`
- `bool get_use_id_buffer_picking() const;`

**PickResult structure:**
```cpp
struct PickResult
{
    bool valid{false};
    std::string ownerName;
    std::string nodeName;
    std::string nodeParentName;
    std::vector<std::string> nodeChildren;
    std::vector<std::string> nodePath;
    glm::vec3 worldPosition{0.0f};
};

struct PickResultD  // double-precision variant
{
    bool valid{false};
    std::string ownerName;
    std::string nodeName;
    std::string nodeParentName;
    std::vector<std::string> nodeChildren;
    std::vector<std::string> nodePath;
    glm::dvec3 worldPosition{0.0};
};
```

These mirror `VulkanEngine::get_last_pick()` and `_useIdBufferPicking`, letting you choose between:

- CPU raycast picking (immediate, cheaper VRAM).
- ID‑buffer based picking (async, 1‑frame latency, robust for dense scenes).

For glTF picks, `nodeName`/`nodeParentName`/`nodeChildren`/`nodePath` provide hierarchy metadata and the selection can be moved with `select_parent_of_last_pick()` / `select_child_of_last_pick(...)`.

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
    - `RenderObject::OwnerType ownerType` (e.g. `GLTFInstance`, `MeshInstance`)
    - `std::string ownerName`
    - `std::string nodeName`, `std::string nodeParentName`
    - `std::vector<std::string> nodeChildren`, `std::vector<std::string> nodePath`
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

### Transform Structures

The GameAPI provides both single and double-precision transform representations:

```cpp
struct Transform
{
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};

    glm::mat4 to_matrix() const;
    static Transform from_matrix(const glm::mat4& m);
};

struct TransformD  // double-precision variant for large worlds
{
    glm::dvec3 position{0.0};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};

    glm::mat4 to_matrix() const;
    static TransformD from_matrix(const glm::mat4& m);
};
```

Use `TransformD` for positioning objects in large worlds (e.g., space games, flight sims) where single-precision floating point loses sub-meter precision at large coordinates.

### GLTF Instances

**API:**
```cpp
// Add glTF model instance (path relative to assets/models/)
bool add_gltf_instance(const std::string& name,
                       const std::string& modelPath,
                       const Transform& transform = {},
                       bool preloadTextures = true);
bool add_gltf_instance(const std::string& name,
                       const std::string& modelPath,
                       const TransformD& transform,
                       bool preloadTextures = true);

// Add glTF model asynchronously (returns job ID, 0 on failure)
uint32_t add_gltf_instance_async(const std::string& name,
                                  const std::string& modelPath,
                                  const Transform& transform = {},
                                  bool preloadTextures = true);
uint32_t add_gltf_instance_async(const std::string& name,
                                  const std::string& modelPath,
                                  const TransformD& transform,
                                  bool preloadTextures = true);

// Remove glTF instance
bool remove_gltf_instance(const std::string& name);

// Get/set glTF instance transform
bool get_gltf_instance_transform(const std::string& name, Transform& out) const;
bool set_gltf_instance_transform(const std::string& name, const Transform& transform);
bool get_gltf_instance_transform(const std::string& name, TransformD& out) const;
bool set_gltf_instance_transform(const std::string& name, const TransformD& transform);

// Preload textures for an instance
void preload_instance_textures(const std::string& name);

// Clear all dynamic instances
void clear_all_instances();
```

### Primitive Mesh Instances

**API:**
```cpp
// Add primitive mesh instance
bool add_primitive_instance(const std::string& name,
                            PrimitiveType type,
                            const Transform& transform = {});
bool add_primitive_instance(const std::string& name,
                            PrimitiveType type,
                            const TransformD& transform);

// Remove mesh instance
bool remove_mesh_instance(const std::string& name);

// Get/set mesh instance transform
bool get_mesh_instance_transform(const std::string& name, Transform& out) const;
bool set_mesh_instance_transform(const std::string& name, const Transform& transform);
bool get_mesh_instance_transform(const std::string& name, TransformD& out) const;
bool set_mesh_instance_transform(const std::string& name, const TransformD& transform);
```

**Typical usage:**
- Spawn primitives or dynamic meshes at runtime (e.g. projectiles, props).
- Use `set_mesh_instance_transform` every frame to move them based on game logic.

### Textured Primitives

Spawn primitive meshes (cube, sphere, plane, capsule) with custom PBR textures at runtime.

#### PrimitiveMaterial Structure

```cpp
struct PrimitiveMaterial
{
    std::string albedoPath;        // Color/diffuse texture (relative to assets/)
    std::string metalRoughPath;    // Metallic (R) + Roughness (G) texture
    std::string normalPath;        // Tangent-space normal map
    std::string occlusionPath;     // Ambient occlusion (R channel)
    std::string emissivePath;      // Emissive map

    glm::vec4 colorFactor{1.0f};   // Base color multiplier (RGBA)
    float metallic{0.0f};          // Metallic factor (0-1)
    float roughness{0.5f};         // Roughness factor (0-1)
};
```

#### PrimitiveType Enum

```cpp
enum class PrimitiveType
{
    Cube,
    Sphere,
    Plane,
    Capsule
};
```

#### API Functions

```cpp
bool add_textured_primitive(const std::string& name,
                            PrimitiveType type,
                            const PrimitiveMaterial& material,
                            const Transform& transform = {});

bool add_textured_primitive(const std::string& name,
                            PrimitiveType type,
                            const PrimitiveMaterial& material,
                            const TransformD& transform);  // double-precision
```

#### Usage Example

```cpp
GameAPI::Engine api(&engine);

// Create material with textures
GameAPI::PrimitiveMaterial mat;
mat.albedoPath = "textures/brick_albedo.png";
mat.normalPath = "textures/brick_normal.png";
mat.metalRoughPath = "textures/brick_mro.png";  // Metallic-Roughness-Occlusion packed
mat.roughness = 0.7f;
mat.metallic = 0.0f;

// Spawn a textured cube
GameAPI::Transform transform;
transform.position = glm::vec3(0.0f, 1.0f, -5.0f);
transform.scale = glm::vec3(2.0f);

api.add_textured_primitive("my_brick_cube", GameAPI::PrimitiveType::Cube, mat, transform);

// Spawn a textured sphere with different material
GameAPI::PrimitiveMaterial metalMat;
metalMat.albedoPath = "textures/metal_albedo.png";
metalMat.normalPath = "textures/metal_normal.png";
metalMat.metallic = 1.0f;
metalMat.roughness = 0.3f;
metalMat.colorFactor = glm::vec4(0.9f, 0.9f, 1.0f, 1.0f);  // Slight blue tint

GameAPI::Transform sphereT;
sphereT.position = glm::vec3(3.0f, 1.0f, -5.0f);

api.add_textured_primitive("chrome_sphere", GameAPI::PrimitiveType::Sphere, metalMat, sphereT);
```

#### Notes

- Texture paths are relative to the `assets/` directory.
- If a texture path is empty, the engine uses default placeholder textures:
  - Albedo: error checkerboard (magenta/black)
  - Normal: flat normal (0.5, 0.5, 1.0)
  - MetalRough: white (default values from `metallic`/`roughness` factors)
  - Occlusion: white (no occlusion)
  - Emissive: black (no emission)
- Textures are loaded asynchronously via `TextureCache`; placeholders appear until upload completes.
- For non-textured primitives with solid colors, use `add_primitive_instance()` instead.

- GLTF instances (actors)
  - `void addGLTFInstance(const std::string &name, std::shared_ptr<LoadedGLTF> scene, const glm::mat4 &transform = glm::mat4(1.f));`
  - `bool getGLTFInstanceTransform(const std::string &name, glm::mat4 &outTransform);`
  - `bool setGLTFInstanceTransform(const std::string &name, const glm::mat4 &transform);`
  - `bool removeGLTFInstance(const std::string &name);`
  - `void clearGLTFInstances();`
  - Usage pattern:
    - Treat each GLTF instance as an “actor” with a name; use transforms to place characters, doors, props, etc.

### Animations (GLTF)

**API:**
```cpp
// Set animation by index for a glTF instance (-1 to disable)
bool set_instance_animation(const std::string& instanceName, int animationIndex, bool resetTime = true);

// Set animation by name for a glTF instance
bool set_instance_animation(const std::string& instanceName, const std::string& animationName, bool resetTime = true);

// Set animation looping for a glTF instance
bool set_instance_animation_loop(const std::string& instanceName, bool loop);
```

**Notes:**
- All functions return `bool` indicating whether the named instance exists.
- Animation state is **independent per instance**:
  - Each glTF instance has its own `AnimationState`, even when sharing the same `LoadedGLTF`.
- An index `< 0` (e.g. `-1`) disables animation for that instance (pose is frozen at the last evaluated state).
- `SceneManager::update_scene()` advances each active animation state every frame using engine delta time.

**Usage Example:**
```cpp
GameAPI::Engine api(&engine);

// Play walk animation by index
api.set_instance_animation("player", 0, true);  // Reset to start
api.set_instance_animation_loop("player", true);

// Switch to run animation by name
api.set_instance_animation("player", "run", true);

// Stop animation (freeze pose)
api.set_instance_animation("player", -1);
```

### Per‑Instance Node / Joint Control (Non‑Skinned)

For rigid models and simple "joints" (e.g. flaps, doors, turrets), you can apply local‑space pose offsets to individual glTF nodes per instance:

**API:**
```cpp
bool set_instance_node_offset(const std::string& instanceName, const std::string& nodeName, const glm::mat4& offset);
bool clear_instance_node_offset(const std::string& instanceName, const std::string& nodeName);
void clear_all_instance_node_offsets(const std::string& instanceName);
```

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

### Lighting - Directional (Sunlight)

- `void set_sunlight_direction(const glm::vec3& dir);`
- `glm::vec3 get_sunlight_direction() const;`
- `void set_sunlight_color(const glm::vec3& color, float intensity);`
- `glm::vec3 get_sunlight_color() const;`
- `float get_sunlight_intensity() const;`

### Lighting - Point Lights

**Structs:**
```cpp
struct PointLight
{
    glm::vec3 position{0.0f};
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
};

struct PointLightD  // double-precision variant
{
    glm::dvec3 position{0.0};
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
};
```

**API:**
- `size_t add_point_light(const PointLight &light);`
- `size_t add_point_light(const PointLightD &light);`
- `bool remove_point_light(size_t index);`
- `bool get_point_light(size_t index, PointLight &out) const;`
- `bool get_point_light(size_t index, PointLightD &out) const;`
- `bool set_point_light(size_t index, const PointLight &light);`
- `bool set_point_light(size_t index, const PointLightD &light);`
- `size_t get_point_light_count() const;`
- `void clear_point_lights();`

**Typical usage:**
- On level load, add all static lights.
- At runtime, animate or toggle lights based on gameplay events (e.g. explosions, flickering lamps).

### Lighting - Spot Lights

**Structs:**
```cpp
struct SpotLight
{
    glm::vec3 position{0.0f};
    glm::vec3 direction{0.0f, -1.0f, 0.0f};
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
    float inner_angle_deg{15.0f};
    float outer_angle_deg{25.0f};
};

struct SpotLightD  // double-precision variant
{
    glm::dvec3 position{0.0};
    glm::vec3 direction{0.0f, -1.0f, 0.0f};
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
    float inner_angle_deg{15.0f};
    float outer_angle_deg{25.0f};
};
```

**API:**
- `size_t add_spot_light(const SpotLight &light);`
- `size_t add_spot_light(const SpotLightD &light);`
- `bool remove_spot_light(size_t index);`
- `bool get_spot_light(size_t index, SpotLight &out) const;`
- `bool get_spot_light(size_t index, SpotLightD &out) const;`
- `bool set_spot_light(size_t index, const SpotLight &light);`
- `bool set_spot_light(size_t index, const SpotLightD &light);`
- `size_t get_spot_light_count() const;`
- `void clear_spot_lights();`

**Usage Example:**
```cpp
GameAPI::Engine api(&engine);

// Create a flashlight
GameAPI::SpotLight flashlight;
flashlight.position = glm::vec3(0.0f, 1.5f, 0.0f);
flashlight.direction = glm::vec3(0.0f, 0.0f, -1.0f);
flashlight.radius = 20.0f;
flashlight.color = glm::vec3(1.0f, 0.95f, 0.8f);  // Warm white
flashlight.intensity = 50.0f;
flashlight.inner_angle_deg = 10.0f;
flashlight.outer_angle_deg = 25.0f;

size_t idx = api.add_spot_light(flashlight);

// Later: update flashlight direction to follow camera
flashlight.direction = camera_forward;
api.set_spot_light(idx, flashlight);
```

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
        fmt::println("Selected: {} (node='{}')", pick.ownerName, pick.nodeName);
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

### Hierarchical glTF Navigation

```cpp
auto pick = api.get_last_pick();
if (pick.valid && !pick.nodeParentName.empty())
{
    api.select_parent_of_last_pick();
}

if (pick.valid && !pick.nodeChildren.empty())
{
    // First direct child
    api.select_child_of_last_pick();

    // Or a specific child name from pick.nodeChildren
    api.select_child_of_last_pick(pick.nodeChildren.front());
}
```

### PickInfo Structure

```cpp
struct PickInfo
{
    MeshAsset *mesh;                     // Source mesh
    LoadedGLTF *scene;                   // Source glTF
    Node *node;                          // glTF node
    RenderObject::OwnerType ownerType;   // GLTFInstance, MeshInstance
    std::string ownerName;               // Logical name (e.g., "player")
    std::string nodeName;                // Selected glTF node name
    std::string nodeParentName;          // Direct parent node
    std::vector<std::string> nodeChildren; // Direct child nodes
    std::vector<std::string> nodePath;   // Root-to-node path
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

---

## Time and Statistics

Header: `src/core/game_api.h`

### Delta Time

```cpp
// Get delta time in seconds for the current frame (clamped to 0.0-0.1)
float get_delta_time() const;
```

Use this for frame-rate independent movement and animation.

### Engine Statistics

```cpp
struct Stats
{
    float frametime{0.0f};       // ms
    float drawTime{0.0f};        // ms
    float sceneUpdateTime{0.0f}; // ms
    int triangleCount{0};
    int drawCallCount{0};
};

Stats get_stats() const;
```

**Usage Example:**
```cpp
GameAPI::Engine api(&engine);

// Frame-rate independent movement
float dt = api.get_delta_time();
player_position += velocity * dt;

// Display performance stats
GameAPI::Stats stats = api.get_stats();
fmt::println("FPS: {:.1f} | Tris: {} | Draws: {}",
             1000.0f / stats.frametime,
             stats.triangleCount,
             stats.drawCallCount);
```

---

## Volumetrics (Clouds, Smoke, Flame)

Header: `src/core/game_api.h`

The engine supports GPU-based voxel volumetric rendering for clouds, smoke, and flame effects. Up to 4 independent volumes can be active simultaneously.

### API

```cpp
// Enable/disable volumetrics system
void set_volumetrics_enabled(bool enabled);
bool get_volumetrics_enabled() const;

// Get/set voxel volume settings by index (0-3)
bool get_voxel_volume(size_t index, VoxelVolumeSettings& out) const;
bool set_voxel_volume(size_t index, const VoxelVolumeSettings& settings);

// Get maximum number of voxel volumes
size_t get_max_voxel_volumes() const;  // Returns 4
```

### VoxelVolumeSettings Structure

```cpp
struct VoxelVolumeSettings
{
    bool enabled{false};
    VoxelVolumeType type{VoxelVolumeType::Clouds};  // Clouds, Smoke, Flame

    // Volume positioning
    bool followCameraXZ{false};     // Follow camera in XZ, offset in Y
    bool animateVoxels{true};       // Run voxel advection/update compute
    glm::vec3 volumeCenterLocal{0.0f, 2.0f, 0.0f};
    glm::vec3 volumeHalfExtents{8.0f, 8.0f, 8.0f};
    glm::vec3 volumeVelocityLocal{0.0f, 0.0f, 0.0f};  // Drift when not following camera

    // Raymarch/composite controls
    float densityScale{1.0f};
    float coverage{0.0f};        // 0..1 threshold (higher = emptier)
    float extinction{1.0f};      // Absorption/extinction scale
    int stepCount{48};           // Raymarch steps

    // Voxel grid resolution (cubic)
    uint32_t gridResolution{48};

    // Voxel animation (advection + injection) parameters
    glm::vec3 windVelocityLocal{0.0f, 2.0f, 0.0f};  // Local units/sec (buoyancy)
    float dissipation{1.25f};    // Density decay rate (1/sec)
    float noiseStrength{1.0f};   // Injection rate
    float noiseScale{8.0f};      // Noise frequency in UVW space
    float noiseSpeed{1.0f};      // Time scale for injection noise

    // Smoke/flame source in normalized volume UVW space
    glm::vec3 emitterUVW{0.5f, 0.05f, 0.5f};
    float emitterRadius{0.18f};  // Normalized (0..1)

    // Shading
    glm::vec3 albedo{1.0f, 1.0f, 1.0f};  // Scattering tint (cloud/smoke)
    float scatterStrength{1.0f};
    glm::vec3 emissionColor{1.0f, 0.6f, 0.25f};  // Flame emissive tint
    float emissionStrength{0.0f};
};
```

### Usage Example

```cpp
GameAPI::Engine api(&engine);

// Enable volumetrics
api.set_volumetrics_enabled(true);

// Create a flame effect
GameAPI::VoxelVolumeSettings flame;
flame.enabled = true;
flame.type = GameAPI::VoxelVolumeType::Flame;
flame.volumeCenterLocal = glm::vec3(0.0f, 1.0f, -5.0f);
flame.volumeHalfExtents = glm::vec3(2.0f, 3.0f, 2.0f);
flame.gridResolution = 64;
flame.densityScale = 1.5f;
flame.coverage = 0.3f;
flame.windVelocityLocal = glm::vec3(0.0f, 5.0f, 0.0f);  // Upward
flame.emitterUVW = glm::vec3(0.5f, 0.1f, 0.5f);
flame.emitterRadius = 0.2f;
flame.emissionStrength = 2.0f;
flame.emissionColor = glm::vec3(1.0f, 0.5f, 0.1f);

api.set_voxel_volume(0, flame);

// Create cloud layer that follows camera
GameAPI::VoxelVolumeSettings clouds;
clouds.enabled = true;
clouds.type = GameAPI::VoxelVolumeType::Clouds;
clouds.followCameraXZ = true;
clouds.volumeCenterLocal = glm::vec3(0.0f, 50.0f, 0.0f);  // Offset in Y
clouds.volumeHalfExtents = glm::vec3(100.0f, 20.0f, 100.0f);
clouds.gridResolution = 128;
clouds.densityScale = 0.8f;
clouds.coverage = 0.5f;
clouds.albedo = glm::vec3(0.9f, 0.95f, 1.0f);  // Bluish tint

api.set_voxel_volume(1, clouds);
```

---

## Particle Systems

Header: `src/core/game_api.h`

GPU-accelerated particle systems with flipbook animation, soft particles, and flexible spawning.

### API

```cpp
// Create/destroy particle systems
uint32_t create_particle_system(uint32_t particle_count);
bool destroy_particle_system(uint32_t id);
bool resize_particle_system(uint32_t id, uint32_t new_count);

// Get/set particle system settings
bool get_particle_system(uint32_t id, ParticleSystem& out) const;
bool set_particle_system(uint32_t id, const ParticleSystem& system);

// Query particle systems
std::vector<uint32_t> get_particle_system_ids() const;
uint32_t get_allocated_particles() const;
uint32_t get_free_particles() const;
uint32_t get_max_particles() const;

// Preload VFX textures (e.g., "vfx/flame.ktx2")
void preload_particle_texture(const std::string& assetPath);
```

### ParticleSystem Structure

```cpp
struct ParticleSystem
{
    uint32_t id{0};
    uint32_t particleCount{0};
    bool enabled{true};
    bool reset{true};
    ParticleBlendMode blendMode{ParticleBlendMode::Additive};  // Additive or Alpha
    ParticleParams params{};

    // Asset-relative texture paths (e.g., "vfx/flame.ktx2")
    std::string flipbookTexture{"vfx/flame.ktx2"};
    std::string noiseTexture{"vfx/simplex.ktx2"};
};

struct ParticleParams
{
    glm::vec3 emitterPosLocal{0.0f, 0.0f, 0.0f};
    float spawnRadius{0.1f};
    glm::vec3 emitterDirLocal{0.0f, 1.0f, 0.0f};
    float coneAngleDegrees{20.0f};

    float minSpeed{2.0f};
    float maxSpeed{8.0f};
    float minLife{0.5f};
    float maxLife{1.5f};
    float minSize{0.05f};
    float maxSize{0.15f};

    float drag{1.0f};
    float gravity{0.0f};  // Positive pulls down -Y in local space

    glm::vec4 color{1.0f, 0.5f, 0.1f, 1.0f};

    // Soft particles (fade near opaque geometry)
    float softDepthDistance{0.15f};

    // Flipbook animation (atlas layout)
    uint32_t flipbookCols{16};
    uint32_t flipbookRows{4};
    float flipbookFps{30.0f};
    float flipbookIntensity{1.0f};

    // Noise UV distortion
    float noiseScale{6.0f};
    float noiseStrength{0.05f};
    glm::vec2 noiseScroll{0.0f, 0.0f};
};
```

### Usage Example

```cpp
GameAPI::Engine api(&engine);

// Create fire particle system
uint32_t fireId = api.create_particle_system(4096);

GameAPI::ParticleSystem fire;
fire.id = fireId;
fire.particleCount = 4096;
fire.enabled = true;
fire.blendMode = GameAPI::ParticleBlendMode::Additive;
fire.flipbookTexture = "vfx/flame.ktx2";
fire.noiseTexture = "vfx/simplex.ktx2";

fire.params.emitterPosLocal = glm::vec3(0.0f, 0.0f, -5.0f);
fire.params.spawnRadius = 0.5f;
fire.params.emitterDirLocal = glm::vec3(0.0f, 1.0f, 0.0f);
fire.params.coneAngleDegrees = 15.0f;
fire.params.minSpeed = 2.0f;
fire.params.maxSpeed = 4.0f;
fire.params.minLife = 0.8f;
fire.params.maxLife = 1.5f;
fire.params.minSize = 0.3f;
fire.params.maxSize = 0.6f;
fire.params.gravity = -2.0f;  // Upward buoyancy
fire.params.color = glm::vec4(1.0f, 0.7f, 0.3f, 1.0f);
fire.params.flipbookCols = 16;
fire.params.flipbookRows = 4;
fire.params.flipbookFps = 24.0f;

api.set_particle_system(fireId, fire);

// Later: move emitter to follow player
fire.params.emitterPosLocal = player_position;
api.set_particle_system(fireId, fire);

// Reset particles (trigger burst)
fire.reset = true;
api.set_particle_system(fireId, fire);
```

---

## Debug Drawing

Header: `src/core/game_api.h`

Runtime debug visualization for primitives (lines, spheres, boxes, etc.) with optional depth testing and duration.

### Settings API

```cpp
// Enable/disable debug drawing system
void set_debug_draw_enabled(bool enabled);
bool get_debug_draw_enabled() const;

// Control which debug layers are visible (bitmask)
void set_debug_layer_mask(uint32_t mask);
uint32_t get_debug_layer_mask() const;

// Show/hide depth-tested primitives
void set_debug_show_depth_tested(bool show);
bool get_debug_show_depth_tested() const;

// Show/hide overlay (always-on-top) primitives
void set_debug_show_overlay(bool show);
bool get_debug_show_overlay() const;

// Set tessellation quality (segments for circles/spheres)
void set_debug_segments(int segments);
int get_debug_segments() const;

// Clear all debug draw commands
void debug_draw_clear();
```

### Drawing API

All drawing functions support both single and double-precision variants:

```cpp
// Line
void debug_draw_line(const glm::vec3& a, const glm::vec3& b,
                     const glm::vec4& color = glm::vec4(1.0f),
                     float duration_seconds = 0.0f,
                     bool depth_tested = true);
void debug_draw_line(const glm::dvec3& a, const glm::dvec3& b, ...);

// Ray (origin + direction + length)
void debug_draw_ray(const glm::vec3& origin, const glm::vec3& direction, float length,
                    const glm::vec4& color = glm::vec4(1.0f),
                    float duration_seconds = 0.0f,
                    bool depth_tested = true);
void debug_draw_ray(const glm::dvec3& origin, const glm::dvec3& direction, double length, ...);

// AABB (axis-aligned bounding box)
void debug_draw_aabb(const glm::vec3& center, const glm::vec3& half_extents,
                     const glm::vec4& color = glm::vec4(1.0f),
                     float duration_seconds = 0.0f,
                     bool depth_tested = true);
void debug_draw_aabb(const glm::dvec3& center, const glm::vec3& half_extents, ...);

// Sphere
void debug_draw_sphere(const glm::vec3& center, float radius,
                       const glm::vec4& color = glm::vec4(1.0f),
                       float duration_seconds = 0.0f,
                       bool depth_tested = true);
void debug_draw_sphere(const glm::dvec3& center, float radius, ...);

// Capsule (line segment + radius)
void debug_draw_capsule(const glm::vec3& p0, const glm::vec3& p1, float radius,
                        const glm::vec4& color = glm::vec4(1.0f),
                        float duration_seconds = 0.0f,
                        bool depth_tested = true);
void debug_draw_capsule(const glm::dvec3& p0, const glm::dvec3& p1, float radius, ...);

// Circle (center + normal + radius)
void debug_draw_circle(const glm::vec3& center, const glm::vec3& normal, float radius,
                       const glm::vec4& color = glm::vec4(1.0f),
                       float duration_seconds = 0.0f,
                       bool depth_tested = true);
void debug_draw_circle(const glm::dvec3& center, const glm::dvec3& normal, float radius, ...);

// Cone (apex + direction + length + angle)
void debug_draw_cone(const glm::vec3& apex, const glm::vec3& direction,
                     float length, float angle_degrees,
                     const glm::vec4& color = glm::vec4(1.0f),
                     float duration_seconds = 0.0f,
                     bool depth_tested = true);
void debug_draw_cone(const glm::dvec3& apex, const glm::dvec3& direction,
                     float length, float angle_degrees, ...);
```

### Usage Example

```cpp
GameAPI::Engine api(&engine);

// Enable debug drawing
api.set_debug_draw_enabled(true);
api.set_debug_segments(32);  // Smooth circles/spheres

// Visualize player bounds (persistent, depth-tested)
api.debug_draw_aabb(player_pos, glm::vec3(0.5f, 1.0f, 0.5f),
                    glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),
                    0.0f,    // Duration 0 = single frame
                    true);   // Depth tested

// Visualize raycast (red ray, always on top, 2 seconds)
api.debug_draw_ray(ray_origin, ray_dir, 100.0f,
                   glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
                   2.0f,     // Show for 2 seconds
                   false);   // Always on top

// Visualize trigger volume (transparent sphere)
api.debug_draw_sphere(trigger_pos, trigger_radius,
                      glm::vec4(1.0f, 1.0f, 0.0f, 0.3f),  // Yellow, 30% alpha
                      0.0f,
                      true);

// Visualize spot light cone
api.debug_draw_cone(light_pos, light_dir, light_radius, light_angle_deg,
                    glm::vec4(1.0f, 0.9f, 0.7f, 0.5f),
                    0.0f,
                    true);

// One-shot clear (useful for clearing persistent debug viz)
api.debug_draw_clear();
```

**Notes:**
- `duration_seconds = 0.0f`: Draw for a single frame (re-submit each frame for persistent viz).
- `duration_seconds > 0.0f`: Draw for N seconds, then automatically expire.
- `depth_tested = true`: Primitive is occluded by scene geometry.
- `depth_tested = false`: Always on top (overlay mode).
- All primitives support alpha blending via the color's alpha channel.
