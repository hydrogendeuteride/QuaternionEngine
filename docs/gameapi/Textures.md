# Texture Management

The Game API provides texture loading, streaming, and VRAM budget management through an LRU cache system.

## Loading Textures

### From File

```cpp
GameAPI::TextureLoadParams params;
params.srgb = true;          // Use sRGB color space for albedo/emissive
params.mipmapped = true;     // Generate mipmap chain
params.channels = GameAPI::TextureChannels::Auto;  // Auto-detect channels

TextureHandle handle = api.load_texture("textures/brick_albedo.png", params);
```

Paths are relative to `assets/textures/` or can be absolute.

### From Memory

```cpp
std::vector<uint8_t> imageData = download_texture_from_web();
TextureHandle handle = api.load_texture_from_memory(imageData, params);
```

Useful for runtime-generated or downloaded textures. Supports PNG, JPG, KTX2, etc.

## Texture Load Parameters

```cpp
struct TextureLoadParams
{
    bool srgb{false};                      // Use sRGB color space (true for albedo/emissive)
    bool mipmapped{true};                  // Generate mipmap chain
    TextureChannels channels{Auto};        // Channel hint (Auto, R, RG, RGBA)
    uint32_t mipLevels{0};                 // 0 = full chain, otherwise limit to N levels
};
```

**Channel Hints:**
- `Auto` — Detect from source (default)
- `R` — Single channel (e.g., occlusion, metallic)
- `RG` — Two channels (e.g., normal map XY)
- `RGBA` — Full color

## Querying Texture State

```cpp
// Check if texture is loaded and resident in VRAM
if (api.is_texture_loaded(handle)) {
    // Texture is ready to use
}

// Get internal Vulkan image view (for advanced use)
VkImageView view = static_cast<VkImageView>(api.get_texture_image_view(handle));
```

## Pinning Textures

Prevent automatic eviction for critical textures (UI, player character, etc.):

```cpp
// Pin texture to keep it resident
api.pin_texture(handle);

// Check if texture is pinned
if (api.is_texture_pinned(handle)) {
    // Texture won't be evicted
}

// Unpin when no longer critical
api.unpin_texture(handle);
```

Pinned textures are never removed from VRAM by LRU or budget constraints.

## Unloading Textures

```cpp
// Manually unload texture (optional - cache auto-manages)
api.unload_texture(handle);
```

Textures are reference-counted and automatically evicted by LRU when VRAM budget is exceeded, so manual unloading is rarely needed.

## VRAM Budget Management

The texture cache uses a conservative VRAM budget to prevent OOM:

```cpp
// Query current VRAM budget (bytes)
size_t budget = api.get_texture_budget();
fmt::println("Texture budget: {} MiB", budget / (1024 * 1024));
```

### Budget Configuration

The budget is calculated from VMA heap info and config constants in `src/core/config.h`:

- `kTextureBudgetFraction` — Fraction of device-local VRAM for textures (default `0.35`)
- `kTextureBudgetFallbackBytes` — Fallback when memory info unavailable (default `512 MiB`)
- `kTextureBudgetMinBytes` — Minimum budget clamp (default `128 MiB`)

### Runtime Budget Control

```cpp
// Set maximum textures loaded per frame (1-16)
api.set_texture_loads_per_frame(8);
int loadsPerFrame = api.get_texture_loads_per_frame();

// Set upload budget per frame (bytes)
api.set_texture_upload_budget(128 * 1024 * 1024);  // 128 MiB
size_t uploadBudget = api.get_texture_upload_budget();

// Set CPU source data budget (bytes)
api.set_cpu_source_budget(512 * 1024 * 1024);  // 512 MiB
size_t cpuBudget = api.get_cpu_source_budget();

// Set maximum upload dimension (clamps large textures)
api.set_max_upload_dimension(4096);
uint32_t maxDim = api.get_max_upload_dimension();

// Keep CPU source data after GPU upload (for streaming)
api.set_keep_source_bytes(true);
bool keepSource = api.get_keep_source_bytes();
```

### Force Eviction

```cpp
// Force eviction to budget after loading large assets
api.evict_textures_to_budget();
```

This immediately evicts least-recently-used textures until under budget.

## ImGui Integration

Create ImGui descriptor sets for rendering textures in UI:

```cpp
// Create descriptor set for ImGui::Image()
void* imguiId = api.create_imgui_texture(handle, sampler);
ImGui::Image(imguiId, ImVec2(256, 256));

// Free descriptor set when done
api.free_imgui_texture(imguiId);
```

The `sampler` parameter is a `VkSampler` or `nullptr` for default linear sampler.

## Complete Example

```cpp
GameAPI::Engine api(&engine);

// Load UI texture with specific settings
GameAPI::TextureLoadParams uiParams;
uiParams.srgb = true;
uiParams.mipmapped = false;  // UI doesn't need mipmaps
uiParams.channels = GameAPI::TextureChannels::RGBA;

TextureHandle uiTexture = api.load_texture("ui/button.png", uiParams);
api.pin_texture(uiTexture);  // Prevent eviction

// Create ImGui descriptor
void* imguiId = api.create_imgui_texture(uiTexture);

// In UI render loop
if (ImGui::Begin("HUD")) {
    ImGui::Image(imguiId, ImVec2(128, 64));
}
ImGui::End();

// Cleanup
api.free_imgui_texture(imguiId);
api.unpin_texture(uiTexture);
```

## Streaming Details

See also: [Texture Loading System](../TextureLoading.md) for low-level cache implementation details.
