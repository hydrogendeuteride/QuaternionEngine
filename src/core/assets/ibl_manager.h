#pragma once

#include <core/types.h>
#include <string>

class TextureCache;

class EngineContext;

struct PreparedIBLData;

struct IBLPaths
{
    std::string specularCube; // .ktx2 (GPU-ready BC6H or R16G16B16A16)
    std::string diffuseCube; // .ktx2
    std::string brdfLut2D; // .ktx2 (BC5 RG UNORM or similar)
    // Optional separate background environment map (2D equirect .ktx2).
    // When empty, the IBL system falls back to using specularCube for the background.
    std::string background2D;
};

class IBLManager
{
public:
    IBLManager() = default;
    ~IBLManager();

    void init(EngineContext *ctx);

    void set_texture_cache(TextureCache *cache) { _cache = cache; }

    // Load all three textures. Returns true when specular+diffuse (and optional LUT) are resident.
    bool load(const IBLPaths &paths);

    // Asynchronous IBL load:
    // - Performs KTX2 file I/O and SH bake on a background thread.
    // - GPU image creation and SH upload are deferred to pump_async() on the main thread.
    // Returns false if the job could not be queued.
    bool load_async(const IBLPaths &paths);

    struct AsyncResult
    {
        // True when an async job finished since the last pump_async() call.
        bool completed{false};
        // True when the finished job successfully produced new GPU IBL resources.
        bool success{false};
    };

    // Main-thread integration: if a completed async job is pending, destroy the
    // previous IBL images/SH and upload the new ones. Must be called only when
    // the GPU is idle for the previous frame.
    AsyncResult pump_async();

    // Release GPU memory and patch to fallbacks handled by the caller.
    void unload();

    bool resident() const { return _spec.image != VK_NULL_HANDLE || _diff.image != VK_NULL_HANDLE; }

    AllocatedImage specular() const { return _spec; }
    AllocatedImage diffuse() const { return _diff; }
    AllocatedImage brdf() const { return _brdf; }
    // Background environment texture used by the background pass.
    // May alias specular() when a dedicated background is not provided.
    AllocatedImage background() const { return _background; }
    AllocatedBuffer shBuffer() const { return _shBuffer; }
    bool hasSH() const { return _shBuffer.buffer != VK_NULL_HANDLE; }

    // Descriptor set layout used by shaders (set=3)
    VkDescriptorSetLayout descriptorLayout() const { return _iblSetLayout; }

    // Build descriptor set layout without loading images (for early pipeline creation)
    bool ensureLayout();

private:
    EngineContext *_ctx{nullptr};
    TextureCache *_cache{nullptr};
    AllocatedImage _spec{};
    AllocatedImage _diff{};
    AllocatedImage _brdf{};
    AllocatedImage _background{};
    VkDescriptorSetLayout _iblSetLayout = VK_NULL_HANDLE;
    AllocatedBuffer _shBuffer{}; // 9*vec4 coefficients (RGB in .xyz)

    struct AsyncStateData;
    AsyncStateData *_async{nullptr};

    bool commit_prepared(const PreparedIBLData &data);

    // Destroy current GPU images/SH buffer but keep descriptor layout alive.
    void destroy_images_and_sh();

    void shutdown_async();
};
