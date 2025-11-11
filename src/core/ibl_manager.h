#pragma once

#include <core/vk_types.h>
#include <string>

class EngineContext;

struct IBLPaths
{
    std::string specularCube; // .ktx2 (GPU-ready BC6H or R16G16B16A16)
    std::string diffuseCube;  // .ktx2
    std::string brdfLut2D;    // .ktx2 (BC5 RG UNORM or similar)
};

// Minimal IBL asset owner with optional residency control.
class IBLManager
{
public:
    void init(EngineContext* ctx) { _ctx = ctx; }

    // Load all three textures. Returns true when specular+diffuse (and optional LUT) are resident.
    bool load(const IBLPaths& paths);

    // Release GPU memory and patch to fallbacks handled by the caller.
    void unload();

    bool resident() const { return _spec.image != VK_NULL_HANDLE || _diff.image != VK_NULL_HANDLE; }

    AllocatedImage specular() const { return _spec; }
    AllocatedImage diffuse()  const { return _diff; }
    AllocatedImage brdf()     const { return _brdf; }

private:
    EngineContext* _ctx{nullptr};
    AllocatedImage _spec{};
    AllocatedImage _diff{};
    AllocatedImage _brdf{};
};

