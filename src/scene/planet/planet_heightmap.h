#pragma once

#include <scene/planet/cubesphere.h>

#include <cstdint>
#include <string>
#include <vector>

namespace planet
{
    struct HeightMip
    {
        uint32_t width = 0;
        uint32_t height = 0;
        std::vector<uint8_t> texels; // R8 values, row-major
    };

    struct HeightFace
    {
        uint32_t width = 0;
        uint32_t height = 0;
        std::vector<uint8_t> texels; // R8 values, row-major
        std::vector<HeightMip> mips; // Downsampled mip levels starting at level 1
    };

    // Load a KTX2 BC4_UNORM height map and decode it to R8 texels.
    // Returns false on failure (file not found, wrong format, etc.).
    bool load_heightmap_bc4(const std::string &path, HeightFace &out_face);

    // Sample a height face with bilinear interpolation.
    // u, v are in [0..1] range (clamped internally).
    // Returns a normalized height value in [0..1]. mip_level=0 samples the base level.
    float sample_height(const HeightFace &face, float u, float v, uint32_t mip_level = 0);

    // Pick a CPU mip level whose texel spacing roughly matches a patch vertex spacing.
    uint32_t choose_height_mip_level(const HeightFace &face, uint32_t patch_level, uint32_t patch_resolution);

} // namespace planet
