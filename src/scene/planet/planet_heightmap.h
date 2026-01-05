#pragma once

#include <scene/planet/cubesphere.h>

#include <cstdint>
#include <string>
#include <vector>

namespace planet
{
    struct HeightFace
    {
        uint32_t width = 0;
        uint32_t height = 0;
        std::vector<uint8_t> texels; // R8 values, row-major
    };

    // Load a KTX2 BC4_UNORM height map and decode it to R8 texels.
    // Returns false on failure (file not found, wrong format, etc.).
    bool load_heightmap_bc4(const std::string &path, HeightFace &out_face);

    // Sample a height face with bilinear interpolation.
    // u, v are in [0..1] range (clamped internally).
    // Returns a normalized height value in [0..1].
    float sample_height(const HeightFace &face, float u, float v);

} // namespace planet
