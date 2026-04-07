#pragma once

#include <scene/planet/cubesphere.h>

#include <array>
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

    struct HeightVarianceMip
    {
        uint32_t width = 0;
        uint32_t height = 0;
        std::vector<float> texels; // normalized variance [0..1], row-major
    };

    struct HeightFace
    {
        uint32_t width = 0;
        uint32_t height = 0;
        std::vector<uint8_t> texels; // R8 values, row-major
        std::vector<HeightMip> mips; // Mean/downsampled mip levels starting at level 1
        std::vector<HeightVarianceMip> variance_mips; // Variance mip levels matching `mips`
    };

    using HeightFaceSet = std::array<HeightFace, 6>;

    // Load a KTX2 BC4_UNORM height map and decode it to R8 texels.
    // Returns false on failure (file not found, wrong format, etc.).
    bool load_heightmap_bc4(const std::string &path, HeightFace &out_face);

    // Load all six cube faces from a directory containing
    // {px,nx,py,ny,pz,nz}.ktx2 files.
    bool load_heightmap_cube_faces_bc4(const std::string &directory_path, HeightFaceSet &out_faces);

    // Cross-state preload cache used to carry decoded terrain height maps from
    // a loading screen into PlanetSystem without re-decoding the same BC4 files.
    void retain_preloaded_heightmap_faces(const std::string &height_dir_key, HeightFaceSet faces);
    bool take_preloaded_heightmap_faces(const std::string &height_dir_key, HeightFaceSet &out_faces);
    void clear_preloaded_heightmap_faces();

    // Rebuild mean/variance mip chains for a manually prepared HeightFace.
    void rebuild_height_mips(HeightFace &face);

    // Sample a height face with bilinear interpolation.
    // u, v are in [0..1] range (clamped internally).
    // Returns a normalized height value in [0..1]. mip_level=0 samples the base level.
    float sample_height(const HeightFace &face, float u, float v, uint32_t mip_level = 0);

    // Sample normalized local height variance in [0..1]. mip_level=0 returns 0.
    float sample_height_variance(const HeightFace &face, float u, float v, uint32_t mip_level = 0);

    // Pick a CPU mip level whose texel spacing roughly matches a patch vertex spacing.
    uint32_t choose_height_mip_level(const HeightFace &face, uint32_t patch_level, uint32_t patch_resolution);

} // namespace planet
