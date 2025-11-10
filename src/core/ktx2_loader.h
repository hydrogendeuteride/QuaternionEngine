#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <vulkan/vulkan.h>

// Minimal KTX2 reader for 2D textures with pre-transcoded BCn payloads.
// Supports: faceCount==1, layerCount==1, supercompression==0 (none).
// Extracts vkFormat, base width/height, mip count, and per-level byte ranges.

struct KTX2LevelInfo {
    uint64_t offset{0};
    uint64_t length{0};
    uint32_t width{0};
    uint32_t height{0};
};

struct KTX2Image {
    VkFormat format{VK_FORMAT_UNDEFINED};
    uint32_t width{0};
    uint32_t height{0};
    uint32_t mipLevels{0};
    uint32_t faceCount{0};
    uint32_t layerCount{0};
    uint32_t supercompression{0};
    std::vector<uint8_t> data;             // full file payload to back staging buffer copies
    std::vector<KTX2LevelInfo> levels;     // level 0..mipLevels-1
};

// Parse from memory. Returns true on success; on failure, 'err' (if provided) describes the issue.
bool parse_ktx2(const uint8_t* bytes, size_t size, KTX2Image& out, std::string* err = nullptr);

