#include "ktx2_loader.h"

#include <cstring>
#include <algorithm>
#include <fmt/core.h>

namespace {

struct KTX2Header {
    uint8_t  magic[12];
    uint32_t vkFormat;
    uint32_t typeSize;
    uint32_t pixelWidth;
    uint32_t pixelHeight;
    uint32_t pixelDepth;
    uint32_t layerCount;
    uint32_t faceCount;
    uint32_t levelCount;
    uint32_t supercompressionScheme;
    uint64_t dfdByteOffset;
    uint32_t dfdByteLength;
    uint64_t kvdByteOffset;
    uint32_t kvdByteLength;
    uint64_t sgdByteOffset;
    uint64_t sgdByteLength;
};

struct KTX2LevelIndexEntry {
    uint64_t byteOffset;
    uint64_t byteLength;
    uint64_t uncompressedByteLength;
};

constexpr uint8_t KTX2_MAGIC[12] = {
    0xAB,'K','T','X',' ', '2','0', 0xBB, 0x0D, 0x0A, 0x1A, 0x0A
};

template <typename T>
static inline bool read_into(const uint8_t* base, size_t size, size_t off, T& out)
{
    if (off + sizeof(T) > size) return false;
    std::memcpy(&out, base + off, sizeof(T));
    return true;
}

} // namespace

static inline uint32_t bc_block_bytes(VkFormat fmt)
{
    switch (fmt)
    {
        case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
        case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
        case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
        case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
        case VK_FORMAT_BC4_UNORM_BLOCK:
        case VK_FORMAT_BC4_SNORM_BLOCK:
            return 8u;
        case VK_FORMAT_BC2_UNORM_BLOCK:
        case VK_FORMAT_BC2_SRGB_BLOCK:
        case VK_FORMAT_BC3_UNORM_BLOCK:
        case VK_FORMAT_BC3_SRGB_BLOCK:
        case VK_FORMAT_BC5_UNORM_BLOCK:
        case VK_FORMAT_BC5_SNORM_BLOCK:
        case VK_FORMAT_BC6H_UFLOAT_BLOCK:
        case VK_FORMAT_BC6H_SFLOAT_BLOCK:
        case VK_FORMAT_BC7_UNORM_BLOCK:
        case VK_FORMAT_BC7_SRGB_BLOCK:
            return 16u;
        default: return 0u;
    }
}

bool parse_ktx2(const uint8_t* bytes, size_t size, KTX2Image& out, std::string* err)
{
    if (!bytes || size < sizeof(KTX2Header))
    {
        if (err) *err = "KTX2: buffer too small";
        return false;
    }

    KTX2Header hdr{};
    if (!read_into(bytes, size, 0, hdr))
    {
        if (err) *err = "KTX2: failed to read header";
        return false;
    }
    if (std::memcmp(hdr.magic, KTX2_MAGIC, sizeof(KTX2_MAGIC)) != 0)
    {
        if (err) *err = "KTX2: bad magic";
        return false;
    }

    if (hdr.levelCount == 0 || hdr.pixelWidth == 0 || hdr.pixelHeight == 0)
    {
        if (err) *err = "KTX2: invalid dimensions or levels";
        return false;
    }
    if (hdr.layerCount > 1 || hdr.faceCount != 1)
    {
        if (err) *err = "KTX2: only 2D, single-face, single-layer supported";
        return false;
    }
    if (hdr.supercompressionScheme != 0)
    {
        if (err) *err = "KTX2: supercompressed payloads not supported";
        return false;
    }
    if (hdr.vkFormat == 0)
    {
        if (err) *err = "KTX2: vkFormat undefined (expected pre-transcoded BCn)";
        return false;
    }

    // Level index immediately follows header in KTX2 layout.
    const size_t levelIndexSize   = sizeof(KTX2LevelIndexEntry) * static_cast<size_t>(hdr.levelCount);
    auto align8 = [](uint64_t x) { return (x + 7ull) & ~7ull; };

    // Per KTX2 spec, the Level Index immediately follows the fixed-size 80-byte header.
    size_t levelIndexOffset = sizeof(KTX2Header);
    if (levelIndexOffset + levelIndexSize > size)
    {
        if (err) *err = "KTX2: truncated level index";
        return false;
    }

    std::vector<KTX2LevelIndexEntry> levels(hdr.levelCount);
    std::memcpy(levels.data(), bytes + levelIndexOffset, levelIndexSize);

    // Debug header/offsets when requested via env (VE_TEX_DEBUG=1)
    if (const char* dbg = std::getenv("VE_TEX_DEBUG"); dbg && dbg[0] == '1')
    {
        fmt::println("[KTX2] hdr: fmt={}, size={}x{} levels={} dfdOff={} dfdLen={} kvdOff={} kvdLen={} sgdOff={} sgdLen={} liOff={}",
                     (unsigned)hdr.vkFormat, hdr.pixelWidth, hdr.pixelHeight, hdr.levelCount,
                     (unsigned long long)hdr.dfdByteOffset, (unsigned)hdr.dfdByteLength,
                     (unsigned long long)hdr.kvdByteOffset, (unsigned)hdr.kvdByteLength,
                     (unsigned long long)hdr.sgdByteOffset, (unsigned long long)hdr.sgdByteLength,
                     (unsigned long long)levelIndexOffset);
        for (uint32_t i = 0; i < hdr.levelCount; ++i)
        {
            fmt::println("[KTX2] LI[{}]: offRel={} len={} uncomp={}", i,
                         (unsigned long long)levels[i].byteOffset,
                         (unsigned long long)levels[i].byteLength,
                         (unsigned long long)levels[i].uncompressedByteLength);
        }
    }

    // Compute dataStart per spec: after level index and any optional blocks, 8-byte aligned.
    uint64_t afterIndex = align8(static_cast<uint64_t>(levelIndexOffset + levelIndexSize));
    uint64_t dfdEnd = static_cast<uint64_t>(hdr.dfdByteOffset) + static_cast<uint64_t>(hdr.dfdByteLength);
    uint64_t kvdEnd = static_cast<uint64_t>(hdr.kvdByteOffset) + static_cast<uint64_t>(hdr.kvdByteLength);
    uint64_t sgdEnd = static_cast<uint64_t>(hdr.sgdByteOffset) + static_cast<uint64_t>(hdr.sgdByteLength);
    uint64_t dataStart = align8(std::max({ afterIndex, dfdEnd, kvdEnd, sgdEnd }));
    if (dataStart == 0 || dataStart > size)
    {
        if (err) *err = "KTX2: could not locate level data start";
        return false;
    }

    out = {};
    out.format = static_cast<VkFormat>(hdr.vkFormat);
    out.width = hdr.pixelWidth;
    out.height = hdr.pixelHeight;
    out.mipLevels = hdr.levelCount;
    out.faceCount = hdr.faceCount;
    out.layerCount = hdr.layerCount;
    out.supercompression = hdr.supercompressionScheme;
    out.data.assign(bytes, bytes + size); // retain backing store for staging copies
    out.levels.resize(hdr.levelCount);
    // Map entries to mip levels: assign largest byteLength to mip 0, next to mip 1, etc.
    std::vector<uint32_t> order(hdr.levelCount);
    for (uint32_t i = 0; i < hdr.levelCount; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b){ return levels[a].byteLength > levels[b].byteLength; });

    out.levels.resize(hdr.levelCount);
    const uint32_t blockBytes = bc_block_bytes(static_cast<VkFormat>(hdr.vkFormat));
    for (uint32_t mip = 0; mip < hdr.levelCount; ++mip)
    {
        const auto &li = levels[ order[mip] ];
        const uint32_t w = std::max(1u, hdr.pixelWidth  >> mip);
        const uint32_t h = std::max(1u, hdr.pixelHeight >> mip);

        if (blockBytes)
        {
            uint64_t bx = (w + 3u) / 4u;
            uint64_t by = (h + 3u) / 4u;
            uint64_t expected = bx * by * blockBytes;
            if (li.byteLength < expected)
            {
                if (err)
                {
                    char buf[256];
                    snprintf(buf, sizeof(buf),
                             "KTX2: level length smaller than expected footprint (mip=%u fmt=%u w=%u h=%u blocks=%llux%llu blockBytes=%u expected=%llu got=%llu)",
                             mip, (unsigned)hdr.vkFormat, w, h,
                             (unsigned long long)bx, (unsigned long long)by,
                             blockBytes, (unsigned long long)expected, (unsigned long long)li.byteLength);
                    *err = buf;
                }
                return false;
            }
        }

        uint64_t absOff = dataStart + li.byteOffset;
        if (absOff + li.byteLength > size)
        {
            if (err) *err = "KTX2: level range out of bounds";
            return false;
        }
        out.levels[mip] = KTX2LevelInfo{ absOff, li.byteLength, w, h };
    }
    return true;
}
