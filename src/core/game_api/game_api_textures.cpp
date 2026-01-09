#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "core/assets/texture_cache.h"
#include "core/assets/manager.h"

#include <filesystem>

// ImGui integration for texture display
#include "imgui.h"
#include "imgui_impl_vulkan.h"

namespace GameAPI
{

// ----------------------------------------------------------------------------
// Memory / Texture Streaming
// ----------------------------------------------------------------------------

size_t Engine::get_texture_budget() const
{
    return _engine->query_texture_budget_bytes();
}

void Engine::set_texture_loads_per_frame(int count)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_max_loads_per_pump(count);
    }
}

int Engine::get_texture_loads_per_frame() const
{
    return _engine->_textureCache ? _engine->_textureCache->max_loads_per_pump() : 0;
}

void Engine::set_texture_upload_budget(size_t bytes)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_max_bytes_per_pump(bytes);
    }
}

size_t Engine::get_texture_upload_budget() const
{
    return _engine->_textureCache ? _engine->_textureCache->max_bytes_per_pump() : 0;
}

void Engine::set_cpu_source_budget(size_t bytes)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_cpu_source_budget(bytes);
    }
}

size_t Engine::get_cpu_source_budget() const
{
    return _engine->_textureCache ? _engine->_textureCache->cpu_source_budget() : 0;
}

void Engine::set_max_upload_dimension(uint32_t dim)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_max_upload_dimension(dim);
    }
}

uint32_t Engine::get_max_upload_dimension() const
{
    return _engine->_textureCache ? _engine->_textureCache->max_upload_dimension() : 0;
}

void Engine::set_keep_source_bytes(bool keep)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_keep_source_bytes(keep);
    }
}

bool Engine::get_keep_source_bytes() const
{
    return _engine->_textureCache ? _engine->_textureCache->keep_source_bytes() : false;
}

void Engine::evict_textures_to_budget()
{
    if (_engine->_textureCache)
    {
        size_t budget = _engine->query_texture_budget_bytes();
        _engine->_textureCache->evictToBudget(budget);
    }
}

// ----------------------------------------------------------------------------
// Texture Loading
// ----------------------------------------------------------------------------

TextureHandle Engine::load_texture(const std::string& path, const TextureLoadParams& params)
{
    if (!_engine || !_engine->_textureCache || path.empty())
    {
        return InvalidTexture;
    }

    // Resolve path relative to assets/textures/ if not absolute
    std::string resolvedPath = path;
    std::filesystem::path p(path);
    if (p.is_relative())
    {
        resolvedPath = _engine->_assetManager->assetPath(std::string("textures/") + path);
    }

    // Build TextureKey
    TextureCache::TextureKey key{};
    key.kind = TextureCache::TextureKey::SourceKind::FilePath;
    key.path = resolvedPath;
    key.srgb = params.srgb;
    key.mipmapped = params.mipmapped;
    key.mipClampLevels = params.mipLevels;

    // Map channel hint
    switch (params.channels)
    {
        case TextureChannels::R:
            key.channels = TextureCache::TextureKey::ChannelsHint::R;
            break;
        case TextureChannels::RG:
            key.channels = TextureCache::TextureKey::ChannelsHint::RG;
            break;
        case TextureChannels::RGBA:
            key.channels = TextureCache::TextureKey::ChannelsHint::RGBA;
            break;
        case TextureChannels::Auto:
        default:
            key.channels = TextureCache::TextureKey::ChannelsHint::Auto;
            break;
    }

    // Generate hash for deduplication
    std::string id = std::string("PATH:") + key.path + (key.srgb ? "#sRGB" : "#UNORM");
    key.hash = texcache::fnv1a64(id);

    // Use default linear sampler
    VkSampler sampler = VK_NULL_HANDLE;
    if (_engine->_context && _engine->_context->samplers)
    {
        sampler = _engine->_context->samplers->defaultLinear();
    }

    // Request texture from cache
    TextureCache::TextureHandle handle = _engine->_textureCache->request(key, sampler);
    return static_cast<TextureHandle>(handle);
}

TextureHandle Engine::load_texture_from_memory(const std::vector<uint8_t>& data,
                                                 const TextureLoadParams& params)
{
    if (!_engine || !_engine->_textureCache || data.empty())
    {
        return InvalidTexture;
    }

    // Build TextureKey from bytes
    TextureCache::TextureKey key{};
    key.kind = TextureCache::TextureKey::SourceKind::Bytes;
    key.bytes = data;
    key.srgb = params.srgb;
    key.mipmapped = params.mipmapped;
    key.mipClampLevels = params.mipLevels;

    // Map channel hint
    switch (params.channels)
    {
        case TextureChannels::R:
            key.channels = TextureCache::TextureKey::ChannelsHint::R;
            break;
        case TextureChannels::RG:
            key.channels = TextureCache::TextureKey::ChannelsHint::RG;
            break;
        case TextureChannels::RGBA:
            key.channels = TextureCache::TextureKey::ChannelsHint::RGBA;
            break;
        case TextureChannels::Auto:
        default:
            key.channels = TextureCache::TextureKey::ChannelsHint::Auto;
            break;
    }

    // Generate hash for deduplication
    uint64_t h = texcache::fnv1a64(key.bytes.data(), key.bytes.size());
    key.hash = h ^ (key.srgb ? 0x9E3779B97F4A7C15ull : 0ull);

    // Use default linear sampler
    VkSampler sampler = VK_NULL_HANDLE;
    if (_engine->_context && _engine->_context->samplers)
    {
        sampler = _engine->_context->samplers->defaultLinear();
    }

    // Request texture from cache
    TextureCache::TextureHandle handle = _engine->_textureCache->request(key, sampler);
    return static_cast<TextureHandle>(handle);
}

bool Engine::is_texture_loaded(TextureHandle handle) const
{
    if (!_engine || !_engine->_textureCache)
    {
        return false;
    }

    auto cacheHandle = static_cast<TextureCache::TextureHandle>(handle);
    return _engine->_textureCache->state(cacheHandle) == TextureCache::EntryState::Resident;
}

void* Engine::get_texture_image_view(TextureHandle handle) const
{
    if (!_engine || !_engine->_textureCache)
    {
        return nullptr;
    }

    auto cacheHandle = static_cast<TextureCache::TextureHandle>(handle);
    VkImageView view = _engine->_textureCache->image_view(cacheHandle);
    return reinterpret_cast<void*>(view);
}

void Engine::pin_texture(TextureHandle handle)
{
    if (!_engine || !_engine->_textureCache)
    {
        return;
    }

    auto cacheHandle = static_cast<TextureCache::TextureHandle>(handle);
    _engine->_textureCache->pin(cacheHandle);
}

void Engine::unpin_texture(TextureHandle handle)
{
    if (!_engine || !_engine->_textureCache)
    {
        return;
    }

    auto cacheHandle = static_cast<TextureCache::TextureHandle>(handle);
    _engine->_textureCache->unpin(cacheHandle);
}

bool Engine::is_texture_pinned(TextureHandle handle) const
{
    if (!_engine || !_engine->_textureCache)
    {
        return false;
    }

    auto cacheHandle = static_cast<TextureCache::TextureHandle>(handle);
    return _engine->_textureCache->is_pinned(cacheHandle);
}

void Engine::unload_texture(TextureHandle handle)
{
    if (!_engine || !_engine->_textureCache)
    {
        return;
    }

    auto cacheHandle = static_cast<TextureCache::TextureHandle>(handle);
    _engine->_textureCache->unload(cacheHandle);
}

void* Engine::create_imgui_texture(TextureHandle handle, void* sampler)
{
    if (!_engine || !_engine->_textureCache)
    {
        return nullptr;
    }

    auto cacheHandle = static_cast<TextureCache::TextureHandle>(handle);
    VkImageView imageView = _engine->_textureCache->image_view(cacheHandle);

    if (imageView == VK_NULL_HANDLE)
    {
        return nullptr;
    }

    // Use provided sampler or default linear sampler
    VkSampler vkSampler = reinterpret_cast<VkSampler>(sampler);
    if (vkSampler == VK_NULL_HANDLE && _engine->_context && _engine->_context->samplers)
    {
        vkSampler = _engine->_context->samplers->defaultLinear();
    }

    // Create ImGui descriptor set using ImGui_ImplVulkan
    VkDescriptorSet descriptorSet = ImGui_ImplVulkan_AddTexture(
        vkSampler,
        imageView,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
    );

    return reinterpret_cast<void*>(descriptorSet);
}

void Engine::free_imgui_texture(void* imgui_texture_id)
{
    if (imgui_texture_id == nullptr)
    {
        return;
    }

    VkDescriptorSet descriptorSet = reinterpret_cast<VkDescriptorSet>(imgui_texture_id);
    ImGui_ImplVulkan_RemoveTexture(descriptorSet);
}

} // namespace GameAPI
