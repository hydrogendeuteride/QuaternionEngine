#include "planet_system.h"
#include "planet_patch_helpers.h"

#include <core/context.h>
#include <core/device/resource.h>
#include <core/frame/resources.h>
#include <core/types.h>
#include <core/assets/manager.h>
#include <render/materials.h>
#include <core/pipeline/sampler.h>
#include <scene/mesh_bvh.h>
#include <scene/planet/cubesphere.h>
#include <scene/planet/planet_heightmap.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>

#include "device.h"

namespace
{
    constexpr uint32_t k_terrain_face_count = 6u;

    VkDeviceSize align_up(VkDeviceSize value, VkDeviceSize alignment)
    {
        if (alignment <= 1)
        {
            return value;
        }
        return (value + alignment - 1u) & ~(alignment - 1u);
    }

    VkDeviceSize terrain_material_constants_stride(DeviceManager *device)
    {
        VkPhysicalDeviceProperties props{};
        vkGetPhysicalDeviceProperties(device->physicalDevice(), &props);
        return align_up(sizeof(GLTFMetallic_Roughness::MaterialConstants),
                        props.limits.minUniformBufferOffsetAlignment);
    }

    std::string resolve_optional_face_texture_path(AssetManager &assets,
                                                   std::string_view dir,
                                                   const planet::CubeFace face)
    {
        if (dir.empty())
        {
            return {};
        }

        std::string rel = fmt::format("{}/{}.ktx2", dir, planet::cube_face_name(face));
        std::string abs_path = assets.assetPath(rel);
        if (std::filesystem::exists(abs_path))
        {
            return abs_path;
        }

        rel = fmt::format("{}/{}.png", dir, planet::cube_face_name(face));
        abs_path = assets.assetPath(rel);
        if (std::filesystem::exists(abs_path))
        {
            return abs_path;
        }

        return {};
    }
}

PlanetSystem::TerrainState *PlanetSystem::get_or_create_terrain_state(std::string_view name)
{
    if (name.empty())
    {
        return nullptr;
    }

    std::string key{name};
    auto it = _terrain_states.find(key);
    if (it != _terrain_states.end())
    {
        return it->second.get();
    }

    auto state = std::make_unique<TerrainState>();
    state->terrain_name = key;
    refresh_terrain_height_snapshot(*state);
    TerrainState *ptr = state.get();
    _terrain_states.emplace(std::move(key), std::move(state));
    return ptr;
}

PlanetSystem::TerrainState *PlanetSystem::find_terrain_state(std::string_view name)
{
    auto it = _terrain_states.find(std::string{name});
    return (it != _terrain_states.end()) ? it->second.get() : nullptr;
}

const PlanetSystem::TerrainState *PlanetSystem::find_terrain_state(std::string_view name) const
{
    auto it = _terrain_states.find(std::string{name});
    return (it != _terrain_states.end()) ? it->second.get() : nullptr;
}

void PlanetSystem::refresh_terrain_height_snapshot(TerrainState &state)
{
    state.height_faces_snapshot =
            std::make_shared<const std::array<planet::HeightFace, 6>>(state.height_faces);
}

PlanetSystem::TerrainPatch *PlanetSystem::find_terrain_patch(TerrainState &state, const planet::PatchKey &key)
{
    auto it = state.patch_lookup.find(key);
    if (it == state.patch_lookup.end())
    {
        return nullptr;
    }
    const uint32_t idx = it->second;
    if (idx >= state.patches.size())
    {
        return nullptr;
    }
    return &state.patches[idx];
}

const PlanetSystem::TerrainPatch *PlanetSystem::find_terrain_patch(const TerrainState &state,
                                                                   const planet::PatchKey &key) const
{
    auto it = state.patch_lookup.find(key);
    if (it == state.patch_lookup.end())
    {
        return nullptr;
    }
    const uint32_t idx = it->second;
    if (idx >= state.patches.size())
    {
        return nullptr;
    }
    return &state.patches[idx];
}

bool PlanetSystem::is_terrain_patch_ready(const TerrainState &state, const planet::PatchKey &key) const
{
    const TerrainPatch *p = find_terrain_patch(state, key);
    if (!p)
    {
        return false;
    }

    return p->state == TerrainPatchState::Ready &&
           p->vertex_buffer.buffer != VK_NULL_HANDLE &&
           p->vertex_buffer_address != 0;
}

void PlanetSystem::clear_terrain_patch_cache(TerrainState &state)
{
    ++state.terrain_generation;

    if (!state.terrain_name.empty())
    {
        std::lock_guard<std::mutex> lock(_terrain_async.mutex);
        const auto same_terrain = [&](const auto &item) {
            return item.terrain_name == state.terrain_name;
        };

        for (auto it = _terrain_async.pending_requests.begin(); it != _terrain_async.pending_requests.end();)
        {
            if (same_terrain(*it))
            {
                it = _terrain_async.pending_requests.erase(it);
            }
            else
            {
                ++it;
            }
        }

        for (auto it = _terrain_async.completed_results.begin(); it != _terrain_async.completed_results.end();)
        {
            if (same_terrain(*it))
            {
                it = _terrain_async.completed_results.erase(it);
            }
            else
            {
                ++it;
            }
        }

        for (auto it = _terrain_async.latest_request_ids.begin(); it != _terrain_async.latest_request_ids.end();)
        {
            if (it->first.terrain_name == state.terrain_name)
            {
                it = _terrain_async.latest_request_ids.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    ResourceManager *rm = _context ? _context->getResources() : nullptr;
    FrameResources *frame = _context ? _context->currentFrame : nullptr;

    if (rm)
    {
        for (TerrainPatch &p: state.patches)
        {
            if (p.vertex_buffer.buffer != VK_NULL_HANDLE)
            {
                const AllocatedBuffer vb = p.vertex_buffer;
                if (frame)
                {
                    frame->_deletionQueue.push_function([rm, vb]() { rm->destroy_buffer(vb); });
                }
                else
                {
                    rm->destroy_buffer(vb);
                }
            }
            p.vertex_buffer = {};
            p.vertex_buffer_address = 0;
            p.pick_bvh.reset();
        }
    }

    state.patch_lookup.clear();
    state.patch_lru.clear();
    state.patch_free.clear();
    state.patches.clear();
    state.current_render_cut.clear();
}

void PlanetSystem::clear_all_terrain_patch_caches()
{
    for (auto &kv: _terrain_states)
    {
        if (!kv.second)
        {
            continue;
        }
        clear_terrain_patch_cache(*kv.second);
        kv.second->debug_stats = {};
    }
}

void PlanetSystem::clear_terrain_materials(TerrainState &state)
{
    if (_context && _context->textures)
    {
        TextureCache *textures = _context->textures;
        for (MaterialInstance &mat: state.face_materials)
        {
            if (mat.materialSet != VK_NULL_HANDLE)
            {
                textures->unwatchSet(mat.materialSet);
            }
        }
    }

    // Keep descriptor sets allocated so they can be reused if terrain is re-enabled,
    // but force a rebinding on next use.
    state.bound_albedo_dir.clear();
    state.bound_height_texture_dir.clear();
    state.bound_emission_dir.clear();
    state.bound_specular_dir.clear();
    state.bound_detail_normal_dir.clear();
    state.bound_cavity_dir.clear();
}

void PlanetSystem::ensure_earth_patch_index_buffer()
{
    if (_earth_patch_index_buffer.buffer != VK_NULL_HANDLE && _earth_patch_index_resolution == _earth_patch_resolution)
    {
        return;
    }

    if (!_context)
    {
        return;
    }

    ResourceManager *rm = _context->getResources();
    if (!rm)
    {
        return;
    }

    // Resolution changed (or first init): clear existing patch cache and shared index buffer.
    if (_earth_patch_index_buffer.buffer != VK_NULL_HANDLE)
    {
        FrameResources *frame = _context->currentFrame;

        clear_all_terrain_patch_caches();

        const AllocatedBuffer ib = _earth_patch_index_buffer;
        if (frame)
        {
            frame->_deletionQueue.push_function([rm, ib]() { rm->destroy_buffer(ib); });
        }
        else
        {
            rm->destroy_buffer(ib);
        }
        _earth_patch_index_buffer = {};
        _earth_patch_index_count = 0;
        _earth_patch_index_resolution = 0;
        _earth_patch_indices_cpu.clear();
        _earth_patch_indices_cpu_snapshot.reset();
    }

    _earth_patch_indices_cpu.clear();
    planet::build_cubesphere_patch_indices(_earth_patch_indices_cpu, _earth_patch_resolution);
    _earth_patch_indices_cpu_snapshot =
            std::make_shared<const std::vector<uint32_t>>(_earth_patch_indices_cpu);
    _earth_patch_index_count = static_cast<uint32_t>(_earth_patch_indices_cpu.size());
    _earth_patch_index_buffer =
            rm->upload_buffer(_earth_patch_indices_cpu.data(),
                              _earth_patch_indices_cpu.size() * sizeof(uint32_t),
                              VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                              VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR);
    _earth_patch_index_resolution = _earth_patch_resolution;
}

void PlanetSystem::ensure_earth_patch_material_layout()
{
    if (_earth_patch_material_layout != VK_NULL_HANDLE)
    {
        return;
    }

    if (!_context)
    {
        return;
    }

    DeviceManager *device = _context->getDevice();
    if (!device)
    {
        return;
    }

    DescriptorLayoutBuilder layoutBuilder;
    layoutBuilder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    layoutBuilder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    _earth_patch_material_layout = layoutBuilder.build(device->device(),
                                                       VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                                       nullptr,
                                                       VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
}

void PlanetSystem::ensure_terrain_material_constants_buffer(TerrainState &state,
                                                            const PlanetBody &body,
                                                            const glm::vec3 &planet_center_local)
{
    const bool specular_enabled = !body.terrain_specular_dir.empty() && (body.specular_strength > 0.0f);
    const float specular_strength = specular_enabled ? std::clamp(body.specular_strength, 0.0f, 1.0f) : 0.0f;
    const float specular_roughness = std::clamp(body.specular_roughness, 0.04f, 1.0f);
    const bool has_detail_normal = !body.terrain_detail_normal_dir.empty() && (body.terrain_detail_normal_strength != 0.0f);
    const float detail_normal_strength = has_detail_normal ? body.terrain_detail_normal_strength : 0.0f;
    const bool has_cavity = !body.terrain_cavity_dir.empty() && (body.terrain_cavity_strength > 0.0f);
    const float cavity_strength = has_cavity ? std::max(body.terrain_cavity_strength, 0.0f) : 0.0f;
    const bool terminator_enabled =
            body.terrain_enable_terminator_shadow &&
            !body.terrain_height_dir.empty() &&
            (body.terrain_height_max_m > 0.0);

    auto build_constants = [&](const uint32_t face_index) {
        GLTFMetallic_Roughness::MaterialConstants constants =
                planet_helpers::make_planet_constants(body.base_color,
                                                      body.metallic,
                                                      body.roughness,
                                                      body.emission_factor);
        // Terrain path uses constant metallic/roughness instead of a sampled ORM texture.
        constants.extra[0].y = cavity_strength;
        constants.extra[0].z = has_cavity ? 1.0f : 0.0f;
        constants.extra[2].z = specular_enabled ? 1.0f : 0.0f;
        constants.extra[2].w = specular_strength;
        constants.extra[3] = glm::vec4(1.0f, 0.0f, 0.0f, specular_roughness);
        constants.extra[4] = glm::vec4(planet_center_local, static_cast<float>(body.radius_m));
        constants.extra[5] = glm::vec4(static_cast<float>(face_index),
                                       detail_normal_strength,
                                       cavity_strength,
                                       terminator_enabled ? 1.0f : 0.0f);
        constants.extra[6] = glm::vec4(static_cast<float>(body.terrain_height_max_m), 0.0f, 0.0f, 0.0f);
        return constants;
    };

    if (!_context)
    {
        return;
    }

    ResourceManager *rm = _context->getResources();
    DeviceManager *device = _context->getDevice();
    if (!rm || !device)
    {
        return;
    }

    const VkDeviceSize stride = terrain_material_constants_stride(device);
    const VkDeviceSize required_size = stride * k_terrain_face_count;

    if (state.material_constants_buffer.buffer != VK_NULL_HANDLE &&
        state.material_constants_stride != 0 &&
        state.material_constants_stride != stride)
    {
        rm->destroy_buffer(state.material_constants_buffer);
        state.material_constants_buffer = {};
        state.material_constants_stride = 0;
    }

    if (state.material_constants_buffer.buffer == VK_NULL_HANDLE)
    {
        state.material_constants_buffer =
                rm->create_buffer(required_size,
                                  VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                  VMA_MEMORY_USAGE_CPU_TO_GPU);
        state.material_constants_stride = stride;
    }

    if (state.material_constants_buffer.buffer == VK_NULL_HANDLE)
    {
        return;
    }

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(device->allocator(), state.material_constants_buffer.allocation, &allocInfo);
    auto *mapped = static_cast<std::byte *>(allocInfo.pMappedData);
    if (mapped)
    {
        for (uint32_t face_index = 0; face_index < k_terrain_face_count; ++face_index)
        {
            const GLTFMetallic_Roughness::MaterialConstants constants = build_constants(face_index);
            std::memcpy(mapped + stride * face_index, &constants, sizeof(constants));
        }
        vmaFlushAllocation(device->allocator(), state.material_constants_buffer.allocation, 0, required_size);

        state.bound_base_color = body.base_color;
        state.bound_metallic = body.metallic;
        state.bound_roughness = body.roughness;
        state.bound_emission_factor = body.emission_factor;
        state.bound_specular_enabled = specular_enabled;
        state.bound_specular_strength = specular_strength;
        state.bound_specular_roughness = specular_roughness;
    }
}

void PlanetSystem::ensure_terrain_face_materials(TerrainState &state,
                                                 const PlanetBody &body,
                                                 const glm::vec3 &planet_center_local)
{
    if (!_context || !body.material)
    {
        return;
    }

    DeviceManager *device = _context->getDevice();
    SamplerManager *samplers = _context->getSamplers();
    AssetManager *assets = _context->assets;
    TextureCache *textures = _context->textures;
    if (!device || !assets)
    {
        return;
    }

    ensure_earth_patch_material_layout();
    ensure_terrain_material_constants_buffer(state, body, planet_center_local);

    if (_earth_patch_material_layout == VK_NULL_HANDLE ||
        state.material_constants_buffer.buffer == VK_NULL_HANDLE)
    {
        return;
    }

    if (!_earth_patch_material_allocator_initialized)
    {
        std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> sizes = {
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1},
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 7},
        };
        _earth_patch_material_allocator.init(device->device(), 16, sizes);
        _earth_patch_material_allocator_initialized = true;
    }

    VkSampler tileSampler = samplers ? samplers->linearClampEdge() : VK_NULL_HANDLE;
    if (tileSampler == VK_NULL_HANDLE && samplers)
    {
        tileSampler = samplers->defaultLinear();
    }
    if (tileSampler == VK_NULL_HANDLE)
    {
        return;
    }

    VkSampler specularSampler = tileSampler;
    if (specularSampler == VK_NULL_HANDLE)
    {
        specularSampler = tileSampler;
    }

    VkImageView checker = assets->fallbackCheckerboardView();
    VkImageView white = assets->fallbackWhiteView();
    VkImageView flatNormal = assets->fallbackFlatNormalView();
    VkImageView black = assets->fallbackBlackView();

    if (checker == VK_NULL_HANDLE) checker = white;
    if (white == VK_NULL_HANDLE) white = checker;
    if (flatNormal == VK_NULL_HANDLE) flatNormal = white;
    if (black == VK_NULL_HANDLE) black = white;

    const std::string desired_albedo_dir = body.terrain_albedo_dir;
    const bool albedo_dir_changed = desired_albedo_dir != state.bound_albedo_dir;
    if (albedo_dir_changed)
    {
        state.bound_albedo_dir = desired_albedo_dir;
    }

    const std::string desired_height_dir = body.terrain_height_dir;
    const bool height_dir_changed = desired_height_dir != state.bound_height_texture_dir;
    if (height_dir_changed)
    {
        state.bound_height_texture_dir = desired_height_dir;
    }

    const std::string desired_detail_normal_dir = body.terrain_detail_normal_dir;
    const bool detail_normal_dir_changed = desired_detail_normal_dir != state.bound_detail_normal_dir;
    if (detail_normal_dir_changed)
    {
        state.bound_detail_normal_dir = desired_detail_normal_dir;
    }

    const std::string desired_cavity_dir = body.terrain_cavity_dir;
    const bool cavity_dir_changed = desired_cavity_dir != state.bound_cavity_dir;
    if (cavity_dir_changed)
    {
        state.bound_cavity_dir = desired_cavity_dir;
    }

    const std::string desired_emission_dir = body.terrain_emission_dir;
    const bool emission_dir_changed = desired_emission_dir != state.bound_emission_dir;
    if (emission_dir_changed)
    {
        state.bound_emission_dir = desired_emission_dir;
    }

    const std::string desired_specular_dir = body.terrain_specular_dir;
    const bool specular_dir_changed = desired_specular_dir != state.bound_specular_dir;
    if (specular_dir_changed)
    {
        state.bound_specular_dir = desired_specular_dir;
    }

    auto watch_albedo = [&](size_t face_index, VkDescriptorSet set) {
        if (!textures || desired_albedo_dir.empty())
        {
            return;
        }

        const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
        const std::string rel = fmt::format("{}/{}.ktx2", desired_albedo_dir, planet::cube_face_name(face));

        TextureCache::TextureKey tk{};
        tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
        tk.path = assets->assetPath(rel);
        tk.srgb = true;
        tk.mipmapped = true;

        TextureCache::TextureHandle h = textures->request(tk, tileSampler);
        textures->watchBinding(h, set, 1u, tileSampler, checker);
    };

    auto watch_height = [&](size_t face_index, VkDescriptorSet set) {
        if (!textures || desired_height_dir.empty())
        {
            return;
        }

        const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
        const std::string abs_path = resolve_optional_face_texture_path(*assets, desired_height_dir, face);
        if (abs_path.empty())
        {
            return;
        }

        TextureCache::TextureKey tk{};
        tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
        tk.path = abs_path;
        tk.srgb = false;
        tk.mipmapped = true;
        tk.channels = TextureCache::TextureKey::ChannelsHint::R;

        TextureCache::TextureHandle h = textures->request(tk, tileSampler);
        textures->watchBinding(h, set, 2u, tileSampler, white);
    };

    auto watch_detail_normal = [&](size_t face_index, VkDescriptorSet set) {
        if (!textures || desired_detail_normal_dir.empty())
        {
            return;
        }

        const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
        const std::string abs_path = resolve_optional_face_texture_path(*assets, desired_detail_normal_dir, face);
        if (abs_path.empty())
        {
            return;
        }

        TextureCache::TextureKey tk{};
        tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
        tk.path = abs_path;
        tk.srgb = false;
        tk.mipmapped = true;

        TextureCache::TextureHandle h = textures->request(tk, tileSampler);
        textures->watchBinding(h, set, 3u, tileSampler, flatNormal);
    };

    auto watch_cavity = [&](size_t face_index, VkDescriptorSet set) {
        if (!textures || desired_cavity_dir.empty())
        {
            return;
        }

        const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
        const std::string abs_path = resolve_optional_face_texture_path(*assets, desired_cavity_dir, face);
        if (abs_path.empty())
        {
            return;
        }

        TextureCache::TextureKey tk{};
        tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
        tk.path = abs_path;
        tk.srgb = false;
        tk.mipmapped = true;
        tk.channels = TextureCache::TextureKey::ChannelsHint::R;

        TextureCache::TextureHandle h = textures->request(tk, tileSampler);
        textures->watchBinding(h, set, 4u, tileSampler, white);
    };

    auto watch_emission = [&](size_t face_index, VkDescriptorSet set) {
        if (!textures || desired_emission_dir.empty())
        {
            return;
        }

        const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
        const std::string abs_path = resolve_optional_face_texture_path(*assets, desired_emission_dir, face);
        if (abs_path.empty())
        {
            return;
        }

        TextureCache::TextureKey tk{};
        tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
        tk.path = abs_path;
        tk.srgb = true;
        tk.mipmapped = true;

        TextureCache::TextureHandle h = textures->request(tk, tileSampler);
        textures->watchBinding(h, set, 5u, tileSampler, black);
    };

    auto watch_specular = [&](size_t face_index, VkDescriptorSet set) {
        if (!textures || desired_specular_dir.empty())
        {
            return;
        }

        const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
        const std::string abs_path = resolve_optional_face_texture_path(*assets, desired_specular_dir, face);
        if (abs_path.empty())
        {
            return;
        }

        TextureCache::TextureKey tk{};
        tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
        tk.path = abs_path;
        tk.srgb = false;
        tk.mipmapped = true;
        tk.channels = TextureCache::TextureKey::ChannelsHint::R;

        TextureCache::TextureHandle h = textures->request(tk, specularSampler);
        textures->watchBinding(h, set, 6u, specularSampler, black);
    };

    for (size_t face_index = 0; face_index < state.face_materials.size(); ++face_index)
    {
        MaterialInstance &mat = state.face_materials[face_index];

        mat.pipeline = body.material->data.pipeline;
        mat.passType = body.material->data.passType;

        if (mat.materialSet == VK_NULL_HANDLE)
        {
            mat.materialSet = _earth_patch_material_allocator.allocate(device->device(), _earth_patch_material_layout);

            DescriptorWriter writer;
            writer.write_buffer(0,
                                state.material_constants_buffer.buffer,
                                sizeof(GLTFMetallic_Roughness::MaterialConstants),
                                state.material_constants_stride * face_index,
                                VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
            writer.write_image(1,
                               checker,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(2,
                               white,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(3,
                               flatNormal,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(4,
                               white,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(5,
                               black,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(6,
                               black,
                               specularSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.update_set(device->device(), mat.materialSet);

            watch_albedo(face_index, mat.materialSet);
            watch_height(face_index, mat.materialSet);
            watch_detail_normal(face_index, mat.materialSet);
            watch_cavity(face_index, mat.materialSet);
            watch_emission(face_index, mat.materialSet);
            watch_specular(face_index, mat.materialSet);
        }
        else if ((albedo_dir_changed || height_dir_changed || detail_normal_dir_changed ||
                  cavity_dir_changed || emission_dir_changed || specular_dir_changed) &&
                 textures &&
                 tileSampler != VK_NULL_HANDLE)
        {
            textures->unwatchSet(mat.materialSet);

            DescriptorWriter writer;
            if (albedo_dir_changed)
            {
                writer.write_image(1,
                                   checker,
                                   tileSampler,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            }
            if (height_dir_changed)
            {
                writer.write_image(2,
                                   white,
                                   tileSampler,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            }
            if (detail_normal_dir_changed)
            {
                writer.write_image(3,
                                   flatNormal,
                                   tileSampler,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            }
            if (cavity_dir_changed)
            {
                writer.write_image(4,
                                   white,
                                   tileSampler,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            }
            if (emission_dir_changed)
            {
                writer.write_image(5,
                                   black,
                                   tileSampler,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            }
            if (specular_dir_changed)
            {
                writer.write_image(6,
                                   black,
                                   specularSampler,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            }
            writer.update_set(device->device(), mat.materialSet);

            watch_albedo(face_index, mat.materialSet);
            watch_height(face_index, mat.materialSet);
            watch_detail_normal(face_index, mat.materialSet);
            watch_cavity(face_index, mat.materialSet);
            watch_emission(face_index, mat.materialSet);
            watch_specular(face_index, mat.materialSet);
        }
    }
}

void PlanetSystem::ensure_terrain_height_maps(TerrainState &state, const PlanetBody &body)
{
    if (!_context || !_context->assets)
    {
        return;
    }

    const std::string desired_dir = body.terrain_height_dir;
    const double desired_max_m = body.terrain_height_max_m;
    const bool changed =
            desired_dir != state.bound_height_dir ||
            desired_max_m != state.bound_height_max_m;

    const bool want_height = !desired_dir.empty() && (desired_max_m > 0.0);
    const bool have_height = [&]() -> bool {
        for (const planet::HeightFace &f: state.height_faces)
        {
            if (f.width == 0 || f.height == 0 || f.texels.empty())
            {
                return false;
            }
        }
        return true;
    }();

    const bool needs_load = changed || (want_height && !have_height);
    if (!needs_load)
    {
        return;
    }

    if (changed)
    {
        // Height affects vertex positions/normals; regenerate patch meshes if parameters change.
        clear_terrain_patch_cache(state);
        state.patch_cache_dirty = false;

        state.bound_height_dir = desired_dir;
        state.bound_height_max_m = desired_max_m;
        for (planet::HeightFace &f: state.height_faces)
        {
            f = {};
        }
        refresh_terrain_height_snapshot(state);
    }

    if (!want_height)
    {
        for (planet::HeightFace &f: state.height_faces)
        {
            f = {};
        }
        refresh_terrain_height_snapshot(state);
        return;
    }

    AssetManager *assets = _context->assets;
    planet::HeightFaceSet loaded_faces{};
    bool ok = planet::take_preloaded_heightmap_faces(desired_dir, loaded_faces);

    if (!ok)
    {
        for (size_t face_index = 0; face_index < loaded_faces.size(); ++face_index)
        {
            const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
            const std::string rel = fmt::format("{}/{}.ktx2", desired_dir, planet::cube_face_name(face));
            const std::string abs = assets->assetPath(rel);

            planet::HeightFace face_data{};
            if (!planet::load_heightmap_bc4(abs, face_data))
            {
                Logger::error("[PlanetSystem] Failed to load height face '{}'", abs);
                ok = false;
                break;
            }

            ok = true;
            loaded_faces[face_index] = std::move(face_data);
        }
    }

    if (!ok)
    {
        // If this was a retry (parameters didn't change), keep existing geometry and retry later.
        // If parameters changed, we already cleared meshes/faces above.
        return;
    }

    state.height_faces = std::move(loaded_faces);
    refresh_terrain_height_snapshot(state);

    if (!changed)
    {
        // Recovered height data after a previous failure; regenerate meshes so displacement applies.
        clear_terrain_patch_cache(state);
        state.patch_cache_dirty = false;
    }
}

PlanetSystem::TerrainPatch *PlanetSystem::ensure_terrain_patch(TerrainState &state,
                                                               const planet::PatchKey &key,
                                                               const uint32_t frame_index)
{
    if (TerrainPatch *patch = find_terrain_patch(state, key))
    {
        patch->last_used_frame = frame_index;
        state.patch_lru.splice(state.patch_lru.begin(), state.patch_lru, patch->lru_it);
        return patch;
    }

    uint32_t idx = 0;
    if (!state.patch_free.empty())
    {
        idx = state.patch_free.back();
        state.patch_free.pop_back();
    }
    else
    {
        idx = static_cast<uint32_t>(state.patches.size());
        state.patches.emplace_back();
    }

    if (idx >= state.patches.size())
    {
        return nullptr;
    }

    TerrainPatch &patch = state.patches[idx];
    patch = TerrainPatch{};
    patch.key = key;
    patch.state = TerrainPatchState::Allocating;
    patch.last_used_frame = frame_index;
    state.patch_lru.push_front(idx);
    patch.lru_it = state.patch_lru.begin();
    state.patch_lookup[key] = idx;
    return &patch;
}

PlanetSystem::TerrainBuildSnapshot PlanetSystem::make_terrain_build_snapshot(const TerrainState &state,
                                                                             const PlanetBody &body) const
{
    TerrainBuildSnapshot snapshot{};
    snapshot.radius_m = body.radius_m;
    snapshot.height_max_m = body.terrain_height_max_m;
    snapshot.patch_resolution = std::max(2u, _earth_patch_resolution);
    snapshot.debug_tint_by_lod = _earth_debug_tint_patches_by_lod;
    snapshot.height_faces = state.height_faces_snapshot;
    snapshot.patch_indices = _earth_patch_indices_cpu_snapshot;
    return snapshot;
}

void PlanetSystem::start_terrain_patch_workers()
{
    std::lock_guard<std::mutex> lock(_terrain_async.mutex);
    if (_terrain_async.running)
    {
        return;
    }

    _terrain_async.running = true;
    _terrain_async.next_request_id = 1;

    const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
    const std::size_t worker_count = std::clamp<std::size_t>(
        static_cast<std::size_t>(hw_threads > 1 ? hw_threads - 1 : 1), 1, 2);
    _terrain_async.workers.reserve(worker_count);
    for (std::size_t i = 0; i < worker_count; ++i)
    {
        _terrain_async.workers.emplace_back(&PlanetSystem::terrain_patch_worker_loop, this);
    }
}

void PlanetSystem::stop_terrain_patch_workers()
{
    std::vector<std::thread> workers;
    {
        std::lock_guard<std::mutex> lock(_terrain_async.mutex);
        if (!_terrain_async.running && _terrain_async.workers.empty())
        {
            return;
        }

        _terrain_async.running = false;
        _terrain_async.pending_requests.clear();
        _terrain_async.latest_request_ids.clear();
        workers = std::move(_terrain_async.workers);
    }

    _terrain_async.cv.notify_all();
    for (std::thread &worker: workers)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }

    std::lock_guard<std::mutex> lock(_terrain_async.mutex);
    _terrain_async.completed_results.clear();
    _terrain_async.next_request_id = 1;
}

void PlanetSystem::terrain_patch_worker_loop()
{
    while (true)
    {
        TerrainBuildRequest request{};
        {
            std::unique_lock<std::mutex> lock(_terrain_async.mutex);
            _terrain_async.cv.wait(lock, [&]() {
                return !_terrain_async.running || !_terrain_async.pending_requests.empty();
            });

            if (!_terrain_async.running && _terrain_async.pending_requests.empty())
            {
                return;
            }

            request = std::move(_terrain_async.pending_requests.front());
            _terrain_async.pending_requests.pop_front();

            const TerrainBuildJobKey job_key{request.terrain_name, request.key};
            auto latest_it = _terrain_async.latest_request_ids.find(job_key);
            if (latest_it == _terrain_async.latest_request_ids.end() ||
                latest_it->second != request.request_id)
            {
                continue;
            }
        }

        TerrainBuildResult result{};
        result.terrain_name = request.terrain_name;
        result.key = request.key;
        result.edge_stitch_mask = request.edge_stitch_mask;
        result.request_id = request.request_id;
        result.terrain_generation = request.terrain_generation;
        result.success = build_terrain_patch_cpu(request, result);

        {
            std::lock_guard<std::mutex> lock(_terrain_async.mutex);
            if (!_terrain_async.running)
            {
                return;
            }

            const TerrainBuildJobKey job_key{request.terrain_name, request.key};
            auto latest_it = _terrain_async.latest_request_ids.find(job_key);
            if (latest_it != _terrain_async.latest_request_ids.end() &&
                latest_it->second == request.request_id)
            {
                _terrain_async.completed_results.push_back(std::move(result));
            }
        }
    }
}

bool PlanetSystem::build_terrain_patch_cpu(const TerrainBuildRequest &request, TerrainBuildResult &out_result)
{
    if (!request.snapshot.patch_indices || request.snapshot.patch_indices->empty())
    {
        return false;
    }

    const glm::vec4 vertex_color =
            request.snapshot.debug_tint_by_lod
                    ? planet_helpers::debug_color_for_level(request.key.level)
                    : glm::vec4(1.0f);

    thread_local std::vector<Vertex> scratch_vertices;
    const uint32_t safe_res = std::max(2u, request.snapshot.patch_resolution);
    scratch_vertices.clear();
    scratch_vertices.reserve(
        static_cast<size_t>(safe_res) * static_cast<size_t>(safe_res) +
        static_cast<size_t>(4u) * static_cast<size_t>(safe_res));

    const glm::dvec3 patch_center_dir =
            planet::build_cubesphere_patch_vertices(scratch_vertices,
                                                    request.snapshot.radius_m,
                                                    request.key.face,
                                                    request.key.level,
                                                    request.key.x,
                                                    request.key.y,
                                                    safe_res,
                                                    vertex_color);
    if (scratch_vertices.empty())
    {
        return false;
    }

    const auto *height_faces = request.snapshot.height_faces.get();
    if (request.snapshot.height_max_m > 0.0 && height_faces)
    {
        const uint32_t face_index = static_cast<uint32_t>(request.key.face);
        if (face_index < height_faces->size())
        {
            const planet::HeightFace &height_face = (*height_faces)[face_index];
            if (height_face.width > 0 && height_face.height > 0 && !height_face.texels.empty())
            {
                const float scale = static_cast<float>(request.snapshot.height_max_m);
                const uint32_t height_mip_level =
                        planet::choose_height_mip_level(height_face, request.key.level, safe_res);
                constexpr float kFaceEdgeEpsilon = 1e-6f;
                for (Vertex &vertex: scratch_vertices)
                {
                    float h01 = 0.0f;
                    const bool on_cube_face_edge =
                            (vertex.uv_x <= kFaceEdgeEpsilon) ||
                            (vertex.uv_x >= 1.0f - kFaceEdgeEpsilon) ||
                            (vertex.uv_y <= kFaceEdgeEpsilon) ||
                            (vertex.uv_y >= 1.0f - kFaceEdgeEpsilon);

                    if (on_cube_face_edge)
                    {
                        planet::CubeFace sample_face = request.key.face;
                        double sample_u = static_cast<double>(vertex.uv_x);
                        double sample_v = static_cast<double>(vertex.uv_y);
                        const glm::vec3 normal = glm::normalize(vertex.normal);
                        const glm::dvec3 dir(static_cast<double>(normal.x),
                                             static_cast<double>(normal.y),
                                             static_cast<double>(normal.z));
                        if (planet::cubesphere_direction_to_face_uv(dir, sample_face, sample_u, sample_v))
                        {
                            const uint32_t sample_face_index = static_cast<uint32_t>(sample_face);
                            if (sample_face_index < height_faces->size())
                            {
                                const planet::HeightFace &sample_height_face = (*height_faces)[sample_face_index];
                                if (sample_height_face.width > 0 &&
                                    sample_height_face.height > 0 &&
                                    !sample_height_face.texels.empty())
                                {
                                    h01 = planet::sample_height(sample_height_face,
                                                                static_cast<float>(sample_u),
                                                                static_cast<float>(sample_v),
                                                                height_mip_level);
                                }
                                else
                                {
                                    h01 = planet::sample_height(
                                        height_face, vertex.uv_x, vertex.uv_y, height_mip_level);
                                }
                            }
                            else
                            {
                                h01 = planet::sample_height(
                                    height_face, vertex.uv_x, vertex.uv_y, height_mip_level);
                            }
                        }
                        else
                        {
                            h01 = planet::sample_height(
                                height_face, vertex.uv_x, vertex.uv_y, height_mip_level);
                        }
                    }
                    else
                    {
                        h01 = planet::sample_height(
                            height_face, vertex.uv_x, vertex.uv_y, height_mip_level);
                    }

                    vertex.position += vertex.normal * (h01 * scale);
                }

                planet_helpers::stitch_patch_edges_to_parent_grid(
                    scratch_vertices, safe_res, request.edge_stitch_mask);
                planet_helpers::recompute_patch_normals(scratch_vertices, safe_res);
                planet_helpers::refine_patch_edge_normals_from_height(scratch_vertices,
                                                                      safe_res,
                                                                      patch_center_dir,
                                                                      request.snapshot.radius_m,
                                                                      request.key.level,
                                                                      height_mip_level,
                                                                      request.edge_stitch_mask,
                                                                      request.snapshot.height_max_m,
                                                                      *height_faces);
            }
        }
    }

    planet_helpers::reinforce_patch_skirts(scratch_vertices,
                                           safe_res,
                                           patch_center_dir,
                                           request.snapshot.radius_m,
                                           request.key.level);

    const planet_helpers::PatchBoundsData bounds = planet_helpers::compute_patch_bounds(scratch_vertices);

    std::shared_ptr<MeshBVH> patch_pick_bvh{};
    {
        MeshAsset pick_mesh{};
        GeoSurface pick_surface{};
        pick_surface.startIndex = 0;
        pick_surface.count = static_cast<uint32_t>(request.snapshot.patch_indices->size());
        pick_mesh.surfaces.push_back(pick_surface);

        std::unique_ptr<MeshBVH> built_bvh =
                build_mesh_bvh(pick_mesh,
                               std::span<const Vertex>(scratch_vertices),
                               std::span<const uint32_t>(*request.snapshot.patch_indices));
        if (built_bvh)
        {
            patch_pick_bvh = std::move(built_bvh);
        }
    }

    out_result.success = true;
    out_result.vertices = scratch_vertices;
    out_result.pick_bvh = std::move(patch_pick_bvh);
    out_result.bounds_origin = bounds.origin;
    out_result.bounds_extents = bounds.extents;
    out_result.bounds_sphere_radius = bounds.sphere_radius;
    out_result.patch_center_dir = patch_center_dir;
    return true;
}

uint64_t PlanetSystem::request_terrain_patch_build(TerrainState &state,
                                                   const PlanetBody &body,
                                                   const planet::PatchKey &key,
                                                   const uint32_t frame_index,
                                                   const uint8_t edge_stitch_mask)
{
    start_terrain_patch_workers();

    TerrainPatch *patch = ensure_terrain_patch(state, key, frame_index);
    if (!patch)
    {
        return 0;
    }

    if (patch->build_requested &&
        patch->pending_edge_stitch_mask == edge_stitch_mask &&
        patch->pending_request_id != 0)
    {
        return patch->pending_request_id;
    }

    if (patch->state == TerrainPatchState::Ready &&
        patch->vertex_buffer.buffer != VK_NULL_HANDLE &&
        patch->vertex_buffer_address != 0 &&
        patch->edge_stitch_mask == edge_stitch_mask)
    {
        return 0;
    }

    const TerrainBuildSnapshot snapshot = make_terrain_build_snapshot(state, body);
    if (!snapshot.patch_indices || snapshot.patch_indices->empty())
    {
        return 0;
    }

    TerrainBuildRequest request{};
    request.terrain_name = body.name;
    request.key = key;
    request.edge_stitch_mask = edge_stitch_mask;
    request.terrain_generation = state.terrain_generation;
    request.snapshot = snapshot;

    {
        std::lock_guard<std::mutex> lock(_terrain_async.mutex);
        if (!_terrain_async.running)
        {
            return 0;
        }

        request.request_id = _terrain_async.next_request_id++;
        _terrain_async.latest_request_ids[TerrainBuildJobKey{request.terrain_name, request.key}] = request.request_id;
        _terrain_async.pending_requests.push_back(request);
    }

    patch->build_requested = true;
    patch->pending_request_id = request.request_id;
    patch->pending_edge_stitch_mask = edge_stitch_mask;
    if (patch->state != TerrainPatchState::Ready)
    {
        patch->state = TerrainPatchState::Allocating;
    }

    _terrain_async.cv.notify_one();
    return request.request_id;
}

bool PlanetSystem::commit_terrain_patch_result(TerrainState &state, const TerrainBuildResult &result)
{
    TerrainPatch *patch = find_terrain_patch(state, result.key);
    if (!patch)
    {
        return false;
    }

    const auto clear_request_tracking = [&]() {
        patch->build_requested = false;
        patch->pending_request_id = 0;
        patch->pending_edge_stitch_mask = 0;
        std::lock_guard<std::mutex> lock(_terrain_async.mutex);
        const TerrainBuildJobKey job_key{result.terrain_name, result.key};
        auto latest_it = _terrain_async.latest_request_ids.find(job_key);
        if (latest_it != _terrain_async.latest_request_ids.end() &&
            latest_it->second == result.request_id)
        {
            _terrain_async.latest_request_ids.erase(latest_it);
        }
    };

    if (state.terrain_generation != result.terrain_generation ||
        !patch->build_requested ||
        patch->pending_request_id != result.request_id)
    {
        return false;
    }

    if (!result.success || result.vertices.empty() || !_context)
    {
        if (patch->state != TerrainPatchState::Ready)
        {
            patch->state = TerrainPatchState::Allocating;
        }
        clear_request_tracking();
        return false;
    }

    ResourceManager *rm = _context->getResources();
    DeviceManager *device = _context->getDevice();
    if (!rm || !device)
    {
        if (patch->state != TerrainPatchState::Ready)
        {
            patch->state = TerrainPatchState::Allocating;
        }
        clear_request_tracking();
        return false;
    }

    AllocatedBuffer vb =
            rm->upload_buffer(result.vertices.data(),
                              result.vertices.size() * sizeof(Vertex),
                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                              VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR);
    if (vb.buffer == VK_NULL_HANDLE)
    {
        if (patch->state != TerrainPatchState::Ready)
        {
            patch->state = TerrainPatchState::Allocating;
        }
        clear_request_tracking();
        return false;
    }

    VkBufferDeviceAddressInfo addr_info{.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    addr_info.buffer = vb.buffer;
    const VkDeviceAddress addr = vkGetBufferDeviceAddress(device->device(), &addr_info);

    if (patch->vertex_buffer.buffer != VK_NULL_HANDLE)
    {
        FrameResources *frame = _context->currentFrame;
        const AllocatedBuffer vb_old = patch->vertex_buffer;
        if (frame)
        {
            frame->_deletionQueue.push_function([rm, vb_old]() { rm->destroy_buffer(vb_old); });
        }
        else
        {
            rm->destroy_buffer(vb_old);
        }
    }

    patch->state = TerrainPatchState::Ready;
    patch->edge_stitch_mask = result.edge_stitch_mask;
    patch->vertex_buffer = vb;
    patch->vertex_buffer_address = addr;
    patch->pick_bvh = result.pick_bvh;
    patch->bounds_origin = result.bounds_origin;
    patch->bounds_extents = result.bounds_extents;
    patch->bounds_sphere_radius = result.bounds_sphere_radius;
    patch->patch_center_dir = result.patch_center_dir;

    clear_request_tracking();
    return true;
}

void PlanetSystem::pump_completed_terrain_patch_builds(TerrainState &state,
                                                       std::string_view terrain_name,
                                                       uint32_t &created_patches,
                                                       double &ms_patch_create,
                                                       const uint32_t max_create,
                                                       const double max_create_ms)
{
    using Clock = std::chrono::steady_clock;

    while (true)
    {
        const bool hit_count_budget = (max_create != 0u) && (created_patches >= max_create);
        const bool hit_time_budget = (max_create_ms > 0.0) && (ms_patch_create >= max_create_ms);
        if (hit_count_budget || hit_time_budget)
        {
            break;
        }

        TerrainBuildResult result{};
        bool found_result = false;
        {
            std::lock_guard<std::mutex> lock(_terrain_async.mutex);
            for (auto it = _terrain_async.completed_results.begin();
                 it != _terrain_async.completed_results.end();)
            {
                if (find_terrain_state(it->terrain_name) == nullptr)
                {
                    it = _terrain_async.completed_results.erase(it);
                    continue;
                }

                if (it->terrain_name != terrain_name)
                {
                    ++it;
                    continue;
                }

                if (it->terrain_generation != state.terrain_generation)
                {
                    it = _terrain_async.completed_results.erase(it);
                    continue;
                }

                result = std::move(*it);
                it = _terrain_async.completed_results.erase(it);
                found_result = true;
                break;
            }
        }

        if (!found_result)
        {
            break;
        }

        const Clock::time_point t0 = Clock::now();
        const bool committed = commit_terrain_patch_result(state, result);
        const Clock::time_point t1 = Clock::now();
        ms_patch_create += std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (committed)
        {
            ++created_patches;
        }
    }
}

void PlanetSystem::trim_terrain_patch_cache(TerrainState &state)
{
    if (_earth_patch_cache_max == 0)
    {
        return;
    }

    if (state.patch_lookup.size() <= static_cast<size_t>(_earth_patch_cache_max))
    {
        return;
    }

    if (!_context)
    {
        return;
    }

    ResourceManager *rm = _context->getResources();
    if (!rm)
    {
        return;
    }

    FrameResources *frame = _context->currentFrame;
    const uint32_t now = state.patch_frame_stamp;
    std::unordered_set<planet::PatchKey, planet::PatchKeyHash> protected_keys;
    protected_keys.reserve(state.current_render_cut.size() * 2u + 128u);
    for (const planet::PatchKey &key: state.current_render_cut)
    {
        protected_keys.insert(key);
    }

    size_t guard = 0;
    const size_t guard_limit = state.patch_lru.size();

    while (state.patch_lookup.size() > static_cast<size_t>(_earth_patch_cache_max) && !state.patch_lru.empty())
    {
        if (guard++ >= guard_limit)
        {
            // No evictable patches (all used this frame). Avoid thrashing.
            break;
        }

        const uint32_t idx = state.patch_lru.back();
        if (idx >= state.patches.size())
        {
            state.patch_lru.pop_back();
            continue;
        }

        TerrainPatch &p = state.patches[idx];
        if (p.last_used_frame == now)
        {
            state.patch_lru.splice(state.patch_lru.begin(), state.patch_lru, p.lru_it);
            continue;
        }

        if (is_pinned_patch_key(p.key) || protected_keys.contains(p.key) || p.build_requested)
        {
            state.patch_lru.splice(state.patch_lru.begin(), state.patch_lru, p.lru_it);
            continue;
        }

        guard = 0;

        state.patch_lru.erase(p.lru_it);
        state.patch_lookup.erase(p.key);

        if (p.vertex_buffer.buffer != VK_NULL_HANDLE)
        {
            const AllocatedBuffer vb = p.vertex_buffer;
            if (frame)
            {
                frame->_deletionQueue.push_function([rm, vb]() { rm->destroy_buffer(vb); });
            }
            else
            {
                rm->destroy_buffer(vb);
            }
        }

        p = TerrainPatch{};
        state.patch_free.push_back(idx);
    }
}
