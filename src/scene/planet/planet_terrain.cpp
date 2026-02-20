#include "planet_system.h"
#include "planet_patch_helpers.h"

#include <core/context.h>
#include <core/device/resource.h>
#include <core/frame/resources.h>
#include <core/types.h>
#include <core/assets/manager.h>
#include <render/materials.h>
#include <core/pipeline/sampler.h>
#include <scene/planet/cubesphere.h>
#include <scene/planet/planet_heightmap.h>

#include <algorithm>
#include <cmath>
#include <filesystem>

#include "device.h"

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

void PlanetSystem::clear_terrain_patch_cache(TerrainState &state)
{
    if (!_context)
    {
        return;
    }

    ResourceManager *rm = _context->getResources();
    FrameResources *frame = _context->currentFrame;

    if (rm)
    {
        for (TerrainPatch &p: state.patches)
        {
            if (p.vertex_buffer.buffer == VK_NULL_HANDLE)
            {
                continue;
            }

            const AllocatedBuffer vb = p.vertex_buffer;
            if (frame)
            {
                frame->_deletionQueue.push_function([rm, vb]() { rm->destroy_buffer(vb); });
            }
            else
            {
                rm->destroy_buffer(vb);
            }

            p.vertex_buffer = {};
            p.vertex_buffer_address = 0;
        }
    }

    state.patch_lookup.clear();
    state.patch_lru.clear();
    state.patch_free.clear();
    state.patches.clear();
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
    }

    std::vector<uint32_t> indices;
    planet::build_cubesphere_patch_indices(indices, _earth_patch_resolution);
    _earth_patch_index_count = static_cast<uint32_t>(indices.size());
    _earth_patch_index_buffer =
            rm->upload_buffer(indices.data(),
                              indices.size() * sizeof(uint32_t),
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

    _earth_patch_material_layout = layoutBuilder.build(device->device(),
                                                       VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                                       nullptr,
                                                       VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
}

void PlanetSystem::ensure_terrain_material_constants_buffer(TerrainState &state, const PlanetBody &body)
{
    if (state.material_constants_buffer.buffer != VK_NULL_HANDLE)
    {
        if (!_context)
        {
            return;
        }

        DeviceManager *device = _context->getDevice();
        if (!device)
        {
            return;
        }

        const bool same_constants =
                state.bound_base_color == body.base_color &&
                state.bound_metallic == body.metallic &&
                state.bound_roughness == body.roughness &&
                state.bound_emission_factor == body.emission_factor;
        if (same_constants)
        {
            return;
        }

        const GLTFMetallic_Roughness::MaterialConstants constants =
                planet_helpers::make_planet_constants(body.base_color, body.metallic, body.roughness, body.emission_factor);

        VmaAllocationInfo allocInfo{};
        vmaGetAllocationInfo(device->allocator(), state.material_constants_buffer.allocation, &allocInfo);
        auto *mapped = static_cast<GLTFMetallic_Roughness::MaterialConstants *>(allocInfo.pMappedData);
        if (mapped)
        {
            *mapped = constants;
            vmaFlushAllocation(device->allocator(),
                               state.material_constants_buffer.allocation,
                               0,
                               sizeof(constants));

            state.bound_base_color = body.base_color;
            state.bound_metallic = body.metallic;
            state.bound_roughness = body.roughness;
            state.bound_emission_factor = body.emission_factor;
        }
        return;
    }

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

    const GLTFMetallic_Roughness::MaterialConstants constants =
            planet_helpers::make_planet_constants(body.base_color, body.metallic, body.roughness, body.emission_factor);

    state.material_constants_buffer =
            rm->create_buffer(sizeof(GLTFMetallic_Roughness::MaterialConstants),
                              VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                              VMA_MEMORY_USAGE_CPU_TO_GPU);

    if (state.material_constants_buffer.buffer == VK_NULL_HANDLE)
    {
        return;
    }

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(device->allocator(), state.material_constants_buffer.allocation, &allocInfo);
    auto *mapped = static_cast<GLTFMetallic_Roughness::MaterialConstants *>(allocInfo.pMappedData);
    if (mapped)
    {
        *mapped = constants;
        vmaFlushAllocation(device->allocator(), state.material_constants_buffer.allocation, 0, sizeof(constants));

        state.bound_base_color = body.base_color;
        state.bound_metallic = body.metallic;
        state.bound_roughness = body.roughness;
        state.bound_emission_factor = body.emission_factor;
    }
}

void PlanetSystem::ensure_terrain_face_materials(TerrainState &state, const PlanetBody &body)
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
    ensure_terrain_material_constants_buffer(state, body);

    if (_earth_patch_material_layout == VK_NULL_HANDLE ||
        state.material_constants_buffer.buffer == VK_NULL_HANDLE)
    {
        return;
    }

    if (!_earth_patch_material_allocator_initialized)
    {
        std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> sizes = {
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1},
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 6},
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

    const std::string desired_emission_dir = body.terrain_emission_dir;
    const bool emission_dir_changed = desired_emission_dir != state.bound_emission_dir;
    if (emission_dir_changed)
    {
        state.bound_emission_dir = desired_emission_dir;
    }

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
                                0,
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
            writer.update_set(device->device(), mat.materialSet);

            if (textures && tileSampler != VK_NULL_HANDLE && !desired_albedo_dir.empty())
            {
                const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                const std::string rel = fmt::format("{}/{}.ktx2", desired_albedo_dir, planet::cube_face_name(face));

                TextureCache::TextureKey tk{};
                tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
                tk.path = assets->assetPath(rel);
                tk.srgb = true;
                tk.mipmapped = true;

                TextureCache::TextureHandle h = textures->request(tk, tileSampler);
                textures->watchBinding(h, mat.materialSet, 1u, tileSampler, checker);
            }

            if (textures && tileSampler != VK_NULL_HANDLE && !desired_emission_dir.empty())
            {
                const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                // Try .ktx2 first, then .png
                std::string rel = fmt::format("{}/{}.ktx2", desired_emission_dir, planet::cube_face_name(face));
                std::string abs_path = assets->assetPath(rel);
                if (!std::filesystem::exists(abs_path))
                {
                    rel = fmt::format("{}/{}.png", desired_emission_dir, planet::cube_face_name(face));
                    abs_path = assets->assetPath(rel);
                }

                TextureCache::TextureKey tk{};
                tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
                tk.path = abs_path;
                tk.srgb = true;
                tk.mipmapped = true;

                TextureCache::TextureHandle h = textures->request(tk, tileSampler);
                textures->watchBinding(h, mat.materialSet, 5u, tileSampler, black);
            }
        }
        else if ((albedo_dir_changed || emission_dir_changed) && textures && tileSampler != VK_NULL_HANDLE)
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
            if (emission_dir_changed)
            {
                writer.write_image(5,
                                   black,
                                   tileSampler,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            }
            writer.update_set(device->device(), mat.materialSet);

            if (albedo_dir_changed && !desired_albedo_dir.empty())
            {
                const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                const std::string rel = fmt::format("{}/{}.ktx2", desired_albedo_dir, planet::cube_face_name(face));

                TextureCache::TextureKey tk{};
                tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
                tk.path = assets->assetPath(rel);
                tk.srgb = true;
                tk.mipmapped = true;

                TextureCache::TextureHandle h = textures->request(tk, tileSampler);
                textures->watchBinding(h, mat.materialSet, 1u, tileSampler, checker);
            }

            if (emission_dir_changed && !desired_emission_dir.empty())
            {
                const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                // Try .ktx2 first, then .png
                std::string rel = fmt::format("{}/{}.ktx2", desired_emission_dir, planet::cube_face_name(face));
                std::string abs_path = assets->assetPath(rel);
                if (!std::filesystem::exists(abs_path))
                {
                    rel = fmt::format("{}/{}.png", desired_emission_dir, planet::cube_face_name(face));
                    abs_path = assets->assetPath(rel);
                }

                TextureCache::TextureKey tk{};
                tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
                tk.path = abs_path;
                tk.srgb = true;
                tk.mipmapped = true;

                TextureCache::TextureHandle h = textures->request(tk, tileSampler);
                textures->watchBinding(h, mat.materialSet, 5u, tileSampler, black);
            }
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
    }

    if (!want_height)
    {
        for (planet::HeightFace &f: state.height_faces)
        {
            f = {};
        }
        return;
    }

    AssetManager *assets = _context->assets;
    std::array<planet::HeightFace, 6> loaded_faces{};
    bool ok = true;
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
        loaded_faces[face_index] = std::move(face_data);
    }

    if (!ok)
    {
        // If this was a retry (parameters didn't change), keep existing geometry and retry later.
        // If parameters changed, we already cleared meshes/faces above.
        return;
    }

    state.height_faces = std::move(loaded_faces);

    if (!changed)
    {
        // Recovered height data after a previous failure; regenerate meshes so displacement applies.
        clear_terrain_patch_cache(state);
        state.patch_cache_dirty = false;
    }
}

PlanetSystem::TerrainPatch *PlanetSystem::get_or_create_terrain_patch(TerrainState &state,
                                                                      const PlanetBody &body,
                                                                      const planet::PatchKey &key,
                                                                      uint32_t frame_index,
                                                                      uint8_t edge_stitch_mask)
{
    uint32_t reuse_idx = UINT32_MAX;
    if (TerrainPatch *p = find_terrain_patch(state, key))
    {
        p->last_used_frame = frame_index;
        state.patch_lru.splice(state.patch_lru.begin(), state.patch_lru, p->lru_it);
        if (p->edge_stitch_mask == edge_stitch_mask)
        {
            return p;
        }

        reuse_idx = static_cast<uint32_t>(p - state.patches.data());
        if (_context && p->vertex_buffer.buffer != VK_NULL_HANDLE)
        {
            ResourceManager *rm_existing = _context->getResources();
            FrameResources *frame_existing = _context->currentFrame;
            if (rm_existing)
            {
                const AllocatedBuffer vb_old = p->vertex_buffer;
                if (frame_existing)
                {
                    frame_existing->_deletionQueue.push_function([rm_existing, vb_old]() { rm_existing->destroy_buffer(vb_old); });
                }
                else
                {
                    rm_existing->destroy_buffer(vb_old);
                }
            }
            p->vertex_buffer = {};
            p->vertex_buffer_address = 0;
        }
    }

    if (!_context)
    {
        return nullptr;
    }

    ResourceManager *rm = _context->getResources();
    DeviceManager *device = _context->getDevice();
    if (!rm || !device)
    {
        return nullptr;
    }

    if (_earth_patch_index_buffer.buffer == VK_NULL_HANDLE || _earth_patch_index_count == 0)
    {
        return nullptr;
    }

    const glm::vec4 vertex_color =
            _earth_debug_tint_patches_by_lod ? planet_helpers::debug_color_for_level(key.level) : glm::vec4(1.0f);

    thread_local std::vector<Vertex> scratch_vertices;
    const uint32_t safe_res = std::max(2u, _earth_patch_resolution);
    scratch_vertices.reserve(
        static_cast<size_t>(safe_res) * static_cast<size_t>(safe_res) +
        static_cast<size_t>(4u) * static_cast<size_t>(safe_res));
    const glm::dvec3 patch_center_dir =
            planet::build_cubesphere_patch_vertices(scratch_vertices,
                                                    body.radius_m,
                                                    key.face,
                                                    key.level,
                                                    key.x,
                                                    key.y,
                                                    safe_res,
                                                    vertex_color);

    if (scratch_vertices.empty())
    {
        return nullptr;
    }

    if (body.terrain_height_max_m > 0.0)
    {
        const uint32_t face_index = static_cast<uint32_t>(key.face);
        if (face_index < state.height_faces.size())
        {
            const planet::HeightFace &height_face = state.height_faces[face_index];
            if (height_face.width > 0 && height_face.height > 0 && !height_face.texels.empty())
            {
                const float scale = static_cast<float>(body.terrain_height_max_m);
                constexpr float kFaceEdgeEpsilon = 1e-6f;
                for (Vertex &v: scratch_vertices)
                {
                    float h01 = 0.0f;

                    // On cube-face boundaries (u/v at 0 or 1), sample via direction mapping so
                    // both neighboring faces resolve to the same boundary samples.
                    const bool on_cube_face_edge =
                            (v.uv_x <= kFaceEdgeEpsilon) ||
                            (v.uv_x >= 1.0f - kFaceEdgeEpsilon) ||
                            (v.uv_y <= kFaceEdgeEpsilon) ||
                            (v.uv_y >= 1.0f - kFaceEdgeEpsilon);

                    if (on_cube_face_edge)
                    {
                        planet::CubeFace sample_face = key.face;
                        double sample_u = static_cast<double>(v.uv_x);
                        double sample_v = static_cast<double>(v.uv_y);
                        const glm::vec3 n = glm::normalize(v.normal);
                        const glm::dvec3 dir(static_cast<double>(n.x),
                                             static_cast<double>(n.y),
                                             static_cast<double>(n.z));
                        if (planet::cubesphere_direction_to_face_uv(dir, sample_face, sample_u, sample_v))
                        {
                            const uint32_t sample_face_index = static_cast<uint32_t>(sample_face);
                            if (sample_face_index < state.height_faces.size())
                            {
                                const planet::HeightFace &sample_height_face = state.height_faces[sample_face_index];
                                if (sample_height_face.width > 0 &&
                                    sample_height_face.height > 0 &&
                                    !sample_height_face.texels.empty())
                                {
                                    h01 = planet::sample_height(sample_height_face,
                                                                static_cast<float>(sample_u),
                                                                static_cast<float>(sample_v));
                                }
                                else
                                {
                                    h01 = planet::sample_height(height_face, v.uv_x, v.uv_y);
                                }
                            }
                            else
                            {
                                h01 = planet::sample_height(height_face, v.uv_x, v.uv_y);
                            }
                        }
                        else
                        {
                            h01 = planet::sample_height(height_face, v.uv_x, v.uv_y);
                        }
                    }
                    else
                    {
                        h01 = planet::sample_height(height_face, v.uv_x, v.uv_y);
                    }

                    const float h_m = h01 * scale;
                    v.position += v.normal * h_m;
                }

                planet_helpers::stitch_patch_edges_to_parent_grid(scratch_vertices, safe_res, edge_stitch_mask);
                planet_helpers::recompute_patch_normals(scratch_vertices, safe_res);
                planet_helpers::refine_patch_edge_normals_from_height(scratch_vertices,
                                                      safe_res,
                                                      patch_center_dir,
                                                      body.radius_m,
                                                      key.level,
                                                      edge_stitch_mask,
                                                      body.terrain_height_max_m,
                                                      state.height_faces);
            }
        }
    }

    planet_helpers::reinforce_patch_skirts(scratch_vertices,
                           safe_res,
                           patch_center_dir,
                           body.radius_m,
                           key.level);

    const planet_helpers::PatchBoundsData bounds = planet_helpers::compute_patch_bounds(scratch_vertices);

    AllocatedBuffer vb =
            rm->upload_buffer(scratch_vertices.data(),
                              scratch_vertices.size() * sizeof(Vertex),
                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                              VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR);
    if (vb.buffer == VK_NULL_HANDLE)
    {
        return nullptr;
    }

    VkBufferDeviceAddressInfo addrInfo{.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    addrInfo.buffer = vb.buffer;
    VkDeviceAddress addr = vkGetBufferDeviceAddress(device->device(), &addrInfo);

    uint32_t idx = 0;
    if (reuse_idx != UINT32_MAX)
    {
        idx = reuse_idx;
    }
    else if (!state.patch_free.empty())
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

    TerrainPatch &p = state.patches[idx];
    p.key = key;
    p.state = TerrainPatchState::Ready;
    p.edge_stitch_mask = edge_stitch_mask;
    p.vertex_buffer = vb;
    p.vertex_buffer_address = addr;
    p.bounds_origin = bounds.origin;
    p.bounds_extents = bounds.extents;
    p.bounds_sphere_radius = bounds.sphere_radius;
    p.patch_center_dir = patch_center_dir;
    p.last_used_frame = frame_index;
    if (reuse_idx == UINT32_MAX)
    {
        state.patch_lru.push_front(idx);
        p.lru_it = state.patch_lru.begin();
        state.patch_lookup.emplace(key, idx);
    }
    return &p;
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
