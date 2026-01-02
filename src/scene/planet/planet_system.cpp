#include "planet_system.h"

#include <core/context.h>
#include <core/device/resource.h>
#include <core/frame/resources.h>
#include <core/types.h>
#include <core/assets/manager.h>
#include <render/materials.h>
#include <render/primitives.h>
#include <core/pipeline/sampler.h>
#include <scene/planet/cubesphere.h>
#include <scene/tangent_space.h>
#include <scene/vk_scene.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

#include "device.h"

namespace
{
    constexpr double kEarthRadiusM = 6378137.0;    // WGS84 equatorial radius
    constexpr double kMoonRadiusM = 1737400.0;     // mean radius
    constexpr double kMoonDistanceM = 384400000.0; // mean Earth-Moon distance

    struct PatchBoundsData
    {
        glm::vec3 origin{0.0f};
        glm::vec3 extents{0.5f};
        float sphere_radius{0.5f};
    };

    PatchBoundsData compute_patch_bounds(const std::vector<Vertex> &vertices)
    {
        PatchBoundsData b{};
        if (vertices.empty())
        {
            return b;
        }

        glm::vec3 minpos = vertices[0].position;
        glm::vec3 maxpos = vertices[0].position;
        for (const auto &v : vertices)
        {
            minpos = glm::min(minpos, v.position);
            maxpos = glm::max(maxpos, v.position);
        }
        b.origin = (maxpos + minpos) * 0.5f;
        b.extents = (maxpos - minpos) * 0.5f;
        b.sphere_radius = glm::length(b.extents);
        return b;
    }

    GLTFMetallic_Roughness::MaterialConstants make_planet_constants()
    {
        GLTFMetallic_Roughness::MaterialConstants c{};
        c.colorFactors = glm::vec4(1.0f);
        // metal_rough_factors.x = metallic, .y = roughness
        c.metal_rough_factors = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
        return c;
    }

    GLTFMetallic_Roughness::MaterialConstants make_planet_constants(const glm::vec4 &base_color,
                                                                    float metallic,
                                                                    float roughness)
    {
        GLTFMetallic_Roughness::MaterialConstants c = make_planet_constants();
        c.colorFactors = base_color;
        c.metal_rough_factors = glm::vec4(metallic, roughness, 0.0f, 0.0f);
        return c;
    }

    glm::vec4 debug_color_for_level(uint32_t level)
    {
        const float t = static_cast<float>(level) * 0.37f;
        const float r = 0.35f + 0.65f * std::sin(t + 0.0f);
        const float g = 0.35f + 0.65f * std::sin(t + 2.1f);
        const float b = 0.35f + 0.65f * std::sin(t + 4.2f);
        return glm::vec4(r, g, b, 1.0f);
    }
} // namespace

void PlanetSystem::init(EngineContext *context)
{
    _context = context;
}

void PlanetSystem::set_earth_debug_tint_patches_by_lod(bool enabled)
{
    if (_earth_debug_tint_patches_by_lod == enabled)
    {
        return;
    }
    _earth_debug_tint_patches_by_lod = enabled;
    _earth_patch_cache_dirty = true;
}

void PlanetSystem::set_earth_patch_resolution(uint32_t resolution)
{
    const uint32_t clamped = std::max(2u, resolution);
    if (_earth_patch_resolution == clamped)
    {
        return;
    }

    _earth_patch_resolution = clamped;
    _earth_patch_cache_dirty = true;
}

void PlanetSystem::cleanup()
{
    if (!_context)
    {
        return;
    }

    TextureCache *textures = _context->textures;
    if (textures)
    {
        for (MaterialInstance &mat : _earth_face_materials)
        {
            if (mat.materialSet != VK_NULL_HANDLE)
            {
                textures->unwatchSet(mat.materialSet);
            }
        }
    }

    ResourceManager *rm = _context->getResources();
    if (rm)
    {
        for (EarthPatch &p : _earth_patches)
        {
            if (p.vertex_buffer.buffer != VK_NULL_HANDLE)
            {
                rm->destroy_buffer(p.vertex_buffer);
                p.vertex_buffer = {};
                p.vertex_buffer_address = 0;
            }
        }

        if (_earth_patch_index_buffer.buffer != VK_NULL_HANDLE)
        {
            rm->destroy_buffer(_earth_patch_index_buffer);
            _earth_patch_index_buffer = {};
        }

        if (_earth_patch_material_constants_buffer.buffer != VK_NULL_HANDLE)
        {
            rm->destroy_buffer(_earth_patch_material_constants_buffer);
            _earth_patch_material_constants_buffer = {};
        }
    }

    if (_earth_patch_material_allocator_initialized)
    {
        if (DeviceManager *device = _context->getDevice())
        {
            _earth_patch_material_allocator.destroy_pools(device->device());
        }
        _earth_patch_material_allocator_initialized = false;
    }

    if (_earth_patch_material_layout != VK_NULL_HANDLE)
    {
        if (DeviceManager *device = _context->getDevice())
        {
            vkDestroyDescriptorSetLayout(device->device(), _earth_patch_material_layout, nullptr);
        }
        _earth_patch_material_layout = VK_NULL_HANDLE;
    }

    _earth_patch_lookup.clear();
    _earth_patch_lru.clear();
    _earth_patch_free.clear();
    _earth_patches.clear();
    _earth_face_materials = {};

    _earth_patch_index_count = 0;
    _earth_patch_index_resolution = 0;
    _earth_patch_frame_stamp = 0;

    _bodies.clear();
}

PlanetSystem::PlanetBody *PlanetSystem::find_body_by_name(std::string_view name)
{
    ensure_bodies_created();
    for (PlanetBody &b : _bodies)
    {
        if (b.name == name)
        {
            return &b;
        }
    }
    return nullptr;
}

PlanetSystem::PlanetBody *PlanetSystem::create_mesh_planet(const MeshPlanetCreateInfo &info)
{
    if (info.name.empty())
    {
        return nullptr;
    }

    // Avoid implicit default creation when the user is explicitly managing planets.
    for (const PlanetBody &b : _bodies)
    {
        if (b.name == info.name)
        {
            return nullptr;
        }
    }

    PlanetBody body{};
    body.name = info.name;
    body.center_world = info.center_world;
    body.radius_m = info.radius_m;
    body.visible = info.visible;
    body.terrain = false;

    if (_context && _context->assets)
    {
        AssetManager *assets = _context->assets;

        const std::string asset_name = fmt::format("Planet_{}", info.name);
        const GLTFMetallic_Roughness::MaterialConstants mc =
            make_planet_constants(info.base_color, info.metallic, info.roughness);

        body.material = assets->createMaterialFromConstants(asset_name, mc, MaterialPass::MainColor);

        std::vector<Vertex> verts;
        std::vector<uint32_t> inds;
        primitives::buildSphere(verts, inds,
                                static_cast<int>(std::max(3u, info.sectors)),
                                static_cast<int>(std::max(2u, info.stacks)));
        geom::generate_tangents(verts, inds);

        body.mesh = assets->createMesh(asset_name, verts, inds, body.material);
    }

    _bodies.push_back(std::move(body));
    return &_bodies.back();
}

bool PlanetSystem::destroy_planet(std::string_view name)
{
    if (name.empty())
    {
        return false;
    }

    auto it = std::find_if(_bodies.begin(), _bodies.end(), [&](const PlanetBody &b) { return b.name == name; });
    if (it == _bodies.end())
    {
        return false;
    }

    // If we destroy the active terrain body, make sure we free its patch cache.
    if (it->terrain)
    {
        clear_earth_patch_cache();
        _earth_debug_stats = {};
    }

    if (it->mesh && _context && _context->assets)
    {
        AssetManager *assets = _context->assets;
        if (_context->currentFrame)
        {
            assets->removeMeshDeferred(it->mesh->name, _context->currentFrame->_deletionQueue);
        }
        else
        {
            assets->removeMesh(it->mesh->name);
        }
    }

    _bodies.erase(it);
    return true;
}

void PlanetSystem::clear_planets(bool destroy_mesh_assets)
{
    if (destroy_mesh_assets && _context && _context->assets)
    {
        AssetManager *assets = _context->assets;
        for (const PlanetBody &b : _bodies)
        {
            if (!b.mesh) continue;
            if (_context->currentFrame)
            {
                assets->removeMeshDeferred(b.mesh->name, _context->currentFrame->_deletionQueue);
            }
            else
            {
                assets->removeMesh(b.mesh->name);
            }
        }
    }

    // Terrain patches can be very large; clear them even if we keep the
    // shared index/material resources around.
    clear_earth_patch_cache();
    _earth_debug_stats = {};

    _bodies.clear();
}

void PlanetSystem::ensure_bodies_created()
{
    if (!_auto_create_defaults || !_bodies.empty())
    {
        return;
    }

    PlanetBody earth{};
    earth.name = "Earth";
    earth.center_world = WorldVec3(0.0, 0.0, 0.0);
    earth.radius_m = kEarthRadiusM;
    earth.terrain = true;

    PlanetBody moon{};
    moon.name = "Moon";
    moon.center_world = WorldVec3(kMoonDistanceM, 0.0, 0.0);
    moon.radius_m = kMoonRadiusM;
    moon.terrain = false;

    if (_context && _context->assets)
    {
        AssetManager *assets = _context->assets;

        // Earth: cube-sphere quadtree patches (Milestones B2-B4). Material is shared.
        {
            GLTFMetallic_Roughness::MaterialConstants mc = make_planet_constants();
            mc.colorFactors = glm::vec4(1.0f);
            earth.material = assets->createMaterialFromConstants("Planet_EarthMaterial", mc, MaterialPass::MainColor);
        }

        // Moon: constant albedo (no texture yet).
        {
            GLTFMetallic_Roughness::MaterialConstants mc = make_planet_constants();
            mc.colorFactors = glm::vec4(0.72f, 0.72f, 0.75f, 1.0f);

            moon.material = assets->createMaterialFromConstants("Planet_MoonMaterial", mc, MaterialPass::MainColor);

            std::vector<Vertex> verts;
            std::vector<uint32_t> inds;
            primitives::buildSphere(verts, inds, 48, 24);
            geom::generate_tangents(verts, inds);

            moon.mesh = assets->createMesh("Planet_MoonSphere", verts, inds, moon.material);
        }
    }

    _bodies.push_back(std::move(earth));
    _bodies.push_back(std::move(moon));
}

PlanetSystem::PlanetBody *PlanetSystem::find_terrain_body()
{
    for (PlanetBody &b : _bodies)
    {
        if (b.terrain)
        {
            return &b;
        }
    }
    return nullptr;
}

const PlanetSystem::PlanetBody *PlanetSystem::find_terrain_body() const
{
    for (const PlanetBody &b : _bodies)
    {
        if (b.terrain)
        {
            return &b;
        }
    }
    return nullptr;
}

PlanetSystem::EarthPatch *PlanetSystem::find_earth_patch(const planet::PatchKey &key)
{
    auto it = _earth_patch_lookup.find(key);
    if (it == _earth_patch_lookup.end())
    {
        return nullptr;
    }
    const uint32_t idx = it->second;
    if (idx >= _earth_patches.size())
    {
        return nullptr;
    }
    return &_earth_patches[idx];
}

void PlanetSystem::clear_earth_patch_cache()
{
    if (!_context)
    {
        return;
    }

    ResourceManager *rm = _context->getResources();
    FrameResources *frame = _context->currentFrame;

    if (rm)
    {
        for (EarthPatch &p : _earth_patches)
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

    _earth_patch_lookup.clear();
    _earth_patch_lru.clear();
    _earth_patch_free.clear();
    _earth_patches.clear();
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

        // Destroy per-patch vertex buffers.
        for (const auto &kv : _earth_patch_lookup)
        {
            const uint32_t idx = kv.second;
            if (idx >= _earth_patches.size())
            {
                continue;
            }
            EarthPatch &p = _earth_patches[idx];
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
        }

        _earth_patch_lookup.clear();
        _earth_patch_lru.clear();
        _earth_patch_free.clear();
        _earth_patches.clear();

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

void PlanetSystem::ensure_earth_patch_material_constants_buffer()
{
    if (_earth_patch_material_constants_buffer.buffer != VK_NULL_HANDLE)
    {
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

    const GLTFMetallic_Roughness::MaterialConstants constants = make_planet_constants();

    _earth_patch_material_constants_buffer =
        rm->create_buffer(sizeof(GLTFMetallic_Roughness::MaterialConstants),
                          VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                          VMA_MEMORY_USAGE_CPU_TO_GPU);

    if (_earth_patch_material_constants_buffer.buffer == VK_NULL_HANDLE)
    {
        return;
    }

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(device->allocator(), _earth_patch_material_constants_buffer.allocation, &allocInfo);
    auto *mapped = static_cast<GLTFMetallic_Roughness::MaterialConstants *>(allocInfo.pMappedData);
    if (mapped)
    {
        *mapped = constants;
        vmaFlushAllocation(device->allocator(), _earth_patch_material_constants_buffer.allocation, 0, sizeof(constants));
    }
}

void PlanetSystem::ensure_earth_face_materials(const PlanetBody &earth)
{
    if (!_context || !earth.material)
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
    ensure_earth_patch_material_constants_buffer();

    if (_earth_patch_material_layout == VK_NULL_HANDLE ||
        _earth_patch_material_constants_buffer.buffer == VK_NULL_HANDLE)
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

    VkImageView checker = assets->fallback_checkerboard_view();
    VkImageView white = assets->fallback_white_view();
    VkImageView flatNormal = assets->fallback_flat_normal_view();
    VkImageView black = assets->fallback_black_view();

    if (checker == VK_NULL_HANDLE) checker = white;
    if (white == VK_NULL_HANDLE) white = checker;
    if (flatNormal == VK_NULL_HANDLE) flatNormal = white;
    if (black == VK_NULL_HANDLE) black = white;

    auto face_legacy = [](planet::CubeFace f) -> const char * {
        switch (f)
        {
            case planet::CubeFace::PosX: return "px";
            case planet::CubeFace::NegX: return "nx";
            case planet::CubeFace::PosY: return "py";
            case planet::CubeFace::NegY: return "ny";
            case planet::CubeFace::PosZ: return "pz";
            case planet::CubeFace::NegZ: return "nz";
        }
        return "px";
    };

    for (size_t face_index = 0; face_index < _earth_face_materials.size(); ++face_index)
    {
        MaterialInstance &mat = _earth_face_materials[face_index];

        mat.pipeline = earth.material->data.pipeline;
        mat.passType = earth.material->data.passType;

        if (mat.materialSet == VK_NULL_HANDLE)
        {
            mat.materialSet = _earth_patch_material_allocator.allocate(device->device(), _earth_patch_material_layout);

            DescriptorWriter writer;
            writer.write_buffer(0,
                                _earth_patch_material_constants_buffer.buffer,
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

            if (textures && tileSampler != VK_NULL_HANDLE)
            {
                const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                const std::string rel = fmt::format("planets/earth/albedo/L0/{}.ktx2", face_legacy(face));

                TextureCache::TextureKey tk{};
                tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
                tk.path = assets->assetPath(rel);
                tk.srgb = true;
                tk.mipmapped = true;

                TextureCache::TextureHandle h = textures->request(tk, tileSampler);
                textures->watchBinding(h, mat.materialSet, 1u, tileSampler, checker);
                textures->pin(h);
            }
        }
    }
}

PlanetSystem::EarthPatch *PlanetSystem::get_or_create_earth_patch(const PlanetBody &earth,
                                                                  const planet::PatchKey &key,
                                                                  uint32_t frame_index)
{
    if (EarthPatch *p = find_earth_patch(key))
    {
        p->last_used_frame = frame_index;
        _earth_patch_lru.splice(_earth_patch_lru.begin(), _earth_patch_lru, p->lru_it);
        return p;
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
        _earth_debug_tint_patches_by_lod ? debug_color_for_level(key.level) : glm::vec4(1.0f);

    thread_local std::vector<Vertex> scratch_vertices;
    const uint32_t safe_res = std::max(2u, _earth_patch_resolution);
    scratch_vertices.reserve(
        static_cast<size_t>(safe_res) * static_cast<size_t>(safe_res) +
        static_cast<size_t>(4u) * static_cast<size_t>(safe_res));
    const glm::dvec3 patch_center_dir =
        planet::build_cubesphere_patch_vertices(scratch_vertices,
                                                earth.radius_m,
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

    const PatchBoundsData bounds = compute_patch_bounds(scratch_vertices);

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
    if (!_earth_patch_free.empty())
    {
        idx = _earth_patch_free.back();
        _earth_patch_free.pop_back();
    }
    else
    {
        idx = static_cast<uint32_t>(_earth_patches.size());
        _earth_patches.emplace_back();
    }

    if (idx >= _earth_patches.size())
    {
        return nullptr;
    }

    EarthPatch &p = _earth_patches[idx];
    p.key = key;
    p.state = EarthPatchState::Ready;
    p.vertex_buffer = vb;
    p.vertex_buffer_address = addr;
    p.bounds_origin = bounds.origin;
    p.bounds_extents = bounds.extents;
    p.bounds_sphere_radius = bounds.sphere_radius;
    p.patch_center_dir = patch_center_dir;
    p.last_used_frame = frame_index;
    _earth_patch_lru.push_front(idx);
    p.lru_it = _earth_patch_lru.begin();

    _earth_patch_lookup.emplace(key, idx);
    return &p;
}

void PlanetSystem::trim_earth_patch_cache()
{
    if (_earth_patch_cache_max == 0)
    {
        return;
    }

    if (_earth_patch_lookup.size() <= static_cast<size_t>(_earth_patch_cache_max))
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
    const uint32_t now = _earth_patch_frame_stamp;

    size_t guard = 0;
    const size_t guard_limit = _earth_patch_lru.size();

    while (_earth_patch_lookup.size() > static_cast<size_t>(_earth_patch_cache_max) && !_earth_patch_lru.empty())
    {
        if (guard++ >= guard_limit)
        {
            // No evictable patches (all used this frame). Avoid thrashing.
            break;
        }

        const uint32_t idx = _earth_patch_lru.back();
        if (idx >= _earth_patches.size())
        {
            _earth_patch_lru.pop_back();
            continue;
        }

        EarthPatch &p = _earth_patches[idx];
        if (p.last_used_frame == now)
        {
            // Keep all patches referenced this frame.
            _earth_patch_lru.splice(_earth_patch_lru.begin(), _earth_patch_lru, p.lru_it);
            continue;
        }

        // Made progress: we found an evictable patch.
        guard = 0;

        _earth_patch_lru.erase(p.lru_it);
        _earth_patch_lookup.erase(p.key);

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

        p = EarthPatch{};
        _earth_patch_free.push_back(idx);
    }
}

void PlanetSystem::update_and_emit(const SceneManager &scene, DrawContext &draw_context)
{
    if (!_enabled)
    {
        return;
    }

    ensure_bodies_created();

    const WorldVec3 origin_world = scene.get_world_origin();

    // Terrain body: quadtree patches (defaults to Earth).
    {
        using Clock = std::chrono::steady_clock;

        PlanetBody *earth = find_terrain_body();
        if (earth && earth->visible && earth->material && _context)
        {
            if (_earth_patch_cache_dirty)
            {
                clear_earth_patch_cache();
                _earth_patch_cache_dirty = false;
            }

            const Clock::time_point t0 = Clock::now();

            _earth_quadtree.set_settings(_earth_quadtree_settings);

            const VkExtent2D logical_extent = _context->getLogicalRenderExtent();
            const WorldVec3 cam_world = scene.getMainCamera().position_world;

            const Clock::time_point t_q0 = Clock::now();
            _earth_quadtree.update(earth->center_world,
                                   earth->radius_m,
                                   cam_world,
                                   origin_world,
                                   scene.getSceneData(),
                                   logical_extent,
                                   _earth_patch_resolution);
            const Clock::time_point t_q1 = Clock::now();

            ensure_earth_patch_index_buffer();
            ensure_earth_face_materials(*earth);
            if (_context->textures)
            {
                for (const MaterialInstance &mat : _earth_face_materials)
                {
                    if (mat.materialSet != VK_NULL_HANDLE)
                    {
                        _context->textures->markSetUsed(mat.materialSet, _context->frameIndex);
                    }
                }
            }

            size_t desired_capacity =
                static_cast<size_t>(_earth_patches.size()) +
                static_cast<size_t>(_earth_patch_create_budget_per_frame) +
                32u;
            if (_earth_patch_cache_max != 0)
            {
                desired_capacity = std::max(
                    desired_capacity,
                    static_cast<size_t>(_earth_patch_cache_max) +
                        static_cast<size_t>(_earth_patch_create_budget_per_frame) +
                        32u);
            }
            if (_earth_patches.capacity() < desired_capacity)
            {
                _earth_patches.reserve(desired_capacity);
            }

            uint32_t created_patches = 0;
            double ms_patch_create = 0.0;
            const uint32_t max_create = _earth_patch_create_budget_per_frame;
            const double max_create_ms =
                (_earth_patch_create_budget_ms > 0.0f) ? static_cast<double>(_earth_patch_create_budget_ms) : 0.0;
            const uint32_t frame_index = ++_earth_patch_frame_stamp;

            const Clock::time_point t_emit0 = Clock::now();

            // Patch creation priority: create higher-LOD (smaller) patches first so we fill
            // near-camera terrain before spending budget on far patches.
            std::vector<planet::PatchKey> create_queue = _earth_quadtree.visible_leaves();
            std::sort(create_queue.begin(), create_queue.end(),
                      [](const planet::PatchKey &a, const planet::PatchKey &b)
                      {
                          if (a.level != b.level) return a.level > b.level;
                          if (a.face != b.face) return a.face < b.face;
                          if (a.x != b.x) return a.x < b.x;
                          return a.y < b.y;
                      });

            for (const planet::PatchKey &k : create_queue)
            {
                if (find_earth_patch(k))
                {
                    continue;
                }

                const bool hit_count_budget = (max_create != 0u) && (created_patches >= max_create);
                const bool hit_time_budget = (max_create_ms > 0.0) && (ms_patch_create >= max_create_ms);
                if (hit_count_budget || hit_time_budget)
                {
                    break;
                }

                const Clock::time_point t_c0 = Clock::now();
                EarthPatch *patch = get_or_create_earth_patch(*earth, k, frame_index);
                const Clock::time_point t_c1 = Clock::now();

                if (patch)
                {
                    created_patches++;
                }
                ms_patch_create += std::chrono::duration<double, std::milli>(t_c1 - t_c0).count();
            }

            std::vector<uint32_t> ready_patch_indices;
            ready_patch_indices.reserve(_earth_quadtree.visible_leaves().size());

            for (const planet::PatchKey &k : _earth_quadtree.visible_leaves())
            {
                EarthPatch *patch = find_earth_patch(k);
                if (!patch)
                {
                    continue;
                }

                patch->last_used_frame = frame_index;
                _earth_patch_lru.splice(_earth_patch_lru.begin(), _earth_patch_lru, patch->lru_it);

                {
                    const uint32_t idx = static_cast<uint32_t>(patch - _earth_patches.data());
                    ready_patch_indices.push_back(idx);
                }
            }

            for (uint32_t idx : ready_patch_indices)
            {
                if (idx >= _earth_patches.size())
                {
                    continue;
                }

                EarthPatch &patch = _earth_patches[idx];
                if (patch.state != EarthPatchState::Ready ||
                    patch.vertex_buffer.buffer == VK_NULL_HANDLE ||
                    patch.vertex_buffer_address == 0 ||
                    _earth_patch_index_buffer.buffer == VK_NULL_HANDLE ||
                    _earth_patch_index_count == 0)
                {
                    continue;
                }

                const uint32_t face_index = static_cast<uint32_t>(patch.key.face);
                if (face_index >= _earth_face_materials.size())
                {
                    continue;
                }
                MaterialInstance *material = &_earth_face_materials[face_index];
                if (material->materialSet == VK_NULL_HANDLE || material->pipeline == nullptr)
                {
                    continue;
                }

                const WorldVec3 patch_center_world =
                    earth->center_world + patch.patch_center_dir * earth->radius_m;
                const glm::vec3 patch_center_local = world_to_local(patch_center_world, origin_world);
                const glm::mat4 transform = glm::translate(glm::mat4(1.0f), patch_center_local);

                Bounds b{};
                b.origin = patch.bounds_origin;
                b.extents = patch.bounds_extents;
                b.sphereRadius = patch.bounds_sphere_radius;
                b.type = BoundsType::Box;

                RenderObject obj{};
                obj.indexCount = _earth_patch_index_count;
                obj.firstIndex = 0;
                obj.indexBuffer = _earth_patch_index_buffer.buffer;
                obj.vertexBuffer = patch.vertex_buffer.buffer;
                obj.vertexBufferAddress = patch.vertex_buffer_address;
                obj.material = material;
                obj.bounds = b;
                obj.transform = transform;
                // Planet terrain patches are not meaningful RT occluders; skip BLAS/TLAS builds.
                obj.sourceMesh = nullptr;
                obj.surfaceIndex = 0;
                obj.objectID = draw_context.nextID++;
                obj.ownerType = RenderObject::OwnerType::MeshInstance;
                obj.ownerName = earth->name;

                draw_context.OpaqueSurfaces.push_back(obj);
            }
            const Clock::time_point t_emit1 = Clock::now();

            trim_earth_patch_cache();

            const uint32_t visible_patches = static_cast<uint32_t>(_earth_quadtree.visible_leaves().size());
            const uint32_t n = _earth_patch_resolution;
            const uint32_t patch_tris = (n >= 2u) ? (2u * (n - 1u) * (n + 3u)) : 0u;
            const uint32_t estimated_tris = patch_tris * visible_patches;

            _earth_debug_stats = {};
            _earth_debug_stats.quadtree = _earth_quadtree.stats();
            _earth_debug_stats.visible_patches = visible_patches;
            _earth_debug_stats.created_patches = created_patches;
            _earth_debug_stats.patch_cache_size = static_cast<uint32_t>(_earth_patch_lookup.size());
            _earth_debug_stats.estimated_triangles = estimated_tris;
            _earth_debug_stats.ms_quadtree = static_cast<float>(std::chrono::duration<double, std::milli>(t_q1 - t_q0).count());
            _earth_debug_stats.ms_patch_create = static_cast<float>(ms_patch_create);
            const double ms_emit_total = std::chrono::duration<double, std::milli>(t_emit1 - t_emit0).count();
            _earth_debug_stats.ms_emit = static_cast<float>(std::max(0.0, ms_emit_total - ms_patch_create));
            _earth_debug_stats.ms_total = static_cast<float>(std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
        }
    }

    // Other bodies (moon etc.): regular mesh instances.
    for (size_t body_index = 0; body_index < _bodies.size(); ++body_index)
    {
        PlanetBody &b = _bodies[body_index];

        if (b.terrain)
        {
            continue;
        }

        if (!b.visible || !b.mesh || b.mesh->surfaces.empty())
        {
            continue;
        }

        const glm::vec3 t_local = world_to_local(b.center_world, origin_world);
        const float r = static_cast<float>(b.radius_m);
        const glm::vec3 s = glm::vec3(r * 2.0f); // primitive sphere radius is 0.5
        const glm::quat q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        const glm::mat4 transform = make_trs_matrix(t_local, q, s);

        uint32_t surface_index = 0;
        for (const GeoSurface &surf : b.mesh->surfaces)
        {
            RenderObject obj{};
            obj.indexCount = surf.count;
            obj.firstIndex = surf.startIndex;
            obj.indexBuffer = b.mesh->meshBuffers.indexBuffer.buffer;
            obj.vertexBuffer = b.mesh->meshBuffers.vertexBuffer.buffer;
            obj.vertexBufferAddress = b.mesh->meshBuffers.vertexBufferAddress;
            obj.material = surf.material ? &surf.material->data : nullptr;
            obj.bounds = surf.bounds;
            obj.transform = transform;
            obj.sourceMesh = b.mesh.get();
            obj.surfaceIndex = surface_index++;
            obj.objectID = draw_context.nextID++;
            obj.ownerType = RenderObject::OwnerType::MeshInstance;
            obj.ownerName = b.name;

            if (obj.material && obj.material->passType == MaterialPass::Transparent)
            {
                draw_context.TransparentSurfaces.push_back(obj);
            }
            else
            {
                draw_context.OpaqueSurfaces.push_back(obj);
            }
        }
    }
}
