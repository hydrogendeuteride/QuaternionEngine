#include "planet_system.h"
#include "planet_patch_helpers.h"

#include <core/context.h>
#include <core/device/resource.h>
#include <core/frame/resources.h>
#include <core/types.h>
#include <core/assets/manager.h>
#include <render/materials.h>
#include <render/primitives.h>
#include <core/pipeline/sampler.h>
#include <scene/planet/cubesphere.h>
#include <scene/planet/planet_heightmap.h>
#include <scene/tangent_space.h>
#include <scene/vk_scene.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "device.h"

namespace
{
    using PatchKey = planet::PatchKey;
    using PatchKeySet = std::unordered_set<PatchKey, planet::PatchKeyHash>;
    using PatchLevelMap = std::unordered_map<PatchKey, uint32_t, planet::PatchKeyHash>;

    constexpr std::array<planet::CubeFace, 6> k_cube_faces{
        planet::CubeFace::PosX,
        planet::CubeFace::NegX,
        planet::CubeFace::PosY,
        planet::CubeFace::NegY,
        planet::CubeFace::PosZ,
        planet::CubeFace::NegZ,
    };

    PatchKey make_parent_key(const PatchKey &child)
    {
        return PatchKey{child.face, child.level - 1u, child.x >> 1u, child.y >> 1u};
    }

    void append_child_keys(const PatchKey &parent, std::vector<PatchKey> &out)
    {
        const uint32_t child_level = parent.level + 1u;
        const uint32_t child_x = parent.x * 2u;
        const uint32_t child_y = parent.y * 2u;

        out.push_back(PatchKey{parent.face, child_level, child_x + 0u, child_y + 0u});
        out.push_back(PatchKey{parent.face, child_level, child_x + 1u, child_y + 0u});
        out.push_back(PatchKey{parent.face, child_level, child_x + 0u, child_y + 1u});
        out.push_back(PatchKey{parent.face, child_level, child_x + 1u, child_y + 1u});
    }

    PatchKey child_on_leaf_path(const PatchKey &parent, const PatchKey &leaf)
    {
        const uint32_t child_level = parent.level + 1u;
        const uint32_t shift = leaf.level - child_level;
        const uint32_t child_x = parent.x * 2u + ((leaf.x >> shift) & 1u);
        const uint32_t child_y = parent.y * 2u + ((leaf.y >> shift) & 1u);
        return PatchKey{parent.face, child_level, child_x, child_y};
    }

    void append_pinned_patch_keys(uint32_t max_level, std::vector<PatchKey> &out)
    {
        for (planet::CubeFace face: k_cube_faces)
        {
            for (uint32_t level = 0u; level <= max_level; ++level)
            {
                const uint32_t tiles = 1u << level;
                for (uint32_t y = 0; y < tiles; ++y)
                {
                    for (uint32_t x = 0; x < tiles; ++x)
                    {
                        out.push_back(PatchKey{face, level, x, y});
                    }
                }
            }
        }
    }

    void append_required_split_groups_for_leaf(const PatchKey &leaf, std::vector<PatchKey> &out)
    {
        PatchKey current{leaf.face, 0u, 0u, 0u};
        while (current.level < leaf.level)
        {
            append_child_keys(current, out);
            current = child_on_leaf_path(current, leaf);
        }
    }

    float ocean_shell_offset_m(double radius_m)
    {
        const double scaled = radius_m * 1.0e-6;
        return static_cast<float>(std::max(2.0, scaled));
    }

    PatchLevelMap build_desired_max_levels(const std::vector<PatchKey> &desired_leaves)
    {
        PatchLevelMap desired_max_levels;
        desired_max_levels.reserve(desired_leaves.size() * 4u + 32u);

        for (const PatchKey &leaf: desired_leaves)
        {
            PatchKey current = leaf;
            while (true)
            {
                auto [it, inserted] = desired_max_levels.emplace(current, leaf.level);
                if (!inserted)
                {
                    it->second = std::max(it->second, leaf.level);
                }

                if (current.level == 0u)
                {
                    break;
                }
                current = make_parent_key(current);
            }
        }

        return desired_max_levels;
    }

    template<typename ReadyFn>
    bool all_children_ready(const PatchKey &parent, ReadyFn &&is_ready)
    {
        std::vector<PatchKey> children;
        children.reserve(4u);
        append_child_keys(parent, children);
        for (const PatchKey &child: children)
        {
            if (!is_ready(child))
            {
                return false;
            }
        }
        return true;
    }

    template<typename ReadyFn>
    std::vector<PatchKey> build_ready_render_cut(const std::vector<PatchKey> &desired_leaves, ReadyFn &&is_ready)
    {
        std::vector<PatchKey> render_keys;
        if (desired_leaves.empty())
        {
            return render_keys;
        }

        const PatchLevelMap desired_max_levels = build_desired_max_levels(desired_leaves);
        render_keys.reserve(k_cube_faces.size());
        for (planet::CubeFace face: k_cube_faces)
        {
            const PatchKey root{face, 0u, 0u, 0u};
            if (!desired_max_levels.contains(root) || !is_ready(root))
            {
                continue;
            }
            render_keys.push_back(root);
        }

        if (render_keys.empty())
        {
            return render_keys;
        }

        constexpr uint32_t k_max_balance_passes = 32u;
        for (uint32_t pass = 0u; pass < k_max_balance_passes; ++pass)
        {
            PatchKeySet render_set;
            render_set.reserve(render_keys.size() * 2u);
            uint32_t max_level_in_render = 0u;
            for (const PatchKey &key: render_keys)
            {
                render_set.insert(key);
                max_level_in_render = std::max(max_level_in_render, key.level);
            }

            bool changed = false;
            std::vector<PatchKey> next_render_cut;
            next_render_cut.reserve(render_keys.size() * 2u);
            for (const PatchKey &key: render_keys)
            {
                const auto desired_it = desired_max_levels.find(key);
                const bool wants_split = desired_it != desired_max_levels.end() && desired_it->second > key.level;
                const bool needs_balance =
                        planet_helpers::patch_needs_balance_split(key, render_set, max_level_in_render);

                if ((wants_split || needs_balance) && all_children_ready(key, is_ready))
                {
                    append_child_keys(key, next_render_cut);
                    changed = true;
                    continue;
                }

                next_render_cut.push_back(key);
            }

            render_keys = std::move(next_render_cut);
            if (!changed)
            {
                break;
            }
        }

        return render_keys;
    }
} // namespace

void PlanetSystem::init(EngineContext *context)
{
    _context = context;
    start_terrain_patch_workers();
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
    stop_terrain_patch_workers();

    if (!_context)
    {
        return;
    }

    if (DeviceManager *device = _context->getDevice())
    {
        VK_CHECK(vkDeviceWaitIdle(device->device()));
    }

    TextureCache *textures = _context->textures;
    if (textures)
    {
        for (auto &kv: _terrain_states)
        {
            TerrainState &state = *kv.second;
            for (MaterialInstance &mat: state.face_materials)
            {
                if (mat.materialSet != VK_NULL_HANDLE)
                {
                    textures->unwatchSet(mat.materialSet);
                }
            }
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

    ResourceManager *rm = _context->getResources();
    if (rm)
    {
        for (auto &kv: _terrain_states)
        {
            TerrainState &state = *kv.second;
            for (TerrainPatch &p: state.patches)
            {
                if (p.vertex_buffer.buffer != VK_NULL_HANDLE)
                {
                    rm->destroy_buffer(p.vertex_buffer);
                    p.vertex_buffer = {};
                    p.vertex_buffer_address = 0;
                }
            }

            state.patch_lookup.clear();
            state.patch_lru.clear();
            state.patch_free.clear();
            state.patches.clear();
            state.current_render_cut.clear();

            if (state.material_constants_buffer.buffer != VK_NULL_HANDLE)
            {
                rm->destroy_buffer(state.material_constants_buffer);
                state.material_constants_buffer = {};
                state.material_constants_stride = 0;
            }

            if (state.patch_index_buffer.buffer != VK_NULL_HANDLE)
            {
                rm->destroy_buffer(state.patch_index_buffer);
                state.patch_index_buffer = {};
                state.patch_index_count = 0;
                state.patch_index_resolution = 0;
                state.patch_indices_cpu.clear();
                state.patch_indices_cpu_snapshot.reset();
            }
        }
    }

    if (_earth_patch_material_layout != VK_NULL_HANDLE)
    {
        if (DeviceManager *device = _context->getDevice())
        {
            vkDestroyDescriptorSetLayout(device->device(), _earth_patch_material_layout, nullptr);
        }
        _earth_patch_material_layout = VK_NULL_HANDLE;
    }

    _terrain_states.clear();

    _bodies.clear();
}

PlanetSystem::PlanetBody *PlanetSystem::find_body_by_name(std::string_view name)
{
    auto it = std::find_if(_bodies.begin(), _bodies.end(),
                           [&](const PlanetBody &b) { return b.name == name; });
    return it != _bodies.end() ? &(*it) : nullptr;
}

double PlanetSystem::sample_terrain_displacement_m(const PlanetBody &body, const glm::dvec3 &dir_from_center) const
{
    if (!body.terrain || !(body.terrain_height_max_m > 0.0))
    {
        return 0.0;
    }

    const TerrainState *state = find_terrain_state(body.name);
    if (!state)
    {
        return 0.0;
    }

    planet::CubeFace face = planet::CubeFace::PosX;
    double u01 = 0.0;
    double v01 = 0.0;
    if (!planet::cubesphere_direction_to_face_uv(dir_from_center, face, u01, v01))
    {
        return 0.0;
    }

    const uint32_t face_index = static_cast<uint32_t>(face);
    if (face_index >= state->height_faces.size())
    {
        return 0.0;
    }

    const planet::HeightFace &height_face = state->height_faces[face_index];
    if (height_face.width == 0 || height_face.height == 0 || height_face.texels.empty())
    {
        return 0.0;
    }

    const float h01 = planet::sample_height(height_face, static_cast<float>(u01), static_cast<float>(v01));
    return static_cast<double>(h01) * body.terrain_height_max_m;
}

const PlanetSystem::EarthDebugStats &PlanetSystem::terrain_debug_stats(std::string_view name) const
{
    const TerrainState *state = find_terrain_state(name);
    return state ? state->debug_stats : _empty_debug_stats;
}

const PlanetSystem::EarthDebugStats &PlanetSystem::earth_debug_stats() const
{
    const PlanetBody *body = find_terrain_body();
    return body ? terrain_debug_stats(body->name) : _empty_debug_stats;
}

uint32_t PlanetSystem::terrain_patch_resolution_for_body(const PlanetBody &body) const
{
    return std::max(2u, body.patch_resolution_override != 0u ? body.patch_resolution_override : _earth_patch_resolution);
}

float PlanetSystem::terrain_target_sse_for_body(const PlanetBody &body) const
{
    const float base_target = std::max(_earth_quadtree_settings.target_sse_px, 0.1f);
    return (body.target_sse_px_override > 0.0f) ? body.target_sse_px_override : base_target;
}

PlanetSystem::PlanetBody *PlanetSystem::create_mesh_planet(const MeshPlanetCreateInfo &info)
{
    if (info.name.empty() || find_body_by_name(info.name))
    {
        return nullptr;
    }

    PlanetBody body{};
    body.name = info.name;
    body.center_world = info.center_world;
    body.radius_m = info.radius_m;
    body.visible = info.visible;
    body.terrain = false;
    body.base_color = info.base_color;
    body.metallic = info.metallic;
    body.roughness = info.roughness;

    if (_context && _context->assets)
    {
        AssetManager *assets = _context->assets;

        const std::string asset_name = fmt::format("Planet_{}", info.name);
        const GLTFMetallic_Roughness::MaterialConstants mc =
                planet_helpers::make_planet_constants(info.base_color, info.metallic, info.roughness);

        body.material = assets->createMaterialFromConstants(asset_name, mc, MaterialPass::MainColor);

        std::vector<Vertex> verts;
        std::vector<uint32_t> inds;
        primitives::buildSphere(verts, inds,
                                static_cast<int>(std::max(3u, info.sectors)),
                                static_cast<int>(std::max(2u, info.stacks)));
        geom::generate_tangents(verts, inds);

        body.mesh = assets->createMesh(asset_name,
                                       std::span<Vertex>(verts),
                                       std::span<uint32_t>(inds),
                                       body.material);
    }

    _bodies.push_back(std::move(body));
    return &_bodies.back();
}

PlanetSystem::PlanetBody *PlanetSystem::create_terrain_planet(const TerrainPlanetCreateInfo &info)
{
    if (info.name.empty() || find_body_by_name(info.name))
    {
        return nullptr;
    }

    PlanetBody body{};
    body.name = info.name;
    body.center_world = info.center_world;
    body.radius_m = info.radius_m;
    body.visible = info.visible;

    body.terrain = true;
    body.base_color = info.base_color;
    body.metallic = info.metallic;
    body.roughness = info.roughness;
    body.terrain_albedo_dir = info.albedo_dir;
    body.terrain_height_dir = info.height_dir;
    body.terrain_height_max_m = (!info.height_dir.empty()) ? std::max(0.0, info.height_max_m) : 0.0;
    body.terrain_detail_normal_dir = info.detail_normal_dir;
    body.terrain_detail_normal_strength = info.detail_normal_strength;
    body.terrain_cavity_dir = info.cavity_dir;
    body.terrain_cavity_strength = info.cavity_strength;
    body.terrain_enable_terminator_shadow = info.enable_terminator_shadow;
    body.patch_resolution_override = info.patch_resolution_override;
    body.target_sse_px_override = info.target_sse_px_override;
    body.terrain_emission_dir = info.emission_dir;
    body.emission_factor = info.emission_factor;
    body.terrain_specular_dir = info.specular_dir;
    body.specular_strength = info.specular_strength;
    body.specular_roughness = info.specular_roughness;

    if (_context && _context->assets)
    {
        AssetManager *assets = _context->assets;

        const std::string asset_name = fmt::format("Planet_{}_TerrainMaterial", info.name);
        const GLTFMetallic_Roughness::MaterialConstants mc =
                planet_helpers::make_planet_constants(info.base_color, info.metallic, info.roughness, info.emission_factor);
        body.material = assets->createMaterialFromConstants(asset_name, mc, MaterialPass::MainColor);
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

    // Clean up any terrain state for this planet (even if not currently marked as terrain).
    if (TerrainState *state = find_terrain_state(it->name))
    {
        clear_terrain_patch_cache(*state);
        clear_terrain_materials(*state);
        if (_context && state->material_constants_buffer.buffer != VK_NULL_HANDLE)
        {
            ResourceManager *rm = _context->getResources();
            FrameResources *frame = _context->currentFrame;
            const AllocatedBuffer buf = state->material_constants_buffer;
            if (rm)
            {
                if (frame)
                {
                    frame->_deletionQueue.push_function([rm, buf]() { rm->destroy_buffer(buf); });
                }
                else
                {
                    rm->destroy_buffer(buf);
                }
            }
            state->material_constants_buffer = {};
            state->material_constants_stride = 0;
        }

        if (_context && state->patch_index_buffer.buffer != VK_NULL_HANDLE)
        {
            ResourceManager *rm = _context->getResources();
            FrameResources *frame = _context->currentFrame;
            const AllocatedBuffer buf = state->patch_index_buffer;
            if (rm)
            {
                if (frame)
                {
                    frame->_deletionQueue.push_function([rm, buf]() { rm->destroy_buffer(buf); });
                }
                else
                {
                    rm->destroy_buffer(buf);
                }
            }
            state->patch_index_buffer = {};
            state->patch_index_count = 0;
            state->patch_index_resolution = 0;
            state->patch_indices_cpu.clear();
            state->patch_indices_cpu_snapshot.reset();
        }
    }
    _terrain_states.erase(std::string{name});

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
        for (const PlanetBody &b: _bodies)
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
    for (auto &kv: _terrain_states)
    {
        TerrainState &state = *kv.second;
        clear_terrain_patch_cache(state);
        clear_terrain_materials(state);
        if (_context && state.material_constants_buffer.buffer != VK_NULL_HANDLE)
        {
            ResourceManager *rm = _context->getResources();
            FrameResources *frame = _context->currentFrame;
            const AllocatedBuffer buf = state.material_constants_buffer;
            if (rm)
            {
                if (frame)
                {
                    frame->_deletionQueue.push_function([rm, buf]() { rm->destroy_buffer(buf); });
                }
                else
                {
                    rm->destroy_buffer(buf);
                }
            }
            state.material_constants_buffer = {};
            state.material_constants_stride = 0;
        }

        if (_context && state.patch_index_buffer.buffer != VK_NULL_HANDLE)
        {
            ResourceManager *rm = _context->getResources();
            FrameResources *frame = _context->currentFrame;
            const AllocatedBuffer buf = state.patch_index_buffer;
            if (rm)
            {
                if (frame)
                {
                    frame->_deletionQueue.push_function([rm, buf]() { rm->destroy_buffer(buf); });
                }
                else
                {
                    rm->destroy_buffer(buf);
                }
            }
            state.patch_index_buffer = {};
            state.patch_index_count = 0;
            state.patch_index_resolution = 0;
            state.patch_indices_cpu.clear();
            state.patch_indices_cpu_snapshot.reset();
        }
    }
    _terrain_states.clear();

    _bodies.clear();
}

bool PlanetSystem::set_planet_center(std::string_view name, const WorldVec3 &center_world)
{
    PlanetBody *b = find_body_by_name(name);
    if (!b) return false;
    b->center_world = center_world;
    return true;
}

bool PlanetSystem::set_planet_radius(std::string_view name, double radius_m)
{
    PlanetBody *b = find_body_by_name(name);
    if (!b) return false;

    const double safe_radius = std::max(1.0, radius_m);
    if (b->radius_m == safe_radius) return true;

    b->radius_m = safe_radius;
    if (b->terrain)
    {
        if (TerrainState *state = find_terrain_state(b->name))
        {
            clear_terrain_patch_cache(*state);
            state->debug_stats = {};
        }
    }
    return true;
}

bool PlanetSystem::set_planet_visible(std::string_view name, bool visible)
{
    PlanetBody *b = find_body_by_name(name);
    if (!b) return false;
    b->visible = visible;
    return true;
}

bool PlanetSystem::set_planet_terrain(std::string_view name, bool terrain)
{
    PlanetBody *target = find_body_by_name(name);
    if (!target) return false;

    if (target->terrain == terrain)
    {
        return true;
    }

    target->terrain = terrain;
    if (terrain)
    {
        if (TerrainState *state = get_or_create_terrain_state(target->name))
        {
            state->patch_cache_dirty = true;
        }
    }
    else
    {
        if (TerrainState *state = find_terrain_state(target->name))
        {
            clear_terrain_patch_cache(*state);
            clear_terrain_materials(*state);
            state->debug_stats = {};
        }
    }
    return true;
}

PlanetSystem::PlanetBody *PlanetSystem::find_terrain_body()
{
    auto it = std::find_if(_bodies.begin(), _bodies.end(),
                           [](const PlanetBody &b) { return b.terrain; });
    return it != _bodies.end() ? &(*it) : nullptr;
}

const PlanetSystem::PlanetBody *PlanetSystem::find_terrain_body() const
{
    auto it = std::find_if(_bodies.begin(), _bodies.end(),
                           [](const PlanetBody &b) { return b.terrain; });
    return it != _bodies.end() ? &(*it) : nullptr;
}

void PlanetSystem::update_and_emit(const SceneManager &scene, DrawContext &draw_context)
{
    if (!_enabled)
    {
        return;
    }

    const WorldVec3 origin_world = scene.get_world_origin();

    // Terrain bodies: quadtree patches (each terrain planet has independent cache/state).
    {
        using Clock = std::chrono::steady_clock;

        if (_context == nullptr)
        {
            return;
        }

        if (_earth_patch_cache_dirty)
        {
            clear_all_terrain_patch_caches();
            _earth_patch_cache_dirty = false;
        }

        const VkExtent2D logical_extent = _context->getLogicalRenderExtent();
        const WorldVec3 cam_world = scene.getMainCamera().position_world;

        for (PlanetBody &body: _bodies)
        {
            if (!body.terrain || !body.visible || !body.material)
            {
                continue;
            }

            TerrainState *state = get_or_create_terrain_state(body.name);
            if (!state)
            {
                continue;
            }

            if (state->patch_cache_dirty)
            {
                clear_terrain_patch_cache(*state);
                state->patch_cache_dirty = false;
            }

            const uint32_t effective_patch_resolution = terrain_patch_resolution_for_body(body);
            const float effective_target_sse_px = terrain_target_sse_for_body(body);
            state->effective_patch_resolution = effective_patch_resolution;
            state->effective_target_sse_px = effective_target_sse_px;
            ensure_terrain_patch_index_buffer(*state, effective_patch_resolution);

            const Clock::time_point t0 = Clock::now();

            ensure_terrain_height_maps(*state, body);

            planet::PlanetQuadtree::Settings effective_quadtree_settings = _earth_quadtree_settings;
            effective_quadtree_settings.target_sse_px = effective_target_sse_px;
            state->quadtree.set_settings(effective_quadtree_settings);

            const Clock::time_point t_q0 = Clock::now();
            state->quadtree.update(body.center_world,
                                   body.radius_m,
                                   body.terrain_height_max_m,
                                   cam_world,
                                   origin_world,
                                   scene.getSceneData(),
                                   logical_extent,
                                   effective_patch_resolution,
                                   &state->height_faces);
            const Clock::time_point t_q1 = Clock::now();

            const glm::vec3 body_center_local = world_to_local(body.center_world, origin_world);
            ensure_terrain_face_materials(*state, body, body_center_local);
            if (_context->textures)
            {
                for (const MaterialInstance &mat: state->face_materials)
                {
                    if (mat.materialSet != VK_NULL_HANDLE)
                    {
                        _context->textures->markSetUsed(mat.materialSet, _context->frameIndex);
                    }
                }
            }

            size_t desired_capacity =
                    static_cast<size_t>(state->patches.size()) +
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
            if (state->patches.capacity() < desired_capacity)
            {
                state->patches.reserve(desired_capacity);
            }

            uint32_t created_patches = 0;
            double ms_patch_create = 0.0;
            const uint32_t max_create = _earth_patch_create_budget_per_frame;
            const double max_create_ms =
                    (_earth_patch_create_budget_ms > 0.0f) ? static_cast<double>(_earth_patch_create_budget_ms) : 0.0;
            const uint32_t frame_index = ++state->patch_frame_stamp;

            const Clock::time_point t_emit0 = Clock::now();

            pump_completed_terrain_patch_builds(*state,
                                                body.name,
                                                created_patches,
                                                ms_patch_create,
                                                max_create,
                                                max_create_ms);

            const std::vector<planet::PatchKey> &desired_leaves = state->quadtree.visible_leaves();
            PatchKeySet create_seen;
            std::vector<planet::PatchKey> create_queue;
            create_seen.reserve(desired_leaves.size() * 8u + 160u);
            create_queue.reserve(desired_leaves.size() * 8u + 160u);

            auto enqueue_patch = [&](const planet::PatchKey &key) {
                if (create_seen.emplace(key).second)
                {
                    create_queue.push_back(key);
                }
            };

            std::vector<planet::PatchKey> scratch_keys;
            scratch_keys.reserve(32u);
            append_pinned_patch_keys(k_pinned_patch_level, scratch_keys);
            for (const planet::PatchKey &key: scratch_keys)
            {
                enqueue_patch(key);
            }

            for (const planet::PatchKey &leaf: desired_leaves)
            {
                scratch_keys.clear();
                append_required_split_groups_for_leaf(leaf, scratch_keys);
                for (const planet::PatchKey &key: scratch_keys)
                {
                    enqueue_patch(key);
                }
            }

            std::sort(create_queue.begin(), create_queue.end(),
                      [](const planet::PatchKey &a, const planet::PatchKey &b) {
                          if (a.level != b.level) return a.level < b.level;
                          if (a.face != b.face) return a.face < b.face;
                          if (a.x != b.x) return a.x < b.x;
                          return a.y < b.y;
                      });

            for (const planet::PatchKey &k: create_queue)
            {
                if (is_terrain_patch_ready(*state, k))
                {
                    continue;
                }
                request_terrain_patch_build(*state, body, k, frame_index, 0u);
            }

            const std::vector<planet::PatchKey> render_keys =
                    build_ready_render_cut(desired_leaves,
                                           [&](const planet::PatchKey &key) { return is_terrain_patch_ready(*state, key); });
            state->current_render_cut = render_keys;

            PatchKeySet render_set;
            render_set.reserve(render_keys.size() * 2u);
            uint32_t max_level_in_render = 0u;
            for (const planet::PatchKey &key: render_keys)
            {
                render_set.insert(key);
                max_level_in_render = std::max(max_level_in_render, key.level);
            }

            std::unordered_map<planet::PatchKey, uint8_t, planet::PatchKeyHash> edge_stitch_masks;
            edge_stitch_masks.reserve(render_keys.size());
            for (const planet::PatchKey &key: render_keys)
            {
                const uint8_t mask = planet_helpers::compute_patch_edge_stitch_mask(key, render_set, max_level_in_render);
                if (mask != 0u)
                {
                    edge_stitch_masks.emplace(key, mask);
                }
            }

            auto stitch_mask_for = [&](const planet::PatchKey &key) -> uint8_t {
                auto it = edge_stitch_masks.find(key);
                return (it != edge_stitch_masks.end()) ? it->second : 0u;
            };

            std::vector<uint32_t> ready_patch_indices;
            ready_patch_indices.reserve(render_keys.size());

            for (const planet::PatchKey &k: render_keys)
            {
                const uint8_t stitch_mask = stitch_mask_for(k);
                TerrainPatch *patch = find_terrain_patch(*state, k);
                if (!patch)
                {
                    request_terrain_patch_build(*state, body, k, frame_index, stitch_mask);
                    continue;
                }

                if (patch->edge_stitch_mask != stitch_mask)
                {
                    request_terrain_patch_build(*state, body, k, frame_index, stitch_mask);
                }

                if (patch->state != TerrainPatchState::Ready ||
                    patch->vertex_buffer.buffer == VK_NULL_HANDLE ||
                    patch->vertex_buffer_address == 0)
                {
                    continue;
                }

                patch->last_used_frame = frame_index;
                state->patch_lru.splice(state->patch_lru.begin(), state->patch_lru, patch->lru_it); {
                    const uint32_t idx = static_cast<uint32_t>(patch - state->patches.data());
                    ready_patch_indices.push_back(idx);
                }
            }

            for (uint32_t idx: ready_patch_indices)
            {
                if (idx >= state->patches.size())
                {
                    continue;
                }

                TerrainPatch &patch = state->patches[idx];
                if (patch.state != TerrainPatchState::Ready ||
                    patch.vertex_buffer.buffer == VK_NULL_HANDLE ||
                    patch.vertex_buffer_address == 0 ||
                    state->patch_index_buffer.buffer == VK_NULL_HANDLE ||
                    state->patch_index_count == 0)
                {
                    continue;
                }

                const uint32_t face_index = static_cast<uint32_t>(patch.key.face);
                if (face_index >= state->face_materials.size())
                {
                    continue;
                }
                MaterialInstance *material = &state->face_materials[face_index];
                if (material->materialSet == VK_NULL_HANDLE || material->pipeline == nullptr)
                {
                    continue;
                }

                const WorldVec3 patch_center_world =
                        body.center_world + patch.patch_center_dir * body.radius_m;
                const glm::vec3 patch_center_local = world_to_local(patch_center_world, origin_world);
                const glm::mat4 transform = glm::translate(glm::mat4(1.0f), patch_center_local);

                Bounds b{};
                b.origin = patch.bounds_origin;
                b.extents = patch.bounds_extents;
                b.sphereRadius = patch.bounds_sphere_radius;
                b.type = patch.pick_bvh ? BoundsType::Mesh : BoundsType::Box;

                RenderObject obj{};
                obj.indexCount = state->patch_index_count;
                obj.firstIndex = 0;
                obj.indexBuffer = state->patch_index_buffer.buffer;
                obj.vertexBuffer = patch.vertex_buffer.buffer;
                obj.vertexBufferAddress = patch.vertex_buffer_address;
                obj.material = material;
                obj.bounds = b;
                obj.transform = transform;
                // Planet terrain patches are not meaningful RT occluders; skip BLAS/TLAS builds.
                obj.sourceMesh = nullptr;
                obj.pickBVH = patch.pick_bvh.get();
                obj.surfaceIndex = 0;
                obj.objectID = draw_context.nextID++;
                obj.ownerType = RenderObject::OwnerType::MeshInstance;
                obj.ownerName = body.name;

                draw_context.OpaqueSurfaces.push_back(obj);

                const bool ocean_enabled = !body.terrain_specular_dir.empty() && (body.specular_strength > 0.0f);
                if (ocean_enabled)
                {
                    OceanRenderObject ocean{};
                    ocean.surface = obj;
                    ocean.body_center_local = body_center_local;
                    ocean.sea_level_radius = static_cast<float>(body.radius_m);
                    ocean.shell_offset = ocean_shell_offset_m(body.radius_m);
                    draw_context.OceanSurfaces.push_back(ocean);
                }
            }
            const Clock::time_point t_emit1 = Clock::now();

            trim_terrain_patch_cache(*state);

            const uint32_t visible_patches = static_cast<uint32_t>(state->quadtree.visible_leaves().size());
            const uint32_t n = effective_patch_resolution;
            const uint32_t patch_tris = (n >= 2u) ? (2u * (n - 1u) * (n + 3u)) : 0u;
            const uint32_t estimated_tris = patch_tris * visible_patches;

            state->debug_stats = {};
            state->debug_stats.quadtree = state->quadtree.stats();
            state->debug_stats.visible_patches = visible_patches;
            state->debug_stats.rendered_patches = static_cast<uint32_t>(ready_patch_indices.size());
            state->debug_stats.created_patches = created_patches;
            state->debug_stats.patch_cache_size = static_cast<uint32_t>(state->patch_lookup.size());
            state->debug_stats.estimated_triangles = estimated_tris;
            state->debug_stats.ms_quadtree = static_cast<float>(std::chrono::duration<double, std::milli>(t_q1 - t_q0).
                count());
            state->debug_stats.ms_patch_create = static_cast<float>(ms_patch_create);
            const double ms_emit_total = std::chrono::duration<double, std::milli>(t_emit1 - t_emit0).count();
            state->debug_stats.ms_emit = static_cast<float>(std::max(0.0, ms_emit_total - ms_patch_create));
            state->debug_stats.ms_total = static_cast<float>(std::chrono::duration<double, std::milli>(
                Clock::now() - t0).count());
        }
    }

    // Other bodies (moon): regular mesh instances.
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
        for (const GeoSurface &surf: b.mesh->surfaces)
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
            else if (obj.material && obj.material->passType == MaterialPass::MeshVFX)
            {
                draw_context.MeshVfxSurfaces.push_back(obj);
            }
            else
            {
                draw_context.OpaqueSurfaces.push_back(obj);
            }
        }
    }
}
