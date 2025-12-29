#include "planet_system.h"

#include <core/context.h>
#include <core/frame/resources.h>
#include <core/types.h>
#include <core/assets/manager.h>
#include <render/materials.h>
#include <render/primitives.h>
#include <scene/planet/cubesphere.h>
#include <scene/tangent_space.h>
#include <scene/vk_scene.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace
{
    constexpr double kEarthRadiusM = 6378137.0;    // WGS84 equatorial radius
    constexpr double kMoonRadiusM = 1737400.0;     // mean radius
    constexpr double kMoonDistanceM = 384400000.0; // mean Earth-Moon distance

    GLTFMetallic_Roughness::MaterialConstants make_planet_constants()
    {
        GLTFMetallic_Roughness::MaterialConstants c{};
        c.colorFactors = glm::vec4(1.0f);
        // metal_rough_factors.x = metallic, .y = roughness
        c.metal_rough_factors = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
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

const PlanetSystem::PlanetBody *PlanetSystem::get_body(BodyID id) const
{
    size_t i = static_cast<size_t>(id);
    if (i >= _bodies.size())
    {
        return nullptr;
    }
    return &_bodies[i];
}

PlanetSystem::PlanetBody *PlanetSystem::get_body(BodyID id)
{
    size_t i = static_cast<size_t>(id);
    if (i >= _bodies.size())
    {
        return nullptr;
    }
    return &_bodies[i];
}

void PlanetSystem::ensure_bodies_created()
{
    if (!_bodies.empty())
    {
        return;
    }

    PlanetBody earth{};
    earth.name = "Earth";
    earth.center_world = WorldVec3(0.0, 0.0, 0.0);
    earth.radius_m = kEarthRadiusM;

    PlanetBody moon{};
    moon.name = "Moon";
    moon.center_world = WorldVec3(kMoonDistanceM, 0.0, 0.0);
    moon.radius_m = kMoonRadiusM;

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

std::shared_ptr<MeshAsset> PlanetSystem::get_or_create_earth_patch_mesh(const PlanetBody &earth,
                                                                        const planet::PatchKey &key)
{
    auto it = _earth_patch_cache.find(key);
    if (it != _earth_patch_cache.end())
    {
        it->second.last_used_frame = _context ? _context->frameIndex : 0;
        _earth_patch_lru.splice(_earth_patch_lru.begin(), _earth_patch_lru, it->second.lru_it);
        return it->second.mesh;
    }

    if (!_context || !_context->assets || !earth.material)
    {
        return {};
    }

    planet::CubeSpherePatchMesh mesh{};
    planet::build_cubesphere_patch_mesh(mesh,
                                        earth.center_world,
                                        earth.radius_m,
                                        key.face,
                                        key.level,
                                        key.x,
                                        key.y,
                                        _earth_patch_resolution,
                                        debug_color_for_level(key.level),
                                        /*generate_tangents=*/false);

    const uint32_t face_i = static_cast<uint32_t>(key.face);
    const std::string name =
        "Planet_EarthPatch_f" + std::to_string(face_i) +
        "_L" + std::to_string(key.level) +
        "_X" + std::to_string(key.x) +
        "_Y" + std::to_string(key.y);

    std::shared_ptr<MeshAsset> out =
        _context->assets->createMesh(name, mesh.vertices, mesh.indices, earth.material, /*build_bvh=*/false);

    EarthPatchCacheEntry entry{};
    entry.mesh = out;
    entry.patch_center_dir = planet::cubesphere_patch_center_direction(key.face, key.level, key.x, key.y);
    entry.last_used_frame = _context ? _context->frameIndex : 0;
    _earth_patch_lru.push_front(key);
    entry.lru_it = _earth_patch_lru.begin();
    _earth_patch_cache.emplace(key, std::move(entry));
    return out;
}

void PlanetSystem::trim_earth_patch_cache()
{
    if (_earth_patch_cache_max == 0)
    {
        return;
    }

    if (_earth_patch_cache.size() <= static_cast<size_t>(_earth_patch_cache_max))
    {
        return;
    }

    if (!_context || !_context->assets)
    {
        return;
    }

    AssetManager *assets = _context->assets;
    FrameResources *frame = _context->currentFrame;

    while (_earth_patch_cache.size() > static_cast<size_t>(_earth_patch_cache_max) && !_earth_patch_lru.empty())
    {
        const planet::PatchKey key = _earth_patch_lru.back();
        _earth_patch_lru.pop_back();

        auto it = _earth_patch_cache.find(key);
        if (it == _earth_patch_cache.end())
        {
            continue;
        }

        std::shared_ptr<MeshAsset> mesh = std::move(it->second.mesh);
        _earth_patch_cache.erase(it);

        if (!mesh)
        {
            continue;
        }

        if (frame)
        {
            assets->removeMeshDeferred(mesh->name, frame->_deletionQueue);
        }
        else
        {
            assets->removeMesh(mesh->name);
        }
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

    // Earth: quadtree patches.
    {
        using Clock = std::chrono::steady_clock;

        PlanetBody *earth = get_body(BodyID::Earth);
        if (earth && earth->visible && earth->material && _context)
        {
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
                                   logical_extent);
            const Clock::time_point t_q1 = Clock::now();

            uint32_t created_patches = 0;
            double ms_patch_create = 0.0;
            const uint32_t max_create = _earth_patch_create_budget_per_frame;
            const double max_create_ms =
                (_earth_patch_create_budget_ms > 0.0f) ? static_cast<double>(_earth_patch_create_budget_ms) : 0.0;
            const uint32_t frame_index = _context->frameIndex;

            const Clock::time_point t_emit0 = Clock::now();
            for (const planet::PatchKey &k : _earth_quadtree.visible_leaves())
            {
                EarthPatchCacheEntry *entry = nullptr;
                {
                    auto it = _earth_patch_cache.find(k);
                    if (it != _earth_patch_cache.end())
                    {
                        it->second.last_used_frame = frame_index;
                        _earth_patch_lru.splice(_earth_patch_lru.begin(), _earth_patch_lru, it->second.lru_it);
                        entry = &it->second;
                    }
                    else
                    {
                        const bool hit_count_budget = (max_create != 0u) && (created_patches >= max_create);
                        const bool hit_time_budget = (max_create_ms > 0.0) && (ms_patch_create >= max_create_ms);
                        if (!hit_count_budget && !hit_time_budget)
                        {
                            const Clock::time_point t_c0 = Clock::now();
                            (void)get_or_create_earth_patch_mesh(*earth, k);
                            const Clock::time_point t_c1 = Clock::now();

                            created_patches++;
                            ms_patch_create += std::chrono::duration<double, std::milli>(t_c1 - t_c0).count();
                        }

                        auto it2 = _earth_patch_cache.find(k);
                        if (it2 != _earth_patch_cache.end())
                        {
                            entry = &it2->second;
                        }
                    }
                }
                if (!entry || !entry->mesh || entry->mesh->surfaces.empty())
                {
                    continue;
                }

                const std::shared_ptr<MeshAsset> &mesh = entry->mesh;

                const WorldVec3 patch_center_world =
                    earth->center_world + entry->patch_center_dir * earth->radius_m;
                const glm::vec3 patch_center_local = world_to_local(patch_center_world, origin_world);
                const glm::mat4 transform = glm::translate(glm::mat4(1.0f), patch_center_local);

                uint32_t surface_index = 0;
                for (const GeoSurface &surf : mesh->surfaces)
                {
                    RenderObject obj{};
                    obj.indexCount = surf.count;
                    obj.firstIndex = surf.startIndex;
                    obj.indexBuffer = mesh->meshBuffers.indexBuffer.buffer;
                    obj.vertexBuffer = mesh->meshBuffers.vertexBuffer.buffer;
                    obj.vertexBufferAddress = mesh->meshBuffers.vertexBufferAddress;
                    obj.material = surf.material ? &surf.material->data : nullptr;
                    obj.bounds = surf.bounds;
                    obj.transform = transform;
                    // Planet terrain patches are not meaningful RT occluders; skip BLAS/TLAS builds.
                    obj.sourceMesh = nullptr;
                    obj.surfaceIndex = surface_index++;
                    obj.objectID = draw_context.nextID++;
                    obj.ownerType = RenderObject::OwnerType::MeshInstance;
                    obj.ownerName = earth->name;

                    draw_context.OpaqueSurfaces.push_back(obj);
                }
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
            _earth_debug_stats.patch_cache_size = static_cast<uint32_t>(_earth_patch_cache.size());
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

        if (body_index == static_cast<size_t>(BodyID::Earth))
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
