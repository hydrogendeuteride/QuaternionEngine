#include "planet_system.h"

#include <core/context.h>
#include <core/types.h>
#include <core/assets/manager.h>
#include <render/materials.h>
#include <render/primitives.h>
#include <scene/tangent_space.h>
#include <scene/vk_scene.h>

#include <glm/gtc/quaternion.hpp>

namespace
{
    constexpr double kEarthRadiusM = 6378137.0;  // WGS84 equatorial radius
    constexpr double kMoonRadiusM = 1737400.0;   // mean radius
    constexpr double kMoonDistanceM = 384400000.0; // mean Earth-Moon distance

    GLTFMetallic_Roughness::MaterialConstants make_planet_constants()
    {
        GLTFMetallic_Roughness::MaterialConstants c{};
        c.colorFactors = glm::vec4(1.0f);
        // metal_rough_factors.x = metallic, .y = roughness
        c.metal_rough_factors = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
        return c;
    }
}

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

        // Earth: textured sphere (albedo only for now).
        {
            AssetManager::MeshCreateInfo ci{};
            ci.name = "Planet_EarthSphere";
            ci.geometry.type = AssetManager::MeshGeometryDesc::Type::Sphere;
            ci.geometry.sectors = 64;
            ci.geometry.stacks = 32;

            ci.material.kind = AssetManager::MeshMaterialDesc::Kind::Textured;
            ci.material.options.albedoPath = "earth/earth_8k.jpg";
            ci.material.options.albedoSRGB = true;
            ci.material.options.constants = make_planet_constants();
            ci.material.options.pass = MaterialPass::MainColor;

            earth.mesh = assets->createMesh(ci);
            if (earth.mesh && !earth.mesh->surfaces.empty())
            {
                earth.material = earth.mesh->surfaces[0].material;
            }
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

void PlanetSystem::update_and_emit(const SceneManager &scene, DrawContext &draw_context)
{
    if (!_enabled)
    {
        return;
    }

    ensure_bodies_created();

    const WorldVec3 origin_world = scene.get_world_origin();

    for (PlanetBody &b : _bodies)
    {
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
