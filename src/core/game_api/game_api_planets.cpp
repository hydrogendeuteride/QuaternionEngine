#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "scene/vk_scene.h"
#include "scene/planet/planet_system.h"

namespace GameAPI
{

// ----------------------------------------------------------------------------
// Planets - Create/Destroy
// ----------------------------------------------------------------------------

bool Engine::add_planet_sphere(const PlanetSphere &planet)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    if (!planets)
    {
        return false;
    }

    PlanetSystem::MeshPlanetCreateInfo info{};
    info.name = planet.name;
    info.center_world = WorldVec3(planet.center);
    info.radius_m = planet.radius_m;
    info.visible = planet.visible;
    info.base_color = planet.base_color;
    info.metallic = planet.metallic;
    info.roughness = planet.roughness;
    info.sectors = planet.sectors;
    info.stacks = planet.stacks;

    return planets->create_mesh_planet(info) != nullptr;
}

bool Engine::add_planet_terrain(const PlanetTerrain &planet)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    if (!planets)
    {
        return false;
    }

    PlanetSystem::TerrainPlanetCreateInfo info{};
    info.name = planet.name;
    info.center_world = WorldVec3(planet.center);
    info.radius_m = planet.radius_m;
    info.visible = planet.visible;
    info.base_color = planet.base_color;
    info.metallic = planet.metallic;
    info.roughness = planet.roughness;
    info.albedo_dir = planet.albedo_dir;
    info.height_dir = planet.height_dir;
    info.height_max_m = planet.height_max_m;
    info.emission_dir = planet.emission_dir;
    info.emission_factor = planet.emission_factor;

    return planets->create_terrain_planet(info) != nullptr;
}

bool Engine::remove_planet(const std::string &name)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->destroy_planet(name) : false;
}

void Engine::clear_planets(bool destroy_mesh_assets)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return;
    }

    if (PlanetSystem *planets = _engine->_sceneManager->get_planet_system())
    {
        planets->clear_planets(destroy_mesh_assets);
    }
}

// ----------------------------------------------------------------------------
// Planets - Query/Edit
// ----------------------------------------------------------------------------

bool Engine::get_planet(const std::string &name, PlanetInfo &out) const
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    if (!planets)
    {
        return false;
    }

    for (const PlanetSystem::PlanetBody &b : planets->bodies())
    {
        if (b.name == name)
        {
            out.name = b.name;
            out.center = glm::dvec3(b.center_world);
            out.radius_m = b.radius_m;
            out.visible = b.visible;
            out.terrain = b.terrain;
            return true;
        }
    }
    return false;
}

size_t Engine::get_planet_count() const
{
    if (!_engine || !_engine->_sceneManager)
    {
        return 0;
    }

    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    if (!planets)
    {
        return 0;
    }
    return planets->bodies().size();
}

bool Engine::get_planet(size_t index, PlanetInfo &out) const
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    if (!planets)
    {
        return false;
    }
    const auto &bodies = planets->bodies();
    if (index >= bodies.size())
    {
        return false;
    }

    const PlanetSystem::PlanetBody &b = bodies[index];
    out.name = b.name;
    out.center = glm::dvec3(b.center_world);
    out.radius_m = b.radius_m;
    out.visible = b.visible;
    out.terrain = b.terrain;
    return true;
}

bool Engine::set_planet_center(const std::string &name, const glm::dvec3 &center)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->set_planet_center(name, WorldVec3(center)) : false;
}

bool Engine::set_planet_radius(const std::string &name, double radius_m)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->set_planet_radius(name, radius_m) : false;
}

bool Engine::set_planet_visible(const std::string &name, bool visible)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->set_planet_visible(name, visible) : false;
}

bool Engine::set_planet_terrain(const std::string &name, bool terrain)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->set_planet_terrain(name, terrain) : false;
}

// ----------------------------------------------------------------------------
// Planet System Global
// ----------------------------------------------------------------------------

void Engine::set_planet_system_enabled(bool enabled)
{
    if (!_engine || !_engine->_sceneManager) return;
    if (PlanetSystem *planets = _engine->_sceneManager->get_planet_system())
    {
        planets->set_enabled(enabled);
    }
}

bool Engine::get_planet_system_enabled() const
{
    if (!_engine || !_engine->_sceneManager) return false;
    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->enabled() : false;
}

// ----------------------------------------------------------------------------
// Planet Terrain LOD (Quadtree) Settings
// ----------------------------------------------------------------------------

void Engine::set_planet_quadtree_settings(const PlanetQuadtreeSettings &settings)
{
    if (!_engine || !_engine->_sceneManager) return;
    PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    if (!planets) return;

    planet::PlanetQuadtree::Settings s = planets->earth_quadtree_settings();
    s.max_level = settings.maxLevel;
    s.target_sse_px = settings.targetScreenSpaceError;
    s.max_patches_visible = settings.maxPatchesVisible;
    s.frustum_cull = settings.frustumCull;
    s.horizon_cull = settings.horizonCull;
    planets->set_earth_quadtree_settings(s);
}

PlanetQuadtreeSettings Engine::get_planet_quadtree_settings() const
{
    PlanetQuadtreeSettings out{};
    if (!_engine || !_engine->_sceneManager) return out;
    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    if (!planets) return out;

    const auto &s = planets->earth_quadtree_settings();
    out.maxLevel = s.max_level;
    out.targetScreenSpaceError = s.target_sse_px;
    out.maxPatchesVisible = s.max_patches_visible;
    out.frustumCull = s.frustum_cull;
    out.horizonCull = s.horizon_cull;
    return out;
}

// ----------------------------------------------------------------------------
// Planet Terrain Patch Budget/Resolution
// ----------------------------------------------------------------------------

void Engine::set_planet_patch_create_budget(uint32_t patchesPerFrame)
{
    if (!_engine || !_engine->_sceneManager) return;
    if (PlanetSystem *planets = _engine->_sceneManager->get_planet_system())
    {
        planets->set_earth_patch_create_budget_per_frame(patchesPerFrame);
    }
}

uint32_t Engine::get_planet_patch_create_budget() const
{
    if (!_engine || !_engine->_sceneManager) return 0;
    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->earth_patch_create_budget_per_frame() : 0;
}

void Engine::set_planet_patch_create_budget_ms(float budgetMs)
{
    if (!_engine || !_engine->_sceneManager) return;
    if (PlanetSystem *planets = _engine->_sceneManager->get_planet_system())
    {
        planets->set_earth_patch_create_budget_ms(budgetMs);
    }
}

float Engine::get_planet_patch_create_budget_ms() const
{
    if (!_engine || !_engine->_sceneManager) return 0.0f;
    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->earth_patch_create_budget_ms() : 0.0f;
}

void Engine::set_planet_patch_resolution(uint32_t resolution)
{
    if (!_engine || !_engine->_sceneManager) return;
    if (PlanetSystem *planets = _engine->_sceneManager->get_planet_system())
    {
        planets->set_earth_patch_resolution(resolution);
    }
}

uint32_t Engine::get_planet_patch_resolution() const
{
    if (!_engine || !_engine->_sceneManager) return 0;
    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->earth_patch_resolution() : 0;
}

void Engine::set_planet_patch_cache_max(uint32_t maxPatches)
{
    if (!_engine || !_engine->_sceneManager) return;
    if (PlanetSystem *planets = _engine->_sceneManager->get_planet_system())
    {
        planets->set_earth_patch_cache_max(maxPatches);
    }
}

uint32_t Engine::get_planet_patch_cache_max() const
{
    if (!_engine || !_engine->_sceneManager) return 0;
    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->earth_patch_cache_max() : 0;
}

// ----------------------------------------------------------------------------
// Planet Terrain Debug
// ----------------------------------------------------------------------------

void Engine::set_planet_debug_tint_by_lod(bool enabled)
{
    if (!_engine || !_engine->_sceneManager) return;
    if (PlanetSystem *planets = _engine->_sceneManager->get_planet_system())
    {
        planets->set_earth_debug_tint_patches_by_lod(enabled);
    }
}

bool Engine::get_planet_debug_tint_by_lod() const
{
    if (!_engine || !_engine->_sceneManager) return false;
    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    return planets ? planets->earth_debug_tint_patches_by_lod() : false;
}

PlanetTerrainStats Engine::get_planet_terrain_stats(const std::string &name) const
{
    PlanetTerrainStats out{};
    if (!_engine || !_engine->_sceneManager) return out;
    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    if (!planets) return out;

    const PlanetSystem::EarthDebugStats &s = name.empty()
        ? planets->earth_debug_stats()
        : planets->terrain_debug_stats(name);

    out.visiblePatches = s.visible_patches;
    out.renderedPatches = s.rendered_patches;
    out.createdPatches = s.created_patches;
    out.patchCacheSize = s.patch_cache_size;
    out.estimatedTriangles = s.estimated_triangles;
    out.maxLevelUsed = s.quadtree.max_level_used;
    out.msQuadtree = s.ms_quadtree;
    out.msPatchCreate = s.ms_patch_create;
    out.msTotal = s.ms_total;
    return out;
}

double Engine::sample_planet_terrain_height(const std::string &name, const glm::dvec3 &dirFromCenter) const
{
    if (!_engine || !_engine->_sceneManager) return 0.0;
    const PlanetSystem *planets = _engine->_sceneManager->get_planet_system();
    if (!planets) return 0.0;

    for (const auto &body : planets->bodies())
    {
        if (body.name == name)
        {
            return planets->sample_terrain_displacement_m(body, dirFromCenter);
        }
    }
    return 0.0;
}

// ----------------------------------------------------------------------------
// Atmosphere
// ----------------------------------------------------------------------------

void Engine::set_atmosphere_enabled(bool enabled)
{
    if (!_engine || !_engine->_context) return;
    _engine->_context->enableAtmosphere = enabled;
}

bool Engine::get_atmosphere_enabled() const
{
    if (!_engine || !_engine->_context) return false;
    return _engine->_context->enableAtmosphere;
}

void Engine::set_atmosphere_settings(const AtmosphereSettings &settings)
{
    if (!_engine || !_engine->_context) return;

    ::AtmosphereSettings &dst = _engine->_context->atmosphere;
    dst.bodyName = settings.bodyName;
    dst.atmosphereHeightM = settings.atmosphereHeightM;
    dst.rayleighScaleHeightM = settings.rayleighScaleHeightM;
    dst.mieScaleHeightM = settings.mieScaleHeightM;
    dst.rayleighScattering = settings.rayleighScattering;
    dst.mieScattering = settings.mieScattering;
    dst.mieG = settings.mieG;
    dst.intensity = settings.intensity;
    dst.sunDiskIntensity = settings.sunDiskIntensity;
    dst.sunHaloIntensity = settings.sunHaloIntensity;
    dst.sunHaloRadiusDeg = settings.sunHaloRadiusDeg;
    dst.sunStarburstIntensity = settings.sunStarburstIntensity;
    dst.sunStarburstRadiusDeg = settings.sunStarburstRadiusDeg;
    dst.sunStarburstSpikes = settings.sunStarburstSpikes;
    dst.sunStarburstSharpness = settings.sunStarburstSharpness;
    dst.jitterStrength = settings.jitterStrength;
    dst.planetSurfaceSnapM = settings.planetSurfaceSnapM;
    dst.viewSteps = settings.viewSteps;
    dst.lightSteps = settings.lightSteps;
}

AtmosphereSettings Engine::get_atmosphere_settings() const
{
    AtmosphereSettings out{};
    if (!_engine || !_engine->_context) return out;

    const ::AtmosphereSettings &src = _engine->_context->atmosphere;
    out.bodyName = src.bodyName;
    out.atmosphereHeightM = src.atmosphereHeightM;
    out.rayleighScaleHeightM = src.rayleighScaleHeightM;
    out.mieScaleHeightM = src.mieScaleHeightM;
    out.rayleighScattering = src.rayleighScattering;
    out.mieScattering = src.mieScattering;
    out.mieG = src.mieG;
    out.intensity = src.intensity;
    out.sunDiskIntensity = src.sunDiskIntensity;
    out.sunHaloIntensity = src.sunHaloIntensity;
    out.sunHaloRadiusDeg = src.sunHaloRadiusDeg;
    out.sunStarburstIntensity = src.sunStarburstIntensity;
    out.sunStarburstRadiusDeg = src.sunStarburstRadiusDeg;
    out.sunStarburstSpikes = src.sunStarburstSpikes;
    out.sunStarburstSharpness = src.sunStarburstSharpness;
    out.jitterStrength = src.jitterStrength;
    out.planetSurfaceSnapM = src.planetSurfaceSnapM;
    out.viewSteps = src.viewSteps;
    out.lightSteps = src.lightSteps;
    return out;
}

void Engine::reset_atmosphere_to_earth()
{
    if (!_engine || !_engine->_context) return;
    std::string keepName = _engine->_context->atmosphere.bodyName;
    _engine->_context->atmosphere = ::AtmosphereSettings{};
    _engine->_context->atmosphere.bodyName = std::move(keepName);
}

// ----------------------------------------------------------------------------
// Sun Shadow (Penumbra)
// ----------------------------------------------------------------------------

void Engine::set_sun_shadow_settings(const SunShadowSettings &settings)
{
    if (!_engine || !_engine->_context) return;
    _engine->_context->shadowSettings.planetSunAngularRadiusDeg = settings.angularRadiusDeg;
}

SunShadowSettings Engine::get_sun_shadow_settings() const
{
    SunShadowSettings out{};
    if (!_engine || !_engine->_context) return out;
    out.angularRadiusDeg = _engine->_context->shadowSettings.planetSunAngularRadiusDeg;
    return out;
}

} // namespace GameAPI
