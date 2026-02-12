# Planet System

Procedural planets with sphere meshes or LOD terrain, atmosphere scattering, and cloud layers.

## Planet Types

The engine supports two planet types:

1. **Sphere Planets** — Simple procedural sphere meshes (fast, no LOD)
2. **Terrain Planets** — Cube-sphere quadtree patches with displacement (LOD, high detail)

## Planet Sphere

Simple sphere planet with procedural mesh.

### Data Structure

```cpp
struct PlanetSphere
{
    std::string name;
    glm::dvec3 center{0.0};         // World position (double precision)
    double radius_m{1.0};           // Radius in meters
    bool visible{true};

    // Material
    glm::vec4 base_color{1.0f};
    float metallic{0.0f};
    float roughness{1.0f};

    // Mesh tessellation
    uint32_t sectors{48};
    uint32_t stacks{24};
};
```

### Creating Sphere Planets

```cpp
GameAPI::PlanetSphere moon;
moon.name = "Moon";
moon.center = glm::dvec3(0.0, 0.0, -384400000.0);  // ~384,400 km
moon.radius_m = 1737400.0;  // ~1,737 km
moon.visible = true;
moon.base_color = glm::vec4(0.6f, 0.6f, 0.6f, 1.0f);  // Gray
moon.metallic = 0.0f;
moon.roughness = 0.9f;
moon.sectors = 64;
moon.stacks = 32;

bool success = api.add_planet_sphere(moon);
```

## Planet Terrain

High-detail terrain planet with quadtree LOD and optional textures.

### Data Structure

```cpp
struct PlanetTerrain
{
    std::string name;
    glm::dvec3 center{0.0};
    double radius_m{1.0};
    bool visible{true};

    // Material
    glm::vec4 base_color{1.0f};
    float metallic{0.0f};
    float roughness{1.0f};

    // Terrain textures (cube faces: {px,nx,py,ny,pz,nz}.ktx2)
    std::string albedo_dir;         // e.g., "planets/earth/albedo/L0"
    std::string height_dir;         // Height map for displacement
    double height_max_m{6400.0};    // Height range in meters for [0..1] texel values

    // Emission textures (optional, e.g., city lights)
    std::string emission_dir;       // e.g., "planets/earth/emission/L0"
    glm::vec3 emission_factor{0.0f, 0.0f, 0.0f};  // Emission intensity multiplier
};
```

### Creating Terrain Planets

```cpp
GameAPI::PlanetTerrain earth;
earth.name = "Earth";
earth.center = glm::dvec3(0.0, 0.0, 0.0);
earth.radius_m = 6371000.0;  // ~6,371 km
earth.visible = true;
earth.base_color = glm::vec4(0.2f, 0.4f, 0.8f, 1.0f);  // Blue
earth.metallic = 0.0f;
earth.roughness = 0.9f;

// Terrain textures (cube faces)
earth.albedo_dir = "planets/earth/albedo/L0";
earth.height_dir = "planets/earth/height/L0";
earth.height_max_m = 8848.0;  // Mt. Everest height

// City lights at night
earth.emission_dir = "planets/earth/emission/L0";
earth.emission_factor = glm::vec3(1.0f, 0.9f, 0.7f);

bool success = api.add_planet_terrain(earth);
```

**Texture Layout:**
- Each directory should contain 6 KTX2 files: `px.ktx2`, `nx.ktx2`, `py.ktx2`, `ny.ktx2`, `pz.ktx2`, `nz.ktx2`
- Albedo: sRGB color (BC7/ASTC)
- Height: Linear R8 or BC4 (0 = radius, 1 = radius + height_max_m)
- Emission: sRGB color (BC7/ASTC)

## Managing Planets

### Remove Planet

```cpp
bool removed = api.remove_planet("Moon");
```

### Clear All Planets

```cpp
// Clear all planets, optionally destroy mesh assets
api.clear_planets(true);  // true = destroy mesh assets
```

### Enable/Disable Planet System

```cpp
api.set_planet_system_enabled(true);
bool enabled = api.get_planet_system_enabled();
```

## Querying Planets

```cpp
// Get planet by name
GameAPI::PlanetInfo info;
if (api.get_planet("Earth", info)) {
    fmt::println("Radius: {} km", info.radius_m / 1000.0);
}

// Get planet count
size_t count = api.get_planet_count();

// Get planet by index
GameAPI::PlanetInfo info;
if (api.get_planet(0, info)) {
    fmt::println("Planet 0: {}", info.name);
}
```

### PlanetInfo Structure

```cpp
struct PlanetInfo
{
    std::string name;
    glm::dvec3 center{0.0};
    double radius_m{1.0};
    bool visible{true};
    bool terrain{false};  // True if terrain planet, false if sphere
};
```

## Editing Planets

### Set Planet Center

```cpp
api.set_planet_center("Earth", glm::dvec3(1000000.0, 0.0, 0.0));
```

### Set Planet Radius

```cpp
api.set_planet_radius("Earth", 6371000.0);
```

### Set Planet Visibility

```cpp
api.set_planet_visible("Moon", false);  // Hide moon
```

### Convert Sphere to Terrain (or vice versa)

```cpp
api.set_planet_terrain("Earth", true);  // Enable terrain LOD
api.set_planet_terrain("Moon", false);  // Disable terrain (use sphere mesh)
```

## Terrain LOD Settings

Control quadtree LOD behavior for terrain planets.

### Quadtree Settings

```cpp
struct PlanetQuadtreeSettings
{
    uint32_t maxLevel{14};                 // Max quadtree depth
    float targetScreenSpaceError{32.0f};   // Target SSE in pixels
    uint32_t maxPatchesVisible{8192};      // Max patches to render
    bool frustumCull{true};                // Enable frustum culling
    bool horizonCull{true};                // Enable horizon culling
};

GameAPI::PlanetQuadtreeSettings settings;
settings.maxLevel = 16;
settings.targetScreenSpaceError = 16.0f;  // Higher detail (lower SSE)
settings.maxPatchesVisible = 16384;
api.set_planet_quadtree_settings(settings);

GameAPI::PlanetQuadtreeSettings current = api.get_planet_quadtree_settings();
```

### Patch Creation Budget

```cpp
// Limit patches created per frame (prevents hitches)
api.set_planet_patch_create_budget(32);
uint32_t budget = api.get_planet_patch_create_budget();

// Or set budget by time (milliseconds)
api.set_planet_patch_create_budget_ms(2.0f);
float budgetMs = api.get_planet_patch_create_budget_ms();
```

### Patch Resolution and Cache

```cpp
// Set patch mesh resolution (vertices per edge)
api.set_planet_patch_resolution(64);
uint32_t resolution = api.get_planet_patch_resolution();

// Set max patches in cache
api.set_planet_patch_cache_max(4096);
uint32_t maxCache = api.get_planet_patch_cache_max();
```

## Terrain Debugging

### LOD Tinting

```cpp
// Tint patches by LOD level (debug visualization)
api.set_planet_debug_tint_by_lod(true);
bool tinting = api.get_planet_debug_tint_by_lod();
```

### Terrain Statistics

```cpp
GameAPI::PlanetTerrainStats stats = api.get_planet_terrain_stats("Earth");

fmt::println("Visible patches: {}", stats.visiblePatches);
fmt::println("Rendered patches: {}", stats.renderedPatches);
fmt::println("Created patches: {}", stats.createdPatches);
fmt::println("Cache size: {}", stats.patchCacheSize);
fmt::println("Triangles: {}", stats.estimatedTriangles);
fmt::println("Max LOD: {}", stats.maxLevelUsed);
fmt::println("Quadtree time: {:.2f} ms", stats.msQuadtree);
fmt::println("Patch create time: {:.2f} ms", stats.msPatchCreate);
fmt::println("Total time: {:.2f} ms", stats.msTotal);
```

### Sample Terrain Height

```cpp
// Get terrain height at a direction from planet center
glm::dvec3 direction = glm::normalize(sample_position - planet_center);
double height_m = api.sample_planet_terrain_height("Earth", direction);
```

Returns meters above base radius (0 = sea level).

## Atmosphere

### Atmosphere Settings

```cpp
struct AtmosphereSettings
{
    std::string bodyName{};           // Planet name (empty = auto-select closest)
    float atmosphereHeightM{80000.0f};

    // Scale heights
    float rayleighScaleHeightM{8000.0f};
    float mieScaleHeightM{1200.0f};

    // Scattering coefficients (1/m)
    glm::vec3 rayleighScattering{5.802e-6f, 13.558e-6f, 33.1e-6f};
    glm::vec3 mieScattering{21.0e-6f};

    // Absorption
    glm::vec3 absorptionColor{1.0f, 1.0f, 1.0f};
    float absorptionStrength{0.0f};

    // Phase function
    float mieG{0.76f};

    // Artistic controls
    float intensity{1.0f};
    float sunDiskIntensity{1.0f};
    float sunHaloIntensity{0.0f};
    float sunHaloRadiusDeg{2.0f};
    float sunStarburstIntensity{0.0f};
    float sunStarburstRadiusDeg{6.0f};
    int sunStarburstSpikes{8};
    float sunStarburstSharpness{12.0f};

    // Rendering quality
    float jitterStrength{0.0f};
    float planetSurfaceSnapM{200.0f};
    int viewSteps{16};
    int lightSteps{8};
};
```

### Enable/Configure Atmosphere

```cpp
// Enable atmosphere rendering
api.set_atmosphere_enabled(true);
bool enabled = api.get_atmosphere_enabled();

// Set atmosphere settings
GameAPI::AtmosphereSettings atmo;
atmo.bodyName = "Earth";
atmo.atmosphereHeightM = 100000.0f;
atmo.intensity = 1.2f;
atmo.viewSteps = 24;
atmo.lightSteps = 10;
api.set_atmosphere_settings(atmo);

// Get current settings
GameAPI::AtmosphereSettings current = api.get_atmosphere_settings();

// Reset to Earth defaults
api.reset_atmosphere_to_earth();
```

## Planet Clouds

### Cloud Settings

```cpp
struct PlanetCloudSettings
{
    float baseHeightM{2000.0f};
    float thicknessM{8000.0f};
    float densityScale{1.0f};
    float coverage{0.45f};

    // Overlay texture (cloud texture wrapped on sphere)
    std::string overlayTexturePath{"planets/earth/cloud/earth_clouds_4k.ktx2"};
    float overlayRotationRad{0.0f};
    bool overlayFlipV{false};

    // Procedural noise
    float noiseScale{1.5f};
    float detailScale{12.0f};

    // Animation
    float windSpeed{0.0f};
    float windAngleRad{0.0f};

    // Rendering quality
    int cloudSteps{32};
};
```

### Enable/Configure Clouds

```cpp
// Enable planet clouds
api.set_planet_clouds_enabled(true);
bool enabled = api.get_planet_clouds_enabled();

// Set cloud settings
GameAPI::PlanetCloudSettings clouds;
clouds.baseHeightM = 3000.0f;
clouds.thicknessM = 10000.0f;
clouds.coverage = 0.5f;
clouds.windSpeed = 5.0f;
clouds.cloudSteps = 48;
api.set_planet_clouds_settings(clouds);

// Get current settings
GameAPI::PlanetCloudSettings current = api.get_planet_clouds_settings();

// Reset to defaults
api.reset_planet_clouds_defaults();
```

## Sun Shadows (Penumbra)

Control soft shadows from the sun on planets.

```cpp
struct SunShadowSettings
{
    float angularRadiusDeg{0.27f};  // Sun half-angle (0 = hard edge, ~0.27 = real sun)
};

GameAPI::SunShadowSettings sunShadow;
sunShadow.angularRadiusDeg = 0.5f;  // Larger sun (softer shadows)
api.set_sun_shadow_settings(sunShadow);

GameAPI::SunShadowSettings current = api.get_sun_shadow_settings();
```

## Complete Example

```cpp
GameAPI::Engine api(&engine);

// Create Earth with terrain
GameAPI::PlanetTerrain earth;
earth.name = "Earth";
earth.center = glm::dvec3(0.0, 0.0, 0.0);
earth.radius_m = 6371000.0;
earth.albedo_dir = "planets/earth/albedo/L0";
earth.height_dir = "planets/earth/height/L0";
earth.height_max_m = 8848.0;
earth.emission_dir = "planets/earth/emission/L0";
earth.emission_factor = glm::vec3(1.0f, 0.9f, 0.7f);
api.add_planet_terrain(earth);

// Configure terrain LOD
GameAPI::PlanetQuadtreeSettings quadtree;
quadtree.maxLevel = 18;
quadtree.targetScreenSpaceError = 16.0f;
api.set_planet_quadtree_settings(quadtree);

// Enable atmosphere
api.set_atmosphere_enabled(true);
api.reset_atmosphere_to_earth();

// Enable clouds
api.set_planet_clouds_enabled(true);
GameAPI::PlanetCloudSettings clouds;
clouds.coverage = 0.5f;
clouds.windSpeed = 10.0f;
api.set_planet_clouds_settings(clouds);

// Setup camera to orbit Earth
api.set_camera_position(glm::dvec3(0.0, 0.0, 15000000.0));  // 15,000 km
api.camera_look_at(glm::dvec3(0.0, 0.0, 0.0));
```

## See Also

- [Planet System](../PlanetSystem.md) — Low-level implementation details
- [Lighting](Lighting.md) — Sun direction for atmosphere
- [Camera](Camera.md) — Camera positioning for planet rendering
- [Floating Origin](../FloatingOrigin.md) — Large-world coordinate handling
