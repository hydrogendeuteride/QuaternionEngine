# Planet System

A system for rendering planetary-scale terrain and celestial bodies using cube-sphere based LOD quadtree tessellation.

## Overview

PlanetSystem supports two types of planets:

1. **Mesh Planet (PlanetSphere)**: Simple sphere mesh rendering (e.g., Moon)
2. **Terrain Planet (PlanetTerrain)**: Detailed terrain with cube-sphere quadtree patches (e.g., Earth)

## Architecture

```
src/scene/planet/
├── planet_system.h/.cpp    # Main system class
├── planet_quadtree.h/.cpp  # LOD quadtree management
├── cubesphere.h/.cpp       # Cube-sphere geometry generation
└── planet_heightmap.h/.cpp # Heightmap loading and sampling
```

### Core Components

- **PlanetSystem**: Manages planet creation, updates, and rendering
- **PlanetQuadtree**: View-dependent LOD selection and visible patch determination
- **Cubesphere**: Generates cube-sphere geometry with 6 faces projected onto a sphere
- **HeightFace**: Loads and samples BC4 compressed KTX2 heightmaps

## GameAPI Usage

### Basic Planet Creation

```cpp
#include "core/game_api.h"

GameAPI::Engine api(&engine);

// Earth-scale constants
constexpr double kEarthRadiusM = 6378137.0;      // 6,378km
constexpr double kMoonRadiusM = 1737400.0;       // 1,737km
constexpr double kMoonDistanceM = 384400000.0;   // 384,400km

// Tessellated terrain planet (Earth)
GameAPI::PlanetTerrain earth{};
earth.name = "Earth";
earth.center = glm::dvec3(0.0, 0.0, 0.0);
earth.radius_m = kEarthRadiusM;
earth.visible = true;
earth.base_color = glm::vec4(1.0f);
earth.metallic = 0.0f;
earth.roughness = 1.0f;

// Texture paths (relative to assets/)
earth.albedo_dir = "planets/earth/albedo/L0";
earth.height_dir = "planets/earth/height/L0";
earth.height_max_m = 6400.0;  // Maximum heightmap displacement in meters
earth.emission_dir = "planets/earth/emission/L0";
earth.emission_factor = glm::vec3(1.0f, 1.0f, 1.0f);

api.add_planet_terrain(earth);

// Simple sphere planet (Moon)
GameAPI::PlanetSphere moon{};
moon.name = "Moon";
moon.center = glm::dvec3(kMoonDistanceM, 0.0, 0.0);
moon.radius_m = kMoonRadiusM;
moon.visible = true;
moon.base_color = glm::vec4(0.72f, 0.72f, 0.75f, 1.0f);
moon.metallic = 0.0f;
moon.roughness = 1.0f;
moon.sectors = 48;  // Longitude divisions
moon.stacks = 24;   // Latitude divisions

api.add_planet_sphere(moon);
```

### Camera Setup

For planetary-scale scenes, camera speed adjustment is essential:

```cpp
// Increase camera speed for planetary scale
GameAPI::FreeCameraSettings free = api.get_free_camera_settings();
free.moveSpeed = 20000.0f;  // 20km/s
api.set_free_camera_settings(free);

// Position camera in orbit looking at planet
api.set_camera_position(glm::dvec3(0.0, 0.0, kEarthRadiusM + 1.0e6));
api.camera_look_at(glm::dvec3(0.0, 0.0, 0.0));
```

### Quadtree Settings

```cpp
GameAPI::PlanetQuadtreeSettings qt{};
qt.maxLevel = 14;                   // Maximum LOD level (2^14 subdivisions)
qt.targetScreenSpaceError = 32.0f;  // Target screen space error in pixels
qt.maxPatchesVisible = 8192;        // Maximum visible patches
qt.frustumCull = true;              // Enable frustum culling
qt.horizonCull = true;              // Enable horizon culling

api.set_planet_quadtree_settings(qt);
```

### Patch Creation Control

```cpp
// Per-frame patch creation budget
api.set_planet_patch_create_budget(16);        // Max patches per frame
api.set_planet_patch_create_budget_ms(2.0f);   // Max time per frame (milliseconds)

// Patch resolution (33 = 32x32 vertex grid + skirts)
api.set_planet_patch_resolution(33);

// Patch cache size
api.set_planet_patch_cache_max(2048);
```

### Debug and Statistics

```cpp
// Color tinting by LOD level (for debugging)
api.set_planet_debug_tint_by_lod(true);

// Query terrain statistics
GameAPI::PlanetTerrainStats stats = api.get_planet_terrain_stats("Earth");
fmt::println("Visible patches: {}", stats.visiblePatches);
fmt::println("Rendered patches: {}", stats.renderedPatches);
fmt::println("Cache size: {}", stats.patchCacheSize);
fmt::println("Estimated triangles: {}", stats.estimatedTriangles);

// Sample terrain height at a direction
glm::dvec3 direction = glm::normalize(glm::dvec3(1.0, 0.0, 0.0));
double height_m = api.sample_planet_terrain_height("Earth", direction);
```

## Texture Asset Structure

Planet textures are organized in cube-face format, with each face stored as KTX2:

```
assets/planets/earth/
├── albedo/L0/
│   ├── px.ktx2    # +X face (east)
│   ├── nx.ktx2    # -X face (west)
│   ├── py.ktx2    # +Y face (north pole)
│   ├── ny.ktx2    # -Y face (south pole)
│   ├── pz.ktx2    # +Z face (front)
│   └── nz.ktx2    # -Z face (back)
├── height/L0/
│   └── {px,nx,py,ny,pz,nz}.ktx2  # BC4/R8, linear
└── emission/L0/
    └── {px,nx,py,ny,pz,nz}.ktx2  # or .png, sRGB
```

### Texture Formats

| Texture Type | Format | Color Space | Purpose |
|--------------|--------|-------------|---------|
| Albedo | KTX2 (BC7 recommended) | sRGB | Base color |
| Height | KTX2 (BC4/R8) | Linear | Terrain displacement |
| Emission | KTX2 or PNG | sRGB | Night lights |

## Cube-Sphere Geometry

### Face Order

Follows Vulkan/KTX cubemap face ordering:

| Index | Face | Direction |
|-------|------|-----------|
| 0 | PosX | +X (right) |
| 1 | NegX | -X (left) |
| 2 | PosY | +Y (up) |
| 3 | NegY | -Y (down) |
| 4 | PosZ | +Z (front) |
| 5 | NegZ | -Z (back) |

### LOD Quadtree

Each face is recursively subdivided as a quadtree:

- **Level 0**: 1 patch per face (6 total)
- **Level N**: 4^N patches per face
- **Max Level 14**: ~1.6 billion virtual patches (only needed ones are created)

Subdivision criteria:
1. **Screen Space Error**: Split when patch's screen error exceeds threshold
2. **Frustum Culling**: Exclude patches outside view frustum
3. **Horizon Culling**: Exclude patches below the horizon

## Rendering Pipeline

### Patch Mesh Structure

Each patch consists of:

- **Vertex Grid**: `resolution × resolution` (default 33×33)
- **Skirts**: Extra vertices to prevent LOD boundary gaps (4 × resolution)
- **Index Buffer**: Shared by all patches (identical when resolution matches)

### Material System

Per-patch cube-face textures are bound:

```
Binding 0: Material Constants (UBO)
Binding 1: Albedo Texture (Combined Sampler)
Binding 2: Metal-Roughness Texture
Binding 3: Normal Map
Binding 4: Occlusion
Binding 5: Emission Texture
```

### Height Displacement

When heightmaps are provided, displacement is applied during vertex processing:

```cpp
float h01 = sample_height(height_face, uv.x, uv.y);
float h_m = h01 * height_max_m;
vertex.position += vertex.normal * h_m;
```

Normals are recomputed using central differencing after displacement.

## Performance Considerations

### Memory

- **Patch Cache**: Control VRAM usage with `set_planet_patch_cache_max()`
- **LRU Policy**: Least recently used patches are evicted first

### CPU Load

- **Patch Creation Budget**: Limit per-frame creation with `set_planet_patch_create_budget()`
- **Time Budget**: Limit creation time with `set_planet_patch_create_budget_ms()`

### GPU Load

- **Patch Resolution**: Adjust triangle count with `set_planet_patch_resolution()`
- **Screen Space Error**: Increasing `targetScreenSpaceError` reduces patch count

## Debug Statistics

`PlanetTerrainStats` structure:

| Field | Description |
|-------|-------------|
| `visiblePatches` | Patches selected by quadtree |
| `renderedPatches` | Actually rendered patches |
| `createdPatches` | Patches created this frame |
| `patchCacheSize` | Total patches in cache |
| `estimatedTriangles` | Estimated triangle count |
| `ms_quadtree` | Quadtree update time |
| `ms_patch_create` | Patch creation time |
| `ms_emit` | Draw command generation time |
| `ms_total` | Total system time |

## Related Documentation

- `docs/FloatingOrigin.md` - Large-world coordinate handling
- `docs/GameAPI.md` - Complete game API reference
- `docs/TextureLoading.md` - KTX2 texture loading
- `docs/Scene.md` - Scene management and cameras
