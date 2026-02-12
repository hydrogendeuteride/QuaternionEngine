# Image-Based Lighting (IBL)

Control global and local IBL volumes for realistic ambient lighting and reflections.

## IBL Assets

IBL requires pre-processed cubemap textures:

```cpp
struct IBLPaths
{
    std::string specularCube;  // .ktx2 pre-filtered specular cubemap
    std::string diffuseCube;   // .ktx2 irradiance cubemap
    std::string brdfLut;       // .ktx2 BRDF lookup table
    std::string background;    // .ktx2 background (optional, falls back to specular)
};
```

All paths are relative to `assets/`.

## Global IBL

The global IBL is used when the camera is not inside any local IBL volume.

### Loading Global IBL

```cpp
GameAPI::IBLPaths globalIBL;
globalIBL.specularCube = "ibl/outdoor_spec.ktx2";
globalIBL.diffuseCube = "ibl/outdoor_diff.ktx2";
globalIBL.brdfLut = "ibl/brdf_lut.ktx2";
globalIBL.background = "ibl/outdoor_bg.ktx2";  // Optional

bool success = api.load_global_ibl(globalIBL);
```

This loads textures asynchronously. Returns `false` if failed to queue.

### Get/Set Global IBL Paths

```cpp
// Get current paths (doesn't trigger reload)
GameAPI::IBLPaths paths = api.get_global_ibl_paths();

// Set new paths (doesn't trigger reload - use load_global_ibl to reload)
api.set_global_ibl_paths(paths);
```

## Local IBL Volumes

Local IBL volumes override global IBL when the camera is inside them. Useful for interiors, tunnels, etc.

### Data Structures

```cpp
enum class IBLVolumeShape : uint8_t
{
    Box = 0,
    Sphere = 1
};

// Single precision
struct IBLVolume
{
    glm::vec3 center{0.0f};
    glm::vec3 halfExtents{10.0f};  // Box only
    IBLPaths paths;
    bool enabled{true};
    IBLVolumeShape shape{IBLVolumeShape::Box};
    float radius{10.0f};           // Sphere only
};

// Double precision
struct IBLVolumeD
{
    glm::dvec3 center{0.0};
    glm::vec3 halfExtents{10.0f};
    IBLPaths paths;
    bool enabled{true};
    IBLVolumeShape shape{IBLVolumeShape::Box};
    float radius{10.0f};
};
```

### Adding Box Volumes

```cpp
// Single precision
GameAPI::IBLVolume interior;
interior.center = glm::vec3(10.0f, 2.0f, -5.0f);
interior.halfExtents = glm::vec3(5.0f, 3.0f, 5.0f);
interior.shape = GameAPI::IBLVolumeShape::Box;
interior.paths.specularCube = "ibl/indoor_spec.ktx2";
interior.paths.diffuseCube = "ibl/indoor_diff.ktx2";
interior.paths.brdfLut = "ibl/brdf_lut.ktx2";
interior.enabled = true;

size_t idx = api.add_ibl_volume(interior);

// Double precision
GameAPI::IBLVolumeD interiorD;
interiorD.center = glm::dvec3(1000000.0, 10.0, 500000.0);
interiorD.halfExtents = glm::vec3(50.0f, 30.0f, 50.0f);
interiorD.shape = GameAPI::IBLVolumeShape::Box;
interiorD.paths = interior.paths;
interiorD.enabled = true;

size_t idxD = api.add_ibl_volume(interiorD);
```

### Adding Sphere Volumes

```cpp
// Single precision helper
size_t idx = api.add_ibl_sphere_volume(
    glm::vec3(0.0f, 0.0f, 0.0f),  // Center
    20.0f,                         // Radius
    paths,                         // IBL paths
    true                           // Enabled
);

// Double precision helper
size_t idxD = api.add_ibl_sphere_volume(
    glm::dvec3(1000000.0, 0.0, 0.0),
    500.0f,
    paths,
    true
);

// Or construct manually
GameAPI::IBLVolume sphere;
sphere.center = glm::vec3(0.0f, 10.0f, 0.0f);
sphere.radius = 15.0f;
sphere.shape = GameAPI::IBLVolumeShape::Sphere;
sphere.paths = paths;
sphere.enabled = true;

size_t idx = api.add_ibl_volume(sphere);
```

### Querying Volumes

```cpp
// Get volume count
size_t count = api.get_ibl_volume_count();

// Get specific volume (single precision)
GameAPI::IBLVolume volume;
if (api.get_ibl_volume(idx, volume)) {
    fmt::println("IBL center: {}, {}, {}", volume.center.x, volume.center.y, volume.center.z);
}

// Get specific volume (double precision)
GameAPI::IBLVolumeD volumeD;
if (api.get_ibl_volume(idx, volumeD)) {
    // ...
}

// Get active volume index
int activeIdx = api.get_active_ibl_volume();
if (activeIdx == -1) {
    // Using global IBL
} else {
    // Using local volume at index activeIdx
}
```

### Updating Volumes

```cpp
// Single precision
GameAPI::IBLVolume volume;
api.get_ibl_volume(idx, volume);
volume.enabled = false;  // Disable volume
api.set_ibl_volume(idx, volume);

// Double precision
GameAPI::IBLVolumeD volumeD;
api.get_ibl_volume(idx, volumeD);
volumeD.center = glm::dvec3(2000000.0, 0.0, 0.0);
api.set_ibl_volume(idx, volumeD);
```

### Removing Volumes

```cpp
bool removed = api.remove_ibl_volume(idx);
```

### Clearing All Volumes

```cpp
api.clear_ibl_volumes();
```

## Volume Selection

The engine automatically selects the active IBL volume based on camera position:

1. Check if camera is inside any enabled local volume (box or sphere)
2. If inside multiple volumes, use the first match
3. If inside no volumes, use global IBL

## Complete Example

```cpp
GameAPI::Engine api(&engine);

// Load global outdoor IBL
GameAPI::IBLPaths outdoor;
outdoor.specularCube = "ibl/sky_spec.ktx2";
outdoor.diffuseCube = "ibl/sky_diff.ktx2";
outdoor.brdfLut = "ibl/brdf_lut.ktx2";
outdoor.background = "ibl/sky_bg.ktx2";
api.load_global_ibl(outdoor);

// Add interior room volume (box)
GameAPI::IBLVolume room1;
room1.center = glm::vec3(0.0f, 2.0f, -10.0f);
room1.halfExtents = glm::vec3(8.0f, 3.0f, 8.0f);
room1.shape = GameAPI::IBLVolumeShape::Box;
room1.paths.specularCube = "ibl/room1_spec.ktx2";
room1.paths.diffuseCube = "ibl/room1_diff.ktx2";
room1.paths.brdfLut = "ibl/brdf_lut.ktx2";
room1.enabled = true;
size_t room1Idx = api.add_ibl_volume(room1);

// Add cave entrance volume (sphere)
size_t caveIdx = api.add_ibl_sphere_volume(
    glm::vec3(50.0f, 0.0f, 20.0f),
    15.0f,
    cave_paths,
    true
);

// Query which IBL is active
int activeIdx = api.get_active_ibl_volume();
if (activeIdx == -1) {
    fmt::println("Using global outdoor IBL");
} else if (activeIdx == room1Idx) {
    fmt::println("Inside room 1");
} else if (activeIdx == caveIdx) {
    fmt::println("Inside cave entrance");
}
```

## Use Cases

### Interior/Exterior Transitions

```cpp
// Place box volume at doorway
GameAPI::IBLVolume doorway;
doorway.center = door_position;
doorway.halfExtents = glm::vec3(2.0f, 3.0f, 1.0f);
doorway.shape = GameAPI::IBLVolumeShape::Box;
doorway.paths = interior_ibl;
api.add_ibl_volume(doorway);
```

### Tunnel Lighting

```cpp
// Chain of sphere volumes along tunnel
for (const glm::vec3& pos : tunnel_positions) {
    api.add_ibl_sphere_volume(pos, 10.0f, tunnel_ibl, true);
}
```

### Dynamic Volume Enabling

```cpp
// Enable/disable room volumes based on quest state
GameAPI::IBLVolume room;
api.get_ibl_volume(roomIdx, room);
room.enabled = quest_completed;
api.set_ibl_volume(roomIdx, room);
```

## Performance Considerations

- Each volume incurs a small per-frame cost for camera-inside checks
- Prefer fewer, larger volumes over many small overlapping volumes
- IBL texture loading is async and doesn't block rendering
- Textures are cached and shared across volumes (reuse `brdfLut` path)

## Generating IBL Textures

IBL textures must be pre-processed from HDR environment maps. Tools:

- [cmftStudio](https://github.com/dariomanesku/cmftStudio) — Cubemap filtering
- [IBLBaker](https://github.com/derkreature/IBLBaker) — PBR IBL generation
- Custom: Use glTF-IBL-Sampler or similar

Expected formats:
- Specular: Pre-filtered cubemap with mip chain (roughness levels)
- Diffuse: Irradiance cubemap (single mip)
- BRDF LUT: 2D lookup table (512×512 typical)

## See Also

- [Lighting](Lighting.md) — Point, spot, and directional lights
- [Post-Processing](PostProcessing.md) — Reflections (SSR/RT)
- [IBL System](../IBL.md) — Low-level implementation details
