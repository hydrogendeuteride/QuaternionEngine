# Scene Management

The Game API provides instance management for glTF models and primitive meshes with support for both single and double-precision transforms.

## Transform Types

### Single Precision (Standard)

```cpp
struct Transform
{
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};

    glm::mat4 to_matrix() const;
    static Transform from_matrix(const glm::mat4& m);
};
```

Use for typical games where world coordinates fit in single precision.

### Double Precision (Large Worlds)

```cpp
struct TransformD
{
    glm::dvec3 position{0.0};      // Double-precision position
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};

    glm::mat4 to_matrix() const;
    static TransformD from_matrix(const glm::mat4& m);
};
```

Use for space games, flight simulators, or any scenario with large world coordinates where single-precision floats lose sub-meter precision.

## glTF Instances

### Adding Instances

```cpp
// Single precision
GameAPI::Transform t;
t.position = glm::vec3(0.0f, 0.0f, -5.0f);
t.rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
t.scale = glm::vec3(1.5f);

bool success = api.add_gltf_instance("player", "characters/player.gltf", t, true);

// Double precision (for large worlds)
GameAPI::TransformD td;
td.position = glm::dvec3(1000000.0, 0.0, 500000.0);  // Large coordinates
bool success = api.add_gltf_instance("spaceship", "vehicles/ship.gltf", td, true);
```

**Parameters:**
- `name` — Unique identifier for this instance
- `modelPath` — Relative to `assets/models/`
- `transform` — Initial transform (single or double-precision)
- `preloadTextures` — If `true`, immediately queue textures for loading

### Async Loading

For large models, use async loading to avoid blocking:

```cpp
uint32_t jobId = api.add_gltf_instance_async("boss", "enemies/boss.gltf", t, true);

if (jobId == 0) {
    // Failed to queue job
}
```

Returns a job ID (0 on failure). The instance will appear in the scene when loading completes.

### Removing Instances

```cpp
bool removed = api.remove_gltf_instance("player");
```

Returns `false` if instance doesn't exist.

### Transform Queries

```cpp
// Single precision
GameAPI::Transform t;
if (api.get_gltf_instance_transform("player", t)) {
    fmt::println("Position: {}, {}, {}", t.position.x, t.position.y, t.position.z);
}

// Double precision
GameAPI::TransformD td;
if (api.get_gltf_instance_transform("spaceship", td)) {
    fmt::println("Position: {}, {}, {}", td.position.x, td.position.y, td.position.z);
}
```

### Transform Updates

```cpp
// Single precision
GameAPI::Transform newTransform;
newTransform.position = glm::vec3(10.0f, 0.0f, 0.0f);
api.set_gltf_instance_transform("player", newTransform);

// Double precision
GameAPI::TransformD newTransformD;
newTransformD.position = glm::dvec3(2000000.0, 0.0, 1000000.0);
api.set_gltf_instance_transform("spaceship", newTransformD);
```

Returns `false` if instance doesn't exist.

### Texture Preloading

```cpp
// Preload textures for an instance before it becomes visible
api.preload_instance_textures("boss");
```

Useful for background loading while player approaches.

## Primitive Mesh Instances

### Primitive Types

```cpp
enum class PrimitiveType
{
    Cube,
    Sphere,
    Plane,
    Capsule
};
```

### Adding Primitives (Untextured)

```cpp
// Single precision
GameAPI::Transform t;
t.position = glm::vec3(0.0f, 1.0f, -5.0f);
t.scale = glm::vec3(2.0f, 1.0f, 2.0f);

api.add_primitive_instance("platform", GameAPI::PrimitiveType::Cube, t);

// Double precision
GameAPI::TransformD td;
td.position = glm::dvec3(1000000.0, 0.0, 0.0);
api.add_primitive_instance("asteroid", GameAPI::PrimitiveType::Sphere, td);
```

Untextured primitives use default placeholder materials.

### Adding Textured Primitives

```cpp
// Define PBR material
GameAPI::PrimitiveMaterial mat;
mat.albedoPath = "textures/brick_albedo.png";
mat.normalPath = "textures/brick_normal.png";
mat.metalRoughPath = "textures/brick_mro.png";  // Metallic-Roughness-Occlusion
mat.roughness = 0.7f;
mat.metallic = 0.0f;
mat.colorFactor = glm::vec4(1.0f);  // White (no tint)

// Spawn textured cube
GameAPI::Transform t;
t.position = glm::vec3(0.0f, 1.0f, -5.0f);
t.scale = glm::vec3(2.0f);

api.add_textured_primitive("brick_wall", GameAPI::PrimitiveType::Cube, mat, t);
```

**Material Paths:**
- All paths are relative to `assets/`
- Empty paths use default placeholder textures:
  - Albedo: magenta/black checkerboard
  - Normal: flat (0.5, 0.5, 1.0)
  - MetalRough: white
  - Occlusion: white (no occlusion)
  - Emissive: black (no emission)

**PrimitiveMaterial Structure:**

```cpp
struct PrimitiveMaterial
{
    std::string albedoPath;        // Color/diffuse texture
    std::string metalRoughPath;    // Metallic (R) + Roughness (G)
    std::string normalPath;        // Tangent-space normal map
    std::string occlusionPath;     // Ambient occlusion (R channel)
    std::string emissivePath;      // Emissive map

    glm::vec4 colorFactor{1.0f};   // Base color multiplier (RGBA)
    float metallic{0.0f};          // Metallic factor (0-1)
    float roughness{0.5f};         // Roughness factor (0-1)
};
```

### Removing Mesh Instances

```cpp
bool removed = api.remove_mesh_instance("platform");
```

Works for both untextured and textured primitives.

### Mesh Transform Queries

```cpp
// Single precision
GameAPI::Transform t;
if (api.get_mesh_instance_transform("platform", t)) {
    // ...
}

// Double precision
GameAPI::TransformD td;
if (api.get_mesh_instance_transform("asteroid", td)) {
    // ...
}
```

### Mesh Transform Updates

```cpp
// Single precision
GameAPI::Transform newT;
newT.position = glm::vec3(5.0f, 1.0f, -5.0f);
api.set_mesh_instance_transform("platform", newT);

// Double precision
GameAPI::TransformD newTD;
newTD.position = glm::dvec3(1000010.0, 0.0, 0.0);
api.set_mesh_instance_transform("asteroid", newTD);
```

## Clearing All Instances

```cpp
// Remove all dynamic instances (glTF + primitives)
api.clear_all_instances();
```

## Complete Example

```cpp
GameAPI::Engine api(&engine);

// Add character
GameAPI::Transform charT;
charT.position = glm::vec3(0.0f, 0.0f, 0.0f);
api.add_gltf_instance("player", "characters/player.gltf", charT);

// Add textured ground plane
GameAPI::PrimitiveMaterial groundMat;
groundMat.albedoPath = "textures/ground_albedo.png";
groundMat.normalPath = "textures/ground_normal.png";
groundMat.roughness = 0.9f;

GameAPI::Transform groundT;
groundT.position = glm::vec3(0.0f, -0.5f, 0.0f);
groundT.scale = glm::vec3(50.0f, 1.0f, 50.0f);
api.add_textured_primitive("ground", GameAPI::PrimitiveType::Plane, groundMat, groundT);

// Game loop: update player position
while (running) {
    float dt = api.get_delta_time();

    // Update player position
    GameAPI::Transform playerT;
    if (api.get_gltf_instance_transform("player", playerT)) {
        playerT.position += glm::vec3(velocity * dt);
        api.set_gltf_instance_transform("player", playerT);
    }
}
```

## See Also

- [Animation](Animation.md) — Control glTF animations
- [Picking](Picking.md) — Select and interact with instances
- [Scene Manager](../Scene.md) — Low-level scene graph details
