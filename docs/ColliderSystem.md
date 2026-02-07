## Collider System: glTF-Based Physics Collider Authoring

Provides a workflow for authoring physics colliders in 3D modeling tools (e.g., Blender) and automatically extracting them at glTF load time. Supports both embedded marker nodes and separate sidecar files.

### Components

- `gltf_collider_parser.h/.cpp` (src/physics/)
  - Core collider parsing logic with `GltfColliderMarkerType` enum.
  - `parse_collider_marker_type()` — parses `COL_*` prefixes (case-insensitive).
  - `build_colliders_from_markers()` — scans a glTF scene for marker nodes.
  - `build_colliders_from_sidecar()` — maps sidecar colliders to destination scene nodes.

- `collider_asset.h/.cpp` (src/physics/)
  - Uniform scaling utilities for collision shapes.
  - Used when instances have different scales than the base asset.

- `LoadedGLTF` collider extensions (src/scene/vk_loader.h/.cpp)
  - Thin wrappers that delegate to `Physics::build_colliders_*` functions.
  - Stores collider results in `collider_compounds` map.

- `SceneManager` collider sync (src/scene/vk_scene.h/.cpp)
  - Creates kinematic physics bodies from collider definitions.
  - Synchronizes body transforms with animated node transforms each frame.

### Authoring Workflow

#### 1. Marker Node Convention

In your 3D modeling tool, create empty objects (nodes without geometry) with specific naming prefixes:

| Prefix | Shape | Scale Interpretation |
|--------|-------|---------------------|
| `COL_BOX` | Box | X/Y/Z → full extents |
| `COL_SPHERE` | Sphere | max(X,Y,Z) → diameter |
| `COL_CAPSULE` | Capsule | max(X,Z) → diameter, Y → total height |
| `COL_CYLINDER` | Cylinder | max(X,Z) → diameter, Y → total height |
| `COL_TAPERED` | Tapered Cylinder | X → top diameter, Z → bottom diameter, Y → height |

The node's **local scale** defines the collider dimensions. Position and rotation relative to the parent define the collider's placement.

#### 2. Parent Hierarchy

Collider markers should be children of the node they belong to:

```
Character (MeshNode)
├── COL_CAPSULE_body
├── Arm_L (MeshNode)
│   └── COL_BOX_arm
└── Arm_R (MeshNode)
    └── COL_BOX_arm
```

The system finds the closest non-collider ancestor (preferring MeshNodes) as the "owner". Multiple colliders under the same owner are combined into a single `CompoundShape`.

#### 3. Sidecar Files (Optional)

For cleaner separation, you can export colliders to a separate file:

- Main model: `character.glb`
- Colliders: `character.colliders.glb`

The sidecar file should mirror the node hierarchy of the main file. Collider ownership is resolved by matching ancestor node names between sidecar and main scene.

Benefits:
- Keep visual assets clean (no extra empties in the main file).
- Iterate on colliders without re-exporting visuals.
- Different collider sets for different use cases.

### Automatic Loading

When `AssetManager::loadGLTF()` loads a model:

1. Check for sidecar file (`*.colliders.glb` or `*.colliders.gltf`).
2. If sidecar exists and loads successfully:
   - Build colliders from sidecar via `build_colliders_from_sidecar()`.
   - Set `colliders_from_sidecar = true`.
3. Otherwise:
   - Build colliders from embedded markers via `build_colliders_from_markers()`.
   - Set `colliders_from_sidecar = false`.

Result is stored in `LoadedGLTF::collider_compounds`:

```cpp
// Node name → compound shape (one or more primitives)
std::unordered_map<std::string, Physics::CompoundShape> collider_compounds;
```

### Runtime Collider Sync

For animated models, use `SceneManager::enableColliderSync()` to create physics bodies that track node transforms:

```cpp
// After adding a glTF instance
scene->addGLTFInstance("enemy", enemyGltf, transform);

// Enable collider sync with physics world
size_t bodyCount = scene->enableColliderSync(
    "enemy",           // instance name
    physicsWorld,      // Physics::PhysicsWorld*
    LAYER_ENEMY,       // collision layer
    enemyUserData      // optional user data for callbacks
);

fmt::println("Created {} collider bodies", bodyCount);
```

The system:
1. Creates one kinematic body per collider-owning node.
2. Scales collider shapes by the instance's uniform scale.
3. Each frame in `update_scene()`, updates body positions/rotations to match animated node transforms.

### SceneManager Collider API

**Enable/Disable:**

```cpp
// Create physics bodies for instance's colliders
size_t enableColliderSync(const std::string &instanceName,
                          Physics::PhysicsWorld *world,
                          uint32_t layer = 0,
                          uint64_t user_data = 0);

// Destroy physics bodies (called automatically on removeGLTFInstance)
bool disableColliderSync(const std::string &instanceName);

// Check if sync is enabled
bool isColliderSyncEnabled(const std::string &instanceName) const;
```

**Manual Update:**

```cpp
// Called automatically in update_scene(), but can be called manually
void syncColliders();
```

**Debug:**

```cpp
// Get all physics body IDs for an instance
std::vector<Physics::BodyId> getColliderSyncBodies(const std::string &instanceName) const;
```

### Collider Scaling

Collider shapes are defined at asset scale (scale 1.0). When an instance has a different scale, shapes are scaled uniformly:

```cpp
// In collider_asset.h
Physics::CompoundShape scale_compound_uniform(
    const Physics::CompoundShape &compound,
    float uniform_scale);

std::optional<Physics::CollisionShape> scale_collision_shape_uniform(
    const Physics::CollisionShape &shape,
    float uniform_scale);
```

Non-uniform instance scales use the X component and log a warning.

### Collider Parser API (Physics namespace)

Core collider extraction functions in `src/physics/gltf_collider_parser.h`:

**Marker type enum:**

```cpp
enum class GltfColliderMarkerType : uint8_t
{
    Box,
    Sphere,
    Capsule,
    Cylinder,
    TaperedCylinder,
    Unknown,
};
```

**Parse marker type from node name:**

```cpp
// Case-insensitive prefix match (e.g., "COL_BOX_hitbox" → Box)
GltfColliderMarkerType parse_collider_marker_type(std::string_view node_name);
```

**Build colliders from markers:**

```cpp
// Scan scene for COL_* nodes, group by owner node, store in out_compounds
void build_colliders_from_markers(
    std::unordered_map<std::string, CompoundShape>& out_compounds,
    const LoadedGLTF& scene,
    bool clear_existing = true);
```

**Build colliders from sidecar:**

```cpp
// Map sidecar colliders to destination scene by matching ancestor names
void build_colliders_from_sidecar(
    std::unordered_map<std::string, CompoundShape>& out_compounds,
    const LoadedGLTF& sidecar_scene,
    const std::unordered_set<std::string_view>& dst_node_names,
    bool clear_existing = true);
```

### LoadedGLTF Collider API

Convenience wrappers in `LoadedGLTF` that delegate to `Physics::build_colliders_*`:

**Build from markers (embedded in same file):**

```cpp
void build_colliders_from_markers(bool clear_existing = true);
```

**Build from sidecar (separate collider file):**

```cpp
void build_colliders_from_sidecar(const LoadedGLTF &sidecar, bool clear_existing = true);
```

**Collider data:**

```cpp
// Node name → compound shape
std::unordered_map<std::string, Physics::CompoundShape> collider_compounds;

// True if loaded from sidecar file
bool colliders_from_sidecar = false;

// Path to sidecar file (if used)
std::string collider_source_path;
```

### Usage Examples

**Basic Setup:**

```cpp
// Load model (colliders are extracted automatically)
auto model = assetManager->loadGLTF("models/robot.glb");

// Check what colliders were found
for (const auto& [nodeName, compound] : model->collider_compounds)
{
    fmt::println("Node '{}' has {} collider primitives",
                 nodeName, compound.children.size());
}

// Add instance and enable physics
scene->addGLTFInstance("robot1", model, transform);
scene->enableColliderSync("robot1", physicsWorld, LAYER_ROBOT);
```

**Animated Character:**

```cpp
// Load animated character
auto character = assetManager->loadGLTF("models/character.glb");

// Spawn multiple instances
for (int i = 0; i < 10; ++i)
{
    std::string name = fmt::format("enemy_{}", i);
    scene->addGLTFInstance(name, character, transforms[i]);
    scene->setGLTFInstanceAnimation(name, "Idle");
    scene->enableColliderSync(name, physicsWorld, LAYER_ENEMY, i);
}

// Colliders automatically follow animation each frame
```

**Query Collider Bodies:**

```cpp
// Get physics bodies for ray casting or collision queries
auto bodies = scene->getColliderSyncBodies("robot1");
for (Physics::BodyId bodyId : bodies)
{
    // Use bodyId for physics queries
    if (physicsWorld->ray_cast(ray, bodyId, hit))
    {
        // Handle hit
    }
}
```

### Stable Node Names

The loader generates unique, stable node names for consistent collider mapping:

- Named nodes keep their original name.
- Unnamed nodes get `__node_N` where N is the node index.
- Duplicate names get `#2`, `#3`, etc. suffixes.

This ensures `collider_compounds` keys are consistent across loads and can be used reliably for physics body lookup.

### Tips

- Keep collider geometry simple — use boxes and capsules for best physics performance.
- Place collider markers as direct children of the node they represent.
- For skinned meshes, attach colliders to bone nodes for automatic skeletal following.
- Use meaningful marker names (e.g., `COL_BOX_hitbox_head`) for debugging.
- Non-uniform scale on instances is not fully supported; use uniform scale when possible.
- Collider sync creates kinematic bodies — for dynamic physics, copy transforms to dynamic bodies manually.
- Call `disableColliderSync()` before removing instances to clean up physics bodies (done automatically by `removeGLTFInstance()`).
