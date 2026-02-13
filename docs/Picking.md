## Picking System: Object Selection and Interaction

Unified picking system that handles single-click selection, hover detection, and drag-box multi-select. Supports both CPU ray-casting (via BVH) and GPU ID-buffer picking.

### Components

- `PickingSystem` (src/core/picking/picking_system.h/.cpp)
  - Main entry point for all picking operations.
  - Consumes engine `InputSystem` mouse events (click, drag, release).
  - Maintains per-frame hover picks and last click selection.
  - Integrates with RenderGraph for async ID-buffer readback.

- `SceneManager` picking helpers (src/scene/vk_scene_picking.cpp)
  - CPU ray-casting against `RenderObject` bounds.
  - BVH-accelerated mesh picking for precise triangle-level hits.
  - Rectangle selection in NDC space.

### PickInfo Structure

Result of any pick operation:

```cpp
struct PickInfo
{
    MeshAsset *mesh = nullptr;           // Source mesh asset
    LoadedGLTF *scene = nullptr;         // Source glTF scene
    Node *node = nullptr;                // glTF node that owns this surface
    RenderObject::OwnerType ownerType;   // GLTFInstance, MeshInstance
    std::string ownerName;               // Logical name (e.g., "player", "cube01")
    std::string nodeName;                // Selected glTF node name (if glTF pick)
    std::string nodeParentName;          // Direct parent node name
    std::vector<std::string> nodeChildren; // Direct child node names
    std::vector<std::string> nodePath;   // Root-to-node hierarchy path
    WorldVec3 worldPos;                  // Hit position in world space (double-precision)
    glm::mat4 worldTransform;            // Object's world transform
    uint32_t indexCount;                 // Index count of picked surface
    uint32_t firstIndex;                 // First index of picked surface
    uint32_t surfaceIndex;               // Surface index within mesh
    bool valid = false;                  // True if pick hit something
};
```

### Picking Modes

#### 1. CPU Ray Picking (Default)

Uses camera ray against object bounds with BVH acceleration:

- **Sphere bounds**: Quick bounding sphere test.
- **Box bounds**: Ray-box intersection in local space.
- **Capsule bounds**: Combined cylinder + sphere caps intersection.
- **Mesh bounds**: Full BVH traversal for triangle-level precision.

Advantages:
- Immediate results (same frame).
- No GPU readback latency.
- Precise triangle-level hits with mesh BVH.

#### 2. ID-Buffer Picking

Renders object IDs to a GPU buffer and reads back single pixel:

- Each `RenderObject` has a unique `objectID` assigned during draw.
- Geometry pass writes IDs to an R32_UINT attachment.
- `PickReadback` render pass copies single pixel to CPU-readable buffer.
- Result resolved on next frame after GPU fence wait.

Advantages:
- Pixel-perfect accuracy (no false positives from bounds).
- Handles complex overlapping geometry.
- Works with any object shape.

Enable via:
```cpp
picking_system.set_use_id_buffer_picking(true);
```

### PickingSystem API

**Initialization:**

```cpp
void init(EngineContext *context);
void cleanup();
```

**Frame Lifecycle:**

```cpp
void begin_frame();  // Resolve pending async picks after fence wait
void process_input(const InputSystem &input, bool ui_want_capture_mouse);
void update_hover(bool ui_want_capture_mouse); // Update hover pick each frame
```

**Results Access:**

```cpp
const PickInfo& last_pick() const;           // Last click selection
const PickInfo& hover_pick() const;          // Current hover (under cursor)
const std::vector<PickInfo>& drag_selection() const;  // Multi-select results
uint32_t last_pick_object_id() const;        // Raw object ID of last pick
bool move_last_pick_to_parent();
bool move_last_pick_to_child(size_t child_index = 0);
bool move_last_pick_to_child(const std::string &child_name);
```

**Configuration:**

```cpp
PickingSystem::Settings& settings();
const PickingSystem::Settings& settings() const;

bool use_id_buffer_picking() const;
void set_use_id_buffer_picking(bool enabled);

bool debug_draw_bvh() const;
void set_debug_draw_bvh(bool enabled);
```

**Utilities:**

```cpp
// Clear picks for removed objects (call when deleting instances)
void clear_owner_picks(RenderObject::OwnerType owner_type, const std::string &owner_name);

// Mutable access for engine integration
PickInfo* mutable_last_pick();
PickInfo* mutable_hover_pick();
```

**RenderGraph Integration:**

```cpp
// Called during RenderGraph build when ID-buffer picking is enabled
void register_id_buffer_readback(RenderGraph &graph,
                                 RGImageHandle id_buffer,
                                 VkExtent2D draw_extent,
                                 VkExtent2D swapchain_extent);
```

### SceneManager Picking API

Lower-level picking functions for custom use:

```cpp
// Single-object ray pick
bool pick(const glm::vec2 &mousePosPixels, RenderObject &outObject, WorldVec3 &outWorldPos);

// Resolve ID from ID-buffer back to RenderObject
bool resolveObjectID(uint32_t id, RenderObject &outObject) const;

// Rectangle selection (drag-box)
void selectRect(const glm::vec2 &p0, const glm::vec2 &p1,
                std::vector<RenderObject> &outObjects) const;
```

### Usage Examples

**Basic Click Selection:**

```cpp
void Game::handle_selection(PickingSystem& picking)
{
    const PickingSystem::PickInfo& pick = picking.last_pick();

    if (pick.valid)
    {
        fmt::println("Selected: {} at ({}, {}, {})",
                     pick.ownerName,
                     pick.worldPos.x, pick.worldPos.y, pick.worldPos.z);

        // Handle different owner types
        switch (pick.ownerType)
        {
            case RenderObject::OwnerType::GLTFInstance:
                select_actor(pick.ownerName);
                break;
            case RenderObject::OwnerType::MeshInstance:
                select_prop(pick.ownerName);
                break;
            default:
                break;
        }
    }
}
```

**Hierarchical glTF Selection:**

```cpp
const auto& pick = picking.last_pick();
if (pick.valid && pick.ownerType == RenderObject::OwnerType::GLTFInstance)
{
    if (!pick.nodeParentName.empty())
    {
        picking.move_last_pick_to_parent();
    }

    if (!pick.nodeChildren.empty())
    {
        // First child
        picking.move_last_pick_to_child();

        // Or explicit child by name
        picking.move_last_pick_to_child(pick.nodeChildren.front());
    }
}
```

**Hover Tooltips:**

```cpp
void Game::update_ui(PickingSystem& picking)
{
    const PickingSystem::PickInfo& hover = picking.hover_pick();

    if (hover.valid)
    {
        show_tooltip(hover.ownerName);
    }
}
```

**Multi-Select with Drag Box:**

```cpp
void Game::process_drag_selection(PickingSystem& picking)
{
    const auto& selection = picking.drag_selection();

    for (const PickingSystem::PickInfo& pick : selection)
    {
        if (pick.valid)
        {
            add_to_selection(pick.ownerName);
        }
    }
}
```

**Custom Ray Pick:**

```cpp
void Game::custom_pick(SceneManager& scene, const glm::vec2& screen_pos)
{
    RenderObject hit_object{};
    WorldVec3 hit_pos{};

    if (scene.pick(screen_pos, hit_object, hit_pos))
    {
        // hit_object contains the picked render object
        // hit_pos is the precise world-space hit position
        spawn_effect_at(hit_pos);
    }
}
```

### Bounds Types

Objects can use different bounds types for picking:

```cpp
enum class BoundsType : uint8_t
{
    None = 0,    // Not pickable
    Box = 1,     // AABB (default for most objects)
    Sphere = 2,  // Bounding sphere
    Capsule = 3, // Cylinder + hemisphere caps
    Mesh = 4,    // Full BVH mesh intersection
};
```

Set bounds type when adding instances:
```cpp
scene->addMeshInstance("capsule_enemy", mesh, transform, BoundsType::Capsule);
```

### Mesh BVH Picking

For precise triangle-level picking on complex meshes:

1. Mesh assets automatically build a BVH during loading.
2. `BoundsType::Mesh` triggers BVH traversal instead of simple bounds test.
3. Returns exact triangle hit position, not just bounding box intersection.
4. `PickInfo::firstIndex` and `indexCount` are refined to the exact triangle.

Debug BVH visualization:
```cpp
picking.set_debug_draw_bvh(true);  // Shows BVH nodes in debug overlay
```

### Coordinate Space Handling

The picking system handles multiple coordinate transformations:

1. **Window pixels** (`InputEvent::mouse_pos`, top-left origin)
2. **Swapchain pixels** (scaled for HiDPI displays)
3. **Logical render pixels** (internal render resolution)
4. **NDC** (normalized device coordinates, -1 to 1)
5. **World space** (double-precision `WorldVec3`)

The `window_to_swapchain_pixels()` helper handles HiDPI scaling. Letterboxing is accounted for when render resolution differs from swapchain size.

### Frame Flow

1. `begin_frame()` — Resolve pending async ID-buffer picks from previous frame.
2. `process_input()` — Handle mouse events (click start/end, motion).
3. `update_hover()` — CPU ray-cast for current hover under cursor.
4. RenderGraph build — If ID-buffer picking enabled, register readback pass.
5. Next frame — Async pick result becomes available.

### Integration with ImGui

The picking system respects ImGui's input capture:

```cpp
// In main loop (after InputSystem::pump_events())
bool ui_capture = imgui_system.wantCaptureMouse();
picking_system.process_input(input_system, ui_capture);
picking_system.update_hover(ui_capture);
```

When `ui_want_capture_mouse` is true:
- Click/drag interactions are ignored (no picks started).
- Mouse motion still updates cursor position for future picks.
- Hover picking is cleared while captured (configurable via `PickingSystem::Settings`).

### Tips

- Use CPU ray picking (`set_use_id_buffer_picking(false)`) for immediate feedback.
- Use ID-buffer picking for pixel-perfect selection in dense scenes.
- Call `clear_owner_picks()` when removing instances to avoid stale picks.
- For mesh BVH to work, ensure the mesh was loaded with BVH generation enabled.
- Select buttons are controlled by `settings().select_button_mask` (default: Left). Bits: 1<<0 Left, 1<<1 Middle, 1<<2 Right, 1<<3 X1, 1<<4 X2.
- Click vs drag is controlled by `settings().click_threshold_px` (default: 3 px).
- In the built-in `Picking & Gizmo` tab, glTF picks expose parent/child node navigation using these hierarchy fields.
