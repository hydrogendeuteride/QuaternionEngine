# Picking System

Object selection, hover detection, and hierarchy navigation for glTF and mesh instances.

## Picking Modes

The engine supports two picking modes:

### CPU Raycast Picking (Default)

- BVH-accelerated ray-triangle intersection
- Immediate results (same frame)
- Lower VRAM usage (no ID buffer)
- Good for sparse scenes

### ID-Buffer Picking

- Renders object IDs to off-screen buffer
- GPU-driven, pixel-perfect
- 1-frame latency (async readback)
- Better for dense scenes

```cpp
// Enable ID-buffer picking
api.set_use_id_buffer_picking(true);
bool useIdBuffer = api.get_use_id_buffer_picking();

// Disable (use CPU raycast)
api.set_use_id_buffer_picking(false);
```

## Enable/Disable Picking

```cpp
api.set_picking_enabled(true);
bool enabled = api.get_picking_enabled();
```

## Picking Settings

### Select Button Mask

Control which mouse buttons trigger picking.

```cpp
// Left button only (default)
api.set_picking_select_button_mask(1 << 0);

// Right button only
api.set_picking_select_button_mask(1 << 2);

// Left or right
api.set_picking_select_button_mask((1 << 0) | (1 << 2));

// All buttons
api.set_picking_select_button_mask(0b11111);

uint32_t mask = api.get_picking_select_button_mask();
```

**Button bits:**
- Bit 0: Left
- Bit 1: Middle
- Bit 2: Right
- Bit 3: X1
- Bit 4: X2

### Click Threshold

Prevent drag from triggering pick.

```cpp
// Strict (2 pixels)
api.set_picking_click_threshold_px(2.0f);

// Default (8 pixels)
api.set_picking_click_threshold_px(8.0f);

// Generous (16 pixels)
api.set_picking_click_threshold_px(16.0f);

float threshold = api.get_picking_click_threshold_px();
```

Mouse movement within this threshold during mouse-down-to-up is considered a click (not a drag).

## Pick Results

### Data Structures

```cpp
// Single precision
struct PickResult
{
    bool valid{false};
    std::string ownerName;              // Instance name
    std::string nodeName;               // glTF node name (if applicable)
    std::string nodeParentName;         // Direct parent node
    std::vector<std::string> nodeChildren;  // Direct child nodes
    std::vector<std::string> nodePath;  // Full path from root to node
    glm::vec3 worldPosition{0.0f};      // Hit position
};

// Double precision
struct PickResultD
{
    bool valid{false};
    std::string ownerName;
    std::string nodeName;
    std::string nodeParentName;
    std::vector<std::string> nodeChildren;
    std::vector<std::string> nodePath;
    glm::dvec3 worldPosition{0.0};
};
```

### Get Last Click Selection

```cpp
// Single precision
GameAPI::PickResult pick = api.get_last_pick();
if (pick.valid) {
    fmt::println("Selected: {} (node: {})", pick.ownerName, pick.nodeName);
    fmt::println("Hit position: {}, {}, {}",
                 pick.worldPosition.x, pick.worldPosition.y, pick.worldPosition.z);
}

// Double precision
GameAPI::PickResultD pickD = api.get_last_pick_d();
if (pickD.valid) {
    fmt::println("Hit position: {}, {}, {}",
                 pickD.worldPosition.x, pickD.worldPosition.y, pickD.worldPosition.z);
}
```

## Hierarchy Navigation

For glTF instances, you can navigate the node hierarchy:

### Select Parent

```cpp
// Move selection to parent node
if (api.select_parent_of_last_pick()) {
    GameAPI::PickResult pick = api.get_last_pick();
    fmt::println("Selected parent: {}", pick.nodeName);
}
```

Returns `false` if last pick has no parent.

### Select Child

```cpp
// Select first child
if (api.select_child_of_last_pick()) {
    GameAPI::PickResult pick = api.get_last_pick();
    fmt::println("Selected child: {}", pick.nodeName);
}

// Select child by index
if (api.select_child_of_last_pick(2)) {  // Third child
    // ...
}

// Select child by name
if (api.select_child_of_last_pick("LeftArm")) {
    // ...
}
```

Returns `false` if no child at index/name, or last pick has no children.

## Complete Examples

### Basic Selection

```cpp
GameAPI::Engine api(&engine);
api.set_picking_enabled(true);

// In game loop
GameAPI::PickResult pick = api.get_last_pick();
if (pick.valid) {
    fmt::println("Selected: {}", pick.ownerName);

    // Highlight selected object
    highlight_object(pick.ownerName);

    // Show details in UI
    show_object_properties(pick);
}
```

### Interactive Object Spawning

```cpp
// Click to place object at pick position
GameAPI::PickResult pick = api.get_last_pick();
if (pick.valid && input.key_pressed(Key::E)) {
    GameAPI::Transform t;
    t.position = pick.worldPosition + glm::vec3(0.0f, 1.0f, 0.0f);  // Offset up
    api.add_primitive_instance("spawned_object", GameAPI::PrimitiveType::Cube, t);
}
```

### Hierarchy Exploration

```cpp
GameAPI::PickResult pick = api.get_last_pick();
if (pick.valid) {
    fmt::println("Selected: {} / {}", pick.ownerName, pick.nodeName);
    fmt::println("Parent: {}", pick.nodeParentName);
    fmt::println("Children ({}):", pick.nodeChildren.size());
    for (const std::string& child : pick.nodeChildren) {
        fmt::println("  - {}", child);
    }
    fmt::println("Path:");
    for (const std::string& node : pick.nodePath) {
        fmt::println("  {}", node);
    }
}

// Navigate with keyboard
if (input.key_pressed(Key::Up)) {
    api.select_parent_of_last_pick();
}
if (input.key_pressed(Key::Down)) {
    api.select_child_of_last_pick(0);  // First child
}
```

### Camera Target from Pick

```cpp
// Orbit selected object
if (api.set_camera_target_from_last_pick()) {
    api.set_camera_mode(GameAPI::CameraMode::Orbit);

    GameAPI::OrbitCameraSettings orbit;
    orbit.distance = 10.0;
    api.set_orbit_camera_settings(orbit);
}
```

### Selection Ring

```cpp
// Draw selection ring around picked object
GameAPI::PickResult pick = api.get_last_pick();
if (pick.valid) {
    api.debug_draw_circle(
        pick.worldPosition,
        glm::vec3(0.0f, 1.0f, 0.0f),  // Up
        2.0f,  // Radius
        glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),  // Yellow
        0.0f,
        false  // Always on top
    );
}
```

### Context Menu

```cpp
GameAPI::PickResult pick = api.get_last_pick();
if (pick.valid && input.mouse_pressed(MouseButton::Right)) {
    // Show context menu at cursor
    show_context_menu(pick.ownerName, pick.nodeName);
}
```

### Deletion

```cpp
GameAPI::PickResult pick = api.get_last_pick();
if (pick.valid && input.key_pressed(Key::Delete)) {
    // Try removing as glTF instance
    if (!api.remove_gltf_instance(pick.ownerName)) {
        // Try removing as mesh instance
        api.remove_mesh_instance(pick.ownerName);
    }
}
```

### Node Transform Editing

```cpp
GameAPI::PickResult pick = api.get_last_pick();
if (pick.valid && !pick.nodeName.empty()) {
    // Apply rotation to selected node
    glm::mat4 offset = glm::rotate(glm::mat4(1.0f),
                                   glm::radians(delta_rotation),
                                   glm::vec3(0.0f, 1.0f, 0.0f));
    api.set_instance_node_offset(pick.ownerName, pick.nodeName, offset);
}
```

### Multi-Object Selection

The picking system doesn't directly support multi-select, but you can build it:

```cpp
std::set<std::string> selectedObjects;

// Add to selection with Ctrl+Click
GameAPI::PickResult pick = api.get_last_pick();
if (pick.valid) {
    if (input.modifiers().ctrl) {
        selectedObjects.insert(pick.ownerName);
    } else {
        selectedObjects.clear();
        selectedObjects.insert(pick.ownerName);
    }
}

// Visualize all selected objects
for (const std::string& name : selectedObjects) {
    // ... draw selection outline
}
```

## Troubleshooting

### Picking Not Working

1. Check `api.get_picking_enabled()` returns `true`
2. Verify mouse button is in select button mask
3. Ensure you're not moving mouse during click (check threshold)
4. For ID-buffer mode, check 1-frame latency

### Wrong Object Selected

- **CPU mode**: Check BVH is up to date (happens automatically)
- **ID-buffer mode**: Ensure sufficient precision (should be pixel-perfect)
- Check for overlapping geometry (closest object wins)

### Performance Issues

- **CPU mode**: Many triangles or deep BVH can be slow
  - Consider switching to ID-buffer mode
- **ID-buffer mode**: Adds GPU overhead and VRAM for ID buffer
  - Consider switching to CPU mode

## See Also

- [Scene Management](Scene.md) — Object lifecycle
- [Camera](Camera.md) — Camera target from pick
- [Debug Drawing](Debug.md) — Visualize selections
- [Picking System](../Picking.md) — Low-level implementation details
