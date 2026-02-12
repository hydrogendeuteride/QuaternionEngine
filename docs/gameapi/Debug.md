# Debug Drawing

Runtime visualization for primitives (lines, spheres, boxes, capsules, etc.) with depth testing and duration control.

## Enable/Disable

```cpp
api.set_debug_draw_enabled(true);
bool enabled = api.get_debug_draw_enabled();
```

## Visibility Controls

### Depth-Tested vs. Overlay

```cpp
// Show depth-tested primitives (occluded by geometry)
api.set_debug_show_depth_tested(true);
bool show = api.get_debug_show_depth_tested();

// Show overlay primitives (always on top)
api.set_debug_show_overlay(true);
bool show = api.get_debug_show_overlay();
```

You can toggle these independently to show/hide specific primitive groups.

### Layer Mask

```cpp
// Show all layers
api.set_debug_layer_mask(0xFFFFFFFF);

// Show only layer 0
api.set_debug_layer_mask(0b00000001);

// Show layers 0 and 2
api.set_debug_layer_mask(0b00000101);

uint32_t mask = api.get_debug_layer_mask();
```

Layers are user-defined (not currently exposed per-primitive, but reserved for future use).

### Tessellation Quality

```cpp
// Set segments for circles/spheres (higher = smoother)
api.set_debug_segments(32);   // Smooth
api.set_debug_segments(16);   // Default
api.set_debug_segments(8);    // Coarse

int segments = api.get_debug_segments();
```

## Clear Commands

```cpp
// Clear all debug draw commands
api.debug_draw_clear();
```

Useful for one-shot debugging or clearing persistent visualizations.

## Drawing Primitives

All primitives support:
- **Single and double precision** variants
- **Color** with alpha (RGBA, 0-1 range)
- **Duration** (0 = single frame, >0 = persist for N seconds)
- **Depth testing** (true = occluded by geometry, false = always on top)

### Line

```cpp
// Single precision
api.debug_draw_line(
    glm::vec3(0.0f, 0.0f, 0.0f),     // Start
    glm::vec3(5.0f, 0.0f, 0.0f),     // End
    glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),  // Red
    0.0f,                             // Duration (single frame)
    true                              // Depth tested
);

// Double precision
api.debug_draw_line(
    glm::dvec3(1000000.0, 0.0, 0.0),
    glm::dvec3(1000005.0, 0.0, 0.0),
    glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
    2.0f,    // Show for 2 seconds
    false    // Always on top
);
```

### Ray

```cpp
// Ray = origin + direction + length
api.debug_draw_ray(
    glm::vec3(0.0f, 1.0f, 0.0f),     // Origin
    glm::vec3(0.0f, -1.0f, 0.0f),    // Direction (normalized)
    10.0f,                            // Length
    glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),  // Green
    0.0f,
    true
);

// Double precision
api.debug_draw_ray(
    glm::dvec3(0.0, 100.0, 0.0),
    glm::dvec3(0.0, -1.0, 0.0),
    50.0,
    glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),
    0.0f,
    true
);
```

### AABB (Axis-Aligned Bounding Box)

```cpp
// Single precision
api.debug_draw_aabb(
    glm::vec3(0.0f, 1.0f, -5.0f),    // Center
    glm::vec3(1.0f, 1.0f, 1.0f),     // Half extents
    glm::vec4(0.0f, 1.0f, 1.0f, 0.5f),  // Cyan, semi-transparent
    0.0f,
    true
);

// Double precision
api.debug_draw_aabb(
    glm::dvec3(1000000.0, 0.0, 0.0),
    glm::vec3(10.0f, 10.0f, 10.0f),
    glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),  // Yellow
    0.0f,
    true
);
```

### Sphere

```cpp
// Single precision
api.debug_draw_sphere(
    glm::vec3(0.0f, 2.0f, -5.0f),    // Center
    1.5f,                             // Radius
    glm::vec4(1.0f, 0.0f, 1.0f, 0.3f),  // Magenta, transparent
    0.0f,
    true
);

// Double precision
api.debug_draw_sphere(
    glm::dvec3(0.0, 0.0, 0.0),
    5.0f,
    glm::vec4(1.0f, 1.0f, 1.0f, 0.2f),
    0.0f,
    true
);
```

### Capsule

```cpp
// Capsule = line segment + radius
api.debug_draw_capsule(
    glm::vec3(0.0f, 0.0f, -5.0f),    // P0
    glm::vec3(0.0f, 3.0f, -5.0f),    // P1
    0.5f,                             // Radius
    glm::vec4(0.0f, 0.0f, 1.0f, 1.0f),  // Blue
    0.0f,
    true
);

// Double precision
api.debug_draw_capsule(
    glm::dvec3(1000000.0, 0.0, 0.0),
    glm::dvec3(1000000.0, 10.0, 0.0),
    2.0f,
    glm::vec4(1.0f, 0.5f, 0.0f, 1.0f),
    0.0f,
    true
);
```

### Circle

```cpp
// Circle = center + normal + radius
api.debug_draw_circle(
    glm::vec3(0.0f, 0.0f, -5.0f),    // Center
    glm::vec3(0.0f, 1.0f, 0.0f),     // Normal (up)
    2.0f,                             // Radius
    glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),  // Yellow
    0.0f,
    true
);

// Double precision
api.debug_draw_circle(
    glm::dvec3(0.0, 0.0, 0.0),
    glm::dvec3(0.0, 0.0, 1.0),  // Normal (forward)
    10.0f,
    glm::vec4(0.0f, 1.0f, 1.0f, 1.0f),
    0.0f,
    true
);
```

### Cone

```cpp
// Cone = apex + direction + length + angle
api.debug_draw_cone(
    glm::vec3(0.0f, 3.0f, -5.0f),    // Apex
    glm::vec3(0.0f, -1.0f, 0.0f),    // Direction (down)
    2.0f,                             // Length
    30.0f,                            // Angle (degrees)
    glm::vec4(1.0f, 0.5f, 0.0f, 0.5f),  // Orange, semi-transparent
    0.0f,
    true
);

// Double precision
api.debug_draw_cone(
    glm::dvec3(0.0, 100.0, 0.0),
    glm::dvec3(0.0, -1.0, 0.0),
    50.0f,
    45.0f,
    glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
    0.0f,
    true
);
```

## Complete Examples

### Visualize Player Bounds

```cpp
GameAPI::Engine api(&engine);
api.set_debug_draw_enabled(true);

// Draw AABB for player (every frame)
api.debug_draw_aabb(
    player_position,
    glm::vec3(0.5f, 1.0f, 0.5f),  // Half extents
    glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),  // Green
    0.0f,    // Single frame (re-draw each frame)
    true
);
```

### Raycast Visualization

```cpp
// Visualize raycast (red if miss, green if hit)
glm::vec4 color = hit ? glm::vec4(0.0f, 1.0f, 0.0f, 1.0f) : glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);

api.debug_draw_ray(
    ray_origin,
    ray_direction,
    ray_length,
    color,
    0.0f,
    false  // Always on top
);

// Draw hit point as sphere
if (hit) {
    api.debug_draw_sphere(
        hit_position,
        0.2f,
        glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),  // Yellow
        2.0f,   // Show for 2 seconds
        false
    );
}
```

### Trigger Volume

```cpp
// Draw trigger zone (transparent sphere)
api.debug_draw_sphere(
    trigger_center,
    trigger_radius,
    glm::vec4(1.0f, 1.0f, 0.0f, 0.3f),  // Yellow, 30% alpha
    0.0f,
    true
);
```

### Spot Light Cone

```cpp
// Visualize spot light direction and cone
api.debug_draw_cone(
    light_position,
    light_direction,
    light_radius,
    light_outer_angle_deg,
    glm::vec4(1.0f, 0.9f, 0.7f, 0.4f),  // Warm, transparent
    0.0f,
    true
);
```

### Pathfinding

```cpp
// Draw waypoint path (persistent for 5 seconds)
for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    api.debug_draw_line(
        waypoints[i],
        waypoints[i + 1],
        glm::vec4(0.0f, 1.0f, 1.0f, 1.0f),  // Cyan
        5.0f,   // Show for 5 seconds
        false   // Always on top
    );
}

// Draw waypoint markers
for (const glm::vec3& wp : waypoints) {
    api.debug_draw_sphere(
        wp,
        0.3f,
        glm::vec4(1.0f, 0.0f, 1.0f, 1.0f),  // Magenta
        5.0f,
        false
    );
}
```

### Physics Debug

```cpp
// Collision shapes
for (const Collider& collider : colliders) {
    switch (collider.type) {
        case ColliderType::Sphere:
            api.debug_draw_sphere(collider.center, collider.radius,
                                   glm::vec4(0.0f, 1.0f, 0.0f, 0.3f), 0.0f, true);
            break;
        case ColliderType::Box:
            api.debug_draw_aabb(collider.center, collider.half_extents,
                                 glm::vec4(0.0f, 0.0f, 1.0f, 0.3f), 0.0f, true);
            break;
        case ColliderType::Capsule:
            api.debug_draw_capsule(collider.p0, collider.p1, collider.radius,
                                    glm::vec4(1.0f, 0.0f, 1.0f, 0.3f), 0.0f, true);
            break;
    }
}
```

### One-Shot Events

```cpp
// On explosion: draw expanding sphere for 1 second
if (explosion_triggered) {
    api.debug_draw_sphere(
        explosion_position,
        explosion_radius,
        glm::vec4(1.0f, 0.5f, 0.0f, 0.5f),  // Orange flash
        1.0f,   // 1 second
        false   // Always visible
    );
}
```

## Performance Considerations

- Debug primitives are batched and rendered in a single draw call per type
- Depth-tested and overlay primitives are rendered separately
- Persistent primitives (duration > 0) accumulate until expired
- Use `debug_draw_clear()` periodically to prevent buildup
- Adjust tessellation segments to balance quality and performance

## See Also

- [Picking](Picking.md) — Object selection for interactive debugging
- [ImGui System](../ImGuiSystem.md) — UI-based debugging tools
