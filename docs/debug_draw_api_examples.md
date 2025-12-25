# Debug Draw API Examples

The debug drawing system has been integrated into the GameAPI for easy visualization and UI development.

## Basic Setup

```cpp
#include "core/game_api.h"

GameAPI::Engine* api = /* your GameAPI::Engine instance */;

// Enable debug drawing
api->set_debug_draw_enabled(true);
```

## Drawing Primitives

### Lines

```cpp
// Draw a simple line (single frame)
api->debug_draw_line(
    glm::vec3(0, 0, 0),     // start point
    glm::vec3(10, 0, 0),    // end point
    glm::vec4(1, 0, 0, 1),  // red color
    0.0f,                    // duration (0 = one frame)
    true                     // depth tested
);

// Draw a persistent line (3 seconds)
api->debug_draw_line(
    glm::vec3(0, 0, 0),
    glm::vec3(0, 10, 0),
    glm::vec4(0, 1, 0, 1),  // green color
    3.0f,                    // 3 seconds
    false                    // always on top
);
```

### Rays

```cpp
// Visualize a raycast
glm::vec3 origin(0, 1, 0);
glm::vec3 direction = glm::normalize(glm::vec3(1, 0, 1));
float length = 10.0f;

api->debug_draw_ray(
    origin,
    direction,
    length,
    glm::vec4(1, 1, 0, 1),  // yellow
    1.0f                     // 1 second
);
```

### Bounding Boxes (AABB)

```cpp
// Draw object bounds
glm::vec3 object_center(5, 2, 5);
glm::vec3 half_extents(1, 1, 1);  // 2x2x2 box

api->debug_draw_aabb(
    object_center,
    half_extents,
    glm::vec4(0, 1, 1, 1),  // cyan
    0.0f,                    // one frame
    true
);
```

### Spheres

```cpp
// Draw sphere colliders
api->debug_draw_sphere(
    glm::vec3(0, 5, 0),     // center
    2.5f,                    // radius
    glm::vec4(1, 0, 1, 1),  // magenta
    2.0f,                    // 2 seconds
    true
);
```

### Capsules

```cpp
// Draw character controller capsule
api->debug_draw_capsule(
    glm::vec3(0, 0.5, 0),   // bottom point
    glm::vec3(0, 1.5, 0),   // top point
    0.3f,                    // radius
    glm::vec4(0, 0, 1, 1),  // blue
    0.0f,
    true
);
```

### Circles

```cpp
// Draw area of effect
glm::vec3 center(0, 0.1, 0);
glm::vec3 normal(0, 1, 0);  // facing up
float radius = 5.0f;

api->debug_draw_circle(
    center,
    normal,
    radius,
    glm::vec4(1, 0.5, 0, 0.5),  // semi-transparent orange
    1.0f
);
```

### Cones

```cpp
// Draw spotlight cone
glm::vec3 apex(0, 10, 0);
glm::vec3 direction(0, -1, 0);  // pointing down
float length = 5.0f;
float angle_degrees = 30.0f;

api->debug_draw_cone(
    apex,
    direction,
    length,
    angle_degrees,
    glm::vec4(1, 1, 1, 0.3),  // white semi-transparent
    0.0f
);
```

## Settings Control

```cpp
// Toggle visibility
api->set_debug_draw_enabled(true);
api->set_debug_show_depth_tested(true);   // show depth-tested primitives
api->set_debug_show_overlay(true);        // show always-on-top primitives

// Adjust quality (segments for circles/spheres)
api->set_debug_segments(32);  // default, higher = smoother

// Clear all debug draw commands
api->debug_draw_clear();
```

## Layer Masking

```cpp
// The debug system uses layers internally, but all GameAPI calls use the "Misc" layer
// Layer mask can be controlled if you need finer control:
uint32_t mask = api->get_debug_layer_mask();
// Modify mask to show/hide different debug categories
api->set_debug_layer_mask(mask);
```

## Simple UI Example

Here's a simple debugging UI that shows object bounds:

```cpp
void update_debug_ui(GameAPI::Engine* api)
{
    if (!api->get_debug_draw_enabled())
        return;

    // Draw coordinate axes at origin
    api->debug_draw_line(glm::vec3(0), glm::vec3(5, 0, 0), glm::vec4(1, 0, 0, 1), 0.0f, false);  // X - red
    api->debug_draw_line(glm::vec3(0), glm::vec3(0, 5, 0), glm::vec4(0, 1, 0, 1), 0.0f, false);  // Y - green
    api->debug_draw_line(glm::vec3(0), glm::vec3(0, 0, 5), glm::vec4(0, 0, 1, 1), 0.0f, false);  // Z - blue

    // Draw grid on ground plane
    float grid_size = 50.0f;
    float grid_step = 1.0f;
    glm::vec4 grid_color(0.3f, 0.3f, 0.3f, 0.5f);

    for (float x = -grid_size; x <= grid_size; x += grid_step)
    {
        api->debug_draw_line(
            glm::vec3(x, 0, -grid_size),
            glm::vec3(x, 0, grid_size),
            grid_color
        );
    }

    for (float z = -grid_size; z <= grid_size; z += grid_step)
    {
        api->debug_draw_line(
            glm::vec3(-grid_size, 0, z),
            glm::vec3(grid_size, 0, z),
            grid_color
        );
    }

    // Highlight picked object
    auto pick = api->get_last_pick();
    if (pick.valid)
    {
        api->debug_draw_sphere(
            pick.worldPosition,
            0.5f,
            glm::vec4(1, 1, 0, 1),  // yellow highlight
            0.1f,
            false  // always on top
        );
    }
}
```

## Performance Notes

- **Duration 0.0f**: Commands are cleared every frame (good for dynamic visualization)
- **Duration > 0.0f**: Commands persist for the specified time (good for trails, persistent markers)
- **depth_tested = true**: Primitives are occluded by geometry (good for in-world debugging)
- **depth_tested = false**: Primitives always visible (good for UI overlays)
- Call `debug_draw_clear()` to immediately remove all debug primitives

## Integration with Game Logic

```cpp
// Example: Visualize AI navigation
void debug_ai_path(GameAPI::Engine* api, const std::vector<glm::vec3>& waypoints)
{
    if (waypoints.empty()) return;

    // Draw path
    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        api->debug_draw_line(
            waypoints[i],
            waypoints[i + 1],
            glm::vec4(1, 0.5f, 0, 1),
            2.0f  // persist for 2 seconds
        );
    }

    // Draw waypoints
    for (const auto& wp : waypoints)
    {
        api->debug_draw_sphere(wp, 0.2f, glm::vec4(0, 1, 0, 1), 2.0f);
    }
}

// Example: Visualize physics simulation
void debug_physics(GameAPI::Engine* api)
{
    // Draw velocity vectors
    glm::vec3 position(10, 1, 0);
    glm::vec3 velocity(2, 5, 1);

    api->debug_draw_ray(
        position,
        glm::normalize(velocity),
        glm::length(velocity),
        glm::vec4(1, 0, 0, 1)
    );

    // Draw collision sphere
    api->debug_draw_sphere(
        position,
        1.0f,
        glm::vec4(0, 1, 1, 0.3f)  // semi-transparent cyan
    );
}
```
