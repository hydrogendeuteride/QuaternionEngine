# Debug Draw

> Immediate-mode wireframe debug visualization system with depth testing, layer filtering, and timed commands.

## Purpose

Provides a CPU-side command buffer for submitting wireframe primitives (lines, boxes,
spheres, capsules, circles, cones, cylinders, planes) that are rendered as line lists
each frame. Supports two depth modes (depth-tested / always-on-top), per-layer
visibility masking, and optional time-to-live so commands can persist across multiple
frames.

A companion helper (`engine_debug_draw`) emits per-frame engine-level debug visuals
(picking bounds, light shapes, particle emitters, volumetric volumes) so the main
render loop stays lean.

## Directory Layout

```
debug_draw/
├── debug_draw.h           — DebugDrawSystem class, enums, vertex format
├── debug_draw.cpp         — primitive tessellation and vertex generation
├── engine_debug_draw.h    — free function for engine-layer debug visuals
└── engine_debug_draw.cpp  — implementation (picking, lights, particles, volumetrics)
```

## Key Types

| Type | Role |
|------|------|
| `DebugDrawSystem` | Central command buffer — stores draw commands, builds per-frame line vertices |
| `DebugDrawSystem::Settings` | Runtime toggles: enabled, depth/overlay visibility, layer mask, segment count |
| `DebugDrawSystem::LineVertexLists` | Output of `build_line_vertices()` — interleaved depth + overlay vertex arrays |
| `DebugDrawVertex` | 32-byte GPU vertex: `vec3 position`, `float _pad`, `vec4 color` |
| `DebugDepth` | Enum — `DepthTested` (occluded by geometry) or `AlwaysOnTop` (overlay) |
| `DebugDrawLayer` | Bitmask enum — `Physics`, `Picking`, `Lights`, `Particles`, `Volumetrics`, `Misc` |

## Lifecycle

```
DebugDrawSystem dd;
dd.settings().enabled = true;

begin_frame(dt_seconds)
  └─ prunes expired timed commands, removes one-frame commands from previous frame

add_line / add_ray / add_aabb / add_sphere / add_capsule /
add_circle / add_cone / add_obb / add_cylinder /
add_tapered_cylinder / add_plane_patch
  └─ appends typed command with color, depth mode, layer, optional TTL

debug_draw_engine_layers(dd, origin, ctx, scene, picking, rpm)
  └─ emits picking bounds, light shapes, particle emitters, volumetric volumes

build_line_vertices(origin_world)
  └─ tessellates all commands into DebugDrawVertex line-list
  └─ splits into depth-tested + overlay buckets
  └─ converts WorldVec3 → render-local coordinates via floating origin

clear()
  └─ immediately removes all pending commands
```

## Supported Primitives

| Method | Shape | Notes |
|--------|-------|-------|
| `add_line` | Line segment | Two endpoints |
| `add_ray` | Directed line | Origin + direction + length |
| `add_aabb` | Axis-aligned box | Center + half extents (12 edge lines) |
| `add_sphere` | Wireframe sphere | 3 great circles (XY, XZ, YZ planes) |
| `add_capsule` | Wireframe capsule | Two end rings + side lines + hemisphere arcs |
| `add_circle` | Circle | Center + normal + radius, configurable segments |
| `add_cone` | Cone | Apex + direction + angle, base circle + spokes |
| `add_obb` | Oriented bounding box | 8 world-space corners (12 edge lines) |
| `add_cylinder` | Cylinder | Center + axis + radius + half height (2 circles + 4 side lines) |
| `add_tapered_cylinder` | Tapered cylinder / cone | Different top/bottom radii |
| `add_plane_patch` | Square patch | Point + normal + half size |

## Usage

### One-frame primitives

```cpp
// Draw a sphere for one frame (default seconds = 0)
dd->add_sphere(center_world, 2.0f,
               glm::vec4(1, 0, 0, 1),
               0.0f,
               DebugDepth::AlwaysOnTop,
               DebugDrawLayer::Physics);
```

### Timed primitives

```cpp
// Draw a line that persists for 3 seconds
dd->add_line(a_world, b_world,
             glm::vec4(0, 1, 0, 1),
             3.0f,
             DebugDepth::DepthTested,
             DebugDrawLayer::Misc);
```

### Building vertex output

```cpp
auto lists = dd->build_line_vertices(floating_origin);
// lists.vertices         — all line vertices (depth first, then overlay)
// lists.depth_vertex_count   — number of depth-tested vertices
// lists.overlay_vertex_count — number of overlay vertices
// Upload to GPU buffer and draw as VK_PRIMITIVE_TOPOLOGY_LINE_LIST
```

## Engine Debug Layers

`debug_draw_engine_layers()` automatically visualizes engine subsystems:

| Layer | Visualization |
|-------|--------------|
| `Picking` | BVH root bounds (cyan) + picked surface bounds (yellow) as OBBs |
| `Lights` | Point light radius spheres + spot light cones + spot origin spheres |
| `Particles` | Emitter position sphere + spawn radius circle + emission cone |
| `Volumetrics` | Volume AABBs (follows camera XZ if enabled) + wind direction rays |

## Integration

`DebugDrawSystem` is owned by `VulkanEngine` and exposed via `EngineContext`.
The `GameAPI` wraps it with convenience methods (`debug_draw_line`, `debug_draw_sphere`, etc.)
that accept `glm::vec3` local coordinates and convert to `WorldVec3` internally.

The render pass draws the output as `VK_PRIMITIVE_TOPOLOGY_LINE_LIST` in two sub-draws:
depth-tested first (with depth test enabled), then overlay (depth test disabled).

## Related Docs

- [docs/debug_draw_api_examples.md](../../../docs/debug_draw_api_examples.md) — GameAPI usage examples
