# Picking

> Unified object selection system supporting click, hover, and drag-box multi-select via CPU ray-casting or GPU ID-buffer readback.

## Purpose

Provides a central system for picking objects in the 3D viewport. Consumes
mouse input (click, drag, release) via the engine `InputSystem`, maintains per-frame hover and selection
state, and integrates with the RenderGraph for asynchronous GPU ID-buffer
readback. Two picking backends are available: immediate CPU ray-casting through
the SceneManager BVH, and deferred GPU ID-buffer picking with single-pixel
readback.

## Directory Layout

```
picking/
├── picking_system.h    — PickingSystem class, PickInfo result struct
└── picking_system.cpp  — implementation (event handling, readback, coordinate transforms)
```

## Key Types

| Type | Role |
|------|------|
| `PickingSystem` | Central entry point — event processing, pick state, RenderGraph integration |
| `PickInfo` | Result of a pick: mesh, node, owner, world position, transform, surface indices, and glTF node hierarchy metadata |
| `PickRequest` | Internal pending pick request with window/ID-buffer coordinates |
| `DragState` | Internal mouse drag tracking (start, current, threshold detection) |

## Lifecycle

```
init(EngineContext*)
  └─ stores context, creates CPU-readable readback buffer (1 × uint32)

begin_frame()
  └─ resolves pending async ID-buffer pick from previous frame
     ├─ reads picked objectID from readback buffer
     └─ calls SceneManager::resolveObjectID → populates _last_pick

process_input(InputSystem, ui_want_capture_mouse)
  ├─ MouseMove → updates cursor position, tracks drag
  ├─ MouseButtonDown → starts potential drag
  └─ MouseButtonUp
     ├─ click (<= threshold motion) → CPU ray pick or queues ID-buffer pick
     └─ drag  → rectangle multi-select via SceneManager::selectRect

update_hover(ui_want_capture_mouse)
  └─ CPU ray-cast under cursor → populates _hover_pick

register_id_buffer_readback(RenderGraph, id_buffer, ...)
  └─ adds PickReadback transfer pass to copy single pixel → readback buffer

cleanup()
  └─ destroys readback buffer, resets all state
```

## Usage

### CPU Ray Picking (default)

```cpp
// Run once per-frame after InputSystem::pump_events()
const bool ui_capture_mouse = engine->ui() && engine->ui()->want_capture_mouse();
picking.process_input(*engine->input(), ui_capture_mouse);
picking.update_hover(ui_capture_mouse);

// Optional: allow multiple selection buttons (Left + Right)
picking.settings().select_button_mask =
    (1u << static_cast<uint32_t>(MouseButton::Left)) |
    (1u << static_cast<uint32_t>(MouseButton::Right));

// Query results
const auto& pick = picking.last_pick();
if (pick.valid)
    fmt::println("Selected: {} (node='{}')", pick.ownerName, pick.nodeName);
```

### Hierarchical glTF Selection

```cpp
// Move current glTF selection to parent node.
picking.move_last_pick_to_parent();

// Move to first child (or choose a specific child by index/name).
picking.move_last_pick_to_child();
picking.move_last_pick_to_child(2);
picking.move_last_pick_to_child("Armature/Spine");
```

For glTF picks, `PickInfo` now includes:

- `nodeName` — selected node name in `LoadedGLTF::nodes`
- `nodeParentName` — direct parent node name (empty for roots)
- `nodeChildren` — direct child node names
- `nodePath` — root→selected node path

### GPU ID-Buffer Picking

```cpp
picking.set_use_id_buffer_picking(true);

// During RenderGraph build, after the geometry pass produces the ID buffer:
picking.register_id_buffer_readback(graph, id_buffer, draw_extent, swapchain_extent);

// Result is available next frame via begin_frame() → last_pick()
```

### Drag-Box Multi-Select

```cpp
const auto& selection = picking.drag_selection();
for (const auto& pick : selection)
{
    if (pick.valid)
        add_to_selection(pick.ownerName);
}
```

## Integration

`PickingSystem` is owned by `VulkanEngine` and exposed via `EngineContext`.
It interacts with:

- **SceneManager** — CPU ray-casting (`pick`, `selectRect`, `resolveObjectID`)
- **RenderGraph** — registers a `PickReadback` transfer pass for ID-buffer mode
- **InputSystem** — consumes mouse input (`process_input`, `MouseButton`, `InputEvent`)
- **ImGui** — respects `ui_want_capture_mouse` to avoid picking through UI

Coordinate transforms handle HiDPI scaling (window pixels → drawable pixels →
swapchain pixels) and letterboxing when render resolution differs from swapchain
size.

## Related Docs

- [docs/Picking.md](../../../docs/Picking.md) — detailed picking system documentation
