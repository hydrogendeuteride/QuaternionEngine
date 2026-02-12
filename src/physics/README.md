# Physics

> Backend-agnostic rigid body simulation framework with collision detection, raycasting, and contact events.

## Purpose

Provides a unified physics abstraction layer that supports:
- Rigid body simulation with Static/Kinematic/Dynamic motion types
- Collision shapes: primitives (box, sphere, capsule, cylinder, cone, plane), triangle meshes, compounds
- Spatial queries: raycasts, shape sweeps, overlap tests with layer filtering
- Contact events: collision and trigger callbacks (Begin/Stay/End)
- Joints: fixed, hinge, slider constraints
- Floating origin support for large-world simulation via `PhysicsContext`

## Directory Layout

```
physics/
├── physics_world.h              — abstract PhysicsWorld interface + BodyBuilder fluent API
├── physics_context.h/.cpp       — PhysicsContext for floating origin management
├── physics_body.h/.cpp          — BodyId, BodyHandle, query types (RayHit, OverlapHit, etc.)
├── body_settings.h              — BodySettings, MotionType enum, Layer constants
├── collision_shape.h            — shape types (BoxShape, SphereShape, CompoundShape, etc.)
├── collider_asset.h/.cpp        — collider scaling utilities for glTF instances
├── collider_mesh_instance.h     — triangle mesh collider metadata
├── gltf_collider_parser.h/.cpp  — COL_* marker parsing for glTF collider authoring
└── jolt/                        — Jolt Physics backend implementation (see jolt/README.md)
    ├── jolt_physics_world.h/.cpp
    └── jolt_query_filters.h
```

## Key Types

| Type | Role |
|------|------|
| `PhysicsWorld` | Abstract interface for all physics operations (bodies, queries, simulation) |
| `PhysicsContext` | Manages coordinate system origins and rebasing for floating origin |
| `BodyId` | Strongly-typed body identifier (uint32_t wrapper) |
| `BodyHandle` | RAII wrapper that auto-destroys body on scope exit |
| `BodySettings` | Body creation parameters (shape, mass, friction, layer, etc.) |
| `BodyBuilder` | Fluent API for body creation with chainable setters |
| `CollisionShape` | Unified shape representation (variant of primitive/mesh/compound) |
| `CompoundShape` | Multi-primitive collider with child shapes at relative transforms |
| `MotionType` | Static (immovable), Kinematic (script-driven), Dynamic (fully simulated) |
| `RayHit` | Raycast result with position, normal, distance, body, user_data |
| `CollisionEvent` | Contact event data (type, bodies, point, normal, penetration) |
| `TriggerEvent` | Sensor overlap event data (type, bodies, point) |

## Lifecycle

```
PhysicsWorld* physics = new JoltPhysicsWorld(config);
PhysicsContext* ctx = new PhysicsContext();
ctx->set_physics_world(physics);

// Body creation
BodySettings settings;
settings.set_shape(CollisionShape::Sphere(0.5f))
        .set_position(0.0, 10.0, 0.0)
        .set_dynamic();
BodyId ball = physics->create_body(settings);

// Or use BodyBuilder fluent API
BodyId box = BodyBuilder(physics)
    .box(1.0f, 1.0f, 1.0f)
    .position(5.0, 2.0, 0.0)
    .dynamic_body()
    .mass(10.0f)
    .layer(Layer::Dynamic)
    .build();

// Simulation loop
while (running) {
    physics->step(1.0f / 60.0f);  // 60 Hz fixed timestep

    // Query transforms
    glm::dvec3 pos = physics->get_position(ball);
    glm::quat rot = physics->get_rotation(ball);

    // Raycast
    RayHit hit = physics->raycast(origin, direction, max_distance);

    // Floating origin rebasing
    ctx->maybe_rebase_origin_to_body(ball.value, 1000.0, 100.0);
}

// Cleanup
physics->destroy_body(ball);
physics->destroy_body(box);
delete ctx;
delete physics;
```

## Usage

### Creating bodies with BodyBuilder

```cpp
// Dynamic sphere
BodyId sphere = BodyBuilder(physics)
    .sphere(0.5f)
    .position(0.0, 10.0, 0.0)
    .dynamic_body()
    .mass(1.0f)
    .friction(0.8f)
    .restitution(0.5f)  // Bounciness
    .layer(Layer::Dynamic)
    .build();

// Kinematic platform
BodyId platform = BodyBuilder(physics)
    .box(5.0f, 0.2f, 5.0f)
    .position(0.0, 0.0, 0.0)
    .kinematic_body()
    .layer(Layer::Kinematic)
    .build();

// Static ground plane
BodyId ground = BodyBuilder(physics)
    .plane(glm::vec3(0, 1, 0))  // Up-facing plane
    .position(0.0, -1.0, 0.0)
    .static_body()
    .layer(Layer::Static)
    .build();
```

### Compound shapes

```cpp
// Character capsule + feet box
CompoundShape character;
character.add_capsule(0.4f, 0.9f, glm::vec3(0.0f, 1.0f, 0.0f));
character.add_box(glm::vec3(0.3f, 0.1f, 0.2f), glm::vec3(0.0f, 0.0f, 0.0f));

BodyId player = BodyBuilder(physics)
    .compound(character)
    .position(0.0, 2.0, 0.0)
    .kinematic_body()
    .layer(Layer::Player)
    .user_data(entity_id)
    .build();
```

### Raycasting with filtering

```cpp
RaycastOptions options;
options.max_distance = 100.0f;
options.layer_mask = (1u << Layer::Enemy) | (1u << Layer::Static);
options.ignore_body = player_body;
options.include_sensors = false;

RayHit hit = physics->raycast(camera_pos, camera_dir, options);
if (hit.hit) {
    fmt::println("Hit body {} at {}", hit.body_id.value, hit.distance);
}
```

### Contact callbacks

```cpp
PhysicsWorld::BodyCallbacks callbacks;

callbacks.on_collision = [](const CollisionEvent& e) {
    if (e.type == ContactEventType::Begin) {
        // Collision started
        handle_collision(e.self, e.other, e.normal, e.penetration_depth);
    }
};

callbacks.on_trigger = [](const TriggerEvent& e) {
    if (e.type == ContactEventType::Begin) {
        // Sensor entered
        handle_trigger_enter(e.self, e.other);
    }
};

physics->set_body_callbacks(sensor_body, callbacks);
```

### Floating origin rebasing

```cpp
// Manual rebasing
if (glm::length(player_pos) > 1000.0) {
    glm::dvec3 shift = -player_pos;
    physics->shift_origin(shift);
    physics_context->set_origin_world(physics_context->origin_world() + shift);
}

// Automatic rebasing via PhysicsContext
bool rebased = physics_context->maybe_rebase_origin_to_body(
    player_body.value,  // Track this body
    1000.0,             // Rebase when |local_pos| > 1000m
    100.0               // Snap to 100m grid
);
```

## Integration

`PhysicsWorld*` is stored in `EngineContext::physics` and typically points to a `JoltPhysicsWorld` instance.
`PhysicsContext*` is stored in `EngineContext::physics_context` and manages coordinate system rebasing.

**SceneManager** integrates physics via:
- `enableColliderSync(instanceName, physicsWorld, layer, user_data)` — creates kinematic bodies from glTF colliders
- `syncColliders()` — updates physics body transforms to match animated node transforms each frame

**AssetManager** automatically extracts colliders from glTF files via:
- Embedded `COL_*` marker nodes (see `gltf_collider_parser.h`)
- Sidecar files (`model.colliders.glb`)

## Related Docs

- [docs/PhysicsSystem.md](../../docs/PhysicsSystem.md) — detailed physics system documentation
- [docs/JoltBackend.md](../../docs/JoltBackend.md) — Jolt Physics backend implementation
- [docs/ColliderSystem.md](../../docs/ColliderSystem.md) — glTF collider authoring workflow
- [docs/FloatingOrigin.md](../../docs/FloatingOrigin.md) — large-world coordinate management
