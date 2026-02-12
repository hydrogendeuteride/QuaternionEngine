# Jolt Physics Backend

> Jolt Physics implementation of the PhysicsWorld interface with multi-threaded simulation and optimized broad-phase.

## Purpose

Concrete implementation of `Physics::PhysicsWorld` using the Jolt Physics library (https://github.com/jrouwe/JoltPhysics).

Features:
- High-performance rigid body simulation with job system parallelization
- 2-layer broad-phase hierarchy (non_moving vs. moving)
- Shape caching for triangle meshes
- Contact event queuing with Begin/Stay/End callbacks
- Layer-based collision filtering with 16×16 bitmask matrix
- Floating origin support via body translation and velocity shifting
- Debug instrumentation (performance counters, body iteration)

## Directory Layout

```
jolt/
├── jolt_physics_world.h    — JoltPhysicsWorld class declaration
├── jolt_physics_world.cpp  — implementation (~2000 lines)
└── jolt_query_filters.h    — LayerMaskFilter, IgnoreBodyAndSensorsFilter
```

## Key Types

| Type | Role |
|------|------|
| `JoltPhysicsWorld` | Main backend class implementing `PhysicsWorld` interface |
| `JoltPhysicsWorld::Config` | Configuration struct (max_bodies, gravity, allocator size) |
| `JoltGlobals` | Reference-counted global Jolt initialization (allocator, factory) |
| `BPLayerInterfaceImpl` | Maps object layers to broad-phase layers (non_moving/moving) |
| `ObjectLayerPairFilterImpl` | Collision matrix filtering using bitmask array |
| `ContactListenerImpl` | Jolt contact listener that queues events for deferred dispatch |
| `LayerMaskFilter` | Query filter by layer bitmask |
| `IgnoreBodyAndSensorsFilter` | Query filter ignoring specific body + sensors |

## Lifecycle

```
// Create with default config
auto jolt_world = std::make_unique<JoltPhysicsWorld>();

// Or custom config
JoltPhysicsWorld::Config config;
config.max_bodies = 8192;
config.gravity = glm::vec3(0.0f, -3.71f, 0.0f);  // Mars gravity
auto jolt_world = std::make_unique<JoltPhysicsWorld>(config);

// Use via PhysicsWorld interface
PhysicsWorld* physics = jolt_world.get();

// Create bodies
BodyId ball = BodyBuilder(physics)
    .sphere(0.5f)
    .position(0.0, 10.0, 0.0)
    .dynamic_body()
    .build();

// Step simulation (internally calls JPH::PhysicsSystem::Update)
physics->step(1.0f / 60.0f);

// Contact events dispatched after step
// Callbacks invoked from _queued_events

// Cleanup (auto-destroys all bodies, joints, cached shapes)
jolt_world.reset();
```

## Implementation Details

### Broad-Phase Hierarchy

```
Jolt 2-layer broad-phase:
  BroadPhaseLayers::non_moving (0) ← Static + Kinematic bodies
  BroadPhaseLayers::moving (1)     ← Dynamic bodies

ObjectLayers (0-15) map to engine's Physics::Layer constants:
  Layer::Static    → non_moving
  Layer::Kinematic → non_moving
  Layer::Dynamic   → moving
  Layer::Player    → moving (if Dynamic) or non_moving (if Kinematic)
  ...
```

### Shape Caching

```cpp
// Triangle meshes built once and cached by shared_ptr key
std::unordered_map<std::shared_ptr<const TriangleMeshData>,
                   JPH::RefConst<JPH::Shape>> _mesh_shape_cache;

// Example: load mesh collider
auto mesh_data = load_mesh("terrain.obj");
CollisionShape shape = CollisionShape::TriangleMesh(mesh_data);

BodyId terrain1 = BodyBuilder(physics).shape(shape).static_body().build();
BodyId terrain2 = BodyBuilder(physics).shape(shape).static_body().build();
// ^^^ Both reuse the same Jolt MeshShape (cache hit)
```

### Contact Event Flow

```
1. Jolt detects contact during PhysicsSystem::Update()
   ↓
2. ContactListenerImpl::OnContactAdded/Persisted/Removed
   ↓
3. Event queued in _queued_events (thread-safe)
   ↓
4. step() completes
   ↓
5. dispatch_contact_events() invokes user callbacks
   ↓
6. Callbacks receive CollisionEvent or TriggerEvent
```

### Collision Matrix

```cpp
// 16 layers × 16 layers, stored as bitmask per layer
uint32_t _layer_collision_mask[Layer::Count];

// Set collision between layers
physics->set_layer_collision(Layer::Player, Layer::Enemy, true);
physics->set_layer_collision(Layer::Player, Layer::Trigger, false);

// ObjectLayerPairFilterImpl checks:
bool ShouldCollide(ObjectLayer layer1, ObjectLayer layer2) const {
    return (_layer_collision_mask[layer1] & (1u << layer2)) != 0;
}
```

### Floating Origin

```cpp
// Position origin shift (translates all bodies)
void shift_origin(const glm::dvec3& delta_local) {
    for (all bodies) {
        RVec3 pos = body_interface.GetPosition(body_id);
        body_interface.SetPosition(body_id, pos + delta_local);
    }
}

// Velocity origin shift (Galilean transform)
void shift_velocity_origin(const glm::dvec3& delta_local_velocity) {
    for (all bodies) {
        Vec3 vel = body_interface.GetLinearVelocity(body_id);
        body_interface.SetLinearVelocity(body_id, vel - delta_local_velocity);
    }
}
```

## Configuration

### Compile Flags

```cmake
# Enable Jolt backend in CMakeLists.txt
set(VULKAN_ENGINE_USE_JOLT ON)

# Jolt options (optional)
set(JPH_ENABLE_ASSERTS ON)        # Assertion checks (debug)
set(JPH_PROFILE_ENABLED ON)       # Profiling hooks
set(JPH_DOUBLE_PRECISION OFF)     # Use float (default)
set(JPH_OBJECT_LAYER_BITS 16)     # 16-bit object layers
```

### Runtime Config

```cpp
JoltPhysicsWorld::Config config;
config.max_bodies = 4096;                      // Body pool size
config.max_body_pairs = 4096;                  // Contact pair pool
config.max_contact_constraints = 4096;         // Constraint pool
config.temp_allocator_size = 10 * 1024 * 1024; // 10 MB temp allocator
config.gravity = glm::vec3(0.0f, -9.81f, 0.0f);

auto jolt_world = std::make_unique<JoltPhysicsWorld>(config);
```

## Usage

### Direct Jolt access (advanced)

```cpp
// Get Jolt PhysicsSystem
JPH::PhysicsSystem& jolt_sys = jolt_world->jolt_system();

// Get BodyInterface for advanced operations
JPH::BodyInterface& body_iface = jolt_world->body_interface();

// Manual body creation with Jolt API
JPH::BodyCreationSettings settings;
settings.mMotionType = JPH::EMotionType::Dynamic;
settings.mObjectLayer = 2;
settings.SetShape(new JPH::SphereShape(0.5f));
settings.mPosition = JPH::RVec3(0, 10, 0);

JPH::Body* body = body_iface.CreateBody(settings);
body_iface.AddBody(body->GetID(), JPH::EActivation::Activate);
```

### Debug stats

```cpp
auto stats = jolt_world->debug_stats();
fmt::println("Step: {:.2f}ms (avg {:.2f}ms)", stats.last_step_ms, stats.avg_step_ms);
fmt::println("Bodies: {} ({} active)", stats.body_count, stats.active_body_count);
fmt::println("Joints: {}", stats.joint_count);
fmt::println("Contacts: {}", stats.contact_event_count);
```

### Debug rendering

```cpp
jolt_world->for_each_debug_body([&](const PhysicsWorld::DebugBodyView& body) {
    if (body.shape.is_box()) {
        const auto* box = body.shape.as_box();
        draw_debug_box(body.position, body.rotation, box->half_extents);
    } else if (body.shape.is_sphere()) {
        const auto* sphere = body.shape.as_sphere();
        draw_debug_sphere(body.position, sphere->radius);
    }

    // Color by layer or type
    glm::vec4 color = body.is_sensor ? glm::vec4(0, 1, 0, 0.5f)
                                     : glm::vec4(1, 1, 1, 1);
});
```

## Performance Tips

- **Broad-phase**: Keep static/kinematic bodies in `non_moving` layer for fast culling
- **Shape complexity**: Prefer boxes/spheres/capsules over compounds/meshes for dynamics
- **Job system**: Jolt auto-detects CPU cores; typical config is `num_cores - 1` worker threads
- **Temp allocator**: Increase `temp_allocator_size` if seeing allocation warnings
- **Layer filtering**: Use narrow layer masks in queries to reduce collision checks
- **Sleeping**: Enable `allow_sleeping` for bodies that frequently come to rest

## Limitations

- **Single-precision**: Jolt uses `float` internally; double-precision maintained via origin shifting
- **Triangle meshes**: Static-only (Jolt `MeshShape` limitation); use convex decomposition for dynamics
- **CCD**: Continuous collision detection not exposed; requires direct Jolt API access
- **Constraints**: Only fixed/hinge/slider exposed; Jolt supports more (distance, cone, gear)

## Thread Safety

All `PhysicsWorld` API methods are thread-safe via internal mutexes:
- `_callbacks_mutex` — body callback registration
- `_events_mutex` — contact event queuing
- `_layer_mutex` — collision matrix updates
- `_mesh_shape_cache_mutex` — mesh shape cache
- `_joints_mutex` — joint map
- `_debug_bodies_mutex` — debug metadata

Jolt's `BodyInterface` is thread-safe for queries but not for writes (use locking or schedule writes for step boundary).

## Global Initialization

Jolt requires global setup (`JPH::RegisterDefaultAllocator`, `JPH::Factory`, etc.). This is handled via reference-counted `JoltGlobals`:

```cpp
struct JoltGlobals {
    JoltGlobals() {
        if (ref_count++ == 0) {
            JPH::RegisterDefaultAllocator();
            JPH::Factory::sInstance = new JPH::Factory();
            JPH::RegisterTypes();
        }
    }
    ~JoltGlobals() {
        if (--ref_count == 0) {
            JPH::UnregisterTypes();
            delete JPH::Factory::sInstance;
        }
    }
};
```

Multiple `JoltPhysicsWorld` instances can coexist safely; Jolt is initialized on first instance and cleaned up on last destruction.

## Related Docs

- [docs/JoltBackend.md](../../../docs/JoltBackend.md) — detailed Jolt backend documentation
- [docs/PhysicsSystem.md](../../../docs/PhysicsSystem.md) — abstract physics system API
- Jolt Physics documentation: https://jrouwe.github.io/JoltPhysics/
