## Jolt Physics Backend: Implementation Details

Jolt Physics (https://github.com/jrouwe/JoltPhysics) backend for the engine's physics system. Implements the abstract `PhysicsWorld` interface with high-performance rigid body simulation, broad-phase optimization, and contact event handling.

### Overview

`JoltPhysicsWorld` is the concrete implementation of the `Physics::PhysicsWorld` interface using the Jolt Physics library. It provides:

- Multi-threaded physics simulation with job system
- Optimized broad-phase collision detection (2-layer hierarchy)
- Shape caching for triangle meshes
- Contact event queuing with Begin/Stay/End callbacks
- Layer-based collision filtering with bitmask matrix
- Floating origin support via `shift_origin()` and `shift_velocity_origin()`
- Debug instrumentation for profiling and visualization

### Key Components

#### `JoltPhysicsWorld` — Main Backend Class

Located in `src/physics/jolt/jolt_physics_world.h/.cpp`, this class wraps the Jolt Physics API and implements all `PhysicsWorld` methods.

**Configuration:**

```cpp
struct Config {
    uint32_t max_bodies = 4096;                      // Body pool size
    uint32_t max_body_pairs = 4096;                  // Contact pair pool
    uint32_t max_contact_constraints = 4096;         // Constraint pool
    uint32_t temp_allocator_size = 10 * 1024 * 1024; // 10 MB temp allocator
    glm::vec3 gravity{0.0f, -9.81f, 0.0f};          // Gravity vector
};

// Create with default config
auto jolt_world = std::make_unique<Physics::JoltPhysicsWorld>();

// Create with custom config
Physics::JoltPhysicsWorld::Config config;
config.max_bodies = 8192;
config.gravity = glm::vec3(0.0f, -3.71f, 0.0f);  // Mars gravity
auto jolt_world = std::make_unique<Physics::JoltPhysicsWorld>(config);
```

**Direct Jolt access (advanced):**

```cpp
// Access Jolt PhysicsSystem directly
JPH::PhysicsSystem& jolt_sys = jolt_world->jolt_system();

// Access BodyInterface for advanced operations
JPH::BodyInterface& body_iface = jolt_world->body_interface();
```

#### Collision Layer Mapping

Jolt uses a 2-layer broad-phase hierarchy for optimization:

**Broad-phase layers:**
- `BroadPhaseLayers::non_moving` — Static and kinematic bodies
- `BroadPhaseLayers::moving` — Dynamic bodies

**Object layers:**
- Map 1:1 to engine's `Physics::Layer` constants (0-15)
- Filtered by `ObjectLayerPairFilterImpl` using collision matrix bitmask

**Collision matrix:**

```cpp
// 16 layers × 16 layers, stored as bitmask array
uint32_t _layer_collision_mask[Layer::Count];  // [i] = bitmask of layers that collide with layer i

// Set via PhysicsWorld API
physics->set_layer_collision(Layer::Player, Layer::Enemy, true);
physics->set_layer_collision(Layer::Player, Layer::Projectile, false);
```

#### Shape Creation and Caching

**Primitive shapes:**
- Converted directly to Jolt shapes (BoxShape, SphereShape, CapsuleShape, etc.)
- Tapered cylinder maps to Jolt's `TaperedCylinderShape` (supports cones)

**Compound shapes:**
- Converted to Jolt `StaticCompoundShape`
- Each child primitive positioned with `CompoundShapeSettings::AddShape()`

**Triangle meshes:**
- Built once from `TriangleMeshData`, cached in `_mesh_shape_cache`
- Shared pointers used as cache key to avoid rebuilding identical meshes
- Thread-safe access via `_mesh_shape_cache_mutex`

```cpp
// Internal caching logic (automatic)
JPH::RefConst<JPH::Shape> JoltPhysicsWorld::create_jolt_shape(const CollisionShape& shape) {
    if (shape.is_triangle_mesh()) {
        auto* mesh_shape = shape.as_triangle_mesh();
        // Check cache first
        {
            std::lock_guard lock(_mesh_shape_cache_mutex);
            auto it = _mesh_shape_cache.find(mesh_shape->mesh);
            if (it != _mesh_shape_cache.end()) {
                return it->second;  // Cache hit
            }
        }
        // Build and cache
        JPH::RefConst<JPH::Shape> jolt_mesh = build_jolt_mesh(mesh_shape);
        {
            std::lock_guard lock(_mesh_shape_cache_mutex);
            _mesh_shape_cache[mesh_shape->mesh] = jolt_mesh;
        }
        return jolt_mesh;
    }
    // ... handle other shape types
}
```

#### Contact Event System

Jolt's `ContactListener` is wrapped to queue events for deferred dispatch:

**Event flow:**
1. Jolt detects contacts during `PhysicsSystem::Update()`
2. `ContactListenerImpl` receives `OnContactAdded`, `OnContactPersisted`, `OnContactRemoved`
3. Events queued in `_queued_events` (protected by `_events_mutex`)
4. `dispatch_contact_events()` called after step, invokes user callbacks

**Pair cache:**
- Tracks contact pairs with `SubShapeIDPair` hash
- Stores `is_trigger`, `user_data`, `layer` for removed event reconstruction
- Cleared on body destruction to prevent dangling references

```cpp
// Example: ContactListener callback
void JoltPhysicsWorld::ContactListenerImpl::OnContactAdded(
    const JPH::Body& body1,
    const JPH::Body& body2,
    const JPH::ContactManifold& manifold,
    JPH::ContactSettings& settings)
{
    bool is_trigger = body1.IsSensor() || body2.IsSensor();

    ContactEventRecord rec;
    rec.is_trigger = is_trigger;

    if (is_trigger) {
        rec.trigger.type = ContactEventType::Begin;
        rec.trigger.self.value = body1.GetID().GetIndexAndSequenceNumber();
        rec.trigger.other.value = body2.GetID().GetIndexAndSequenceNumber();
        // ... populate fields
    } else {
        rec.collision.type = ContactEventType::Begin;
        rec.collision.self.value = body1.GetID().GetIndexAndSequenceNumber();
        rec.collision.other.value = body2.GetID().GetIndexAndSequenceNumber();
        rec.collision.normal = jolt_to_glm(manifold.mWorldSpaceNormal);
        rec.collision.penetration_depth = manifold.mPenetrationDepth;
        // ... populate fields
    }

    std::lock_guard lock(_world->_events_mutex);
    _world->_queued_events.push_back(rec);
}
```

#### Query Filters

Located in `src/physics/jolt/jolt_query_filters.h`, these helper classes implement Jolt's filtering interfaces for raycasts and shape queries.

**`LayerMaskFilter`:**
- Implements `BroadPhaseLayerFilter` and `ObjectLayerFilter`
- Filters queries by layer bitmask

```cpp
// Example: Raycast only against Enemy and Static layers
uint32_t layer_mask = (1u << Physics::Layer::Enemy) | (1u << Physics::Layer::Static);
Physics::JoltQuery::LayerMaskFilter filter(layer_mask);

// Used internally in raycast()
```

**`IgnoreBodyAndSensorsFilter`:**
- Implements `BodyFilter`
- Ignores specific body ID (e.g., shooter in weapon raycast)
- Optionally ignores sensor bodies

```cpp
// Example: Raycast ignoring player body
Physics::JoltQuery::IgnoreBodyAndSensorsFilter filter(
    player_body,           // Ignore this body
    false,                 // Exclude sensors
    body_lock_interface
);
```

### Floating Origin Support

Jolt bodies use single-precision (`JPH::RVec3` = `float` in non-double mode), but the engine uses double-precision world positions. The backend handles this via origin shifting:

**Position origin shift:**

```cpp
void JoltPhysicsWorld::shift_origin(const glm::dvec3& delta_local) {
    JPH::BodyInterface& body_iface = _physics_system.GetBodyInterface();

    // Translate all bodies by delta_local
    for (uint32_t i = 0; i < max_bodies; ++i) {
        JPH::BodyID jolt_id(i);
        if (body_iface.IsAdded(jolt_id)) {
            JPH::RVec3 pos = body_iface.GetPosition(jolt_id);
            body_iface.SetPosition(jolt_id, pos + glm_to_jolt(delta_local),
                                   JPH::EActivation::DontActivate);
        }
    }
}
```

**Velocity origin shift (Galilean transform):**

```cpp
void JoltPhysicsWorld::shift_velocity_origin(const glm::dvec3& delta_local_velocity) {
    JPH::BodyInterface& body_iface = _physics_system.GetBodyInterface();

    for (uint32_t i = 0; i < max_bodies; ++i) {
        JPH::BodyID jolt_id(i);
        if (body_iface.IsAdded(jolt_id)) {
            JPH::Vec3 vel = body_iface.GetLinearVelocity(jolt_id);
            body_iface.SetLinearVelocity(jolt_id, vel - glm_to_jolt(delta_local_velocity));
        }
    }
}
```

**Usage:**
- Must be called **before** `step()` in the same frame
- Typically invoked via `PhysicsContext::maybe_rebase_origin_to_body()`

### Multi-Threading

Jolt uses a job system for parallel simulation:

**Job system initialization:**

```cpp
// In JoltPhysicsWorld::init()
int num_threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1);
_job_system = std::make_unique<JPH::JobSystemThreadPool>(
    JPH::cMaxPhysicsJobs,           // Max concurrent jobs
    JPH::cMaxPhysicsBarriers,       // Max barriers
    num_threads                     // Worker threads
);
```

**Thread-safe operations:**
- All `PhysicsWorld` API methods are thread-safe via internal locking
- Callback registration (`set_body_callbacks`) protected by `_callbacks_mutex`
- Contact event queuing protected by `_events_mutex`
- Layer collision matrix protected by `_layer_mutex`

### Debug Instrumentation

**Performance stats:**

```cpp
PhysicsWorld::DebugStats JoltPhysicsWorld::debug_stats() const {
    DebugStats stats;
    stats.last_step_ms = _debug_last_step_ms.load();
    stats.avg_step_ms = _debug_avg_step_ms.load();
    stats.last_dt = _debug_last_dt.load();
    stats.body_count = _debug_body_count.load();
    stats.active_body_count = _debug_active_body_count.load();
    stats.joint_count = _debug_joint_count.load();
    stats.contact_event_count = _debug_contact_event_count.load();
    return stats;
}
```

**Body iteration for debug rendering:**

```cpp
physics->for_each_debug_body([&](const PhysicsWorld::DebugBodyView& body) {
    // body.id, body.position, body.rotation, body.shape, body.layer, etc.
    render_debug_shape(body.shape, body.position, body.rotation);
});
```

**Debug metadata storage:**
- `_debug_bodies` map stores `BodyDebugRecord` for each body
- Contains shape, motion type, layer, is_sensor flag
- Updated on body creation, cleared on destruction
- Thread-safe access via `_debug_bodies_mutex`

### Coordinate Conversion

**GLM ↔ Jolt conversions:**

```cpp
// Internal helper functions
inline JPH::Vec3 glm_to_jolt(const glm::vec3& v) {
    return JPH::Vec3(v.x, v.y, v.z);
}

inline JPH::RVec3 glm_to_jolt(const glm::dvec3& v) {
    return JPH::RVec3(static_cast<float>(v.x),
                      static_cast<float>(v.y),
                      static_cast<float>(v.z));
}

inline glm::vec3 jolt_to_glm(const JPH::Vec3& v) {
    return glm::vec3(v.GetX(), v.GetY(), v.GetZ());
}

inline glm::dvec3 jolt_to_glm_dvec3(const JPH::RVec3& v) {
    return glm::dvec3(v.GetX(), v.GetY(), v.GetZ());
}

inline JPH::Quat glm_to_jolt(const glm::quat& q) {
    return JPH::Quat(q.x, q.y, q.z, q.w);
}

inline glm::quat jolt_to_glm(const JPH::Quat& q) {
    return glm::quat(q.GetW(), q.GetX(), q.GetY(), q.GetZ());
}
```

### Global Initialization

Jolt requires global initialization (`JPH::RegisterDefaultAllocator`, `JPH::Factory::sInstance->Register()`, etc.). The backend uses reference-counted initialization via `JoltGlobals` to handle multiple `JoltPhysicsWorld` instances:

```cpp
struct JoltGlobals {
    JoltGlobals() {
        std::lock_guard lock(mutex());
        if (ref_count()++ == 0) {
            JPH::RegisterDefaultAllocator();
            JPH::Factory::sInstance = new JPH::Factory();
            JPH::RegisterTypes();
            // Install trace/assert handlers
        }
    }

    ~JoltGlobals() {
        std::lock_guard lock(mutex());
        if (--ref_count() == 0) {
            JPH::UnregisterTypes();
            delete JPH::Factory::sInstance;
            JPH::Factory::sInstance = nullptr;
        }
    }
};
```

Each `JoltPhysicsWorld` instance holds a `_globals` member (first member for proper init order), ensuring Jolt is initialized before use and cleaned up when the last instance is destroyed.

### Limitations & Caveats

- **Single-precision physics**: Jolt uses `float` internally; double-precision positions maintained via origin shifting.
- **Triangle meshes**: Static-only (Jolt `MeshShape` limitation); use convex decomposition for dynamic meshes.
- **Continuous collision detection (CCD)**: Not exposed in the abstraction layer; use Jolt's `BodyInterface` directly if needed.
- **Constraints**: Only fixed, hinge, and slider joints exposed; Jolt supports more (distance, cone, gear) via direct API access.
- **Sleeping bodies**: Automatically managed by Jolt; `activate()`/`deactivate()` can force state.

### Build Configuration

**CMake options:**

```cmake
# Enable Jolt backend
set(VULKAN_ENGINE_USE_JOLT ON)

# Jolt compile flags (optional)
set(JPH_ENABLE_ASSERTS ON)          # Assertion checks
set(JPH_PROFILE_ENABLED ON)         # Profiling hooks
set(JPH_DOUBLE_PRECISION OFF)       # Use float (default)
set(JPH_OBJECT_LAYER_BITS 16)       # 16-bit object layers
```

**Include paths:**

```cpp
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
// ... other Jolt headers
#endif
```

### Troubleshooting

**Assertion failures:**
- Ensure `JPH_ENABLE_ASSERTS` is enabled in debug builds
- Check `assert_failed_impl()` logs for detailed error info
- Common issues: invalid body IDs, double-destruction, constraint errors

**Performance issues:**
- Check `debug_stats().last_step_ms` for frame time spikes
- Reduce `max_body_pairs` or `max_contact_constraints` if memory-bound
- Use layer filtering to reduce broad-phase checks

**Contact events not firing:**
- Verify `set_layer_collision()` allows collision between layers
- Check body motion types (static-static contacts don't generate events)
- Ensure callbacks registered via `set_body_callbacks()` before simulation

**Floating origin drift:**
- Call `shift_origin()` **before** `step()` each frame
- Use `PhysicsContext::maybe_rebase_origin_to_body()` for automatic rebasing
- Verify origin shift delta matches render world shift

### File Structure

```
src/physics/jolt/
├── jolt_physics_world.h           # JoltPhysicsWorld class declaration
├── jolt_physics_world.cpp         # Implementation (~2000 lines)
│   ├── Global init (JoltGlobals)
│   ├── Constructor/destructor
│   ├── Shape creation + caching
│   ├── Body API implementation
│   ├── Query API (raycast, sweep, overlap)
│   ├── Contact listener + event dispatch
│   ├── Layer filtering
│   ├── Joint API
│   └── Debug instrumentation
└── jolt_query_filters.h           # LayerMaskFilter, IgnoreBodyAndSensorsFilter

third_party/JoltPhysics/           # Jolt library (git submodule or vendored)
```

### Performance Tips

- **Broad-phase optimization**: Keep static/kinematic bodies in `non_moving` layer, dynamics in `moving` layer.
- **Shape complexity**: Prefer simple shapes (box, sphere, capsule) over compound/mesh for dynamic bodies.
- **Layer filtering**: Use narrow layer masks in queries to reduce collision checks.
- **Sleeping**: Allow `allow_sleeping = true` for static/resting dynamic bodies.
- **Job system**: More worker threads ≠ always faster; tune based on workload (typical: `num_cores - 1`).
- **Temp allocator**: Increase `temp_allocator_size` if seeing allocation warnings in logs.

### Advanced Usage

**Direct Jolt access:**

```cpp
// Get Jolt PhysicsSystem
JPH::PhysicsSystem& jolt_sys = jolt_world->jolt_system();

// Manually add body with advanced settings
JPH::BodyCreationSettings settings;
settings.mMotionType = JPH::EMotionType::Dynamic;
settings.mObjectLayer = 2;
settings.SetShape(new JPH::SphereShape(0.5f));
settings.mPosition = JPH::RVec3(0, 10, 0);
settings.mAllowSleeping = true;
settings.mFriction = 0.8f;

JPH::Body* body = jolt_sys.GetBodyInterface().CreateBody(settings);
jolt_sys.GetBodyInterface().AddBody(body->GetID(), JPH::EActivation::Activate);
```

**Custom contact listener:**

```cpp
// Access internal contact listener (advanced)
// Note: ContactListenerImpl is private; use set_body_callbacks() instead
```

**Profiling hooks (if `JPH_PROFILE_ENABLED`):**

```cpp
// Jolt's built-in profiler outputs to stdout
// Integrate with engine profiler by wrapping JPH_PROFILE macros
```

### See Also

- [PhysicsSystem.md](PhysicsSystem.md) — Abstract physics system API
- [ColliderSystem.md](ColliderSystem.md) — glTF collider authoring
- [FloatingOrigin.md](FloatingOrigin.md) — Large-world coordinate management
- Jolt Physics documentation: https://jrouwe.github.io/JoltPhysics/
