## Physics System: Abstraction Layer for Rigid Body Simulation

Backend-agnostic physics simulation framework with collision detection, raycasting, and contact events. Lives under `src/physics` and is accessed via `EngineContext::physics`. Currently implemented with Jolt Physics backend.

### Concepts

- **PhysicsWorld**: Abstract interface for physics simulation with bodies, queries, and collision management.
- **Bodies**: Rigid bodies with three motion types: Static (immovable), Kinematic (script-driven), Dynamic (fully simulated).
- **Shapes**: Collision geometry including primitives (box, sphere, capsule, cylinder, cone, plane), triangle meshes, and compounds.
- **Layers**: Collision filtering system with 16 configurable layers (e.g., Player, Enemy, Projectile, Trigger).
- **Queries**: Raycasts, shape sweeps, and overlap tests with filtering options.
- **Contacts**: Collision and trigger events with callbacks (Begin/Stay/End).
- **Joints**: Constraints connecting bodies (fixed, hinge, slider).
- **PhysicsContext**: Manages floating origin rebasing for large-world simulation.

### Key Types

#### `BodyId` — Strongly-Typed Body Identifier

```cpp
struct BodyId {
    uint32_t value{0};

    bool is_valid() const;
    explicit operator bool() const;
};
```

#### `BodySettings` — Body Creation Parameters

```cpp
struct BodySettings {
    CollisionShape shape;              // Collision geometry
    uint64_t user_data{0};             // Game entity ID or pointer

    // Transform
    glm::dvec3 position{0.0};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};

    // Motion
    MotionType motion_type = MotionType::Dynamic;

    // Physical properties
    float mass = 1.0f;                 // Dynamic bodies only
    float friction = 0.5f;
    float restitution = 0.0f;          // Bounciness (0-1)
    float linear_damping = 0.0f;
    float angular_damping = 0.05f;

    // Collision filtering
    uint32_t layer = Layer::Default;
    bool is_sensor = false;            // Trigger volume (no physical response)

    // Gravity
    float gravity_scale = 1.0f;        // 0 = no gravity, 2 = double gravity

    // Builder-style setters (chainable)
    BodySettings& set_shape(const CollisionShape& s);
    BodySettings& set_position(const glm::dvec3& p);
    BodySettings& set_dynamic();       // Set motion type
    BodySettings& set_sensor(bool s = true);
    // ... etc.
};
```

#### `CollisionShape` — Unified Shape Representation

```cpp
struct CollisionShape {
    ShapeVariant shape;  // Variant of BoxShape, SphereShape, etc.

    // Factory methods
    static CollisionShape Box(float hx, float hy, float hz);
    static CollisionShape Box(const glm::vec3& half_extents);
    static CollisionShape Sphere(float radius);
    static CollisionShape Capsule(float radius, float half_height);
    static CollisionShape Cylinder(float radius, float half_height);
    static CollisionShape Cone(float radius, float half_height, bool tip_up = true);
    static CollisionShape Plane(const glm::vec3& normal = {0, 1, 0});
    static CollisionShape TriangleMesh(std::shared_ptr<const TriangleMeshData> mesh);
    static CollisionShape Compound(CompoundShape compound);

    // Type queries
    bool is_box() const;
    bool is_sphere() const;
    bool is_compound() const;

    // Accessors
    const BoxShape* as_box() const;
    const CompoundShape* as_compound() const;
};
```

#### `CompoundShape` — Multi-Primitive Collider

```cpp
struct CompoundShape {
    std::vector<CompoundShapeChild> children;

    CompoundShape& add_box(const glm::vec3& half_extents,
                           const glm::vec3& position = {0.0f},
                           const glm::quat& rotation = {1.0f, 0.0f, 0.0f, 0.0f});
    CompoundShape& add_sphere(float radius, const glm::vec3& position = {0.0f});
    CompoundShape& add_capsule(float radius, float half_height,
                               const glm::vec3& position = {0.0f},
                               const glm::quat& rotation = {1.0f, 0.0f, 0.0f, 0.0f});
    CompoundShape& add_cone(float radius, float half_height, bool tip_up = true);
};
```

### API Surface

#### PhysicsWorld — Core Interface

**Simulation:**

| Method | Description |
|--------|-------------|
| `void step(float dt)` | Advance simulation by delta time |
| `void shift_origin(const glm::dvec3& delta)` | Translate all bodies for floating origin |
| `void shift_velocity_origin(const glm::dvec3& delta)` | Galilean velocity transform |

**Body Creation/Destruction:**

| Method | Description |
|--------|-------------|
| `BodyId create_body(const BodySettings& settings)` | Create body, returns ID |
| `void destroy_body(BodyId id)` | Destroy body |
| `BodyHandle create_body_handle(const BodySettings&)` | RAII wrapper (auto-destroys) |
| `bool is_body_valid(BodyId id) const` | Check if body exists |

**Body Queries:**

| Method | Description |
|--------|-------------|
| `BodyTransform get_transform(BodyId) const` | Get position + rotation |
| `glm::dvec3 get_position(BodyId) const` | Get position (double precision) |
| `glm::quat get_rotation(BodyId) const` | Get rotation |
| `glm::mat4 get_transform_matrix(BodyId) const` | Get 4x4 transform |
| `glm::vec3 get_linear_velocity(BodyId) const` | Get velocity |
| `glm::vec3 get_angular_velocity(BodyId) const` | Get angular velocity |
| `uint64_t get_user_data(BodyId) const` | Get user data |

**Body Manipulation:**

| Method | Description |
|--------|-------------|
| `void set_position(BodyId, const glm::dvec3&)` | Set position |
| `void set_rotation(BodyId, const glm::quat&)` | Set rotation |
| `void set_transform(BodyId, pos, rot)` | Set position + rotation |
| `void set_linear_velocity(BodyId, const glm::vec3&)` | Set velocity |
| `void add_force(BodyId, const glm::vec3&)` | Apply force (dynamic only) |
| `void add_impulse(BodyId, const glm::vec3&)` | Apply impulse (instant velocity change) |
| `void add_torque(BodyId, const glm::vec3&)` | Apply torque |
| `void activate(BodyId)` | Wake up sleeping body |
| `void deactivate(BodyId)` | Put body to sleep |

**Raycasting:**

| Method | Description |
|--------|-------------|
| `RayHit raycast(origin, direction, max_distance)` | Simple raycast |
| `RayHit raycast(origin, direction, RaycastOptions)` | Raycast with filtering |

**Shape Queries:**

| Method | Description |
|--------|-------------|
| `RayHit sweep(shape, origin, rotation, direction, SweepOptions)` | Cast shape along ray |
| `void overlap(shape, position, rotation, OverlapOptions, out_hits)` | Find overlapping bodies |

**Collision Filtering:**

| Method | Description |
|--------|-------------|
| `void set_layer_collision(layer_a, layer_b, should_collide)` | Configure layer collision matrix |
| `bool get_layer_collision(layer_a, layer_b) const` | Query layer collision setting |

**Contact Events:**

| Method | Description |
|--------|-------------|
| `void set_body_callbacks(BodyId, const BodyCallbacks&)` | Register collision/trigger callbacks |
| `void clear_body_callbacks(BodyId)` | Remove callbacks |

**Joints:**

| Method | Description |
|--------|-------------|
| `JointId create_fixed_joint(body_a, body_b)` | Create fixed constraint |
| `JointId create_hinge_joint(body_a, body_b, settings)` | Create hinge (door, wheel) |
| `JointId create_slider_joint(body_a, body_b, settings)` | Create slider (piston) |
| `void destroy_joint(JointId)` | Destroy joint |
| `bool is_joint_valid(JointId) const` | Check if joint exists |

**World Settings:**

| Method | Description |
|--------|-------------|
| `void set_gravity(const glm::vec3&)` | Set gravity vector |
| `glm::vec3 get_gravity() const` | Get gravity vector |

**Debug:**

| Method | Description |
|--------|-------------|
| `DebugStats debug_stats() const` | Get performance counters |
| `void for_each_debug_body(const DebugBodyFn&) const` | Iterate bodies for debug rendering |

### Quick Start — Creating Bodies

**Using BodySettings struct:**

```cpp
BodySettings settings;
settings.shape = CollisionShape::Sphere(0.5f);
settings.position = glm::dvec3(0.0, 10.0, 0.0);
settings.motion_type = MotionType::Dynamic;
settings.mass = 1.0f;
settings.layer = Physics::Layer::Dynamic;

BodyId ball = physics->create_body(settings);
```

**Using builder-style setters:**

```cpp
BodySettings settings;
settings.set_shape(CollisionShape::Box(1.0f, 1.0f, 1.0f))
        .set_position(0.0, 5.0, 0.0)
        .set_dynamic()
        .set_mass(10.0f)
        .set_friction(0.8f)
        .set_layer(Physics::Layer::Dynamic);

BodyId crate = physics->create_body(settings);
```

**Using BodyBuilder fluent API:**

```cpp
BodyId player = Physics::BodyBuilder(physics)
    .capsule(0.4f, 0.9f)           // radius, half_height
    .position(0.0, 2.0, 0.0)
    .dynamic_body()
    .mass(70.0f)
    .friction(0.0f)
    .angular_damping(1.0f)         // Prevent spinning
    .layer(Physics::Layer::Player)
    .build();
```

**Using RAII handle (auto-cleanup):**

```cpp
{
    BodyHandle temp = Physics::BodyBuilder(physics)
        .sphere(0.25f)
        .position(0.0, 5.0, 0.0)
        .dynamic_body()
        .build_handle();

    // Use temp...
} // Automatically destroyed here
```

### Quick Start — Compound Shapes

```cpp
// Character with capsule body + box feet collider
CompoundShape character;
character.add_capsule(0.4f, 0.9f, glm::vec3(0.0f, 1.0f, 0.0f));  // Torso
character.add_box(glm::vec3(0.3f, 0.1f, 0.2f), glm::vec3(0.0f, 0.0f, 0.0f));  // Feet

BodyId characterBody = Physics::BodyBuilder(physics)
    .compound(character)
    .position(0.0, 2.0, 0.0)
    .kinematic_body()
    .layer(Physics::Layer::Player)
    .build();
```

### Quick Start — Raycasting

**Simple raycast:**

```cpp
glm::dvec3 origin = camera_pos;
glm::vec3 direction = glm::normalize(camera_forward);
float max_distance = 100.0f;

RayHit hit = physics->raycast(origin, direction, max_distance);
if (hit.hit) {
    fmt::println("Hit body {} at distance {}", hit.body_id.value, hit.distance);
    fmt::println("Hit point: ({}, {}, {})", hit.position.x, hit.position.y, hit.position.z);
    fmt::println("Normal: ({}, {}, {})", hit.normal.x, hit.normal.y, hit.normal.z);
}
```

**Raycast with filtering:**

```cpp
RaycastOptions options;
options.max_distance = 50.0f;
options.layer_mask = (1u << Physics::Layer::Enemy) | (1u << Physics::Layer::Static);
options.ignore_body = player_body;
options.include_sensors = false;
options.backface_culling = true;

RayHit hit = physics->raycast(origin, direction, options);
```

### Quick Start — Contact Events

```cpp
// Register callbacks for a body
PhysicsWorld::BodyCallbacks callbacks;

callbacks.on_collision = [](const CollisionEvent& e) {
    if (e.type == ContactEventType::Begin) {
        fmt::println("Collision started: {} hit {}", e.self.value, e.other.value);
        fmt::println("Penetration depth: {}", e.penetration_depth);
    }
};

callbacks.on_trigger = [](const TriggerEvent& e) {
    if (e.type == ContactEventType::Begin) {
        fmt::println("Trigger entered: {} entered {}", e.other.value, e.self.value);
    }
};

physics->set_body_callbacks(sensor_body, callbacks);
```

### Quick Start — Collision Layers

```cpp
// Setup layer collision matrix
physics->set_layer_collision(Physics::Layer::Player, Physics::Layer::Enemy, true);
physics->set_layer_collision(Physics::Layer::Player, Physics::Layer::Projectile, false);
physics->set_layer_collision(Physics::Layer::Enemy, Physics::Layer::Projectile, true);

// Create bodies with specific layers
BodyId enemy = Physics::BodyBuilder(physics)
    .sphere(0.5f)
    .position(5.0, 1.0, 0.0)
    .dynamic_body()
    .layer(Physics::Layer::Enemy)
    .build();

BodyId projectile = Physics::BodyBuilder(physics)
    .sphere(0.1f)
    .position(0.0, 1.0, 0.0)
    .dynamic_body()
    .layer(Physics::Layer::Projectile)
    .build();
```

### Quick Start — Joints

**Fixed joint (attach objects rigidly):**

```cpp
JointId joint = physics->create_fixed_joint(body_a, body_b);
```

**Hinge joint (door, wheel):**

```cpp
HingeJointSettings hinge;
hinge.anchor = glm::vec3(0.0f, 0.0f, 0.0f);
hinge.axis = glm::vec3(0.0f, 1.0f, 0.0f);  // Y-axis rotation
hinge.enable_limits = true;
hinge.limit_min = -glm::radians(90.0f);
hinge.limit_max = glm::radians(90.0f);

JointId door_hinge = physics->create_hinge_joint(door_body, frame_body, hinge);
```

**Slider joint (piston):**

```cpp
SliderJointSettings slider;
slider.anchor = glm::vec3(0.0f, 0.0f, 0.0f);
slider.axis = glm::vec3(0.0f, 1.0f, 0.0f);  // Y-axis motion
slider.enable_limits = true;
slider.limit_min = 0.0f;
slider.limit_max = 2.0f;

JointId piston = physics->create_slider_joint(piston_body, base_body, slider);
```

### Integration With Engine

**Initialization (in Engine::init):**

```cpp
// Physics world is typically created by the engine
auto jolt_world = std::make_unique<Physics::JoltPhysicsWorld>();
ctx->physics = jolt_world.get();

// Create physics context for floating origin management
ctx->physics_context = std::make_unique<Physics::PhysicsContext>(*ctx->physics);
```

**Update loop (in Engine::update):**

```cpp
// Step physics simulation
float fixed_dt = 1.0f / 60.0f;  // 60 Hz physics
physics->step(fixed_dt);

// Sync physics bodies to render transforms
for (auto& entity : entities) {
    if (entity.physics_body.is_valid()) {
        entity.transform = physics->get_transform_matrix(entity.physics_body);
    }
}
```

**Floating origin rebasing:**

```cpp
// In game loop, when camera moves far from origin
if (glm::length(camera_pos) > 1000.0) {
    glm::dvec3 shift = -camera_pos;
    physics->shift_origin(shift);
    physics_context->set_origin_world(physics_context->origin_world() + shift);
}

// Or use automatic rebasing in PhysicsContext
physics_context->maybe_rebase_origin_to_body(player_body.value, 1000.0, 100.0);
```

### PhysicsContext — Floating Origin Management

```cpp
// Track physics coordinate system origin
physics_context->set_origin_world(WorldVec3{1000.0, 0.0, 500.0});

// Automatic rebasing when body exceeds threshold
bool rebased = physics_context->maybe_rebase_origin_to_body(
    player_body.value,  // Body to track
    1000.0,             // Rebase when |local_pos| > 1000m
    100.0               // Snap to 100m grid
);

// Velocity origin for high-speed scenarios (orbital mechanics, etc.)
physics_context->set_velocity_origin_world(spacecraft_velocity);
physics_context->maybe_rebase_velocity_to_body(spacecraft_body.value, 100.0);
```

### Collision Layers

Predefined layers in `Physics::Layer`:

| Layer | Value | Typical Use |
|-------|-------|-------------|
| `Default` | 0 | Uncategorized objects |
| `Static` | 1 | Immovable environment |
| `Dynamic` | 2 | Movable objects |
| `Kinematic` | 3 | Script-controlled objects |
| `Player` | 4 | Player character |
| `Enemy` | 5 | Enemy NPCs |
| `Projectile` | 6 | Bullets, arrows |
| `Trigger` | 7 | Trigger volumes |
| `Debris` | 8 | Small debris |

Custom layers can be defined up to `Layer::Count` (16 total).

### File Structure

```
src/physics/
├── physics_world.h             # Abstract PhysicsWorld interface
├── physics_context.h/.cpp      # Floating origin management
├── physics_body.h/.cpp         # BodyId, BodyHandle, query types
├── body_settings.h             # BodySettings, MotionType, Layer constants
├── collision_shape.h           # Shape types and CompoundShape API
├── collider_asset.h/.cpp       # Collider scaling utilities
├── collider_mesh_instance.h    # Triangle mesh collider metadata
├── gltf_collider_parser.h/.cpp # glTF collider marker parsing
└── jolt/                       # Jolt Physics backend (see JoltBackend.md)
    ├── jolt_physics_world.h/.cpp
    └── jolt_query_filters.h

src/core/
└── engine_context.h            # Contains PhysicsWorld* and PhysicsContext*

src/scene/
├── vk_scene.h/.cpp             # Collider sync (kinematic bodies track node transforms)
└── vk_loader.h/.cpp            # glTF collider loading
```

### Notes & Limits

- **Double precision positions**: `glm::dvec3` for world-space positions, `glm::vec3` for velocities/forces.
- **Collision matrix**: 16 layers max, each layer can collide with any subset of other layers.
- **Sensor bodies**: `is_sensor = true` generates trigger events but no physical response.
- **RAII handles**: `BodyHandle` auto-destroys body on destruction; use `release()` to prevent cleanup.
- **User data**: 64-bit field for game entity pointers or IDs (cast to `uintptr_t` for pointers).
- **Contact events**: Dispatched during `step()`, callbacks must be thread-safe if using async simulation.
- **Triangle meshes**: Typically static-only (backend-dependent); use convex decomposition for dynamic meshes.
- **Joints**: Destroyed automatically when either connected body is destroyed.

### Debugging

**Debug stats:**

```cpp
auto stats = physics->debug_stats();
fmt::println("Physics step: {:.2f}ms (avg {:.2f}ms)",
             stats.last_step_ms, stats.avg_step_ms);
fmt::println("Bodies: {} ({} active)", stats.body_count, stats.active_body_count);
fmt::println("Contacts: {}", stats.contact_event_count);
```

**Debug rendering:**

```cpp
physics->for_each_debug_body([&](const PhysicsWorld::DebugBodyView& body) {
    glm::dvec3 pos = body.position;
    glm::quat rot = body.rotation;

    // Render shape based on body.shape type
    if (body.shape.is_box()) {
        const auto* box = body.shape.as_box();
        draw_debug_box(pos, rot, box->half_extents);
    }
    else if (body.shape.is_sphere()) {
        const auto* sphere = body.shape.as_sphere();
        draw_debug_sphere(pos, sphere->radius);
    }

    // Color by layer or motion type
    glm::vec4 color = body.is_sensor ? glm::vec4(0, 1, 0, 0.5f)
                                     : glm::vec4(1, 1, 1, 1);
});
```

**Validation layers (Jolt backend):**

- Compile with `JPH_ENABLE_ASSERTS` for assertion checks.
- Enable validation in `JoltPhysicsWorld::Config` for detailed error messages.
- Use `JPH_PROFILE_ENABLED` for performance profiling.

### See Also

- [JoltBackend.md](JoltBackend.md) — Jolt Physics backend implementation details
- [ColliderSystem.md](ColliderSystem.md) — glTF-based collider authoring workflow
- [FloatingOrigin.md](FloatingOrigin.md) — Large-world coordinate system rebasing
