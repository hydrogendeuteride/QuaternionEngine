#include "jolt_physics_world.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <thread>

namespace Physics
{

// ============================================================================
// JoltGlobals implementation
// ============================================================================

std::mutex& JoltPhysicsWorld::JoltGlobals::mutex()
{
    static std::mutex m;
    return m;
}

int& JoltPhysicsWorld::JoltGlobals::ref_count()
{
    static int count = 0;
    return count;
}

JoltPhysicsWorld::JoltGlobals::JoltGlobals()
{
    std::scoped_lock lock(mutex());
    int& count = ref_count();
    if (count++ == 0)
    {
        JPH::RegisterDefaultAllocator();

        JPH::Trace = &JoltPhysicsWorld::trace_impl;
#ifdef JPH_ENABLE_ASSERTS
        JPH::AssertFailed = &JoltPhysicsWorld::assert_failed_impl;
#endif

        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
}

JoltPhysicsWorld::JoltGlobals::~JoltGlobals()
{
    std::scoped_lock lock(mutex());
    int& count = ref_count();
    if (--count == 0)
    {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

// ============================================================================
// BPLayerInterfaceImpl implementation
// ============================================================================

JoltPhysicsWorld::BPLayerInterfaceImpl::BPLayerInterfaceImpl()
{
    // Map all layers to broad phase layers
    for (uint32_t i = 0; i < Layer::Count; ++i)
    {
        // Static layer maps to non_moving, everything else to moving
        if (i == Layer::Static)
        {
            _object_to_broad_phase[i] = BroadPhaseLayers::non_moving;
        }
        else
        {
            _object_to_broad_phase[i] = BroadPhaseLayers::moving;
        }
    }
}

JPH::uint JoltPhysicsWorld::BPLayerInterfaceImpl::GetNumBroadPhaseLayers() const
{
    return BroadPhaseLayers::count;
}

JPH::BroadPhaseLayer JoltPhysicsWorld::BPLayerInterfaceImpl::GetBroadPhaseLayer(JPH::ObjectLayer layer) const
{
    if (layer < Layer::Count)
    {
        return _object_to_broad_phase[layer];
    }
    return BroadPhaseLayers::moving;
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
const char* JoltPhysicsWorld::BPLayerInterfaceImpl::GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const
{
    switch (layer.GetValue())
    {
        case 0: return "NON_MOVING";
        case 1: return "MOVING";
        default: return "INVALID";
    }
}
#endif

// ============================================================================
// ObjectVsBroadPhaseLayerFilterImpl implementation
// ============================================================================

bool JoltPhysicsWorld::ObjectVsBroadPhaseLayerFilterImpl::ShouldCollide(
    JPH::ObjectLayer layer1, JPH::BroadPhaseLayer layer2) const
{
    // Static objects only collide with moving objects
    if (layer1 == Layer::Static)
    {
        return layer2 == BroadPhaseLayers::moving;
    }
    // Everything else can collide with everything
    return true;
}

// ============================================================================
// ObjectLayerPairFilterImpl implementation
// ============================================================================

bool JoltPhysicsWorld::ObjectLayerPairFilterImpl::ShouldCollide(
    JPH::ObjectLayer layer1, JPH::ObjectLayer layer2) const
{
    // Static objects don't collide with each other
    if (layer1 == Layer::Static && layer2 == Layer::Static)
    {
        return false;
    }
    // Triggers only detect, no physical response (handled separately)
    // For now, allow all other collisions
    return true;
}

// ============================================================================
// JoltPhysicsWorld implementation
// ============================================================================

JoltPhysicsWorld::JoltPhysicsWorld()
{
    init(Config{});
}

JoltPhysicsWorld::JoltPhysicsWorld(const Config& config)
{
    init(config);
}

JoltPhysicsWorld::~JoltPhysicsWorld()
{
    // Bodies are automatically cleaned up by Jolt
}

void JoltPhysicsWorld::init(const Config& config)
{
    if (_initialized)
    {
        return;
    }

    _temp_allocator = std::make_unique<JPH::TempAllocatorImpl>(config.temp_allocator_size);
    _job_system = std::make_unique<JPH::JobSystemThreadPool>(
        JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, compute_worker_threads());

    _physics_system.Init(
        config.max_bodies,
        0,  // num_body_mutexes (0 = auto)
        config.max_body_pairs,
        config.max_contact_constraints,
        _broad_phase_layer_interface,
        _object_vs_broad_phase_layer_filter,
        _object_layer_pair_filter);

    _physics_system.SetGravity(JPH::Vec3(config.gravity.x, config.gravity.y, config.gravity.z));

    _initialized = true;
}

int JoltPhysicsWorld::compute_worker_threads()
{
    const uint32_t hw = std::max(1u, std::thread::hardware_concurrency());
    return static_cast<int>(hw > 1 ? hw - 1 : 1);
}

void JoltPhysicsWorld::trace_impl(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    std::fputc('\n', stderr);
    va_end(args);
}

#ifdef JPH_ENABLE_ASSERTS
bool JoltPhysicsWorld::assert_failed_impl(const char* expression, const char* message, const char* file, JPH::uint line)
{
    std::fprintf(stderr, "[Jolt][Assert] %s:%u: %s", file, static_cast<unsigned>(line), expression);
    if (message != nullptr)
    {
        std::fprintf(stderr, " (%s)", message);
    }
    std::fputc('\n', stderr);
    return false;  // Don't trigger breakpoint
}
#endif

// ============================================================================
// Simulation
// ============================================================================

void JoltPhysicsWorld::step(float dt)
{
    if (!_initialized)
    {
        return;
    }

    _physics_system.Update(dt, 1, _temp_allocator.get(), _job_system.get());
}

// ============================================================================
// Body creation / destruction
// ============================================================================

JPH::RefConst<JPH::Shape> JoltPhysicsWorld::create_jolt_shape(const CollisionShape& shape) const
{
    return std::visit([](const auto& s) -> JPH::RefConst<JPH::Shape>
    {
        using T = std::decay_t<decltype(s)>;

        if constexpr (std::is_same_v<T, BoxShape>)
        {
            return new JPH::BoxShape(JPH::Vec3(s.half_extents.x, s.half_extents.y, s.half_extents.z));
        }
        else if constexpr (std::is_same_v<T, SphereShape>)
        {
            return new JPH::SphereShape(s.radius);
        }
        else if constexpr (std::is_same_v<T, CapsuleShape>)
        {
            return new JPH::CapsuleShape(s.half_height, s.radius);
        }
        else if constexpr (std::is_same_v<T, CylinderShape>)
        {
            return new JPH::CylinderShape(s.half_height, s.radius);
        }
        else if constexpr (std::is_same_v<T, PlaneShape>)
        {
            // Use a very large thin box as a plane approximation
            return new JPH::BoxShape(JPH::Vec3(1000.0f, 0.01f, 1000.0f));
        }
        else
        {
            // Default fallback
            return new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f));
        }
    }, shape.shape);
}

JPH::EMotionType JoltPhysicsWorld::to_jolt_motion_type(MotionType type)
{
    switch (type)
    {
        case MotionType::Static:    return JPH::EMotionType::Static;
        case MotionType::Kinematic: return JPH::EMotionType::Kinematic;
        case MotionType::Dynamic:   return JPH::EMotionType::Dynamic;
        default:                    return JPH::EMotionType::Dynamic;
    }
}

JPH::ObjectLayer JoltPhysicsWorld::to_jolt_layer(uint32_t layer, MotionType motion)
{
    // If user specified a layer, use it (clamped to valid range)
    if (layer > 0 && layer < Layer::Count)
    {
        return static_cast<JPH::ObjectLayer>(layer);
    }

    // Otherwise, assign based on motion type
    switch (motion)
    {
        case MotionType::Static:    return Layer::Static;
        case MotionType::Kinematic: return Layer::Kinematic;
        case MotionType::Dynamic:   return Layer::Dynamic;
        default:                    return Layer::Default;
    }
}

BodyId JoltPhysicsWorld::create_body(const BodySettings& settings)
{
    if (!_initialized)
    {
        return BodyId{};
    }

    JPH::RefConst<JPH::Shape> jolt_shape = create_jolt_shape(settings.shape);

    JPH::BodyCreationSettings body_settings(
        jolt_shape,
        JPH::RVec3(settings.position.x, settings.position.y, settings.position.z),
        JPH::Quat(settings.rotation.x, settings.rotation.y, settings.rotation.z, settings.rotation.w),
        to_jolt_motion_type(settings.motion_type),
        to_jolt_layer(settings.layer, settings.motion_type));

    // Physical properties
    body_settings.mFriction = settings.friction;
    body_settings.mRestitution = settings.restitution;
    body_settings.mLinearDamping = settings.linear_damping;
    body_settings.mAngularDamping = settings.angular_damping;
    body_settings.mGravityFactor = settings.gravity_scale;

    // Sensor flag
    body_settings.mIsSensor = settings.is_sensor;

    // Activation
    JPH::EActivation activation = settings.start_active
        ? JPH::EActivation::Activate
        : JPH::EActivation::DontActivate;

    // For static bodies, never activate
    if (settings.motion_type == MotionType::Static)
    {
        activation = JPH::EActivation::DontActivate;
    }

    JPH::BodyID jolt_id = _physics_system.GetBodyInterface().CreateAndAddBody(body_settings, activation);

    if (jolt_id.IsInvalid())
    {
        return BodyId{};
    }

    return BodyId{jolt_id.GetIndexAndSequenceNumber()};
}

void JoltPhysicsWorld::destroy_body(BodyId id)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    JPH::BodyInterface& bi = _physics_system.GetBodyInterface();

    bi.RemoveBody(jolt_id);
    bi.DestroyBody(jolt_id);
}

bool JoltPhysicsWorld::is_body_valid(BodyId id) const
{
    if (!_initialized || !id.is_valid())
    {
        return false;
    }

    JPH::BodyID jolt_id(id.value);
    return _physics_system.GetBodyInterface().IsAdded(jolt_id);
}

// ============================================================================
// Body queries
// ============================================================================

BodyTransform JoltPhysicsWorld::get_transform(BodyId id) const
{
    BodyTransform result;

    if (!_initialized || !id.is_valid())
    {
        return result;
    }

    JPH::BodyID jolt_id(id.value);
    JPH::BodyLockRead lock(_physics_system.GetBodyLockInterface(), jolt_id);

    if (!lock.Succeeded())
    {
        return result;
    }

    const JPH::Body& body = lock.GetBody();
    const JPH::RVec3 p = body.GetPosition();
    const JPH::Quat q = body.GetRotation();

    result.position = glm::vec3(
        static_cast<float>(p.GetX()),
        static_cast<float>(p.GetY()),
        static_cast<float>(p.GetZ()));

    result.rotation = glm::quat(q.GetW(), q.GetX(), q.GetY(), q.GetZ());

    return result;
}

glm::vec3 JoltPhysicsWorld::get_position(BodyId id) const
{
    return get_transform(id).position;
}

glm::quat JoltPhysicsWorld::get_rotation(BodyId id) const
{
    return get_transform(id).rotation;
}

glm::mat4 JoltPhysicsWorld::get_transform_matrix(BodyId id) const
{
    return get_transform(id).to_matrix();
}

glm::vec3 JoltPhysicsWorld::get_linear_velocity(BodyId id) const
{
    if (!_initialized || !id.is_valid())
    {
        return glm::vec3(0.0f);
    }

    JPH::BodyID jolt_id(id.value);
    JPH::Vec3 v = _physics_system.GetBodyInterface().GetLinearVelocity(jolt_id);
    return glm::vec3(v.GetX(), v.GetY(), v.GetZ());
}

glm::vec3 JoltPhysicsWorld::get_angular_velocity(BodyId id) const
{
    if (!_initialized || !id.is_valid())
    {
        return glm::vec3(0.0f);
    }

    JPH::BodyID jolt_id(id.value);
    JPH::Vec3 v = _physics_system.GetBodyInterface().GetAngularVelocity(jolt_id);
    return glm::vec3(v.GetX(), v.GetY(), v.GetZ());
}

// ============================================================================
// Body manipulation
// ============================================================================

void JoltPhysicsWorld::set_position(BodyId id, const glm::vec3& position)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().SetPosition(
        jolt_id,
        JPH::RVec3(position.x, position.y, position.z),
        JPH::EActivation::Activate);
}

void JoltPhysicsWorld::set_rotation(BodyId id, const glm::quat& rotation)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().SetRotation(
        jolt_id,
        JPH::Quat(rotation.x, rotation.y, rotation.z, rotation.w),
        JPH::EActivation::Activate);
}

void JoltPhysicsWorld::set_transform(BodyId id, const glm::vec3& position, const glm::quat& rotation)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().SetPositionAndRotation(
        jolt_id,
        JPH::RVec3(position.x, position.y, position.z),
        JPH::Quat(rotation.x, rotation.y, rotation.z, rotation.w),
        JPH::EActivation::Activate);
}

void JoltPhysicsWorld::set_linear_velocity(BodyId id, const glm::vec3& velocity)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().SetLinearVelocity(
        jolt_id,
        JPH::Vec3(velocity.x, velocity.y, velocity.z));
}

void JoltPhysicsWorld::set_angular_velocity(BodyId id, const glm::vec3& velocity)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().SetAngularVelocity(
        jolt_id,
        JPH::Vec3(velocity.x, velocity.y, velocity.z));
}

void JoltPhysicsWorld::add_force(BodyId id, const glm::vec3& force)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().AddForce(
        jolt_id,
        JPH::Vec3(force.x, force.y, force.z));
}

void JoltPhysicsWorld::add_impulse(BodyId id, const glm::vec3& impulse)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().AddImpulse(
        jolt_id,
        JPH::Vec3(impulse.x, impulse.y, impulse.z));
}

void JoltPhysicsWorld::add_torque(BodyId id, const glm::vec3& torque)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().AddTorque(
        jolt_id,
        JPH::Vec3(torque.x, torque.y, torque.z));
}

void JoltPhysicsWorld::activate(BodyId id)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().ActivateBody(jolt_id);
}

void JoltPhysicsWorld::deactivate(BodyId id)
{
    if (!_initialized || !id.is_valid())
    {
        return;
    }

    JPH::BodyID jolt_id(id.value);
    _physics_system.GetBodyInterface().DeactivateBody(jolt_id);
}

bool JoltPhysicsWorld::is_active(BodyId id) const
{
    if (!_initialized || !id.is_valid())
    {
        return false;
    }

    JPH::BodyID jolt_id(id.value);
    return _physics_system.GetBodyInterface().IsActive(jolt_id);
}

// ============================================================================
// Raycasting
// ============================================================================

RayHit JoltPhysicsWorld::raycast(const glm::vec3& origin, const glm::vec3& direction, float max_distance) const
{
    RaycastOptions options;
    options.max_distance = max_distance;
    return raycast(origin, direction, options);
}

RayHit JoltPhysicsWorld::raycast(const glm::vec3& origin, const glm::vec3& direction, const RaycastOptions& options) const
{
    RayHit result;

    if (!_initialized)
    {
        return result;
    }

    glm::vec3 dir_norm = glm::length(direction) > 0.0f
        ? glm::normalize(direction)
        : glm::vec3(0.0f, -1.0f, 0.0f);

    JPH::RRayCast ray(
        JPH::RVec3(origin.x, origin.y, origin.z),
        JPH::Vec3(dir_norm.x, dir_norm.y, dir_norm.z) * options.max_distance);

    class RaycastFilter : public JPH::BroadPhaseLayerFilter, public JPH::ObjectLayerFilter
    {
    public:
        RaycastFilter(uint32_t layer_mask, BodyId ignore_body, bool include_sensors,
                      const JPH::PhysicsSystem& physics_system)
            : _layer_mask(layer_mask)
            , _ignore_body(ignore_body)
            , _include_sensors(include_sensors)
            , _physics_system(physics_system)
        {}

        // BroadPhaseLayerFilter
        bool ShouldCollide(JPH::BroadPhaseLayer layer) const override
        {
            // Always allow both moving and non-moving layers to be tested
            return true;
        }

        // ObjectLayerFilter
        bool ShouldCollide(JPH::ObjectLayer layer) const override
        {
            // Check if this layer is in the mask
            return (_layer_mask & (1u << layer)) != 0;
        }

    private:
        uint32_t _layer_mask;
        BodyId _ignore_body;
        bool _include_sensors;
        const JPH::PhysicsSystem& _physics_system;
    };

    // Custom body filter for ignore_body and sensor filtering
    class RaycastBodyFilter : public JPH::BodyFilter
    {
    public:
        RaycastBodyFilter(BodyId ignore_body, bool include_sensors, const JPH::BodyLockInterface& lock_interface)
            : _ignore_body(ignore_body)
            , _include_sensors(include_sensors)
            , _lock_interface(lock_interface)
        {}

        bool ShouldCollide(const JPH::BodyID& body_id) const override
        {
            // Check if this is the body to ignore
            if (_ignore_body.is_valid() && body_id.GetIndexAndSequenceNumber() == _ignore_body.value)
            {
                return false;
            }

            // Check sensor filtering
            if (!_include_sensors)
            {
                JPH::BodyLockRead lock(_lock_interface, body_id);
                if (lock.Succeeded())
                {
                    const JPH::Body& body = lock.GetBody();
                    if (body.IsSensor())
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        bool ShouldCollideLocked(const JPH::Body& body) const override
        {
            // Check if this is the body to ignore
            if (_ignore_body.is_valid() && body.GetID().GetIndexAndSequenceNumber() == _ignore_body.value)
            {
                return false;
            }

            // Check sensor filtering
            if (!_include_sensors && body.IsSensor())
            {
                return false;
            }

            return true;
        }

    private:
        BodyId _ignore_body;
        bool _include_sensors;
        const JPH::BodyLockInterface& _lock_interface;
    };

    RaycastFilter broad_phase_filter(options.layer_mask, options.ignore_body, options.include_sensors, _physics_system);
    RaycastBodyFilter body_filter(options.ignore_body, options.include_sensors, _physics_system.GetBodyLockInterface());

    JPH::RayCastSettings ray_settings;
    ray_settings.mBackFaceModeTriangles = options.backface_culling
        ? JPH::EBackFaceMode::IgnoreBackFaces
        : JPH::EBackFaceMode::CollideWithBackFaces;
    ray_settings.mBackFaceModeConvex = options.backface_culling
        ? JPH::EBackFaceMode::IgnoreBackFaces
        : JPH::EBackFaceMode::CollideWithBackFaces;

    JPH::RayCastResult hit;
    JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
    _physics_system.GetNarrowPhaseQuery().CastRay(ray, ray_settings, collector, broad_phase_filter, broad_phase_filter, body_filter);

    if (collector.HadHit())
    {
        const JPH::RayCastResult& hit = collector.mHit;

        result.hit = true;
        result.distance = hit.mFraction * options.max_distance;

        JPH::RVec3 hp = ray.GetPointOnRay(hit.mFraction);
        result.position = glm::vec3(
            static_cast<float>(hp.GetX()),
            static_cast<float>(hp.GetY()),
            static_cast<float>(hp.GetZ()));

        result.sub_shape_id = hit.mSubShapeID2.GetValue();

        {
            JPH::BodyLockRead lock(_physics_system.GetBodyLockInterface(), hit.mBodyID);
            if (lock.Succeeded())
            {
                const JPH::Body& body = lock.GetBody();
                const JPH::Vec3 n = body.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, hp);
                result.normal = glm::vec3(n.GetX(), n.GetY(), n.GetZ());
                result.layer = body.GetObjectLayer();
            }
        }
        result.body_id = BodyId{hit.mBodyID.GetIndexAndSequenceNumber()};
    }

    return result;
}

// ============================================================================
// World settings
// ============================================================================

void JoltPhysicsWorld::set_gravity(const glm::vec3& gravity)
{
    if (!_initialized)
    {
        return;
    }

    _physics_system.SetGravity(JPH::Vec3(gravity.x, gravity.y, gravity.z));
}

glm::vec3 JoltPhysicsWorld::get_gravity() const
{
    if (!_initialized)
    {
        return glm::vec3(0.0f, -9.81f, 0.0f);
    }

    JPH::Vec3 g = _physics_system.GetGravity();
    return glm::vec3(g.GetX(), g.GetY(), g.GetZ());
}

} // namespace Physics

#endif // VULKAN_ENGINE_USE_JOLT
