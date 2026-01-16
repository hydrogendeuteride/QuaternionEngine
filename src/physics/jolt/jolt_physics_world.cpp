#include "jolt_physics_world.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <thread>

namespace Physics
{
    // ============================================================================
    // Jolt helpers
    // ============================================================================

    glm::vec3 JoltPhysicsWorld::safe_normalize(const glm::vec3 &v, const glm::vec3 &fallback)
    {
        const float len = glm::length(v);
        if (len > 0.0f)
        {
            return v / len;
        }
        return fallback;
    }

    void JoltPhysicsWorld::compute_basis(const glm::vec3 &axis, glm::vec3 &out_normal)
    {
        // Pick a normal that is guaranteed not to be parallel with axis.
        const glm::vec3 a = safe_normalize(axis, glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::vec3 ref = std::abs(a.y) < 0.99f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        out_normal = glm::normalize(glm::cross(ref, a));
    }

    // ============================================================================
    // Contact listener
    // ============================================================================

    class JoltPhysicsWorld::ContactListenerImpl final : public JPH::ContactListener
    {
    public:
        explicit ContactListenerImpl(JoltPhysicsWorld *world) : _world(world)
        {
        }

        void OnContactAdded(const JPH::Body &body1, const JPH::Body &body2, const JPH::ContactManifold &manifold,
                            JPH::ContactSettings &) override
        {
            queue_contact_event(ContactEventType::Begin, body1, body2, manifold);
        }

        void OnContactPersisted(const JPH::Body &body1, const JPH::Body &body2, const JPH::ContactManifold &manifold,
                                JPH::ContactSettings &) override
        {
            queue_contact_event(ContactEventType::Stay, body1, body2, manifold);
        }

        void OnContactRemoved(const JPH::SubShapeIDPair &pair) override
        {
            PairCacheEntry cached;
            bool found = false; {
                std::scoped_lock lock(_world->_pair_cache_mutex);
                auto it = _world->_pair_cache.find(pair);
                if (it != _world->_pair_cache.end())
                {
                    cached = it->second;
                    _world->_pair_cache.erase(it);
                    found = true;
                }
            }

            if (!found)
            {
                cached.is_trigger = false;
            }

            const BodyId body1_id{pair.GetBody1ID().GetIndexAndSequenceNumber()};
            const BodyId body2_id{pair.GetBody2ID().GetIndexAndSequenceNumber()};

            if (cached.is_trigger)
            {
                TriggerEvent e1;
                e1.type = ContactEventType::End;
                e1.self = body1_id;
                e1.other = body2_id;
                e1.self_sub_shape_id = pair.GetSubShapeID1().GetValue();
                e1.other_sub_shape_id = pair.GetSubShapeID2().GetValue();
                e1.self_is_sensor = cached.is_sensor1;
                e1.other_is_sensor = cached.is_sensor2;
                e1.self_layer = cached.layer1;
                e1.other_layer = cached.layer2;
                e1.self_user_data = cached.user_data1;
                e1.other_user_data = cached.user_data2;

                TriggerEvent e2 = e1;
                e2.self = body2_id;
                e2.other = body1_id;
                e2.self_sub_shape_id = pair.GetSubShapeID2().GetValue();
                e2.other_sub_shape_id = pair.GetSubShapeID1().GetValue();
                e2.self_is_sensor = cached.is_sensor2;
                e2.other_is_sensor = cached.is_sensor1;
                e2.self_layer = cached.layer2;
                e2.other_layer = cached.layer1;
                e2.self_user_data = cached.user_data2;
                e2.other_user_data = cached.user_data1;

                std::scoped_lock lock(_world->_events_mutex);
                _world->_queued_events.push_back(ContactEventRecord{true, CollisionEvent{}, e1});
                _world->_queued_events.push_back(ContactEventRecord{true, CollisionEvent{}, e2});
            }
            else
            {
                CollisionEvent e1;
                e1.type = ContactEventType::End;
                e1.self = body1_id;
                e1.other = body2_id;
                e1.self_sub_shape_id = pair.GetSubShapeID1().GetValue();
                e1.other_sub_shape_id = pair.GetSubShapeID2().GetValue();
                e1.self_layer = cached.layer1;
                e1.other_layer = cached.layer2;
                e1.self_user_data = cached.user_data1;
                e1.other_user_data = cached.user_data2;

                CollisionEvent e2 = e1;
                e2.self = body2_id;
                e2.other = body1_id;
                e2.self_sub_shape_id = pair.GetSubShapeID2().GetValue();
                e2.other_sub_shape_id = pair.GetSubShapeID1().GetValue();
                e2.self_layer = cached.layer2;
                e2.other_layer = cached.layer1;
                e2.self_user_data = cached.user_data2;
                e2.other_user_data = cached.user_data1;

                std::scoped_lock lock(_world->_events_mutex);
                _world->_queued_events.push_back(ContactEventRecord{false, e1, TriggerEvent{}});
                _world->_queued_events.push_back(ContactEventRecord{false, e2, TriggerEvent{}});
            }
        }

    private:
        template<typename TVec>
        static glm::vec3 to_glm_vec3(const TVec &v)
        {
            return glm::vec3(static_cast<float>(v.GetX()), static_cast<float>(v.GetY()), static_cast<float>(v.GetZ()));
        }

        void queue_contact_event(ContactEventType type, const JPH::Body &body1, const JPH::Body &body2,
                                 const JPH::ContactManifold &manifold)
        {
            const bool is_trigger = body1.IsSensor() || body2.IsSensor();

            // Cache info for OnContactRemoved (can't access bodies there)
            const JPH::SubShapeIDPair pair(body1.GetID(), manifold.mSubShapeID1, body2.GetID(),
                                           manifold.mSubShapeID2); {
                PairCacheEntry entry;
                entry.is_trigger = is_trigger;
                entry.user_data1 = body1.GetUserData();
                entry.user_data2 = body2.GetUserData();
                entry.layer1 = body1.GetObjectLayer();
                entry.layer2 = body2.GetObjectLayer();
                entry.is_sensor1 = body1.IsSensor();
                entry.is_sensor2 = body2.IsSensor();

                std::scoped_lock lock(_world->_pair_cache_mutex);
                _world->_pair_cache[pair] = entry;
            }

            glm::vec3 p1{0.0f};
            glm::vec3 p2{0.0f};
            if (!manifold.mRelativeContactPointsOn1.empty())
            {
                p1 = to_glm_vec3(manifold.GetWorldSpaceContactPointOn1(0));
                p2 = to_glm_vec3(manifold.GetWorldSpaceContactPointOn2(0));
            }
            else
            {
                p1 = to_glm_vec3(body1.GetPosition());
                p2 = to_glm_vec3(body2.GetPosition());
            }

            if (is_trigger)
            {
                TriggerEvent e1;
                e1.type = type;
                e1.self = BodyId{body1.GetID().GetIndexAndSequenceNumber()};
                e1.other = BodyId{body2.GetID().GetIndexAndSequenceNumber()};
                e1.self_sub_shape_id = manifold.mSubShapeID1.GetValue();
                e1.other_sub_shape_id = manifold.mSubShapeID2.GetValue();
                e1.point = p1;
                e1.self_is_sensor = body1.IsSensor();
                e1.other_is_sensor = body2.IsSensor();
                e1.self_layer = body1.GetObjectLayer();
                e1.other_layer = body2.GetObjectLayer();
                e1.self_user_data = body1.GetUserData();
                e1.other_user_data = body2.GetUserData();

                TriggerEvent e2 = e1;
                e2.self = e1.other;
                e2.other = e1.self;
                e2.self_sub_shape_id = e1.other_sub_shape_id;
                e2.other_sub_shape_id = e1.self_sub_shape_id;
                e2.point = p2;
                e2.self_is_sensor = body2.IsSensor();
                e2.other_is_sensor = body1.IsSensor();
                e2.self_layer = body2.GetObjectLayer();
                e2.other_layer = body1.GetObjectLayer();
                e2.self_user_data = body2.GetUserData();
                e2.other_user_data = body1.GetUserData();

                std::scoped_lock lock(_world->_events_mutex);
                _world->_queued_events.push_back(ContactEventRecord{true, CollisionEvent{}, e1});
                _world->_queued_events.push_back(ContactEventRecord{true, CollisionEvent{}, e2});
            }
            else
            {
                CollisionEvent e1;
                e1.type = type;
                e1.self = BodyId{body1.GetID().GetIndexAndSequenceNumber()};
                e1.other = BodyId{body2.GetID().GetIndexAndSequenceNumber()};
                e1.self_sub_shape_id = manifold.mSubShapeID1.GetValue();
                e1.other_sub_shape_id = manifold.mSubShapeID2.GetValue();
                e1.point = p1;
                e1.normal = to_glm_vec3(manifold.mWorldSpaceNormal);
                e1.penetration_depth = manifold.mPenetrationDepth;
                e1.self_layer = body1.GetObjectLayer();
                e1.other_layer = body2.GetObjectLayer();
                e1.self_user_data = body1.GetUserData();
                e1.other_user_data = body2.GetUserData();

                CollisionEvent e2 = e1;
                e2.self = e1.other;
                e2.other = e1.self;
                e2.self_sub_shape_id = e1.other_sub_shape_id;
                e2.other_sub_shape_id = e1.self_sub_shape_id;
                e2.point = p2;
                e2.normal = -e1.normal;
                e2.self_layer = body2.GetObjectLayer();
                e2.other_layer = body1.GetObjectLayer();
                e2.self_user_data = body2.GetUserData();
                e2.other_user_data = body1.GetUserData();

                std::scoped_lock lock(_world->_events_mutex);
                _world->_queued_events.push_back(ContactEventRecord{false, e1, TriggerEvent{}});
                _world->_queued_events.push_back(ContactEventRecord{false, e2, TriggerEvent{}});
            }
        }

        JoltPhysicsWorld *_world{nullptr};
    };

    // ============================================================================
    // JoltGlobals implementation
    // ============================================================================

    std::mutex &JoltPhysicsWorld::JoltGlobals::mutex()
    {
        static std::mutex m;
        return m;
    }

    int &JoltPhysicsWorld::JoltGlobals::ref_count()
    {
        static int count = 0;
        return count;
    }

    JoltPhysicsWorld::JoltGlobals::JoltGlobals()
    {
        std::scoped_lock lock(mutex());
        int &count = ref_count();
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
        int &count = ref_count();
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
    const char *JoltPhysicsWorld::BPLayerInterfaceImpl::GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const
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
        if (_layer_collision_mask != nullptr)
        {
            const uint32_t l1 = static_cast<uint32_t>(layer1);
            const uint32_t l2 = static_cast<uint32_t>(layer2);
            if (l1 < _mask_count && l2 < _mask_count)
            {
                return (_layer_collision_mask[l1] & (1u << l2)) != 0;
            }
        }

        // Fallback behavior: allow everything except static-static
        if (layer1 == Layer::Static && layer2 == Layer::Static)
        {
            return false;
        }
        return true;
    }

    // ============================================================================
    // JoltPhysicsWorld implementation
    // ============================================================================

    JoltPhysicsWorld::JoltPhysicsWorld()
    {
        init(Config{});
    }

    JoltPhysicsWorld::JoltPhysicsWorld(const Config &config)
    {
        init(config);
    }

    JoltPhysicsWorld::~JoltPhysicsWorld()
    {
        // Bodies are automatically cleaned up by Jolt
    }

    void JoltPhysicsWorld::init(const Config &config)
    {
        if (_initialized)
        {
            return;
        }

        const uint32_t all_layers_mask = Layer::Count >= 32
                                             ? 0xFFFFFFFFu
                                             : ((1u << Layer::Count) - 1u);

        for (uint32_t i = 0; i < Layer::Count; ++i)
        {
            _layer_collision_mask[i] = all_layers_mask;
        }
        // Default: don't collide static with static
        _layer_collision_mask[Layer::Static] &= ~(1u << Layer::Static);

        _object_layer_pair_filter.set_layer_collision_mask(_layer_collision_mask, Layer::Count);

        _temp_allocator = std::make_unique<JPH::TempAllocatorImpl>(config.temp_allocator_size);
        _job_system = std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, compute_worker_threads());

        _physics_system.Init(
            config.max_bodies,
            0, // num_body_mutexes (0 = auto)
            config.max_body_pairs,
            config.max_contact_constraints,
            _broad_phase_layer_interface,
            _object_vs_broad_phase_layer_filter,
            _object_layer_pair_filter);

        _physics_system.SetGravity(JPH::Vec3(config.gravity.x, config.gravity.y, config.gravity.z));

        _contact_listener = std::make_unique<ContactListenerImpl>(this);
        _physics_system.SetContactListener(_contact_listener.get());

        _initialized = true;
    }

    int JoltPhysicsWorld::compute_worker_threads()
    {
        const uint32_t hw = std::max(1u, std::thread::hardware_concurrency());
        return static_cast<int>(hw > 1 ? hw - 1 : 1);
    }

    void JoltPhysicsWorld::trace_impl(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        std::vfprintf(stderr, fmt, args);
        std::fputc('\n', stderr);
        va_end(args);
    }

#ifdef JPH_ENABLE_ASSERTS
    bool JoltPhysicsWorld::assert_failed_impl(const char *expression, const char *message, const char *file,
                                              JPH::uint line)
    {
        std::fprintf(stderr, "[Jolt][Assert] %s:%u: %s", file, static_cast<unsigned>(line), expression);
        if (message != nullptr)
        {
            std::fprintf(stderr, " (%s)", message);
        }
        std::fputc('\n', stderr);
        return false; // Don't trigger breakpoint
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
        } {
            std::scoped_lock lock(_layer_mutex);
            _physics_system.Update(dt, 1, _temp_allocator.get(), _job_system.get());
        }

        dispatch_contact_events();
    }

    void JoltPhysicsWorld::dispatch_contact_events()
    {
        std::vector<ContactEventRecord> events; {
            std::scoped_lock lock(_events_mutex);
            events.swap(_queued_events);
        }

        for (const ContactEventRecord &e: events)
        {
            if (e.is_trigger)
            {
                std::function<void(const TriggerEvent &)> cb;

                {
                    std::scoped_lock lock(_callbacks_mutex);
                    auto it = _callbacks.find(e.trigger.self.value);
                    if (it != _callbacks.end())
                    {
                        cb = it->second.on_trigger;
                    }
                }

                if (cb)
                {
                    cb(e.trigger);
                }
            }
            else
            {
                std::function<void(const CollisionEvent &)> cb;

                {
                    std::scoped_lock lock(_callbacks_mutex);
                    auto it = _callbacks.find(e.collision.self.value);
                    if (it != _callbacks.end())
                    {
                        cb = it->second.on_collision;
                    }
                }

                if (cb)
                {
                    cb(e.collision);
                }
            }
        }
    }

    // ============================================================================
    // Body creation / destruction
    // ============================================================================

    JPH::RefConst<JPH::Shape> JoltPhysicsWorld::create_jolt_shape(const CollisionShape &shape) const
    {
        return std::visit([](const auto &s) -> JPH::RefConst<JPH::Shape> {
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
            case MotionType::Static: return JPH::EMotionType::Static;
            case MotionType::Kinematic: return JPH::EMotionType::Kinematic;
            case MotionType::Dynamic: return JPH::EMotionType::Dynamic;
            default: return JPH::EMotionType::Dynamic;
        }
    }

    JPH::ObjectLayer JoltPhysicsWorld::to_jolt_layer(uint32_t layer, MotionType motion)
    {
        if (layer > 0 && layer < Layer::Count)
        {
            return static_cast<JPH::ObjectLayer>(layer);
        }

        // Otherwise, assign based on motion type
        switch (motion)
        {
            case MotionType::Static: return Layer::Static;
            case MotionType::Kinematic: return Layer::Kinematic;
            case MotionType::Dynamic: return Layer::Dynamic;
            default: return Layer::Default;
        }
    }

    BodyId JoltPhysicsWorld::create_body(const BodySettings &settings)
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

        body_settings.mUserData = settings.user_data;

        // Physical properties
        body_settings.mFriction = settings.friction;
        body_settings.mRestitution = settings.restitution;
        body_settings.mLinearDamping = settings.linear_damping;
        body_settings.mAngularDamping = settings.angular_damping;
        body_settings.mGravityFactor = settings.gravity_scale;
        body_settings.mAllowSleeping = settings.allow_sleeping;

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
        } {
            std::scoped_lock lock(_callbacks_mutex);
            _callbacks.erase(id.value);
        }

        JPH::BodyID jolt_id(id.value);
        JPH::BodyInterface &bi = _physics_system.GetBodyInterface();

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

        const JPH::Body &body = lock.GetBody();
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

    uint64_t JoltPhysicsWorld::get_user_data(BodyId id) const
    {
        if (!_initialized || !id.is_valid())
        {
            return 0;
        }

        JPH::BodyID jolt_id(id.value);
        return _physics_system.GetBodyInterface().GetUserData(jolt_id);
    }

    // ============================================================================
    // Body manipulation
    // ============================================================================

    void JoltPhysicsWorld::set_position(BodyId id, const glm::vec3 &position)
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

    void JoltPhysicsWorld::set_rotation(BodyId id, const glm::quat &rotation)
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

    void JoltPhysicsWorld::set_transform(BodyId id, const glm::vec3 &position, const glm::quat &rotation)
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

    void JoltPhysicsWorld::set_linear_velocity(BodyId id, const glm::vec3 &velocity)
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

    void JoltPhysicsWorld::set_angular_velocity(BodyId id, const glm::vec3 &velocity)
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

    void JoltPhysicsWorld::set_user_data(BodyId id, uint64_t user_data)
    {
        if (!_initialized || !id.is_valid())
        {
            return;
        }

        JPH::BodyID jolt_id(id.value);
        _physics_system.GetBodyInterface().SetUserData(jolt_id, user_data);
    }

    void JoltPhysicsWorld::add_force(BodyId id, const glm::vec3 &force)
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

    void JoltPhysicsWorld::add_impulse(BodyId id, const glm::vec3 &impulse)
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

    void JoltPhysicsWorld::add_torque(BodyId id, const glm::vec3 &torque)
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

    RayHit JoltPhysicsWorld::raycast(const glm::vec3 &origin, const glm::vec3 &direction, float max_distance) const
    {
        RaycastOptions options;
        options.max_distance = max_distance;
        return raycast(origin, direction, options);
    }

    RayHit JoltPhysicsWorld::raycast(const glm::vec3 &origin, const glm::vec3 &direction,
                                     const RaycastOptions &options) const
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
                          const JPH::PhysicsSystem &physics_system)
                : _layer_mask(layer_mask)
                  , _ignore_body(ignore_body)
                  , _include_sensors(include_sensors)
                  , _physics_system(physics_system)
            {
            }

            // BroadPhaseLayerFilter
            bool ShouldCollide(JPH::BroadPhaseLayer) const override
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
            const JPH::PhysicsSystem &_physics_system;
        };

        // Custom body filter for ignore_body and sensor filtering
        class RaycastBodyFilter : public JPH::BodyFilter
        {
        public:
            RaycastBodyFilter(BodyId ignore_body, bool include_sensors, const JPH::BodyLockInterface &lock_interface)
                : _ignore_body(ignore_body)
                  , _include_sensors(include_sensors)
                  , _lock_interface(lock_interface)
            {
            }

            bool ShouldCollide(const JPH::BodyID &body_id) const override
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
                        const JPH::Body &body = lock.GetBody();
                        if (body.IsSensor())
                        {
                            return false;
                        }
                    }
                }

                return true;
            }

            bool ShouldCollideLocked(const JPH::Body &body) const override
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
            const JPH::BodyLockInterface &_lock_interface;
        };

        RaycastFilter broad_phase_filter(options.layer_mask, options.ignore_body, options.include_sensors,
                                         _physics_system);
        RaycastBodyFilter body_filter(options.ignore_body, options.include_sensors,
                                      _physics_system.GetBodyLockInterface());

        JPH::RayCastSettings ray_settings;
        ray_settings.mBackFaceModeTriangles = options.backface_culling
                                                  ? JPH::EBackFaceMode::IgnoreBackFaces
                                                  : JPH::EBackFaceMode::CollideWithBackFaces;
        ray_settings.mBackFaceModeConvex = options.backface_culling
                                               ? JPH::EBackFaceMode::IgnoreBackFaces
                                               : JPH::EBackFaceMode::CollideWithBackFaces;

        JPH::RayCastResult hit;
        JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
        _physics_system.GetNarrowPhaseQuery().CastRay(ray, ray_settings, collector, broad_phase_filter,
                                                      broad_phase_filter, body_filter);

        if (collector.HadHit())
        {
            const JPH::RayCastResult &hit = collector.mHit;

            result.hit = true;
            result.distance = hit.mFraction * options.max_distance;

            JPH::RVec3 hp = ray.GetPointOnRay(hit.mFraction);
            result.position = glm::vec3(
                static_cast<float>(hp.GetX()),
                static_cast<float>(hp.GetY()),
                static_cast<float>(hp.GetZ()));

            result.sub_shape_id = hit.mSubShapeID2.GetValue(); {
                JPH::BodyLockRead lock(_physics_system.GetBodyLockInterface(), hit.mBodyID);
                if (lock.Succeeded())
                {
                    const JPH::Body &body = lock.GetBody();
                    const JPH::Vec3 n = body.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, hp);
                    result.normal = glm::vec3(n.GetX(), n.GetY(), n.GetZ());
                    result.layer = body.GetObjectLayer();
                }
            }
            result.body_id = BodyId{hit.mBodyID.GetIndexAndSequenceNumber()};
            result.user_data = get_user_data(result.body_id);
        }

        return result;
    }

    // ============================================================================
    // Shape queries
    // ============================================================================

    RayHit JoltPhysicsWorld::sweep(const CollisionShape &shape,
                                   const glm::vec3 &origin,
                                   const glm::quat &rotation,
                                   const glm::vec3 &direction,
                                   const SweepOptions &options) const
    {
        RayHit result;

        if (!_initialized)
        {
            return result;
        }

        const glm::vec3 dir_norm = safe_normalize(direction, glm::vec3(0.0f, -1.0f, 0.0f));

        JPH::RefConst<JPH::Shape> jolt_shape = create_jolt_shape(shape);
        const JPH::Mat44 shape_com = JPH::Mat44::sTranslation(jolt_shape->GetCenterOfMass());

        const JPH::RMat44 com_start =
                JPH::RMat44::sTranslation(JPH::RVec3(origin.x, origin.y, origin.z)) *
                JPH::Mat44::sRotation(JPH::Quat(rotation.x, rotation.y, rotation.z, rotation.w)) *
                shape_com;

        const JPH::Vec3 cast_dir = JPH::Vec3(dir_norm.x, dir_norm.y, dir_norm.z) * options.max_distance;
        const JPH::RShapeCast shape_cast(jolt_shape.GetPtr(), JPH::Vec3::sOne(), com_start, cast_dir);

        class ShapeQueryFilter : public JPH::BroadPhaseLayerFilter, public JPH::ObjectLayerFilter
        {
        public:
            explicit ShapeQueryFilter(uint32_t layer_mask) : _layer_mask(layer_mask)
            {
            }

            bool ShouldCollide(JPH::BroadPhaseLayer) const override { return true; }

            bool ShouldCollide(JPH::ObjectLayer layer) const override
            {
                return (_layer_mask & (1u << layer)) != 0;
            }

        private:
            uint32_t _layer_mask;
        };

        class ShapeQueryBodyFilter : public JPH::BodyFilter
        {
        public:
            ShapeQueryBodyFilter(BodyId ignore_body, bool include_sensors, const JPH::BodyLockInterface &lock_interface)
                : _ignore_body(ignore_body)
                  , _include_sensors(include_sensors)
                  , _lock_interface(lock_interface)
            {
            }

            bool ShouldCollide(const JPH::BodyID &body_id) const override
            {
                if (_ignore_body.is_valid() && body_id.GetIndexAndSequenceNumber() == _ignore_body.value)
                {
                    return false;
                }

                if (!_include_sensors)
                {
                    JPH::BodyLockRead lock(_lock_interface, body_id);
                    if (lock.Succeeded() && lock.GetBody().IsSensor())
                    {
                        return false;
                    }
                }

                return true;
            }

            bool ShouldCollideLocked(const JPH::Body &body) const override
            {
                if (_ignore_body.is_valid() && body.GetID().GetIndexAndSequenceNumber() == _ignore_body.value)
                {
                    return false;
                }

                if (!_include_sensors && body.IsSensor())
                {
                    return false;
                }

                return true;
            }

        private:
            BodyId _ignore_body;
            bool _include_sensors;
            const JPH::BodyLockInterface &_lock_interface;
        };

        ShapeQueryFilter layer_filter(options.layer_mask);
        ShapeQueryBodyFilter body_filter(options.ignore_body, options.include_sensors,
                                         _physics_system.GetBodyLockInterface());

        JPH::ShapeCastSettings cast_settings;
        cast_settings.mReturnDeepestPoint = true;
        cast_settings.mBackFaceModeTriangles = options.backface_culling
                                                   ? JPH::EBackFaceMode::IgnoreBackFaces
                                                   : JPH::EBackFaceMode::CollideWithBackFaces;
        cast_settings.mBackFaceModeConvex = options.backface_culling
                                                ? JPH::EBackFaceMode::IgnoreBackFaces
                                                : JPH::EBackFaceMode::CollideWithBackFaces;

        JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
        _physics_system.GetNarrowPhaseQuery().CastShape(shape_cast, cast_settings, JPH::RVec3::sZero(), collector,
                                                        layer_filter, layer_filter, body_filter);

        if (collector.HadHit())
        {
            const JPH::ShapeCastResult &hit = collector.mHit;

            result.hit = true;
            result.distance = hit.mFraction * options.max_distance;
            result.position = glm::vec3(hit.mContactPointOn2.GetX(), hit.mContactPointOn2.GetY(),
                                        hit.mContactPointOn2.GetZ());

            const float n_len = hit.mPenetrationAxis.Length();
            if (n_len > 0.0f)
            {
                const JPH::Vec3 n = -hit.mPenetrationAxis / n_len;
                result.normal = glm::vec3(n.GetX(), n.GetY(), n.GetZ());
            }

            result.sub_shape_id = hit.mSubShapeID2.GetValue();
            result.body_id = BodyId{hit.mBodyID2.GetIndexAndSequenceNumber()}; {
                JPH::BodyLockRead lock(_physics_system.GetBodyLockInterface(), hit.mBodyID2);
                if (lock.Succeeded())
                {
                    const JPH::Body &body = lock.GetBody();
                    result.layer = body.GetObjectLayer();
                }
            }
            result.user_data = get_user_data(result.body_id);
        }

        return result;
    }

    void JoltPhysicsWorld::overlap(const CollisionShape &shape,
                                   const glm::vec3 &position,
                                   const glm::quat &rotation,
                                   const OverlapOptions &options,
                                   std::vector<OverlapHit> &out_hits) const
    {
        out_hits.clear();

        if (!_initialized)
        {
            return;
        }

        JPH::RefConst<JPH::Shape> jolt_shape = create_jolt_shape(shape);
        const JPH::Mat44 shape_com = JPH::Mat44::sTranslation(jolt_shape->GetCenterOfMass());

        const JPH::RMat44 com_transform =
                JPH::RMat44::sTranslation(JPH::RVec3(position.x, position.y, position.z)) *
                JPH::Mat44::sRotation(JPH::Quat(rotation.x, rotation.y, rotation.z, rotation.w)) *
                shape_com;

        class ShapeQueryFilter : public JPH::BroadPhaseLayerFilter, public JPH::ObjectLayerFilter
        {
        public:
            explicit ShapeQueryFilter(uint32_t layer_mask) : _layer_mask(layer_mask)
            {
            }

            bool ShouldCollide(JPH::BroadPhaseLayer) const override { return true; }

            bool ShouldCollide(JPH::ObjectLayer layer) const override
            {
                return (_layer_mask & (1u << layer)) != 0;
            }

        private:
            uint32_t _layer_mask;
        };

        class ShapeQueryBodyFilter : public JPH::BodyFilter
        {
        public:
            ShapeQueryBodyFilter(BodyId ignore_body, bool include_sensors, const JPH::BodyLockInterface &lock_interface)
                : _ignore_body(ignore_body)
                  , _include_sensors(include_sensors)
                  , _lock_interface(lock_interface)
            {
            }

            bool ShouldCollide(const JPH::BodyID &body_id) const override
            {
                if (_ignore_body.is_valid() && body_id.GetIndexAndSequenceNumber() == _ignore_body.value)
                {
                    return false;
                }

                if (!_include_sensors)
                {
                    JPH::BodyLockRead lock(_lock_interface, body_id);
                    if (lock.Succeeded() && lock.GetBody().IsSensor())
                    {
                        return false;
                    }
                }

                return true;
            }

            bool ShouldCollideLocked(const JPH::Body &body) const override
            {
                if (_ignore_body.is_valid() && body.GetID().GetIndexAndSequenceNumber() == _ignore_body.value)
                {
                    return false;
                }

                if (!_include_sensors && body.IsSensor())
                {
                    return false;
                }

                return true;
            }

        private:
            BodyId _ignore_body;
            bool _include_sensors;
            const JPH::BodyLockInterface &_lock_interface;
        };

        ShapeQueryFilter layer_filter(options.layer_mask);
        ShapeQueryBodyFilter body_filter(options.ignore_body, options.include_sensors,
                                         _physics_system.GetBodyLockInterface());

        JPH::CollideShapeSettings collide_settings;
        collide_settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

        JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
        _physics_system.GetNarrowPhaseQuery().CollideShape(
            jolt_shape.GetPtr(),
            JPH::Vec3::sOne(),
            com_transform,
            collide_settings,
            JPH::RVec3::sZero(),
            collector,
            layer_filter,
            layer_filter,
            body_filter);

        if (!collector.HadHit())
        {
            return;
        }

        out_hits.reserve(collector.mHits.size());
        for (const JPH::CollideShapeResult &h: collector.mHits)
        {
            OverlapHit out;
            out.body_id = BodyId{h.mBodyID2.GetIndexAndSequenceNumber()};
            out.sub_shape_id = h.mSubShapeID2.GetValue(); {
                JPH::BodyLockRead lock(_physics_system.GetBodyLockInterface(), h.mBodyID2);
                if (lock.Succeeded())
                {
                    const JPH::Body &body = lock.GetBody();
                    out.layer = body.GetObjectLayer();
                }
            }

            out.user_data = get_user_data(out.body_id);
            out_hits.push_back(out);
        }
    }

    // ============================================================================
    // Collision filtering
    // ============================================================================

    void JoltPhysicsWorld::set_layer_collision(uint32_t layer_a, uint32_t layer_b, bool should_collide)
    {
        if (layer_a >= Layer::Count || layer_b >= Layer::Count)
        {
            return;
        }

        const uint32_t bit_a = 1u << layer_a;
        const uint32_t bit_b = 1u << layer_b;

        std::scoped_lock lock(_layer_mutex);
        if (should_collide)
        {
            _layer_collision_mask[layer_a] |= bit_b;
            _layer_collision_mask[layer_b] |= bit_a;
        }
        else
        {
            _layer_collision_mask[layer_a] &= ~bit_b;
            _layer_collision_mask[layer_b] &= ~bit_a;
        }
    }

    bool JoltPhysicsWorld::get_layer_collision(uint32_t layer_a, uint32_t layer_b) const
    {
        if (layer_a >= Layer::Count || layer_b >= Layer::Count)
        {
            return false;
        }

        const uint32_t bit_b = 1u << layer_b;
        std::scoped_lock lock(_layer_mutex);
        return (_layer_collision_mask[layer_a] & bit_b) != 0;
    }

    // ============================================================================
    // Contact callbacks
    // ============================================================================

    void JoltPhysicsWorld::set_body_callbacks(BodyId id, const BodyCallbacks &callbacks)
    {
        if (!id.is_valid())
        {
            return;
        }

        std::scoped_lock lock(_callbacks_mutex);
        _callbacks[id.value] = callbacks;
    }

    void JoltPhysicsWorld::clear_body_callbacks(BodyId id)
    {
        std::scoped_lock lock(_callbacks_mutex);
        _callbacks.erase(id.value);
    }

    // ============================================================================
    // Joints
    // ============================================================================

    JointId JoltPhysicsWorld::create_fixed_joint(BodyId body_a, BodyId body_b)
    {
        if (!_initialized || !body_a.is_valid() || !body_b.is_valid())
        {
            return JointId{};
        }

        const JPH::BodyID bodies[2] = {JPH::BodyID(body_a.value), JPH::BodyID(body_b.value)};
        JPH::BodyLockMultiWrite lock(_physics_system.GetBodyLockInterface(), bodies, 2);
        JPH::Body *b1 = lock.GetBody(0);
        JPH::Body *b2 = lock.GetBody(1);
        if (b1 == nullptr || b2 == nullptr)
        {
            return JointId{};
        }

        JPH::FixedConstraintSettings s;
        s.mSpace = JPH::EConstraintSpace::WorldSpace;
        s.mAutoDetectPoint = true;

        JPH::Constraint *constraint = s.Create(*b1, *b2);
        if (constraint == nullptr)
        {
            return JointId{};
        }

        _physics_system.AddConstraint(constraint);

        uint32_t id = 0; {
            std::scoped_lock lock_joints(_joints_mutex);
            id = _next_joint_id++;
            _joints[id] = constraint;
        }

        return JointId{id};
    }

    JointId JoltPhysicsWorld::create_hinge_joint(BodyId body_a, BodyId body_b, const HingeJointSettings &settings)
    {
        if (!_initialized || !body_a.is_valid() || !body_b.is_valid())
        {
            return JointId{};
        }

        const glm::vec3 axis = safe_normalize(settings.axis, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::vec3 normal;
        compute_basis(axis, normal);

        const JPH::BodyID bodies[2] = {JPH::BodyID(body_a.value), JPH::BodyID(body_b.value)};
        JPH::BodyLockMultiWrite lock(_physics_system.GetBodyLockInterface(), bodies, 2);
        JPH::Body *b1 = lock.GetBody(0);
        JPH::Body *b2 = lock.GetBody(1);
        if (b1 == nullptr || b2 == nullptr)
        {
            return JointId{};
        }

        JPH::HingeConstraintSettings s;
        s.mSpace = JPH::EConstraintSpace::WorldSpace;
        s.mPoint1 = JPH::RVec3(settings.anchor.x, settings.anchor.y, settings.anchor.z);
        s.mPoint2 = s.mPoint1;
        s.mHingeAxis1 = JPH::Vec3(axis.x, axis.y, axis.z);
        s.mHingeAxis2 = s.mHingeAxis1;
        s.mNormalAxis1 = JPH::Vec3(normal.x, normal.y, normal.z);
        s.mNormalAxis2 = s.mNormalAxis1;

        if (settings.enable_limits)
        {
            s.mLimitsMin = settings.limit_min;
            s.mLimitsMax = settings.limit_max;
        }
        else
        {
            s.mLimitsMin = -JPH::JPH_PI;
            s.mLimitsMax = JPH::JPH_PI;
        }

        JPH::Constraint *constraint = s.Create(*b1, *b2);
        if (constraint == nullptr)
        {
            return JointId{};
        }

        _physics_system.AddConstraint(constraint);

        uint32_t id = 0; {
            std::scoped_lock lock_joints(_joints_mutex);
            id = _next_joint_id++;
            _joints[id] = constraint;
        }

        return JointId{id};
    }

    JointId JoltPhysicsWorld::create_slider_joint(BodyId body_a, BodyId body_b, const SliderJointSettings &settings)
    {
        if (!_initialized || !body_a.is_valid() || !body_b.is_valid())
        {
            return JointId{};
        }

        const glm::vec3 axis = safe_normalize(settings.axis, glm::vec3(1.0f, 0.0f, 0.0f));
        glm::vec3 normal;
        compute_basis(axis, normal);

        const JPH::BodyID bodies[2] = {JPH::BodyID(body_a.value), JPH::BodyID(body_b.value)};
        JPH::BodyLockMultiWrite lock(_physics_system.GetBodyLockInterface(), bodies, 2);
        JPH::Body *b1 = lock.GetBody(0);
        JPH::Body *b2 = lock.GetBody(1);
        if (b1 == nullptr || b2 == nullptr)
        {
            return JointId{};
        }

        JPH::SliderConstraintSettings s;
        s.mSpace = JPH::EConstraintSpace::WorldSpace;
        s.mAutoDetectPoint = false;
        s.mPoint1 = JPH::RVec3(settings.anchor.x, settings.anchor.y, settings.anchor.z);
        s.mPoint2 = s.mPoint1;
        s.mSliderAxis1 = JPH::Vec3(axis.x, axis.y, axis.z);
        s.mSliderAxis2 = s.mSliderAxis1;
        s.mNormalAxis1 = JPH::Vec3(normal.x, normal.y, normal.z);
        s.mNormalAxis2 = s.mNormalAxis1;

        if (settings.enable_limits)
        {
            s.mLimitsMin = settings.limit_min;
            s.mLimitsMax = settings.limit_max;
        }
        else
        {
            s.mLimitsMin = -FLT_MAX;
            s.mLimitsMax = FLT_MAX;
        }

        JPH::Constraint *constraint = s.Create(*b1, *b2);
        if (constraint == nullptr)
        {
            return JointId{};
        }

        _physics_system.AddConstraint(constraint);

        uint32_t id = 0; {
            std::scoped_lock lock_joints(_joints_mutex);
            id = _next_joint_id++;
            _joints[id] = constraint;
        }

        return JointId{id};
    }

    void JoltPhysicsWorld::destroy_joint(JointId id)
    {
        if (!_initialized || !id.is_valid())
        {
            return;
        }

        JPH::Constraint *constraint = nullptr; {
            std::scoped_lock lock_joints(_joints_mutex);
            auto it = _joints.find(id.value);
            if (it == _joints.end())
            {
                return;
            }
            constraint = it->second;
            _joints.erase(it);
        }

        if (constraint != nullptr)
        {
            _physics_system.RemoveConstraint(constraint);
        }
    }

    bool JoltPhysicsWorld::is_joint_valid(JointId id) const
    {
        if (!id.is_valid())
        {
            return false;
        }

        std::scoped_lock lock_joints(_joints_mutex);
        return _joints.find(id.value) != _joints.end();
    }

    // ============================================================================
    // World settings
    // ============================================================================

    void JoltPhysicsWorld::set_gravity(const glm::vec3 &gravity)
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
