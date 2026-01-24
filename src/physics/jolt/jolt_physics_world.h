#pragma once

#include "physics/physics_world.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>

#include <memory>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <vector>

namespace Physics
{
    // ============================================================================
    // JoltPhysicsWorld: Jolt Physics implementation of PhysicsWorld interface
    // ============================================================================

    class JoltPhysicsWorld final : public PhysicsWorld
    {
    public:
        struct Config
        {
            uint32_t max_bodies = 4096;
            uint32_t max_body_pairs = 4096;
            uint32_t max_contact_constraints = 4096;
            uint32_t temp_allocator_size = 10 * 1024 * 1024; // 10 MB
            glm::vec3 gravity{0.0f, -9.81f, 0.0f};
        };

        JoltPhysicsWorld();

        explicit JoltPhysicsWorld(const Config &config);

        ~JoltPhysicsWorld() override;

        // Non-copyable, non-movable
        JoltPhysicsWorld(const JoltPhysicsWorld &) = delete;

        JoltPhysicsWorld &operator=(const JoltPhysicsWorld &) = delete;

        // ========================================================================
        // PhysicsWorld interface implementation
        // ========================================================================

        void step(float dt) override;
        void shift_origin(const glm::dvec3 &delta_local) override;
        void shift_velocity_origin(const glm::dvec3 &delta_local_velocity) override;

        DebugStats debug_stats() const override;
        void for_each_debug_body(const DebugBodyFn &fn) const override;

        BodyId create_body(const BodySettings &settings) override;

        void destroy_body(BodyId id) override;

        bool is_body_valid(BodyId id) const override;

        BodyTransform get_transform(BodyId id) const override;

        glm::dvec3 get_position(BodyId id) const override;

        glm::quat get_rotation(BodyId id) const override;

        glm::mat4 get_transform_matrix(BodyId id) const override;

        glm::vec3 get_linear_velocity(BodyId id) const override;

        glm::vec3 get_angular_velocity(BodyId id) const override;

        uint64_t get_user_data(BodyId id) const override;

        void set_position(BodyId id, const glm::dvec3 &position) override;

        void set_rotation(BodyId id, const glm::quat &rotation) override;

        void set_transform(BodyId id, const glm::dvec3 &position, const glm::quat &rotation) override;

        void set_linear_velocity(BodyId id, const glm::vec3 &velocity) override;

        void set_angular_velocity(BodyId id, const glm::vec3 &velocity) override;

        void set_user_data(BodyId id, uint64_t user_data) override;

        void add_force(BodyId id, const glm::vec3 &force) override;

        void add_impulse(BodyId id, const glm::vec3 &impulse) override;

        void add_torque(BodyId id, const glm::vec3 &torque) override;

        void activate(BodyId id) override;

        void deactivate(BodyId id) override;

        bool is_active(BodyId id) const override;

        RayHit raycast(const glm::dvec3 &origin, const glm::vec3 &direction, float max_distance) const override;

        RayHit raycast(const glm::dvec3 &origin, const glm::vec3 &direction,
                       const RaycastOptions &options) const override;

        RayHit sweep(const CollisionShape &shape,
                     const glm::dvec3 &origin,
                     const glm::quat &rotation,
                     const glm::vec3 &direction,
                     const SweepOptions &options) const override;

        void overlap(const CollisionShape &shape,
                     const glm::dvec3 &position,
                     const glm::quat &rotation,
                     const OverlapOptions &options,
                     std::vector<OverlapHit> &out_hits) const override;

        void set_layer_collision(uint32_t layer_a, uint32_t layer_b, bool should_collide) override;

        bool get_layer_collision(uint32_t layer_a, uint32_t layer_b) const override;

        void set_body_callbacks(BodyId id, const BodyCallbacks &callbacks) override;

        void clear_body_callbacks(BodyId id) override;

        JointId create_fixed_joint(BodyId body_a, BodyId body_b) override;

        JointId create_hinge_joint(BodyId body_a, BodyId body_b, const HingeJointSettings &settings) override;

        JointId create_slider_joint(BodyId body_a, BodyId body_b, const SliderJointSettings &settings) override;

        void destroy_joint(JointId id) override;

        bool is_joint_valid(JointId id) const override;

        void set_gravity(const glm::vec3 &gravity) override;

        glm::vec3 get_gravity() const override;

        // ========================================================================
        // Jolt-specific access (for advanced use cases)
        // ========================================================================

        JPH::PhysicsSystem &jolt_system() { return _physics_system; }
        const JPH::PhysicsSystem &jolt_system() const { return _physics_system; }

        JPH::BodyInterface &body_interface() { return _physics_system.GetBodyInterface(); }
        const JPH::BodyInterface &body_interface() const { return _physics_system.GetBodyInterface(); }

    private:
        // ========================================================================
        // Jolt global initialization (reference counted)
        // ========================================================================

        struct JoltGlobals
        {
            JoltGlobals();

            ~JoltGlobals();

            static std::mutex &mutex();

            static int &ref_count();
        };

        // ========================================================================
        // Collision layer mapping
        // ========================================================================

        struct BroadPhaseLayers
        {
            static constexpr JPH::BroadPhaseLayer non_moving{0};
            static constexpr JPH::BroadPhaseLayer moving{1};
            static constexpr uint32_t count = 2;
        };

        class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
        {
        public:
            BPLayerInterfaceImpl();

            JPH::uint GetNumBroadPhaseLayers() const override;

            JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
            const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override;
#endif

        private:
            JPH::BroadPhaseLayer _object_to_broad_phase[Layer::Count];
        };

        class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer layer1, JPH::BroadPhaseLayer layer2) const override;
        };

        class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
        {
        public:
            void set_layer_collision_mask(const uint32_t *mask, uint32_t mask_count)
            {
                _layer_collision_mask = mask;
                _mask_count = mask_count;
            }

            bool ShouldCollide(JPH::ObjectLayer layer1, JPH::ObjectLayer layer2) const override;

        private:
            const uint32_t *_layer_collision_mask{nullptr};
            uint32_t _mask_count{0};
        };

        // ========================================================================
        // Helper methods
        // ========================================================================

        void init(const Config &config);

        static int compute_worker_threads();

        JPH::RefConst<JPH::Shape> create_jolt_shape(const CollisionShape &shape) const;

        static JPH::EMotionType to_jolt_motion_type(MotionType type);

        static JPH::ObjectLayer to_jolt_layer(uint32_t layer, MotionType motion);

        static glm::vec3 safe_normalize(const glm::vec3 &v, const glm::vec3 &fallback);

        static void compute_basis(const glm::vec3 &axis, glm::vec3 &out_normal);

        void dispatch_contact_events();

        // Jolt callbacks
        static void trace_impl(const char *fmt, ...);
#ifdef JPH_ENABLE_ASSERTS
        static bool assert_failed_impl(const char *expression, const char *message, const char *file, JPH::uint line);
#endif

        // ========================================================================
        // Member data
        // ========================================================================

        struct BodyDebugRecord
        {
            CollisionShape shape{};
            MotionType motion_type{MotionType::Static};
            uint32_t layer{0};
            bool is_sensor{false};
        };

        JoltGlobals _globals; // Must be first to ensure proper initialization order
        bool _initialized{false};

        BPLayerInterfaceImpl _broad_phase_layer_interface;
        ObjectVsBroadPhaseLayerFilterImpl _object_vs_broad_phase_layer_filter;
        ObjectLayerPairFilterImpl _object_layer_pair_filter;

        JPH::PhysicsSystem _physics_system;
        std::unique_ptr<JPH::TempAllocatorImpl> _temp_allocator;
        std::unique_ptr<JPH::JobSystemThreadPool> _job_system;

        // Collision matrix (bitmask per layer)
        uint32_t _layer_collision_mask[Layer::Count]{};
        mutable std::mutex _layer_mutex;

        // Per-body callbacks
        mutable std::mutex _callbacks_mutex;
        std::unordered_map<uint32_t, BodyCallbacks> _callbacks;

        // Contact event queue
        struct ContactEventRecord
        {
            bool is_trigger{false};
            CollisionEvent collision;
            TriggerEvent trigger;
        };

        mutable std::mutex _events_mutex;
        std::vector<ContactEventRecord> _queued_events;

        // For removed events: remember if a sub-shape pair was a trigger and cache ids
        struct PairCacheEntry
        {
            bool is_trigger{false};
            uint64_t user_data1{0};
            uint64_t user_data2{0};
            uint32_t layer1{0};
            uint32_t layer2{0};
            bool is_sensor1{false};
            bool is_sensor2{false};
        };

        mutable std::mutex _pair_cache_mutex;
        std::unordered_map<JPH::SubShapeIDPair, PairCacheEntry> _pair_cache;

        // Mesh shape cache (TriangleMeshData → built Jolt MeshShape)
        mutable std::mutex _mesh_shape_cache_mutex;
        mutable std::unordered_map<std::shared_ptr<const TriangleMeshData>, JPH::RefConst<JPH::Shape>> _mesh_shape_cache;

        // Joints
        uint32_t _next_joint_id{1};
        mutable std::mutex _joints_mutex;
        std::unordered_map<uint32_t, JPH::Constraint *> _joints;

        // Debug: body metadata (shapes, types) for visualization and UI
        mutable std::mutex _debug_bodies_mutex;
        std::unordered_map<uint32_t, BodyDebugRecord> _debug_bodies;

        // Debug: step instrumentation
        std::atomic<float> _debug_last_step_ms{0.0f};
        std::atomic<float> _debug_avg_step_ms{0.0f};
        std::atomic<float> _debug_last_dt{0.0f};
        std::atomic<uint32_t> _debug_body_count{0};
        std::atomic<uint32_t> _debug_active_body_count{0};
        std::atomic<uint32_t> _debug_joint_count{0};
        std::atomic<uint32_t> _debug_contact_event_count{0};

        // Contact listener
        class ContactListenerImpl;
        std::unique_ptr<ContactListenerImpl> _contact_listener;
    };
} // namespace Physics

#endif // VULKAN_ENGINE_USE_JOLT
