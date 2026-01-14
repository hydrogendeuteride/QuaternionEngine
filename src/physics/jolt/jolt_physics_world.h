#pragma once

#include "physics/physics_world.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

#include <memory>
#include <mutex>

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
        uint32_t temp_allocator_size = 10 * 1024 * 1024;  // 10 MB
        glm::vec3 gravity{0.0f, -9.81f, 0.0f};
    };

    JoltPhysicsWorld();
    explicit JoltPhysicsWorld(const Config& config);
    ~JoltPhysicsWorld() override;

    // Non-copyable, non-movable
    JoltPhysicsWorld(const JoltPhysicsWorld&) = delete;
    JoltPhysicsWorld& operator=(const JoltPhysicsWorld&) = delete;

    // ========================================================================
    // PhysicsWorld interface implementation
    // ========================================================================

    void step(float dt) override;

    BodyId create_body(const BodySettings& settings) override;
    void destroy_body(BodyId id) override;

    bool is_body_valid(BodyId id) const override;

    BodyTransform get_transform(BodyId id) const override;
    glm::vec3 get_position(BodyId id) const override;
    glm::quat get_rotation(BodyId id) const override;
    glm::mat4 get_transform_matrix(BodyId id) const override;

    glm::vec3 get_linear_velocity(BodyId id) const override;
    glm::vec3 get_angular_velocity(BodyId id) const override;

    void set_position(BodyId id, const glm::vec3& position) override;
    void set_rotation(BodyId id, const glm::quat& rotation) override;
    void set_transform(BodyId id, const glm::vec3& position, const glm::quat& rotation) override;

    void set_linear_velocity(BodyId id, const glm::vec3& velocity) override;
    void set_angular_velocity(BodyId id, const glm::vec3& velocity) override;

    void add_force(BodyId id, const glm::vec3& force) override;
    void add_impulse(BodyId id, const glm::vec3& impulse) override;
    void add_torque(BodyId id, const glm::vec3& torque) override;

    void activate(BodyId id) override;
    void deactivate(BodyId id) override;
    bool is_active(BodyId id) const override;

    RayHit raycast(const glm::vec3& origin, const glm::vec3& direction, float max_distance) const override;

    void set_gravity(const glm::vec3& gravity) override;
    glm::vec3 get_gravity() const override;

    // ========================================================================
    // Jolt-specific access (for advanced use cases)
    // ========================================================================

    JPH::PhysicsSystem& jolt_system() { return _physics_system; }
    const JPH::PhysicsSystem& jolt_system() const { return _physics_system; }

    JPH::BodyInterface& body_interface() { return _physics_system.GetBodyInterface(); }
    const JPH::BodyInterface& body_interface() const { return _physics_system.GetBodyInterface(); }

private:
    // ========================================================================
    // Jolt global initialization (reference counted)
    // ========================================================================

    struct JoltGlobals
    {
        JoltGlobals();
        ~JoltGlobals();

        static std::mutex& mutex();
        static int& ref_count();
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
        const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override;
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
        bool ShouldCollide(JPH::ObjectLayer layer1, JPH::ObjectLayer layer2) const override;
    };

    // ========================================================================
    // Helper methods
    // ========================================================================

    void init(const Config& config);
    static int compute_worker_threads();

    JPH::RefConst<JPH::Shape> create_jolt_shape(const CollisionShape& shape) const;
    static JPH::EMotionType to_jolt_motion_type(MotionType type);
    static JPH::ObjectLayer to_jolt_layer(uint32_t layer, MotionType motion);

    // Jolt callbacks
    static void trace_impl(const char* fmt, ...);
#ifdef JPH_ENABLE_ASSERTS
    static bool assert_failed_impl(const char* expression, const char* message, const char* file, JPH::uint line);
#endif

    // ========================================================================
    // Member data
    // ========================================================================

    JoltGlobals _globals;  // Must be first to ensure proper initialization order
    bool _initialized{false};

    BPLayerInterfaceImpl _broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl _object_vs_broad_phase_layer_filter;
    ObjectLayerPairFilterImpl _object_layer_pair_filter;

    JPH::PhysicsSystem _physics_system;
    std::unique_ptr<JPH::TempAllocatorImpl> _temp_allocator;
    std::unique_ptr<JPH::JobSystemThreadPool> _job_system;
};

} // namespace Physics

#endif // VULKAN_ENGINE_USE_JOLT
