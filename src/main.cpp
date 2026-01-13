// Two modes available:
// 1. Legacy mode: Uses VulkanEngine::run() directly (simple, no game separation)
// 2. GameRuntime mode: Uses GameRuntime for clean game/engine separation
//
// Set USE_GAME_RUNTIME to 1

#define USE_GAME_RUNTIME 1

#include "core/engine.h"

#if USE_GAME_RUNTIME
#include "runtime/game_runtime.h"
#include <glm/gtx/transform.hpp>

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#endif

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <thread>

class ExampleGame : public GameRuntime::IGameCallbacks
{
public:
    void on_init(GameRuntime::Runtime &runtime) override
    {
        _runtime = &runtime;

        auto &api = runtime.api();
        VulkanEngine *renderer = runtime.renderer();
        if (renderer && renderer->_assetManager)
        {
            GameAPI::IBLPaths ibl{};
            ibl.specularCube = renderer->_assetManager->assetPath("ibl/sky.ktx2");
            ibl.diffuseCube = renderer->_assetManager->assetPath("ibl/sky.ktx2");
            ibl.brdfLut = renderer->_assetManager->assetPath("ibl/brdf_lut.ktx2");
            ibl.background = renderer->_assetManager->assetPath("ibl/sky.ktx2");
            api.load_global_ibl(ibl);
        }

        // {
        //     constexpr double kEarthRadiusM = 6378137.0;
        //     constexpr double kMoonRadiusM = 1737400.0;
        //     constexpr double kMoonDistanceM = 384400000.0;
        //
        //     GameAPI::PlanetTerrain earth{};
        //     earth.name = "Earth";
        //     earth.center = glm::dvec3(0.0, 0.0, 0.0);
        //     earth.radius_m = kEarthRadiusM;
        //     earth.visible = true;
        //     earth.base_color = glm::vec4(1.0f);
        //     earth.metallic = 0.0f;
        //     earth.roughness = 1.0f;
        //     earth.albedo_dir = "planets/earth/albedo/L0";
        //     earth.height_dir = "planets/earth/height/L0";
        //     earth.height_max_m = 6400.0;
        //     earth.emission_dir = "planets/earth/emission/L0";
        //     earth.emission_factor = glm::vec3(1.0f, 1.0f, 1.0f);
        //     api.add_planet_terrain(earth);
        //
        //     GameAPI::PlanetSphere moon{};
        //     moon.name = "Moon";
        //     moon.center = glm::dvec3(kMoonDistanceM, 0.0, 0.0);
        //     moon.radius_m = kMoonRadiusM;
        //     moon.visible = true;
        //     moon.base_color = glm::vec4(0.72f, 0.72f, 0.75f, 1.0f);
        //     moon.metallic = 0.0f;
        //     moon.roughness = 1.0f;
        //     api.add_planet_sphere(moon);
        //
        //     GameAPI::FreeCameraSettings free = api.get_free_camera_settings();
        //     free.moveSpeed = 20000.0f;
        //     api.set_free_camera_settings(free);
        //
        //     api.set_camera_position(glm::dvec3(0.0, 0.0, kEarthRadiusM + 1.0e6));
        //     api.camera_look_at(glm::dvec3(0.0, 0.0, 0.0));
        // }

        // api.add_gltf_instance_async("example_model", "models/example.gltf",
        //     GameAPI::Transform{.position = {0, 0, 0}});

        // api.add_primitive_instance("test_cube", GameAPI::PrimitiveType::Cube,
        //     GameAPI::Transform{.position = {2, 0, 0}});

        api.set_camera_position(glm::vec3(0.0f, 4.0f, 10.0f));
        api.camera_look_at(glm::vec3(0.0f, 1.5f, 10.0f));

        // Visuals
        {
            GameAPI::Transform ground_tr{};
            ground_tr.position = {0.0f, 0.0f, 0.0f};
            ground_tr.scale = {50.0f, 1.0f, 50.0f};
            api.add_primitive_instance(_ground_instance, GameAPI::PrimitiveType::Plane, ground_tr);

            GameAPI::Transform sphere_tr{};
            sphere_tr.position = {0.0f, _spawn_height_m, 0.0f};
            sphere_tr.scale = {1.0f, 1.0f, 1.0f};
            api.add_primitive_instance(_sphere_instance, GameAPI::PrimitiveType::Sphere, sphere_tr);
        }

        fmt::println("aaa");
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        // Physics (Jolt): static floor + one dynamic sphere
        _physics = std::make_unique<JoltPhysicsWorld>();
        _runtime->set_physics_world(_physics.get());

        _physics->create_static_floor(glm::vec3(50.0f, 1.0f, 50.0f), glm::vec3(0.0f, -1.0f, 0.0f));
        _sphere_body = _physics->create_dynamic_sphere(_sphere_radius_m, glm::vec3(0.0f, _spawn_height_m, 0.0f));
#endif
        fmt::println("aaa");
    }

    void on_update(float dt) override
    {
        _elapsed += dt;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics && _sphere_body != 0)
        {
            glm::mat4 m(1.0f);
            _physics->get_body_transform(_sphere_body, m);

            GameAPI::Transform tr = GameAPI::Transform::from_matrix(m);
            tr.scale = glm::vec3(1.0f); // keep primitive at unit scale (sphere mesh radius is 0.5)
            _runtime->api().set_mesh_instance_transform(_sphere_instance, tr);

            // Simple reset when it falls too far
            if (tr.position.y < -50.0f)
            {
                _physics->reset_body(_sphere_body, glm::vec3(0.0f, _spawn_height_m, 0.0f));
            }
        }
#endif
    }

    void on_fixed_update(float fixed_dt) override
    {
        (void) fixed_dt;
    }

    void on_shutdown() override
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_runtime)
        {
            _runtime->set_physics_world(nullptr);
        }
        _physics.reset();
#endif
        _runtime = nullptr;
    }

private:
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
    // Minimal Jolt wrapper implementing GameRuntime::IPhysicsWorld.
    class JoltPhysicsWorld final : public GameRuntime::IPhysicsWorld
    {
    public:
        JoltPhysicsWorld()
            : _temp_allocator(10 * 1024 * 1024)
              , _job_system(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, compute_worker_threads())
        {
            init();
        }

        ~JoltPhysicsWorld() override = default;

        void step(float dt) override
        {
            if (!_initialized)
            {
                return;
            }

            (void) _physics_system.Update(dt, 1, &_temp_allocator, &_job_system);
        }

        void get_body_transform(uint32_t id, glm::mat4 &out) override
        {
            out = glm::mat4(1.0f);
            if (!_initialized || id == 0)
            {
                return;
            }

            const JPH::BodyID body_id(id);
            JPH::BodyLockRead lock(_physics_system.GetBodyLockInterface(), body_id);
            if (!lock.Succeeded())
            {
                return;
            }

            const JPH::Body &body = lock.GetBody();
            const JPH::RVec3 p = body.GetPosition();
            const JPH::Quat q = body.GetRotation();

            const glm::vec3 pos = {
                static_cast<float>(p.GetX()), static_cast<float>(p.GetY()), static_cast<float>(p.GetZ())
            };
            const glm::quat rot = {q.GetW(), q.GetX(), q.GetY(), q.GetZ()};

            out = glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(rot);
        }

        RayHit raycast(const glm::vec3 &origin, const glm::vec3 &direction, float maxDistance) override
        {
            RayHit out{};
            if (!_initialized)
            {
                return out;
            }

            const glm::vec3 dir_norm = glm::length(direction) > 0.0f
                                           ? glm::normalize(direction)
                                           : glm::vec3(0.0f, -1.0f, 0.0f);

            const JPH::RRayCast ray(
                JPH::RVec3(origin.x, origin.y, origin.z),
                JPH::Vec3(dir_norm.x, dir_norm.y, dir_norm.z) * maxDistance);

            JPH::RayCastResult hit{};
            if (_physics_system.GetNarrowPhaseQuery().CastRay(ray, hit))
            {
                out.hit = true;
                out.distance = hit.mFraction * maxDistance;
                const JPH::RVec3 hp = ray.GetPointOnRay(hit.mFraction);
                out.position = glm::vec3(static_cast<float>(hp.GetX()), static_cast<float>(hp.GetY()),
                                         static_cast<float>(hp.GetZ()));
                out.normal = glm::vec3(0.0f, 1.0f, 0.0f); // TODO: query hit normal
                out.bodyId = hit.mBodyID.GetIndexAndSequenceNumber();
            }

            return out;
        }

        void create_static_floor(const glm::vec3 &half_extents, const glm::vec3 &center)
        {
            if (!_initialized)
            {
                return;
            }

            const JPH::RefConst<JPH::Shape> shape = new JPH::BoxShape(
                JPH::Vec3(half_extents.x, half_extents.y, half_extents.z));
            const JPH::BodyCreationSettings settings(
                shape,
                JPH::RVec3(center.x, center.y, center.z),
                JPH::Quat::sIdentity(),
                JPH::EMotionType::Static,
                Layers::non_moving);

            _physics_system.GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::DontActivate);
        }

        uint32_t create_dynamic_sphere(float radius_m, const glm::vec3 &center)
        {
            if (!_initialized)
            {
                return 0;
            }

            const JPH::RefConst<JPH::Shape> shape = new JPH::SphereShape(radius_m);

            JPH::BodyCreationSettings settings(
                shape,
                JPH::RVec3(center.x, center.y, center.z),
                JPH::Quat::sIdentity(),
                JPH::EMotionType::Dynamic,
                Layers::moving);
            settings.mFriction = 0.6f;
            settings.mRestitution = 0.1f;
            settings.mLinearDamping = 0.02f;

            const JPH::BodyID id = _physics_system.GetBodyInterface().CreateAndAddBody(
                settings, JPH::EActivation::Activate);
            return id.IsInvalid() ? 0u : id.GetIndexAndSequenceNumber();
        }

        void reset_body(uint32_t id, const glm::vec3 &position)
        {
            if (!_initialized || id == 0)
            {
                return;
            }

            JPH::BodyInterface &bi = _physics_system.GetBodyInterface();
            const JPH::BodyID body_id(id);

            bi.SetLinearVelocity(body_id, JPH::Vec3::sZero());
            bi.SetAngularVelocity(body_id, JPH::Vec3::sZero());
            bi.SetPositionAndRotation(body_id, JPH::RVec3(position.x, position.y, position.z), JPH::Quat::sIdentity(),
                                      JPH::EActivation::Activate);
        }

    private:
        // Ensures Jolt global state is initialized before any Jolt objects that allocate memory.
        struct JoltGlobals final
        {
            JoltGlobals()
            {
                std::scoped_lock lock(mutex());
                int &count = ref_count();
                if (count++ == 0)
                {
                    // This needs to be done before any other Jolt function is called.
                    JPH::RegisterDefaultAllocator();

                    // Install trace and assert callbacks (default implementation asserts).
                    JPH::Trace = &trace_impl;
#ifdef JPH_ENABLE_ASSERTS
                    JPH::AssertFailed = &assert_failed_impl;
#endif

                    // Required for (de)serialization and type registration.
                    JPH::Factory::sInstance = new JPH::Factory();
                    JPH::RegisterTypes();
                }
            }

            ~JoltGlobals()
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

            static std::mutex &mutex()
            {
                static std::mutex m;
                return m;
            }

            static int &ref_count()
            {
                static int count = 0;
                return count;
            }
        };

        struct Layers
        {
            static constexpr JPH::ObjectLayer non_moving = 0;
            static constexpr JPH::ObjectLayer moving = 1;
            static constexpr uint32_t count = 2;
        };

        struct BroadPhaseLayers
        {
            static constexpr JPH::BroadPhaseLayer non_moving{0};
            static constexpr JPH::BroadPhaseLayer moving{1};
            static constexpr uint32_t count = 2;
        };

        class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
        {
        public:
            BPLayerInterfaceImpl()
            {
                _object_to_broad_phase[Layers::non_moving] = BroadPhaseLayers::non_moving;
                _object_to_broad_phase[Layers::moving] = BroadPhaseLayers::moving;
            }

            JPH::uint GetNumBroadPhaseLayers() const override
            {
                return BroadPhaseLayers::count;
            }

            JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
            {
                return _object_to_broad_phase[layer];
            }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
            const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
            {
                switch (layer.GetValue())
                {
                    case 0: return "NON_MOVING";
                    case 1: return "MOVING";
                    default: return "INVALID";
                }
            }
#endif

        private:
            JPH::BroadPhaseLayer _object_to_broad_phase[Layers::count];
        };

        class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer layer1, JPH::BroadPhaseLayer layer2) const override
            {
                switch (layer1)
                {
                    case Layers::non_moving:
                        return layer2 == BroadPhaseLayers::moving;
                    case Layers::moving:
                        return true;
                    default:
                        return false;
                }
            }
        };

        class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer layer1, JPH::ObjectLayer layer2) const override
            {
                switch (layer1)
                {
                    case Layers::non_moving:
                        return layer2 == Layers::moving;
                    case Layers::moving:
                        return true;
                    default:
                        return false;
                }
            }
        };

        void init()
        {
            if (_initialized)
            {
                return;
            }

            constexpr uint32_t max_bodies = 1024;
            constexpr uint32_t num_body_mutexes = 0;
            constexpr uint32_t max_body_pairs = 1024;
            constexpr uint32_t max_contact_constraints = 1024;

            _physics_system.Init(
                max_bodies,
                num_body_mutexes,
                max_body_pairs,
                max_contact_constraints,
                _broad_phase_layer_interface,
                _object_vs_broad_phase_layer_filter,
                _object_layer_pair_filter);

            _physics_system.SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));

            _initialized = true;
        }

        static int compute_worker_threads()
        {
            const uint32_t hw = std::max(1u, std::thread::hardware_concurrency());
            const uint32_t workers = hw > 1 ? hw - 1 : 1;
            return static_cast<int>(workers);
        }

        static void trace_impl(const char *fmt, ...)
        {
            va_list args;
            va_start(args, fmt);
            std::vfprintf(stderr, fmt, args);
            std::fputc('\n', stderr);
            va_end(args);
        }

#ifdef JPH_ENABLE_ASSERTS
        static bool assert_failed_impl(const char *expression, const char *message, const char *file, JPH::uint line)
        {
            std::fprintf(stderr, "[Jolt][Assert] %s:%u: %s", file, static_cast<unsigned>(line), expression);
            if (message != nullptr)
            {
                std::fprintf(stderr, " (%s)", message);
            }
            std::fputc('\n', stderr);

            // Return false: don't trigger a breakpoint / trap (keeps the sample running in release-like environments).
            return false;
        }
#endif

        JoltGlobals _globals;

        bool _initialized{false};

        BPLayerInterfaceImpl _broad_phase_layer_interface;
        ObjectVsBroadPhaseLayerFilterImpl _object_vs_broad_phase_layer_filter;
        ObjectLayerPairFilterImpl _object_layer_pair_filter;

        JPH::PhysicsSystem _physics_system;
        JPH::TempAllocatorImpl _temp_allocator;
        JPH::JobSystemThreadPool _job_system;
    };
#endif // VULKAN_ENGINE_USE_JOLT

    GameRuntime::Runtime *_runtime{nullptr};

    float _elapsed{0.0f};

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
    std::unique_ptr<JoltPhysicsWorld> _physics;
    uint32_t _sphere_body{0};
#endif

    static constexpr const char *_ground_instance = "physics_ground";
    static constexpr const char *_sphere_instance = "physics_sphere";

    static constexpr float _sphere_radius_m = 0.5f;
    static constexpr float _spawn_height_m = 10.0f;
};
#endif // USE_GAME_RUNTIME

int main(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    VulkanEngine engine;
    engine.init();

#if USE_GAME_RUNTIME
    {
        GameRuntime::Runtime runtime(&engine);
        ExampleGame game;
        runtime.run(&game);
    }
#else
    // Legacy
    engine.run();
#endif

    engine.cleanup();
    return 0;
}
