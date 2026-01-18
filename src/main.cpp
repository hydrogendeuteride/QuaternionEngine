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

#include "physics/jolt/jolt_physics_world.h"
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

        api.set_camera_position(glm::vec3(-15.0f, 6.0f, 0.0f));
        api.camera_look_at(glm::vec3(1.0f, 0.0f, 0.0f));

        // Visuals
        {
            build_box_stack_layout();

            GameAPI::Transform ground_tr{};
            ground_tr.position = {0.0f, 0.0f, 0.0f};
            ground_tr.scale = {50.0f, 1.0f, 50.0f};
            api.add_primitive_instance(_ground_instance, GameAPI::PrimitiveType::Plane, ground_tr);

            GameAPI::Transform sphere_tr{};
            sphere_tr.position = _sphere_spawn_pos;
            sphere_tr.scale = {1.0f, 1.0f, 1.0f};
            api.add_primitive_instance(_sphere_instance, GameAPI::PrimitiveType::Sphere, sphere_tr);

            for (const BoxInstance &box : _boxes)
            {
                GameAPI::Transform box_tr{};
                box_tr.position = box.spawn_position;
                box_tr.rotation = box.spawn_rotation;
                box_tr.scale = box.visual_scale;
                api.add_primitive_instance(box.instance_name, GameAPI::PrimitiveType::Cube, box_tr);
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        // Physics (wrapper): floor + box stack + sphere
        _physics = std::make_unique<Physics::JoltPhysicsWorld>();

        _physics->create_body(Physics::BodySettings{}
                                  .set_shape(Physics::CollisionShape::Box(25.0f, 1.0f, 25.0f))
                                  .set_position(0.0f, -1.0f, 0.0f)
                                  .set_static()
                                  .set_friction(0.8f));

        _sphere_body = Physics::BodyBuilder(_physics.get())
                           .sphere(_sphere_radius_m)
                           .position(_sphere_spawn_pos)
                           .dynamic_body()
                           .friction(0.6f)
                           .restitution(0.1f)
                           .linear_damping(0.02f)
                           .build();

        _box_bodies.clear();
        _box_bodies.reserve(_boxes.size());
        _box_interps.clear();
        _box_interps.resize(_boxes.size());
        for (size_t i = 0; i < _boxes.size(); ++i)
        {
            const BoxInstance &box = _boxes[i];
            _box_bodies.push_back(Physics::BodyBuilder(_physics.get())
                                      .box(box.half_extents)
                                      .position(box.spawn_position)
                                      .rotation(box.spawn_rotation)
                                      .dynamic_body()
                                      .friction(0.8f)
                                      .restitution(0.0f)
                                      .linear_damping(0.02f)
                                      .angular_damping(0.05f)
                                      .build());

            // Initialize interpolation state
            _box_interps[i].prev_position = box.spawn_position;
            _box_interps[i].prev_rotation = box.spawn_rotation;
            _box_interps[i].curr_position = box.spawn_position;
            _box_interps[i].curr_rotation = box.spawn_rotation;
        }

        // Initialize sphere interpolation state
        _sphere_interp.prev_position = _sphere_spawn_pos;
        _sphere_interp.prev_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        _sphere_interp.curr_position = _sphere_spawn_pos;
        _sphere_interp.curr_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
#endif

        _fixed_time = 0.0f;
        _sphere_launched = false;
    }

    void on_update(float dt) override
    {
        _elapsed += dt;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics)
        {
            bool reset_requested = false;
            const float alpha = _runtime->interpolation_alpha();

            // Sphere -> visual (interpolated)
            if (_sphere_body.is_valid())
            {
                glm::vec3 pos = _sphere_interp.interpolated_position(alpha);
                glm::quat rot = _sphere_interp.interpolated_rotation(alpha);

                GameAPI::Transform tr{};
                tr.position = pos;
                tr.rotation = rot;
                tr.scale = glm::vec3(1.0f); // keep primitive at unit scale (sphere mesh radius is 0.5)
                _runtime->api().set_mesh_instance_transform(_sphere_instance, tr);

                if (_sphere_interp.curr_position.y < -50.0f)
                {
                    reset_requested = true;
                }
            }

            // Boxes -> visuals (interpolated)
            const size_t count = std::min(_boxes.size(), _box_bodies.size());
            for (size_t i = 0; i < count; ++i)
            {
                const Physics::BodyId body_id = _box_bodies[i];
                if (!body_id.is_valid())
                {
                    continue;
                }

                glm::vec3 pos = _box_interps[i].interpolated_position(alpha);
                glm::quat rot = _box_interps[i].interpolated_rotation(alpha);

                GameAPI::Transform tr{};
                tr.position = pos;
                tr.rotation = rot;
                tr.scale = _boxes[i].visual_scale;
                _runtime->api().set_mesh_instance_transform(_boxes[i].instance_name, tr);

                if (_box_interps[i].curr_position.y < -50.0f)
                {
                    reset_requested = true;
                }
            }

            if (reset_requested)
            {
                reset_stack_scene();
            }
        }
#endif
    }

    void on_fixed_update(float fixed_dt) override
    {
        _fixed_time += fixed_dt;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics)
        {
            // Store previous transforms before physics step
            if (_sphere_body.is_valid())
            {
                _sphere_interp.store_current_as_previous();
            }

            const size_t count = std::min(_boxes.size(), _box_bodies.size());
            for (size_t i = 0; i < count; ++i)
            {
                if (_box_bodies[i].is_valid())
                {
                    _box_interps[i].store_current_as_previous();
                }
            }

            // Step physics
            _physics->step(fixed_dt);

            // Capture new current transforms after physics step
            if (_sphere_body.is_valid())
            {
                Physics::BodyTransform t = _physics->get_transform(_sphere_body);
                _sphere_interp.curr_position = t.position;
                _sphere_interp.curr_rotation = t.rotation;
            }

            for (size_t i = 0; i < count; ++i)
            {
                if (_box_bodies[i].is_valid())
                {
                    Physics::BodyTransform t = _physics->get_transform(_box_bodies[i]);
                    _box_interps[i].curr_position = t.position;
                    _box_interps[i].curr_rotation = t.rotation;
                }
            }
        }

        // Launch a "bowling ball" after the stack has had a moment to settle.
        if (_physics && !_sphere_launched && _sphere_body.is_valid() && _fixed_time >= 10.0f)
        {
            _physics->set_linear_velocity(_sphere_body, glm::vec3(18.0f, 0.0f, 4.0f));
            _sphere_launched = true;
        }
#endif
    }

    void on_shutdown() override
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        _physics.reset();
#endif
        _runtime = nullptr;
    }

private:
    struct BoxInstance
    {
        std::string instance_name;
        glm::vec3 half_extents{0.5f};
        glm::vec3 spawn_position{0.0f};
        glm::quat spawn_rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 visual_scale{1.0f};
    };

    struct InterpolatedTransform
    {
        glm::vec3 prev_position{0.0f};
        glm::quat prev_rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 curr_position{0.0f};
        glm::quat curr_rotation{1.0f, 0.0f, 0.0f, 0.0f};

        glm::vec3 interpolated_position(float alpha) const
        {
            return glm::mix(prev_position, curr_position, alpha);
        }

        glm::quat interpolated_rotation(float alpha) const
        {
            return glm::slerp(prev_rotation, curr_rotation, alpha);
        }

        void store_current_as_previous()
        {
            prev_position = curr_position;
            prev_rotation = curr_rotation;
        }
    };

    void build_box_stack_layout()
    {
        _boxes.clear();

        constexpr int layers = 6; // 6..1 (91 boxes)
        constexpr float gap = 0.02f;

        const glm::vec3 he = _box_half_extents_m;
        const glm::vec3 box_size = he * 2.0f;

        int idx = 0;
        for (int layer = 0; layer < layers; ++layer)
        {
            const int n = layers - layer;
            const float y = he.y + static_cast<float>(layer) * (box_size.y + gap);

            const float layer_extent_x = (static_cast<float>(n - 1)) * (box_size.x + gap) * 0.5f;
            const float layer_extent_z = (static_cast<float>(n - 1)) * (box_size.z + gap) * 0.5f;

            for (int ix = 0; ix < n; ++ix)
            {
                for (int iz = 0; iz < n; ++iz)
                {
                    BoxInstance box{};
                    box.instance_name = "stack_box_" + std::to_string(idx++);
                    box.half_extents = he;
                    box.spawn_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                    box.spawn_position = glm::vec3(
                        static_cast<float>(ix) * (box_size.x + gap) - layer_extent_x,
                        y,
                        static_cast<float>(iz) * (box_size.z + gap) - layer_extent_z);
                    box.visual_scale = box_size;
                    _boxes.push_back(box);
                }
            }
        }

        // A couple of extra towers on the side for variety.
        for (int t = 0; t < 10; ++t)
        {
            BoxInstance box{};
            box.instance_name = "tower_box_" + std::to_string(t);
            box.half_extents = glm::vec3(0.45f, 0.45f, 0.45f);
            box.spawn_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            box.spawn_position = glm::vec3(6.0f, box.half_extents.y + static_cast<float>(t) * (box.half_extents.y * 2.0f + gap), 0.0f);
            box.visual_scale = box.half_extents * 2.0f;
            _boxes.push_back(box);
        }

        for (int t = 0; t < 8; ++t)
        {
            BoxInstance box{};
            box.instance_name = "tower_box_b_" + std::to_string(t);
            box.half_extents = glm::vec3(0.35f, 0.70f, 0.35f);
            box.spawn_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            box.spawn_position = glm::vec3(-6.0f, box.half_extents.y + static_cast<float>(t) * (box.half_extents.y * 2.0f + gap), 1.5f);
            box.visual_scale = box.half_extents * 2.0f;
            _boxes.push_back(box);
        }
    }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
    void reset_stack_scene()
    {
        if (!_physics || !_runtime)
        {
            return;
        }

        if (_sphere_body.is_valid())
        {
            const glm::quat rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            _physics->set_transform(_sphere_body, _sphere_spawn_pos, rot);
            _physics->set_linear_velocity(_sphere_body, glm::vec3(0.0f));
            _physics->set_angular_velocity(_sphere_body, glm::vec3(0.0f));
            _physics->activate(_sphere_body);

            // Reset interpolation state
            _sphere_interp.prev_position = _sphere_spawn_pos;
            _sphere_interp.prev_rotation = rot;
            _sphere_interp.curr_position = _sphere_spawn_pos;
            _sphere_interp.curr_rotation = rot;
        }

        const size_t count = std::min(_boxes.size(), _box_bodies.size());
        for (size_t i = 0; i < count; ++i)
        {
            const Physics::BodyId body_id = _box_bodies[i];
            if (!body_id.is_valid())
            {
                continue;
            }

            _physics->set_transform(body_id, _boxes[i].spawn_position, _boxes[i].spawn_rotation);
            _physics->set_linear_velocity(body_id, glm::vec3(0.0f));
            _physics->set_angular_velocity(body_id, glm::vec3(0.0f));
            _physics->activate(body_id);

            // Reset interpolation state
            _box_interps[i].prev_position = _boxes[i].spawn_position;
            _box_interps[i].prev_rotation = _boxes[i].spawn_rotation;
            _box_interps[i].curr_position = _boxes[i].spawn_position;
            _box_interps[i].curr_rotation = _boxes[i].spawn_rotation;
        }

        // Snap visuals to spawn transforms immediately (avoids a frame of "old" transforms).
        auto &api = _runtime->api();
        {
            GameAPI::Transform sphere_tr{};
            sphere_tr.position = _sphere_spawn_pos;
            sphere_tr.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            sphere_tr.scale = glm::vec3(1.0f);
            api.set_mesh_instance_transform(_sphere_instance, sphere_tr);
        }

        for (const BoxInstance &box : _boxes)
        {
            GameAPI::Transform box_tr{};
            box_tr.position = box.spawn_position;
            box_tr.rotation = box.spawn_rotation;
            box_tr.scale = box.visual_scale;
            api.set_mesh_instance_transform(box.instance_name, box_tr);
        }

        _fixed_time = 0.0f;
        _sphere_launched = false;
    }
#endif

    GameRuntime::Runtime *_runtime{nullptr};

    float _elapsed{0.0f};
    float _fixed_time{0.0f};
    bool _sphere_launched{false};

    std::vector<BoxInstance> _boxes;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
    std::unique_ptr<Physics::JoltPhysicsWorld> _physics;
    Physics::BodyId _sphere_body;
    std::vector<Physics::BodyId> _box_bodies;

    InterpolatedTransform _sphere_interp;
    std::vector<InterpolatedTransform> _box_interps;
#endif

    static constexpr const char *_ground_instance = "physics_ground";
    static constexpr const char *_sphere_instance = "physics_sphere";

    static constexpr float _sphere_radius_m = 0.5f;

    static inline const glm::vec3 _box_half_extents_m = glm::vec3(0.5f, 0.5f, 0.5f);
    static inline const glm::vec3 _sphere_spawn_pos = glm::vec3(-14.0f, 2.0f, 0.0f);
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
