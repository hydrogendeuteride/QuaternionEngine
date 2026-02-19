#include "example_game.h"
#include "runtime/game_runtime.h"
#include "core/game_api.h"
#include "core/engine.h"

#include "imgui.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

#include <fmt/core.h>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <cmath>

namespace Game
{
    const char *contact_event_type_name(Physics::ContactEventType t)
    {
        switch (t)
        {
            case Physics::ContactEventType::Begin: return "Begin";
            case Physics::ContactEventType::Stay: return "Stay";
            case Physics::ContactEventType::End: return "End";
        }
        return "Unknown";
    }

    void ExampleGame::on_init(GameRuntime::Runtime &runtime)
    {
        _runtime = &runtime;
        auto &api = runtime.api();
        _world.set_api(&api);
        _audio_test_event = "assets/sounds/dripping.mp3";
        _audio_test_preloaded = false;
        _audio_test_last_status.clear();

        // Setup camera
        VulkanEngine *renderer = runtime.renderer();
        if (renderer && renderer->_assetManager)
        {
            GameAPI::IBLPaths ibl{};
            ibl.specularCube = renderer->_assetManager->assetPath("ibl/sky.ktx2");
            ibl.diffuseCube = renderer->_assetManager->assetPath("ibl/sky.ktx2");
            ibl.brdfLut = renderer->_assetManager->assetPath("ibl/brdf_lut.ktx2");
            ibl.background = renderer->_assetManager->assetPath("ibl/sky.ktx2");
            api.load_global_ibl(ibl);

            _audio_test_event = renderer->_assetManager->assetPath("sounds/dripping.mp3");
        }

        api.set_camera_position(glm::vec3(-15.0f, 6.0f, 0.0f));
        api.camera_look_at(glm::vec3(1.0f, 0.0f, 0.0f));

        // Build layout data
        build_box_stack_layout();

        // Setup game scene (entities + render/physics resources)
        setup_scene();
        spawn_test_decals();

        // Rocket-plume style Mesh VFX demo:
        // - no albedo texture (falls back to white)
        // - two layered capsule instances (outer cone + bright core)
        // - scrolling dual-noise + UV gradient + fresnel
        {
            GameAPI::MeshVfxMaterialSettings outer{};
            outer.albedoPath = ""; // intentionally empty: no albedo texture
            outer.noise1Path = "vfx/perlin.ktx2";
            outer.noise2Path = "vfx/simplex.ktx2";
            outer.noise1SRGB = false;
            outer.noise2SRGB = false;
            outer.tint = glm::vec3(1.0f, 0.92f, 0.86f);
            outer.opacity = 0.72f;
            outer.fresnelPower = 2.2f;
            outer.fresnelStrength = 1.35f;
            outer.scrollVelocity1 = glm::vec2(0.02f, -2.6f);
            outer.scrollVelocity2 = glm::vec2(-0.03f, -1.35f);
            outer.distortionStrength = 0.19f;
            outer.noiseBlend = 0.35f;
            outer.coreColor = glm::vec3(0.95f, 0.98f, 1.05f);
            outer.edgeColor = glm::vec3(1.0f, 0.38f, 0.06f);
            outer.gradientAxis = 1.0f;
            outer.gradientStart = 0.04f;
            outer.gradientEnd = 0.92f;
            outer.emissionStrength = 3.8f;

            GameAPI::MeshVfxMaterialSettings inner{};
            inner.albedoPath = ""; // intentionally empty: no albedo texture
            inner.noise1Path = "vfx/simplex.ktx2";
            inner.noise2Path = "vfx/perlin.ktx2";
            inner.noise1SRGB = false;
            inner.noise2SRGB = false;
            inner.tint = glm::vec3(0.88f, 0.98f, 1.2f);
            inner.opacity = 0.95f;
            inner.fresnelPower = 1.35f;
            inner.fresnelStrength = 0.6f;
            inner.scrollVelocity1 = glm::vec2(-0.01f, -3.8f);
            inner.scrollVelocity2 = glm::vec2(0.01f, -2.1f);
            inner.distortionStrength = 0.08f;
            inner.noiseBlend = 0.58f;
            inner.coreColor = glm::vec3(1.2f, 1.45f, 1.9f);
            inner.edgeColor = glm::vec3(0.62f, 0.9f, 1.35f);
            inner.gradientAxis = 1.0f;
            inner.gradientStart = 0.02f;
            inner.gradientEnd = 0.68f;
            inner.emissionStrength = 6.2f;

            const bool outer_ok = api.create_or_update_mesh_vfx_material(_plume_outer_material_name, outer);
            const bool inner_ok = api.create_or_update_mesh_vfx_material(_plume_inner_material_name, inner);
            if (outer_ok && inner_ok)
            {
                const glm::vec3 nozzle = glm::vec3(_plume_nozzle_pos);

                GameAPI::Transform outer_tr{};
                outer_tr.position = nozzle + glm::vec3(0.0f, -1.1f, 0.0f);
                outer_tr.scale = glm::vec3(0.34f, 1.55f, 0.34f);

                GameAPI::Transform inner_tr{};
                inner_tr.position = nozzle + glm::vec3(0.0f, -0.85f, 0.0f);
                inner_tr.scale = glm::vec3(0.18f, 1.1f, 0.18f);

                const bool spawned_outer = api.add_primitive_instance(_plume_outer_instance_name,
                                                                       GameAPI::PrimitiveType::Capsule,
                                                                       outer_tr);
                const bool applied_outer = spawned_outer && api.apply_mesh_vfx_material_to_primitive(
                    _plume_outer_instance_name,
                    _plume_outer_material_name);

                const bool spawned_inner = api.add_primitive_instance(_plume_inner_instance_name,
                                                                       GameAPI::PrimitiveType::Capsule,
                                                                       inner_tr);
                const bool applied_inner = spawned_inner && api.apply_mesh_vfx_material_to_primitive(
                    _plume_inner_instance_name,
                    _plume_inner_material_name);

                _plume_spawned = applied_outer && applied_inner;
                if (!_plume_spawned)
                {
                    if (spawned_outer) api.remove_mesh_instance(_plume_outer_instance_name);
                    if (spawned_inner) api.remove_mesh_instance(_plume_inner_instance_name);
                }
            }
        }

        // Blackbody hot-metal demo:
        // - nozzle/barrel style heat profile in object space
        // - triplanar + axial streak noise modulates breakup/turbulence
        // - emissiveTex is repurposed as a noise texture when enabled
        {
            GameAPI::BlackbodyMaterialSettings bb{};
            bb.colorFactor = glm::vec4(0.05f, 0.05f, 0.05f, 1.0f);
            bb.metallic = 0.1f;
            bb.roughness = 0.72f;
            bb.normalScale = 1.0f;

            bb.blackbody.noisePath = "vfx/simplex.ktx2";
            bb.blackbody.intensity = 1.0f;
            bb.blackbody.tempMinK = 1000.0f;
            bb.blackbody.tempMaxK = 1600.0f;
            bb.blackbody.noiseScale = 1.0f;
            bb.blackbody.noiseContrast = 1.0f;
            bb.blackbody.noiseScroll = glm::vec2(1.0f, -1.0f);
            bb.blackbody.noiseSpeed = 0.0f;
            bb.blackbody.heatAxisLocal = glm::vec3(0.0f, 1.0f, 0.0f);
            bb.blackbody.hotEndBias = -1.0f;
            bb.blackbody.hotRangeStart = 0.52f;
            bb.blackbody.hotRangeEnd = 0.98f;

            const bool mat_ok = api.create_or_update_blackbody_material(_blackbody_material_name, bb);
            if (mat_ok)
            {
                GameAPI::Transform tr{};
                tr.position = glm::vec3(0.5f, 1.45f, -4.0f);
                tr.scale = glm::vec3(0.32f, 1.35f, 0.32f);

                const bool spawned = api.add_primitive_instance(_blackbody_instance_name,
                                                               GameAPI::PrimitiveType::Capsule,
                                                               tr);
                const bool applied = spawned && api.apply_blackbody_material_to_primitive(
                    _blackbody_instance_name,
                    _blackbody_material_name);

                _blackbody_spawned = applied;
                if (!_blackbody_spawned && spawned)
                {
                    api.remove_mesh_instance(_blackbody_instance_name);
                }
            }
        }

        _world.set_rebase_anchor(_sphere_entity);
        _world.set_rebase_settings(GameWorld::RebaseSettings{
            .origin_threshold_m = 500.0,
            .origin_snap_m = 100.0,
            .velocity_threshold_mps = 1000.0
        });

        // Game ImGui panels
        if (VulkanEngine *renderer = runtime.renderer())
        {
            if (renderer->ui())
            {
                renderer->ui()->addDrawCallback([this]() { draw_contact_debug_ui(); });
            }
        }

        // Minimal ImGui image sample: load a texture from assets/textures.
        GameAPI::TextureLoadParams ui_tex_params{};
        ui_tex_params.srgb = true;
        ui_tex_params.mipmapped = false;
        _imgui_example_texture = api.load_texture("grass_albedo.png", ui_tex_params);
        if (_imgui_example_texture != GameAPI::InvalidTexture)
        {
            api.pin_texture(_imgui_example_texture);
        }

        if (auto *audio = runtime.audio())
        {
            _audio_test_preloaded = audio->preload(_audio_test_event);
            if (_audio_test_preloaded)
            {
                const auto sound = audio->play_2d(_audio_test_event,
                                                  GameRuntime::IAudioSystem::Bus::Sfx,
                                                  _audio_test_volume,
                                                  _audio_test_pitch,
                                                  false);
                _audio_test_last_status = (sound != GameRuntime::IAudioSystem::INVALID_SOUND_HANDLE)
                                              ? "Init: played dripping.mp3"
                                              : "Init: play failed";
            }
            else
            {
                _audio_test_last_status = "Init: preload failed";
                fmt::println("[ExampleGame] Failed to preload audio test '{}'", _audio_test_event);
            }
        }
        else
        {
            _audio_test_last_status = "Audio system not available";
        }

        _fixed_time = 0.0f;
        _sphere_launched = false;
    }

    void ExampleGame::on_update(float dt)
    {
        _elapsed += dt;

        if (!_runtime)
        {
            return;
        }

        const float alpha = _runtime->interpolation_alpha();
        auto &api = _runtime->api();

        if (_plume_spawned)
        {
            // Throttle-like pulse on geometry while shader time drives UV/noise animation.
            float throttle = 0.82f
                + 0.16f * std::sin(_elapsed * 2.9f)
                + 0.08f * std::sin(_elapsed * 11.0f);
            throttle = std::clamp(throttle, 0.35f, 1.2f);

            const glm::vec3 nozzle = glm::vec3(_plume_nozzle_pos);
            const float jitter_x = 0.02f * std::sin(_elapsed * 17.0f);
            const float jitter_z = 0.02f * std::sin(_elapsed * 14.0f + 0.7f);

            GameAPI::Transform outer_tr{};
            outer_tr.position = nozzle;
            outer_tr.scale = glm::vec3(0.34f + 0.05f * throttle,
                                       1.35f + 1.05f * throttle,
                                       0.34f + 0.05f * throttle);

            GameAPI::Transform inner_tr{};
            inner_tr.position = nozzle;
            inner_tr.scale = glm::vec3(0.16f + 0.025f * throttle,
                                       1.0f + 0.82f * throttle,
                                       0.16f + 0.025f * throttle);

            api.set_mesh_instance_transform(_plume_outer_instance_name, outer_tr);
            api.set_mesh_instance_transform(_plume_inner_instance_name, inner_tr);
        }

        // Sync all entities to render (world-space, double precision)
        _world.entities().sync_to_render(api, alpha);

        // Check for reset condition
        bool reset_requested = false;

        if (Entity *sphere = _world.entities().find(_sphere_entity))
        {
            if (sphere->position_world().y < -50.0)
            {
                reset_requested = true;
            }
        }

        for (EntityId box_id: _box_entities)
        {
            if (Entity *box = _world.entities().find(box_id))
            {
                if (box->position_world().y < -50.0)
                {
                    reset_requested = true;
                    break;
                }
            }
        }

        if (reset_requested)
        {
            reset_scene();
        }
    }

    void ExampleGame::on_fixed_update(float fixed_dt)
    {
        _fixed_time += fixed_dt;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_runtime || !_physics)
        {
            return;
        }

        // Pre-physics: interpolation + automatic rebasing (if configured)
        _world.pre_physics_step();

        // Step physics
        _physics->step(fixed_dt);

        // Post-physics: update entity transforms from physics
        _world.post_physics_step();

        // Launch bowling ball after settling
        if (!_sphere_launched && _fixed_time >= 10.0f)
        {
            Entity *sphere = _world.entities().find(_sphere_entity);
            if (sphere && sphere->has_physics())
            {
                Physics::BodyId body_id{sphere->physics_body_value()};
                _physics->set_linear_velocity(body_id, glm::vec3(18.0f, 0.0f, 4.0f));
                _sphere_launched = true;
            }
        }
#endif
    }

    void ExampleGame::on_shutdown()
    {
        if (_runtime)
        {
            auto &api = _runtime->api();
            clear_test_decals();
            if (_imgui_example_texture_id != nullptr)
            {
                api.free_imgui_texture(_imgui_example_texture_id);
                _imgui_example_texture_id = nullptr;
            }
            if (_imgui_example_texture != GameAPI::InvalidTexture)
            {
                api.unpin_texture(_imgui_example_texture);
                api.unload_texture(_imgui_example_texture);
                _imgui_example_texture = GameAPI::InvalidTexture;
            }
            if (_plume_spawned)
            {
                api.remove_mesh_instance(_plume_outer_instance_name);
                api.remove_mesh_instance(_plume_inner_instance_name);
                _plume_spawned = false;
            }
            (void)api.remove_mesh_vfx_material(_plume_outer_material_name);
            (void)api.remove_mesh_vfx_material(_plume_inner_material_name);

            if (_blackbody_spawned)
            {
                api.remove_mesh_instance(_blackbody_instance_name);
                _blackbody_spawned = false;
            }
            (void)api.remove_blackbody_material(_blackbody_material_name);

            if (auto *audio = _runtime->audio())
            {
                if (_audio_test_preloaded)
                {
                    audio->unload(_audio_test_event);
                }
            }
        }
        _audio_test_preloaded = false;
        _audio_test_last_status.clear();

        _world.clear_rebase_anchor();

        _world.clear();
        _world.set_physics(nullptr);
        _world.set_physics_context(nullptr);
        _world.set_api(nullptr);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_runtime)
        {
            if (VulkanEngine *renderer = _runtime->renderer())
            {
                if (renderer->_context)
                {
                    if (renderer->_context->physics_context == _physics_context.get())
                    {
                        renderer->_context->physics_context = nullptr;
                    }
                }
            }
        }
        if (_physics && _ground_collider_body.is_valid())
        {
            _physics->destroy_body(_ground_collider_body);
            _ground_collider_body = Physics::BodyId{};
        }
        _physics_context.reset();
        _physics.reset();
#endif

        _runtime = nullptr;
    }

    void ExampleGame::setup_scene()
    {
        if (!_runtime)
        {
            return;
        }

        _ground_entity = EntityId{};
        _sphere_entity = EntityId{};
        _box_entities.clear();
        _initial_pose.clear();
        _ground_collider_body = Physics::BodyId{};

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        _physics = std::make_unique<Physics::JoltPhysicsWorld>();

        _physics_context = std::make_unique<Physics::PhysicsContext>();
        _physics_context->set_physics_world(_physics.get());

        _world.set_physics(_physics.get());
        _world.set_physics_context(_physics_context.get());
#else
        _physics.reset();
        _physics_context.reset();
        _world.set_physics(nullptr);
        _world.set_physics_context(nullptr);
#endif

        // Expose to EngineContext for debug systems (physics debug draw, etc.)
        if (VulkanEngine *renderer = _runtime->renderer())
        {
            if (renderer->_context)
            {
                renderer->_context->physics_context = _physics_context.get();
            }
        }

        // Ground (render)
        {
            Transform tr{};
            tr.position_world = {0.0, 0.0, 0.0};
            tr.scale = {50.0f, 1.0f, 50.0f};
            if (Entity *ground = _world.builder("ground")
                    .transform(tr)
                    .render_primitive(GameAPI::PrimitiveType::Plane)
                    .build())
            {
                _ground_entity = ground->id();
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        // Ground (physics collider; placed so the top surface is at y=0)
        if (_physics && _ground_entity.is_valid())
        {
            _ground_collider_body = _physics->create_body(Physics::BodySettings{}
                .set_shape(Physics::CollisionShape::Box(25.0f, 1.0f, 25.0f))
                .set_position(0.0f, -1.0f, 0.0f)
                .set_user_data(static_cast<uint64_t>(_ground_entity.value))
                .set_static()
                .set_friction(0.8f));
        }
#endif

        // Sphere
        {
            Transform tr{};
            tr.position_world = WorldVec3(SPHERE_SPAWN_POS);
            tr.scale = {1.0f, 1.0f, 1.0f};

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics)
            {
                Physics::BodySettings settings{};
                settings.set_shape(Physics::CollisionShape::Sphere(SPHERE_RADIUS))
                        .set_dynamic()
                        .set_friction(0.6f)
                        .set_restitution(0.1f)
                        .set_linear_damping(0.02f);

                if (Entity *sphere = _world.builder("sphere")
                        .transform(tr)
                        .render_primitive(GameAPI::PrimitiveType::Sphere)
                        .physics(settings)
                        .build())
                {
                    _sphere_entity = sphere->id();
                    _initial_pose[_sphere_entity.value] = InitialPose{tr.position_world, tr.rotation};
                }
            }
            else
#endif
            {
                if (Entity *sphere = _world.builder("sphere")
                        .transform(tr)
                        .render_primitive(GameAPI::PrimitiveType::Sphere)
                        .build())
                {
                    _sphere_entity = sphere->id();
                    _initial_pose[_sphere_entity.value] = InitialPose{tr.position_world, tr.rotation};
                }
            }
        }

        // Boxes
        _box_entities.reserve(_box_layouts.size());

        for (const BoxLayout &layout: _box_layouts)
        {
            Transform tr{};
            tr.position_world = WorldVec3(layout.position);
            tr.rotation = layout.rotation;
            tr.scale = layout.half_extents * 2.0f;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics)
            {
                Physics::BodySettings settings{};
                settings.set_shape(Physics::CollisionShape::Box(layout.half_extents))
                        .set_dynamic()
                        .set_friction(0.8f)
                        .set_restitution(0.0f)
                        .set_linear_damping(0.02f)
                        .set_angular_damping(0.05f);

                if (Entity *box = _world.builder(layout.name)
                        .transform(tr)
                        .render_primitive(GameAPI::PrimitiveType::Cube)
                        .physics(settings)
                        .build())
                {
                    _box_entities.push_back(box->id());
                    _initial_pose[box->id().value] = InitialPose{tr.position_world, tr.rotation};
                }
                continue;
            }
#endif

            if (Entity *box = _world.builder(layout.name)
                    .transform(tr)
                    .render_primitive(GameAPI::PrimitiveType::Cube)
                    .build())
            {
                _box_entities.push_back(box->id());
                _initial_pose[box->id().value] = InitialPose{tr.position_world, tr.rotation};
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        install_contact_callbacks();
#endif
    }

    void ExampleGame::spawn_test_decals()
    {
        if (!_runtime)
        {
            return;
        }

        auto &api = _runtime->api();
        clear_test_decals();
        if (!api.get_debug_draw_enabled())
        {
            api.set_debug_draw_enabled(true);
        }

        GameAPI::TextureLoadParams albedo_params{};
        albedo_params.srgb = true;
        albedo_params.mipmapped = true;
        _decal_albedo_texture = api.load_texture("velvet.png", albedo_params);
        if (_decal_albedo_texture != GameAPI::InvalidTexture)
        {
            api.pin_texture(_decal_albedo_texture);
        }

        GameAPI::TextureLoadParams normal_params{};
        normal_params.srgb = false;
        normal_params.mipmapped = true;
        normal_params.channels = GameAPI::TextureChannels::RG;
        _decal_normal_texture = api.load_texture("velvet_normal.png", normal_params);
        if (_decal_normal_texture != GameAPI::InvalidTexture)
        {
            api.pin_texture(_decal_normal_texture);
        }

        GameAPI::Decal box{};
        box.shape = GameAPI::DecalShape::Box;
        box.position = glm::dvec3(-1.5, 0.6, -1.0);
        box.rotation = glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        box.halfExtents = glm::vec3(2.2f, 2.2f, 0.8f);
        box.albedoTexture = _decal_albedo_texture;
        box.normalTexture = _decal_normal_texture;
        box.tint = glm::vec3(1.0f, 0.95f, 0.95f);
        box.opacity = 0.9f;
        box.normalStrength = 1.0f;

        GameAPI::Decal sphere{};
        sphere.shape = GameAPI::DecalShape::Sphere;
        sphere.position = glm::dvec3(3.0, 1.4, 0.5);
        sphere.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        sphere.halfExtents = glm::vec3(1.2f, 1.2f, 1.2f);
        sphere.albedoTexture = _decal_albedo_texture;
        sphere.normalTexture = _decal_normal_texture;
        sphere.tint = glm::vec3(0.8f, 0.9f, 1.0f);
        sphere.opacity = 0.8f;
        sphere.normalStrength = 0.9f;

        const bool box_ok = api.set_decal(_decal_box_name, box);
        const bool sphere_ok = api.set_decal(_decal_sphere_name, sphere);
        _decal_test_spawned = box_ok || sphere_ok;

        if (!_decal_test_spawned)
        {
            fmt::println("[ExampleGame] Failed to spawn decal test fixtures.");
        }
    }

    void ExampleGame::clear_test_decals()
    {
        if (!_runtime)
        {
            return;
        }

        auto &api = _runtime->api();
        (void) api.remove_decal(_decal_box_name);
        (void) api.remove_decal(_decal_sphere_name);
        _decal_test_spawned = false;

        if (_decal_albedo_texture != GameAPI::InvalidTexture)
        {
            api.unpin_texture(_decal_albedo_texture);
            api.unload_texture(_decal_albedo_texture);
            _decal_albedo_texture = GameAPI::InvalidTexture;
        }
        if (_decal_normal_texture != GameAPI::InvalidTexture)
        {
            api.unpin_texture(_decal_normal_texture);
            api.unload_texture(_decal_normal_texture);
            _decal_normal_texture = GameAPI::InvalidTexture;
        }
    }

    void ExampleGame::build_box_stack_layout()
    {
        _box_layouts.clear();

        constexpr int layers = 6;
        constexpr float gap = 0.02f;
        const glm::vec3 he = BOX_HALF_EXTENTS;
        const glm::vec3 box_size = he * 2.0f;

        int idx = 0;

        // Pyramid stack
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
                    BoxLayout layout;
                    layout.name = "stack_box_" + std::to_string(idx++);
                    layout.half_extents = he;
                    layout.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                    layout.position = glm::vec3(
                        static_cast<float>(ix) * (box_size.x + gap) - layer_extent_x,
                        y,
                        static_cast<float>(iz) * (box_size.z + gap) - layer_extent_z);
                    _box_layouts.push_back(layout);
                }
            }
        }

        // Side tower A
        for (int t = 0; t < 10; ++t)
        {
            BoxLayout layout;
            layout.name = "tower_box_" + std::to_string(t);
            layout.half_extents = glm::vec3(0.45f);
            layout.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            layout.position = glm::vec3(
                6.0f, layout.half_extents.y + static_cast<float>(t) * (layout.half_extents.y * 2.0f + gap), 0.0f);
            _box_layouts.push_back(layout);
        }

        // Side tower B
        for (int t = 0; t < 8; ++t)
        {
            BoxLayout layout;
            layout.name = "tower_box_b_" + std::to_string(t);
            layout.half_extents = glm::vec3(0.35f, 0.70f, 0.35f);
            layout.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            layout.position = glm::vec3(
                -6.0f, layout.half_extents.y + static_cast<float>(t) * (layout.half_extents.y * 2.0f + gap), 1.5f);
            _box_layouts.push_back(layout);
        }
    }

    void ExampleGame::reset_scene()
    {
        if (!_runtime)
        {
            return;
        }

        auto &api = _runtime->api();
        auto &entities = _world.entities();

        const auto reset_entity_pose = [&](EntityId id) {
            Entity *ent = entities.find(id);
            if (!ent)
            {
                return;
            }

            WorldVec3 pos_world = ent->position_world();
            glm::quat rot = ent->rotation();
            if (auto it = _initial_pose.find(id.value); it != _initial_pose.end())
            {
                pos_world = it->second.position_world;
                rot = it->second.rotation;
            }
            else if (id == _sphere_entity)
            {
                pos_world = WorldVec3(SPHERE_SPAWN_POS);
                rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            }

            if (_physics)
            {
                const WorldVec3 physics_origin_world =
                        (_physics_context ? _physics_context->origin_world() : WorldVec3{0.0, 0.0, 0.0});
                entities.teleport(id, pos_world, rot, *_physics, physics_origin_world);
            }
            else
            {
                ent->set_position_world(pos_world);
                ent->set_rotation(rot);
                if (ent->uses_interpolation())
                {
                    ent->interpolation().set_immediate(pos_world, rot);
                }
            }
        };

        reset_entity_pose(_sphere_entity);
        for (EntityId id: _box_entities)
        {
            reset_entity_pose(id);
        }

        // Immediate sync to render to avoid visual blending
        entities.sync_to_render(api, 1.0f);

        _fixed_time = 0.0f;
        _sphere_launched = false;
    }

    void ExampleGame::install_contact_callbacks()
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_physics)
        {
            return;
        }

        Physics::PhysicsWorld::BodyCallbacks callbacks{};
        callbacks.on_collision = [this](const Physics::CollisionEvent &e) {
            if (!_contact_log_enabled)
            {
                return;
            }
            if (e.type == Physics::ContactEventType::Stay && !_contact_log_stay)
            {
                return;
            }

            ContactLogEntry entry{};
            entry.is_trigger = false;
            entry.type = e.type;
            entry.self_body = e.self.value;
            entry.other_body = e.other.value;
            entry.self_user_data = e.self_user_data;
            entry.other_user_data = e.other_user_data;
            entry.self_layer = e.self_layer;
            entry.other_layer = e.other_layer;
            entry.point = e.point;
            entry.normal = e.normal;
            entry.penetration_depth = e.penetration_depth;
            entry.time = _fixed_time;
            entry.self_label = make_body_label(entry.self_body, entry.self_user_data);
            entry.other_label = make_body_label(entry.other_body, entry.other_user_data);

            _contact_log.push_back(entry);
            while (_contact_log.size() > _contact_log_capacity)
            {
                _contact_log.pop_front();
            }

            if (_contact_print_console)
            {
                fmt::println(
                    "[Collision][{}][{}] {}({}) <-> {}({}) p=({:.2f},{:.2f},{:.2f}) n=({:.2f},{:.2f},{:.2f}) depth={:.3f}",
                    contact_event_type_name(entry.type),
                    entry.time,
                    entry.self_label, entry.self_body,
                    entry.other_label, entry.other_body,
                    entry.point.x, entry.point.y, entry.point.z,
                    entry.normal.x, entry.normal.y, entry.normal.z,
                    entry.penetration_depth);
            }
        };

        callbacks.on_trigger = [this](const Physics::TriggerEvent &e) {
            if (!_contact_log_enabled || !_contact_log_triggers)
            {
                return;
            }
            if (e.type == Physics::ContactEventType::Stay && !_contact_log_stay)
            {
                return;
            }

            ContactLogEntry entry{};
            entry.is_trigger = true;
            entry.type = e.type;
            entry.self_body = e.self.value;
            entry.other_body = e.other.value;
            entry.self_user_data = e.self_user_data;
            entry.other_user_data = e.other_user_data;
            entry.self_layer = e.self_layer;
            entry.other_layer = e.other_layer;
            entry.point = e.point;
            entry.normal = glm::vec3(0.0f);
            entry.penetration_depth = 0.0f;
            entry.time = _fixed_time;
            entry.self_label = make_body_label(entry.self_body, entry.self_user_data);
            entry.other_label = make_body_label(entry.other_body, entry.other_user_data);

            _contact_log.push_back(entry);
            while (_contact_log.size() > _contact_log_capacity)
            {
                _contact_log.pop_front();
            }

            if (_contact_print_console)
            {
                fmt::println("[Trigger][{}][{}] {}({}) <-> {}({}) p=({:.2f},{:.2f},{:.2f})",
                             contact_event_type_name(entry.type),
                             entry.time,
                             entry.self_label, entry.self_body,
                             entry.other_label, entry.other_body,
                             entry.point.x, entry.point.y, entry.point.z);
            }
        };

        auto install_for_entity = [&](EntityId id) {
            Entity *ent = _world.entities().find(id);
            if (!ent || !ent->has_physics())
            {
                return;
            }

            Physics::BodyId body_id{ent->physics_body_value()};
            if (!_physics->is_body_valid(body_id))
            {
                return;
            }

            _physics->set_body_callbacks(body_id, callbacks);
        };

        auto clear_for_entity = [&](EntityId id) {
            Entity *ent = _world.entities().find(id);
            if (!ent || !ent->has_physics())
            {
                return;
            }

            Physics::BodyId body_id{ent->physics_body_value()};
            if (!_physics->is_body_valid(body_id))
            {
                return;
            }

            _physics->clear_body_callbacks(body_id);
        };

        clear_for_entity(_sphere_entity);
        for (EntityId box_id: _box_entities)
        {
            clear_for_entity(box_id);
        }

        install_for_entity(_sphere_entity);

        if (_contact_callbacks_all_bodies)
        {
            for (EntityId box_id: _box_entities)
            {
                install_for_entity(box_id);
            }
        }

        _contact_callbacks_installed_all_bodies = _contact_callbacks_all_bodies;
#endif
    }

    std::string ExampleGame::make_body_label(uint32_t body_value, uint64_t user_data) const
    {
        if (user_data != 0)
        {
            if (const Entity *ent = _world.entities().find(EntityId{static_cast<uint32_t>(user_data)}))
            {
                if (!ent->name().empty())
                {
                    return ent->name();
                }
            }
        }

        return fmt::format("body_{}", body_value);
    }

    void ExampleGame::draw_contact_debug_ui()
    {
        if (!_contact_ui_open)
        {
            return;
        }

        if (!ImGui::Begin("Physics Contacts", &_contact_ui_open))
        {
            ImGui::End();
            return;
        }

        ImGui::Checkbox("Log Enabled", &_contact_log_enabled);
        ImGui::SameLine();
        ImGui::Checkbox("Print Console", &_contact_print_console);

        ImGui::Checkbox("Include Stay", &_contact_log_stay);
        ImGui::SameLine();
        ImGui::Checkbox("Include Triggers", &_contact_log_triggers);

        if (ImGui::Checkbox("Callbacks: All Bodies", &_contact_callbacks_all_bodies))
        {
            install_contact_callbacks();
        }

        int cap = static_cast<int>(_contact_log_capacity);
        if (ImGui::SliderInt("Max Entries", &cap, 16, 2048))
        {
            if (cap < 16)
            {
                cap = 16;
            }
            _contact_log_capacity = static_cast<size_t>(cap);
            while (_contact_log.size() > _contact_log_capacity)
            {
                _contact_log.pop_front();
            }
        }

        if (ImGui::Button("Clear"))
        {
            _contact_log.clear();
        }

        ImGui::SameLine();
        ImGui::Text("Entries: %d", static_cast<int>(_contact_log.size()));

        if (_contact_callbacks_installed_all_bodies != _contact_callbacks_all_bodies)
        {
            install_contact_callbacks();
        }

        ImGui::Separator();
        if (ImGui::CollapsingHeader("ImGui Texture Example", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextUnformatted("Texture: assets/textures/grass_albedo.png");

            if (_runtime && _imgui_example_texture != GameAPI::InvalidTexture)
            {
                auto &api = _runtime->api();
                if (_imgui_example_texture_id == nullptr && api.is_texture_loaded(_imgui_example_texture))
                {
                    _imgui_example_texture_id = api.create_imgui_texture(_imgui_example_texture);
                }

                if (_imgui_example_texture_id != nullptr)
                {
                    ImGui::Image(_imgui_example_texture_id, ImVec2(256.0f, 256.0f));
                }
                else
                {
                    ImGui::TextUnformatted("Loading texture...");
                }
            }
            else
            {
                ImGui::TextUnformatted("Texture handle is invalid.");
            }
        }

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Audio Playback Test", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Text("Sound: %s", _audio_test_event.c_str());
            ImGui::SliderFloat("Audio Volume", &_audio_test_volume, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("Audio Pitch", &_audio_test_pitch, 0.25f, 2.0f, "%.2f");

            GameRuntime::IAudioSystem *audio = (_runtime != nullptr) ? _runtime->audio() : nullptr;
            if (audio != nullptr)
            {
                if (ImGui::Button("Preload##dripping"))
                {
                    _audio_test_preloaded = audio->preload(_audio_test_event);
                    _audio_test_last_status = _audio_test_preloaded ? "Preload succeeded" : "Preload failed";
                }
                ImGui::SameLine();
                if (ImGui::Button("Unload##dripping"))
                {
                    audio->unload(_audio_test_event);
                    _audio_test_preloaded = false;
                    _audio_test_last_status = "Unloaded";
                }

                if (ImGui::Button("Play 2D##dripping"))
                {
                    const auto sound = audio->play_2d(_audio_test_event,
                                                      GameRuntime::IAudioSystem::Bus::Sfx,
                                                      _audio_test_volume,
                                                      _audio_test_pitch,
                                                      false);
                    _audio_test_last_status = (sound != GameRuntime::IAudioSystem::INVALID_SOUND_HANDLE)
                                                  ? "Played 2D"
                                                  : "Play 2D failed";
                }
                ImGui::SameLine();
                if (ImGui::Button("Play 3D @ Sphere##dripping"))
                {
                    glm::vec3 sound_pos = SPHERE_SPAWN_POS;
                    if (const Entity *sphere = _world.entities().find(_sphere_entity))
                    {
                        const WorldVec3 p = sphere->position_world();
                        sound_pos = glm::vec3(static_cast<float>(p.x),
                                              static_cast<float>(p.y),
                                              static_cast<float>(p.z));
                    }

                    const auto sound = audio->play_3d(_audio_test_event,
                                                      sound_pos,
                                                      GameRuntime::IAudioSystem::Bus::Sfx,
                                                      _audio_test_volume,
                                                      _audio_test_pitch);
                    _audio_test_last_status = (sound != GameRuntime::IAudioSystem::INVALID_SOUND_HANDLE)
                                                  ? "Played 3D at sphere"
                                                  : "Play 3D failed";
                }
            }
            else
            {
                ImGui::TextUnformatted("Audio system not available.");
            }

            if (!_audio_test_last_status.empty())
            {
                ImGui::Text("Status: %s", _audio_test_last_status.c_str());
            }
        }

        ImGui::Separator();

        if (ImGui::BeginChild("contact_log", ImVec2(0, 0), true))
        {
            for (const ContactLogEntry &e: _contact_log)
            {
                const char *kind = e.is_trigger ? "Trigger" : "Collision";
                ImGui::Text("[%6.2f] %s %s: %s(%u) <-> %s(%u)",
                            e.time,
                            kind,
                            contact_event_type_name(e.type),
                            e.self_label.c_str(), e.self_body,
                            e.other_label.c_str(), e.other_body);

                if (!e.is_trigger)
                {
                    ImGui::Text("   p(%.2f %.2f %.2f)  n(%.2f %.2f %.2f)  depth=%.3f",
                                e.point.x, e.point.y, e.point.z,
                                e.normal.x, e.normal.y, e.normal.z,
                                e.penetration_depth);
                }
                else
                {
                    ImGui::Text("   p(%.2f %.2f %.2f)", e.point.x, e.point.y, e.point.z);
                }
            }
        }
        ImGui::EndChild();

        ImGui::End();
    }
} // namespace Game
