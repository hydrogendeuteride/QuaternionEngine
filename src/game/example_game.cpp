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

namespace Game
{
    namespace
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
    } // namespace

    void ExampleGame::on_init(GameRuntime::Runtime &runtime)
    {
        _runtime = &runtime;
        auto &api = runtime.api();
        _world.set_api(&api);

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
        }

        api.set_camera_position(glm::vec3(-15.0f, 6.0f, 0.0f));
        api.camera_look_at(glm::vec3(1.0f, 0.0f, 0.0f));

        // Build layout data
        build_box_stack_layout();

        // Setup game scene (entities + render/physics resources)
        setup_scene();

        // Anchor floating origin to the active physics object (ExampleGame: the sphere).
        if (Entity *sphere = _world.entities().find(_sphere_entity))
        {
            api.set_physics_origin_anchor(glm::dvec3(sphere->position()));
        }

        // Game ImGui panels
        if (VulkanEngine *renderer = runtime.renderer())
        {
            if (renderer->ui())
            {
                renderer->ui()->add_draw_callback([this]() { draw_contact_debug_ui(); });
            }
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

        // Sync all entities to render
        _world.entities().sync_to_render(api, alpha);

        // Check for reset condition
        bool reset_requested = false;

        if (Entity *sphere = _world.entities().find(_sphere_entity))
        {
            if (sphere->position().y < -50.0f)
            {
                reset_requested = true;
            }
        }

        for (EntityId box_id: _box_entities)
        {
            if (Entity *box = _world.entities().find(box_id))
            {
                if (box->position().y < -50.0f)
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

        auto &api = _runtime->api();

        // Pre-physics: store current transforms as previous for interpolation
        _world.entities().pre_physics_step();

        // Velocity rebasing: keep local physics velocities small even when the absolute world velocity is large.
        // This is an inertial (Galilean) frame change: subtract a constant velocity from every body.
        if (Entity *sphere = _world.entities().find(_sphere_entity))
        {
            if (sphere->has_physics())
            {
                constexpr double kOriginRecenterThresholdM = 500.0;
                constexpr double kOriginSnapSizeM = 100.0;
                (void)api.maybe_rebase_physics_origin_to_body(sphere->physics_body_value(),
                                                             kOriginRecenterThresholdM,
                                                             kOriginSnapSizeM);

                constexpr double kVelocityRebaseThresholdMps = 1000.0;
                (void)api.maybe_rebase_physics_velocity_to_body(sphere->physics_body_value(),
                                                               kVelocityRebaseThresholdMps);
            }
        }

        // Step physics
        _physics->step(fixed_dt);

        // Post-physics: update entity transforms from physics
        _world.entities().post_physics_step(*_physics, WorldVec3(api.get_physics_origin()));

        // Keep the floating origin anchored to the "active" physics entity (ExampleGame: the sphere)
        if (Entity *sphere = _world.entities().find(_sphere_entity))
        {
            api.set_physics_origin_anchor(glm::dvec3(sphere->position()));
        }

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
            _runtime->api().clear_physics_origin_anchor();
        }

        _world.clear();
        _world.set_physics(nullptr);
        _world.set_api(nullptr);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_runtime)
        {
            if (VulkanEngine *renderer = _runtime->renderer())
            {
                if (renderer->_context && renderer->_context->physics == _physics.get())
                {
                    renderer->_context->physics = nullptr;
                }
            }
        }
        if (_physics && _ground_collider_body.is_valid())
        {
            _physics->destroy_body(_ground_collider_body);
            _ground_collider_body = Physics::BodyId{};
        }
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
        _world.set_physics(_physics.get());
#else
        _physics.reset();
        _world.set_physics(nullptr);
#endif

        // Expose the active physics world to the renderer for debug UI / debug draw (optional).
        if (VulkanEngine *renderer = _runtime->renderer())
        {
            if (renderer->_context)
            {
                renderer->_context->physics = _physics.get();
            }
        }

        // Ground (render)
        {
            Transform tr{};
            tr.position = {0.0f, 0.0f, 0.0f};
            tr.scale = {50.0f, 1.0f, 50.0f};
            if (Entity* ground = _world.spawn_primitive("ground", GameAPI::PrimitiveType::Plane, tr))
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
            tr.position = SPHERE_SPAWN_POS;
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

                if (Entity* sphere = _world.spawn_primitive_rigid_body("sphere", GameAPI::PrimitiveType::Sphere, tr, settings))
                {
                    _sphere_entity = sphere->id();
                    _initial_pose[_sphere_entity.value] = InitialPose{tr.position, tr.rotation};
                }
            }
            else
#endif
            {
                if (Entity* sphere = _world.spawn_primitive("sphere", GameAPI::PrimitiveType::Sphere, tr))
                {
                    _sphere_entity = sphere->id();
                    _initial_pose[_sphere_entity.value] = InitialPose{tr.position, tr.rotation};
                }
            }
        }

        // Boxes
        _box_entities.reserve(_box_layouts.size());

        for (const BoxLayout &layout: _box_layouts)
        {
            Transform tr{};
            tr.position = layout.position;
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

                if (Entity* box = _world.spawn_primitive_rigid_body(layout.name, GameAPI::PrimitiveType::Cube, tr, settings))
                {
                    _box_entities.push_back(box->id());
                    _initial_pose[box->id().value] = InitialPose{tr.position, tr.rotation};
                }
                continue;
            }
#endif

            if (Entity* box = _world.spawn_primitive(layout.name, GameAPI::PrimitiveType::Cube, tr))
            {
                _box_entities.push_back(box->id());
                _initial_pose[box->id().value] = InitialPose{tr.position, tr.rotation};
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        install_contact_callbacks();
#endif
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
            Entity* ent = entities.find(id);
            if (!ent)
            {
                return;
            }

            glm::vec3 pos = ent->position();
            glm::quat rot = ent->rotation();
            if (auto it = _initial_pose.find(id.value); it != _initial_pose.end())
            {
                pos = it->second.position;
                rot = it->second.rotation;
            }
            else if (id == _sphere_entity)
            {
                pos = SPHERE_SPAWN_POS;
                rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            }

            if (_physics)
            {
                entities.teleport(id, pos, rot, *_physics, WorldVec3(api.get_physics_origin()));
            }
            else
            {
                ent->set_position(pos);
                ent->set_rotation(rot);
                if (ent->uses_interpolation())
                {
                    ent->interpolation().set_immediate(pos, rot);
                }
            }
        };

        reset_entity_pose(_sphere_entity);
        for (EntityId id : _box_entities)
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
