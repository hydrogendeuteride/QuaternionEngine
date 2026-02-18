#pragma once

#include "runtime/i_game_callbacks.h"
#include "game_world.h"
#include "physics/physics_world.h"
#include "physics/physics_context.h"
#include "core/world.h"
#include "core/game_api.h"

#include <memory>
#include <vector>
#include <deque>
#include <unordered_map>

namespace Game
{

// ============================================================================
// ExampleGame: Demo game using EntityManager for physics-render sync
// ============================================================================

class ExampleGame : public GameRuntime::IGameCallbacks
{
public:
    ExampleGame() = default;
    ~ExampleGame() override = default;

    // IGameCallbacks
    void on_init(GameRuntime::Runtime& runtime) override;
    void on_update(float dt) override;
    void on_fixed_update(float fixed_dt) override;
    void on_shutdown() override;

private:
    // Setup helpers
    void setup_scene();
    void build_box_stack_layout();
    void reset_scene();
    void spawn_test_decals();
    void clear_test_decals();

    // Debug helpers
    void install_contact_callbacks();
    void draw_contact_debug_ui();
    std::string make_body_label(uint32_t body_value, uint64_t user_data) const;

    // Runtime reference
    GameRuntime::Runtime* _runtime{nullptr};

    // Game world (entities + resource lifetime)
    GameWorld _world;

    // Physics world
    std::unique_ptr<Physics::PhysicsWorld> _physics;
    std::unique_ptr<Physics::PhysicsContext> _physics_context;

    // Box stack layout data (before entity creation)
    struct BoxLayout
    {
        std::string name;
        glm::vec3 half_extents{0.5f};
        glm::vec3 position{0.0f};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    };
    std::vector<BoxLayout> _box_layouts;

    // Entity IDs for quick access
    EntityId _ground_entity;
    EntityId _sphere_entity;
    std::vector<EntityId> _box_entities;

    // World-owned static collider for the ground. Not bound to the ground entity,
    // because the render plane sits at y=0 while the physics box is centered at y=-1.
    Physics::BodyId _ground_collider_body;

    // Timing
    float _elapsed{0.0f};
    float _fixed_time{0.0f};
    bool _sphere_launched{false};

    // Initial transforms for reset (EntityId::value -> pose)
    struct InitialPose
    {
        WorldVec3 position_world{0.0, 0.0, 0.0};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    };
    std::unordered_map<uint32_t, InitialPose> _initial_pose;

    // Contact debug logging
    struct ContactLogEntry
    {
        bool is_trigger{false};
        Physics::ContactEventType type{Physics::ContactEventType::Begin};

        uint32_t self_body{0};
        uint32_t other_body{0};

        uint64_t self_user_data{0};
        uint64_t other_user_data{0};

        uint32_t self_layer{0};
        uint32_t other_layer{0};

        glm::vec3 point{0.0f};
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
        float penetration_depth{0.0f};

        float time{0.0f};

        std::string self_label;
        std::string other_label;
    };

    std::deque<ContactLogEntry> _contact_log;
    size_t _contact_log_capacity{256};
    bool _contact_ui_open{true};
    bool _contact_log_enabled{true};
    bool _contact_log_stay{false};
    bool _contact_log_triggers{false};
    bool _contact_print_console{false};
    bool _contact_callbacks_all_bodies{false};
    bool _contact_callbacks_installed_all_bodies{false};

    // Audio playback test state (assets/sounds/dripping.mp3).
    std::string _audio_test_event{"assets/sounds/dripping.mp3"};
    float _audio_test_volume{1.0f};
    float _audio_test_pitch{1.0f};
    bool _audio_test_preloaded{false};
    std::string _audio_test_last_status;

    // Minimal ImGui image example state.
    GameAPI::TextureHandle _imgui_example_texture{GameAPI::InvalidTexture};
    void *_imgui_example_texture_id{nullptr};

    // Mesh VFX rocket-plume style demo (no albedo texture).
    std::string _plume_outer_material_name{"example.mesh_vfx.plume.outer"};
    std::string _plume_inner_material_name{"example.mesh_vfx.plume.inner"};
    std::string _plume_outer_instance_name{"example.mesh_vfx.plume.outer_instance"};
    std::string _plume_inner_instance_name{"example.mesh_vfx.plume.inner_instance"};
    bool _plume_spawned{false};
    WorldVec3 _plume_nozzle_pos{2.5, 1.9, -2.0};

    // Decal test fixtures (one box + one sphere).
    std::string _decal_box_name{"example.decal.box"};
    std::string _decal_sphere_name{"example.decal.sphere"};
    GameAPI::TextureHandle _decal_albedo_texture{GameAPI::InvalidTexture};
    GameAPI::TextureHandle _decal_normal_texture{GameAPI::InvalidTexture};
    bool _decal_test_spawned{false};

    // Constants
    static constexpr float SPHERE_RADIUS = 0.5f;
    static inline const glm::vec3 BOX_HALF_EXTENTS{0.5f, 0.5f, 0.5f};
    static inline const glm::vec3 SPHERE_SPAWN_POS{-14.0f, 2.0f, 0.0f};
};

} // namespace Game
