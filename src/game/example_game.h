#pragma once

#include "runtime/i_game_callbacks.h"
#include "entity_manager.h"
#include "physics/physics_world.h"

#include <memory>
#include <vector>
#include <deque>

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
    void setup_visuals();
    void setup_physics();
    void build_box_stack_layout();
    void reset_scene();

    // Debug helpers
    void install_contact_callbacks();
    void draw_contact_debug_ui();
    std::string make_body_label(uint32_t body_value, uint64_t user_data) const;

    // Runtime reference
    GameRuntime::Runtime* _runtime{nullptr};

    // Entity system
    EntityManager _entities;

    // Physics world
    std::unique_ptr<Physics::PhysicsWorld> _physics;

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

    // Timing
    float _elapsed{0.0f};
    float _fixed_time{0.0f};
    bool _sphere_launched{false};

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

    // Constants
    static constexpr float SPHERE_RADIUS = 0.5f;
    static inline const glm::vec3 BOX_HALF_EXTENTS{0.5f, 0.5f, 0.5f};
    static inline const glm::vec3 SPHERE_SPAWN_POS{-14.0f, 2.0f, 0.0f};
};

} // namespace Game
