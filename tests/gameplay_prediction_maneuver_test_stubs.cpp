#include "game/states/gameplay/gameplay_state.h"

#include "core/game_api.h"
#include "core/input/input_system.h"
#include "core/picking/picking_system.h"
#include "game/component/ship_controller.h"
#include "game/entity_manager.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <unordered_map>

namespace
{
    std::unordered_map<uint32_t, Game::Entity *> g_entities;
    bool g_has_ship_controller_input_override = false;
    Game::ThrustInput g_ship_controller_input_override{};
}

namespace GameplayTestHooks
{
    void register_entity(Game::Entity *entity)
    {
        if (!entity)
        {
            return;
        }
        g_entities[entity->id().value] = entity;
    }

    void clear_entities()
    {
        g_entities.clear();
    }

    void set_ship_controller_input_override(const Game::ThrustInput &input)
    {
        g_ship_controller_input_override = input;
        g_has_ship_controller_input_override = true;
    }

    void clear_ship_controller_input_override()
    {
        g_has_ship_controller_input_override = false;
        g_ship_controller_input_override = {};
    }
} // namespace GameplayTestHooks

namespace Game
{
    ScenarioConfig default_earth_moon_config()
    {
        ScenarioConfig cfg{};
        cfg.system_center = WorldVec3{0.0, 0.0, 0.0};
        return cfg;
    }

    GameplayState::GameplayState()
        : _scenario_config(default_earth_moon_config())
    {
    }

    GameplayState::~GameplayState() = default;

    void GameplayState::on_enter(GameStateContext &ctx) { (void) ctx; }
    void GameplayState::on_exit(GameStateContext &ctx) { (void) ctx; }
    void GameplayState::on_update(GameStateContext &ctx, float dt) { (void) ctx; (void) dt; }
    void GameplayState::on_fixed_update(GameStateContext &ctx, float fixed_dt) { (void) ctx; (void) fixed_dt; }
    void GameplayState::on_draw_ui(GameStateContext &ctx) { (void) ctx; }

    void GameplayState::setup_scene(GameStateContext &ctx) { (void) ctx; }
    void GameplayState::setup_environment(GameStateContext &ctx) { (void) ctx; }
    void GameplayState::init_orbitsim(WorldVec3 &player_pos_world, glm::dvec3 &player_vel_world)
    {
        (void) player_pos_world;
        (void) player_vel_world;
    }

    void GameplayState::reset_time_warp_state()
    {
        _time_warp.warp_level = 0;
        _time_warp.mode = TimeWarpState::Mode::Realtime;
        _rails_warp_active = false;
        _last_sim_step_dt_s = 0.0;
        _rails_thrust_applied_this_tick = false;
        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);
    }

    void GameplayState::handle_time_warp_input(GameStateContext &ctx) { (void) ctx; }

    Physics::BodyId GameplayState::create_orbiter_physics_body(const bool render_is_gltf,
                                                               Entity &entity,
                                                               const Physics::BodySettings &settings,
                                                               const WorldVec3 &position_world,
                                                               const glm::quat &rotation,
                                                               glm::vec3 *out_origin_offset_local)
    {
        (void) render_is_gltf;
        (void) entity;
        (void) settings;
        (void) position_world;
        (void) rotation;
        if (out_origin_offset_local)
        {
            *out_origin_offset_local = glm::vec3(0.0f);
        }
        return {};
    }

    bool GameplayState::destroy_orbiter_physics_body(const bool render_is_gltf, Entity &entity)
    {
        (void) render_is_gltf;
        (void) entity;
        return false;
    }

    ComponentContext GameplayState::build_component_context(GameStateContext &ctx, float alpha)
    {
        ComponentContext comp_ctx{};
        comp_ctx.world = &_world;
        comp_ctx.api = ctx.api;
        comp_ctx.input = ctx.input;
        comp_ctx.physics = _physics.get();
        comp_ctx.ui_capture_keyboard = false;
        comp_ctx.interpolation_alpha = alpha;
        return comp_ctx;
    }

    OrbiterInfo *GameplayState::find_player_orbiter()
    {
        for (auto &o : _orbiters)
        {
            if (o.is_player)
            {
                return &o;
            }
        }
        return nullptr;
    }

    const OrbiterInfo *GameplayState::find_player_orbiter() const
    {
        for (const auto &o : _orbiters)
        {
            if (o.is_player)
            {
                return &o;
            }
        }
        return nullptr;
    }

    EntityId GameplayState::player_entity() const
    {
        const OrbiterInfo *p = find_player_orbiter();
        return p ? p->entity : EntityId{};
    }

    OrbiterInfo *GameplayState::find_orbiter(const EntityId entity)
    {
        if (!entity.is_valid())
        {
            return nullptr;
        }

        for (auto &orbiter : _orbiters)
        {
            if (orbiter.entity == entity)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    const OrbiterInfo *GameplayState::find_orbiter(const EntityId entity) const
    {
        if (!entity.is_valid())
        {
            return nullptr;
        }

        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.entity == entity)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    OrbiterInfo *GameplayState::find_orbiter(const std::string_view name)
    {
        if (name.empty())
        {
            return nullptr;
        }

        for (auto &orbiter : _orbiters)
        {
            if (orbiter.name == name)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    const OrbiterInfo *GameplayState::find_orbiter(const std::string_view name) const
    {
        if (name.empty())
        {
            return nullptr;
        }

        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.name == name)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    EntityId GameplayState::select_rebase_anchor_entity() const
    {
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.is_rebase_anchor && orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.is_player && orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }
        return EntityId{};
    }

    void GameplayState::update_rebase_anchor()
    {
        const EntityId next_anchor = select_rebase_anchor_entity();
        if (!next_anchor.is_valid())
        {
            _world.clear_rebase_anchor();
            return;
        }

        if (next_anchor != _world.rebase_anchor())
        {
            _world.set_rebase_anchor(next_anchor);
        }
    }

    void GameStateContext::quit() {}
    float GameStateContext::delta_time() const { return 0.0f; }
    float GameStateContext::fixed_delta_time() const { return 0.0f; }
    float GameStateContext::interpolation_alpha() const { return 1.0f; }

    Entity *EntityManager::find(EntityId id)
    {
        auto it = g_entities.find(id.value);
        return (it != g_entities.end()) ? it->second : nullptr;
    }

    const Entity *EntityManager::find(EntityId id) const
    {
        auto it = g_entities.find(id.value);
        return (it != g_entities.end()) ? it->second : nullptr;
    }

    bool GameWorld::bind_physics(EntityId id,
                                 uint32_t body_value,
                                 bool use_interpolation,
                                 bool override_user_data,
                                 const glm::vec3 &origin_offset_local)
    {
        (void) id;
        (void) body_value;
        (void) use_interpolation;
        (void) override_user_data;
        (void) origin_offset_local;
        return true;
    }

} // namespace Game

bool InputState::key_down(Key key) const
{
    (void) key;
    return false;
}

bool InputState::key_pressed(Key key) const
{
    (void) key;
    return false;
}

bool InputState::mouse_down(MouseButton button) const
{
    (void) button;
    return false;
}

bool InputState::mouse_pressed(MouseButton button) const
{
    (void) button;
    return false;
}

glm::mat4 Camera::getRotationMatrix() const
{
    return glm::mat4_cast(orientation);
}

namespace vkutil
{
    VkRect2D compute_letterbox_rect(VkExtent2D srcSize, VkExtent2D dstSize)
    {
        (void) srcSize;
        return VkRect2D{
                .offset = {0, 0},
                .extent = dstSize,
        };
    }

    bool map_window_to_letterbox_src(const glm::vec2 &windowPosPixels,
                                     VkExtent2D srcSize,
                                     VkExtent2D dstSize,
                                     glm::vec2 &outSrcPosPixels)
    {
        (void) srcSize;
        (void) dstSize;
        outSrcPosPixels = windowPosPixels;
        return true;
    }
} // namespace vkutil

namespace Game
{
    ThrustInput ShipController::read_input(const InputState *input, const ShipKeybinds *binds,
                                           bool ui_capture_keyboard, bool &sas_toggle_prev_down)
    {
        (void) input;
        (void) binds;
        (void) ui_capture_keyboard;
        sas_toggle_prev_down = false;
        if (g_has_ship_controller_input_override)
        {
            return g_ship_controller_input_override;
        }
        return {};
    }

    void ShipController::on_fixed_update(ComponentContext &ctx, float fixed_dt)
    {
        (void) ctx;
        (void) fixed_dt;
    }
} // namespace Game

namespace GameAPI
{
    bool Engine::set_mesh_instance_transform(const std::string &name, const Transform &transform)
    {
        (void) name;
        (void) transform;
        return true;
    }

    bool Engine::set_mesh_instance_transform(const std::string &name, const TransformD &transform)
    {
        (void) name;
        (void) transform;
        return true;
    }

    bool Engine::set_planet_center(const std::string &name, const glm::dvec3 &center)
    {
        (void) name;
        (void) center;
        return true;
    }

    void Engine::debug_draw_line(const glm::vec3 &a, const glm::vec3 &b,
                                 const glm::vec4 &color, float duration_seconds, bool depth_tested)
    {
        (void) a; (void) b; (void) color; (void) duration_seconds; (void) depth_tested;
    }

    void Engine::debug_draw_line(const glm::dvec3 &a, const glm::dvec3 &b,
                                 const glm::vec4 &color, float duration_seconds, bool depth_tested)
    {
        (void) a; (void) b; (void) color; (void) duration_seconds; (void) depth_tested;
    }

    void Engine::debug_draw_ray(const glm::vec3 &origin, const glm::vec3 &direction, float length,
                                const glm::vec4 &color, float duration_seconds, bool depth_tested)
    {
        (void) origin; (void) direction; (void) length;
        (void) color; (void) duration_seconds; (void) depth_tested;
    }

    void Engine::debug_draw_ray(const glm::dvec3 &origin, const glm::dvec3 &direction, double length,
                                const glm::vec4 &color, float duration_seconds, bool depth_tested)
    {
        (void) origin; (void) direction; (void) length;
        (void) color; (void) duration_seconds; (void) depth_tested;
    }

    void Engine::debug_draw_sphere(const glm::vec3 &center, float radius,
                                   const glm::vec4 &color, float duration_seconds, bool depth_tested)
    {
        (void) center; (void) radius; (void) color; (void) duration_seconds; (void) depth_tested;
    }

    void Engine::debug_draw_sphere(const glm::dvec3 &center, float radius,
                                   const glm::vec4 &color, float duration_seconds, bool depth_tested)
    {
        (void) center; (void) radius; (void) color; (void) duration_seconds; (void) depth_tested;
    }
} // namespace GameAPI

void PickingSystem::clear_line_picks() {}

uint32_t PickingSystem::add_line_pick_group(std::string owner_name)
{
    (void) owner_name;
    return 0;
}

void PickingSystem::add_line_pick_segment(const uint32_t group_id,
                                          const WorldVec3 &a_world,
                                          const WorldVec3 &b_world,
                                          const double a_time_s,
                                          const double b_time_s)
{
    (void) group_id;
    (void) a_world;
    (void) b_world;
    (void) a_time_s;
    (void) b_time_s;
}
