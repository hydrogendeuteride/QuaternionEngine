// gameplay_state_test_stubs.cpp
//
// Minimal stub implementations of GameplayState lifecycle methods, GameAPI::Engine
// debug-draw overloads, and PickingSystem that are not compiled into this test target
// but are required for linkage.

#include "game/states/gameplay/gameplay_state.h"
#include "core/game_api.h"
#include "core/picking/picking_system.h"
#include "core/input/input_system.h"
#include "game/component/ship_controller.h"
#include "game/entity_manager.h"

namespace Game
{
    ScenarioConfig default_earth_moon_config()
    {
        return {};
    }

    GameplayState::GameplayState()  = default;
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
        _time_warp.mode       = TimeWarpState::Mode::Realtime;
        _rails_warp_active    = false;
    }

    void GameplayState::handle_time_warp_input(GameStateContext &ctx) { (void) ctx; }

    ComponentContext GameplayState::build_component_context(GameStateContext &ctx, float alpha)
    {
        ComponentContext comp_ctx{};
        comp_ctx.world               = &_world;
        comp_ctx.api                 = ctx.api;
        comp_ctx.input               = ctx.input;
        comp_ctx.physics             = _physics.get();
        comp_ctx.ui_capture_keyboard = false;
        comp_ctx.interpolation_alpha = alpha;
        return comp_ctx;
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

    void GameplayState::update_rebase_anchor() {}

    void GameplayState::mark_prediction_dirty()
    {
        _prediction_dirty = true;
    }

    void GameStateContext::quit() {}
    float GameStateContext::delta_time() const { return 0.0f; }
    float GameStateContext::fixed_delta_time() const { return 0.0f; }
    float GameStateContext::interpolation_alpha() const { return 1.0f; }

} // namespace Game

bool InputState::key_down(Key key) const
{
    (void) key;
    return false;
}

namespace Game
{
    Entity *EntityManager::find(EntityId id)
    {
        (void) id;
        return nullptr;
    }

    const Entity *EntityManager::find(EntityId id) const
    {
        (void) id;
        return nullptr;
    }
} // namespace Game

namespace Game
{
    void InterpolatedTransform::store_current_as_previous()
    {
        prev_position = curr_position;
        prev_rotation = curr_rotation;
    }

    void InterpolatedTransform::set_immediate(const WorldVec3 &pos, const glm::quat &rot)
    {
        prev_position = pos;
        curr_position = pos;
        prev_rotation = rot;
        curr_rotation = rot;
    }

    ThrustInput ShipController::read_input(const InputState *input, bool ui_capture_keyboard, bool &sas_toggle_prev_down)
    {
        (void) input;
        (void) ui_capture_keyboard;
        sas_toggle_prev_down = false;
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
    (void) group_id; (void) a_world; (void) b_world; (void) a_time_s; (void) b_time_s;
}
