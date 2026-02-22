#include "gameplay_state.h"
#include "orbit_helpers.h"
#include "core/game_api.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

#include "orbitsim/trajectories.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <cmath>

namespace Game
{
    using detail::finite_vec3;
    using detail::point_mass_accel;
    using detail::orbitsim_nbody_accel_earth_fixed;

    // ---- Physics simulation ----

    void GameplayState::step_physics([[maybe_unused]] GameStateContext &ctx, float fixed_dt)
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_physics || !_physics_context)
        {
            return;
        }

        _world.pre_physics_step();

        const bool use_orbitsim = _orbitsim && _orbitsim->earth_id != orbitsim::kInvalidBodyId;
        if (use_orbitsim)
        {
            _orbitsim->sim.step(static_cast<double>(fixed_dt));

            const orbitsim::MassiveBody *earth = _orbitsim->sim.body_by_id(_orbitsim->earth_id);
            const orbitsim::MassiveBody *moon = _orbitsim->sim.body_by_id(_orbitsim->moon_id);
            if (earth && moon)
            {
                const WorldVec3 moon_pos_world =
                        _planet_center_world + WorldVec3(moon->state.position_m - earth->state.position_m);
                if (Entity *moon_ent = _world.entities().find(_moon_entity))
                {
                    moon_ent->set_position_world(moon_pos_world);
                    moon_ent->set_rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
                }
            }
        }

        auto gravity_accel_world_at = [&](const WorldVec3 &p_world) -> glm::dvec3 {
            if (!_orbitsim)
            {
                return glm::dvec3(0.0);
            }

            const glm::dvec3 p_rel = glm::dvec3(p_world - _planet_center_world);
            if (use_orbitsim)
            {
                return orbitsim_nbody_accel_earth_fixed(*_orbitsim, p_rel);
            }

            if (!(_orbitsim->earth_mass_kg > 0.0))
            {
                return glm::dvec3(0.0);
            }

            return point_mass_accel(orbitsim::kGravitationalConstant_SI, _orbitsim->earth_mass_kg, p_rel, 0.0);
        };

        // Velocity-origin handling:
        //  PerStepAnchorSync: Galilean shift every step to keep anchor v_local near 0.
        //  FreeFallAnchorFrame: integrate velocity-origin from anchor acceleration.
        glm::dvec3 anchor_accel_world(0.0);
        bool have_anchor_accel = false;
        const bool per_step_sync = _velocity_origin_mode == VelocityOriginMode::PerStepAnchorSync;
        const WorldVec3 physics_origin_world = _physics_context->origin_world();

        if (_ship_entity.is_valid())
        {
            Entity *anchor = _world.entities().find(_ship_entity);
            if (anchor && anchor->has_physics())
            {
                const Physics::BodyId anchor_body{anchor->physics_body_value()};
                if (_physics->is_body_valid(anchor_body))
                {
                    const double dt = static_cast<double>(fixed_dt);
                    if (per_step_sync)
                    {
                        const glm::vec3 v_local_f = _physics->get_linear_velocity(anchor_body);
                        const glm::dvec3 v_world = _physics_context->velocity_origin_world() + glm::dvec3(v_local_f);
                        (void) _physics_context->set_velocity_origin_world(v_world);
                        _physics->shift_velocity_origin(glm::dvec3(v_local_f));
                    }
                    else
                    {
                        const glm::dvec3 p_local_anchor = _physics->get_position(anchor_body);
                        const WorldVec3 p_world_anchor = physics_origin_world + WorldVec3(p_local_anchor);
                        anchor_accel_world = gravity_accel_world_at(p_world_anchor);
                        have_anchor_accel = true;

                        const glm::dvec3 v_origin_next =
                                _physics_context->velocity_origin_world() + anchor_accel_world * dt;
                        (void) _physics_context->set_velocity_origin_world(v_origin_next);
                    }
                }
            }
        }

        const glm::dvec3 frame_accel_world =
                (!per_step_sync && have_anchor_accel) ? anchor_accel_world : glm::dvec3(0.0);

        auto apply_gravity_accel = [&](EntityId id) {
            Entity *ent = _world.entities().find(id);
            if (!ent || !ent->has_physics())
            {
                return;
            }

            const Physics::BodyId body_id{ent->physics_body_value()};
            if (!_physics->is_body_valid(body_id))
            {
                return;
            }

            const glm::dvec3 p_local = _physics->get_position(body_id);
            const WorldVec3 p_world = local_to_world_d(p_local, physics_origin_world);

            const glm::dvec3 a_local = gravity_accel_world_at(p_world) - frame_accel_world;

            glm::vec3 v_local = _physics->get_linear_velocity(body_id);
            v_local += glm::vec3(a_local) * fixed_dt;
            _physics->set_linear_velocity(body_id, v_local);
            _physics->activate(body_id);
        };

        apply_gravity_accel(_ship_entity);
        apply_gravity_accel(_probe_entity);

        _physics->step(fixed_dt);

        // Advance moving frame: x_world = x_origin + x_local, d/dt(x_origin) = v_origin.
        const glm::dvec3 v_origin = _physics_context->velocity_origin_world();
        if (finite_vec3(v_origin))
        {
            const double dt = static_cast<double>(fixed_dt);
            (void) _physics_context->set_origin_world(
                    _physics_context->origin_world() + WorldVec3(v_origin * dt));
        }

        _world.post_physics_step();
#endif
    }

    // ---- Ship state query ----

    bool GameplayState::get_ship_world_state(WorldVec3 &out_pos_world,
                                             glm::dvec3 &out_vel_world,
                                             glm::vec3 &out_vel_local) const
    {
        const Entity *ship = _world.entities().find(_ship_entity);
        if (!ship)
        {
            return false;
        }

        out_pos_world = ship->position_world();
        out_vel_world = glm::dvec3(0.0);
        out_vel_local = glm::vec3(0.0f);

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics && _physics_context && ship->has_physics())
        {
            const Physics::BodyId body_id{ship->physics_body_value()};
            if (_physics->is_body_valid(body_id))
            {
                out_vel_local = _physics->get_linear_velocity(body_id);
                out_vel_world = _physics_context->velocity_origin_world() + glm::dvec3(out_vel_local);
            }
        }
#endif

        return true;
    }

    // ---- Orbit prediction ----

    float GameplayState::effective_prediction_interval() const
    {
        constexpr float kDefault = 0.25f;
        return (std::isfinite(_prediction_update_interval_s) && _prediction_update_interval_s > 0.0f)
            ? _prediction_update_interval_s : kDefault;
    }

    void GameplayState::update_prediction(GameStateContext &ctx, float fixed_dt)
    {
        if (!_prediction_enabled)
        {
            if (!_prediction_altitude_km.empty() || !_prediction_speed_kmps.empty() ||
                !_prediction_points_world.empty())
            {
                _prediction_update_accum_s = 0.0f;
                _prediction_altitude_km.clear();
                _prediction_speed_kmps.clear();
                _prediction_points_world.clear();
            }
            return;
        }

        const float interval_s = effective_prediction_interval();
        _prediction_update_accum_s += fixed_dt;

        if (!_prediction_points_world.empty() && _prediction_update_accum_s < interval_s)
        {
            return;
        }

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);

        if (get_ship_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
        {
            update_orbit_prediction_cache(ship_pos_world, ship_vel_world);
            emit_orbit_prediction_debug(ctx);
        }
        else
        {
            _prediction_altitude_km.clear();
            _prediction_speed_kmps.clear();
            _prediction_points_world.clear();
        }

        _prediction_update_accum_s = std::fmod(_prediction_update_accum_s, interval_s);
    }

    void GameplayState::update_orbit_prediction_cache(const WorldVec3 &ship_pos_world, const glm::dvec3 &ship_vel_world)
    {
        _prediction_altitude_km.clear();
        _prediction_speed_kmps.clear();
        _prediction_points_world.clear();

        if (!_orbitsim)
        {
            return;
        }

        if (!std::isfinite(_prediction_dt_s) || _prediction_dt_s <= 0.0)
        {
            return;
        }

        if (!std::isfinite(_prediction_horizon_s) || _prediction_horizon_s <= 0.0)
        {
            return;
        }

        const double dt_s = std::clamp(_prediction_dt_s, 0.01, 60.0);
        const double horizon_s = std::clamp(_prediction_horizon_s, dt_s, 36'000.0);

        constexpr int max_steps = 512;
        const int steps = std::clamp(static_cast<int>(std::ceil(horizon_s / dt_s)), 2, max_steps);

        _prediction_altitude_km.reserve(static_cast<size_t>(steps) + 1);
        _prediction_speed_kmps.reserve(static_cast<size_t>(steps) + 1);
        _prediction_points_world.reserve(static_cast<size_t>(steps) + 1);

        OrbitsimDemo demo_pred = *_orbitsim;

        auto safe_length = [](const glm::dvec3 &v) -> double {
            const double len2 = glm::dot(v, v);
            if (!std::isfinite(len2) || len2 <= 0.0)
            {
                return 0.0;
            }
            return std::sqrt(len2);
        };

        const bool can_use_orbitsim_predict = demo_pred.earth_id != orbitsim::kInvalidBodyId;
        if (can_use_orbitsim_predict)
        {
            const orbitsim::MassiveBody *earth = demo_pred.sim.body_by_id(demo_pred.earth_id);
            if (earth)
            {
                const glm::dvec3 ship_rel_pos_m = glm::dvec3(ship_pos_world - _planet_center_world);
                const glm::dvec3 ship_bary_pos_m = earth->state.position_m + ship_rel_pos_m;
                const glm::dvec3 ship_bary_vel_mps = earth->state.velocity_mps + ship_vel_world;

                orbitsim::Spacecraft ship_sc{};
                ship_sc.state = orbitsim::make_state(ship_bary_pos_m, ship_bary_vel_mps);
                ship_sc.dry_mass_kg = 1.0;

                const auto ship_h = demo_pred.sim.create_spacecraft(ship_sc);
                if (ship_h.valid())
                {
                    orbitsim::TrajectoryOptions opt{};
                    opt.duration_s = horizon_s;
                    opt.sample_dt_s = dt_s;
                    opt.spacecraft_sample_dt_s = dt_s;
                    opt.spacecraft_lookup_dt_s = dt_s;
                    opt.celestial_dt_s = dt_s;
                    opt.max_samples = static_cast<size_t>(steps) + 1;
                    opt.include_start = true;
                    opt.include_end = true;
                    opt.stop_on_impact = false;

                    const orbitsim::CelestialEphemeris eph = orbitsim::build_celestial_ephemeris(demo_pred.sim, opt);
                    const std::vector<orbitsim::TrajectorySample> traj_inertial =
                            orbitsim::predict_spacecraft_trajectory(demo_pred.sim, eph, ship_h.id, opt);

                    if (!traj_inertial.empty())
                    {
                        const std::vector<orbitsim::TrajectorySample> traj_earth_centered =
                                orbitsim::trajectory_to_body_centered_inertial(traj_inertial, eph, *earth);

                        if (!traj_earth_centered.empty())
                        {
                            _prediction_altitude_km.reserve(traj_earth_centered.size());
                            _prediction_speed_kmps.reserve(traj_earth_centered.size());
                            _prediction_points_world.reserve(traj_earth_centered.size());

                            for (const orbitsim::TrajectorySample &sample: traj_earth_centered)
                            {
                                const double r_m = safe_length(sample.position_m);
                                const double alt_km = (r_m - _planet_radius_m) * 1.0e-3;
                                const double spd_kmps = safe_length(sample.velocity_mps) * 1.0e-3;

                                _prediction_altitude_km.push_back(static_cast<float>(alt_km));
                                _prediction_speed_kmps.push_back(static_cast<float>(spd_kmps));
                                _prediction_points_world.push_back(_planet_center_world + WorldVec3(sample.position_m));
                            }
                            return;
                        }
                    }
                }
            }
        }

        // Fallback: legacy local integration when orbitsim prediction is unavailable.
        glm::dvec3 p_rel_m = glm::dvec3(ship_pos_world - _planet_center_world);
        glm::dvec3 v_rel_mps = ship_vel_world;
        const bool use_orbitsim = demo_pred.earth_id != orbitsim::kInvalidBodyId;

        for (int i = 0; i <= steps; ++i)
        {
            const double r_m = safe_length(p_rel_m);
            const double alt_km = (r_m - _planet_radius_m) * 1.0e-3;
            const double spd_kmps = safe_length(v_rel_mps) * 1.0e-3;

            _prediction_altitude_km.push_back(static_cast<float>(alt_km));
            _prediction_speed_kmps.push_back(static_cast<float>(spd_kmps));
            _prediction_points_world.push_back(_planet_center_world + WorldVec3(p_rel_m));

            if (i == steps)
            {
                break;
            }

            glm::dvec3 a_rel(0.0);
            if (use_orbitsim)
            {
                a_rel = orbitsim_nbody_accel_earth_fixed(demo_pred, p_rel_m);
                demo_pred.sim.step(dt_s);
            }
            else if (demo_pred.earth_mass_kg > 0.0)
            {
                a_rel = point_mass_accel(orbitsim::kGravitationalConstant_SI, demo_pred.earth_mass_kg, p_rel_m, 0.0);
            }

            if (!finite_vec3(a_rel))
            {
                a_rel = glm::dvec3(0.0);
            }

            v_rel_mps += a_rel * dt_s;
            p_rel_m += v_rel_mps * dt_s;
        }
    }

    void GameplayState::emit_orbit_prediction_debug(GameStateContext &ctx)
    {
        if (!_prediction_enabled || !_debug_draw_enabled)
        {
            return;
        }

        if (!ctx.api)
        {
            return;
        }

        if (_prediction_points_world.size() < 2)
        {
            return;
        }

        const float interval_s = effective_prediction_interval();
        const float ttl_s = std::max(std::max(_prediction_debug_ttl_s, interval_s + 0.05f), 0.11f);

        constexpr glm::vec4 color_orbit{0.2f, 0.9f, 0.2f, 0.65f};
        constexpr glm::vec4 color_velocity{1.0f, 0.35f, 0.1f, 1.0f};

        for (size_t i = 1; i < _prediction_points_world.size(); ++i)
        {
            ctx.api->debug_draw_line(glm::dvec3(_prediction_points_world[i - 1]),
                                     glm::dvec3(_prediction_points_world[i]),
                                     color_orbit,
                                     ttl_s,
                                     true);
        }

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (get_ship_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
        {
            const double speed_mps = glm::length(ship_vel_world);
            double len_m = 40.0;
            if (std::isfinite(speed_mps) && speed_mps > 1.0)
            {
                len_m = std::clamp(speed_mps * 0.002, 10.0, 250.0);
            }

            ctx.api->debug_draw_ray(glm::dvec3(ship_pos_world), ship_vel_world, len_m, color_velocity, ttl_s, true);
        }
    }
} // namespace Game
