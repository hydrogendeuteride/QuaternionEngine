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
#include <limits>
#include <utility>

namespace Game
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kEscapeDefaultPeriodS = 7200.0;
        constexpr double kOrbitDrawMaxDtS = 1.0; // visual subdivision step (reduces polyline chord error)

        double safe_length(const glm::dvec3 &v)
        {
            const double len2 = glm::dot(v, v);
            if (!std::isfinite(len2) || len2 <= 0.0)
            {
                return 0.0;
            }
            return std::sqrt(len2);
        }

        double estimate_orbital_period_s(const double mu_m3_s2, const glm::dvec3 &r_m, const glm::dvec3 &v_mps)
        {
            const double r = safe_length(r_m);
            const double v = safe_length(v_mps);
            if (!(mu_m3_s2 > 0.0) || !(r > 0.0) || !std::isfinite(mu_m3_s2) || !std::isfinite(r) || !std::isfinite(v))
            {
                return kEscapeDefaultPeriodS;
            }

            const double specific_energy = 0.5 * (v * v) - mu_m3_s2 / r;
            if (!std::isfinite(specific_energy) || specific_energy >= 0.0)
            {
                return kEscapeDefaultPeriodS;
            }

            const double a_m = -mu_m3_s2 / (2.0 * specific_energy);
            if (!(a_m > 0.0) || !std::isfinite(a_m))
            {
                return kEscapeDefaultPeriodS;
            }

            const double period_s = 2.0 * kPi * std::sqrt((a_m * a_m * a_m) / mu_m3_s2);
            if (!std::isfinite(period_s) || period_s <= 0.0)
            {
                return kEscapeDefaultPeriodS;
            }
            return period_s;
        }

        struct OrbitalElementsEstimate
        {
            bool valid{false};
            double semi_major_axis_m{0.0};
            double eccentricity{0.0};
            double orbital_period_s{0.0};
            double periapsis_m{0.0};
            double apoapsis_m{std::numeric_limits<double>::infinity()};
        };

        OrbitalElementsEstimate compute_orbital_elements(const double mu_m3_s2, const glm::dvec3 &r_m,
                                                         const glm::dvec3 &v_mps)
        {
            OrbitalElementsEstimate out{};
            if (!(mu_m3_s2 > 0.0) || !std::isfinite(mu_m3_s2))
            {
                return out;
            }

            const double r = safe_length(r_m);
            const double v2 = glm::dot(v_mps, v_mps);
            if (!(r > 0.0) || !std::isfinite(r) || !std::isfinite(v2))
            {
                return out;
            }

            const glm::dvec3 h = glm::cross(r_m, v_mps);
            const double h2 = glm::dot(h, h);

            const glm::dvec3 e_vec = (glm::cross(v_mps, h) / mu_m3_s2) - (r_m / r);
            const double e = safe_length(e_vec);
            if (!std::isfinite(e))
            {
                return out;
            }

            const double specific_energy = 0.5 * v2 - mu_m3_s2 / r;
            if (!std::isfinite(specific_energy))
            {
                return out;
            }

            out.eccentricity = std::max(0.0, e);

            if (std::abs(specific_energy) > 1e-12)
            {
                out.semi_major_axis_m = -mu_m3_s2 / (2.0 * specific_energy);
            }

            if (out.semi_major_axis_m > 0.0 && std::isfinite(out.semi_major_axis_m) && out.eccentricity < 1.0)
            {
                out.orbital_period_s = 2.0 * kPi *
                                       std::sqrt((out.semi_major_axis_m * out.semi_major_axis_m *
                                                  out.semi_major_axis_m) /
                                                 mu_m3_s2);
                out.periapsis_m = out.semi_major_axis_m * (1.0 - out.eccentricity);
                out.apoapsis_m = out.semi_major_axis_m * (1.0 + out.eccentricity);
            }
            else if (h2 > 0.0 && std::isfinite(h2))
            {
                const double denom = mu_m3_s2 * (1.0 + out.eccentricity);
                if (denom > 0.0 && std::isfinite(denom))
                {
                    out.periapsis_m = h2 / denom;
                }
                out.orbital_period_s = 0.0;
                out.apoapsis_m = std::numeric_limits<double>::infinity();
            }

            if (!std::isfinite(out.periapsis_m) || out.periapsis_m <= 0.0)
            {
                out.periapsis_m = r;
            }

            out.valid = true;
            return out;
        }

        std::pair<double, double> select_prediction_horizon_and_dt(const double mu_m3_s2, const glm::dvec3 &r_m,
                                                                    const glm::dvec3 &v_mps)
        {
            const double period_s = estimate_orbital_period_s(mu_m3_s2, r_m, v_mps);
            const double horizon_s = std::clamp(period_s * 1.1, 60.0, 36'000.0);
            const double target_samples = std::clamp(horizon_s / 2.0, 500.0, 2000.0);
            const double dt_s = std::clamp(horizon_s / target_samples, 0.01, 60.0);
            return {horizon_s, dt_s};
        }

        WorldVec3 hermite_position_world(const WorldVec3 &ref_body_world,
                                         const orbitsim::TrajectorySample &a,
                                         const orbitsim::TrajectorySample &b,
                                         const double t_s)
        {
            const double ta = a.t_s;
            const double tb = b.t_s;
            const double h = tb - ta;
            if (!std::isfinite(h) || !(h > 0.0))
            {
                return ref_body_world + WorldVec3(a.position_m);
            }

            double u = (t_s - ta) / h;
            if (!std::isfinite(u))
            {
                u = 0.0;
            }
            u = std::clamp(u, 0.0, 1.0);

            const double u2 = u * u;
            const double u3 = u2 * u;

            const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
            const double h10 = u3 - (2.0 * u2) + u;
            const double h01 = (-2.0 * u3) + (3.0 * u2);
            const double h11 = u3 - u2;

            const glm::dvec3 p0 = glm::dvec3(a.position_m);
            const glm::dvec3 p1 = glm::dvec3(b.position_m);
            const glm::dvec3 m0 = glm::dvec3(a.velocity_mps) * h;
            const glm::dvec3 m1 = glm::dvec3(b.velocity_mps) * h;

            const glm::dvec3 p = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
            return ref_body_world + WorldVec3(p);
        }

    } // namespace

    bool GameplayState::get_player_world_state(WorldVec3 &out_pos_world,
                                               glm::dvec3 &out_vel_world,
                                               glm::vec3 &out_vel_local) const
    {
        const EntityId player_eid = player_entity();
        const Entity *player = _world.entities().find(player_eid);
        if (!player)
        {
            return false;
        }

        out_pos_world = player->position_world();
        out_vel_world = glm::dvec3(0.0);
        out_vel_local = glm::vec3(0.0f);

        if (_rails_warp_active && _orbitsim)
        {
            const OrbiterInfo *p = find_player_orbiter();
            const orbitsim::MassiveBody *ref_sim = _orbitsim->reference_sim_body();
            if (p && p->rails.active() && ref_sim)
            {
                if (const orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(p->rails.sc_id))
                {
                    out_pos_world = _scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                    out_vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
                    out_vel_local = glm::vec3(0.0f);
                    return true;
                }
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_physics && _physics_context && player->has_physics())
        {
            const Physics::BodyId body_id{player->physics_body_value()};
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

    void GameplayState::mark_prediction_dirty()
    {
        _prediction_dirty = true;
    }

    void GameplayState::update_prediction(GameStateContext &ctx, float fixed_dt)
    {
        (void) ctx;

        if (!_prediction_enabled)
        {
            _prediction_cache.clear();
            _prediction_dirty = true;
            return;
        }

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);

        if (!get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
        {
            _prediction_cache.clear();
            _prediction_dirty = true;
            return;
        }

        const double now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;

        const bool thrusting = player_thrust_applied_this_tick();
        bool rebuild_prediction = _prediction_dirty || !_prediction_cache.valid;

        if (!rebuild_prediction && thrusting)
        {
            const double dt_since_build_s = now_s - _prediction_cache.build_time_s;
            rebuild_prediction = !_prediction_cache.valid || dt_since_build_s >= _prediction_thrust_refresh_s;
        }

        if (!rebuild_prediction && _prediction_periodic_refresh_s > 0.0)
        {
            const double dt_since_build_s = now_s - _prediction_cache.build_time_s;
            rebuild_prediction = dt_since_build_s >= _prediction_periodic_refresh_s;
        }

        // Ensure the cache covers the desired future window; otherwise the highlighted segment will
        // shrink and clamp at the end of the current trajectory.
        if (!rebuild_prediction && _prediction_cache.valid && !_prediction_cache.trajectory_bci.empty())
        {
            const double cache_end_s = _prediction_cache.trajectory_bci.back().t_s;
            const double required_ahead_s = std::max(0.0, _prediction_draw_future_segment ? _prediction_future_window_s : 0.0);
            const double margin_s = std::max(0.0, static_cast<double>(fixed_dt));
            rebuild_prediction = (cache_end_s - now_s) < (required_ahead_s + margin_s);
        }

        if (rebuild_prediction)
        {
            update_orbit_prediction_cache(ship_pos_world, ship_vel_world, thrusting);
            _prediction_dirty = !_prediction_cache.valid;
        }
    }

    bool GameplayState::player_thrust_applied_this_tick() const
    {
        if (_rails_warp_active && _time_warp.mode == TimeWarpState::Mode::RailsWarp)
        {
            return _rails_thrust_applied_this_tick;
        }

        const EntityId player_eid = player_entity();
        if (!player_eid.is_valid())
        {
            return false;
        }

        const Entity *player = _world.entities().find(player_eid);
        if (!player)
        {
            return false;
        }

        const ShipController *sc = player->get_component<ShipController>();
        return sc && sc->thrust_applied_this_tick();
    }

    WorldVec3 GameplayState::prediction_reference_body_world() const
    {
        if (!_orbitsim)
        {
            return _scenario_config.system_center;
        }

        const CelestialBodyInfo *ref_info = _orbitsim->reference_body();
        if (ref_info && ref_info->render_entity.is_valid())
        {
            if (const Entity *ref_entity = _world.entities().find(ref_info->render_entity))
            {
                return ref_entity->position_world();
            }
        }

        return _scenario_config.system_center;
    }

    void GameplayState::refresh_prediction_world_points()
    {
        if (!_prediction_cache.valid || _prediction_cache.trajectory_bci.empty())
        {
            _prediction_cache.points_world.clear();
            return;
        }

        const WorldVec3 ref_body_world = prediction_reference_body_world();
        _prediction_cache.points_world.resize(_prediction_cache.trajectory_bci.size());

        for (size_t i = 0; i < _prediction_cache.trajectory_bci.size(); ++i)
        {
            _prediction_cache.points_world[i] =
                    ref_body_world + WorldVec3(_prediction_cache.trajectory_bci[i].position_m);
        }
    }

    void GameplayState::update_orbit_prediction_cache(const WorldVec3 &ship_pos_world,
                                                      const glm::dvec3 &ship_vel_world,
                                                      const bool thrusting)
    {
        _prediction_cache.clear();

        if (!_orbitsim)
        {
            return;
        }

        const double build_sim_time_s = _orbitsim->sim.time_s();

        const CelestialBodyInfo *ref_info = _orbitsim->reference_body();
        if (!ref_info || !(ref_info->mass_kg > 0.0))
        {
            return;
        }

        const double mu_ref_m3_s2 = _orbitsim->sim.config().gravitational_constant * ref_info->mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return;
        }

        const double planet_radius_m = ref_info->radius_m;
        const WorldVec3 ref_body_world = prediction_reference_body_world();
        const glm::dvec3 ship_rel_pos_m = glm::dvec3(ship_pos_world - ref_body_world);
        const glm::dvec3 ship_rel_vel_mps = ship_vel_world;

        const auto [horizon_s_auto, dt_s_auto] = select_prediction_horizon_and_dt(
                mu_ref_m3_s2, ship_rel_pos_m, ship_rel_vel_mps);

        double horizon_s = std::clamp(horizon_s_auto, 60.0, 36'000.0);
        double dt_s = std::clamp(dt_s_auto, 0.01, 60.0);
        int max_steps = 2'000;

        if (thrusting)
        {
            // While thrusting, prioritize responsiveness over long-horizon stability.
            // A shorter horizon lets us rebuild more frequently without visible hitching.
            const double thrust_horizon_cap_s =
                    std::clamp(std::max(120.0, _prediction_future_window_s * 1.25), 120.0, 3'600.0);
            horizon_s = std::min(horizon_s, thrust_horizon_cap_s);

            const double target_samples = std::clamp(horizon_s / 1.0, 300.0, 800.0);
            dt_s = std::clamp(horizon_s / target_samples, 0.02, 20.0);
            max_steps = 1'000;
        }

        horizon_s = std::clamp(horizon_s, dt_s, 36'000.0);
        const int steps = std::clamp(static_cast<int>(std::ceil(horizon_s / dt_s)), 2, max_steps);

        auto fill_cache_from_trajectory = [&](std::vector<orbitsim::TrajectorySample> trajectory_bci) -> bool {
            if (trajectory_bci.size() < 2)
            {
                return false;
            }

            _prediction_cache.trajectory_bci = std::move(trajectory_bci);
            _prediction_cache.altitude_km.reserve(_prediction_cache.trajectory_bci.size());
            _prediction_cache.speed_kmps.reserve(_prediction_cache.trajectory_bci.size());

            for (const orbitsim::TrajectorySample &sample : _prediction_cache.trajectory_bci)
            {
                const double r_m = safe_length(sample.position_m);
                const double alt_km = (r_m - planet_radius_m) * 1.0e-3;
                const double spd_kmps = safe_length(sample.velocity_mps) * 1.0e-3;
                _prediction_cache.altitude_km.push_back(static_cast<float>(alt_km));
                _prediction_cache.speed_kmps.push_back(static_cast<float>(spd_kmps));
            }

            const OrbitalElementsEstimate elements =
                    compute_orbital_elements(mu_ref_m3_s2, ship_rel_pos_m, ship_rel_vel_mps);
            if (elements.valid)
            {
                _prediction_cache.semi_major_axis_m = elements.semi_major_axis_m;
                _prediction_cache.eccentricity = elements.eccentricity;
                _prediction_cache.orbital_period_s = elements.orbital_period_s;
                _prediction_cache.periapsis_alt_km = (elements.periapsis_m - planet_radius_m) * 1.0e-3;
                _prediction_cache.apoapsis_alt_km = std::isfinite(elements.apoapsis_m)
                                                        ? (elements.apoapsis_m - planet_radius_m) * 1.0e-3
                                                        : std::numeric_limits<double>::infinity();
            }

            _prediction_cache.build_time_s = build_sim_time_s;
            _prediction_cache.build_pos_world = ship_pos_world;
            _prediction_cache.build_vel_world = ship_vel_world;
            _prediction_cache.valid = true;
            refresh_prediction_world_points();
            return true;
        };

        OrbitalScenario scenario_pred = *_orbitsim;
        const orbitsim::MassiveBody *ref_sim = scenario_pred.reference_sim_body();
        if (!ref_sim)
        {
            return;
        }

        const glm::dvec3 ship_bary_pos_m = ref_sim->state.position_m + ship_rel_pos_m;
        const glm::dvec3 ship_bary_vel_mps = ref_sim->state.velocity_mps + ship_rel_vel_mps;

        orbitsim::Spacecraft ship_sc{};
        ship_sc.state = orbitsim::make_state(ship_bary_pos_m, ship_bary_vel_mps);
        ship_sc.dry_mass_kg = 1.0;

        const auto ship_h = scenario_pred.sim.create_spacecraft(ship_sc);
        if (!ship_h.valid())
        {
            return;
        }

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

        const orbitsim::CelestialEphemeris eph = orbitsim::build_celestial_ephemeris(scenario_pred.sim, opt);
        const std::vector<orbitsim::TrajectorySample> traj_inertial =
                orbitsim::predict_spacecraft_trajectory(scenario_pred.sim, eph, ship_h.id, opt);

        if (traj_inertial.empty())
        {
            return;
        }

        const std::vector<orbitsim::TrajectorySample> traj_centered =
                orbitsim::trajectory_to_body_centered_inertial(traj_inertial, eph, *ref_sim);
        if (!fill_cache_from_trajectory(traj_centered))
        {
            return;
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

        if (!_orbitsim || !_prediction_cache.valid)
        {
            return;
        }

        refresh_prediction_world_points();
        if (_prediction_cache.points_world.size() < 2 ||
            _prediction_cache.trajectory_bci.size() != _prediction_cache.points_world.size())
        {
            return;
        }

        const double t0 = _prediction_cache.trajectory_bci.front().t_s;
        const double t1 = _prediction_cache.trajectory_bci.back().t_s;
        if (!(t1 > t0))
        {
            return;
        }

        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double interp_dt_s =
                (_last_sim_step_dt_s > 0.0) ? _last_sim_step_dt_s : static_cast<double>(ctx.fixed_delta_time());
        double now_s = _orbitsim->sim.time_s();

        // Match render interpolation: entities are rendered between prev/curr using `alpha_f`,
        // so treat "now" as within the previous->current fixed step interval.
        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }

        if (!std::isfinite(now_s))
        {
            return;
        }

        now_s = std::clamp(now_s, t0, t1);

        // Debug commands are pruned in engine draw begin_frame(dt) after update_scene(), so ttl must
        // be > dt to survive until the current frame is rendered.
        const float ttl_s = std::clamp(ctx.delta_time(), 0.0f, 0.1f) + 0.002f;

        constexpr glm::vec4 color_orbit_full{0.2f, 0.9f, 0.2f, 0.22f};
        constexpr glm::vec4 color_orbit_future{0.2f, 0.9f, 0.2f, 0.75f};
        constexpr glm::vec4 color_velocity{1.0f, 0.35f, 0.1f, 1.0f};

        WorldVec3 ship_pos_world_state{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (!get_player_world_state(ship_pos_world_state, ship_vel_world, ship_vel_local_f))
        {
            return;
        }

        WorldVec3 ship_pos_world = ship_pos_world_state;
        const EntityId player_eid = player_entity();
        if (const Entity *player = _world.entities().find(player_eid))
        {
            ship_pos_world = player->get_render_position_world(alpha_f);
        }

        const WorldVec3 ref_body_world = prediction_reference_body_world();

        auto lower_bound_by_time = [&](double t_s) {
            return std::lower_bound(_prediction_cache.trajectory_bci.cbegin(),
                                    _prediction_cache.trajectory_bci.cend(),
                                    t_s,
                                    [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
        };

        const auto it_hi = lower_bound_by_time(now_s);
        size_t i_hi = static_cast<size_t>(std::distance(_prediction_cache.trajectory_bci.cbegin(), it_hi));
        if (i_hi >= _prediction_cache.trajectory_bci.size())
        {
            return;
        }

        // Align the curve to the ship at "now" to hide polyline chord error and keep the plot
        // visually attached even with small solver/physics drift.
        WorldVec3 predicted_now_world = _prediction_cache.points_world[i_hi];
        if (i_hi > 0)
        {
            predicted_now_world = hermite_position_world(ref_body_world,
                                                        _prediction_cache.trajectory_bci[i_hi - 1],
                                                        _prediction_cache.trajectory_bci[i_hi],
                                                        now_s);
        }

        WorldVec3 align_delta = ship_pos_world - predicted_now_world;
        const double align_len = glm::length(glm::dvec3(align_delta));
        if (!std::isfinite(align_len) || align_len > 10'000.0)
        {
            align_delta = WorldVec3(0.0, 0.0, 0.0);
        }

        auto draw_world = [&](const WorldVec3 &p_world) -> WorldVec3 { return p_world + align_delta; };

        auto draw_hermite = [&](const orbitsim::TrajectorySample &a,
                                const orbitsim::TrajectorySample &b,
                                const double t_s) -> WorldVec3 {
            return hermite_position_world(ref_body_world, a, b, t_s) + align_delta;
        };

        auto draw_window = [&](double t_start_s, double t_end_s, const glm::vec4 &color, WorldVec3 prev_world) {
            if (!(t_end_s > t_start_s))
            {
                return;
            }

            const auto &traj = _prediction_cache.trajectory_bci;
            const size_t n = traj.size();
            if (n < 2)
            {
                return;
            }

            auto it_start_hi = lower_bound_by_time(t_start_s);
            size_t i_start_hi = static_cast<size_t>(std::distance(traj.cbegin(), it_start_hi));
            if (i_start_hi >= n)
            {
                return;
            }

            size_t seg = (i_start_hi == 0) ? 0 : (i_start_hi - 1);
            double t = std::clamp(t_start_s, traj.front().t_s, traj.back().t_s);
            const double t_end = std::clamp(t_end_s, traj.front().t_s, traj.back().t_s);

            while (t < t_end && (seg + 1) < n)
            {
                const orbitsim::TrajectorySample &a = traj[seg];
                const orbitsim::TrajectorySample &b = traj[seg + 1];

                const double seg_start = std::max(t, a.t_s);
                const double seg_end = std::min(t_end, b.t_s);
                const double seg_len = seg_end - seg_start;
                if (!(seg_len > 0.0) || !std::isfinite(seg_len))
                {
                    ++seg;
                    continue;
                }

                const int sub = std::max(1, static_cast<int>(std::ceil(seg_len / kOrbitDrawMaxDtS)));
                for (int j = 1; j <= sub; ++j)
                {
                    const double u = static_cast<double>(j) / static_cast<double>(sub);
                    const double tj = seg_start + seg_len * u;
                    const WorldVec3 p = draw_hermite(a, b, tj);
                    ctx.api->debug_draw_line(glm::dvec3(prev_world), glm::dvec3(p), color, ttl_s, true);
                    prev_world = p;
                }

                t = seg_end;
                if (t >= b.t_s)
                {
                    ++seg;
                }
            }
        };

        // Full orbit (typically one period) for context.
        if (_prediction_draw_full_orbit)
        {
            double t_full_end = t1;
            if (_prediction_cache.orbital_period_s > 0.0 && std::isfinite(_prediction_cache.orbital_period_s))
            {
                t_full_end = std::min(t0 + _prediction_cache.orbital_period_s, t1);
            }

            draw_window(t0, t_full_end, color_orbit_full, draw_world(_prediction_cache.points_world.front()));
        }

        // Future segment highlight (windowed).
        if (_prediction_draw_future_segment)
        {
            const double window_s = std::max(0.0, _prediction_future_window_s);
            const double t_end = (window_s > 0.0) ? std::min(now_s + window_s, t1) : t1;

            draw_window(now_s, t_end, color_orbit_future, ship_pos_world);
        }

        if (_prediction_draw_velocity_ray)
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
