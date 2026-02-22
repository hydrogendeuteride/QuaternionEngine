#pragma once

#include "orbitsim/game_sim.hpp"
#include "orbitsim/orbit_utils.hpp"

#include <cmath>
#include <glm/glm.hpp>

#include "physics/physics_world.h"

namespace Game
{
    struct OrbitsimDemo
    {
        orbitsim::GameSimulation sim{};
        orbitsim::BodyId earth_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId moon_id{orbitsim::kInvalidBodyId};

        double earth_mass_kg{0.0};
    };

    namespace detail
    {
        inline bool finite_vec3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        inline const char *contact_event_type_name(Physics::ContactEventType type)
        {
            switch (type)
            {
                case Physics::ContactEventType::Begin:
                    return "Begin";
                case Physics::ContactEventType::Stay:
                    return "Stay";
                case Physics::ContactEventType::End:
                    return "End";
            }
            return "Unknown";
        }

        struct OrbitRelativeState
        {
            orbitsim::Vec3 position_m{0.0, 0.0, 0.0};
            orbitsim::Vec3 velocity_mps{0.0, 0.0, 0.0};
        };

        inline OrbitRelativeState circular_orbit_relative_state_xz(const double gravitational_constant,
                                                                    const double central_mass_kg,
                                                                    const double orbital_radius_m,
                                                                    const double arg_latitude_rad = 0.0)
        {
            if (!(central_mass_kg > 0.0) || !(orbital_radius_m > 0.0) || !std::isfinite(gravitational_constant))
            {
                return {};
            }

            const double mu = gravitational_constant * central_mass_kg;
            const double v_circ = std::sqrt(mu / orbital_radius_m);

            const double cos_u = std::cos(arg_latitude_rad);
            const double sin_u = std::sin(arg_latitude_rad);

            OrbitRelativeState out;
            out.position_m = orbitsim::Vec3{orbital_radius_m * cos_u, 0.0, orbital_radius_m * sin_u};
            out.velocity_mps = orbitsim::Vec3{-v_circ * sin_u, 0.0, v_circ * cos_u};
            return out;
        }

        inline orbitsim::TwoBodyBarycentricStates two_body_circular_barycentric_xz(
                const double gravitational_constant,
                const double mass_a_kg,
                const double mass_b_kg,
                const double separation_m,
                const double arg_latitude_rad = 0.0)
        {
            const double m_tot = mass_a_kg + mass_b_kg;
            if (!(m_tot > 0.0) || !(separation_m > 0.0) || !std::isfinite(m_tot))
            {
                return {};
            }

            const OrbitRelativeState rel = circular_orbit_relative_state_xz(
                    gravitational_constant, m_tot, separation_m, arg_latitude_rad);

            const double frac_a = mass_b_kg / m_tot;
            const double frac_b = mass_a_kg / m_tot;

            orbitsim::TwoBodyBarycentricStates out;
            out.state_a = orbitsim::make_state(-frac_a * rel.position_m, -frac_a * rel.velocity_mps);
            out.state_b = orbitsim::make_state(frac_b * rel.position_m, frac_b * rel.velocity_mps);
            return out;
        }

        inline glm::dvec3 point_mass_accel(const double gravitational_constant,
                                            const double mass_kg,
                                            const glm::dvec3 &r_m,
                                            const double softening_length2_m2)
        {
            if (!(gravitational_constant > 0.0) || !(mass_kg > 0.0))
            {
                return glm::dvec3(0.0);
            }

            const double r2 = glm::dot(r_m, r_m) + softening_length2_m2;
            if (!std::isfinite(r2) || r2 <= 0.0)
            {
                return glm::dvec3(0.0);
            }

            const double inv_r = 1.0 / std::sqrt(r2);
            const double inv_r3 = inv_r * inv_r * inv_r;
            const glm::dvec3 a = (-gravitational_constant * mass_kg) * r_m * inv_r3;

            if (!finite_vec3(a))
            {
                return glm::dvec3(0.0);
            }

            return a;
        }

        // Acceleration in a translating Earth-centered frame:
        //   a_rel = a_sc_bary - a_earth_bary
        // where barycentric acceleration is computed from all massive bodies.
        inline glm::dvec3 orbitsim_nbody_accel_earth_fixed(const OrbitsimDemo &demo, const glm::dvec3 &p_rel_m)
        {
            const orbitsim::MassiveBody *earth = demo.sim.body_by_id(demo.earth_id);
            if (!earth)
            {
                return glm::dvec3(0.0);
            }

            const double G = demo.sim.config().gravitational_constant;
            const double eps_m = demo.sim.config().softening_length_m;
            const double eps2 = eps_m * eps_m;

            const glm::dvec3 p_earth_bary = earth->state.position_m;
            const glm::dvec3 p_sc_bary = p_earth_bary + p_rel_m;

            glm::dvec3 a_sc_bary(0.0);
            glm::dvec3 a_earth_bary(0.0);

            a_sc_bary += point_mass_accel(G, earth->mass_kg, p_rel_m, eps2);

            for (const orbitsim::MassiveBody &body: demo.sim.massive_bodies())
            {
                if (body.id == demo.earth_id)
                {
                    continue;
                }

                a_sc_bary += point_mass_accel(G, body.mass_kg, p_sc_bary - glm::dvec3(body.state.position_m), eps2);
                a_earth_bary += point_mass_accel(G, body.mass_kg,
                                                  p_earth_bary - glm::dvec3(body.state.position_m), eps2);
            }

            return a_sc_bary - a_earth_bary;
        }
    } // namespace detail
} // namespace Game
