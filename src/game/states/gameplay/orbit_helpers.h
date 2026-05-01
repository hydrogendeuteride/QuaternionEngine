#pragma once

#include "game/states/gameplay/orbit_runtime_types.h"

#include "orbitsim/orbit_utils.hpp"

#include <cmath>
#include <glm/glm.hpp>

#include "physics/physics_body.h"

namespace Game
{
    // ============================================================================
    // Helpers (detail namespace)
    // ============================================================================

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

        // Acceleration in a translating reference-body-centered frame:
        //   a_rel = a_sc_bary - a_ref_bary
        // where barycentric acceleration is computed from all massive bodies(tidal force).
        inline glm::dvec3 nbody_accel_body_centered(const OrbitalScenario &scenario, const glm::dvec3 &p_rel_m)
        {
            const orbitsim::MassiveBody *ref = scenario.world_reference_sim_body();
            if (!ref)
            {
                return glm::dvec3(0.0);
            }

            const orbitsim::BodyId ref_id = ref->id;
            const double G = scenario.sim.config().gravitational_constant;
            const double eps_m = scenario.sim.config().softening_length_m;
            const double eps2 = eps_m * eps_m;

            const glm::dvec3 p_ref_bary = ref->state.position_m;
            const glm::dvec3 p_sc_bary = p_ref_bary + p_rel_m;

            glm::dvec3 a_sc_bary(0.0);
            glm::dvec3 a_ref_bary(0.0);

            // Acceleration from reference body on spacecraft
            a_sc_bary += point_mass_accel(G, ref->mass_kg, p_rel_m, eps2);

            // Acceleration from all other bodies
            for (const orbitsim::MassiveBody &body : scenario.sim.massive_bodies())
            {
                if (body.id == ref_id)
                {
                    continue;
                }

                a_sc_bary += point_mass_accel(G, body.mass_kg, p_sc_bary - glm::dvec3(body.state.position_m), eps2);
                a_ref_bary += point_mass_accel(G, body.mass_kg,
                                                p_ref_bary - glm::dvec3(body.state.position_m), eps2);
            }

            return a_sc_bary - a_ref_bary;
        }
    } // namespace detail
} // namespace Game
