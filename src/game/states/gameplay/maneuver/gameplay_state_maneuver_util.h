#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"

#include <glm/glm.hpp>

namespace Game::ManeuverUtil
{
    inline glm::dvec3 compose_basis_vector(const glm::dvec3 &components,
                                           const glm::dvec3 &basis_r,
                                           const glm::dvec3 &basis_t,
                                           const glm::dvec3 &basis_n)
    {
        return basis_r * components.x + basis_t * components.y + basis_n * components.z;
    }

    inline glm::dvec3 project_basis_vector(const glm::dvec3 &world_vec,
                                           const glm::dvec3 &basis_r,
                                           const glm::dvec3 &basis_t,
                                           const glm::dvec3 &basis_n)
    {
        return glm::dvec3(glm::dot(world_vec, basis_r),
                          glm::dot(world_vec, basis_t),
                          glm::dot(world_vec, basis_n));
    }

    inline glm::dvec3 dv_rtn_to_display_basis(const ManeuverNode &node)
    {
        const glm::dvec3 dv_world = compose_basis_vector(node.dv_rtn_mps,
                                                          node.maneuver_basis_r_world,
                                                          node.maneuver_basis_t_world,
                                                          node.maneuver_basis_n_world);
        return project_basis_vector(dv_world,
                                    node.basis_r_world,
                                    node.basis_t_world,
                                    node.basis_n_world);
    }

    inline const char *basis_mode_label(const ManeuverGizmoBasisMode mode)
    {
        switch (mode)
        {
            case ManeuverGizmoBasisMode::ProgradeOutwardNormal:
                return "Prograde / Outward / Normal";
            case ManeuverGizmoBasisMode::RTN:
                return "RTN";
            default:
                return "Unknown";
        }
    }

    inline const char *axis_short_label(const ManeuverGizmoBasisMode mode,
                                        const ManeuverHandleAxis axis)
    {
        switch (axis)
        {
            case ManeuverHandleAxis::TangentialPos:
                return mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal ? "+P" : "+T";
            case ManeuverHandleAxis::TangentialNeg:
                return mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal ? "-P" : "-T";
            case ManeuverHandleAxis::RadialPos:
                return mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal ? "+O" : "+R";
            case ManeuverHandleAxis::RadialNeg:
                return mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal ? "-O" : "-R";
            case ManeuverHandleAxis::NormalPos:
                return "+N";
            case ManeuverHandleAxis::NormalNeg:
                return "-N";
            default:
                return "";
        }
    }
} // namespace Game::ManeuverUtil
