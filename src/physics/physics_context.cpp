#include "physics_context.h"
#include "physics_world.h"
#include "physics_body.h"

#include <glm/gtx/norm.hpp>

namespace Physics
{
    bool PhysicsContext::set_origin_world(const WorldVec3 &new_origin)
    {
        if (is_zero(new_origin - _origin_world))
        {
            return false;
        }
        _origin_world = new_origin;
        ++_origin_revision;
        return true;
    }

    bool PhysicsContext::set_velocity_origin_world(const glm::dvec3 &new_velocity)
    {
        if (is_zero(new_velocity - _velocity_origin_world))
        {
            return false;
        }
        _velocity_origin_world = new_velocity;
        ++_velocity_origin_revision;
        return true;
    }

    void PhysicsContext::set_anchor_world(const WorldVec3 &anchor)
    {
        _anchor_world = anchor;
        _anchor_enabled = true;
    }

    void PhysicsContext::clear_anchor()
    {
        _anchor_enabled = false;
    }

    bool PhysicsContext::maybe_rebase_origin_to_body(uint32_t body_value, double threshold_m, double snap_m)
    {
        if (!_physics)
        {
            return false;
        }

        BodyId body_id{body_value};
        if (!_physics->is_body_valid(body_id))
        {
            return false;
        }

        const glm::dvec3 p_local = _physics->get_position(body_id);
        const double dist2 = glm::dot(p_local, p_local);
        const double threshold2 = (threshold_m <= 0.0) ? 0.0 : (threshold_m * threshold_m);
        if (dist2 <= threshold2)
        {
            return false;
        }

        const WorldVec3 origin_before = _origin_world;
        const WorldVec3 anchor_world = origin_before + WorldVec3(p_local);
        const WorldVec3 new_origin = (snap_m > 0.0) ? snap_world(anchor_world, snap_m) : anchor_world;

        const glm::dvec3 delta_local = origin_before - new_origin;
        _physics->shift_origin(delta_local);

        (void) set_origin_world(new_origin);
        return true;
    }

    bool PhysicsContext::maybe_rebase_velocity_to_body(uint32_t body_value, double threshold_mps)
    {
        if (!_physics)
        {
            return false;
        }

        BodyId body_id{body_value};
        if (!_physics->is_body_valid(body_id))
        {
            return false;
        }

        const glm::vec3 v_local_f = _physics->get_linear_velocity(body_id);
        const double speed2 = static_cast<double>(glm::dot(v_local_f, v_local_f));
        const double threshold2 = (threshold_mps <= 0.0) ? 0.0 : (threshold_mps * threshold_mps);
        if (speed2 <= threshold2)
        {
            return false;
        }

        const glm::dvec3 delta_v_local(v_local_f);
        _physics->shift_velocity_origin(delta_v_local);

        _velocity_origin_world += delta_v_local;
        ++_velocity_origin_revision;
        return true;
    }
} // namespace Physics
