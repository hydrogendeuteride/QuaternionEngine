#include "entity.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>

namespace Game
{

// ============================================================================
// Transform
// ============================================================================

glm::mat4 Transform::to_local_matrix(const WorldVec3& origin_world) const
{
    const glm::vec3 local_pos = world_to_local(position_world, origin_world);
    glm::mat4 m = glm::mat4(1.0f);
    m = glm::translate(m, local_pos);
    m = m * glm::mat4_cast(rotation);
    m = glm::scale(m, scale);
    return m;
}

Transform Transform::operator*(const Transform& child) const
{
    Transform result;

    // Combine rotations
    result.rotation = rotation * child.rotation;

    // Scale child position by parent scale, rotate, then add to parent world position
    const glm::vec3 child_local = glm::vec3(child.position_world); // child position as local offset
    const glm::vec3 scaled_child_pos = scale * child_local;
    const glm::dvec3 offset = glm::dvec3(rotation * scaled_child_pos);
    result.position_world = position_world + offset;

    // Combine scales
    result.scale = scale * child.scale;

    return result;
}

// ============================================================================
// InterpolatedTransform
// ============================================================================

WorldVec3 InterpolatedTransform::interpolated_position(float alpha) const
{
    const double a = static_cast<double>(alpha);
    return prev_position + (curr_position - prev_position) * a;
}

glm::quat InterpolatedTransform::interpolated_rotation(float alpha) const
{
    return glm::slerp(prev_rotation, curr_rotation, alpha);
}

void InterpolatedTransform::store_current_as_previous()
{
    prev_position = curr_position;
    prev_rotation = curr_rotation;
}

void InterpolatedTransform::set_immediate(const WorldVec3& pos, const glm::quat& rot)
{
    prev_position = pos;
    curr_position = pos;
    prev_rotation = rot;
    curr_rotation = rot;
}

// ============================================================================
// Attachment
// ============================================================================

glm::mat4 Attachment::get_local_matrix() const
{
    glm::mat4 m = glm::mat4(1.0f);
    m = glm::translate(m, local_position);
    m = m * glm::mat4_cast(local_rotation);
    m = glm::scale(m, local_scale);
    return m;
}

Transform Attachment::get_local_transform() const
{
    Transform t;
    // Attachment local_position is treated as a local offset, stored in position_world
    t.position_world = WorldVec3(local_position);
    t.rotation = local_rotation;
    t.scale = local_scale;
    return t;
}

// ============================================================================
// Entity
// ============================================================================

Entity::Entity(EntityId id, const std::string& name)
    : _id(id), _name(name)
{
}

WorldVec3 Entity::get_render_position_world(float alpha) const
{
    if (_use_interpolation)
    {
        return _interp.interpolated_position(alpha);
    }
    return _transform.position_world;
}

glm::quat Entity::get_render_rotation(float alpha) const
{
    if (_use_interpolation)
    {
        return _interp.interpolated_rotation(alpha);
    }
    return _transform.rotation;
}

glm::mat4 Entity::get_render_local_matrix(float alpha, const WorldVec3& origin_world) const
{
    const WorldVec3 pos_world = get_render_position_world(alpha);
    const glm::vec3 pos_local = world_to_local(pos_world, origin_world);
    const glm::quat rot = get_render_rotation(alpha);

    glm::mat4 m = glm::mat4(1.0f);
    m = glm::translate(m, pos_local);
    m = m * glm::mat4_cast(rot);
    m = glm::scale(m, _transform.scale);
    return m;
}

void Entity::add_attachment(const Attachment& attachment)
{
    // Check for duplicate name
    auto it = std::find_if(_attachments.begin(), _attachments.end(),
                           [&](const Attachment& a) { return a.name == attachment.name; });

    if (it != _attachments.end())
    {
        // Replace existing
        *it = attachment;
    }
    else
    {
        _attachments.push_back(attachment);
    }
}

bool Entity::remove_attachment(const std::string& name)
{
    auto it = std::find_if(_attachments.begin(), _attachments.end(),
                           [&](const Attachment& a) { return a.name == name; });

    if (it != _attachments.end())
    {
        _attachments.erase(it);
        return true;
    }
    return false;
}

Attachment* Entity::find_attachment(const std::string& name)
{
    auto it = std::find_if(_attachments.begin(), _attachments.end(),
                           [&](const Attachment& a) { return a.name == name; });

    if (it != _attachments.end())
    {
        return &(*it);
    }
    return nullptr;
}

const Attachment* Entity::find_attachment(const std::string& name) const
{
    auto it = std::find_if(_attachments.begin(), _attachments.end(),
                           [&](const Attachment& a) { return a.name == name; });

    if (it != _attachments.end())
    {
        return &(*it);
    }
    return nullptr;
}

glm::mat4 Entity::get_attachment_local_matrix(const Attachment& att, const WorldVec3& origin_world) const
{
    return _transform.to_local_matrix(origin_world) * att.get_local_matrix();
}

glm::mat4 Entity::get_attachment_local_matrix(const Attachment& att, float alpha, const WorldVec3& origin_world) const
{
    return get_render_local_matrix(alpha, origin_world) * att.get_local_matrix();
}

} // namespace Game
