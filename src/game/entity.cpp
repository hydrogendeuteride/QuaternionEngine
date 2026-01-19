#include "entity.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>

namespace Game
{

// ============================================================================
// Transform
// ============================================================================

glm::mat4 Transform::to_matrix() const
{
    glm::mat4 m = glm::mat4(1.0f);
    m = glm::translate(m, position);
    m = m * glm::mat4_cast(rotation);
    m = glm::scale(m, scale);
    return m;
}

Transform Transform::from_matrix(const glm::mat4& m)
{
    Transform t;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(m, t.scale, t.rotation, t.position, skew, perspective);
    return t;
}

Transform Transform::operator*(const Transform& child) const
{
    Transform result;

    // Combine rotations
    result.rotation = rotation * child.rotation;

    // Scale child position by parent scale, rotate, then add parent position
    glm::vec3 scaled_child_pos = scale * child.position;
    result.position = position + rotation * scaled_child_pos;

    // Combine scales
    result.scale = scale * child.scale;

    return result;
}

// ============================================================================
// InterpolatedTransform
// ============================================================================

glm::vec3 InterpolatedTransform::interpolated_position(float alpha) const
{
    return glm::mix(prev_position, curr_position, alpha);
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

void InterpolatedTransform::set_immediate(const glm::vec3& pos, const glm::quat& rot)
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
    t.position = local_position;
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

glm::vec3 Entity::get_render_position(float alpha) const
{
    if (_use_interpolation)
    {
        return _interp.interpolated_position(alpha);
    }
    return _transform.position;
}

glm::quat Entity::get_render_rotation(float alpha) const
{
    if (_use_interpolation)
    {
        return _interp.interpolated_rotation(alpha);
    }
    return _transform.rotation;
}

glm::mat4 Entity::get_render_matrix(float alpha) const
{
    if (_use_interpolation)
    {
        glm::mat4 m = glm::mat4(1.0f);
        m = glm::translate(m, _interp.interpolated_position(alpha));
        m = m * glm::mat4_cast(_interp.interpolated_rotation(alpha));
        m = glm::scale(m, _transform.scale);
        return m;
    }
    return _transform.to_matrix();
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

glm::mat4 Entity::get_attachment_world_matrix(const Attachment& att) const
{
    return _transform.to_matrix() * att.get_local_matrix();
}

glm::mat4 Entity::get_attachment_world_matrix(const Attachment& att, float alpha) const
{
    return get_render_matrix(alpha) * att.get_local_matrix();
}

} // namespace Game
