#include "physics_body.h"
#include "physics_world.h"
#include <glm/gtc/matrix_transform.hpp>

namespace Physics
{

// ============================================================================
// BodyHandle implementation
// ============================================================================

BodyHandle::BodyHandle(PhysicsWorld* world, BodyId id)
    : _world(world)
    , _id(id)
{
}

BodyHandle::~BodyHandle()
{
    if (_world && _id.is_valid())
    {
        _world->destroy_body(_id);
    }
}

BodyHandle::BodyHandle(BodyHandle&& other) noexcept
    : _world(other._world)
    , _id(other._id)
{
    other._world = nullptr;
    other._id = BodyId{};
}

BodyHandle& BodyHandle::operator=(BodyHandle&& other) noexcept
{
    if (this != &other)
    {
        // Destroy current body if any
        if (_world && _id.is_valid())
        {
            _world->destroy_body(_id);
        }

        _world = other._world;
        _id = other._id;

        other._world = nullptr;
        other._id = BodyId{};
    }
    return *this;
}

BodyId BodyHandle::release()
{
    BodyId id = _id;
    _world = nullptr;
    _id = BodyId{};
    return id;
}

// ============================================================================
// BodyTransform implementation
// ============================================================================

glm::mat4 BodyTransform::to_matrix() const
{
    return glm::translate(glm::mat4(1.0f), position) * glm::mat4_cast(rotation);
}

} // namespace Physics
