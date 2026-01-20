#pragma once

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT

#include "physics/physics_body.h"

#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace Physics::JoltQuery
{
    class LayerMaskFilter final : public JPH::BroadPhaseLayerFilter, public JPH::ObjectLayerFilter
    {
    public:
        explicit LayerMaskFilter(uint32_t layer_mask) : _layer_mask(layer_mask)
        {
        }

        bool ShouldCollide(JPH::BroadPhaseLayer) const override { return true; }

        bool ShouldCollide(JPH::ObjectLayer layer) const override
        {
            return (_layer_mask & (1u << layer)) != 0;
        }

    private:
        uint32_t _layer_mask{0};
    };

    class IgnoreBodyAndSensorsFilter final : public JPH::BodyFilter
    {
    public:
        IgnoreBodyAndSensorsFilter(BodyId ignore_body, bool include_sensors, const JPH::BodyLockInterface &lock_interface)
            : _ignore_body(ignore_body)
              , _include_sensors(include_sensors)
              , _lock_interface(lock_interface)
        {
        }

        bool ShouldCollide(const JPH::BodyID &body_id) const override
        {
            if (_ignore_body.is_valid() && body_id.GetIndexAndSequenceNumber() == _ignore_body.value)
            {
                return false;
            }

            if (!_include_sensors)
            {
                JPH::BodyLockRead lock(_lock_interface, body_id);
                if (lock.Succeeded() && lock.GetBody().IsSensor())
                {
                    return false;
                }
            }

            return true;
        }

        bool ShouldCollideLocked(const JPH::Body &body) const override
        {
            if (_ignore_body.is_valid() && body.GetID().GetIndexAndSequenceNumber() == _ignore_body.value)
            {
                return false;
            }

            if (!_include_sensors && body.IsSensor())
            {
                return false;
            }

            return true;
        }

    private:
        BodyId _ignore_body;
        bool _include_sensors{true};
        const JPH::BodyLockInterface &_lock_interface;
    };
}

#endif // defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT

