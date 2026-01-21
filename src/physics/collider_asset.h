#pragma once

#include "collision_shape.h"

#include <optional>

namespace Physics
{
    // Utilities for working with collision shapes loaded from assets (e.g., glTF collider sidecars).
    // Keep this minimal: game code can opt-in to scaling shapes for uniformly-scaled instances.

    // Returns a uniformly scaled copy of a compound shape.
    // - Scales child positions and primitive dimensions by |uniform_scale|.
    // - Returns an empty compound if scale is non-finite or <= 0.
    CompoundShape scale_compound_uniform(const CompoundShape &compound, float uniform_scale);

    // Returns a uniformly scaled copy of a collision shape.
    // - Supports primitives and compound shapes.
    // - Returns nullopt if scale is non-finite or <= 0.
    std::optional<CollisionShape> scale_collision_shape_uniform(const CollisionShape &shape, float uniform_scale);
} // namespace Physics

