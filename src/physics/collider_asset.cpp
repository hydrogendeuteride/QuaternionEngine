#include "collider_asset.h"

#include <cmath>

namespace Physics
{
    namespace
    {
        static bool valid_uniform_scale(float s)
        {
            return std::isfinite(s) && s > 0.0f;
        }

        static PrimitiveShapeVariant scale_primitive_uniform(const PrimitiveShapeVariant &shape, float s)
        {
            return std::visit([s](const auto &prim) -> PrimitiveShapeVariant {
                using T = std::decay_t<decltype(prim)>;

                if constexpr (std::is_same_v<T, BoxShape>)
                {
                    return BoxShape{prim.half_extents * s};
                }
                else if constexpr (std::is_same_v<T, SphereShape>)
                {
                    return SphereShape{prim.radius * s};
                }
                else if constexpr (std::is_same_v<T, CapsuleShape>)
                {
                    return CapsuleShape{prim.radius * s, prim.half_height * s};
                }
                else if constexpr (std::is_same_v<T, CylinderShape>)
                {
                    return CylinderShape{prim.radius * s, prim.half_height * s};
                }
                else if constexpr (std::is_same_v<T, TaperedCylinderShape>)
                {
                    return TaperedCylinderShape{prim.half_height * s, prim.top_radius * s, prim.bottom_radius * s};
                }
                else if constexpr (std::is_same_v<T, PlaneShape>)
                {
                    // Uniform scaling of a plane keeps its normal, scales its offset.
                    return PlaneShape{prim.normal, prim.offset * s};
                }
                else
                {
                    return prim;
                }
            }, shape);
        }
    } // namespace

    CompoundShape scale_compound_uniform(const CompoundShape &compound, float uniform_scale)
    {
        CompoundShape out{};
        if (!valid_uniform_scale(uniform_scale))
        {
            return out;
        }

        const float s = std::abs(uniform_scale);
        out.children.reserve(compound.children.size());

        for (const CompoundShapeChild &child : compound.children)
        {
            CompoundShapeChild scaled{};
            scaled.shape = scale_primitive_uniform(child.shape, s);
            scaled.position = child.position * s;
            scaled.rotation = child.rotation;
            scaled.user_data = child.user_data;
            out.children.push_back(std::move(scaled));
        }

        return out;
    }

    std::optional<CollisionShape> scale_collision_shape_uniform(const CollisionShape &shape, float uniform_scale)
    {
        if (!valid_uniform_scale(uniform_scale))
        {
            return {};
        }

        const float s = std::abs(uniform_scale);

        return std::visit([s](const auto &src) -> std::optional<CollisionShape> {
            using T = std::decay_t<decltype(src)>;

            if constexpr (std::is_same_v<T, BoxShape>)
            {
                return CollisionShape::Box(src.half_extents * s);
            }
            else if constexpr (std::is_same_v<T, SphereShape>)
            {
                return CollisionShape::Sphere(src.radius * s);
            }
            else if constexpr (std::is_same_v<T, CapsuleShape>)
            {
                return CollisionShape::Capsule(src.radius * s, src.half_height * s);
            }
            else if constexpr (std::is_same_v<T, CylinderShape>)
            {
                return CollisionShape::Cylinder(src.radius * s, src.half_height * s);
            }
            else if constexpr (std::is_same_v<T, TaperedCylinderShape>)
            {
                return CollisionShape::TaperedCylinder(src.half_height * s, src.top_radius * s, src.bottom_radius * s);
            }
            else if constexpr (std::is_same_v<T, PlaneShape>)
            {
                return CollisionShape::Plane(src.normal, src.offset * s);
            }
            else if constexpr (std::is_same_v<T, CompoundShape>)
            {
                return CollisionShape::Compound(scale_compound_uniform(src, s));
            }
            else
            {
                return CollisionShape{src};
            }
        }, shape.shape);
    }
} // namespace Physics

