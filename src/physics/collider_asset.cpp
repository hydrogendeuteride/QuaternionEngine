#include "collider_asset.h"

#include <cmath>
#include <numbers>

namespace Physics
{
    namespace
    {
        struct PrimitiveMassInfo
        {
            double mass{0.0};
            glm::dvec3 center_of_mass{0.0, 0.0, 0.0};
        };

        static bool valid_uniform_scale(float s)
        {
            return std::isfinite(s) && s > 0.0f;
        }

        static PrimitiveMassInfo primitive_mass_info(const PrimitiveShapeVariant &shape)
        {
            using std::numbers::pi_v;

            return std::visit([](const auto &prim) -> PrimitiveMassInfo {
                using T = std::decay_t<decltype(prim)>;

                if constexpr (std::is_same_v<T, BoxShape>)
                {
                    return PrimitiveMassInfo{
                        8.0 * static_cast<double>(prim.half_extents.x) *
                            static_cast<double>(prim.half_extents.y) *
                            static_cast<double>(prim.half_extents.z),
                        glm::dvec3(0.0)};
                }
                else if constexpr (std::is_same_v<T, SphereShape>)
                {
                    const double r = static_cast<double>(prim.radius);
                    return PrimitiveMassInfo{
                        (4.0 / 3.0) * pi_v<double> * r * r * r,
                        glm::dvec3(0.0)};
                }
                else if constexpr (std::is_same_v<T, CapsuleShape>)
                {
                    const double r = static_cast<double>(prim.radius);
                    const double hh = static_cast<double>(prim.half_height);
                    return PrimitiveMassInfo{
                        2.0 * pi_v<double> * r * r * hh +
                            (4.0 / 3.0) * pi_v<double> * r * r * r,
                        glm::dvec3(0.0)};
                }
                else if constexpr (std::is_same_v<T, CylinderShape>)
                {
                    const double r = static_cast<double>(prim.radius);
                    const double hh = static_cast<double>(prim.half_height);
                    return PrimitiveMassInfo{
                        2.0 * pi_v<double> * r * r * hh,
                        glm::dvec3(0.0)};
                }
                else if constexpr (std::is_same_v<T, TaperedCylinderShape>)
                {
                    const double hh = static_cast<double>(prim.half_height);
                    const double rt = static_cast<double>(prim.top_radius);
                    const double rb = static_cast<double>(prim.bottom_radius);
                    const double sum = rb * rb + rb * rt + rt * rt;
                    if (!(hh > 0.0) || !(sum > 0.0))
                    {
                        return PrimitiveMassInfo{};
                    }

                    const double volume = (2.0 * pi_v<double> * hh / 3.0) * sum;
                    const double y = hh * (rt * rt - rb * rb) / (2.0 * sum);
                    return PrimitiveMassInfo{volume, glm::dvec3(0.0, y, 0.0)};
                }
                else
                {
                    return PrimitiveMassInfo{};
                }
            }, shape);
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
            scaled.mass = child.mass;
            scaled.name = child.name;
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
            else if constexpr (std::is_same_v<T, TriangleMeshShape>)
            {
                TriangleMeshShape out = src;
                out.scale *= s;
                return CollisionShape{out};
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

    glm::vec3 compound_center_of_mass(const CompoundShape &compound)
    {
        double total_mass = 0.0;
        glm::dvec3 weighted_center{0.0, 0.0, 0.0};

        for (const CompoundShapeChild &child : compound.children)
        {
            const PrimitiveMassInfo info = primitive_mass_info(child.shape);
            const double mass =
                (std::isfinite(child.mass) && child.mass > 0.0f) ? static_cast<double>(child.mass) : info.mass;
            if (!(std::isfinite(mass) && mass > 0.0))
            {
                continue;
            }

            const float qlen2 = glm::dot(child.rotation, child.rotation);
            const glm::quat rotation =
                qlen2 > 1.0e-8f ? glm::normalize(child.rotation) : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            const glm::dvec3 child_center =
                glm::dvec3(child.position) + glm::dvec3(rotation * glm::vec3(info.center_of_mass));

            total_mass += mass;
            weighted_center += child_center * mass;
        }

        if (!(std::isfinite(total_mass) && total_mass > 0.0))
        {
            return glm::vec3(0.0f);
        }

        return glm::vec3(weighted_center / total_mass);
    }
} // namespace Physics
