#pragma once

#include <glm/vec3.hpp>
#include <variant>

namespace Physics
{
    // ============================================================================
    // Shape definitions
    // ============================================================================

    struct BoxShape
    {
        glm::vec3 half_extents{0.5f};

        BoxShape() = default;

        explicit BoxShape(float uniform) : half_extents(uniform)
        {
        }

        BoxShape(float hx, float hy, float hz) : half_extents(hx, hy, hz)
        {
        }

        explicit BoxShape(const glm::vec3 &he) : half_extents(he)
        {
        }
    };

    struct SphereShape
    {
        float radius{0.5f};

        SphereShape() = default;

        explicit SphereShape(float r) : radius(r)
        {
        }
    };

    struct CapsuleShape
    {
        float radius{0.5f};
        float half_height{0.5f}; // Half of the cylinder part height

        CapsuleShape() = default;

        CapsuleShape(float r, float hh) : radius(r), half_height(hh)
        {
        }
    };

    struct CylinderShape
    {
        float radius{0.5f};
        float half_height{0.5f};

        CylinderShape() = default;

        CylinderShape(float r, float hh) : radius(r), half_height(hh)
        {
        }
    };

    struct PlaneShape
    {
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
        float offset{0.0f};

        PlaneShape() = default;

        PlaneShape(const glm::vec3 &n, float o = 0.0f) : normal(n), offset(o)
        {
        }
    };

    // ============================================================================
    // Unified shape type
    // ============================================================================

    using ShapeVariant = std::variant<BoxShape, SphereShape, CapsuleShape, CylinderShape, PlaneShape>;

    struct CollisionShape
    {
        ShapeVariant shape;

        CollisionShape() : shape(BoxShape{})
        {
        }

        CollisionShape(const ShapeVariant &s) : shape(s)
        {
        }

        // Convenience factory methods
        static CollisionShape Box(float hx, float hy, float hz) { return CollisionShape{BoxShape{hx, hy, hz}}; }
        static CollisionShape Box(const glm::vec3 &half_extents) { return CollisionShape{BoxShape{half_extents}}; }
        static CollisionShape Cube(float half_size) { return CollisionShape{BoxShape{half_size}}; }

        static CollisionShape Sphere(float radius) { return CollisionShape{SphereShape{radius}}; }

        static CollisionShape Capsule(float radius, float half_height)
        {
            return CollisionShape{CapsuleShape{radius, half_height}};
        }

        static CollisionShape Cylinder(float radius, float half_height)
        {
            return CollisionShape{CylinderShape{radius, half_height}};
        }

        static CollisionShape Plane(const glm::vec3 &normal = {0, 1, 0}, float offset = 0.0f)
        {
            return CollisionShape{PlaneShape{normal, offset}};
        }

        // Type queries
        bool is_box() const { return std::holds_alternative<BoxShape>(shape); }
        bool is_sphere() const { return std::holds_alternative<SphereShape>(shape); }
        bool is_capsule() const { return std::holds_alternative<CapsuleShape>(shape); }
        bool is_cylinder() const { return std::holds_alternative<CylinderShape>(shape); }
        bool is_plane() const { return std::holds_alternative<PlaneShape>(shape); }

        // Accessors (returns nullptr if wrong type)
        const BoxShape *as_box() const { return std::get_if<BoxShape>(&shape); }
        const SphereShape *as_sphere() const { return std::get_if<SphereShape>(&shape); }
        const CapsuleShape *as_capsule() const { return std::get_if<CapsuleShape>(&shape); }
        const CylinderShape *as_cylinder() const { return std::get_if<CylinderShape>(&shape); }
        const PlaneShape *as_plane() const { return std::get_if<PlaneShape>(&shape); }
    };
} // namespace Physics
