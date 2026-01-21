#include "gltf_collider_parser.h"
#include "scene/vk_loader.h"
#include "core/engine.h"
#include "core/types.h"

#include <fmt/core.h>
#include <cctype>
#include <cmath>
#include <optional>

namespace Physics
{
    namespace
    {
        bool starts_with_icase(std::string_view s, std::string_view prefix)
        {
            if (s.size() < prefix.size()) return false;
            for (size_t i = 0; i < prefix.size(); ++i)
            {
                const unsigned char a = static_cast<unsigned char>(s[i]);
                const unsigned char b = static_cast<unsigned char>(prefix[i]);
                if (std::toupper(a) != std::toupper(b)) return false;
            }
            return true;
        }

        bool approx_equal_rel(float a, float b, float eps)
        {
            const float aa = std::abs(a);
            const float bb = std::abs(b);
            const float denom = std::max(1.0f, std::max(aa, bb));
            return std::abs(a - b) <= eps * denom;
        }

        std::optional<PrimitiveShapeVariant> make_collider_shape_from_scale(
            GltfColliderMarkerType type,
            const glm::vec3& scale,
            std::string_view scene_name,
            std::string_view node_name)
        {
            const float sx = std::abs(scale.x);
            const float sy = std::abs(scale.y);
            const float sz = std::abs(scale.z);

            auto finite_nonnegative = [](float v) { return std::isfinite(v) && v >= 0.0f; };
            if (!finite_nonnegative(sx) || !finite_nonnegative(sy) || !finite_nonnegative(sz))
            {
                fmt::println("[GLTF][Colliders] '{}' node '{}' has non-finite scale; skipping",
                             scene_name, node_name);
                return {};
            }

            constexpr float kUniformEps = 1.0e-3f;

            switch (type)
            {
                case GltfColliderMarkerType::Box:
                {
                    const glm::vec3 he = glm::vec3(sx, sy, sz) * 0.5f;
                    if (he.x <= 0.0f || he.y <= 0.0f || he.z <= 0.0f)
                    {
                        fmt::println("[GLTF][Colliders] '{}' node '{}' box has zero size; skipping",
                                     scene_name, node_name);
                        return {};
                    }
                    return BoxShape{he};
                }
                case GltfColliderMarkerType::Sphere:
                {
                    if (!approx_equal_rel(sx, sy, kUniformEps) || !approx_equal_rel(sx, sz, kUniformEps))
                    {
                        fmt::println("[GLTF][Colliders] '{}' node '{}' sphere scale is non-uniform (x={}, y={}, z={}); using max as radius",
                                     scene_name, node_name, sx, sy, sz);
                    }
                    const float r = 0.5f * std::max({sx, sy, sz});
                    if (r <= 0.0f)
                    {
                        fmt::println("[GLTF][Colliders] '{}' node '{}' sphere has zero radius; skipping",
                                     scene_name, node_name);
                        return {};
                    }
                    return SphereShape{r};
                }
                case GltfColliderMarkerType::Capsule:
                {
                    if (!approx_equal_rel(sx, sz, kUniformEps))
                    {
                        fmt::println("[GLTF][Colliders] '{}' node '{}' capsule scale X/Z is non-uniform (x={}, z={}); using max as radius",
                                     scene_name, node_name, sx, sz);
                    }
                    const float radius = 0.5f * std::max(sx, sz);
                    const float half_height = std::max(0.0f, 0.5f * sy - radius);
                    if (radius <= 0.0f)
                    {
                        fmt::println("[GLTF][Colliders] '{}' node '{}' capsule has zero radius; skipping",
                                     scene_name, node_name);
                        return {};
                    }
                    return CapsuleShape{radius, half_height};
                }
                case GltfColliderMarkerType::Cylinder:
                {
                    if (!approx_equal_rel(sx, sz, kUniformEps))
                    {
                        fmt::println("[GLTF][Colliders] '{}' node '{}' cylinder scale X/Z is non-uniform (x={}, z={}); using max as radius",
                                     scene_name, node_name, sx, sz);
                    }
                    const float radius = 0.5f * std::max(sx, sz);
                    const float half_height = 0.5f * sy;
                    if (radius <= 0.0f || half_height <= 0.0f)
                    {
                        fmt::println("[GLTF][Colliders] '{}' node '{}' cylinder has zero size; skipping",
                                     scene_name, node_name);
                        return {};
                    }
                    return CylinderShape{radius, half_height};
                }
                case GltfColliderMarkerType::TaperedCylinder:
                {
                    const float half_height = 0.5f * sy;
                    const float top_radius = 0.5f * sx;
                    const float bottom_radius = 0.5f * sz;
                    if (half_height <= 0.0f || (top_radius <= 0.0f && bottom_radius <= 0.0f))
                    {
                        fmt::println("[GLTF][Colliders] '{}' node '{}' tapered cylinder has zero size; skipping",
                                     scene_name, node_name);
                        return {};
                    }
                    return TaperedCylinderShape{half_height, top_radius, bottom_radius};
                }
                default:
                    break;
            }

            return {};
        }

        template <typename OwnerPredicate>
        void build_compounds_from_scene_markers_impl(
            std::unordered_map<std::string, CompoundShape>& out,
            const LoadedGLTF& scene,
            const OwnerPredicate& resolve_owner)
        {
            std::unordered_map<const Node*, std::string_view> name_by_ptr;
            name_by_ptr.reserve(scene.nodes.size());
            for (const auto& [name, node_ptr] : scene.nodes)
            {
                if (!node_ptr) continue;
                name_by_ptr[node_ptr.get()] = name;
            }

            const std::string_view scene_name = scene.debugName.empty() ? "<unnamed>" : std::string_view(scene.debugName);

            for (const auto& [node_name, node_ptr] : scene.nodes)
            {
                if (!node_ptr) continue;

                const GltfColliderMarkerType type = parse_collider_marker_type(node_name);
                if (type == GltfColliderMarkerType::Unknown)
                {
                    continue;
                }

                const Node* owner = resolve_owner(node_ptr.get(), name_by_ptr);
                if (!owner)
                {
                    fmt::println("[GLTF][Colliders] '{}' collider node '{}' has no valid owner; skipping",
                                 scene_name, node_name);
                    continue;
                }

                auto owner_name_it = name_by_ptr.find(owner);
                if (owner_name_it == name_by_ptr.end())
                {
                    fmt::println("[GLTF][Colliders] '{}' collider node '{}' owner missing name mapping; skipping",
                                 scene_name, node_name);
                    continue;
                }
                const std::string owner_name(owner_name_it->second);

                const glm::mat4 rel = glm::inverse(owner->worldTransform) * node_ptr->worldTransform;
                glm::vec3 t{0.0f};
                glm::quat r{1.0f, 0.0f, 0.0f, 0.0f};
                glm::vec3 s{1.0f};
                decompose_trs_matrix(rel, t, r, s);
                r = glm::normalize(r);

                std::optional<PrimitiveShapeVariant> shape =
                    make_collider_shape_from_scale(type, s, scene_name, node_name);
                if (!shape.has_value())
                {
                    continue;
                }

                out[owner_name].add_child(*shape, t, r, 0);
            }
        }

    } // anonymous namespace

    GltfColliderMarkerType parse_collider_marker_type(std::string_view node_name)
    {
        if (starts_with_icase(node_name, "COL_BOX")) return GltfColliderMarkerType::Box;
        if (starts_with_icase(node_name, "COL_SPHERE")) return GltfColliderMarkerType::Sphere;
        if (starts_with_icase(node_name, "COL_CAPSULE")) return GltfColliderMarkerType::Capsule;
        if (starts_with_icase(node_name, "COL_CYLINDER")) return GltfColliderMarkerType::Cylinder;
        if (starts_with_icase(node_name, "COL_TAPERED")) return GltfColliderMarkerType::TaperedCylinder;
        return GltfColliderMarkerType::Unknown;
    }

    void build_colliders_from_markers(
        std::unordered_map<std::string, CompoundShape>& out_compounds,
        const LoadedGLTF& scene,
        bool clear_existing)
    {
        if (clear_existing)
        {
            out_compounds.clear();
        }

        auto resolve_owner = [](const Node* collider_node,
                                const std::unordered_map<const Node*, std::string_view>& name_by_ptr) -> const Node* {
            const Node* best_owner = nullptr;
            auto p = collider_node ? collider_node->parent.lock() : std::shared_ptr<Node>{};
            while (p)
            {
                const Node* cur = p.get();
                auto it = name_by_ptr.find(cur);
                if (it == name_by_ptr.end())
                {
                    break;
                }

                const std::string_view cur_name = it->second;
                if (parse_collider_marker_type(cur_name) != GltfColliderMarkerType::Unknown)
                {
                    p = p->parent.lock();
                    continue;
                }

                if (!best_owner)
                {
                    best_owner = cur;
                }
                if (dynamic_cast<const MeshNode*>(cur) != nullptr)
                {
                    return cur; // prefer closest mesh node
                }

                p = p->parent.lock();
            }

            return best_owner;
        };

        build_compounds_from_scene_markers_impl(out_compounds, scene, resolve_owner);
    }

    void build_colliders_from_sidecar(
        std::unordered_map<std::string, CompoundShape>& out_compounds,
        const LoadedGLTF& sidecar_scene,
        const std::unordered_set<std::string_view>& dst_node_names,
        bool clear_existing)
    {
        if (clear_existing)
        {
            out_compounds.clear();
        }

        auto resolve_owner = [&dst_node_names](const Node* collider_node,
                                               const std::unordered_map<const Node*, std::string_view>& name_by_ptr) -> const Node* {
            auto p = collider_node ? collider_node->parent.lock() : std::shared_ptr<Node>{};
            while (p)
            {
                const Node* cur = p.get();
                auto it = name_by_ptr.find(cur);
                if (it == name_by_ptr.end())
                {
                    break;
                }

                const std::string_view cur_name = it->second;
                if (parse_collider_marker_type(cur_name) != GltfColliderMarkerType::Unknown)
                {
                    p = p->parent.lock();
                    continue;
                }
                if (dst_node_names.contains(cur_name))
                {
                    return cur;
                }
                p = p->parent.lock();
            }
            return nullptr;
        };

        build_compounds_from_scene_markers_impl(out_compounds, sidecar_scene, resolve_owner);
    }

} // namespace Physics
