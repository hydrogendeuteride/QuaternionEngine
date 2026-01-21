#pragma once

#include "collision_shape.h"

#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <memory>

struct Node;
struct LoadedGLTF;

namespace Physics
{
    // Collider marker type parsed from glTF node names (e.g., "COL_BOX", "COL_SPHERE").
    enum class GltfColliderMarkerType : uint8_t
    {
        Box,
        Sphere,
        Capsule,
        Cylinder,
        TaperedCylinder,
        Unknown,
    };

    // Parse collider marker type from a node name (case-insensitive prefix match).
    GltfColliderMarkerType parse_collider_marker_type(std::string_view node_name);

    // Build compound shapes from COL_* marker nodes within a glTF scene.
    // Colliders are grouped by their owner node (nearest non-collider ancestor, preferring mesh nodes).
    // Results are stored in out_compounds keyed by the owner node's stable name.
    void build_colliders_from_markers(
        std::unordered_map<std::string, CompoundShape>& out_compounds,
        const LoadedGLTF& scene,
        bool clear_existing = true);

    // Build compound shapes from a separate collider-only glTF sidecar.
    // Colliders are mapped to dst_scene's nodes by matching ancestor node names.
    // dst_node_names provides the set of valid node names in the destination scene.
    void build_colliders_from_sidecar(
        std::unordered_map<std::string, CompoundShape>& out_compounds,
        const LoadedGLTF& sidecar_scene,
        const std::unordered_set<std::string_view>& dst_node_names,
        bool clear_existing = true);

} // namespace Physics
