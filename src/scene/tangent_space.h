#pragma once

#include <vector>
#include <cstddef>
#include <core/types.h>

namespace geom {

// Generate per-vertex tangents with a robust fallback when MikkTSpace is not available.
// - Fills Vertex.tangent (xyz = tangent, w = handedness sign for B = sign * cross(N, T))
// - Expects valid normals and UVs; if UVs are degenerate, builds an arbitrary orthonormal basis.
void generate_tangents(std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices);

// Range variant for submeshes (indices [indexStart, indexStart+indexCount), vertices [vertexStart, vertexStart+vertexCount))
void generate_tangents_range(std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices,
                             size_t indexStart, size_t indexCount,
                             size_t vertexStart, size_t vertexCount);

} // namespace geom

