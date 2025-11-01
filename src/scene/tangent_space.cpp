#include "tangent_space.h"

#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
#include <algorithm>

#include "glm/gtx/norm.hpp"

namespace
{
    struct Range
    {
        size_t indexStart;
        size_t indexCount;
        size_t vertexStart;
        size_t vertexCount;
    };

    static inline glm::vec3 orthonormal_tangent(const glm::vec3 &n)
    {
        // Build tangent orthogonal to n from an arbitrary axis
        glm::vec3 a = (std::abs(n.z) < 0.999f) ? glm::vec3(0, 0, 1) : glm::vec3(0, 1, 0);
        glm::vec3 t = glm::normalize(glm::cross(a, n));
        return t;
    }

    static void generate_fallback(std::vector<Vertex> &vtx, const Range &r)
    {
        for (size_t i = 0; i < r.vertexCount; ++i)
        {
            Vertex &v = vtx[r.vertexStart + i];
            glm::vec3 T = orthonormal_tangent(glm::normalize(v.normal));
            v.tangent = glm::vec4(T, 1.0f);
        }
    }
} // namespace

namespace geom
{
#ifdef MIKKTS_ENABLE
#include <mikktspace.h>

    struct MikkAdapter
    {
        std::vector<Vertex> *verts;
        const std::vector<uint32_t> *inds;
        Range range;
    };

    static int mikk_get_num_faces(const SMikkTSpaceContext *ctx)
    {
        const MikkAdapter *ad = reinterpret_cast<const MikkAdapter *>(ctx->m_pUserData);
        return static_cast<int>(ad->range.indexCount / 3);
    }

    static int mikk_get_num_verts_of_face(const SMikkTSpaceContext *, const int /*face*/) { return 3; }

    static void mikk_get_position(const SMikkTSpaceContext *ctx, float outpos[], const int face, const int vert)
    {
        const MikkAdapter *ad = reinterpret_cast<const MikkAdapter *>(ctx->m_pUserData);
        uint32_t idx = ad->inds->at(ad->range.indexStart + face * 3 + vert);
        const Vertex &v = ad->verts->at(idx);
        outpos[0] = v.position.x;
        outpos[1] = v.position.y;
        outpos[2] = v.position.z;
    }

    static void mikk_get_normal(const SMikkTSpaceContext *ctx, float outnormal[], const int face, const int vert)
    {
        const MikkAdapter *ad = reinterpret_cast<const MikkAdapter *>(ctx->m_pUserData);
        uint32_t idx = ad->inds->at(ad->range.indexStart + face * 3 + vert);
        const Vertex &v = ad->verts->at(idx);
        outnormal[0] = v.normal.x;
        outnormal[1] = v.normal.y;
        outnormal[2] = v.normal.z;
    }

    static void mikk_get_texcoord(const SMikkTSpaceContext *ctx, float outuv[], const int face, const int vert)
    {
        const MikkAdapter *ad = reinterpret_cast<const MikkAdapter *>(ctx->m_pUserData);
        uint32_t idx = ad->inds->at(ad->range.indexStart + face * 3 + vert);
        const Vertex &v = ad->verts->at(idx);
        outuv[0] = v.uv_x;
        outuv[1] = v.uv_y;
    }

    static void mikk_set_tspace_basic(const SMikkTSpaceContext *ctx, const float tangent[], const float sign,
                                      const int face, const int vert)
    {
        const MikkAdapter *ad = reinterpret_cast<const MikkAdapter *>(ctx->m_pUserData);
        uint32_t idx = ad->inds->at(ad->range.indexStart + face * 3 + vert);
        Vertex &v = ad->verts->at(idx);
        v.tangent = glm::vec4(tangent[0], tangent[1], tangent[2], sign);
    }

    static bool generate_mikk(std::vector<Vertex> &vertices, const std::vector<uint32_t> &indices, const Range &r)
    {
        SMikkTSpaceInterface iface{};
        iface.m_getNumFaces = mikk_get_num_faces;
        iface.m_getNumVerticesOfFace = mikk_get_num_verts_of_face;
        iface.m_getPosition = mikk_get_position;
        iface.m_getNormal = mikk_get_normal;
        iface.m_getTexCoord = mikk_get_texcoord;
        iface.m_setTSpaceBasic = mikk_set_tspace_basic;

        MikkAdapter ad{&vertices, &indices, r};
        SMikkTSpaceContext ctx{};
        ctx.m_pInterface = &iface;
        ctx.m_pUserData = &ad;

        // angle weighting, respect vtx-deriv continuity by default
        return genTangSpaceDefault(&ctx) != 0;
    }
#endif // MIKKTS_ENABLE

    void generate_tangents_range(std::vector<Vertex> &vertices, const std::vector<uint32_t> &indices,
                                 size_t indexStart, size_t indexCount,
                                 size_t vertexStart, size_t vertexCount)
    {
        Range r{indexStart, indexCount, vertexStart, vertexCount};

        if (indexCount < 3 || vertexCount == 0)
        {
            generate_fallback(vertices, r);
            return;
        }

#ifdef MIKKTS_ENABLE
        if (generate_mikk(vertices, indices, r))
        {
            return;
        }
#endif

        std::vector<glm::vec3> tan1(vertexCount, glm::vec3(0.0f));
        std::vector<glm::vec3> bit1(vertexCount, glm::vec3(0.0f));

        bool anyValid = false;
        for (size_t it = 0; it + 2 < indexCount; it += 3)
        {
            uint32_t i0 = indices[r.indexStart + it];
            uint32_t i1 = indices[r.indexStart + it + 1];
            uint32_t i2 = indices[r.indexStart + it + 2];

            // guard against out of range
            if (i0 < r.vertexStart || i1 < r.vertexStart || i2 < r.vertexStart) continue;
            if (i0 >= r.vertexStart + r.vertexCount || i1 >= r.vertexStart + r.vertexCount || i2 >= r.vertexStart + r.
                vertexCount) continue;

            const Vertex &v0 = vertices[i0];
            const Vertex &v1 = vertices[i1];
            const Vertex &v2 = vertices[i2];

            glm::vec3 p0 = v0.position;
            glm::vec3 p1 = v1.position;
            glm::vec3 p2 = v2.position;

            glm::vec2 w0{v0.uv_x, v0.uv_y};
            glm::vec2 w1{v1.uv_x, v1.uv_y};
            glm::vec2 w2{v2.uv_x, v2.uv_y};

            glm::vec3 e1 = p1 - p0;
            glm::vec3 e2 = p2 - p0;
            glm::vec2 d1 = w1 - w0;
            glm::vec2 d2 = w2 - w0;

            float denom = d1.x * d2.y - d1.y * d2.x;
            if (std::abs(denom) < 1e-8f)
            {
                continue; // degenerate UV mapping; skip this tri
            }
            anyValid = true;
            float rcp = 1.0f / denom;
            glm::vec3 t = (e1 * d2.y - e2 * d1.y) * rcp;
            glm::vec3 b = (-e1 * d2.x + e2 * d1.x) * rcp;

            size_t l0 = i0 - r.vertexStart;
            size_t l1 = i1 - r.vertexStart;
            size_t l2 = i2 - r.vertexStart;
            tan1[l0] += t;
            tan1[l1] += t;
            tan1[l2] += t;
            bit1[l0] += b;
            bit1[l1] += b;
            bit1[l2] += b;
        }

        if (!anyValid)
        {
            generate_fallback(vertices, r);
            return;
        }

        for (size_t i = 0; i < r.vertexCount; ++i)
        {
            Vertex &v = vertices[r.vertexStart + i];
            glm::vec3 N = glm::normalize(v.normal);
            glm::vec3 T = tan1[i];
            glm::vec3 B = bit1[i];
            if (glm::length2(T) < 1e-16f)
            {
                T = orthonormal_tangent(N);
                v.tangent = glm::vec4(T, 1.0f);
                continue;
            }
            // Gram-Schmidt orthonormalize
            T = glm::normalize(T - N * glm::dot(N, T));
            // Compute handedness
            float w = (glm::dot(glm::cross(N, T), B) < 0.0f) ? -1.0f : 1.0f;
            v.tangent = glm::vec4(T, w);
        }
    }

    void generate_tangents(std::vector<Vertex> &vertices, const std::vector<uint32_t> &indices)
    {
        if (vertices.empty() || indices.size() < 3) return;
        generate_tangents_range(vertices, indices, 0, indices.size(), 0, vertices.size());
    }
} // namespace geom
