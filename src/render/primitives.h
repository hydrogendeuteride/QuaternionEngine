#pragma once
#include <vector>
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include "core/types.h"
#include "glm/gtx/compatibility.hpp"
#include "glm/gtx/norm.hpp"

namespace primitives
{
    // Axis-aligned unit cube centered at origin, size 1 on each side.
    inline void buildCube(std::vector<Vertex> &vertices, std::vector<uint32_t> &indices)
    {
        vertices.clear();
        indices.clear();

        struct Face
        {
            glm::vec3 normal;
            glm::vec3 v0, v1, v2, v3;
        } faces[6] = {
                    {{0, 0, 1}, {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}, {0.5f, 0.5f, 0.5f}},
                    {
                        {0, 0, -1}, {-0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
                        {0.5f, 0.5f, -0.5f}
                    },
                    {{0, 1, 0}, {-0.5f, 0.5f, 0.5f}, {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, -0.5f}, {0.5f, 0.5f, -0.5f}},
                    {
                        {0, -1, 0}, {-0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, 0.5f},
                        {0.5f, -0.5f, -0.5f}
                    },
                    {{1, 0, 0}, {0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}, {0.5f, 0.5f, -0.5f}},
                    {{-1, 0, 0}, {-0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f}}
                };

        for (auto &f: faces)
        {
            uint32_t start = (uint32_t) vertices.size();
            Vertex v0{f.v0, 0, f.normal, 0, glm::vec4(1.0f), glm::vec4(1, 0, 0, 1)};
            Vertex v1{f.v1, 1, f.normal, 0, glm::vec4(1.0f), glm::vec4(1, 0, 0, 1)};
            Vertex v2{f.v2, 0, f.normal, 1, glm::vec4(1.0f), glm::vec4(1, 0, 0, 1)};
            Vertex v3{f.v3, 1, f.normal, 1, glm::vec4(1.0f), glm::vec4(1, 0, 0, 1)};
            vertices.push_back(v0);
            vertices.push_back(v1);
            vertices.push_back(v2);
            vertices.push_back(v3);
            indices.push_back(start + 0);
            indices.push_back(start + 1);
            indices.push_back(start + 2);
            indices.push_back(start + 2);
            indices.push_back(start + 1);
            indices.push_back(start + 3);
        }
    }

    // Unit sphere centered at origin, radius 0.5.
    inline void buildSphere(std::vector<Vertex> &vertices, std::vector<uint32_t> &indices, int sectors = 16,
                            int stacks = 16)
    {
        vertices.clear();
        indices.clear();
        float radius = 0.5f;
        for (int i = 0; i <= stacks; ++i)
        {
            float v = (float) i / stacks;
            const float phi = v * glm::pi<float>();
            float y = cos(phi);
            float r = sin(phi);
            for (int j = 0; j <= sectors; ++j)
            {
                float u = (float) j / sectors;
                float theta = u * glm::two_pi<float>();
                float x = r * cos(theta);
                float z = r * sin(theta);
                Vertex vert;
                vert.position = glm::vec3(x, y, z) * radius;
                vert.normal = glm::normalize(glm::vec3(x, y, z));
                vert.uv_x = u;
                vert.uv_y = 1.0f - v;
                vert.color = glm::vec4(1.0f);
                vert.tangent = glm::vec4(1, 0, 0, 1);
                vertices.push_back(vert);
            }
        }
        for (int i = 0; i < stacks; ++i)
        {
            for (int j = 0; j < sectors; ++j)
            {
                uint32_t first = i * (sectors + 1) + j;
                uint32_t second = first + sectors + 1;
                indices.push_back(first);
                indices.push_back(second);
                indices.push_back(first + 1);
                indices.push_back(first + 1);
                indices.push_back(second);
                indices.push_back(second + 1);
            }
        }
    }

    // Infinite grid plane trimmed to a unit quad on XZ, centered at origin (Y up).
    inline void buildPlane(std::vector<Vertex> &vertices, std::vector<uint32_t> &indices)
    {
        vertices.clear();
        indices.clear();

        const glm::vec3 n(0.0f, 1.0f, 0.0f);
        const float y = 0.0f;

        Vertex v0{{-0.5f, y, -0.5f}, 0.0f, n, 0.0f, glm::vec4(1.0f), glm::vec4(1, 0, 0, 1)};
        Vertex v1{{ 0.5f, y, -0.5f}, 1.0f, n, 0.0f, glm::vec4(1.0f), glm::vec4(1, 0, 0, 1)};
        Vertex v2{{ 0.5f, y,  0.5f}, 1.0f, n, 1.0f, glm::vec4(1.0f), glm::vec4(1, 0, 0, 1)};
        Vertex v3{{-0.5f, y,  0.5f}, 0.0f, n, 1.0f, glm::vec4(1.0f), glm::vec4(1, 0, 0, 1)};

        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);

        indices.push_back(0);
        indices.push_back(1);
        indices.push_back(2);
        indices.push_back(2);
        indices.push_back(3);
        indices.push_back(0);
    }

    // Capsule aligned with local Y axis.
    // Radius ~0.5, total height ~2.0 (cylinder half-height 0.5).
    inline void buildCapsule(std::vector<Vertex> &vertices, std::vector<uint32_t> &indices,
                             int radialSegments = 16, int stackSegments = 16)
    {
        vertices.clear();
        indices.clear();

        radialSegments = std::max(radialSegments, 3);
        stackSegments = std::max(stackSegments, 2);

        const float radius = 0.5f;
        const float halfHeight = 0.5f; // cylinder half-height
        const float totalHalf = halfHeight + radius;
        const float totalHeight = totalHalf * 2.0f;

        // Build a regular grid in (stackSegments+1) x (radialSegments+1)
        // using an analytical capsule cross-section (capped cylinder).
        for (int iy = 0; iy <= stackSegments; ++iy)
        {
            float v = (float) iy / (float) stackSegments;
            float y = -totalHalf + totalHeight * v;

            float ay = std::abs(y);
            float ringRadius;
            if (ay <= halfHeight)
            {
                ringRadius = radius;
            }
            else
            {
                float dy = ay - halfHeight;
                float inside = radius * radius - dy * dy;
                ringRadius = inside > 0.0f ? std::sqrt(inside) : 0.0f;
            }

            for (int ix = 0; ix <= radialSegments; ++ix)
            {
                float u = (float) ix / (float) radialSegments;
                float theta = u * glm::two_pi<float>();
                float x = ringRadius * std::cos(theta);
                float z = ringRadius * std::sin(theta);

                glm::vec3 pos(x, y, z);

                float cy = glm::clamp(y, -halfHeight, halfHeight);
                glm::vec3 center(0.0f, cy, 0.0f);
                glm::vec3 n = glm::normalize(pos - center);
                if (!glm::all(glm::isfinite(n)) || glm::length2(n) < 1e-8f)
                {
                    n = glm::vec3(0.0f, y >= 0.0f ? 1.0f : -1.0f, 0.0f);
                }

                Vertex vert{};
                vert.position = pos;
                vert.normal = n;
                vert.uv_x = u;
                vert.uv_y = v;
                vert.color = glm::vec4(1.0f);
                vert.tangent = glm::vec4(1, 0, 0, 1);
                vertices.push_back(vert);
            }
        }

        const uint32_t stride = static_cast<uint32_t>(radialSegments + 1);
        for (int iy = 0; iy < stackSegments; ++iy)
        {
            for (int ix = 0; ix < radialSegments; ++ix)
            {
                uint32_t i0 = (uint32_t) iy * stride + (uint32_t) ix;
                uint32_t i1 = i0 + 1;
                uint32_t i2 = i0 + stride;
                uint32_t i3 = i2 + 1;

                indices.push_back(i0);
                indices.push_back(i2);
                indices.push_back(i1);

                indices.push_back(i1);
                indices.push_back(i2);
                indices.push_back(i3);
            }
        }
    }
} // namespace primitives
