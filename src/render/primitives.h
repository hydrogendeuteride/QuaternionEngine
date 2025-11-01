#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include "core/vk_types.h"

namespace primitives {

inline void buildCube(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices) {
    vertices.clear();
    indices.clear();

    struct Face {
        glm::vec3 normal;
        glm::vec3 v0, v1, v2, v3;
    } faces[6] = {
        { {0,0,1}, { -0.5f,-0.5f, 0.5f}, { 0.5f,-0.5f, 0.5f}, { -0.5f, 0.5f, 0.5f}, { 0.5f, 0.5f, 0.5f} },
        { {0,0,-1},{ -0.5f,-0.5f,-0.5f}, { -0.5f, 0.5f,-0.5f}, { 0.5f,-0.5f,-0.5f}, { 0.5f, 0.5f,-0.5f} },
        { {0,1,0}, { -0.5f, 0.5f, 0.5f}, { 0.5f, 0.5f, 0.5f}, { -0.5f, 0.5f,-0.5f}, { 0.5f, 0.5f,-0.5f} },
        { {0,-1,0},{ -0.5f,-0.5f, 0.5f}, { -0.5f,-0.5f,-0.5f}, { 0.5f,-0.5f, 0.5f}, { 0.5f,-0.5f,-0.5f} },
        { {1,0,0}, { 0.5f,-0.5f, 0.5f}, { 0.5f,-0.5f,-0.5f}, { 0.5f, 0.5f, 0.5f}, { 0.5f, 0.5f,-0.5f} },
        { {-1,0,0},{ -0.5f,-0.5f, 0.5f}, { -0.5f, 0.5f, 0.5f}, { -0.5f,-0.5f,-0.5f}, { -0.5f, 0.5f,-0.5f} }
    };

    for (auto& f : faces) {
        uint32_t start = (uint32_t)vertices.size();
        Vertex v0{f.v0, 0, f.normal, 0, glm::vec4(1.0f), glm::vec4(1,0,0,1)};
        Vertex v1{f.v1, 1, f.normal, 0, glm::vec4(1.0f), glm::vec4(1,0,0,1)};
        Vertex v2{f.v2, 0, f.normal, 1, glm::vec4(1.0f), glm::vec4(1,0,0,1)};
        Vertex v3{f.v3, 1, f.normal, 1, glm::vec4(1.0f), glm::vec4(1,0,0,1)};
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

inline void buildSphere(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, int sectors = 16, int stacks = 16) {
    vertices.clear();
    indices.clear();
    float radius = 0.5f;
    for (int i = 0; i <= stacks; ++i) {
        float v = (float)i / stacks;
        const float phi = v * glm::pi<float>();
        float y = cos(phi);
        float r = sin(phi);
        for (int j = 0; j <= sectors; ++j) {
            float u = (float)j / sectors;
            float theta = u * glm::two_pi<float>();
            float x = r * cos(theta);
            float z = r * sin(theta);
            Vertex vert;
            vert.position = glm::vec3(x, y, z) * radius;
            vert.normal = glm::normalize(glm::vec3(x, y, z));
            vert.uv_x = u;
            vert.uv_y = 1.0f - v;
            vert.color = glm::vec4(1.0f);
            vert.tangent = glm::vec4(1,0,0,1);
            vertices.push_back(vert);
        }
    }
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < sectors; ++j) {
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

} // namespace primitives

