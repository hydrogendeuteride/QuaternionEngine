#pragma once

#include "core/world.h"
#include "render/graph/types.h"
#include "render/renderpass.h"

#include <glm/glm.hpp>
#include <vector>

class RenderGraph;

class ParticlePass : public IRenderPass
{
public:
    static constexpr uint32_t k_max_particles = 128u * 1024u;

    enum class BlendMode : uint32_t
    {
        Additive = 0,
        Alpha = 1,
    };

    struct Params
    {
        glm::vec3 emitter_pos_local{0.0f, 0.0f, 0.0f};
        float spawn_radius{0.1f};

        glm::vec3 emitter_dir_local{0.0f, 1.0f, 0.0f};
        float cone_angle_degrees{20.0f};

        float min_speed{2.0f};
        float max_speed{8.0f};

        float min_life{0.5f};
        float max_life{1.5f};

        float min_size{0.05f};
        float max_size{0.15f};

        float drag{1.0f};
        float gravity{0.0f}; // positive pulls down -Y in local space

        glm::vec4 color{1.0f, 0.5f, 0.1f, 1.0f};
    };

    struct System
    {
        uint32_t id{0};
        uint32_t base{0};
        uint32_t count{0};
        bool enabled{true};
        bool reset{true};
        BlendMode blend{BlendMode::Additive};
        Params params{};
    };

    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;
    const char *getName() const override { return "Particles"; }

    void register_graph(RenderGraph *graph, RGImageHandle hdrTarget, RGImageHandle depthHandle);

    uint32_t create_system(uint32_t count);
    bool destroy_system(uint32_t id);
    bool resize_system(uint32_t id, uint32_t new_count);

    std::vector<System> &systems() { return _systems; }
    const std::vector<System> &systems() const { return _systems; }

    uint32_t allocated_particles() const;
    uint32_t free_particles() const;

private:
    struct FreeRange
    {
        uint32_t base{0};
        uint32_t count{0};
    };

    bool allocate_range(uint32_t count, uint32_t &out_base);
    void free_range(uint32_t base, uint32_t count);
    void merge_free_ranges();

    System *find_system(uint32_t id);

    EngineContext *_context = nullptr;

    AllocatedBuffer _particle_pool{};
    VkDeviceSize _particle_pool_size = 0;

    VkDescriptorSetLayout _particle_set_layout = VK_NULL_HANDLE;

    uint32_t _next_system_id = 1;
    std::vector<System> _systems;
    std::vector<FreeRange> _free_ranges;

    float _time_sec = 0.0f;
    float _dt_sec = 0.0f;
    glm::vec3 _origin_delta_local{0.0f};

    WorldVec3 _prev_origin_world{0.0, 0.0, 0.0};
    bool _has_prev_origin = false;
};

