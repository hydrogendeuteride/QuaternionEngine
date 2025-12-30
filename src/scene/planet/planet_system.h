#pragma once

#include <core/world.h>
#include <core/descriptor/descriptors.h>
#include <scene/planet/planet_quadtree.h>

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class EngineContext;
class SceneManager;
struct DrawContext;
class MeshAsset;
struct GLTFMaterial;

class PlanetSystem
{
public:
    enum class BodyID : uint8_t
    {
        Earth = 0,
        Moon = 1,
    };

    struct EarthDebugStats
    {
        planet::PlanetQuadtree::Stats quadtree{};
        uint32_t visible_patches = 0;
        uint32_t created_patches = 0;
        uint32_t patch_cache_size = 0;
        uint32_t estimated_triangles = 0;
        float ms_quadtree = 0.0f;
        float ms_patch_create = 0.0f;
        float ms_emit = 0.0f;
        float ms_total = 0.0f;
    };

    struct PlanetBody
    {
        std::string name;
        WorldVec3 center_world{0.0, 0.0, 0.0};
        double radius_m = 1.0;
        bool visible = true;

        std::shared_ptr<MeshAsset> mesh;
        std::shared_ptr<GLTFMaterial> material;
    };

    void init(EngineContext *context);
    void cleanup();

    void update_and_emit(const SceneManager &scene, DrawContext &draw_context);

    bool enabled() const { return _enabled; }
    void set_enabled(bool enabled) { _enabled = enabled; }

    const PlanetBody *get_body(BodyID id) const;
    PlanetBody *get_body(BodyID id);
    const std::vector<PlanetBody> &bodies() const { return _bodies; }

    const planet::PlanetQuadtree::Settings &earth_quadtree_settings() const { return _earth_quadtree_settings; }
    void set_earth_quadtree_settings(const planet::PlanetQuadtree::Settings &settings) { _earth_quadtree_settings = settings; }
    const EarthDebugStats &earth_debug_stats() const { return _earth_debug_stats; }

    uint32_t earth_patch_create_budget_per_frame() const { return _earth_patch_create_budget_per_frame; }
    void set_earth_patch_create_budget_per_frame(uint32_t budget) { _earth_patch_create_budget_per_frame = budget; }

    float earth_patch_create_budget_ms() const { return _earth_patch_create_budget_ms; }
    void set_earth_patch_create_budget_ms(float budget_ms) { _earth_patch_create_budget_ms = budget_ms; }

    uint32_t earth_patch_cache_max() const { return _earth_patch_cache_max; }
    void set_earth_patch_cache_max(uint32_t max_patches) { _earth_patch_cache_max = max_patches; }

private:
    enum class EarthPatchState : uint8_t
    {
        Allocating = 0,
        Ready = 1,
    };

    struct EarthPatch
    {
        planet::PatchKey key{};
        EarthPatchState state = EarthPatchState::Allocating;

        AllocatedBuffer vertex_buffer{};
        VkDeviceAddress vertex_buffer_address = 0;

        MaterialInstance material_instance{};

        glm::vec3 bounds_origin{0.0f};
        glm::vec3 bounds_extents{0.5f};
        float bounds_sphere_radius = 0.5f;

        WorldVec3 patch_center_dir{0.0, 0.0, 1.0};
        uint32_t last_used_frame = 0;
        std::list<uint32_t>::iterator lru_it{};
    };

    void ensure_bodies_created();
    EarthPatch *find_earth_patch(const planet::PatchKey &key);
    EarthPatch *get_or_create_earth_patch(const PlanetBody &earth,
                                          const planet::PatchKey &key,
                                          uint32_t frame_index);
    void ensure_earth_patch_index_buffer();
    void ensure_earth_patch_material_layout();
    void ensure_earth_patch_material_constants_buffer();
    void ensure_earth_patch_material_instance(EarthPatch &patch, const PlanetBody &earth);
    void trim_earth_patch_cache();

    EngineContext *_context = nullptr;
    bool _enabled = true;
    std::vector<PlanetBody> _bodies;

    // Earth cube-sphere quadtree
    planet::PlanetQuadtree _earth_quadtree{};
    planet::PlanetQuadtree::Settings _earth_quadtree_settings{};
    EarthDebugStats _earth_debug_stats{};
    std::unordered_map<planet::PatchKey, uint32_t, planet::PatchKeyHash> _earth_patch_lookup;
    std::vector<EarthPatch> _earth_patches;
    std::vector<uint32_t> _earth_patch_free;
    std::list<uint32_t> _earth_patch_lru;
    AllocatedBuffer _earth_patch_index_buffer{};
    uint32_t _earth_patch_index_count = 0;
    uint32_t _earth_patch_index_resolution = 0;

    VkDescriptorSetLayout _earth_patch_material_layout = VK_NULL_HANDLE;
    DescriptorAllocatorGrowable _earth_patch_material_allocator{};
    bool _earth_patch_material_allocator_initialized = false;
    AllocatedBuffer _earth_patch_material_constants_buffer{};

    uint32_t _earth_patch_frame_stamp = 0;
    uint32_t _earth_patch_resolution = 33;
    uint32_t _earth_patch_create_budget_per_frame = 16;
    float _earth_patch_create_budget_ms = 2.0f;
    uint32_t _earth_patch_cache_max = 2048;
};
