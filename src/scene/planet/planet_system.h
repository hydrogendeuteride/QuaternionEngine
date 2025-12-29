#pragma once

#include <core/world.h>
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
    struct EarthPatchCacheEntry
    {
        std::shared_ptr<MeshAsset> mesh;
        WorldVec3 patch_center_dir{0.0, 0.0, 1.0};
        uint32_t last_used_frame = 0;
        std::list<planet::PatchKey>::iterator lru_it;
    };

    void ensure_bodies_created();
    std::shared_ptr<MeshAsset> get_or_create_earth_patch_mesh(const PlanetBody &earth, const planet::PatchKey &key);
    void trim_earth_patch_cache();

    EngineContext *_context = nullptr;
    bool _enabled = true;
    std::vector<PlanetBody> _bodies;

    // Earth cube-sphere quadtree (Milestone B4).
    planet::PlanetQuadtree _earth_quadtree{};
    planet::PlanetQuadtree::Settings _earth_quadtree_settings{};
    EarthDebugStats _earth_debug_stats{};
    std::unordered_map<planet::PatchKey, EarthPatchCacheEntry, planet::PatchKeyHash> _earth_patch_cache;
    std::list<planet::PatchKey> _earth_patch_lru;
    uint32_t _earth_patch_resolution = 33;
    uint32_t _earth_patch_create_budget_per_frame = 16;
    float _earth_patch_create_budget_ms = 2.0f;
    uint32_t _earth_patch_cache_max = 2048;
};
