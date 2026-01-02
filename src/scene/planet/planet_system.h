#pragma once

#include <core/world.h>
#include <core/descriptor/descriptors.h>
#include <scene/planet/planet_quadtree.h>

#include <cstdint>
#include <array>
#include <list>
#include <memory>
#include <string>
#include <string_view>
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
    struct MeshPlanetCreateInfo
    {
        std::string name;
        WorldVec3 center_world{0.0, 0.0, 0.0};
        double radius_m = 1.0;
        bool visible = true;

        // Simple PBR constants (uses engine default white/flat/black textures).
        glm::vec4 base_color{1.0f, 1.0f, 1.0f, 1.0f};
        float metallic = 0.0f;
        float roughness = 1.0f;

        // Sphere tessellation (only used when generating a new mesh).
        uint32_t sectors = 48;
        uint32_t stacks = 24;
    };

    struct TerrainPlanetCreateInfo
    {
        std::string name;
        WorldVec3 center_world{0.0, 0.0, 0.0};
        double radius_m = 1.0;
        bool visible = true;

        // Simple PBR constants (uses engine default white/flat/black textures).
        glm::vec4 base_color{1.0f, 1.0f, 1.0f, 1.0f};
        float metallic = 0.0f;
        float roughness = 1.0f;

        // Optional terrain texture root (relative to assets/). If empty, terrain
        // uses fallback textures.
        std::string albedo_dir;
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

        // If true, this body is rendered as a cube-sphere quadtree terrain using the Earth patch path.
        // Otherwise, it is rendered as a regular mesh instance (sphere mesh etc.).
        bool terrain = false;

        // Shared PBR constants (used for mesh planets and terrain patch constants).
        glm::vec4 base_color{1.0f, 1.0f, 1.0f, 1.0f};
        float metallic = 0.0f;
        float roughness = 1.0f;

        // Terrain-only: cube-face texture root for albedo, relative to assets/.
        // Expected files: {px,nx,py,ny,pz,nz}.ktx2 (legacy names used by current asset pack).
        std::string terrain_albedo_dir;

        std::shared_ptr<MeshAsset> mesh;
        std::shared_ptr<GLTFMaterial> material;
    };

    void init(EngineContext *context);
    void cleanup();

    void update_and_emit(const SceneManager &scene, DrawContext &draw_context);

    bool enabled() const { return _enabled; }
    void set_enabled(bool enabled) { _enabled = enabled; }

    PlanetBody *find_body_by_name(std::string_view name);
    const std::vector<PlanetBody> &bodies() const { return _bodies; }

    // Runtime planet management
    // - Planets are identified by 'name' (must be unique).
    PlanetBody *create_mesh_planet(const MeshPlanetCreateInfo &info);
    PlanetBody *create_terrain_planet(const TerrainPlanetCreateInfo &info);
    bool destroy_planet(std::string_view name);
    void clear_planets(bool destroy_mesh_assets = true);

    bool set_planet_center(std::string_view name, const WorldVec3 &center_world);
    bool set_planet_radius(std::string_view name, double radius_m);
    bool set_planet_visible(std::string_view name, bool visible);
    bool set_planet_terrain(std::string_view name, bool terrain);

    const planet::PlanetQuadtree::Settings &earth_quadtree_settings() const { return _earth_quadtree_settings; }
    void set_earth_quadtree_settings(const planet::PlanetQuadtree::Settings &settings) { _earth_quadtree_settings = settings; }
    const EarthDebugStats &earth_debug_stats() const { return _earth_debug_stats; }

    uint32_t earth_patch_create_budget_per_frame() const { return _earth_patch_create_budget_per_frame; }
    void set_earth_patch_create_budget_per_frame(uint32_t budget) { _earth_patch_create_budget_per_frame = budget; }

    float earth_patch_create_budget_ms() const { return _earth_patch_create_budget_ms; }
    void set_earth_patch_create_budget_ms(float budget_ms) { _earth_patch_create_budget_ms = budget_ms; }

    uint32_t earth_patch_resolution() const { return _earth_patch_resolution; }
    void set_earth_patch_resolution(uint32_t resolution);

    uint32_t earth_patch_cache_max() const { return _earth_patch_cache_max; }
    void set_earth_patch_cache_max(uint32_t max_patches) { _earth_patch_cache_max = max_patches; }

    bool earth_debug_tint_patches_by_lod() const { return _earth_debug_tint_patches_by_lod; }
    void set_earth_debug_tint_patches_by_lod(bool enabled);

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

        glm::vec3 bounds_origin{0.0f};
        glm::vec3 bounds_extents{0.5f};
        float bounds_sphere_radius = 0.5f;

        WorldVec3 patch_center_dir{0.0, 0.0, 1.0};
        uint32_t last_used_frame = 0;
        std::list<uint32_t>::iterator lru_it{};
    };

    PlanetBody *find_terrain_body();
    const PlanetBody *find_terrain_body() const;
    EarthPatch *find_earth_patch(const planet::PatchKey &key);
    EarthPatch *get_or_create_earth_patch(const PlanetBody &earth,
                                          const planet::PatchKey &key,
                                          uint32_t frame_index);
    void ensure_earth_patch_index_buffer();
    void ensure_earth_patch_material_layout();
    void ensure_earth_patch_material_constants_buffer(const PlanetBody &earth);
    void ensure_earth_face_materials(const PlanetBody &earth);
    void clear_earth_patch_cache();
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
    glm::vec4 _earth_patch_bound_base_color{1.0f, 1.0f, 1.0f, 1.0f};
    float _earth_patch_bound_metallic = 0.0f;
    float _earth_patch_bound_roughness = 1.0f;
    std::string _earth_patch_bound_albedo_dir;
    std::array<MaterialInstance, 6> _earth_face_materials{};

    uint32_t _earth_patch_frame_stamp = 0;
    uint32_t _earth_patch_resolution = 33;
    uint32_t _earth_patch_create_budget_per_frame = 16;
    float _earth_patch_create_budget_ms = 2.0f;
    uint32_t _earth_patch_cache_max = 2048;

    bool _earth_debug_tint_patches_by_lod = false;
    bool _earth_patch_cache_dirty = false;
};
