#pragma once

#include <core/world.h>
#include <core/descriptor/descriptors.h>
#include <scene/planet/planet_heightmap.h>
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

        // Optional height map root (relative to assets/). If empty, terrain has no displacement.
        // Expected files: {px,nx,py,ny,pz,nz}.ktx2 (BC4/R8, linear).
        std::string height_dir;
        // Height map range in meters for [0..1] texel values.
        double height_max_m = 6400.0;

        // Optional emission texture root (relative to assets/). If empty, no emission.
        // Expected files: {px,nx,py,ny,pz,nz}.ktx2 or .png (sRGB).
        std::string emission_dir;
        // Emission intensity multiplier (vec3 factor applied to texture RGB).
        glm::vec3 emission_factor{0.0f, 0.0f, 0.0f};
    };

    struct EarthDebugStats
    {
        planet::PlanetQuadtree::Stats quadtree{};
        uint32_t visible_patches = 0;
        uint32_t rendered_patches = 0;
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

        // Terrain-only: cube-face texture root for height, relative to assets/.
        // Expected files: {px,nx,py,ny,pz,nz}.ktx2 (BC4/R8, linear).
        std::string terrain_height_dir;
        double terrain_height_max_m = 0.0;

        // Terrain-only: cube-face texture root for emission, relative to assets/.
        // Expected files: {px,nx,py,ny,pz,nz}.ktx2 or .png (sRGB).
        std::string terrain_emission_dir;
        // Emission intensity multiplier (vec3 factor applied to texture RGB).
        glm::vec3 emission_factor{0.0f, 0.0f, 0.0f};

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

    // Returns the terrain displacement height (meters) at the given direction from the planet center.
    // If height mapping is disabled/unavailable, returns 0.
    double sample_terrain_displacement_m(const PlanetBody &body, const glm::dvec3 &dir_from_center) const;

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

    void set_earth_quadtree_settings(const planet::PlanetQuadtree::Settings &settings)
    {
        _earth_quadtree_settings = settings;
    }

    // Terrain debug stats for a specific planet name (returns empty stats if not found).
    const EarthDebugStats &terrain_debug_stats(std::string_view name) const;

    // Back-compat: returns debug stats for the first terrain planet, if any.
    const EarthDebugStats &earth_debug_stats() const;

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
    PlanetBody *find_terrain_body();

    const PlanetBody *find_terrain_body() const;

    enum class TerrainPatchState : uint8_t
    {
        Allocating = 0,
        Ready = 1,
    };

    struct TerrainPatch
    {
        planet::PatchKey key{};
        TerrainPatchState state = TerrainPatchState::Allocating;

        AllocatedBuffer vertex_buffer{};
        VkDeviceAddress vertex_buffer_address = 0;

        glm::vec3 bounds_origin{0.0f};
        glm::vec3 bounds_extents{0.5f};
        float bounds_sphere_radius = 0.5f;

        WorldVec3 patch_center_dir{0.0, 0.0, 1.0};
        uint32_t last_used_frame = 0;
        std::list<uint32_t>::iterator lru_it{};
    };

    struct TerrainState
    {
        planet::PlanetQuadtree quadtree{};
        EarthDebugStats debug_stats{};

        std::unordered_map<planet::PatchKey, uint32_t, planet::PatchKeyHash> patch_lookup;
        std::vector<TerrainPatch> patches;
        std::vector<uint32_t> patch_free;
        std::list<uint32_t> patch_lru;

        AllocatedBuffer material_constants_buffer{};
        glm::vec4 bound_base_color{1.0f, 1.0f, 1.0f, 1.0f};
        float bound_metallic = 0.0f;
        float bound_roughness = 1.0f;
        glm::vec3 bound_emission_factor{0.0f, 0.0f, 0.0f};
        std::string bound_albedo_dir;
        std::string bound_height_dir;
        std::string bound_emission_dir;
        double bound_height_max_m = 0.0;
        std::array<planet::HeightFace, 6> height_faces{};
        std::array<MaterialInstance, 6> face_materials{};

        uint32_t patch_frame_stamp = 0;
        bool patch_cache_dirty = false;
    };

    TerrainState *get_or_create_terrain_state(std::string_view name);

    TerrainState *find_terrain_state(std::string_view name);

    const TerrainState *find_terrain_state(std::string_view name) const;

    TerrainPatch *find_terrain_patch(TerrainState &state, const planet::PatchKey &key);

    TerrainPatch *get_or_create_terrain_patch(TerrainState &state,
                                              const PlanetBody &body,
                                              const planet::PatchKey &key,
                                              uint32_t frame_index);

    void ensure_earth_patch_index_buffer();

    void ensure_earth_patch_material_layout();

    void ensure_terrain_material_constants_buffer(TerrainState &state, const PlanetBody &body);

    void ensure_terrain_face_materials(TerrainState &state, const PlanetBody &body);

    void ensure_terrain_height_maps(TerrainState &state, const PlanetBody &body);

    void clear_terrain_patch_cache(TerrainState &state);

    void trim_terrain_patch_cache(TerrainState &state);

    void clear_all_terrain_patch_caches();

    void clear_terrain_materials(TerrainState &state);

    EngineContext *_context = nullptr;
    bool _enabled = true;
    std::vector<PlanetBody> _bodies;

    planet::PlanetQuadtree::Settings _earth_quadtree_settings{};
    std::unordered_map<std::string, std::unique_ptr<TerrainState> > _terrain_states;
    EarthDebugStats _empty_debug_stats{};
    AllocatedBuffer _earth_patch_index_buffer{};
    uint32_t _earth_patch_index_count = 0;
    uint32_t _earth_patch_index_resolution = 0;

    VkDescriptorSetLayout _earth_patch_material_layout = VK_NULL_HANDLE;
    DescriptorAllocatorGrowable _earth_patch_material_allocator{};
    bool _earth_patch_material_allocator_initialized = false;
    uint32_t _earth_patch_resolution = 33;
    uint32_t _earth_patch_create_budget_per_frame = 16;
    float _earth_patch_create_budget_ms = 2.0f;
    uint32_t _earth_patch_cache_max = 2048;

    bool _earth_debug_tint_patches_by_lod = false;
    bool _earth_patch_cache_dirty = false;
};
