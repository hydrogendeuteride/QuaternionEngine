#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>
#include <optional>
#include <functional>

class VulkanEngine;

namespace GameAPI
{

// ============================================================================
// Forward declarations and simple POD types
// ============================================================================

// Texture handle (opaque reference to a cached texture)
using TextureHandle = uint32_t;
static constexpr TextureHandle InvalidTexture = 0xFFFFFFFFu;

// Texture channel hint for memory optimization
enum class TextureChannels : uint8_t
{
    Auto = 0,  // Detect from source (default)
    R = 1,     // Single channel (e.g., occlusion, metallic)
    RG = 2,    // Two channels (e.g., normal map XY)
    RGBA = 3   // Full color
};

// Texture loading parameters
struct TextureLoadParams
{
    bool srgb{false};                      // Use sRGB color space (true for albedo/emissive)
    bool mipmapped{true};                  // Generate mipmap chain
    TextureChannels channels{TextureChannels::Auto};  // Channel hint
    uint32_t mipLevels{0};                 // 0 = full chain, otherwise limit to N levels
};

// Shadow rendering mode
enum class ShadowMode : uint32_t
{
    ClipmapOnly = 0,   // Raster shadow maps with PCF
    ClipmapPlusRT = 1, // Shadow maps + ray-traced assist at low N.L angles
    RTOnly = 2         // Pure ray-traced shadows (no shadow maps)
};

// Reflection rendering mode
enum class ReflectionMode : uint32_t
{
    SSROnly = 0,       // Screen-space reflections only
    SSRPlusRT = 1,     // SSR with ray-traced fallback
    RTOnly = 2         // Pure ray-traced reflections
};

// Tone mapping operator
enum class TonemapOperator : int
{
    Reinhard = 0,
    ACES = 1
};

// Voxel volume type (cloud/smoke/flame)
enum class VoxelVolumeType : uint32_t
{
    Clouds = 0,
    Smoke = 1,
    Flame = 2
};

// Particle blend mode
enum class ParticleBlendMode : uint32_t
{
    Additive = 0,  // Additive blending (for fire, sparks, etc.)
    Alpha = 1      // Alpha blending with depth sorting
};

// Primitive geometry types
enum class PrimitiveType
{
    Cube,
    Sphere,
    Plane,
    Capsule
};

// Simple procedural planet (sphere mesh) parameters
struct PlanetSphere
{
    std::string name;
    glm::dvec3 center{0.0};
    double radius_m{1.0};
    bool visible{true};

    glm::vec4 base_color{1.0f};
    float metallic{0.0f};
    float roughness{1.0f};

    uint32_t sectors{48};
    uint32_t stacks{24};
};

// Procedural planet terrain (cube-sphere quadtree patches) parameters.
struct PlanetTerrain
{
    std::string name;
    glm::dvec3 center{0.0};
    double radius_m{1.0};
    bool visible{true};

    glm::vec4 base_color{1.0f};
    float metallic{0.0f};
    float roughness{1.0f};

    // Optional terrain texture root relative to assets/ (e.g. "planets/earth/albedo/L0").
    // Expected files: {px,nx,py,ny,pz,nz}.ktx2
    std::string albedo_dir;

    // Optional height map root relative to assets/ (e.g. "planets/earth/height/L0").
    // Expected files: {px,nx,py,ny,pz,nz}.ktx2 (BC4/R8, linear). If empty, no displacement.
    std::string height_dir;
    // Height map range in meters for [0..1] texel values.
    double height_max_m{6400.0};

    // Optional emission texture root relative to assets/ (e.g. "planets/earth/emission/L0").
    // Expected files: {px,nx,py,ny,pz,nz}.ktx2 or .png (sRGB). If empty, no emission.
    std::string emission_dir;
    // Emission intensity multiplier (vec3 factor applied to texture RGB).
    glm::vec3 emission_factor{0.0f, 0.0f, 0.0f};
};

struct PlanetInfo
{
    std::string name;
    glm::dvec3 center{0.0};
    double radius_m{1.0};
    bool visible{true};
    bool terrain{false};
};

// Atmosphere scattering settings
struct AtmosphereSettings
{
    // If non-empty, selects the named PlanetSystem body for atmosphere rendering.
    // If empty, the renderer picks the closest visible planet to the camera.
    std::string bodyName{};

    // Atmosphere height above planet radius (meters).
    float atmosphereHeightM{80000.0f};

    // Scale heights (meters) for exponential density.
    float rayleighScaleHeightM{8000.0f};
    float mieScaleHeightM{1200.0f};

    // Scattering coefficients (1/m). Earth-ish defaults.
    glm::vec3 rayleighScattering{5.802e-6f, 13.558e-6f, 33.1e-6f};
    glm::vec3 mieScattering{21.0e-6f};

    // Henyey-Greenstein phase g (forward scattering).
    float mieG{0.76f};

    // Artistic controls.
    float intensity{1.0f};
    float sunDiskIntensity{1.0f};

    // Sun glare controls (applied by SunDiskPass; independent of atmosphere scattering).
    float sunHaloIntensity{0.0f};
    float sunHaloRadiusDeg{2.0f};
    float sunStarburstIntensity{0.0f};
    float sunStarburstRadiusDeg{6.0f};
    int sunStarburstSpikes{8};
    float sunStarburstSharpness{12.0f};

    // Sampling jitter (0 = off; 1 = full per-pixel jitter).
    float jitterStrength{0.0f};

    // Snap planet pixels to the analytic planet sphere within this distance (meters).
    float planetSurfaceSnapM{200.0f};

    // Integration quality/performance tradeoff.
    int viewSteps{16};
    int lightSteps{8};
};

// Planet quadtree (terrain LOD) settings
struct PlanetQuadtreeSettings
{
    uint32_t maxLevel{14};
    float targetScreenSpaceError{32.0f};
    uint32_t maxPatchesVisible{8192};
    bool frustumCull{true};
    bool horizonCull{true};
};

// Planet terrain debug statistics (read-only)
struct PlanetTerrainStats
{
    uint32_t visiblePatches{0};
    uint32_t renderedPatches{0};
    uint32_t createdPatches{0};
    uint32_t patchCacheSize{0};
    uint32_t estimatedTriangles{0};
    uint32_t maxLevelUsed{0};
    float msQuadtree{0.0f};
    float msPatchCreate{0.0f};
    float msTotal{0.0f};
};

// Sun shadow penumbra settings
struct SunShadowSettings
{
    // Sun angular radius (half-angle) in degrees for soft planet shadows.
    // Set to 0 for a hard edge. Default ~0.27 deg (real sun).
    float angularRadiusDeg{0.27f};
};

// Material description for textured primitives
struct PrimitiveMaterial
{
    std::string albedoPath;        // Color/diffuse texture (relative to assets/)
    std::string metalRoughPath;    // Metallic (R) + Roughness (G) texture
    std::string normalPath;        // Tangent-space normal map
    std::string occlusionPath;     // Ambient occlusion (R channel)
    std::string emissivePath;      // Emissive map

    glm::vec4 colorFactor{1.0f};   // Base color multiplier (RGBA)
    float metallic{0.0f};          // Metallic factor (0-1)
    float roughness{0.5f};         // Roughness factor (0-1)
};

// Point light data
struct PointLight
{
    glm::vec3 position{0.0f};
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
};

// Double-precision world-space point light data (position only).
struct PointLightD
{
    glm::dvec3 position{0.0};
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
};

// Spot light data (cone half-angles in degrees; inner <= outer)
struct SpotLight
{
    glm::vec3 position{0.0f};
    glm::vec3 direction{0.0f, -1.0f, 0.0f};
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
    float inner_angle_deg{15.0f};
    float outer_angle_deg{25.0f};
};

// Double-precision world-space spot light data (position only).
struct SpotLightD
{
    glm::dvec3 position{0.0};
    glm::vec3 direction{0.0f, -1.0f, 0.0f};
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
    float inner_angle_deg{15.0f};
    float outer_angle_deg{25.0f};
};

// Voxel volumetric settings (cloud/smoke/flame)
struct VoxelVolumeSettings
{
    bool enabled{false};
    VoxelVolumeType type{VoxelVolumeType::Clouds};

    // If true, volume follows camera XZ and volumeCenterLocal is treated as offset
    // If false, volumeCenterLocal is absolute render-local space
    bool followCameraXZ{false};

    // If true, run voxel advection/update compute pass every frame
    bool animateVoxels{true};

    // Volume AABB in render-local space
    glm::vec3 volumeCenterLocal{0.0f, 2.0f, 0.0f};
    glm::vec3 volumeHalfExtents{8.0f, 8.0f, 8.0f};

    // Optional volume drift (applied only when followCameraXZ == false)
    glm::vec3 volumeVelocityLocal{0.0f, 0.0f, 0.0f};

    // Raymarch/composite controls
    float densityScale{1.0f};
    float coverage{0.0f};        // 0..1 threshold (higher = emptier)
    float extinction{1.0f};      // absorption/extinction scale
    int stepCount{48};           // raymarch steps

    // Voxel grid resolution (cubic)
    uint32_t gridResolution{48};

    // Voxel animation (advection + injection) parameters
    glm::vec3 windVelocityLocal{0.0f, 2.0f, 0.0f}; // local units/sec (add buoyancy here)
    float dissipation{1.25f};    // density decay rate (1/sec)
    float noiseStrength{1.0f};   // injection rate
    float noiseScale{8.0f};      // noise frequency in UVW space
    float noiseSpeed{1.0f};      // time scale for injection noise

    // Smoke/flame source in normalized volume UVW space
    glm::vec3 emitterUVW{0.5f, 0.05f, 0.5f};
    float emitterRadius{0.18f};  // normalized (0..1-ish)

    // Shading
    glm::vec3 albedo{1.0f, 1.0f, 1.0f}; // scattering tint (cloud/smoke)
    float scatterStrength{1.0f};
    glm::vec3 emissionColor{1.0f, 0.6f, 0.25f}; // flame emissive tint
    float emissionStrength{0.0f};
};

// Particle system parameters
struct ParticleParams
{
    glm::vec3 emitterPosLocal{0.0f, 0.0f, 0.0f};
    float spawnRadius{0.1f};

    glm::vec3 emitterDirLocal{0.0f, 1.0f, 0.0f};
    float coneAngleDegrees{20.0f};

    float minSpeed{2.0f};
    float maxSpeed{8.0f};

    float minLife{0.5f};
    float maxLife{1.5f};

    float minSize{0.05f};
    float maxSize{0.15f};

    float drag{1.0f};
    float gravity{0.0f}; // positive pulls down -Y in local space

    glm::vec4 color{1.0f, 0.5f, 0.1f, 1.0f};

    // Fade particles near opaque geometry intersections (0 disables)
    float softDepthDistance{0.15f};

    // Flipbook sampling (atlas layout and animation)
    uint32_t flipbookCols{16};
    uint32_t flipbookRows{4};
    float flipbookFps{30.0f};
    float flipbookIntensity{1.0f};

    // Noise UV distortion
    float noiseScale{6.0f};
    float noiseStrength{0.05f};
    glm::vec2 noiseScroll{0.0f, 0.0f};
};

// Particle system settings
struct ParticleSystem
{
    uint32_t id{0};
    uint32_t particleCount{0};
    bool enabled{true};
    bool reset{true};
    ParticleBlendMode blendMode{ParticleBlendMode::Additive};
    ParticleParams params{};

    // Asset-relative texture paths (e.g., "vfx/flame.ktx2")
    // Empty string disables the texture
    std::string flipbookTexture{"vfx/flame.ktx2"};
    std::string noiseTexture{"vfx/simplex.ktx2"};
};

// IBL (Image-Based Lighting) paths
	struct IBLPaths
	{
	    std::string specularCube; // .ktx2 specular cubemap
	    std::string diffuseCube;  // .ktx2 diffuse cubemap
	    std::string brdfLut;      // .ktx2 BRDF lookup table
	    std::string background;   // .ktx2 background (optional, falls back to specular)
	};

	enum class IBLVolumeShape : uint8_t
	{
	    Box = 0,
	    Sphere = 1
	};

	// IBL Volume (local reflection probe)
	struct IBLVolume
	{
	    glm::vec3 center{0.0f};
	    glm::vec3 halfExtents{10.0f};
	    IBLPaths paths;
	    bool enabled{true};
	    IBLVolumeShape shape{IBLVolumeShape::Box};
	    float radius{10.0f};
	};

	// Double-precision world-space IBL volume (center only).
	struct IBLVolumeD
	{
	    glm::dvec3 center{0.0};
	    glm::vec3 halfExtents{10.0f};
	    IBLPaths paths;
	    bool enabled{true};
	    IBLVolumeShape shape{IBLVolumeShape::Box};
	    float radius{10.0f};
	};

// Transform decomposition
struct Transform
{
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};

    glm::mat4 to_matrix() const;
    static Transform from_matrix(const glm::mat4& m);
};

// Double-precision world-space transform (position only).
struct TransformD
{
    glm::dvec3 position{0.0};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};

    glm::mat4 to_matrix() const;
    static TransformD from_matrix(const glm::mat4& m);
};

// Engine statistics (read-only)
struct Stats
{
    float frametime{0.0f};       // ms
    float drawTime{0.0f};        // ms
    float sceneUpdateTime{0.0f}; // ms
    int triangleCount{0};
    int drawCallCount{0};
};

// ============================================================================
// Camera Rig Types
// ============================================================================

enum class CameraMode : uint8_t
{
    Free = 0,
    Orbit = 1,
    Follow = 2,
    Chase = 3,
    Fixed = 4
};

enum class CameraTargetType : uint8_t
{
    None = 0,
    WorldPoint = 1,
    MeshInstance = 2,
    GLTFInstance = 3
};

struct CameraTarget
{
    CameraTargetType type{CameraTargetType::None};
    std::string name{};
    glm::dvec3 worldPoint{0.0, 0.0, 0.0};
};

struct FreeCameraSettings
{
    float moveSpeed{1.8f};        // world units / second
    float lookSensitivity{0.0020f};
    float rollSpeed{1.0f};        // radians / second
};

struct OrbitCameraSettings
{
    CameraTarget target{};
    double distance{10.0};
    float yaw{0.0f};              // radians
    float pitch{0.0f};            // radians
    float lookSensitivity{0.0020f};
};

struct FollowCameraSettings
{
    CameraTarget target{};
    glm::vec3 positionOffsetLocal{0.0f, 2.0f, 6.0f};
    glm::quat rotationOffset{1.0f, 0.0f, 0.0f, 0.0f};
};

struct ChaseCameraSettings
{
    CameraTarget target{};
    glm::vec3 positionOffsetLocal{0.0f, 2.0f, 6.0f};
    glm::quat rotationOffset{1.0f, 0.0f, 0.0f, 0.0f};
    float positionLag{8.0f};      // smoothing rate (1/sec), higher = snappier
    float rotationLag{10.0f};     // smoothing rate (1/sec)
};

// ============================================================================
// Main API Class
// ============================================================================

class Engine
{
public:
    explicit Engine(VulkanEngine* engine);
    ~Engine() = default;

    // Non-copyable
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // ------------------------------------------------------------------------
    // Memory / Texture Streaming
    // ------------------------------------------------------------------------

    // Query current VRAM texture budget (bytes)
    size_t get_texture_budget() const;

    // Set maximum textures loaded per frame (1-16)
    void set_texture_loads_per_frame(int count);
    int get_texture_loads_per_frame() const;

    // Set upload budget per frame (bytes, e.g., 128*1024*1024 = 128 MiB)
    void set_texture_upload_budget(size_t bytes);
    size_t get_texture_upload_budget() const;

    // Set CPU source data budget (bytes)
    void set_cpu_source_budget(size_t bytes);
    size_t get_cpu_source_budget() const;

    // Set maximum upload dimension (clamps large textures)
    void set_max_upload_dimension(uint32_t dim);
    uint32_t get_max_upload_dimension() const;

    // Keep CPU source data after GPU upload (useful for streaming)
    void set_keep_source_bytes(bool keep);
    bool get_keep_source_bytes() const;

    // Force eviction to budget (call after loading large assets)
    void evict_textures_to_budget();

    // ------------------------------------------------------------------------
    // Texture Loading
    // ------------------------------------------------------------------------

    // Load a texture from file path (relative to assets/textures/ or absolute)
    // Returns a handle that can be used to query state or bind to materials
    TextureHandle load_texture(const std::string& path,
                                const TextureLoadParams& params = {});

    // Load a texture from memory (compressed image data: PNG, JPG, KTX2, etc.)
    // Useful for runtime-generated or downloaded textures
    TextureHandle load_texture_from_memory(const std::vector<uint8_t>& data,
                                            const TextureLoadParams& params = {});

    // Check if a texture is loaded and resident in VRAM
    bool is_texture_loaded(TextureHandle handle) const;

    // Get the internal Vulkan image view for advanced use cases
    // Returns VK_NULL_HANDLE if texture is not yet loaded
    void* get_texture_image_view(TextureHandle handle) const;  // Returns VkImageView

    // Pin a texture to prevent automatic eviction (useful for UI elements, critical assets)
    // Pinned textures are never removed from VRAM by LRU or budget constraints
    void pin_texture(TextureHandle handle);

    // Unpin a texture, allowing it to be evicted normally
    void unpin_texture(TextureHandle handle);

    // Check if a texture is currently pinned
    bool is_texture_pinned(TextureHandle handle) const;

    // Unload a texture and free VRAM (textures are ref-counted and auto-evicted by LRU)
    // This is optional - the cache manages memory automatically
    void unload_texture(TextureHandle handle);

    // Create an ImGui descriptor set for a texture (for use with ImGui::Image())
    // Returns ImTextureID (actually VkDescriptorSet) that can be used in ImGui
    // The returned descriptor set is managed by ImGui and valid until cleanup
    // sampler: VK_NULL_HANDLE uses default linear sampler
    void* create_imgui_texture(TextureHandle handle, void* sampler = nullptr);

    // Free an ImGui descriptor set created by create_imgui_texture()
    void free_imgui_texture(void* imgui_texture_id);

    // ------------------------------------------------------------------------
    // Shadows
    // ------------------------------------------------------------------------

    void set_shadows_enabled(bool enabled);
    bool get_shadows_enabled() const;

    void set_shadow_mode(ShadowMode mode);
    ShadowMode get_shadow_mode() const;

    // For hybrid mode: which cascades use ray assist (bitmask, bits 0-3)
    void set_hybrid_ray_cascade_mask(uint32_t mask);
    uint32_t get_hybrid_ray_cascade_mask() const;

    // N.L threshold for hybrid ray shadows (0.0 - 1.0)
    void set_hybrid_ray_threshold(float threshold);
    float get_hybrid_ray_threshold() const;

    // ------------------------------------------------------------------------
    // IBL (Image-Based Lighting)
    // ------------------------------------------------------------------------

    // Load global IBL asynchronously (returns false if failed to queue)
    bool load_global_ibl(const IBLPaths& paths);

    // Get/set global IBL paths (does not trigger reload)
    IBLPaths get_global_ibl_paths() const;
    void set_global_ibl_paths(const IBLPaths& paths);

	    // Add a local IBL volume (returns volume index)
	    size_t add_ibl_volume(const IBLVolume& volume);
	    size_t add_ibl_volume(const IBLVolumeD& volume);
	    size_t add_ibl_sphere_volume(const glm::vec3& center, float radius, const IBLPaths& paths, bool enabled = true);
	    size_t add_ibl_sphere_volume(const glm::dvec3& center, float radius, const IBLPaths& paths, bool enabled = true);

    // Remove IBL volume by index
    bool remove_ibl_volume(size_t index);

    // Get/set IBL volume properties
    bool get_ibl_volume(size_t index, IBLVolume& out) const;
    bool set_ibl_volume(size_t index, const IBLVolume& volume);
    bool get_ibl_volume(size_t index, IBLVolumeD& out) const;
    bool set_ibl_volume(size_t index, const IBLVolumeD& volume);

    // Get current active IBL volume index (-1 = global)
    int get_active_ibl_volume() const;

    // Get IBL volume count
    size_t get_ibl_volume_count() const;

    // Clear all IBL volumes
    void clear_ibl_volumes();

    // ------------------------------------------------------------------------
    // Objects / Instances
    // ------------------------------------------------------------------------

    // Add glTF model instance (path relative to assets/models/)
    bool add_gltf_instance(const std::string& name,
                         const std::string& modelPath,
                         const Transform& transform = {},
                         bool preloadTextures = true);
    bool add_gltf_instance(const std::string& name,
                           const std::string& modelPath,
                           const TransformD& transform,
                           bool preloadTextures = true);

    // Add glTF model asynchronously (returns job ID, 0 on failure)
    uint32_t add_gltf_instance_async(const std::string& name,
                                  const std::string& modelPath,
                                  const Transform& transform = {},
                                  bool preloadTextures = true);
    uint32_t add_gltf_instance_async(const std::string& name,
                                     const std::string& modelPath,
                                     const TransformD& transform,
                                     bool preloadTextures = true);

    // Remove glTF instance
    bool remove_gltf_instance(const std::string& name);

    // Get/set glTF instance transform
    bool get_gltf_instance_transform(const std::string& name, Transform& out) const;
    bool set_gltf_instance_transform(const std::string& name, const Transform& transform);
    bool get_gltf_instance_transform(const std::string& name, TransformD& out) const;
    bool set_gltf_instance_transform(const std::string& name, const TransformD& transform);

    // Add primitive mesh instance
    bool add_primitive_instance(const std::string& name,
                              PrimitiveType type,
                              const Transform& transform = {});
    bool add_primitive_instance(const std::string& name,
                                PrimitiveType type,
                                const TransformD& transform);

    // Add primitive mesh instance with textures
    bool add_textured_primitive(const std::string& name,
                                PrimitiveType type,
                                const PrimitiveMaterial& material,
                                const Transform& transform = {});
    bool add_textured_primitive(const std::string& name,
                                PrimitiveType type,
                                const PrimitiveMaterial& material,
                                const TransformD& transform);

    // Remove mesh instance (primitives or custom meshes)
    bool remove_mesh_instance(const std::string& name);

    // Get/set mesh instance transform
    bool get_mesh_instance_transform(const std::string& name, Transform& out) const;
    bool set_mesh_instance_transform(const std::string& name, const Transform& transform);
    bool get_mesh_instance_transform(const std::string& name, TransformD& out) const;
    bool set_mesh_instance_transform(const std::string& name, const TransformD& transform);

    // Preload textures for an instance (useful before it becomes visible)
    void preload_instance_textures(const std::string& name);

    // Clear all dynamic instances
    void clear_all_instances();

    // ------------------------------------------------------------------------
    // Planets
    // ------------------------------------------------------------------------

    // Create/destroy simple sphere planets (procedural mesh in PlanetSystem).
    bool add_planet_sphere(const PlanetSphere &planet);
    bool add_planet_terrain(const PlanetTerrain &planet);
    bool remove_planet(const std::string &name);
    void clear_planets(bool destroy_mesh_assets = true);

    // Planet queries / edits (by name).
    bool get_planet(const std::string &name, PlanetInfo &out) const;
    size_t get_planet_count() const;
    bool get_planet(size_t index, PlanetInfo &out) const;
    bool set_planet_center(const std::string &name, const glm::dvec3 &center);
    bool set_planet_radius(const std::string &name, double radius_m);
    bool set_planet_visible(const std::string &name, bool visible);
    bool set_planet_terrain(const std::string &name, bool terrain);

    // Planet system global enable/disable.
    void set_planet_system_enabled(bool enabled);
    bool get_planet_system_enabled() const;

    // Planet terrain LOD (quadtree) settings.
    void set_planet_quadtree_settings(const PlanetQuadtreeSettings &settings);
    PlanetQuadtreeSettings get_planet_quadtree_settings() const;

    // Planet terrain patch budget/resolution settings.
    void set_planet_patch_create_budget(uint32_t patchesPerFrame);
    uint32_t get_planet_patch_create_budget() const;
    void set_planet_patch_create_budget_ms(float budgetMs);
    float get_planet_patch_create_budget_ms() const;
    void set_planet_patch_resolution(uint32_t resolution);
    uint32_t get_planet_patch_resolution() const;
    void set_planet_patch_cache_max(uint32_t maxPatches);
    uint32_t get_planet_patch_cache_max() const;

    // Planet terrain debug.
    void set_planet_debug_tint_by_lod(bool enabled);
    bool get_planet_debug_tint_by_lod() const;
    PlanetTerrainStats get_planet_terrain_stats(const std::string &name = {}) const;

    // Sample terrain height at a given direction from planet center (returns meters above base radius).
    double sample_planet_terrain_height(const std::string &name, const glm::dvec3 &dirFromCenter) const;

    // ------------------------------------------------------------------------
    // Atmosphere
    // ------------------------------------------------------------------------

    // Enable/disable atmosphere scattering pass.
    void set_atmosphere_enabled(bool enabled);
    bool get_atmosphere_enabled() const;

    // Get/set atmosphere settings.
    void set_atmosphere_settings(const AtmosphereSettings &settings);
    AtmosphereSettings get_atmosphere_settings() const;

    // Reset atmosphere to Earth-like defaults.
    void reset_atmosphere_to_earth();

    // ------------------------------------------------------------------------
    // Sun Shadow (Penumbra)
    // ------------------------------------------------------------------------

    // Sun angular radius for soft planet->scene shadows.
    void set_sun_shadow_settings(const SunShadowSettings &settings);
    SunShadowSettings get_sun_shadow_settings() const;

    // ------------------------------------------------------------------------
    // Animation
    // ------------------------------------------------------------------------

    // Set animation by index for a glTF instance (-1 to disable)
    bool set_instance_animation(const std::string& instanceName, int animationIndex, bool resetTime = true);

    // Set animation by name for a glTF instance
    bool set_instance_animation(const std::string& instanceName, const std::string& animationName, bool resetTime = true);

    // Set animation looping for a glTF instance
    bool set_instance_animation_loop(const std::string& instanceName, bool loop);

    // Playback speed multiplier for a glTF instance (1.0 = realtime).
    bool set_instance_animation_speed(const std::string& instanceName, float speed);

    // Cross-fade transition to a new animation for a glTF instance.
    bool transition_instance_animation(const std::string& instanceName, int animationIndex, float blendDurationSeconds, bool resetTime = true);
    bool transition_instance_animation(const std::string& instanceName, const std::string& animationName, float blendDurationSeconds, bool resetTime = true);

    // Per-node transform offset (local space, layered on animation)
    bool set_instance_node_offset(const std::string& instanceName, const std::string& nodeName, const glm::mat4& offset);
    bool clear_instance_node_offset(const std::string& instanceName, const std::string& nodeName);
    void clear_all_instance_node_offsets(const std::string& instanceName);

    // ------------------------------------------------------------------------
    // Lighting - Directional (Sunlight)
    // ------------------------------------------------------------------------

    // Set sunlight direction (normalized automatically)
    void set_sunlight_direction(const glm::vec3& dir);
    glm::vec3 get_sunlight_direction() const;

    // Set sunlight color and intensity
    void set_sunlight_color(const glm::vec3& color, float intensity);
    glm::vec3 get_sunlight_color() const;
    float get_sunlight_intensity() const;

    // ------------------------------------------------------------------------
    // Lighting - Point Lights
    // ------------------------------------------------------------------------

    // Add point light (returns index)
    size_t add_point_light(const PointLight& light);
    size_t add_point_light(const PointLightD& light);

    // Remove point light by index
    bool remove_point_light(size_t index);

    // Get/set point light properties
    bool get_point_light(size_t index, PointLight& out) const;
    bool set_point_light(size_t index, const PointLight& light);
    bool get_point_light(size_t index, PointLightD& out) const;
    bool set_point_light(size_t index, const PointLightD& light);

    // Get point light count
    size_t get_point_light_count() const;

    // Clear all point lights
    void clear_point_lights();

    // ------------------------------------------------------------------------
    // Lighting - Spot Lights
    // ------------------------------------------------------------------------

    // Add spot light (returns index)
    size_t add_spot_light(const SpotLight& light);
    size_t add_spot_light(const SpotLightD& light);

    // Remove spot light by index
    bool remove_spot_light(size_t index);

    // Get/set spot light properties
    bool get_spot_light(size_t index, SpotLight& out) const;
    bool set_spot_light(size_t index, const SpotLight& light);
    bool get_spot_light(size_t index, SpotLightD& out) const;
    bool set_spot_light(size_t index, const SpotLightD& light);

    // Get spot light count
    size_t get_spot_light_count() const;

    // Clear all spot lights
    void clear_spot_lights();

    // ------------------------------------------------------------------------
    // Post Processing - FXAA
    // ------------------------------------------------------------------------

    void set_fxaa_enabled(bool enabled);
    bool get_fxaa_enabled() const;

    void set_fxaa_edge_threshold(float threshold);
    float get_fxaa_edge_threshold() const;

    void set_fxaa_edge_threshold_min(float threshold);
    float get_fxaa_edge_threshold_min() const;

    // ------------------------------------------------------------------------
    // Post Processing - SSR (Screen Space Reflections)
    // ------------------------------------------------------------------------

    void set_ssr_enabled(bool enabled);
    bool get_ssr_enabled() const;

    void set_reflection_mode(ReflectionMode mode);
    ReflectionMode get_reflection_mode() const;

    // ------------------------------------------------------------------------
    // Post Processing - Tonemapping
    // ------------------------------------------------------------------------

    void set_exposure(float exposure);
    float get_exposure() const;

    void set_tonemap_operator(TonemapOperator op);
    TonemapOperator get_tonemap_operator() const;

    // ------------------------------------------------------------------------
    // Post Processing - Bloom
    // ------------------------------------------------------------------------

    void set_bloom_enabled(bool enabled);
    bool get_bloom_enabled() const;

    void set_bloom_threshold(float threshold);
    float get_bloom_threshold() const;

    void set_bloom_intensity(float intensity);
    float get_bloom_intensity() const;

    // ------------------------------------------------------------------------
    // Camera
    // ------------------------------------------------------------------------

    void set_camera_position(const glm::vec3& position);
    glm::vec3 get_camera_position() const;
    void set_camera_position(const glm::dvec3& position);
    glm::dvec3 get_camera_position_d() const;

    void set_camera_rotation(float pitch, float yaw);
    void get_camera_rotation(float& pitch, float& yaw) const;

    void set_camera_fov(float fovDegrees);
    float get_camera_fov() const;

    // Look at a target position
    void camera_look_at(const glm::vec3& target);
    void camera_look_at(const glm::dvec3& target);

    // Camera mode and per-mode settings
    void set_camera_mode(CameraMode mode);
    CameraMode get_camera_mode() const;

    void set_free_camera_settings(const FreeCameraSettings& settings);
    FreeCameraSettings get_free_camera_settings() const;

    void set_orbit_camera_settings(const OrbitCameraSettings& settings);
    OrbitCameraSettings get_orbit_camera_settings() const;

    void set_follow_camera_settings(const FollowCameraSettings& settings);
    FollowCameraSettings get_follow_camera_settings() const;

    void set_chase_camera_settings(const ChaseCameraSettings& settings);
    ChaseCameraSettings get_chase_camera_settings() const;

    // Convenience: set Orbit/Follow/Chase target from the engine's last pick.
    bool set_camera_target_from_last_pick();

    // Align orbit camera's reference up vector to the target's local up (Y axis).
    void align_orbit_camera_up_to_target();

    // Set orbit camera's reference up vector explicitly.
    void set_orbit_camera_reference_up(const glm::vec3& up);

    // ------------------------------------------------------------------------
    // Floating Origin
    // ------------------------------------------------------------------------

    // Current world origin used by the renderer (double precision).
    glm::dvec3 get_world_origin() const;

    // ------------------------------------------------------------------------
    // Rendering
    // ------------------------------------------------------------------------

    void set_render_scale(float scale); // 0.3 - 1.0
    float get_render_scale() const;

    // Enable/disable specific render passes by name
    void set_pass_enabled(const std::string& passName, bool enabled);
    bool get_pass_enabled(const std::string& passName) const;

    // Hot reload all changed shaders
    void hot_reload_shaders();

    // ------------------------------------------------------------------------
    // Time
    // ------------------------------------------------------------------------

    // Get delta time in seconds for the current frame (clamped to 0.0-0.1)
    float get_delta_time() const;

    // ------------------------------------------------------------------------
    // Statistics (read-only)
    // ------------------------------------------------------------------------

    Stats get_stats() const;

    // ------------------------------------------------------------------------
    // Volumetrics (Cloud/Smoke/Flame)
    // ------------------------------------------------------------------------

    // Enable/disable volumetrics system
    void set_volumetrics_enabled(bool enabled);
    bool get_volumetrics_enabled() const;

    // Get/set voxel volume settings by index (0-3)
    bool get_voxel_volume(size_t index, VoxelVolumeSettings& out) const;
    bool set_voxel_volume(size_t index, const VoxelVolumeSettings& settings);

    // Get maximum number of voxel volumes
    size_t get_max_voxel_volumes() const;

    // ------------------------------------------------------------------------
    // Particle Systems
    // ------------------------------------------------------------------------

    // Create a new particle system (returns system ID, 0 on failure)
    uint32_t create_particle_system(uint32_t particle_count);

    // Destroy a particle system by ID
    bool destroy_particle_system(uint32_t id);

    // Resize a particle system (reallocates particle count)
    bool resize_particle_system(uint32_t id, uint32_t new_count);

    // Get particle system settings by ID
    bool get_particle_system(uint32_t id, ParticleSystem& out) const;

    // Set particle system settings by ID
    bool set_particle_system(uint32_t id, const ParticleSystem& system);

    // Get all particle system IDs
    std::vector<uint32_t> get_particle_system_ids() const;

    // Get particle pool statistics
    uint32_t get_allocated_particles() const;
    uint32_t get_free_particles() const;
    uint32_t get_max_particles() const;

    // Preload a VFX texture (e.g., "vfx/flame.ktx2")
    void preload_particle_texture(const std::string& assetPath);

    // ------------------------------------------------------------------------
    // Picking / Selection
    // ------------------------------------------------------------------------

    struct PickResult
    {
        bool valid{false};
        std::string ownerName;
        glm::vec3 worldPosition{0.0f};
    };

    struct PickResultD
    {
        bool valid{false};
        std::string ownerName;
        glm::dvec3 worldPosition{0.0};
    };

    // Get last click selection result
    PickResult get_last_pick() const;
    PickResultD get_last_pick_d() const;

    // Enable/disable ID buffer picking (vs CPU raycast)
    void set_use_id_buffer_picking(bool use);
    bool get_use_id_buffer_picking() const;

    // ------------------------------------------------------------------------
    // Debug Drawing
    // ------------------------------------------------------------------------

    // Enable/disable debug drawing system
    void set_debug_draw_enabled(bool enabled);
    bool get_debug_draw_enabled() const;

    // Control which debug layers are visible
    void set_debug_layer_mask(uint32_t mask);
    uint32_t get_debug_layer_mask() const;

    // Show/hide depth-tested debug primitives
    void set_debug_show_depth_tested(bool show);
    bool get_debug_show_depth_tested() const;

    // Show/hide overlay (always-on-top) debug primitives
    void set_debug_show_overlay(bool show);
    bool get_debug_show_overlay() const;

    // Set debug primitive tessellation quality (segments for circles/spheres)
    void set_debug_segments(int segments);
    int get_debug_segments() const;

    // Clear all debug draw commands
    void debug_draw_clear();

    // Debug line primitives (world-space positions)
    void debug_draw_line(const glm::vec3& a, const glm::vec3& b,
                         const glm::vec4& color = glm::vec4(1.0f),
                         float duration_seconds = 0.0f,
                         bool depth_tested = true);
    void debug_draw_line(const glm::dvec3& a, const glm::dvec3& b,
                         const glm::vec4& color = glm::vec4(1.0f),
                         float duration_seconds = 0.0f,
                         bool depth_tested = true);

    // Debug ray (origin + direction + length)
    void debug_draw_ray(const glm::vec3& origin, const glm::vec3& direction, float length,
                        const glm::vec4& color = glm::vec4(1.0f),
                        float duration_seconds = 0.0f,
                        bool depth_tested = true);
    void debug_draw_ray(const glm::dvec3& origin, const glm::dvec3& direction, double length,
                        const glm::vec4& color = glm::vec4(1.0f),
                        float duration_seconds = 0.0f,
                        bool depth_tested = true);

    // Debug AABB (axis-aligned bounding box)
    void debug_draw_aabb(const glm::vec3& center, const glm::vec3& half_extents,
                         const glm::vec4& color = glm::vec4(1.0f),
                         float duration_seconds = 0.0f,
                         bool depth_tested = true);
    void debug_draw_aabb(const glm::dvec3& center, const glm::vec3& half_extents,
                         const glm::vec4& color = glm::vec4(1.0f),
                         float duration_seconds = 0.0f,
                         bool depth_tested = true);

    // Debug sphere
    void debug_draw_sphere(const glm::vec3& center, float radius,
                           const glm::vec4& color = glm::vec4(1.0f),
                           float duration_seconds = 0.0f,
                           bool depth_tested = true);
    void debug_draw_sphere(const glm::dvec3& center, float radius,
                           const glm::vec4& color = glm::vec4(1.0f),
                           float duration_seconds = 0.0f,
                           bool depth_tested = true);

    // Debug capsule (line segment + radius)
    void debug_draw_capsule(const glm::vec3& p0, const glm::vec3& p1, float radius,
                            const glm::vec4& color = glm::vec4(1.0f),
                            float duration_seconds = 0.0f,
                            bool depth_tested = true);
    void debug_draw_capsule(const glm::dvec3& p0, const glm::dvec3& p1, float radius,
                            const glm::vec4& color = glm::vec4(1.0f),
                            float duration_seconds = 0.0f,
                            bool depth_tested = true);

    // Debug circle (center + normal + radius)
    void debug_draw_circle(const glm::vec3& center, const glm::vec3& normal, float radius,
                           const glm::vec4& color = glm::vec4(1.0f),
                           float duration_seconds = 0.0f,
                           bool depth_tested = true);
    void debug_draw_circle(const glm::dvec3& center, const glm::dvec3& normal, float radius,
                           const glm::vec4& color = glm::vec4(1.0f),
                           float duration_seconds = 0.0f,
                           bool depth_tested = true);

    // Debug cone (apex + direction + length + angle)
    void debug_draw_cone(const glm::vec3& apex, const glm::vec3& direction,
                         float length, float angle_degrees,
                         const glm::vec4& color = glm::vec4(1.0f),
                         float duration_seconds = 0.0f,
                         bool depth_tested = true);
    void debug_draw_cone(const glm::dvec3& apex, const glm::dvec3& direction,
                         float length, float angle_degrees,
                         const glm::vec4& color = glm::vec4(1.0f),
                         float duration_seconds = 0.0f,
                         bool depth_tested = true);

private:
    VulkanEngine* _engine;
};

} // namespace GameAPI
