#pragma once

// GameAPI: High-level interface for game development
// Wraps VulkanEngine internals and exposes clean, game-friendly functions.

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

// Primitive geometry types
enum class PrimitiveType
{
    Cube,
    Sphere,
    Plane,
    Capsule
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

// IBL (Image-Based Lighting) paths
struct IBLPaths
{
    std::string specularCube; // .ktx2 specular cubemap
    std::string diffuseCube;  // .ktx2 diffuse cubemap
    std::string brdfLut;      // .ktx2 BRDF lookup table
    std::string background;   // .ktx2 background (optional, falls back to specular)
};

// IBL Volume (local reflection probe)
struct IBLVolume
{
    glm::vec3 center{0.0f};
    glm::vec3 halfExtents{10.0f};
    IBLPaths paths;
    bool enabled{true};
};

// Double-precision world-space IBL volume (center only).
struct IBLVolumeD
{
    glm::dvec3 center{0.0};
    glm::vec3 halfExtents{10.0f};
    IBLPaths paths;
    bool enabled{true};
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
    // Animation
    // ------------------------------------------------------------------------

    // Set animation by index for a glTF instance (-1 to disable)
    bool set_instance_animation(const std::string& instanceName, int animationIndex, bool resetTime = true);

    // Set animation by name for a glTF instance
    bool set_instance_animation(const std::string& instanceName, const std::string& animationName, bool resetTime = true);

    // Set animation looping for a glTF instance
    bool set_instance_animation_loop(const std::string& instanceName, bool loop);

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

private:
    VulkanEngine* _engine;
};

} // namespace GameAPI
