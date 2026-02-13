#pragma once

#include <array>
#include <memory>
#include <string>
#include <core/types.h>
#include <core/world.h>
#include <core/descriptor/descriptors.h>
#include <core/config.h>
// Avoid including vk_scene.h here to prevent cycles

struct EngineStats
{
    float frametime;
    int triangle_count;
    int drawcall_count;
    float scene_update_time;
    float mesh_draw_time;
};

class DeviceManager;
class ResourceManager;
class SwapchainManager;
class DescriptorManager;
class SamplerManager;
class SceneManager;
class MeshAsset;
struct DrawContext;
struct GPUSceneData;
class ComputeManager;
class PipelineManager;
struct FrameResources;
struct SDL_Window;
class AssetManager;
class RenderGraph;
class RayTracingManager;
class TextureCache;
class IBLManager;
class InputSystem;
class DebugDrawSystem;
namespace Physics {
    class PhysicsContext;
}

struct ShadowSettings
{
    // 0 = Clipmap only, 1 = Clipmap + RT assist, 2 = RT only
    // Default to non-RT for broader driver compatibility; RT can be enabled at runtime.
    uint32_t mode = 0;
    // Global enable/disable for all shadowing (raster + RT).
    bool enabled = true;
    // Shadow brightness control: minimum visibility applied to sun shadows (0..1).
    // Raises the darkest parts of CSM / RT / planet shadows so fully-shadowed areas
    // are not pitch black. 0 = original behavior (full dark), 0.2 = keep 20% light.
    float shadowMinVisibility = 0.0f;
    // Shadow map resolution (square) used for raster clipmap cascades and stabilization (texel snapping).
    // Changing this at runtime is supported; it affects transient RenderGraph images and light-space snapping.
    uint32_t shadowMapResolution = kShadowMapResolution;
    bool hybridRayQueryEnabled = false;   // derived convenience: (mode != 0)
    uint32_t hybridRayCascadesMask = 0b1110; // bit i => cascade i uses ray query assist (default: 1..3)
    float hybridRayNoLThreshold = 0.25f;  // trigger when N·L below this (mode==1)

    // Analytic planet shadow penumbra control for the directional sun.
    // This is treated as the sun's angular radius (half-angle) in degrees.
    // Set to 0 for a hard edge.
    float planetSunAngularRadiusDeg = 0.27f;
};

struct CloudSettings
{
    // DEPRECATED: use EngineContext::voxelVolumes[0] instead.
    // Kept temporarily to avoid breaking downstream code during refactors.
    bool followCameraXZ = true;
    bool animateVoxels = false;
    glm::vec3 volumeCenterLocal{0.0f, 80.0f, 0.0f};
    glm::vec3 volumeHalfExtents{128.0f, 20.0f, 128.0f};
    glm::vec3 volumeVelocityLocal{0.0f, 0.0f, 0.0f};
    float densityScale = 1.25f;
    float coverage = 0.45f;
    int stepCount = 64;
    uint32_t gridResolution = 64;
    glm::vec3 windVelocityLocal{6.0f, 0.0f, 2.0f};
    float dissipation = 0.35f;
    float noiseStrength = 1.0f;
    float noiseScale = 6.0f;
    float noiseSpeed = 0.15f;
};

enum class VoxelVolumeType : uint32_t
{
    Clouds = 0,
    Smoke = 1,
    Flame = 2,
};

struct VoxelVolumeSettings
{
    bool enabled = false;
    VoxelVolumeType type = VoxelVolumeType::Clouds;

    // If true, the volume is anchored to camera XZ and volumeCenterLocal is treated as an offset.
    // If false, volumeCenterLocal is absolute render-local space and will be compensated for floating-origin shifts.
    bool followCameraXZ = false;
    // If true, run a lightweight voxel advection/update compute pass every frame (procedural motion).
    bool animateVoxels = true;

    // Volume AABB in render-local space.
    glm::vec3 volumeCenterLocal{0.0f, 2.0f, 0.0f};
    glm::vec3 volumeHalfExtents{8.0f, 8.0f, 8.0f};
    // Optional volume drift (applied only when followCameraXZ == false).
    glm::vec3 volumeVelocityLocal{0.0f, 0.0f, 0.0f};

    // Raymarch/composite controls.
    float densityScale = 1.0f;
    float coverage = 0.0f;        // 0..1 threshold (higher = emptier)
    float extinction = 1.0f;      // absorption/extinction scale
    int stepCount = 48;           // raymarch steps

    // Voxel grid resolution (cubic).
    uint32_t gridResolution = 48;

    // Voxel animation (advection + injection) parameters.
    glm::vec3 windVelocityLocal{0.0f, 2.0f, 0.0f}; // local units/sec (add buoyancy here)
    float dissipation = 1.25f;    // density decay rate (1/sec)
    float noiseStrength = 1.0f;   // injection rate
    float noiseScale = 8.0f;      // noise frequency in UVW space
    float noiseSpeed = 1.0f;      // time scale for injection noise

    // Smoke/flame source in normalized volume UVW space.
    glm::vec3 emitterUVW{0.5f, 0.05f, 0.5f};
    float emitterRadius = 0.18f;  // normalized (0..1-ish)

    // Shading.
    glm::vec3 albedo{1.0f, 1.0f, 1.0f}; // scattering tint (cloud/smoke)
    float scatterStrength = 1.0f;
    glm::vec3 emissionColor{1.0f, 0.6f, 0.25f}; // flame emissive tint
    float emissionStrength = 0.0f;
};

struct AtmosphereSettings
{
    // If non-empty, selects the named PlanetSystem body for atmosphere rendering.
    // If empty, the renderer picks the closest visible planet to the camera.
    std::string bodyName{};

    // Atmosphere height above planet radius (meters).
    float atmosphereHeightM = 80000.0f;

    // Scale heights (meters) for exponential density.
    float rayleighScaleHeightM = 8000.0f;
    float mieScaleHeightM = 1200.0f;

    // Scattering coefficients (1/m). Earth-ish defaults.
    glm::vec3 rayleighScattering = glm::vec3(5.802e-6f, 13.558e-6f, 33.1e-6f);
    glm::vec3 mieScattering = glm::vec3(21.0e-6f);

    // Colored absorption/extinction coefficient (1/m).
    // Final absorption used by the shader: betaA = absorptionColor * absorptionStrength.
    // - absorptionColor: per-channel weighting (0..1, linear RGB)
    // - absorptionStrength: overall strength (1/m); 0 disables absorption
    glm::vec3 absorptionColor = glm::vec3(1.0f);
    float absorptionStrength = 0.0f;

    // Henyey-Greenstein phase g (forward scattering).
    float mieG = 0.76f;

    // Artistic controls.
    float intensity = 1.0f;
    float sunDiskIntensity = 1.0f;

    // Sun glare controls (applied by SunDiskPass; independent of atmosphere scattering).
    // Set intensities to 0 to disable the respective component.
    float sunHaloIntensity = 0.0f;
    float sunHaloRadiusDeg = 2.0f;
    float sunStarburstIntensity = 0.0f;
    float sunStarburstRadiusDeg = 6.0f;
    int sunStarburstSpikes = 8;       // number of rays (recommended even)
    float sunStarburstSharpness = 12.0f;

    // Sampling jitter (0 = off; 1 = full per-pixel jitter). Disabling reduces noise but may introduce banding.
    float jitterStrength = 0.0f;

    // When clamping the raymarch to the visible surface, snap planet pixels to the analytic planet sphere
    // if the GBuffer surface is within this distance (meters) to reduce cube-sphere patch faceting artifacts.
    // Set to 0 to disable snapping.
    float planetSurfaceSnapM = 200.0f;

    // Integration quality/performance tradeoff.
    int viewSteps = 16;
    int lightSteps = 8;
};

struct PlanetCloudSettings
{
    // Base height above planet radius (meters).
    float baseHeightM = 2000.0f;

    // Thickness of the cloud layer (meters).
    float thicknessM = 8000.0f;

    // Density scale (unitless). Higher = thicker clouds.
    float densityScale = 1.0f;

    // Coverage threshold (0..1). Higher = emptier.
    float coverage = 0.45f;

    // Relative to assets/ (KTX2 recommended). If empty or missing, the cloud layer is disabled.
    std::string overlayTexturePath = "planets/earth/cloud/earth_clouds_4k.ktx2";

    // Rotate the overlay around +Y (radians) to align the texture seam/orientation.
    float overlayRotationRad = 0.0f;

    // Flip the overlay V coordinate (useful if the source texture is upside-down).
    bool overlayFlipV = false;

    // Procedural noise controls.
    float noiseScale = 1.5f;
    float detailScale = 12.0f;

    // Wind speed along cloud layer (m/s).
    float windSpeed = 0.0f;
    float windAngleRad = 0.0f;

    // Cloud raymarch steps inside the shell.
    int cloudSteps = 32;
};

class EngineContext
{
public:
    static constexpr uint32_t MAX_VOXEL_VOLUMES = 4;

    // Owned shared resources
    std::shared_ptr<DeviceManager> device;
    std::shared_ptr<ResourceManager> resources;
    std::shared_ptr<DescriptorAllocatorGrowable> descriptors;

    // Non-owning pointers to global managers owned by VulkanEngine
    SwapchainManager* swapchain = nullptr;
    DescriptorManager* descriptorLayouts = nullptr;
    SamplerManager* samplers = nullptr;
    SceneManager* scene = nullptr;

    // Per-frame and subsystem pointers for modules to use without VulkanEngine
    FrameResources* currentFrame = nullptr;      // set by engine each frame
    uint32_t frameIndex = 0;                     // incremented by engine each frame
    EngineStats* stats = nullptr;                // points to engine stats
    ComputeManager* compute = nullptr;           // compute subsystem
    PipelineManager* pipelines = nullptr;        // graphics pipeline manager
    RenderGraph* renderGraph = nullptr;          // render graph (built per-frame)
    SDL_Window* window = nullptr;                // SDL window handle
    InputSystem* input = nullptr;                // input system (engine-owned)
    DebugDrawSystem* debug_draw = nullptr;       // debug 3D draw collector (engine-owned)
    Physics::PhysicsContext* physics_context = nullptr;  // optional physics coordinate context (game-owned)

    // Frequently used values
    VkExtent2D drawExtent{};
    VkExtent2D logicalRenderExtent{};

    // ========================================================================
    // Render floating origin (authoritative, double precision)
    // ========================================================================
    WorldVec3 origin_world{0.0, 0.0, 0.0};
    uint64_t origin_revision{0};

    bool set_origin_world(const WorldVec3 &new_origin)
    {
        if (is_zero(new_origin - origin_world))
        {
            return false;
        }
        origin_world = new_origin;
        ++origin_revision;
        return true;
    }

    // Optional convenience content pointers (moved to AssetManager for meshes)

    // Assets
    AssetManager* assets = nullptr;              // non-owning pointer to central AssetManager

    // Runtime settings visible to passes/shaders
    ShadowSettings shadowSettings{};
    bool enableSSR = false;                      // optional screen-space reflections toggle
    // Reflection mode for SSR/RT reflections; encoded into sceneData.rtOptions.w
    // 0 = SSR only, 1 = SSR + RT fallback, 2 = RT only
    uint32_t reflectionMode = 0;

    bool enableClouds = false;                   // DEPRECATED: use enableVolumetrics + voxelVolumes instead.
    CloudSettings cloudSettings{};               // DEPRECATED: kept for compatibility during refactors.

    bool enableVolumetrics = false;              // optional voxel volumetrics toggle (cloud/smoke/flame)
    std::array<VoxelVolumeSettings, MAX_VOXEL_VOLUMES> voxelVolumes{};

    bool enableAtmosphere = false;               // optional atmosphere scattering toggle
    AtmosphereSettings atmosphere{};

    bool enablePlanetClouds = false;             // optional planet cloud layer toggle
    PlanetCloudSettings planetClouds{};

    // Ray tracing manager (optional, nullptr if unsupported)
    RayTracingManager* ray = nullptr;

    // Accessors
    DeviceManager *getDevice() const { return device.get(); }
    ResourceManager *getResources() const { return resources.get(); }
    DescriptorAllocatorGrowable *getDescriptors() const { return descriptors.get(); }
    SwapchainManager* getSwapchain() const { return swapchain; }
    DescriptorManager* getDescriptorLayouts() const { return descriptorLayouts; }
    SamplerManager* getSamplers() const { return samplers; }
    const GPUSceneData& getSceneData() const;
    const DrawContext& getMainDrawContext() const;
    VkExtent2D getDrawExtent() const { return drawExtent; }
    VkExtent2D getLogicalRenderExtent() const { return logicalRenderExtent; }
    AssetManager* getAssets() const { return assets; }
    // Convenience alias (singular) requested
    AssetManager* getAsset() const { return assets; }
    RenderGraph* getRenderGraph() const { return renderGraph; }

    // Shadow map resolution utilities (clamped to device limits).
    uint32_t getShadowMapResolution() const;
    void setShadowMapResolution(uint32_t resolution);

    // Streaming subsystems (engine-owned)
    TextureCache* textures = nullptr;            // texture streaming + cache
    IBLManager*  ibl = nullptr;                  // optional IBL owner (if created by engine)
};
