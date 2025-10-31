#pragma once

// Centralized engine configuration flags
#ifdef NDEBUG
inline constexpr bool kUseValidationLayers = false;
#else
inline constexpr bool kUseValidationLayers = true;
#endif

// VMA diagnostics (stats prints + JSON dumps + allocation naming)
// - Default: disabled to avoid noise and I/O at shutdown.
// - Enable at runtime by setting environment variable `VE_VMA_DEBUG=1`.
#include <cstdlib>
inline constexpr bool kEnableVmaDebugByDefault = false;
inline bool vmaDebugEnabled()
{
    const char *env = std::getenv("VE_VMA_DEBUG");
    if (env && *env)
    {
        // Accept 1/true/yes (case-insensitive)
        return (*env == '1') || (*env == 'T') || (*env == 't') || (*env == 'Y') || (*env == 'y');
    }
    return kEnableVmaDebugByDefault;
}

// Shadow mapping configuration
inline constexpr int kShadowCascadeCount = 4;
// Maximum shadow distance for CSM in view-space units
inline constexpr float kShadowCSMFar = 800.0f;
// Shadow map resolution used for stabilization (texel snapping). Must match actual image size.
inline constexpr float kShadowMapResolution = 2048.0f;
// Extra XY expansion for cascade footprint (safety against FOV/aspect changes)
inline constexpr float kShadowCascadeRadiusScale = 1.1f;
// Additive XY margin in world units beyond the scaled half-size
inline constexpr float kShadowCascadeRadiusMargin = 10.0f;
// Clipmap shadow configuration (used when cascades operate in clipmap mode)
// Base coverage radius of level 0 around the camera (world units). Each level doubles the radius.
inline constexpr float kShadowClipBaseRadius = 20.0f;
// When using dynamic pullback, compute it from the covered XY range of each level.
// pullback = max(kShadowClipPullbackMin, cover * kShadowClipPullbackFactor)
inline constexpr float kShadowClipPullbackFactor = 1.5f;   // fraction of XY half-size behind center
inline constexpr float kShadowClipForwardFactor  = 1.5f;   // fraction of XY half-size in front of center for zFar
inline constexpr float kShadowClipPullbackMin    = 40.0f;   // lower bound on pullback so near levels don’t collapse
// Additional Z padding for the orthographic frustum along light direction
inline constexpr float kShadowClipZPadding = 40.0f;

// Shadow quality & filtering
// Soft cross-fade band between cascades in light-space NDC (0..1)
inline constexpr float kShadowBorderSmoothNDC = 0.08f;
// Base PCF radius in texels for cascade 0; higher cascades scale up slightly
inline constexpr float kShadowPCFBaseRadius   = 1.35f;
// Additional radius added by the farthest cascade (0..+)
inline constexpr float kShadowPCFCascadeGain  = 2.0f;

// Raster depth-bias parameters for shadow map rendering (tuned conservatively)
inline constexpr float kShadowDepthBiasConstant = 1.25f;
inline constexpr float kShadowDepthBiasSlope    = 1.5f;
