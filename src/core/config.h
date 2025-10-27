#pragma once

// Centralized engine configuration flags
#ifdef NDEBUG
inline constexpr bool kUseValidationLayers = false;
#else
inline constexpr bool kUseValidationLayers = true;
#endif

// Shadow mapping configuration
// Shadow system configuration (single source of truth for CPU side)
inline constexpr int   kShadowCascadeCount      = 2;        // total cascades
inline constexpr float kShadowMapResolution     = 2048.0f;  // square map size
inline constexpr float kShadowCSMFar            = 800.0f;   // max view-space distance covered by CSM

// Simple-setup cascade tuning (kept here so Scene/Engine don’t hardcode numbers)
inline constexpr float kShadowLightNear         = 0.2f;     // light-space near plane
inline constexpr float kShadowSplitRatio        = 0.55f;    // split0 = ratio * kShadowCSMFar
inline constexpr float kShadowCascade0XYScale   = 3.2f;     // near cascade orthographic Y scale = tan(fov) * scale; X is scaled by aspect
inline constexpr float kShadowCascade1XYScale   = 9.0f;     // far  cascade orthographic Y scale
inline constexpr float kShadowCascade0LightOffset = 450.0f; // distance from camera along -L
inline constexpr float kShadowCascade1LightOffset = 1200.0f;
inline constexpr float kShadowCascade0Far       = 900.0f;   // light-space far plane
inline constexpr float kShadowCascade1Far       = 2400.0f;

// Rasterization bias for the shadow map pass
inline constexpr float kShadowDepthBiasConstant = 2.0f;
inline constexpr float kShadowDepthBiasSlope    = 2.0f;
inline constexpr float kShadowDepthBiasClamp    = 0.0f;
