#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"

namespace GameAPI
{

// ----------------------------------------------------------------------------
// Volumetrics (Cloud/Smoke/Flame)
// ----------------------------------------------------------------------------

void Engine::set_volumetrics_enabled(bool enabled)
{
    if (!_engine || !_engine->_context) return;
    _engine->_context->enableVolumetrics = enabled;
}

bool Engine::get_volumetrics_enabled() const
{
    if (!_engine || !_engine->_context) return false;
    return _engine->_context->enableVolumetrics;
}

bool Engine::get_voxel_volume(size_t index, VoxelVolumeSettings& out) const
{
    if (!_engine || !_engine->_context) return false;
    if (index >= EngineContext::MAX_VOXEL_VOLUMES) return false;

    const auto& src = _engine->_context->voxelVolumes[index];

    out.enabled = src.enabled;
    out.type = static_cast<VoxelVolumeType>(src.type);
    out.followCameraXZ = src.followCameraXZ;
    out.animateVoxels = src.animateVoxels;
    out.volumeCenterLocal = src.volumeCenterLocal;
    out.volumeHalfExtents = src.volumeHalfExtents;
    out.volumeVelocityLocal = src.volumeVelocityLocal;
    out.densityScale = src.densityScale;
    out.coverage = src.coverage;
    out.extinction = src.extinction;
    out.stepCount = src.stepCount;
    out.gridResolution = src.gridResolution;
    out.windVelocityLocal = src.windVelocityLocal;
    out.dissipation = src.dissipation;
    out.noiseStrength = src.noiseStrength;
    out.noiseScale = src.noiseScale;
    out.noiseSpeed = src.noiseSpeed;
    out.emitterUVW = src.emitterUVW;
    out.emitterRadius = src.emitterRadius;
    out.albedo = src.albedo;
    out.scatterStrength = src.scatterStrength;
    out.emissionColor = src.emissionColor;
    out.emissionStrength = src.emissionStrength;

    return true;
}

bool Engine::set_voxel_volume(size_t index, const VoxelVolumeSettings& settings)
{
    if (!_engine || !_engine->_context) return false;
    if (index >= EngineContext::MAX_VOXEL_VOLUMES) return false;

    auto& dst = _engine->_context->voxelVolumes[index];

    dst.enabled = settings.enabled;
    dst.type = static_cast<::VoxelVolumeType>(settings.type);
    dst.followCameraXZ = settings.followCameraXZ;
    dst.animateVoxels = settings.animateVoxels;
    dst.volumeCenterLocal = settings.volumeCenterLocal;
    dst.volumeHalfExtents = settings.volumeHalfExtents;
    dst.volumeVelocityLocal = settings.volumeVelocityLocal;
    dst.densityScale = settings.densityScale;
    dst.coverage = settings.coverage;
    dst.extinction = settings.extinction;
    dst.stepCount = settings.stepCount;
    dst.gridResolution = settings.gridResolution;
    dst.windVelocityLocal = settings.windVelocityLocal;
    dst.dissipation = settings.dissipation;
    dst.noiseStrength = settings.noiseStrength;
    dst.noiseScale = settings.noiseScale;
    dst.noiseSpeed = settings.noiseSpeed;
    dst.emitterUVW = settings.emitterUVW;
    dst.emitterRadius = settings.emitterRadius;
    dst.albedo = settings.albedo;
    dst.scatterStrength = settings.scatterStrength;
    dst.emissionColor = settings.emissionColor;
    dst.emissionStrength = settings.emissionStrength;

    return true;
}

size_t Engine::get_max_voxel_volumes() const
{
    return EngineContext::MAX_VOXEL_VOLUMES;
}

} // namespace GameAPI
