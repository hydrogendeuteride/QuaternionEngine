#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"

namespace GameAPI
{

// ----------------------------------------------------------------------------
// Rocket Plumes (analytic raymarch)
// ----------------------------------------------------------------------------

void Engine::set_rocket_plumes_enabled(bool enabled)
{
    if (!_engine || !_engine->_context) return;
    _engine->_context->enableRocketPlumes = enabled;
}

bool Engine::get_rocket_plumes_enabled() const
{
    if (!_engine || !_engine->_context) return false;
    return _engine->_context->enableRocketPlumes;
}

bool Engine::get_rocket_plume(size_t index, RocketPlumeSettings& out) const
{
    if (!_engine || !_engine->_context) return false;
    if (index >= EngineContext::MAX_ROCKET_PLUMES) return false;

    const auto& src = _engine->_context->rocketPlumes[index];

    out.enabled = src.enabled;
    out.worldToPlume = src.worldToPlume;
    out.length = src.length;
    out.nozzleRadius = src.nozzleRadius;
    out.expansionAngleRad = src.expansionAngleRad;
    out.radiusExp = src.radiusExp;
    out.intensity = src.intensity;
    out.coreColor = src.coreColor;
    out.plumeColor = src.plumeColor;
    out.coreLength = src.coreLength;
    out.coreStrength = src.coreStrength;
    out.radialFalloff = src.radialFalloff;
    out.axialFalloff = src.axialFalloff;
    out.noiseStrength = src.noiseStrength;
    out.noiseScale = src.noiseScale;
    out.noiseSpeed = src.noiseSpeed;
    out.shockStrength = src.shockStrength;
    out.shockFrequency = src.shockFrequency;
    out.softAbsorption = src.softAbsorption;

    return true;
}

bool Engine::set_rocket_plume(size_t index, const RocketPlumeSettings& settings)
{
    if (!_engine || !_engine->_context) return false;
    if (index >= EngineContext::MAX_ROCKET_PLUMES) return false;

    auto& dst = _engine->_context->rocketPlumes[index];

    dst.enabled = settings.enabled;
    dst.worldToPlume = settings.worldToPlume;
    dst.length = settings.length;
    dst.nozzleRadius = settings.nozzleRadius;
    dst.expansionAngleRad = settings.expansionAngleRad;
    dst.radiusExp = settings.radiusExp;
    dst.intensity = settings.intensity;
    dst.coreColor = settings.coreColor;
    dst.plumeColor = settings.plumeColor;
    dst.coreLength = settings.coreLength;
    dst.coreStrength = settings.coreStrength;
    dst.radialFalloff = settings.radialFalloff;
    dst.axialFalloff = settings.axialFalloff;
    dst.noiseStrength = settings.noiseStrength;
    dst.noiseScale = settings.noiseScale;
    dst.noiseSpeed = settings.noiseSpeed;
    dst.shockStrength = settings.shockStrength;
    dst.shockFrequency = settings.shockFrequency;
    dst.softAbsorption = settings.softAbsorption;

    return true;
}

size_t Engine::get_max_rocket_plumes() const
{
    return EngineContext::MAX_ROCKET_PLUMES;
}

void Engine::set_rocket_plume_noise_texture_path(const std::string& assetPath)
{
    if (!_engine || !_engine->_context) return;
    _engine->_context->rocketPlumeNoiseTexturePath = assetPath;
}

std::string Engine::get_rocket_plume_noise_texture_path() const
{
    if (!_engine || !_engine->_context) return {};
    return _engine->_context->rocketPlumeNoiseTexturePath;
}

} // namespace GameAPI
