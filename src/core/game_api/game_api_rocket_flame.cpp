#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"

namespace GameAPI
{

namespace
{
    static ::RocketFlameQuality to_internal_quality(RocketFlameQuality quality)
    {
        switch (quality)
        {
            case RocketFlameQuality::Low: return ::RocketFlameQuality::Low;
            case RocketFlameQuality::Balanced: return ::RocketFlameQuality::Balanced;
            case RocketFlameQuality::High: return ::RocketFlameQuality::High;
        }
        return ::RocketFlameQuality::Balanced;
    }

    static RocketFlameQuality from_internal_quality(::RocketFlameQuality quality)
    {
        switch (quality)
        {
            case ::RocketFlameQuality::Low: return RocketFlameQuality::Low;
            case ::RocketFlameQuality::Balanced: return RocketFlameQuality::Balanced;
            case ::RocketFlameQuality::High: return RocketFlameQuality::High;
        }
        return RocketFlameQuality::Balanced;
    }

    static void to_internal_settings(const RocketFlameSettings &src, ::RocketFlameSettings &dst)
    {
        dst.length = src.length;
        dst.radiusBase = src.radiusBase;
        dst.expansionRate = src.expansionRate;
        dst.axialFalloff = src.axialFalloff;
        dst.edgeSoftness = src.edgeSoftness;
        dst.coreRadiusFactor = src.coreRadiusFactor;
        dst.coreIntensity = src.coreIntensity;
        dst.outerIntensity = src.outerIntensity;
        dst.coreColor = src.coreColor;
        dst.outerColor = src.outerColor;
        dst.turbulenceStrength = src.turbulenceStrength;
        dst.turbulenceScale = src.turbulenceScale;
        dst.flickerSpeed = src.flickerSpeed;
    }

    static void from_internal_settings(const ::RocketFlameSettings &src, RocketFlameSettings &dst)
    {
        dst.length = src.length;
        dst.radiusBase = src.radiusBase;
        dst.expansionRate = src.expansionRate;
        dst.axialFalloff = src.axialFalloff;
        dst.edgeSoftness = src.edgeSoftness;
        dst.coreRadiusFactor = src.coreRadiusFactor;
        dst.coreIntensity = src.coreIntensity;
        dst.outerIntensity = src.outerIntensity;
        dst.coreColor = src.coreColor;
        dst.outerColor = src.outerColor;
        dst.turbulenceStrength = src.turbulenceStrength;
        dst.turbulenceScale = src.turbulenceScale;
        dst.flickerSpeed = src.flickerSpeed;
    }

    static ::RocketFlameInstance *find_slot(EngineContext *ctx, uint32_t id)
    {
        if (!ctx || id == 0) return nullptr;
        for (auto &slot : ctx->rocketFlames)
        {
            if (slot.active && slot.id == id)
            {
                return &slot;
            }
        }
        return nullptr;
    }

    static const ::RocketFlameInstance *find_slot(const EngineContext *ctx, uint32_t id)
    {
        if (!ctx || id == 0) return nullptr;
        for (const auto &slot : ctx->rocketFlames)
        {
            if (slot.active && slot.id == id)
            {
                return &slot;
            }
        }
        return nullptr;
    }
}

void Engine::set_rocket_flames_enabled(bool enabled)
{
    if (!_engine || !_engine->_context) return;
    _engine->_context->enableRocketFlames = enabled;
}

bool Engine::get_rocket_flames_enabled() const
{
    if (!_engine || !_engine->_context) return false;
    return _engine->_context->enableRocketFlames;
}

void Engine::set_rocket_flame_quality(RocketFlameQuality quality)
{
    if (!_engine || !_engine->_context) return;
    _engine->_context->rocketFlameQuality = to_internal_quality(quality);
}

RocketFlameQuality Engine::get_rocket_flame_quality() const
{
    if (!_engine || !_engine->_context) return RocketFlameQuality::Balanced;
    return from_internal_quality(_engine->_context->rocketFlameQuality);
}

uint32_t Engine::create_rocket_flame()
{
    if (!_engine || !_engine->_context) return 0;
    EngineContext *ctx = _engine->_context.get();

    for (auto &slot : ctx->rocketFlames)
    {
        if (!slot.active)
        {
            const uint32_t id = ctx->nextRocketFlameId++;
            if (ctx->nextRocketFlameId == 0)
            {
                ctx->nextRocketFlameId = 1;
            }

            slot = {};
            slot.active = true;
            slot.id = (id == 0) ? 1u : id;
            slot.enabled = true;
            slot.directionLocal = glm::vec3(0.0f, -1.0f, 0.0f);
            slot.throttle = 1.0f;
            slot.settings = {};
            return slot.id;
        }
    }

    return 0;
}

bool Engine::destroy_rocket_flame(uint32_t id)
{
    if (!_engine || !_engine->_context || id == 0) return false;
    EngineContext *ctx = _engine->_context.get();

    if (::RocketFlameInstance *slot = find_slot(ctx, id))
    {
        *slot = {};
        return true;
    }
    return false;
}

bool Engine::get_rocket_flame(uint32_t id, RocketFlameInstance &out) const
{
    if (!_engine || !_engine->_context || id == 0) return false;

    const EngineContext *ctx = _engine->_context.get();
    const ::RocketFlameInstance *slot = find_slot(ctx, id);
    if (!slot) return false;

    out = {};
    out.id = slot->id;
    out.enabled = slot->enabled;
    out.positionLocal = slot->positionLocal;
    out.directionLocal = slot->directionLocal;
    out.throttle = slot->throttle;
    from_internal_settings(slot->settings, out.settings);
    return true;
}

bool Engine::set_rocket_flame(uint32_t id, const RocketFlameInstance &instance)
{
    if (!_engine || !_engine->_context || id == 0) return false;
    EngineContext *ctx = _engine->_context.get();

    ::RocketFlameInstance *slot = find_slot(ctx, id);
    if (!slot) return false;

    slot->enabled = instance.enabled;
    slot->positionLocal = instance.positionLocal;
    slot->directionLocal = instance.directionLocal;
    slot->throttle = instance.throttle;
    to_internal_settings(instance.settings, slot->settings);
    return true;
}

std::vector<uint32_t> Engine::get_rocket_flame_ids() const
{
    std::vector<uint32_t> ids;
    if (!_engine || !_engine->_context) return ids;

    const EngineContext *ctx = _engine->_context.get();
    ids.reserve(ctx->rocketFlames.size());
    for (const auto &slot : ctx->rocketFlames)
    {
        if (slot.active)
        {
            ids.push_back(slot.id);
        }
    }
    return ids;
}

size_t Engine::get_max_rocket_flames() const
{
    return EngineContext::MAX_ROCKET_FLAMES;
}

} // namespace GameAPI
