#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "core/pipeline/manager.h"
#include "render/renderpass.h"
#include "render/passes/tonemap.h"
#include "render/passes/fxaa.h"

namespace GameAPI
{

// ----------------------------------------------------------------------------
// Post Processing - FXAA
// ----------------------------------------------------------------------------

void Engine::set_fxaa_enabled(bool enabled)
{
    if (!_engine->_renderPassManager) return;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        fxaa->set_enabled(enabled);
    }
}

bool Engine::get_fxaa_enabled() const
{
    if (!_engine->_renderPassManager) return false;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        return fxaa->enabled();
    }
    return false;
}

void Engine::set_fxaa_edge_threshold(float threshold)
{
    if (!_engine->_renderPassManager) return;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        fxaa->set_edge_threshold(threshold);
    }
}

float Engine::get_fxaa_edge_threshold() const
{
    if (!_engine->_renderPassManager) return 0.125f;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        return fxaa->edge_threshold();
    }
    return 0.125f;
}

void Engine::set_fxaa_edge_threshold_min(float threshold)
{
    if (!_engine->_renderPassManager) return;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        fxaa->set_edge_threshold_min(threshold);
    }
}

float Engine::get_fxaa_edge_threshold_min() const
{
    if (!_engine->_renderPassManager) return 0.0312f;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        return fxaa->edge_threshold_min();
    }
    return 0.0312f;
}

// ----------------------------------------------------------------------------
// Post Processing - SSR
// ----------------------------------------------------------------------------

void Engine::set_ssr_enabled(bool enabled)
{
    if (_engine->_context)
    {
        _engine->_context->enableSSR = enabled;
    }
}

bool Engine::get_ssr_enabled() const
{
    return _engine->_context ? _engine->_context->enableSSR : false;
}

void Engine::set_reflection_mode(ReflectionMode mode)
{
    if (_engine->_context)
    {
        // Guard against requesting RT reflection modes on unsupported hardware.
        if (mode != ReflectionMode::SSROnly)
        {
            if (!_engine->_deviceManager
                || !_engine->_deviceManager->supportsRayQuery()
                || !_engine->_deviceManager->supportsAccelerationStructure())
            {
                mode = ReflectionMode::SSROnly;
            }
        }

        _engine->_context->reflectionMode = static_cast<uint32_t>(mode);
    }
}

ReflectionMode Engine::get_reflection_mode() const
{
    if (!_engine->_context) return ReflectionMode::SSROnly;
    return static_cast<ReflectionMode>(_engine->_context->reflectionMode);
}

// ----------------------------------------------------------------------------
// Post Processing - Tonemapping
// ----------------------------------------------------------------------------

void Engine::set_exposure(float exposure)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setExposure(exposure);
    }
}

float Engine::get_exposure() const
{
    if (!_engine->_renderPassManager) return 1.0f;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return tonemap->exposure();
    }
    return 1.0f;
}

void Engine::set_tonemap_operator(TonemapOperator op)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setMode(static_cast<int>(op));
    }
}

TonemapOperator Engine::get_tonemap_operator() const
{
    if (!_engine->_renderPassManager) return TonemapOperator::ACES;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return static_cast<TonemapOperator>(tonemap->mode());
    }
    return TonemapOperator::ACES;
}

// ----------------------------------------------------------------------------
// Post Processing - Bloom
// ----------------------------------------------------------------------------

void Engine::set_bloom_enabled(bool enabled)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setBloomEnabled(enabled);
    }
}

bool Engine::get_bloom_enabled() const
{
    if (!_engine->_renderPassManager) return false;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return tonemap->bloomEnabled();
    }
    return false;
}

void Engine::set_bloom_threshold(float threshold)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setBloomThreshold(threshold);
    }
}

float Engine::get_bloom_threshold() const
{
    if (!_engine->_renderPassManager) return 1.0f;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return tonemap->bloomThreshold();
    }
    return 1.0f;
}

void Engine::set_bloom_intensity(float intensity)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setBloomIntensity(intensity);
    }
}

float Engine::get_bloom_intensity() const
{
    if (!_engine->_renderPassManager) return 0.7f;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return tonemap->bloomIntensity();
    }
    return 0.7f;
}

// ----------------------------------------------------------------------------
// Rendering
// ----------------------------------------------------------------------------

void Engine::set_render_scale(float scale)
{
    _engine->renderScale = glm::clamp(scale, 0.3f, 1.0f);
}

float Engine::get_render_scale() const
{
    return _engine->renderScale;
}

void Engine::set_pass_enabled(const std::string& passName, bool enabled)
{
    _engine->_rgPassToggles[passName] = enabled;
}

bool Engine::get_pass_enabled(const std::string& passName) const
{
    auto it = _engine->_rgPassToggles.find(passName);
    if (it != _engine->_rgPassToggles.end())
    {
        return it->second;
    }
    return true; // Default to enabled if not in map
}

void Engine::hot_reload_shaders()
{
    if (_engine->_pipelineManager)
    {
        _engine->_pipelineManager->hotReloadChanged();
    }
}

// ----------------------------------------------------------------------------
// Time
// ----------------------------------------------------------------------------

float Engine::get_delta_time() const
{
    if (_engine->_sceneManager)
    {
        return _engine->_sceneManager->getDeltaTime();
    }
    return 0.0f;
}

// ----------------------------------------------------------------------------
// Statistics
// ----------------------------------------------------------------------------

Stats Engine::get_stats() const
{
    Stats s;
    s.frametime = _engine->stats.frametime;
    s.drawTime = _engine->stats.mesh_draw_time;
    s.sceneUpdateTime = _engine->stats.scene_update_time;
    s.triangleCount = _engine->stats.triangle_count;
    s.drawCallCount = _engine->stats.drawcall_count;
    return s;
}

} // namespace GameAPI
