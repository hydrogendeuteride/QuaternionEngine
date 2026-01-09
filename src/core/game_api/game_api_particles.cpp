#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "render/renderpass.h"
#include "render/passes/particles.h"

namespace GameAPI
{

// ----------------------------------------------------------------------------
// Particle Systems
// ----------------------------------------------------------------------------

uint32_t Engine::create_particle_system(uint32_t particle_count)
{
    if (!_engine || !_engine->_renderPassManager) return 0;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return 0;

    return particlePass->create_system(particle_count);
}

bool Engine::destroy_particle_system(uint32_t id)
{
    if (!_engine || !_engine->_renderPassManager) return false;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return false;

    return particlePass->destroy_system(id);
}

bool Engine::resize_particle_system(uint32_t id, uint32_t new_count)
{
    if (!_engine || !_engine->_renderPassManager) return false;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return false;

    return particlePass->resize_system(id, new_count);
}

bool Engine::get_particle_system(uint32_t id, ParticleSystem& out) const
{
    if (!_engine || !_engine->_renderPassManager) return false;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return false;

    const auto& systems = particlePass->systems();
    for (const auto& sys : systems)
    {
        if (sys.id == id)
        {
            out.id = sys.id;
            out.particleCount = sys.count;
            out.enabled = sys.enabled;
            out.reset = sys.reset;
            out.blendMode = static_cast<ParticleBlendMode>(sys.blend);
            out.flipbookTexture = sys.flipbook_texture;
            out.noiseTexture = sys.noise_texture;

            // Copy parameters
            const auto& p = sys.params;
            out.params.emitterPosLocal = p.emitter_pos_local;
            out.params.spawnRadius = p.spawn_radius;
            out.params.emitterDirLocal = p.emitter_dir_local;
            out.params.coneAngleDegrees = p.cone_angle_degrees;
            out.params.minSpeed = p.min_speed;
            out.params.maxSpeed = p.max_speed;
            out.params.minLife = p.min_life;
            out.params.maxLife = p.max_life;
            out.params.minSize = p.min_size;
            out.params.maxSize = p.max_size;
            out.params.drag = p.drag;
            out.params.gravity = p.gravity;
            out.params.color = p.color;
            out.params.softDepthDistance = p.soft_depth_distance;
            out.params.flipbookCols = p.flipbook_cols;
            out.params.flipbookRows = p.flipbook_rows;
            out.params.flipbookFps = p.flipbook_fps;
            out.params.flipbookIntensity = p.flipbook_intensity;
            out.params.noiseScale = p.noise_scale;
            out.params.noiseStrength = p.noise_strength;
            out.params.noiseScroll = p.noise_scroll;

            return true;
        }
    }

    return false;
}

bool Engine::set_particle_system(uint32_t id, const ParticleSystem& system)
{
    if (!_engine || !_engine->_renderPassManager) return false;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return false;

    auto& systems = particlePass->systems();
    for (auto& sys : systems)
    {
        if (sys.id == id)
        {
            sys.enabled = system.enabled;
            sys.reset = system.reset;
            sys.blend = static_cast<ParticlePass::BlendMode>(system.blendMode);
            sys.flipbook_texture = system.flipbookTexture;
            sys.noise_texture = system.noiseTexture;

            // Copy parameters
            auto& p = sys.params;
            p.emitter_pos_local = system.params.emitterPosLocal;
            p.spawn_radius = system.params.spawnRadius;
            p.emitter_dir_local = system.params.emitterDirLocal;
            p.cone_angle_degrees = system.params.coneAngleDegrees;
            p.min_speed = system.params.minSpeed;
            p.max_speed = system.params.maxSpeed;
            p.min_life = system.params.minLife;
            p.max_life = system.params.maxLife;
            p.min_size = system.params.minSize;
            p.max_size = system.params.maxSize;
            p.drag = system.params.drag;
            p.gravity = system.params.gravity;
            p.color = system.params.color;
            p.soft_depth_distance = system.params.softDepthDistance;
            p.flipbook_cols = system.params.flipbookCols;
            p.flipbook_rows = system.params.flipbookRows;
            p.flipbook_fps = system.params.flipbookFps;
            p.flipbook_intensity = system.params.flipbookIntensity;
            p.noise_scale = system.params.noiseScale;
            p.noise_strength = system.params.noiseStrength;
            p.noise_scroll = system.params.noiseScroll;

            // Preload textures if changed
            if (!sys.flipbook_texture.empty())
            {
                particlePass->preload_vfx_texture(sys.flipbook_texture);
            }
            if (!sys.noise_texture.empty())
            {
                particlePass->preload_vfx_texture(sys.noise_texture);
            }

            return true;
        }
    }

    return false;
}

std::vector<uint32_t> Engine::get_particle_system_ids() const
{
    std::vector<uint32_t> ids;

    if (!_engine || !_engine->_renderPassManager) return ids;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return ids;

    const auto& systems = particlePass->systems();
    ids.reserve(systems.size());
    for (const auto& sys : systems)
    {
        ids.push_back(sys.id);
    }

    return ids;
}

uint32_t Engine::get_allocated_particles() const
{
    if (!_engine || !_engine->_renderPassManager) return 0;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return 0;

    return particlePass->allocated_particles();
}

uint32_t Engine::get_free_particles() const
{
    if (!_engine || !_engine->_renderPassManager) return 0;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return 0;

    return particlePass->free_particles();
}

uint32_t Engine::get_max_particles() const
{
    return ParticlePass::k_max_particles;
}

void Engine::preload_particle_texture(const std::string& assetPath)
{
    if (!_engine || !_engine->_renderPassManager) return;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return;

    particlePass->preload_vfx_texture(assetPath);
}

} // namespace GameAPI
