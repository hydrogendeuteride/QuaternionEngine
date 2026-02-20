#include "renderpass.h"

#include "passes/background.h"
#include "passes/sun_disk.h"
#include "passes/geometry.h"
#include "passes/decal.h"
#include "passes/imgui_pass.h"
#include "passes/lighting.h"
#include "passes/ssr.h"
#include "passes/clouds.h"
#include "passes/rocket_plume.h"
#include "passes/atmosphere.h"
#include "passes/particles.h"
#include "passes/fxaa.h"
#include "passes/debug_draw.h"
#include "passes/transparent.h"
#include "passes/mesh_vfx.h"
#include "passes/tonemap.h"
#include "passes/auto_exposure.h"
#include "passes/shadow.h"

void RenderPassManager::init(EngineContext *context)
{
    _context = context;

    auto backgroundPass = std::make_unique<BackgroundPass>();
    backgroundPass->init(context);
    addPass(std::move(backgroundPass));

    // Analytic sun disk over background (works in space, independent of atmosphere pass).
    auto sunDiskPass = std::make_unique<SunDiskPass>();
    sunDiskPass->init(context);
    addPass(std::move(sunDiskPass));

    // Shadow map pass comes early in the frame
    auto shadowPass = std::make_unique<ShadowPass>();
    shadowPass->init(context);
    addPass(std::move(shadowPass));

    auto geometryPass = std::make_unique<GeometryPass>();
    geometryPass->init(context);
    addPass(std::move(geometryPass));

    auto decalPass = std::make_unique<DecalPass>();
    decalPass->init(context);
    addPass(std::move(decalPass));

    auto lightingPass = std::make_unique<LightingPass>();
    lightingPass->init(context);
    addPass(std::move(lightingPass));

    // Screen Space Reflections pass (wired between lighting and transparent)
    auto ssrPass = std::make_unique<SSRPass>();
    ssrPass->init(context);
    addPass(std::move(ssrPass));

    // Voxel volumetrics pass (cloud/smoke/flame via voxel density SSBO)
    auto cloudPass = std::make_unique<CloudPass>();
    cloudPass->init(context);
    addPass(std::move(cloudPass));

    // Analytic rocket plume raymarching pass (vacuum-focused emission).
    auto plumePass = std::make_unique<RocketPlumePass>();
    plumePass->init(context);
    addPass(std::move(plumePass));

    // Single-scattering atmosphere post-process (HDR, before transparents/tonemap).
    auto atmospherePass = std::make_unique<AtmospherePass>();
    atmospherePass->init(context);
    addPass(std::move(atmospherePass));

    // GPU particle system (compute update + render)
    auto particlePass = std::make_unique<ParticlePass>();
    particlePass->init(context);
    addPass(std::move(particlePass));

    // Post-process AA (FXAA-like) after tonemapping.
    auto fxaaPass = std::make_unique<FxaaPass>();
    fxaaPass->init(context);
    addPass(std::move(fxaaPass));

    auto debugDrawPass = std::make_unique<DebugDrawPass>();
    debugDrawPass->init(context);
    addPass(std::move(debugDrawPass));

    auto meshVfxPass = std::make_unique<MeshVfxPass>();
    meshVfxPass->init(context);
    addPass(std::move(meshVfxPass));

    auto transparentPass = std::make_unique<TransparentPass>();
    transparentPass->init(context);
    addPass(std::move(transparentPass));

    auto autoExposurePass = std::make_unique<AutoExposurePass>();
    autoExposurePass->init(context);
    addPass(std::move(autoExposurePass));

    auto tonemapPass = std::make_unique<TonemapPass>();
    tonemapPass->init(context);
    addPass(std::move(tonemapPass));
}

void RenderPassManager::cleanup()
{
    for (auto &pass: _passes)
    {
        pass->cleanup();
    }
    if (_imguiPass)
    {
        _imguiPass->cleanup();
    }
    Logger::info("RenderPassManager::cleanup()");
    _passes.clear();
    _imguiPass.reset();
}

void RenderPassManager::addPass(std::unique_ptr<IRenderPass> pass)
{
    _passes.push_back(std::move(pass));
}

void RenderPassManager::setImGuiPass(std::unique_ptr<IRenderPass> imguiPass)
{
    _imguiPass = std::move(imguiPass);
    if (_imguiPass)
    {
        _imguiPass->init(_context);
    }
}

ImGuiPass *RenderPassManager::getImGuiPass()
{
    if (!_imguiPass) return nullptr;
    return dynamic_cast<ImGuiPass *>(_imguiPass.get());
}
