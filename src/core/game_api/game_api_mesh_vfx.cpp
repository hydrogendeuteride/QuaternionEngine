#include "core/game_api.h"
#include "core/engine.h"
#include "core/assets/manager.h"
#include "scene/vk_scene.h"

namespace GameAPI
{

bool Engine::create_or_update_mesh_vfx_material(const std::string &materialName,
                                                const MeshVfxMaterialSettings &settings)
{
    if (!_engine || !_engine->_assetManager)
    {
        return false;
    }

    AssetManager::MeshVfxMaterialSettings s{};
    s.tint = settings.tint;
    s.opacity = settings.opacity;
    s.fresnelPower = settings.fresnelPower;
    s.fresnelStrength = settings.fresnelStrength;
    s.albedoPath = settings.albedoPath;
    s.albedoSRGB = settings.albedoSRGB;
    s.noise1Path = settings.noise1Path;
    s.noise2Path = settings.noise2Path;
    s.noise1SRGB = settings.noise1SRGB;
    s.noise2SRGB = settings.noise2SRGB;
    s.scrollVelocity1 = settings.scrollVelocity1;
    s.scrollVelocity2 = settings.scrollVelocity2;
    s.distortionStrength = settings.distortionStrength;
    s.noiseBlend = settings.noiseBlend;
    s.coreColor = settings.coreColor;
    s.edgeColor = settings.edgeColor;
    s.gradientAxis = settings.gradientAxis;
    s.gradientStart = settings.gradientStart;
    s.gradientEnd = settings.gradientEnd;
    s.emissionStrength = settings.emissionStrength;
    return _engine->_assetManager->createOrUpdateMeshVfxMaterial(materialName, s);
}

bool Engine::get_mesh_vfx_material(const std::string &materialName,
                                   MeshVfxMaterialSettings &out) const
{
    if (!_engine || !_engine->_assetManager)
    {
        return false;
    }

    AssetManager::MeshVfxMaterialSettings s{};
    if (!_engine->_assetManager->getMeshVfxMaterialSettings(materialName, s))
    {
        return false;
    }
    out.tint = s.tint;
    out.opacity = s.opacity;
    out.fresnelPower = s.fresnelPower;
    out.fresnelStrength = s.fresnelStrength;
    out.albedoPath = s.albedoPath;
    out.albedoSRGB = s.albedoSRGB;
    out.noise1Path = s.noise1Path;
    out.noise2Path = s.noise2Path;
    out.noise1SRGB = s.noise1SRGB;
    out.noise2SRGB = s.noise2SRGB;
    out.scrollVelocity1 = s.scrollVelocity1;
    out.scrollVelocity2 = s.scrollVelocity2;
    out.distortionStrength = s.distortionStrength;
    out.noiseBlend = s.noiseBlend;
    out.coreColor = s.coreColor;
    out.edgeColor = s.edgeColor;
    out.gradientAxis = s.gradientAxis;
    out.gradientStart = s.gradientStart;
    out.gradientEnd = s.gradientEnd;
    out.emissionStrength = s.emissionStrength;
    return true;
}

bool Engine::remove_mesh_vfx_material(const std::string &materialName)
{
    if (!_engine || !_engine->_assetManager)
    {
        return false;
    }
    return _engine->_assetManager->removeMeshVfxMaterial(materialName);
}

bool Engine::apply_mesh_vfx_material_to_primitive(const std::string &primitiveName,
                                                  const std::string &materialName)
{
    if (!_engine || !_engine->_assetManager || !_engine->_sceneManager)
    {
        return false;
    }
    auto material = _engine->_assetManager->getMeshVfxMaterial(materialName);
    if (!material)
    {
        return false;
    }
    return _engine->_sceneManager->setMeshInstanceMaterial(primitiveName, material);
}

} // namespace GameAPI

