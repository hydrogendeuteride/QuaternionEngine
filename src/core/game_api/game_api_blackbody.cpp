#include "core/game_api.h"

#include "core/engine.h"
#include "core/assets/manager.h"
#include "scene/vk_scene.h"

namespace GameAPI
{

namespace
{
    static void to_internal(const BlackbodySettings &src, AssetManager::BlackbodySettings &dst)
    {
        dst.noisePath = src.noisePath;
        dst.intensity = src.intensity;
        dst.tempMinK = src.tempMinK;
        dst.tempMaxK = src.tempMaxK;
        dst.noiseScale = src.noiseScale;
        dst.noiseContrast = src.noiseContrast;
        dst.noiseScroll = src.noiseScroll;
        dst.noiseSpeed = src.noiseSpeed;
        dst.heatAxisLocal = src.heatAxisLocal;
        dst.hotEndBias = src.hotEndBias;
        dst.hotRangeStart = src.hotRangeStart;
        dst.hotRangeEnd = src.hotRangeEnd;
    }

    static void from_internal(const AssetManager::BlackbodySettings &src, BlackbodySettings &dst)
    {
        dst.noisePath = src.noisePath;
        dst.intensity = src.intensity;
        dst.tempMinK = src.tempMinK;
        dst.tempMaxK = src.tempMaxK;
        dst.noiseScale = src.noiseScale;
        dst.noiseContrast = src.noiseContrast;
        dst.noiseScroll = src.noiseScroll;
        dst.noiseSpeed = src.noiseSpeed;
        dst.heatAxisLocal = src.heatAxisLocal;
        dst.hotEndBias = src.hotEndBias;
        dst.hotRangeStart = src.hotRangeStart;
        dst.hotRangeEnd = src.hotRangeEnd;
    }

    static void to_internal(const BlackbodyMaterialSettings &src, AssetManager::BlackbodyMaterialSettings &dst)
    {
        dst.colorFactor = src.colorFactor;
        dst.metallic = src.metallic;
        dst.roughness = src.roughness;
        dst.normalScale = src.normalScale;

        dst.albedoPath = src.albedoPath;
        dst.albedoSRGB = src.albedoSRGB;
        dst.metalRoughPath = src.metalRoughPath;
        dst.metalRoughSRGB = src.metalRoughSRGB;
        dst.normalPath = src.normalPath;
        dst.normalSRGB = src.normalSRGB;
        dst.occlusionPath = src.occlusionPath;
        dst.occlusionSRGB = src.occlusionSRGB;
        dst.occlusionStrength = src.occlusionStrength;

        to_internal(src.blackbody, dst.blackbody);
    }

    static void from_internal(const AssetManager::BlackbodyMaterialSettings &src, BlackbodyMaterialSettings &dst)
    {
        dst.colorFactor = src.colorFactor;
        dst.metallic = src.metallic;
        dst.roughness = src.roughness;
        dst.normalScale = src.normalScale;

        dst.albedoPath = src.albedoPath;
        dst.albedoSRGB = src.albedoSRGB;
        dst.metalRoughPath = src.metalRoughPath;
        dst.metalRoughSRGB = src.metalRoughSRGB;
        dst.normalPath = src.normalPath;
        dst.normalSRGB = src.normalSRGB;
        dst.occlusionPath = src.occlusionPath;
        dst.occlusionSRGB = src.occlusionSRGB;
        dst.occlusionStrength = src.occlusionStrength;

        from_internal(src.blackbody, dst.blackbody);
    }
}

bool Engine::create_or_update_blackbody_material(const std::string &materialName,
                                                 const BlackbodyMaterialSettings &settings)
{
    if (!_engine || !_engine->_assetManager)
    {
        return false;
    }

    AssetManager::BlackbodyMaterialSettings s{};
    to_internal(settings, s);
    return _engine->_assetManager->createOrUpdateBlackbodyMaterial(materialName, s);
}

bool Engine::get_blackbody_material(const std::string &materialName,
                                    BlackbodyMaterialSettings &out) const
{
    if (!_engine || !_engine->_assetManager)
    {
        return false;
    }

    AssetManager::BlackbodyMaterialSettings s{};
    if (!_engine->_assetManager->getBlackbodyMaterialSettings(materialName, s))
    {
        return false;
    }

    from_internal(s, out);
    return true;
}

bool Engine::remove_blackbody_material(const std::string &materialName)
{
    if (!_engine || !_engine->_assetManager)
    {
        return false;
    }
    return _engine->_assetManager->removeBlackbodyMaterial(materialName);
}

bool Engine::apply_blackbody_material_to_primitive(const std::string &primitiveName,
                                                   const std::string &materialName)
{
    if (!_engine || !_engine->_assetManager || !_engine->_sceneManager)
    {
        return false;
    }

    auto material = _engine->_assetManager->getBlackbodyMaterial(materialName);
    if (!material)
    {
        return false;
    }
    return _engine->_sceneManager->setMeshInstanceMaterial(primitiveName, material);
}

bool Engine::set_gltf_material_blackbody(const std::string &instanceName,
                                         const std::string &materialName,
                                         const BlackbodySettings &settings)
{
    if (!_engine || !_engine->_assetManager || !_engine->_sceneManager)
    {
        return false;
    }

    auto scene = _engine->_sceneManager->getGLTFInstanceScene(instanceName);
    if (!scene)
    {
        return false;
    }

    AssetManager::BlackbodySettings s{};
    to_internal(settings, s);
    return _engine->_assetManager->applyBlackbodyToGLTFMaterial(*scene, materialName, s);
}

} // namespace GameAPI
