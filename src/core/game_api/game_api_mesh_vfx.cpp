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

