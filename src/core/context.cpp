#include "context.h"
#include "scene/vk_scene.h"

const GPUSceneData &EngineContext::getSceneData() const
{
    return scene->getSceneData();
}

const DrawContext &EngineContext::getMainDrawContext() const
{
    return const_cast<SceneManager *>(scene)->getMainDrawContext();
}

