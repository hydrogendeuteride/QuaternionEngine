#include "vk_renderpass_background.h"
#include <string_view>

#include "vk_swapchain.h"
#include "core/engine_context.h"
#include "core/vk_resource.h"
#include "core/vk_pipeline_manager.h"
#include "core/asset_manager.h"
#include "render/rg_graph.h"

void BackgroundPass::init(EngineContext *context)
{
    _context = context;
    init_background_pipelines();
}

void BackgroundPass::init_background_pipelines()
{
    ComputePipelineCreateInfo createInfo{};
    createInfo.shaderPath = _context->getAssets()->shaderPath("gradient_color.comp.spv");
    createInfo.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE};
    createInfo.pushConstantSize = sizeof(ComputePushConstants);
    _context->pipelines->createComputePipeline("gradient", createInfo);

    createInfo.shaderPath = _context->getAssets()->shaderPath("sky.comp.spv");
    _context->pipelines->createComputePipeline("sky", createInfo);

    _context->pipelines->createComputeInstance("background.gradient", "gradient");
    _context->pipelines->createComputeInstance("background.sky", "sky");
    _context->pipelines->setComputeInstanceStorageImage("background.gradient", 0,
                                                        _context->getSwapchain()->drawImage().imageView);
    _context->pipelines->setComputeInstanceStorageImage("background.sky", 0,
                                                        _context->getSwapchain()->drawImage().imageView);

    ComputeEffect gradient{};
    gradient.name = "gradient";
    gradient.data.data1 = glm::vec4(1, 0, 0, 1);
    gradient.data.data2 = glm::vec4(0, 0, 1, 1);

    ComputeEffect sky{};
    sky.name = "sky";
    sky.data.data1 = glm::vec4(0.1, 0.2, 0.4, 0.97);

    _backgroundEffects.push_back(gradient);
    _backgroundEffects.push_back(sky);
}

void BackgroundPass::execute(VkCommandBuffer)
{
    // Background is executed via the render graph now.
}

void BackgroundPass::register_graph(RenderGraph *graph, RGImageHandle drawHandle, RGImageHandle depthHandle)
{
    (void) depthHandle; // Reserved for future depth transitions.
    if (!graph || !drawHandle.valid() || !_context) return;
    if (_backgroundEffects.empty()) return;

    graph->add_pass(
        "Background",
        RGPassType::Compute,
        [drawHandle](RGPassBuilder &builder, EngineContext *) {
            builder.write(drawHandle, RGImageUsage::ComputeWrite);
        },
        [this, drawHandle](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
            VkImageView drawView = res.image_view(drawHandle);
            if (drawView != VK_NULL_HANDLE)
            {
                _context->pipelines->setComputeInstanceStorageImage("background.gradient", 0, drawView);
                _context->pipelines->setComputeInstanceStorageImage("background.sky", 0, drawView);
            }

            ComputeEffect &effect = _backgroundEffects[_currentEffect];

            ComputeDispatchInfo dispatchInfo = ComputeManager::createDispatch2D(
                ctx->getDrawExtent().width, ctx->getDrawExtent().height);
            dispatchInfo.pushConstants = &effect.data;
            dispatchInfo.pushConstantSize = sizeof(ComputePushConstants);

            const char *instanceName = (std::string_view(effect.name) == std::string_view("gradient"))
                                       ? "background.gradient"
                                       : "background.sky";
            ctx->pipelines->dispatchComputeInstance(cmd, instanceName, dispatchInfo);
        }
    );
}

void BackgroundPass::cleanup()
{
    if (_context && _context->pipelines)
    {
        _context->pipelines->destroyComputeInstance("background.gradient");
        _context->pipelines->destroyComputeInstance("background.sky");
        _context->pipelines->destroyComputePipeline("gradient");
        _context->pipelines->destroyComputePipeline("sky");
    }
    fmt::print("BackgroundPass::cleanup()\n");
    _backgroundEffects.clear();
}
