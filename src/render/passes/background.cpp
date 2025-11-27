#include "background.h"
#include <string_view>

#include "core/device/swapchain.h"
#include "core/context.h"
#include "core/device/resource.h"
#include "core/pipeline/manager.h"
#include "core/assets/manager.h"
#include "render/graph/graph.h"
#include <cstring>

#include "core/frame/resources.h"
#include "core/assets/ibl_manager.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/pipeline/sampler.h"

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
    // Graphics env (cubemap) background mode
    ComputeEffect env{}; env.name = "env";
    _backgroundEffects.push_back(env);

    // Prepare graphics pipeline for environment background (cubemap)
    // Create an empty descriptor set layout to occupy sets 1 and 2 (shader uses set=0 and set=3)
    {
        VkDescriptorSetLayoutCreateInfo info{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        info.bindingCount = 0;
        info.pBindings = nullptr;
        vkCreateDescriptorSetLayout(_context->getDevice()->device(), &info, nullptr, &_emptySetLayout);
    }

    GraphicsPipelineCreateInfo gp{};
    gp.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    gp.fragmentShaderPath = _context->getAssets()->shaderPath("background_env.frag.spv");
    VkDescriptorSetLayout sl0 = _context->getDescriptorLayouts()->gpuSceneDataLayout();
    VkDescriptorSetLayout sl1 = _emptySetLayout; // placeholder for set=1
    VkDescriptorSetLayout sl2 = _emptySetLayout; // placeholder for set=2
    // Ensure IBL layout exists (now owned by IBLManager)
    VkDescriptorSetLayout sl3 = _emptySetLayout;
    if (_context->ibl && _context->ibl->ensureLayout())
        sl3 = _context->ibl->descriptorLayout();
    gp.setLayouts = { sl0, sl1, sl2, sl3 };
    gp.configure = [this](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_depthtest();
        b.disable_blending();
        b.set_color_attachment_format(_context->getSwapchain()->drawImage().imageFormat);
    };
    _context->pipelines->createGraphicsPipeline("background.env", gp);

    // Create fallback 1x1x6 black cube
    {
        const uint32_t faceCount = 6;
        const uint32_t pixel = 0x00000000u; // RGBA8 black
        std::vector<uint8_t> bytes(faceCount * 4);
        for (uint32_t f = 0; f < faceCount; ++f) std::memcpy(bytes.data() + f * 4, &pixel, 4);
        std::vector<VkBufferImageCopy> copies;
        copies.reserve(faceCount);
        for (uint32_t f = 0; f < faceCount; ++f) {
            VkBufferImageCopy r{};
            r.bufferOffset = f * 4;
            r.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            r.imageSubresource.mipLevel = 0;
            r.imageSubresource.baseArrayLayer = f;
            r.imageSubresource.layerCount = 1;
            r.imageExtent = {1,1,1};
            copies.push_back(r);
        }
        _fallbackIblCube = _context->getResources()->create_image_compressed_layers(
            bytes.data(), bytes.size(), VK_FORMAT_R8G8B8A8_UNORM, 1, faceCount, copies,
            VK_IMAGE_USAGE_SAMPLED_BIT, VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT);
    }
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

    // Route to compute or graphics depending on selected mode
    const ComputeEffect &effect = _backgroundEffects[_currentEffect];
    if (std::string_view(effect.name) == std::string_view("env"))
    {
        graph->add_pass(
            "BackgroundEnv",
            RGPassType::Graphics,
            [drawHandle](RGPassBuilder &builder, EngineContext *) {
                builder.write_color(drawHandle);
            },
            [this, drawHandle](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
                VkImageView drawView = res.image_view(drawHandle);
                (void) drawView; // handled by RG

                // pipeline + layout
                if (!ctx->pipelines->getGraphics("background.env", _envPipeline, _envPipelineLayout)) return;

                // Per-frame scene UBO
                AllocatedBuffer ubo = ctx->getResources()->create_buffer(sizeof(GPUSceneData),
                                                                         VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                         VMA_MEMORY_USAGE_CPU_TO_GPU);
                ctx->currentFrame->_deletionQueue.push_function([rm = ctx->getResources(), ubo]() { rm->destroy_buffer(ubo); });
                VmaAllocationInfo ai{}; vmaGetAllocationInfo(ctx->getDevice()->allocator(), ubo.allocation, &ai);
                *reinterpret_cast<GPUSceneData*>(ai.pMappedData) = ctx->getSceneData();
                vmaFlushAllocation(ctx->getDevice()->allocator(), ubo.allocation, 0, sizeof(GPUSceneData));

                VkDescriptorSet global = ctx->currentFrame->_frameDescriptors.allocate(
                    ctx->getDevice()->device(), ctx->getDescriptorLayouts()->gpuSceneDataLayout());
                DescriptorWriter w0; w0.write_buffer(0, ubo.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
                w0.update_set(ctx->getDevice()->device(), global);

                // IBL/background set (set = 3)
                VkDescriptorSet ibl = VK_NULL_HANDLE;
                if (ctx->ibl)
                {
                    VkImageView envView = _fallbackIblCube.imageView;
                    // Prefer a dedicated background texture when available, otherwise reuse specular.
                    if (ctx->ibl->background().imageView)
                    {
                        envView = ctx->ibl->background().imageView;
                    }
                    else if (ctx->ibl->specular().imageView)
                    {
                        envView = ctx->ibl->specular().imageView;
                    }

                    VkDescriptorSetLayout iblLayout = ctx->ibl->descriptorLayout();
                    ibl = ctx->currentFrame->_frameDescriptors.allocate(
                        ctx->getDevice()->device(), iblLayout);
                    DescriptorWriter w3;
                    // Bind background map at binding 3; other bindings are unused in this shader.
                    w3.write_image(3, envView, ctx->getSamplers()->defaultLinear(),
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
                    w3.update_set(ctx->getDevice()->device(), ibl);
                }

                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _envPipeline);
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _envPipelineLayout, 0, 1, &global, 0, nullptr);
                if (ibl != VK_NULL_HANDLE)
                {
                    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _envPipelineLayout, 3, 1, &ibl, 0, nullptr);
                }

                VkExtent2D extent = ctx->getDrawExtent();
                VkViewport vp{0.f, 0.f, float(extent.width), float(extent.height), 0.f, 1.f};
                VkRect2D sc{{0,0}, extent};
                vkCmdSetViewport(cmd, 0, 1, &vp);
                vkCmdSetScissor(cmd, 0, 1, &sc);
                vkCmdDraw(cmd, 3, 1, 0, 0);
            }
        );
    }
    else
    {
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

                ComputeEffect &eff = _backgroundEffects[_currentEffect];

                ComputeDispatchInfo dispatchInfo = ComputeManager::createDispatch2D(
                    ctx->getDrawExtent().width, ctx->getDrawExtent().height);
                dispatchInfo.pushConstants = &eff.data;
                dispatchInfo.pushConstantSize = sizeof(ComputePushConstants);

                const char *instanceName = (std::string_view(eff.name) == std::string_view("gradient"))
                                           ? "background.gradient"
                                           : "background.sky";
                ctx->pipelines->dispatchComputeInstance(cmd, instanceName, dispatchInfo);
            }
        );
    }
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
    if (_envPipeline != VK_NULL_HANDLE || _envPipelineLayout != VK_NULL_HANDLE)
    {
        // Pipelines are owned by PipelineManager and destroyed there on cleanup/hot-reload
        _envPipeline = VK_NULL_HANDLE;
        _envPipelineLayout = VK_NULL_HANDLE;
    }
    if (_emptySetLayout)
    {
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _emptySetLayout, nullptr);
        _emptySetLayout = VK_NULL_HANDLE;
    }
    if (_fallbackIblCube.image)
    {
        _context->getResources()->destroy_image(_fallbackIblCube);
        _fallbackIblCube = {};
    }
    fmt::print("BackgroundPass::cleanup()\n");
    _backgroundEffects.clear();
}
