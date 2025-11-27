#include "imgui_pass.h"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_vulkan.h"
#include "core/device/device.h"
#include "core/device/swapchain.h"
#include "core/util/initializers.h"
#include "core/context.h"
#include "render/graph/graph.h"

void ImGuiPass::init(EngineContext *context)
{
    _context = context;

    VkDescriptorPoolSize pool_sizes[] = {
        {VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
        {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
        {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}
    };

    VkDescriptorPoolCreateInfo pool_info = {};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.maxSets = 1000;
    pool_info.poolSizeCount = (uint32_t) std::size(pool_sizes);
    pool_info.pPoolSizes = pool_sizes;

    VkDescriptorPool imguiPool;
    VK_CHECK(vkCreateDescriptorPool(_context->device->device(), &pool_info, nullptr, &imguiPool));

    ImGui::CreateContext();

    ImGui_ImplSDL2_InitForVulkan(_context->window);

    ImGui_ImplVulkan_InitInfo init_info = {};
    init_info.Instance = _context->getDevice()->instance();
    init_info.PhysicalDevice = _context->getDevice()->physicalDevice();
    init_info.Device = _context->getDevice()->device();
    init_info.Queue = _context->getDevice()->graphicsQueue();
    init_info.DescriptorPool = imguiPool;
    init_info.MinImageCount = 3;
    init_info.ImageCount = 3;
    init_info.UseDynamicRendering = true;

    init_info.PipelineRenderingCreateInfo = {.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO};
    init_info.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
    auto _swapchainImageFormat = _context->getSwapchain()->swapchainImageFormat();
    init_info.PipelineRenderingCreateInfo.pColorAttachmentFormats = &_swapchainImageFormat;

    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

    ImGui_ImplVulkan_Init(&init_info);

    ImGui_ImplVulkan_CreateFontsTexture();

    // add the destroy the imgui created structures
    _deletionQueue.push_function([=]() {
        ImGui_ImplVulkan_Shutdown();
        vkDestroyDescriptorPool(_context->getDevice()->device(), imguiPool, nullptr);
    });
}

void ImGuiPass::cleanup()
{
    fmt::print("ImGuiPass::cleanup()\n");
    _deletionQueue.flush();
}

void ImGuiPass::execute(VkCommandBuffer)
{
    // ImGui is executed via the render graph now.
}

void ImGuiPass::register_graph(RenderGraph *graph, RGImageHandle swapchainHandle)
{
    if (!graph || !swapchainHandle.valid()) return;

    graph->add_pass(
        "ImGui",
        RGPassType::Graphics,
        [swapchainHandle](RGPassBuilder &builder, EngineContext *)
        {
            builder.write_color(swapchainHandle, false, {});
        },
        [this, swapchainHandle](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx)
        {
            draw_imgui(cmd, ctx, res, swapchainHandle);
        });
}

void ImGuiPass::draw_imgui(VkCommandBuffer cmd,
                           EngineContext *context,
                           const RGPassResources &resources,
                           RGImageHandle targetHandle) const
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal) return;

    VkImageView targetImageView = resources.image_view(targetHandle);
    if (targetImageView == VK_NULL_HANDLE) return;

    // Dynamic rendering is handled by the RenderGraph; just render draw data.
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);
}
