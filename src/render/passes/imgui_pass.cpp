#include "imgui_pass.h"

#include "imgui.h"
#include "imgui_impl_vulkan.h"
#include "core/context.h"
#include "render/graph/graph.h"

void ImGuiPass::init(EngineContext *context)
{
    _context = context;
}

void ImGuiPass::cleanup()
{
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
