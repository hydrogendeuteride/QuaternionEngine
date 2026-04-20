#pragma once

#include <core/types.h>
#include <render/renderpass.h>
#include <render/graph/types.h>

class EngineContext;
class RenderGraph;
class RGPassResources;
struct RenderObject;

class HoverOutlinePass final : public IRenderPass
{
public:
    enum class TargetMode : uint32_t
    {
        Composite = 0,
        Primitive = 1,
    };

    struct Settings
    {
        bool enabled = true;
        glm::vec3 color{1.0f, 0.72f, 0.15f};
        float intensity = 0.8f;
        float outline_width_px = 2.0f;
        float blur_radius_px = 3.0f;
        bool half_resolution_blur = true;
        TargetMode target_mode = TargetMode::Composite;
    };

    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer) override;

    const char *getName() const override { return "HoverOutline"; }

    RGImageHandle register_graph(RenderGraph *graph, RGImageHandle ldrInput, RGImageHandle depthHandle);

    Settings &settings() { return _settings; }
    const Settings &settings() const { return _settings; }

    void set_enabled(bool enabled) { _settings.enabled = enabled; }
    bool enabled() const { return _settings.enabled; }
    void set_target_mode(TargetMode mode) { _settings.target_mode = mode; }
    TargetMode target_mode() const { return _settings.target_mode; }

private:
    struct BlurPush
    {
        glm::vec2 inverse_extent{0.0f, 0.0f};
        glm::vec2 direction{0.0f, 0.0f};
        float radius_px = 1.0f;
        float _pad[3]{};
    };

    struct CompositePush
    {
        glm::vec4 color_intensity{1.0f, 1.0f, 1.0f, 1.0f};
        glm::vec4 params{0.0f, 0.0f, 0.0f, 0.0f};
    };

    void draw_mask(VkCommandBuffer cmd,
                   EngineContext *context,
                   const std::vector<const RenderObject *> &draws) const;
    void draw_blur(VkCommandBuffer cmd,
                   EngineContext *context,
                   const RGPassResources &resources,
                   RGImageHandle inputHandle,
                   const glm::vec2 &direction,
                   VkExtent2D render_extent) const;
    void draw_composite(VkCommandBuffer cmd,
                        EngineContext *context,
                        const RGPassResources &resources,
                        RGImageHandle ldrInput,
                        RGImageHandle rawMask,
                        RGImageHandle blurredMask) const;

    EngineContext *_context = nullptr;

    VkPipeline _maskPipeline = VK_NULL_HANDLE;
    VkPipelineLayout _maskPipelineLayout = VK_NULL_HANDLE;
    VkPipeline _blurPipeline = VK_NULL_HANDLE;
    VkPipelineLayout _blurPipelineLayout = VK_NULL_HANDLE;
    VkPipeline _compositePipeline = VK_NULL_HANDLE;
    VkPipelineLayout _compositePipelineLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _singleImageLayout = VK_NULL_HANDLE;

    Settings _settings{};
};
