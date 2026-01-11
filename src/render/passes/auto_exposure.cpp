#include "auto_exposure.h"

#include "tonemap.h"

#include <core/context.h>
#include <core/frame/resources.h>
#include <core/assets/manager.h>
#include <core/device/device.h>
#include <core/device/resource.h>
#include <core/device/swapchain.h>
#include <core/pipeline/manager.h>
#include <core/pipeline/sampler.h>

#include <render/graph/graph.h>
#include <render/graph/resources.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
    // Matches the std430 buffer layout in shaders/auto_exposure.comp (vec4).
    constexpr VkDeviceSize k_readback_size = 16;

    static bool finite_positive(float v)
    {
        return std::isfinite(v) && v > 0.0f;
    }

    static float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    static float exp_smooth_alpha(float speed, float dt)
    {
        if (!std::isfinite(speed) || speed <= 0.0f) return 1.0f;
        if (!std::isfinite(dt) || dt <= 0.0f) return 0.0f;
        return 1.0f - std::exp(-speed * dt);
    }
} // namespace

void AutoExposurePass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->pipelines || !_context->getAssets() || !_context->getResources() || !_context->
        getDevice())
    {
        return;
    }

    // Compute pipeline: binding 0 = HDR input (combined sampler), binding 1 = readback storage buffer.
    ComputePipelineCreateInfo createInfo{};
    createInfo.shaderPath = _context->getAssets()->shaderPath("auto_exposure.comp.spv");
    createInfo.descriptorTypes = {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER};
    _context->pipelines->createComputePipeline("auto_exposure", createInfo);
    _context->pipelines->createComputeInstance("auto_exposure", "auto_exposure");

    // Allocate per-frame-slot readback buffers (host-visible).
    for (uint32_t i = 0; i < k_frame_overlap; ++i)
    {
        _readback_buffers[i] = _context->getResources()->create_buffer(
            k_readback_size,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_CPU_TO_GPU);

        if (_readback_buffers[i].info.pMappedData)
        {
            Readback init{};
            init.avg_log2_lum = 0.0f;
            init.avg_lum = _last_luminance;
            init.valid = 0.0f;
            std::memcpy(_readback_buffers[i].info.pMappedData, &init, sizeof(init));
            vmaFlushAllocation(_context->getDevice()->allocator(), _readback_buffers[i].allocation, 0, k_readback_size);
        }
    }

    // Seed the instance with a valid image to avoid missing bindings on the first dispatch.
    VkImageView fallback = VK_NULL_HANDLE;
    if (_context->getSwapchain())
    {
        fallback = _context->getSwapchain()->drawImage().imageView;
    }
    if (fallback != VK_NULL_HANDLE && _context->getSamplers())
    {
        _context->pipelines->setComputeInstanceSampledImage("auto_exposure", 0, fallback,
                                                            _context->getSamplers()->defaultLinear());
    }
    _context->pipelines->setComputeInstanceBuffer("auto_exposure", 1, _readback_buffers[0].buffer, k_readback_size,
                                                  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 0);
}

void AutoExposurePass::cleanup()
{
    if (_context && _context->pipelines)
    {
        _context->pipelines->destroyComputeInstance("auto_exposure");
        _context->pipelines->destroyComputePipeline("auto_exposure");
    }

    if (_context && _context->getResources())
    {
        for (auto &b: _readback_buffers)
        {
            if (b.buffer != VK_NULL_HANDLE)
            {
                _context->getResources()->destroy_buffer(b);
            }
            b = {};
        }
    }

    _context = nullptr;
}

void AutoExposurePass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

void AutoExposurePass::set_enabled(bool enabled, float current_exposure)
{
    _enabled = enabled;
    if (_enabled)
    {
        if (std::isfinite(current_exposure) && current_exposure > 0.0f)
        {
            _exposure = current_exposure;
            _have_exposure = true;
        }
        else
        {
            _have_exposure = false;
        }
    }
}

void AutoExposurePass::begin_frame(uint32_t frame_slot, float dt_sec, TonemapPass &tonemap)
{
    if (!_enabled || !_context || !_context->getDevice()) return;

    frame_slot %= k_frame_overlap;
    AllocatedBuffer &buf = _readback_buffers[frame_slot];
    if (buf.buffer == VK_NULL_HANDLE || buf.allocation == VK_NULL_HANDLE || buf.info.pMappedData == nullptr)
    {
        return;
    }

    dt_sec = std::clamp(dt_sec, 0.0f, 0.1f);

    // Ensure GPU writes are visible on the CPU.
    vmaInvalidateAllocation(_context->getDevice()->allocator(), buf.allocation, 0, k_readback_size);

    const Readback *rb = reinterpret_cast<const Readback *>(buf.info.pMappedData);
    if (!rb) return;
    if (!(rb->valid >= 0.5f)) return;
    if (!finite_positive(rb->avg_lum)) return;

    const float lum = std::max(rb->avg_lum, 1.0e-4f);
    _last_luminance = lum;

    float key = _key_value;
    if (!std::isfinite(key) || key <= 0.0f) key = 0.18f;

    float comp = _compensation;
    if (!std::isfinite(comp) || comp < 0.0f) comp = 0.0f;

    float target = (key / lum) * comp;

    float min_e = _min_exposure;
    float max_e = _max_exposure;
    if (!std::isfinite(min_e) || min_e <= 0.0f) min_e = 0.0001f;
    if (!std::isfinite(max_e) || max_e <= 0.0f) max_e = min_e;
    if (max_e < min_e) std::swap(max_e, min_e);

    target = std::clamp(target, min_e, max_e);
    _target_exposure = target;

    if (!_have_exposure || !finite_positive(_exposure))
    {
        _exposure = target;
        _have_exposure = true;
    }
    else
    {
        const float speed = (target > _exposure) ? _speed_up : _speed_down;
        const float alpha = exp_smooth_alpha(speed, dt_sec);
        _exposure = lerp(_exposure, target, alpha);
    }

    tonemap.setExposure(_exposure);
}

void AutoExposurePass::register_graph(RenderGraph *graph, RGImageHandle hdrInput)
{
    if (!_enabled || !_context || !graph || !hdrInput.valid()) return;

    const uint32_t slot = _context->frameIndex % k_frame_overlap;
    const VkBuffer out_buf = _readback_buffers[slot].buffer;
    if (out_buf == VK_NULL_HANDLE) return;

    const std::string buf_name = "auto_exposure.readback." + std::to_string(slot);

    graph->add_pass(
        "AutoExposure",
        RGPassType::Compute,
        [hdrInput, out_buf, buf_name](RGPassBuilder &builder, EngineContext *) {
            builder.read(hdrInput, RGImageUsage::SampledCompute);
            builder.write_buffer(out_buf, RGBufferUsage::StorageReadWrite, k_readback_size, buf_name.c_str());
        },
        [this, hdrInput, slot](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
            dispatch_measure(cmd, ctx, res, hdrInput, slot);
        });
}

void AutoExposurePass::dispatch_measure(VkCommandBuffer cmd,
                                        EngineContext *ctx,
                                        const RGPassResources &res,
                                        RGImageHandle hdrInput,
                                        uint32_t frame_slot)
{
    EngineContext *ctxLocal = ctx ? ctx : _context;
    if (!ctxLocal || !ctxLocal->pipelines || !ctxLocal->getSamplers()) return;

    VkImageView hdrView = res.image_view(hdrInput);
    if (hdrView == VK_NULL_HANDLE) return;

    frame_slot %= k_frame_overlap;
    const VkBuffer out_buf = _readback_buffers[frame_slot].buffer;
    if (out_buf == VK_NULL_HANDLE) return;

    ctxLocal->pipelines->setComputeInstanceSampledImage("auto_exposure", 0, hdrView,
                                                        ctxLocal->getSamplers()->defaultLinear(),
                                                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    ctxLocal->pipelines->setComputeInstanceBuffer("auto_exposure", 1, out_buf, k_readback_size,
                                                  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 0);

    ComputeDispatchInfo di{};
    di.groupCountX = 1;
    di.groupCountY = 1;
    di.groupCountZ = 1;
    ctxLocal->pipelines->dispatchComputeInstance(cmd, "auto_exposure", di);
}
