#pragma once

#include <core/types.h>
#include <render/renderpass.h>
#include <render/graph/types.h>

#include <array>

class EngineContext;
class RenderGraph;
class RGPassResources;
class TonemapPass;

// Simple auto-exposure pass:
// - GPU: compute average log-luminance from the final HDR buffer (pre-tonemap)
// - CPU: read back per-frame luminance after fence wait and drive TonemapPass exposure
class AutoExposurePass final : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer) override; // Not used directly; executed via render graph
    const char *getName() const override { return "AutoExposure"; }

    // Read back last luminance for this frame slot after the fence wait, update
    // smoothed exposure, and apply it to the tonemap pass (if enabled).
    void begin_frame(uint32_t frame_slot, float dt_sec, TonemapPass &tonemap);

    // Register luminance measurement into the render graph (compute, no outputs).
    void register_graph(RenderGraph *graph, RGImageHandle hdrInput);

    // Runtime controls
    void set_enabled(bool enabled, float current_exposure = 1.0f);
    bool enabled() const { return _enabled; }

    void set_key_value(float v) { _key_value = v; }
    float key_value() const { return _key_value; }

    void set_compensation(float v) { _compensation = v; }
    float compensation() const { return _compensation; }

    void set_min_exposure(float v) { _min_exposure = v; }
    float min_exposure() const { return _min_exposure; }

    void set_max_exposure(float v) { _max_exposure = v; }
    float max_exposure() const { return _max_exposure; }

    void set_speed_up(float v) { _speed_up = v; }
    float speed_up() const { return _speed_up; }

    void set_speed_down(float v) { _speed_down = v; }
    float speed_down() const { return _speed_down; }

    // Debug values (updated when begin_frame sees valid data)
    float exposure() const { return _exposure; }
    float target_exposure() const { return _target_exposure; }
    float last_luminance() const { return _last_luminance; }

private:
    static constexpr uint32_t k_frame_overlap = 2;

    struct Readback
    {
        float avg_log2_lum = 0.0f;
        float avg_lum = 0.18f;
        float valid = 0.0f;
        float _pad = 0.0f;
    };
    static_assert(sizeof(Readback) == 16);

    void dispatch_measure(VkCommandBuffer cmd,
                          EngineContext *ctx,
                          const RGPassResources &res,
                          RGImageHandle hdrInput,
                          uint32_t frame_slot);

    EngineContext *_context = nullptr;

    bool _enabled = false;

    // Exposure model: exposure = (key / Lavg) * compensation, clamped and smoothed.
    float _key_value = 0.18f;
    float _compensation = 1.0f;
    float _min_exposure = 0.05f;
    float _max_exposure = 2.0f;
    float _speed_up = 4.0f;   // 1/sec
    float _speed_down = 1.0f; // 1/sec

    float _exposure = 1.0f;
    bool _have_exposure = false;

    float _target_exposure = 1.0f;
    float _last_luminance = 0.18f;

    std::array<AllocatedBuffer, k_frame_overlap> _readback_buffers{};
};

