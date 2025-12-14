#pragma once

#include <core/types.h>
#include <render/graph/types.h>
#include <render/graph/resources.h>
#include <render/graph/builder.h>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

class EngineContext;

class RenderGraph
{
public:
    void init(EngineContext* ctx);
    void clear();
    // Destroy any GPU-side state owned by the graph (e.g. query pools).
    // Call during engine shutdown before destroying the VkDevice.
    void shutdown();

	// Import externally owned images (swapchain, drawImage, g-buffers)
	RGImageHandle import_image(const RGImportedImageDesc& desc);

	// Create transient images (not used in v1 skeleton; stubbed for future)
	RGImageHandle create_image(const RGImageDesc& desc);

	// Convenience: create a transient depth image suitable for shadow mapping or depth-only passes
	// Format defaults to D32_SFLOAT; usage is depth attachment + sampled so it can be read later.
	RGImageHandle create_depth_image(const char* name, VkExtent2D extent, VkFormat format = VK_FORMAT_D32_SFLOAT);

	// Buffer import/create helpers
	RGBufferHandle import_buffer(const RGImportedBufferDesc& desc);
	RGBufferHandle create_buffer(const RGBufferDesc& desc);

	// Pass builder API
	using RecordCallback = std::function<void(VkCommandBuffer cmd, const class RGPassResources& res, EngineContext* ctx)>;
	using BuildCallback  = std::function<void(class RGPassBuilder& b, EngineContext* ctx)>;

	void add_pass(const char* name, RGPassType type, BuildCallback build, RecordCallback record);
	// Legacy simple add
	void add_pass(const char* name, RGPassType type, RecordCallback record);

	// Build internal state for this frame (no-op in v1)
	bool compile();

    // Execute in insertion order (skips disabled passes)
    void execute(VkCommandBuffer cmd);

	// Convenience import helpers (read from EngineContext::swapchain)
	RGImageHandle import_draw_image();
	RGImageHandle import_depth_image();
	RGImageHandle import_gbuffer_position();
	RGImageHandle import_gbuffer_normal();
	RGImageHandle import_gbuffer_albedo();
    RGImageHandle import_gbuffer_extra();
	RGImageHandle import_id_buffer();
	RGImageHandle import_swapchain_image(uint32_t index);
    void add_present_chain(RGImageHandle sourceDraw,
                           RGImageHandle targetSwapchain,
                           std::function<void(RenderGraph&)> appendExtra = {});

    // --- Debug helpers ---
    struct RGDebugPassInfo
    {
        std::string name;
        RGPassType type{};
        bool enabled = true;
        uint32_t imageReads = 0;
        uint32_t imageWrites = 0;
        uint32_t bufferReads = 0;
        uint32_t bufferWrites = 0;
        uint32_t colorAttachmentCount = 0;
        bool hasDepth = false;
        // Last frame timings (ms); -1 when unavailable
        float gpuMillis = -1.0f;
        float cpuMillis = -1.0f;
    };

    struct RGDebugImageInfo
    {
        uint32_t id{};
        std::string name;
        bool imported = true;
        VkFormat format = VK_FORMAT_UNDEFINED;
        VkExtent2D extent{0,0};
        VkImageUsageFlags creationUsage = 0;
        int firstUse = -1;
        int lastUse = -1;
    };

    struct RGDebugBufferInfo
    {
        uint32_t id{};
        std::string name;
        bool imported = true;
        VkDeviceSize size = 0;
        VkBufferUsageFlags usage = 0;
        int firstUse = -1;
        int lastUse = -1;
    };

    size_t pass_count() const { return _passes.size(); }
    const char* pass_name(size_t i) const { return i < _passes.size() ? _passes[i].name.c_str() : ""; }
    bool pass_enabled(size_t i) const { return i < _passes.size() ? _passes[i].enabled : false; }
    void set_pass_enabled(size_t i, bool e) { if (i < _passes.size()) _passes[i].enabled = e; }

    void debug_get_passes(std::vector<RGDebugPassInfo>& out) const;
    void debug_get_images(std::vector<RGDebugImageInfo>& out) const;
    void debug_get_buffers(std::vector<RGDebugBufferInfo>& out) const;

    // Resolve GPU timestamps from the previous execute() call. Call after waiting on the render fence.
    void resolve_timings();

private:
	struct ImportedImage
	{
		RGImportedImageDesc desc;
		RGImageHandle handle;
	};

    struct Pass
    {
        std::string name;
        RGPassType type{};
        RecordCallback record;

		// Declarations
		std::vector<RGPassImageAccess> imageReads;
		std::vector<RGPassImageAccess> imageWrites;
		std::vector<RGPassBufferAccess> bufferReads;
		std::vector<RGPassBufferAccess> bufferWrites;
		std::vector<RGAttachmentInfo> colorAttachments;
		bool hasDepth = false;
		RGAttachmentInfo depthAttachment{};

        std::vector<VkImageMemoryBarrier2> preImageBarriers;
        std::vector<VkBufferMemoryBarrier2> preBufferBarriers;

        // Cached rendering info derived from declared attachments (filled at execute)
        bool hasRendering = false;
        VkExtent2D renderExtent{};

        bool enabled = true;
    };

	EngineContext* _context = nullptr;
    RGResourceRegistry _resources;
    std::vector<Pass> _passes;

    // --- Timing data for last executed frame ---
    VkQueryPool _timestampPool = VK_NULL_HANDLE; // holds 2 queries per pass (begin/end)
    std::vector<float> _lastGpuMillis; // per pass
    std::vector<float> _lastCpuMillis; // per pass (command recording time)
    std::vector<bool> _wroteTimestamps; // per pass; true if queries were written in last execute
};
