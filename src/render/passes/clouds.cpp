#include "clouds.h"

#include "core/frame/resources.h"
#include "core/descriptor/descriptors.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/device/swapchain.h"
#include "core/context.h"
#include "core/pipeline/manager.h"
#include "core/assets/manager.h"
#include "core/pipeline/sampler.h"

#include "render/graph/graph.h"
#include "render/graph/resources.h"
#include "render/pipelines.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "vk_scene.h"

namespace
{
    struct VolumePush
    {
        glm::vec4 volume_center_follow; // xyz: center_local (or offset), w: followCameraXZ (0/1)
        glm::vec4 volume_half_extents;  // xyz: half extents (local)
        glm::vec4 density_params;       // x: densityScale, y: coverage, z: extinction, w: time_sec
        glm::vec4 scatter_params;       // rgb: albedo/tint, w: scatterStrength
        glm::vec4 emission_params;      // rgb: emissionColor, w: emissionStrength
        glm::ivec4 misc;                // x: stepCount, y: gridResolution, z: volumeType
    };

    struct VolumeVoxelPush
    {
        glm::vec4 wind_dt;          // xyz: windVelocityLocal, w: dt_sec
        glm::vec4 volume_size_time; // xyz: volume size, w: time_sec
        glm::vec4 sim_params;       // x: dissipation, y: noiseStrength, z: noiseScale, w: noiseSpeed
        glm::vec4 emitter_params;   // xyz: emitterUVW, w: emitterRadius
        glm::ivec4 misc;            // x: gridResolution, y: volumeType
    };

    static uint32_t div_round_up(uint32_t x, uint32_t d)
    {
        return (x + d - 1u) / d;
    }

    static uint32_t hash_u32(uint32_t x)
    {
        x ^= x >> 16;
        x *= 0x7feb352du;
        x ^= x >> 15;
        x *= 0x846ca68bu;
        x ^= x >> 16;
        return x;
    }

    static float hash3_to_unit_float(int x, int y, int z)
    {
        uint32_t h = 0u;
        h ^= hash_u32(static_cast<uint32_t>(x) * 73856093u);
        h ^= hash_u32(static_cast<uint32_t>(y) * 19349663u);
        h ^= hash_u32(static_cast<uint32_t>(z) * 83492791u);
        // 24-bit mantissa-ish to [0,1)
        return static_cast<float>(h & 0x00FFFFFFu) / static_cast<float>(0x01000000u);
    }

    static float smoothstep01(float x)
    {
        x = std::clamp(x, 0.0f, 1.0f);
        return x * x * (3.0f - 2.0f * x);
    }

    static float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    static float value_noise3(float x, float y, float z)
    {
        const int xi0 = static_cast<int>(std::floor(x));
        const int yi0 = static_cast<int>(std::floor(y));
        const int zi0 = static_cast<int>(std::floor(z));
        const int xi1 = xi0 + 1;
        const int yi1 = yi0 + 1;
        const int zi1 = zi0 + 1;

        const float tx = smoothstep01(x - static_cast<float>(xi0));
        const float ty = smoothstep01(y - static_cast<float>(yi0));
        const float tz = smoothstep01(z - static_cast<float>(zi0));

        const float c000 = hash3_to_unit_float(xi0, yi0, zi0);
        const float c100 = hash3_to_unit_float(xi1, yi0, zi0);
        const float c010 = hash3_to_unit_float(xi0, yi1, zi0);
        const float c110 = hash3_to_unit_float(xi1, yi1, zi0);
        const float c001 = hash3_to_unit_float(xi0, yi0, zi1);
        const float c101 = hash3_to_unit_float(xi1, yi0, zi1);
        const float c011 = hash3_to_unit_float(xi0, yi1, zi1);
        const float c111 = hash3_to_unit_float(xi1, yi1, zi1);

        const float x00 = lerp(c000, c100, tx);
        const float x10 = lerp(c010, c110, tx);
        const float x01 = lerp(c001, c101, tx);
        const float x11 = lerp(c011, c111, tx);

        const float y0 = lerp(x00, x10, ty);
        const float y1 = lerp(x01, x11, ty);

        return lerp(y0, y1, tz);
    }

    static float fbm3(float x, float y, float z)
    {
        float sum = 0.0f;
        float amp = 0.55f;
        float freq = 1.0f;
        for (int i = 0; i < 4; ++i)
        {
            sum += amp * value_noise3(x * freq, y * freq, z * freq);
            freq *= 2.02f;
            amp *= 0.5f;
        }
        return std::clamp(sum, 0.0f, 1.0f);
    }
}

void CloudPass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() || !_context->pipelines ||
        !_context->getResources() || !_context->getAssets())
    {
        return;
    }

    VkDevice device = _context->getDevice()->device();

    // Set 1 layout: HDR input, gbuffer position, voxel density SSBO.
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // hdrInput
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // posTex
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);         // voxelDensity
        _inputSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("clouds.frag.spv");
    info.setLayouts = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(), // set = 0 (sceneData UBO + optional TLAS)
        _inputSetLayout                                         // set = 1 (inputs + voxel grid)
    };

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pcr.offset = 0;
    pcr.size = sizeof(VolumePush);
    info.pushConstants = {pcr};

    info.configure = [this](PipelineBuilder &b)
    {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_depthtest();
        b.disable_blending();
        if (_context && _context->getSwapchain())
        {
            b.set_color_attachment_format(_context->getSwapchain()->drawImage().imageFormat);
        }
    };

    _context->pipelines->createGraphicsPipeline("clouds", info);

    // Optional voxel advection compute pipeline (used when VoxelVolumeSettings::animateVoxels is enabled).
    {
        ComputePipelineCreateInfo ci{};
        ci.shaderPath = _context->getAssets()->shaderPath("cloud_voxel_advect.comp.spv");
        ci.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER};
        ci.pushConstantSize = sizeof(VolumeVoxelPush);
        ci.pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;
        _context->pipelines->createComputePipeline("clouds.voxel_advect", ci);
        _context->pipelines->createComputeInstance("clouds.voxel_advect", "clouds.voxel_advect");
    }

    // Voxel buffers are allocated lazily per-volume when enabled.
}

void CloudPass::cleanup()
{
    if (_context && _context->getDevice() && _inputSetLayout)
    {
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _inputSetLayout, nullptr);
        _inputSetLayout = VK_NULL_HANDLE;
    }

    if (_context && _context->getResources())
    {
        ResourceManager *rm = _context->getResources();
        for (auto &vol : _volumes)
        {
            for (auto &buf : vol.voxelDensity)
            {
                if (buf.buffer != VK_NULL_HANDLE)
                {
                    rm->destroy_buffer(buf);
                }
                buf = {};
            }
            vol.voxelReadIndex = 0;
            vol.voxelDensitySize = 0;
            vol.gridResolution = 0;
        }
    }

    _deletionQueue.flush();
}

void CloudPass::execute(VkCommandBuffer)
{
    // Executed via render graph; nothing to do here.
}

RGImageHandle CloudPass::register_graph(RenderGraph *graph, RGImageHandle hdrInput, RGImageHandle gbufPos)
{
    if (!graph || !hdrInput.valid() || !gbufPos.valid())
    {
        return hdrInput;
    }
    if (!_context || !_context->enableVolumetrics)
    {
        return hdrInput;
    }

    update_time_and_origin_delta();

    const float origin_delta_len2 = glm::dot(_origin_delta_local, _origin_delta_local);
    const bool origin_delta_valid = std::isfinite(origin_delta_len2) && origin_delta_len2 > 0.0f;

    std::array<VkBuffer, MAX_VOLUMES> voxForRender{};
    std::array<VkDeviceSize, MAX_VOLUMES> voxSize{};
    std::array<uint32_t, MAX_VOLUMES> gridRes{};
    std::array<VoxelVolumeSettings, MAX_VOLUMES> settings{};
    std::array<bool, MAX_VOLUMES> active{};

    for (uint32_t i = 0; i < MAX_VOLUMES; ++i)
    {
        VoxelVolumeSettings &vs = _context->voxelVolumes[i];
        if (!vs.enabled)
        {
            continue;
        }

        // Keep the volume stable in world-space while render-local origin shifts.
        if (!vs.followCameraXZ)
        {
            if (origin_delta_valid)
            {
                vs.volumeCenterLocal -= _origin_delta_local;
            }

            const float vel_len2 = glm::dot(vs.volumeVelocityLocal, vs.volumeVelocityLocal);
            const bool vel_valid = std::isfinite(vel_len2) && vel_len2 > 0.0f;
            if (vel_valid && _dt_sec > 0.0f)
            {
                vs.volumeCenterLocal += vs.volumeVelocityLocal * _dt_sec;
            }
        }

        VolumeBuffers &bufs = _volumes[i];

        const uint32_t wantRes = std::max(4u, vs.gridResolution);
        if (wantRes != bufs.gridResolution ||
            bufs.voxelDensity[0].buffer == VK_NULL_HANDLE ||
            bufs.voxelDensity[1].buffer == VK_NULL_HANDLE)
        {
            rebuild_voxel_density(i, wantRes, vs);
        }

        const VkDeviceSize size = bufs.voxelDensitySize;
        const VkBuffer voxRead = bufs.voxelDensity[bufs.voxelReadIndex].buffer;
        const VkBuffer voxWrite = bufs.voxelDensity[1u - bufs.voxelReadIndex].buffer;

        VkBuffer voxRender = voxRead;

        if (vs.animateVoxels && voxRead != VK_NULL_HANDLE && voxWrite != VK_NULL_HANDLE && size > 0 && bufs.gridResolution > 0)
        {
            const std::string passName = "Volumetrics.VoxelUpdate." + std::to_string(i);

            graph->add_pass(
                passName.c_str(),
                RGPassType::Compute,
                [voxRead, voxWrite, size, i](RGPassBuilder &builder, EngineContext *)
                {
                    const std::string inName = "volumetrics.voxel_density_in." + std::to_string(i);
                    const std::string outName = "volumetrics.voxel_density_out." + std::to_string(i);
                    builder.read_buffer(voxRead, RGBufferUsage::StorageRead, size, inName.c_str());
                    builder.write_buffer(voxWrite, RGBufferUsage::StorageReadWrite, size, outName.c_str());
                },
                [this, i, voxRead, voxWrite, size, vs](VkCommandBuffer cmd, const RGPassResources &, EngineContext *ctx)
                {
                    EngineContext *ctxLocal = ctx ? ctx : _context;
                    if (!ctxLocal || !ctxLocal->pipelines) return;

                    VolumeBuffers &localBufs = _volumes[i];
                    const uint32_t res = localBufs.gridResolution;
                    if (res == 0 || size == 0) return;

                    // Bind the ping-pong buffers for this frame.
                    ctxLocal->pipelines->setComputeInstanceBuffer("clouds.voxel_advect",
                                                                 0,
                                                                voxRead,
                                                                  size,
                                                                 VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                 0);
                    ctxLocal->pipelines->setComputeInstanceBuffer("clouds.voxel_advect",
                                                                 1,
                                                                 voxWrite,
                                                                 size,
                                                                 VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                 0);

                    VolumeVoxelPush pc{};
                    pc.wind_dt = glm::vec4(vs.windVelocityLocal, _dt_sec);
                    const glm::vec3 volSize = glm::max(vs.volumeHalfExtents * 2.0f, glm::vec3(0.001f));
                    pc.volume_size_time = glm::vec4(volSize, _time_sec);
                    pc.sim_params = glm::vec4(std::max(0.0f, vs.dissipation),
                                              std::max(0.0f, vs.noiseStrength),
                                              std::max(0.001f, vs.noiseScale),
                                              vs.noiseSpeed);
                    pc.emitter_params = glm::vec4(glm::clamp(vs.emitterUVW, glm::vec3(0.0f), glm::vec3(1.0f)),
                                                  std::max(0.0f, vs.emitterRadius));
                    pc.misc = glm::ivec4(static_cast<int>(res), static_cast<int>(vs.type), 0, 0);

                    // Match shader local_size_{x,y,z} = 8.
                    ComputeDispatchInfo di{};
                    di.groupCountX = div_round_up(res, 8);
                    di.groupCountY = div_round_up(res, 8);
                    di.groupCountZ = div_round_up(res, 8);
                    di.pushConstants = &pc;
                    di.pushConstantSize = sizeof(pc);

                    ctxLocal->pipelines->dispatchComputeInstance(cmd, "clouds.voxel_advect", di);
                });

            voxRender = voxWrite;
            bufs.voxelReadIndex = 1u - bufs.voxelReadIndex;
        }

        voxForRender[i] = voxRender;
        voxSize[i] = size;
        gridRes[i] = bufs.gridResolution;
        settings[i] = vs;
        active[i] = (voxRender != VK_NULL_HANDLE && size > 0 && bufs.gridResolution > 0);
    }

    RGImageHandle current = hdrInput;
    for (uint32_t i = 0; i < MAX_VOLUMES; ++i)
    {
        if (!active[i])
        {
            continue;
        }

        RGImageDesc desc{};
        desc.name = "hdr.volumetrics." + std::to_string(i);
        desc.format = (_context && _context->getSwapchain()) ? _context->getSwapchain()->drawImage().imageFormat : VK_FORMAT_R16G16B16A16_SFLOAT;
        desc.extent = _context->getDrawExtent();
        desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        RGImageHandle hdrOutput = graph->create_image(desc);

        const VkBuffer voxBuf = voxForRender[i];
        const VkDeviceSize size = voxSize[i];
        const uint32_t res = gridRes[i];
        const VoxelVolumeSettings vs = settings[i];
        const RGImageHandle hdrIn = current;

        const std::string passName = "Volumetrics." + std::to_string(i);
        graph->add_pass(
            passName.c_str(),
            RGPassType::Graphics,
            [hdrIn, gbufPos, hdrOutput, voxBuf, size, i](RGPassBuilder &builder, EngineContext *)
            {
                builder.read(hdrIn, RGImageUsage::SampledFragment);
                builder.read(gbufPos, RGImageUsage::SampledFragment);
                if (voxBuf != VK_NULL_HANDLE)
                {
                    const std::string voxName = "volumetrics.voxel_density." + std::to_string(i);
                    builder.read_buffer(voxBuf, RGBufferUsage::StorageRead, size, voxName.c_str());
                }
                builder.write_color(hdrOutput, false /*load*/);
            },
            [this, hdrIn, gbufPos, voxBuf, size, res, vs](VkCommandBuffer cmd, const RGPassResources &resGraph, EngineContext *ctx)
            {
                draw_volume(cmd, ctx, resGraph, hdrIn, gbufPos, vs, res, voxBuf, size);
            });

        current = hdrOutput;
    }

    return current;
}

void CloudPass::update_time_and_origin_delta()
{
    _dt_sec = 0.0f;
    _origin_delta_local = glm::vec3(0.0f);

    if (!_context || !_context->scene)
    {
        return;
    }

    _dt_sec = _context->scene->getDeltaTime();
    if (!std::isfinite(_dt_sec)) _dt_sec = 0.0f;
    _dt_sec = std::clamp(_dt_sec, 0.0f, 0.1f);
    _time_sec += _dt_sec;

    const WorldVec3 origin_world = _context->scene->get_world_origin();
    if (_has_prev_origin)
    {
        const WorldVec3 delta_world = origin_world - _prev_origin_world;
        _origin_delta_local = glm::vec3(delta_world);
    }
    _prev_origin_world = origin_world;
    _has_prev_origin = true;
}

void CloudPass::rebuild_voxel_density(uint32_t volume_index, uint32_t resolution, const VoxelVolumeSettings &settings)
{
    if (!_context || !_context->getResources() || !_context->getDevice())
    {
        return;
    }
    if (volume_index >= MAX_VOLUMES)
    {
        return;
    }

    resolution = std::max(4u, resolution);

    ResourceManager *resourceManager = _context->getResources();
    DeviceManager *deviceManager = _context->getDevice();
    VolumeBuffers &vol = _volumes[volume_index];

    const VkDeviceSize voxelCount = static_cast<VkDeviceSize>(resolution) *
                                   static_cast<VkDeviceSize>(resolution) *
                                   static_cast<VkDeviceSize>(resolution);
    const VkDeviceSize sizeBytes = voxelCount * sizeof(float);

    // Stage initial density data to a GPU-only SSBO (and duplicate into both ping-pong buffers).
    AllocatedBuffer staging = resourceManager->create_buffer(
        static_cast<size_t>(sizeBytes),
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VMA_MEMORY_USAGE_CPU_ONLY);

    auto *dst = static_cast<float *>(staging.info.pMappedData);
    if (dst && sizeBytes > 0)
    {
        if (settings.type != VoxelVolumeType::Clouds)
        {
            std::fill(dst, dst + static_cast<size_t>(voxelCount), 0.0f);
        }
        else
        {
            for (uint32_t z = 0; z < resolution; ++z)
            {
                for (uint32_t y = 0; y < resolution; ++y)
                {
                    const float fy = (resolution > 1) ? (static_cast<float>(y) / static_cast<float>(resolution - 1)) : 0.0f;
                    // Height falloff to keep density within a layer.
                    const float low = smoothstep01((fy - 0.00f) / 0.18f);
                    const float high = 1.0f - smoothstep01((fy - 0.78f) / 0.22f);
                    const float heightShape = std::clamp(low * high, 0.0f, 1.0f);

                    for (uint32_t x = 0; x < resolution; ++x)
                    {
                        const float fx = (resolution > 1) ? (static_cast<float>(x) / static_cast<float>(resolution - 1)) : 0.0f;
                        const float fz = (resolution > 1) ? (static_cast<float>(z) / static_cast<float>(resolution - 1)) : 0.0f;

                        // Low-frequency FBM noise in [0,1].
                        float n = fbm3(fx * 6.0f, fy * 6.0f, fz * 6.0f);

                        // Add a soft "blob" bias near center to avoid uniform fog.
                        const float cx = fx * 2.0f - 1.0f;
                        const float cy = fy * 2.0f - 1.0f;
                        const float cz = fz * 2.0f - 1.0f;
                        const float r2 = cx * cx + cy * cy + cz * cz;
                        const float blob = std::clamp(1.0f - r2 * 0.85f, 0.0f, 1.0f);

                        float density = std::clamp(n * heightShape, 0.0f, 1.0f);
                        density = std::clamp(density + 0.35f * blob * heightShape, 0.0f, 1.0f);

                        const uint32_t idx = x + y * resolution + z * resolution * resolution;
                        dst[idx] = density;
                    }
                }
            }
        }

        vmaFlushAllocation(deviceManager->allocator(), staging.allocation, 0, sizeBytes);
    }

    AllocatedBuffer newA = resourceManager->create_buffer(
        static_cast<size_t>(sizeBytes),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VMA_MEMORY_USAGE_GPU_ONLY);
    AllocatedBuffer newB = resourceManager->create_buffer(
        static_cast<size_t>(sizeBytes),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VMA_MEMORY_USAGE_GPU_ONLY);

    resourceManager->immediate_submit([&](VkCommandBuffer cmd) {
        VkBufferCopy region{};
        region.srcOffset = 0;
        region.dstOffset = 0;
        region.size = sizeBytes;
        vkCmdCopyBuffer(cmd, staging.buffer, newA.buffer, 1, &region);
        vkCmdCopyBuffer(cmd, staging.buffer, newB.buffer, 1, &region);
    });

    resourceManager->destroy_buffer(staging);

    if (vol.voxelDensity[0].buffer != VK_NULL_HANDLE || vol.voxelDensity[1].buffer != VK_NULL_HANDLE)
    {
        AllocatedBuffer old0 = vol.voxelDensity[0];
        AllocatedBuffer old1 = vol.voxelDensity[1];
        // Defer destruction if we are mid-frame; otherwise destroy immediately.
        if (_context->currentFrame)
        {
            _context->currentFrame->_deletionQueue.push_function([resourceManager, old0, old1]()
            {
                if (old0.buffer != VK_NULL_HANDLE) resourceManager->destroy_buffer(old0);
                if (old1.buffer != VK_NULL_HANDLE) resourceManager->destroy_buffer(old1);
            });
        }
        else
        {
            if (old0.buffer != VK_NULL_HANDLE) resourceManager->destroy_buffer(old0);
            if (old1.buffer != VK_NULL_HANDLE) resourceManager->destroy_buffer(old1);
        }
    }

    vol.voxelDensity[0] = newA;
    vol.voxelDensity[1] = newB;
    vol.voxelReadIndex = 0;
    vol.voxelDensitySize = sizeBytes;
    vol.gridResolution = resolution;
}

void CloudPass::draw_volume(VkCommandBuffer cmd,
                            EngineContext *context,
                            const RGPassResources &resources,
                            RGImageHandle hdrInput,
                            RGImageHandle gbufPos,
                            const VoxelVolumeSettings &settings,
                            uint32_t grid_resolution,
                            VkBuffer voxelBuffer,
                            VkDeviceSize voxelSize)
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    PipelineManager *pipelineManager = ctxLocal->pipelines;
    if (!resourceManager || !deviceManager || !descriptorLayouts || !pipelineManager) return;

    VkImageView hdrView = resources.image_view(hdrInput);
    VkImageView posView = resources.image_view(gbufPos);
    if (hdrView == VK_NULL_HANDLE || posView == VK_NULL_HANDLE) return;

    if (voxelBuffer == VK_NULL_HANDLE || voxelSize == 0 || grid_resolution == 0)
    {
        return;
    }

    if (!pipelineManager->getGraphics("clouds", _pipeline, _pipelineLayout))
    {
        return;
    }

    // Scene UBO (set=0, binding=0) – mirror SSR/lighting behavior.
    AllocatedBuffer sceneBuf = resourceManager->create_buffer(
        sizeof(GPUSceneData),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VMA_MEMORY_USAGE_CPU_TO_GPU);
    ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, sceneBuf]()
    {
        resourceManager->destroy_buffer(sceneBuf);
    });

    auto *sceneUniformData = static_cast<GPUSceneData *>(sceneBuf.info.pMappedData);
    if (sceneUniformData)
    {
        *sceneUniformData = ctxLocal->getSceneData();
        vmaFlushAllocation(deviceManager->allocator(), sceneBuf.allocation, 0, sizeof(GPUSceneData));
    }

    VkDescriptorSet globalSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), descriptorLayouts->gpuSceneDataLayout());
    {
        DescriptorWriter writer;
        writer.write_buffer(0, sceneBuf.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        writer.update_set(deviceManager->device(), globalSet);
    }

    VkDescriptorSet inputSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), _inputSetLayout);
    {
        DescriptorWriter writer;
        writer.write_image(0, hdrView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(1, posView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_buffer(2, voxelBuffer, static_cast<size_t>(voxelSize), 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
        writer.update_set(deviceManager->device(), inputSet);
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &globalSet, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 1, 1, &inputSet, 0, nullptr);

    VolumePush push{};
    push.volume_center_follow = glm::vec4(settings.volumeCenterLocal, settings.followCameraXZ ? 1.0f : 0.0f);
    push.volume_half_extents = glm::vec4(glm::max(settings.volumeHalfExtents, glm::vec3(0.01f)), 0.0f);
    push.density_params = glm::vec4(std::max(0.0f, settings.densityScale),
                                    std::clamp(settings.coverage, 0.0f, 0.99f),
                                    std::max(0.0f, settings.extinction),
                                    _time_sec);
    push.scatter_params = glm::vec4(glm::clamp(settings.albedo, glm::vec3(0.0f), glm::vec3(1.0f)),
                                    std::max(0.0f, settings.scatterStrength));
    push.emission_params = glm::vec4(glm::max(settings.emissionColor, glm::vec3(0.0f)),
                                     std::max(0.0f, settings.emissionStrength));
    push.misc = glm::ivec4(std::clamp(settings.stepCount, 8, 256),
                           static_cast<int>(grid_resolution),
                           static_cast<int>(settings.type),
                           0);
    vkCmdPushConstants(cmd, _pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(VolumePush), &push);

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport vp{0.f, 0.f, (float)extent.width, (float)extent.height, 0.f, 1.f};
    VkRect2D sc{{0, 0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}
