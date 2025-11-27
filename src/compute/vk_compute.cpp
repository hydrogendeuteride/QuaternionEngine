#include <compute/vk_compute.h>
#include <core/context.h>
#include <render/pipelines.h>
#include <core/util/initializers.h>
#include <iostream>

#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/frame/resources.h"

ComputeBinding ComputeBinding::uniformBuffer(uint32_t binding, VkBuffer buffer, VkDeviceSize size, VkDeviceSize offset)
{
    ComputeBinding result;
    result.binding = binding;
    result.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    result.buffer.buffer = buffer;
    result.buffer.offset = offset;
    result.buffer.size = size;
    return result;
}

ComputeBinding ComputeBinding::storageBuffer(uint32_t binding, VkBuffer buffer, VkDeviceSize size, VkDeviceSize offset)
{
    ComputeBinding result;
    result.binding = binding;
    result.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    result.buffer.buffer = buffer;
    result.buffer.offset = offset;
    result.buffer.size = size;
    return result;
}

ComputeBinding ComputeBinding::sampledImage(uint32_t binding, VkImageView imageView, VkSampler sampler,
                                            VkImageLayout layout)
{
    ComputeBinding result;
    result.binding = binding;
    result.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    result.storageImage.imageView = imageView;
    result.image.sampler = sampler;
    result.storageImage.layout = layout;
    return result;
}

ComputeBinding ComputeBinding::storeImage(uint32_t binding, VkImageView imageView, VkImageLayout layout)
{
    ComputeBinding result;
    result.binding = binding;
    result.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    result.storageImage.imageView = imageView;
    result.storageImage.layout = layout;
    return result;
}

ComputePipeline::~ComputePipeline()
{
    cleanup();
}

ComputePipeline::ComputePipeline(ComputePipeline &&other) noexcept
    : device(other.device)
      , pipeline(other.pipeline)
      , layout(other.layout)
      , descriptorLayout(other.descriptorLayout)
{
    other.device = VK_NULL_HANDLE;
    other.pipeline = VK_NULL_HANDLE;
    other.layout = VK_NULL_HANDLE;
    other.descriptorLayout = VK_NULL_HANDLE;
}

ComputePipeline &ComputePipeline::operator=(ComputePipeline &&other) noexcept
{
    if (this != &other)
    {
        cleanup();

        device = other.device;
        pipeline = other.pipeline;
        layout = other.layout;
        descriptorLayout = other.descriptorLayout;

        other.device = VK_NULL_HANDLE;
        other.pipeline = VK_NULL_HANDLE;
        other.layout = VK_NULL_HANDLE;
        other.descriptorLayout = VK_NULL_HANDLE;
    }
    return *this;
}

void ComputePipeline::cleanup()
{
    if (device != VK_NULL_HANDLE)
    {
        if (pipeline != VK_NULL_HANDLE)
        {
            vkDestroyPipeline(device, pipeline, nullptr);
        }
        if (layout != VK_NULL_HANDLE)
        {
            vkDestroyPipelineLayout(device, layout, nullptr);
        }
        if (descriptorLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, descriptorLayout, nullptr);
        }
    }

    device = VK_NULL_HANDLE;
    pipeline = VK_NULL_HANDLE;
    layout = VK_NULL_HANDLE;
    descriptorLayout = VK_NULL_HANDLE;
}

ComputeManager::~ComputeManager()
{
    cleanup();
}

void ComputeManager::init(EngineContext *context)
{
    this->context = context;

    std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> poolSizes = {
        {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 4},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 4},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4},
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 4}
    };

    descriptorAllocator.init(context->getDevice()->device(), 100, poolSizes);
}

void ComputeManager::cleanup()
{
    pipelines.clear();
    // Destroy instances and their owned resources
    if (context)
    {
        for (auto &kv : instances)
        {
            for (auto &img : kv.second.ownedImages)
            {
                context->getResources()->destroy_image(img);
            }
            for (auto &buf : kv.second.ownedBuffers)
            {
                context->getResources()->destroy_buffer(buf);
            }
        }
        instances.clear();
    }

    if (context)
    {
        descriptorAllocator.destroy_pools(context->getDevice()->device());
    }

    context = nullptr;
}

bool ComputeManager::registerPipeline(const std::string &name, const ComputePipelineCreateInfo &createInfo)
{
    if (pipelines.find(name) != pipelines.end())
    {
        std::cerr << "Pipeline '" << name << "' already exists!" << std::endl;
        return false;
    }

    return createPipeline(name, createInfo);
}

void ComputeManager::unregisterPipeline(const std::string &name)
{
    pipelines.erase(name);
}

bool ComputeManager::hasPipeline(const std::string &name) const
{
    return pipelines.find(name) != pipelines.end();
}

void ComputeManager::dispatch(VkCommandBuffer cmd, const std::string &pipelineName,
                              const ComputeDispatchInfo &dispatchInfo)
{
    auto it = pipelines.find(pipelineName);
    if (it == pipelines.end())
    {
        std::cerr << "Pipeline '" << pipelineName << "' not found!" << std::endl;
        return;
    }

    const ComputePipeline &pipeline = it->second;

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline.getPipeline());

    if (!dispatchInfo.bindings.empty())
    {
        VkDescriptorSet descriptorSet = allocateDescriptorSet(pipeline, dispatchInfo.bindings);
        updateDescriptorSet(descriptorSet, dispatchInfo.bindings);

        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline.getLayout(),
                                0, 1, &descriptorSet, 0, nullptr);
    }

    if (dispatchInfo.pushConstants && dispatchInfo.pushConstantSize > 0)
    {
        vkCmdPushConstants(cmd, pipeline.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT,
                           0, dispatchInfo.pushConstantSize, dispatchInfo.pushConstants);
    }

    insertBarriers(cmd, dispatchInfo);

    vkCmdDispatch(cmd, dispatchInfo.groupCountX, dispatchInfo.groupCountY, dispatchInfo.groupCountZ);
}

void ComputeManager::dispatchImmediate(const std::string &pipelineName, const ComputeDispatchInfo &dispatchInfo)
{
    context->getResources()->immediate_submit([this, pipelineName, dispatchInfo](VkCommandBuffer cmd) {
        dispatch(cmd, pipelineName, dispatchInfo);
    });
}

bool ComputeManager::createInstance(const std::string &instanceName, const std::string &pipelineName)
{
    if (instances.find(instanceName) != instances.end())
    {
        std::cerr << "Compute instance '" << instanceName << "' already exists!" << std::endl;
        return false;
    }
    auto it = pipelines.find(pipelineName);
    if (it == pipelines.end())
    {
        std::cerr << "Pipeline '" << pipelineName << "' not found for instance!" << std::endl;
        return false;
    }

    ComputeInstance inst{};
    inst.pipelineName = pipelineName;

    inst.descriptorSet = descriptorAllocator.allocate(context->getDevice()->device(), it->second.descriptorLayout);

    instances.emplace(instanceName, std::move(inst));
    return true;
}

void ComputeManager::destroyInstance(const std::string &instanceName)
{
    auto it = instances.find(instanceName);
    if (it == instances.end()) return;

    for (auto &img : it->second.ownedImages)
        context->getResources()->destroy_image(img);
    for (auto &buf : it->second.ownedBuffers)
        context->getResources()->destroy_buffer(buf);

    instances.erase(it);
}

static void upsert_binding(std::vector<ComputeBinding> &bindings, const ComputeBinding &b)
{
    for (auto &x : bindings)
    {
        if (x.binding == b.binding)
        {
            x = b;
            return;
        }
    }
    bindings.push_back(b);
}

bool ComputeManager::setInstanceBinding(const std::string &instanceName, const ComputeBinding &binding)
{
    auto it = instances.find(instanceName);
    if (it == instances.end()) return false;
    upsert_binding(it->second.bindings, binding);
    return true;
}

bool ComputeManager::setInstanceStorageImage(const std::string &instanceName, uint32_t binding, VkImageView view,
                                             VkImageLayout layout)
{
    return setInstanceBinding(instanceName, ComputeBinding::storeImage(binding, view, layout));
}

bool ComputeManager::setInstanceSampledImage(const std::string &instanceName, uint32_t binding, VkImageView view,
                                             VkSampler sampler, VkImageLayout layout)
{
    return setInstanceBinding(instanceName, ComputeBinding::sampledImage(binding, view, sampler, layout));
}

bool ComputeManager::setInstanceBuffer(const std::string &instanceName, uint32_t binding, VkBuffer buffer,
                                       VkDeviceSize size, VkDescriptorType type, VkDeviceSize offset)
{
    ComputeBinding b{};
    b.binding = binding;
    b.type = type;
    b.buffer.buffer = buffer;
    b.buffer.size = size;
    b.buffer.offset = offset;
    return setInstanceBinding(instanceName, b);
}

AllocatedImage ComputeManager::createAndBindStorageImage(const std::string &instanceName, uint32_t binding,
                                                        VkExtent3D extent, VkFormat format, VkImageLayout layout,
                                                        VkImageUsageFlags usage)
{
    auto it = instances.find(instanceName);
    if (it == instances.end()) return {};

    AllocatedImage img = context->getResources()->create_image(extent, format, usage);
    it->second.ownedImages.push_back(img);
    setInstanceStorageImage(instanceName, binding, img.imageView, layout);
    return img;
}

AllocatedBuffer ComputeManager::createAndBindStorageBuffer(const std::string &instanceName, uint32_t binding,
                                                          VkDeviceSize size, VkBufferUsageFlags usage,
                                                          VmaMemoryUsage memUsage)
{
    auto it = instances.find(instanceName);
    if (it == instances.end()) return {};
    AllocatedBuffer buf = context->getResources()->create_buffer(size, usage, memUsage);
    it->second.ownedBuffers.push_back(buf);
    setInstanceBuffer(instanceName, binding, buf.buffer, size, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 0);
    return buf;
}

bool ComputeManager::updateInstanceDescriptorSet(const std::string &instanceName)
{
    auto it = instances.find(instanceName);
    if (it == instances.end()) return false;
    updateDescriptorSet(it->second.descriptorSet, it->second.bindings);
    return true;
}

void ComputeManager::dispatchInstance(VkCommandBuffer cmd, const std::string &instanceName,
                                      const ComputeDispatchInfo &dispatchInfo)
{
    auto it = instances.find(instanceName);
    if (it == instances.end())
    {
        std::cerr << "Compute instance '" << instanceName << "' not found!" << std::endl;
        return;
    }
    auto pit = pipelines.find(it->second.pipelineName);
    if (pit == pipelines.end())
    {
        std::cerr << "Pipeline '" << it->second.pipelineName << "' not found for instance dispatch!" << std::endl;
        return;
    }

    const ComputePipeline &pipeline = pit->second;

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline.getPipeline());

    // Allocate a transient per-frame descriptor set to avoid updating a set
    // that might still be in use by a previous in-flight frame.
    VkDescriptorSet transientSet = context->currentFrame
        ? context->currentFrame->_frameDescriptors.allocate(context->getDevice()->device(), pipeline.descriptorLayout)
        : VK_NULL_HANDLE;
    if (transientSet == VK_NULL_HANDLE)
    {
        // Fallback to instance-owned set if per-frame allocator unavailable
        updateDescriptorSet(it->second.descriptorSet, it->second.bindings);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline.getLayout(), 0, 1, &it->second.descriptorSet, 0, nullptr);
    }
    else
    {
        updateDescriptorSet(transientSet, it->second.bindings);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline.getLayout(), 0, 1, &transientSet, 0, nullptr);
    }

    if (dispatchInfo.pushConstants && dispatchInfo.pushConstantSize > 0)
    {
        vkCmdPushConstants(cmd, pipeline.getLayout(), VK_SHADER_STAGE_COMPUTE_BIT, 0, dispatchInfo.pushConstantSize,
                           dispatchInfo.pushConstants);
    }
    insertBarriers(cmd, dispatchInfo);
    vkCmdDispatch(cmd, dispatchInfo.groupCountX, dispatchInfo.groupCountY, dispatchInfo.groupCountZ);
}

uint32_t ComputeManager::calculateGroupCount(uint32_t workItems, uint32_t localSize)
{
    return (workItems + localSize - 1) / localSize;
}

ComputeDispatchInfo ComputeManager::createDispatch2D(uint32_t width, uint32_t height, uint32_t localSizeX,
                                                     uint32_t localSizeY)
{
    ComputeDispatchInfo info;
    info.groupCountX = calculateGroupCount(width, localSizeX);
    info.groupCountY = calculateGroupCount(height, localSizeY);
    info.groupCountZ = 1;
    return info;
}

ComputeDispatchInfo ComputeManager::createDispatch3D(uint32_t width, uint32_t height, uint32_t depth,
                                                     uint32_t localSizeX, uint32_t localSizeY, uint32_t localSizeZ)
{
    ComputeDispatchInfo info;
    info.groupCountX = calculateGroupCount(width, localSizeX);
    info.groupCountY = calculateGroupCount(height, localSizeY);
    info.groupCountZ = calculateGroupCount(depth, localSizeZ);
    return info;
}

void ComputeManager::clearImage(VkCommandBuffer cmd, VkImageView imageView, const glm::vec4 &clearColor)
{
    if (!hasPipeline("clear_image"))
    {
        ComputePipelineCreateInfo createInfo;
        createInfo.shaderPath = "../shaders/clear_image.comp.spv";
        createInfo.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE};
        createInfo.pushConstantSize = sizeof(glm::vec4);
        registerPipeline("clear_image", createInfo);
    }

    ComputeDispatchInfo dispatchInfo;
    dispatchInfo.bindings.push_back(ComputeBinding::storeImage(0, imageView));
    dispatchInfo.pushConstants = &clearColor;
    dispatchInfo.pushConstantSize = sizeof(glm::vec4);

    dispatchInfo.groupCountX = 64;
    dispatchInfo.groupCountY = 64;
    dispatchInfo.groupCountZ = 1;

    dispatch(cmd, "clear_image", dispatchInfo);
}

void ComputeManager::copyBuffer(VkCommandBuffer cmd, VkBuffer src, VkBuffer dst, VkDeviceSize size,
                                VkDeviceSize srcOffset, VkDeviceSize dstOffset)
{
    if (!hasPipeline("copy_buffer"))
    {
        ComputePipelineCreateInfo createInfo;
        createInfo.shaderPath = "../shaders/copy_buffer.comp.spv";
        createInfo.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER};
        createInfo.pushConstantSize = sizeof(uint32_t) * 3;
        registerPipeline("copy_buffer", createInfo);
    }

    ComputeDispatchInfo dispatchInfo;
    dispatchInfo.bindings.push_back(ComputeBinding::storageBuffer(0, src, size, srcOffset));
    dispatchInfo.bindings.push_back(ComputeBinding::storageBuffer(1, dst, size, dstOffset));

    uint32_t pushData[3] = {(uint32_t) size, (uint32_t) srcOffset, (uint32_t) dstOffset};
    dispatchInfo.pushConstants = pushData;
    dispatchInfo.pushConstantSize = sizeof(pushData);

    dispatchInfo.groupCountX = calculateGroupCount(size / 4, 256);
    dispatchInfo.groupCountY = 1;
    dispatchInfo.groupCountZ = 1;

    dispatch(cmd, "copy_buffer", dispatchInfo);
}

bool ComputeManager::createPipeline(const std::string &name, const ComputePipelineCreateInfo &createInfo)
{
    ComputePipeline computePipeline;
    computePipeline.device = context->getDevice()->device();

    VkShaderModule shaderModule;
    if (!vkutil::load_shader_module(createInfo.shaderPath.c_str(), context->getDevice()->device(), &shaderModule))
    {
        std::cerr << "Failed to load compute shader: " << createInfo.shaderPath << std::endl;
        return false;
    }

    if (!createInfo.descriptorTypes.empty())
    {
        DescriptorLayoutBuilder layoutBuilder;
        for (size_t i = 0; i < createInfo.descriptorTypes.size(); ++i)
        {
            layoutBuilder.add_binding(static_cast<uint32_t>(i), createInfo.descriptorTypes[i]);
        }

        // Mark all compute bindings as UPDATE_AFTER_BIND so we can update
        // persistent instance descriptor sets while a previous frame is in-flight.
        std::vector<VkDescriptorBindingFlags> bindingFlags(createInfo.descriptorTypes.size(),
                                                          VK_DESCRIPTOR_BINDING_UPDATE_AFTER_BIND_BIT);
        VkDescriptorSetLayoutBindingFlagsCreateInfo flagsCI{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_BINDING_FLAGS_CREATE_INFO };
        flagsCI.bindingCount = static_cast<uint32_t>(bindingFlags.size());
        flagsCI.pBindingFlags = bindingFlags.data();

        computePipeline.descriptorLayout = layoutBuilder.build(
            context->getDevice()->device(),
            VK_SHADER_STAGE_COMPUTE_BIT,
            &flagsCI,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    VkPipelineLayoutCreateInfo layoutInfo = vkinit::pipeline_layout_create_info();

    if (computePipeline.descriptorLayout != VK_NULL_HANDLE)
    {
        layoutInfo.setLayoutCount = 1;
        layoutInfo.pSetLayouts = &computePipeline.descriptorLayout;
    }

    VkPushConstantRange pushConstantRange = {};
    if (createInfo.pushConstantSize > 0)
    {
        pushConstantRange.offset = 0;
        pushConstantRange.size = createInfo.pushConstantSize;
        pushConstantRange.stageFlags = createInfo.pushConstantStages;

        layoutInfo.pushConstantRangeCount = 1;
        layoutInfo.pPushConstantRanges = &pushConstantRange;
    }

    VK_CHECK(vkCreatePipelineLayout(context->getDevice()->device(), &layoutInfo, nullptr, &computePipeline.layout));

    VkPipelineShaderStageCreateInfo stageInfo = vkinit::pipeline_shader_stage_create_info(
        VK_SHADER_STAGE_COMPUTE_BIT, shaderModule);

    VkSpecializationInfo specializationInfo = {};
    if (!createInfo.specializationEntries.empty())
    {
        specializationInfo.mapEntryCount = createInfo.specializationEntries.size();
        specializationInfo.pMapEntries = createInfo.specializationEntries.data();
        specializationInfo.dataSize = createInfo.specializationData.size() * sizeof(uint32_t);
        specializationInfo.pData = createInfo.specializationData.data();
        stageInfo.pSpecializationInfo = &specializationInfo;
    }

    VkComputePipelineCreateInfo pipelineInfo = {};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineInfo.stage = stageInfo;
    pipelineInfo.layout = computePipeline.layout;

    VK_CHECK(
        vkCreateComputePipelines(context->getDevice()->device(), VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &computePipeline.pipeline))
    ;

    vkDestroyShaderModule(context->getDevice()->device(), shaderModule, nullptr);

    pipelines[name] = std::move(computePipeline);

    return true;
}

VkDescriptorSet ComputeManager::allocateDescriptorSet(const ComputePipeline &pipeline,
                                                      const std::vector<ComputeBinding> &bindings)
{
    if (pipeline.descriptorLayout == VK_NULL_HANDLE)
    {
        return VK_NULL_HANDLE;
    }

    return descriptorAllocator.allocate(context->getDevice()->device(), pipeline.descriptorLayout);
}

void ComputeManager::updateDescriptorSet(VkDescriptorSet descriptorSet, const std::vector<ComputeBinding> &bindings)
{
    if (descriptorSet == VK_NULL_HANDLE)
    {
        return;
    }

    DescriptorWriter writer;

    for (const auto &binding: bindings)
    {
        switch (binding.type)
        {
            case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:
            case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER:
                writer.write_buffer(binding.binding, binding.buffer.buffer, binding.buffer.size,
                                    binding.buffer.offset, binding.type);
                break;

            case VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER:
                writer.write_image(binding.binding, binding.storageImage.imageView, binding.image.sampler,
                                   binding.storageImage.layout, binding.type);
                break;

            case VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:
                writer.write_image(binding.binding, binding.storageImage.imageView, VK_NULL_HANDLE,
                                   binding.storageImage.layout, binding.type);
                break;

            default:
                std::cerr << "Unsupported descriptor type: " << binding.type << std::endl;
                break;
        }
    }

    writer.update_set(context->getDevice()->device(), descriptorSet);
}

void ComputeManager::insertBarriers(VkCommandBuffer cmd, const ComputeDispatchInfo &dispatchInfo)
{
    if (dispatchInfo.memoryBarriers.empty() &&
        dispatchInfo.bufferBarriers.empty() &&
        dispatchInfo.imageBarriers.empty())
    {
        return;
    }

    // Copy and sanitize buffer barriers: replace size=0 with VK_WHOLE_SIZE (VUID 01188)
    std::vector<VkBufferMemoryBarrier2> fixedBufferBarriers = dispatchInfo.bufferBarriers;
    for (auto &b : fixedBufferBarriers)
    {
        if (b.size == 0) b.size = VK_WHOLE_SIZE;
    }

    VkDependencyInfo dependencyInfo{};
    dependencyInfo.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dependencyInfo.memoryBarrierCount = static_cast<uint32_t>(dispatchInfo.memoryBarriers.size());
    dependencyInfo.pMemoryBarriers = dispatchInfo.memoryBarriers.empty() ? nullptr : dispatchInfo.memoryBarriers.data();
    dependencyInfo.bufferMemoryBarrierCount = static_cast<uint32_t>(fixedBufferBarriers.size());
    dependencyInfo.pBufferMemoryBarriers = fixedBufferBarriers.empty() ? nullptr : fixedBufferBarriers.data();
    dependencyInfo.imageMemoryBarrierCount = static_cast<uint32_t>(dispatchInfo.imageBarriers.size());
    dependencyInfo.pImageMemoryBarriers = dispatchInfo.imageBarriers.empty() ? nullptr : dispatchInfo.imageBarriers.data();

    vkCmdPipelineBarrier2(cmd, &dependencyInfo);
}
