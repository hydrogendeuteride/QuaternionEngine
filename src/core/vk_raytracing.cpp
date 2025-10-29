#include "vk_raytracing.h"
#include "vk_device.h"
#include "vk_resource.h"
#include "vk_initializers.h"
#include "scene/vk_loader.h"
#include "scene/vk_scene.h"
#include <cstring>

void RayTracingManager::init(DeviceManager *dev, ResourceManager *res)
{
    _device = dev;
    _resources = res;
    // resolve function pointers
    _vkCreateAccelerationStructureKHR = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(
        vkGetDeviceProcAddr(_device->device(), "vkCreateAccelerationStructureKHR"));
    _vkDestroyAccelerationStructureKHR = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(
        vkGetDeviceProcAddr(_device->device(), "vkDestroyAccelerationStructureKHR"));
    _vkGetAccelerationStructureBuildSizesKHR = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(
        vkGetDeviceProcAddr(_device->device(), "vkGetAccelerationStructureBuildSizesKHR"));
    _vkCmdBuildAccelerationStructuresKHR = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(
        vkGetDeviceProcAddr(_device->device(), "vkCmdBuildAccelerationStructuresKHR"));
    _vkGetAccelerationStructureDeviceAddressKHR = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(
        vkGetDeviceProcAddr(_device->device(), "vkGetAccelerationStructureDeviceAddressKHR"));
}

void RayTracingManager::cleanup()
{
    VkDevice dv = _device->device();
    if (_tlas.handle)
    {
        _vkDestroyAccelerationStructureKHR(dv, _tlas.handle, nullptr);
        _tlas.handle = VK_NULL_HANDLE;
    }
    if (_tlas.storage.buffer)
    {
        _resources->destroy_buffer(_tlas.storage);
        _tlas.storage = {};
    }
    if (_tlasInstanceBuffer.buffer)
    {
        _resources->destroy_buffer(_tlasInstanceBuffer);
        _tlasInstanceBuffer = {};
        _tlasInstanceCapacity = 0;
    }
    for (auto &kv: _blasByVB)
    {
        if (kv.second.handle)
        {
            _vkDestroyAccelerationStructureKHR(dv, kv.second.handle, nullptr);
        }
        if (kv.second.storage.buffer)
        {
            _resources->destroy_buffer(kv.second.storage);
        }
    }
    _blasByVB.clear();
}

static VkDeviceAddress get_buffer_address(VkDevice dev, VkBuffer buf)
{
    VkBufferDeviceAddressInfo info{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    info.buffer = buf;
    return vkGetBufferDeviceAddress(dev, &info);
}

AccelStructureHandle RayTracingManager::getOrBuildBLAS(const std::shared_ptr<MeshAsset> &mesh)
{
    if (!mesh) return {};
    VkBuffer vb = mesh->meshBuffers.vertexBuffer.buffer;
    if (auto it = _blasByVB.find(vb); it != _blasByVB.end())
    {
        return it->second;
    }

    // Build BLAS with one geometry per surface
    std::vector<VkAccelerationStructureGeometryKHR> geoms;
    std::vector<VkAccelerationStructureBuildRangeInfoKHR> ranges;
    geoms.reserve(mesh->surfaces.size());
    ranges.reserve(mesh->surfaces.size());

    VkDeviceAddress vaddr = mesh->meshBuffers.vertexBufferAddress;
    VkDeviceAddress iaddr = mesh->meshBuffers.indexBufferAddress;
    const uint32_t vcount = mesh->meshBuffers.vertexCount;

    for (const auto &s: mesh->surfaces)
    {
        VkAccelerationStructureGeometryTrianglesDataKHR tri{
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR
        };
        tri.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
        tri.vertexData.deviceAddress = vaddr;
        tri.vertexStride = sizeof(Vertex);
        tri.maxVertex = vcount ? (vcount - 1) : 0; // conservative
        tri.indexType = VK_INDEX_TYPE_UINT32;
        tri.indexData.deviceAddress = iaddr + static_cast<VkDeviceAddress>(s.startIndex) * sizeof(uint32_t);
        tri.transformData.deviceAddress = 0; // identity

        VkAccelerationStructureGeometryKHR g{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
        g.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
        g.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
        g.geometry.triangles = tri;

        geoms.push_back(g);

        VkAccelerationStructureBuildRangeInfoKHR r{};
        r.primitiveCount = s.count / 3;
        r.primitiveOffset = 0; // encoded through indexData deviceAddress
        r.firstVertex = 0;
        r.transformOffset = 0;
        ranges.push_back(r);
    }

    VkAccelerationStructureBuildGeometryInfoKHR buildInfo{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR
    };
    buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    buildInfo.geometryCount = static_cast<uint32_t>(geoms.size());
    buildInfo.pGeometries = geoms.data();

    std::vector<uint32_t> maxPrim(geoms.size());
    for (size_t i = 0; i < ranges.size(); ++i) maxPrim[i] = ranges[i].primitiveCount;

    VkAccelerationStructureBuildSizesInfoKHR sizes{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};
    _vkGetAccelerationStructureBuildSizesKHR(_device->device(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                             &buildInfo, maxPrim.data(), &sizes);

    // allocate AS storage and scratch
    AccelStructureHandle blas{};
    blas.storage = _resources->create_buffer(sizes.accelerationStructureSize,
                                             VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                             VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                             VMA_MEMORY_USAGE_GPU_ONLY);

    VkAccelerationStructureCreateInfoKHR asci{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
    asci.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    asci.buffer = blas.storage.buffer;
    asci.size = sizes.accelerationStructureSize;
    VK_CHECK(_vkCreateAccelerationStructureKHR(_device->device(), &asci, nullptr, &blas.handle));

    AllocatedBuffer scratch = _resources->create_buffer(sizes.buildScratchSize,
                                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                        VMA_MEMORY_USAGE_GPU_ONLY);
    VkDeviceAddress scratchAddr = get_buffer_address(_device->device(), scratch.buffer);

    buildInfo.dstAccelerationStructure = blas.handle;
    buildInfo.scratchData.deviceAddress = scratchAddr;

    // build with immediate submit
    std::vector<const VkAccelerationStructureBuildRangeInfoKHR *> pRanges(geoms.size());
    for (size_t i = 0; i < geoms.size(); ++i) pRanges[i] = &ranges[i];
    _resources->immediate_submit([&](VkCommandBuffer cmd) {
        _vkCmdBuildAccelerationStructuresKHR(cmd, 1, &buildInfo, pRanges.data());
    });

    // destroy scratch
    _resources->destroy_buffer(scratch);

    // device address
    VkAccelerationStructureDeviceAddressInfoKHR dai{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR};
    dai.accelerationStructure = blas.handle;
    blas.deviceAddress = _vkGetAccelerationStructureDeviceAddressKHR(_device->device(), &dai);

    _blasByVB.emplace(vb, blas);
    return blas;
}

void RayTracingManager::ensure_tlas_storage(VkDeviceSize requiredASSize, VkDeviceSize /*requiredScratch*/)
{
    // Simple: recreate TLAS storage if size grows
    if (_tlas.handle)
    {
        _vkDestroyAccelerationStructureKHR(_device->device(), _tlas.handle, nullptr);
        _tlas.handle = VK_NULL_HANDLE;
    }
    if (_tlas.storage.buffer)
    {
        _resources->destroy_buffer(_tlas.storage);
        _tlas.storage = {};
    }
    _tlas.storage = _resources->create_buffer(requiredASSize,
                                              VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                              VMA_MEMORY_USAGE_GPU_ONLY);

    VkAccelerationStructureCreateInfoKHR asci{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
    asci.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    asci.buffer = _tlas.storage.buffer;
    asci.size = requiredASSize;
    VK_CHECK(_vkCreateAccelerationStructureKHR(_device->device(), &asci, nullptr, &_tlas.handle));
}

VkAccelerationStructureKHR RayTracingManager::buildTLASFromDrawContext(const DrawContext &dc)
{
    // Collect instances; one per render object (opaque only).
    std::vector<VkAccelerationStructureInstanceKHR> instances;
    instances.reserve(dc.OpaqueSurfaces.size());

    for (const auto &r: dc.OpaqueSurfaces)
    {
        // Find mesh BLAS by vertex buffer
        AccelStructureHandle blas{};
        // We don't have MeshAsset pointer here; BLAS cache is keyed by VB handle; if missing, skip
        auto it = _blasByVB.find(r.vertexBuffer);
        if (it == _blasByVB.end())
        {
            // Can't build BLAS on the fly without mesh topology; skip this instance
            continue;
        }
        blas = it->second;

        VkAccelerationStructureInstanceKHR inst{};
        // Fill 3x4 row-major from GLM column-major mat4
        const glm::mat4 &m = r.transform;
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 4; ++col)
                inst.transform.matrix[row][col] = m[col][row];

        inst.instanceCustomIndex = 0;
        inst.mask = 0xFF;
        inst.instanceShaderBindingTableRecordOffset = 0;
        inst.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR; // two-sided
        inst.accelerationStructureReference = blas.deviceAddress;
        instances.push_back(inst);
    }

    if (instances.empty())
    {
        // nothing to build
        return _tlas.handle;
    }

    // Ensure instance buffer capacity
    if (instances.size() > _tlasInstanceCapacity)
    {
        if (_tlasInstanceBuffer.buffer)
        {
            _resources->destroy_buffer(_tlasInstanceBuffer);
        }
        _tlasInstanceCapacity = instances.size();
        _tlasInstanceBuffer = _resources->create_buffer(
            _tlasInstanceCapacity * sizeof(VkAccelerationStructureInstanceKHR),
            VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
            VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_CPU_TO_GPU);
    }

    // upload instances
    {
        VmaAllocationInfo ai{};
        vmaGetAllocationInfo(_device->allocator(), _tlasInstanceBuffer.allocation, &ai);
        std::memcpy(ai.pMappedData, instances.data(), instances.size() * sizeof(instances[0]));
        vmaFlushAllocation(_device->allocator(), _tlasInstanceBuffer.allocation, 0, VK_WHOLE_SIZE);
    }

    VkDeviceAddress instAddr = get_buffer_address(_device->device(), _tlasInstanceBuffer.buffer);

    VkAccelerationStructureGeometryInstancesDataKHR instData{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR
    };
    instData.arrayOfPointers = VK_FALSE;
    instData.data.deviceAddress = instAddr;

    VkAccelerationStructureGeometryKHR geom{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
    geom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geom.geometry.instances = instData;

    VkAccelerationStructureBuildGeometryInfoKHR buildInfo{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR
    };
    buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    buildInfo.geometryCount = 1;
    buildInfo.pGeometries = &geom;

    uint32_t primCount = static_cast<uint32_t>(instances.size());
    VkAccelerationStructureBuildSizesInfoKHR sizes{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};
    _vkGetAccelerationStructureBuildSizesKHR(_device->device(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                             &buildInfo, &primCount, &sizes);

    ensure_tlas_storage(sizes.accelerationStructureSize, sizes.buildScratchSize);

    buildInfo.dstAccelerationStructure = _tlas.handle;
    AllocatedBuffer scratch = _resources->create_buffer(sizes.buildScratchSize,
                                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                        VMA_MEMORY_USAGE_GPU_ONLY);
    VkDeviceAddress scratchAddr = get_buffer_address(_device->device(), scratch.buffer);
    buildInfo.scratchData.deviceAddress = scratchAddr;

    VkAccelerationStructureBuildRangeInfoKHR range{};
    range.primitiveCount = primCount;
    const VkAccelerationStructureBuildRangeInfoKHR *pRange = &range;

    _resources->immediate_submit([&](VkCommandBuffer cmd) {
        _vkCmdBuildAccelerationStructuresKHR(cmd, 1, &buildInfo, &pRange);
    });
    _resources->destroy_buffer(scratch);

    VkAccelerationStructureDeviceAddressInfoKHR dai{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR};
    dai.accelerationStructure = _tlas.handle;
    _tlas.deviceAddress = _vkGetAccelerationStructureDeviceAddressKHR(_device->device(), &dai);

    return _tlas.handle;
}

void RayTracingManager::removeBLASForBuffer(VkBuffer vertexBuffer)
{
    if (!vertexBuffer) return;
    VkDevice dv = _device->device();
    auto it = _blasByVB.find(vertexBuffer);
    if (it == _blasByVB.end()) return;

    if (it->second.handle)
    {
        _vkDestroyAccelerationStructureKHR(dv, it->second.handle, nullptr);
    }
    if (it->second.storage.buffer)
    {
        _resources->destroy_buffer(it->second.storage);
    }
    _blasByVB.erase(it);
}
