#include "raytracing.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/util/initializers.h"
#include "scene/vk_loader.h"
#include "scene/vk_scene.h"
#include <cstring>
#include <numeric>

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

    // Query AS properties for scratch alignment
    VkPhysicalDeviceAccelerationStructurePropertiesKHR asProps{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR };
    VkPhysicalDeviceProperties2 props2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2, &asProps };
    vkGetPhysicalDeviceProperties2(_device->physicalDevice(), &props2);
    _minScratchAlignment = std::max<VkDeviceSize>(asProps.minAccelerationStructureScratchOffsetAlignment, 256);
}

void RayTracingManager::cleanup()
{
    VkDevice dv = _device->device();
    // Destroy any deferred BLAS first
    flushPendingDeletes();
    _blasBuildQueue.clear();
    _blasPendingMeshes.clear();

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

    // Destroy any remaining cached BLAS that weren't queued for deferred destroy.
    for (auto &kv : _blasByMesh)
    {
        const AccelStructureHandle &as = kv.second;
        if (as.handle)
        {
            _vkDestroyAccelerationStructureKHR(dv, as.handle, nullptr);
        }
        if (as.storage.buffer)
        {
            _resources->destroy_buffer(as.storage);
        }
    }
    _blasByMesh.clear();
}

void RayTracingManager::flushPendingDeletes()
{
    if (_pendingBlasDestroy.empty()) return;

    fmt::println("[RT] flushPendingDeletes: destroying {} BLAS handles", _pendingBlasDestroy.size());
    VkDevice dv = _device->device();
    for (auto &as : _pendingBlasDestroy)
    {
        if (as.handle)
        {
            _vkDestroyAccelerationStructureKHR(dv, as.handle, nullptr);
        }
        if (as.storage.buffer)
        {
            _resources->destroy_buffer(as.storage);
        }
    }
    _pendingBlasDestroy.clear();
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

    const MeshAsset* key = mesh.get();

    // If a BLAS is already cached (even an empty sentinel), return it directly.
    if (auto it = _blasByMesh.find(key); it != _blasByMesh.end())
    {
        fmt::println("[RT] getOrBuildBLAS reuse by mesh mesh='{}' handle={}", mesh->name,
                     static_cast<const void *>(it->second.handle));
        return it->second;
    }

    // If a build is already queued or in progress for this mesh, do not enqueue
    // another job; simply report "not ready yet".
    if (_blasPendingMeshes.find(key) != _blasPendingMeshes.end())
    {
        fmt::println("[RT] getOrBuildBLAS pending build mesh='{}'", mesh->name);
        return {};
    }

    // If uploads are deferred, ensure any pending mesh buffer uploads are flushed
    // before queuing a BLAS that will read from those GPU buffers.
    if (_resources && _resources->deferred_uploads() && _resources->has_pending_uploads())
    {
        fmt::println("[RT] getOrBuildBLAS: flushing pending resource uploads before queuing BLAS build");
        _resources->process_queued_uploads_immediate();
    }

    fmt::println("[RT] getOrBuildBLAS queue build mesh='{}'", mesh->name);
    _blasPendingMeshes.insert(key);
    _blasBuildQueue.push_back(PendingBlasBuild{key});

    // BLAS will be built asynchronously by pump_blas_builds(); until then,
    // callers should treat the empty handle as "not ready yet".
    return {};
}

AccelStructureHandle RayTracingManager::build_blas_for_mesh(const MeshAsset *mesh)
{
    if (!mesh || !_resources || !_device) return {};

    // If uploads are deferred, ensure any pending mesh buffer uploads are flushed
    // before building a BLAS that reads from those GPU buffers.
    if (_resources->deferred_uploads() && _resources->has_pending_uploads())
    {
        fmt::println("[RT] build_blas_for_mesh: flushing pending resource uploads before BLAS build");
        _resources->process_queued_uploads_immediate();
    }

    // Build BLAS with one geometry per surface (skip empty primitives)
    std::vector<VkAccelerationStructureGeometryKHR> geoms;
    std::vector<VkAccelerationStructureBuildRangeInfoKHR> ranges;
    geoms.reserve(mesh->surfaces.size());
    ranges.reserve(mesh->surfaces.size());

    VkDeviceAddress vaddr = mesh->meshBuffers.vertexBufferAddress;
    VkDeviceAddress iaddr = mesh->meshBuffers.indexBufferAddress;
    const uint32_t vcount = mesh->meshBuffers.vertexCount;
    VkBuffer vb = mesh->meshBuffers.vertexBuffer.buffer;

    fmt::println("[RT] build_blas_for_mesh mesh='{}' surfaces={} vcount={}", mesh->name,
                 mesh->surfaces.size(), vcount);

    for (const auto &s: mesh->surfaces)
    {
        // Compute primitive count from index count; skip empty surfaces
        const uint32_t primitiveCount = s.count / 3u;
        if (primitiveCount == 0)
            continue;

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
        r.primitiveCount = primitiveCount;
        r.primitiveOffset = 0; // encoded through indexData deviceAddress
        r.firstVertex = 0;
        r.transformOffset = 0;
        ranges.push_back(r);
    }

    // If no valid geometries, record an empty sentinel to avoid re-queuing.
    if (geoms.empty())
    {
        fmt::println("[RT] build_blas_for_mesh: mesh='{}' has no primitives; skipping BLAS", mesh->name);
        _blasByMesh.emplace(mesh, AccelStructureHandle{});
        return {};
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

    // Allocate scratch with padding to satisfy alignment requirements
    const VkDeviceSize align = _minScratchAlignment;
    const VkDeviceSize padded = sizes.buildScratchSize + (align - 1);
    AllocatedBuffer scratch = _resources->create_buffer(padded,
                                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                        VMA_MEMORY_USAGE_GPU_ONLY);
    VkDeviceAddress scratchBase = get_buffer_address(_device->device(), scratch.buffer);
    VkDeviceAddress scratchAddr = (scratchBase + (align - 1)) & ~VkDeviceAddress(align - 1);

    buildInfo.dstAccelerationStructure = blas.handle;
    buildInfo.scratchData.deviceAddress = scratchAddr;

    // build with immediate submit
    const VkAccelerationStructureBuildRangeInfoKHR* pRange = ranges.data();
    _resources->immediate_submit([&](VkCommandBuffer cmd) {
        // ppBuildRangeInfos is an array of infoCount pointers; we have 1 build info
        fmt::println("[RT] building BLAS for mesh='{}' geoms={} primsTotal={} storageSize={} scratchSize={}",
                     mesh->name,
                     geoms.size(),
                     maxPrim.empty() ? 0u : std::accumulate(maxPrim.begin(), maxPrim.end(), 0u),
                     sizes.accelerationStructureSize,
                     sizes.buildScratchSize);
        _vkCmdBuildAccelerationStructuresKHR(cmd, 1, &buildInfo, &pRange);
    });

    // destroy scratch
    _resources->destroy_buffer(scratch);

    // device address
    VkAccelerationStructureDeviceAddressInfoKHR dai{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR};
    dai.accelerationStructure = blas.handle;
    blas.deviceAddress = _vkGetAccelerationStructureDeviceAddressKHR(_device->device(), &dai);

    _blasByMesh.emplace(mesh, blas);
    return blas;
}

void RayTracingManager::pump_blas_builds(uint32_t max_builds_per_frame)
{
    if (max_builds_per_frame == 0 || _blasBuildQueue.empty())
    {
        return;
    }

    uint32_t built = 0;

    while (built < max_builds_per_frame && !_blasBuildQueue.empty())
    {
        PendingBlasBuild job = _blasBuildQueue.front();
        _blasBuildQueue.pop_front();

        const MeshAsset* mesh = job.mesh;
        if (mesh)
        {
            // Drop the pending flag for this mesh now; if the build ends up
            // with an empty handle, getOrBuildBLAS will see the cache entry
            // (including the empty sentinel) and avoid re-queuing.
            _blasPendingMeshes.erase(mesh);

            // Skip if a BLAS was already created meanwhile.
            if (_blasByMesh.find(mesh) == _blasByMesh.end())
            {
                AccelStructureHandle blas = build_blas_for_mesh(mesh);
                if (blas.handle)
                {
                    ++built;
                }
            }
        }
        else
        {
            // Mesh pointer is null; just drop the pending flag.
            _blasPendingMeshes.erase(mesh);
        }
    }
}

void RayTracingManager::ensure_tlas_storage(VkDeviceSize requiredASSize, VkDeviceSize /*requiredScratch*/, DeletionQueue& dq)
{
    // Recreate TLAS storage if size grows. Defer destruction to the frame DQ to
    // avoid freeing while referenced by in-flight frames.
    if (_tlas.handle || _tlas.storage.buffer)
    {
        AccelStructureHandle old = _tlas;
        fmt::println("[RT] ensure_tlas_storage: scheduling old TLAS destroy handle={} buffer={} size={}",
                     static_cast<const void *>(old.handle),
                     static_cast<const void *>(old.storage.buffer),
                     old.storage.info.size);
        dq.push_function([this, old]() {
            if (old.handle)
                _vkDestroyAccelerationStructureKHR(_device->device(), old.handle, nullptr);
            if (old.storage.buffer)
                _resources->destroy_buffer(old.storage);
        });
        _tlas = {};
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

    fmt::println("[RT] ensure_tlas_storage: created TLAS handle={} buffer={} size={}",
                 static_cast<const void *>(_tlas.handle),
                 static_cast<const void *>(_tlas.storage.buffer),
                 requiredASSize);
}

VkAccelerationStructureKHR RayTracingManager::buildTLASFromDrawContext(const DrawContext &dc, DeletionQueue& dq)
{
    // Collect instances; one per render object (opaque only).
    std::vector<VkAccelerationStructureInstanceKHR> instances;
    instances.reserve(dc.OpaqueSurfaces.size());

    fmt::println("[RT] buildTLASFromDrawContext: opaqueSurfaces={} current TLAS handle={} buffer={}",
                 dc.OpaqueSurfaces.size(),
                 static_cast<const void *>(_tlas.handle),
                 static_cast<const void *>(_tlas.storage.buffer));

    for (const auto &r: dc.OpaqueSurfaces)
    {
        // Find or lazily build BLAS by mesh pointer. We require sourceMesh
        // for ray tracing; objects without it are skipped from TLAS.
        AccelStructureHandle blas{};
        if (r.sourceMesh)
        {
            auto itMesh = _blasByMesh.find(r.sourceMesh);
            if (itMesh != _blasByMesh.end())
            {
                blas = itMesh->second;
            }
            else
            {
                // Queue an async BLAS build if the mesh is still alive
                // (non-owning shared_ptr wrapper). The BLAS will be built
                // over subsequent frames by pump_blas_builds(); until then,
                // this instance will be skipped.
                std::shared_ptr<MeshAsset> nonOwning(const_cast<MeshAsset *>(r.sourceMesh), [](MeshAsset *) {});
                blas = getOrBuildBLAS(nonOwning);
            }
        }

        if (!blas.handle)
        {
            // Can't build BLAS; skip this instance
            continue;
        }

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
        // No instances this frame: defer TLAS destruction to avoid racing with previous frames
        if (_tlas.handle || _tlas.storage.buffer)
        {
            AccelStructureHandle old = _tlas;
            dq.push_function([this, old]() {
                if (old.handle)
                    _vkDestroyAccelerationStructureKHR(_device->device(), old.handle, nullptr);
                if (old.storage.buffer)
                    _resources->destroy_buffer(old.storage);
            });
            _tlas = {};
        }
        return VK_NULL_HANDLE;
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

    ensure_tlas_storage(sizes.accelerationStructureSize, sizes.buildScratchSize, dq);

    buildInfo.dstAccelerationStructure = _tlas.handle;
    const VkDeviceSize align2 = _minScratchAlignment;
    const VkDeviceSize padded2 = sizes.buildScratchSize + (align2 - 1);
    AllocatedBuffer scratch = _resources->create_buffer(padded2,
                                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                        VMA_MEMORY_USAGE_GPU_ONLY);
    VkDeviceAddress scratchBase2 = get_buffer_address(_device->device(), scratch.buffer);
    VkDeviceAddress scratchAddr2 = (scratchBase2 + (align2 - 1)) & ~VkDeviceAddress(align2 - 1);
    buildInfo.scratchData.deviceAddress = scratchAddr2;

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

    // Drop any queued builds referencing this vertex buffer.
    if (!_blasBuildQueue.empty())
    {
        for (auto itQ = _blasBuildQueue.begin(); itQ != _blasBuildQueue.end(); )
        {
            const MeshAsset* mesh = itQ->mesh;
            if (mesh && mesh->meshBuffers.vertexBuffer.buffer == vertexBuffer)
            {
                _blasPendingMeshes.erase(mesh);
                itQ = _blasBuildQueue.erase(itQ);
            }
            else
            {
                ++itQ;
            }
        }
    }

    // Find any mesh whose vertex buffer matches and evict its BLAS.
    for (auto it = _blasByMesh.begin(); it != _blasByMesh.end(); )
    {
        const MeshAsset *mesh = it->first;
        if (mesh && mesh->meshBuffers.vertexBuffer.buffer == vertexBuffer)
        {
            // Defer destruction until after the next fence wait to avoid racing in-flight traces.
            _pendingBlasDestroy.push_back(it->second);
            it = _blasByMesh.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void RayTracingManager::removeBLASForMesh(const MeshAsset *mesh)
{
    if (!mesh) return;

    // Drop any queued builds for this mesh.
    if (!_blasBuildQueue.empty())
    {
        for (auto itQ = _blasBuildQueue.begin(); itQ != _blasBuildQueue.end(); )
        {
            if (itQ->mesh == mesh)
            {
                itQ = _blasBuildQueue.erase(itQ);
            }
            else
            {
                ++itQ;
            }
        }
    }
    _blasPendingMeshes.erase(mesh);

    auto it = _blasByMesh.find(mesh);
    if (it == _blasByMesh.end()) return;

    // Defer destruction until after the next fence wait to avoid racing in-flight traces.
    _pendingBlasDestroy.push_back(it->second);

    _blasByMesh.erase(it);
}
