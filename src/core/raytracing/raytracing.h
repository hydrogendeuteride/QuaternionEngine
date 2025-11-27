 #pragma once
 #include <core/types.h>
 #include <unordered_map>
 #include <vector>
 #include <memory>
 
 class DeviceManager;
 class ResourceManager;
 struct DrawContext;
 struct MeshAsset;
 
 struct AccelStructureHandle {
     VkAccelerationStructureKHR handle{VK_NULL_HANDLE};
     AllocatedBuffer storage{}; // buffer that backs the AS
     VkDeviceAddress deviceAddress{0};
 };
 
// Ray tracing helper that caches BLAS per mesh and rebuilds TLAS per frame
// for hybrid/full ray query shadows. See docs/RayTracing.md.
class RayTracingManager {
public:
    void init(DeviceManager* dev, ResourceManager* res);
    void cleanup();
 
     // Build (or get) BLAS for a mesh. Safe to call multiple times.
     AccelStructureHandle getOrBuildBLAS(const std::shared_ptr<MeshAsset>& mesh);
 
    // Rebuild TLAS from current draw context; returns TLAS handle (or null if unavailable)
    // Destruction of previous TLAS resources is deferred via the provided frame deletion queue
    VkAccelerationStructureKHR buildTLASFromDrawContext(const DrawContext& dc, DeletionQueue& frameDQ);
    VkAccelerationStructureKHR tlas() const { return _tlas.handle; }
    VkDeviceAddress tlasAddress() const { return _tlas.deviceAddress; }

    // Destroy any BLAS resources queued for deferred deletion. Call after GPU fence wait.
    void flushPendingDeletes();

    // Remove and destroy a cached BLAS associated with a vertex buffer.
    // Safe to call even if no BLAS exists for the buffer.
    void removeBLASForBuffer(VkBuffer vertexBuffer);
    // Remove and destroy a cached BLAS associated with a mesh pointer.
    void removeBLASForMesh(const MeshAsset *mesh);
 
private:
     // function pointers (resolved on init)
     PFN_vkCreateAccelerationStructureKHR            _vkCreateAccelerationStructureKHR{};
     PFN_vkDestroyAccelerationStructureKHR           _vkDestroyAccelerationStructureKHR{};
     PFN_vkGetAccelerationStructureBuildSizesKHR     _vkGetAccelerationStructureBuildSizesKHR{};
     PFN_vkCmdBuildAccelerationStructuresKHR         _vkCmdBuildAccelerationStructuresKHR{};
     PFN_vkGetAccelerationStructureDeviceAddressKHR  _vkGetAccelerationStructureDeviceAddressKHR{};
 
    DeviceManager* _device{nullptr};
    ResourceManager* _resources{nullptr};

    // BLAS cache per mesh. BLAS lifetime is tied to MeshAsset lifetime;
    // when a mesh is destroyed or its GPU buffers are freed, the owning code
    // must call removeBLASForMesh/removeBLASForBuffer to drop the cached BLAS.
    std::unordered_map<const MeshAsset*, AccelStructureHandle> _blasByMesh;
 
    // TLAS + scratch / instance buffer (rebuilt per frame)
    AccelStructureHandle _tlas{};
    AllocatedBuffer _tlasInstanceBuffer{};
    size_t _tlasInstanceCapacity{0};

    // BLAS scheduled for destruction once GPU is idle
    std::vector<AccelStructureHandle> _pendingBlasDestroy;

    // Properties
    VkDeviceSize _minScratchAlignment{256};
 
    void ensure_tlas_storage(VkDeviceSize requiredASSize, VkDeviceSize requiredScratch, DeletionQueue& frameDQ);
};
 
