 #pragma once
 #include <core/vk_types.h>
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
    VkAccelerationStructureKHR buildTLASFromDrawContext(const DrawContext& dc);
    VkAccelerationStructureKHR tlas() const { return _tlas.handle; }
    VkDeviceAddress tlasAddress() const { return _tlas.deviceAddress; }

    // Remove and destroy a cached BLAS associated with a vertex buffer.
    // Safe to call even if no BLAS exists for the buffer.
    void removeBLASForBuffer(VkBuffer vertexBuffer);
 
 private:
     // function pointers (resolved on init)
     PFN_vkCreateAccelerationStructureKHR            _vkCreateAccelerationStructureKHR{};
     PFN_vkDestroyAccelerationStructureKHR           _vkDestroyAccelerationStructureKHR{};
     PFN_vkGetAccelerationStructureBuildSizesKHR     _vkGetAccelerationStructureBuildSizesKHR{};
     PFN_vkCmdBuildAccelerationStructuresKHR         _vkCmdBuildAccelerationStructuresKHR{};
     PFN_vkGetAccelerationStructureDeviceAddressKHR  _vkGetAccelerationStructureDeviceAddressKHR{};
 
     DeviceManager* _device{nullptr};
     ResourceManager* _resources{nullptr};
 
     // BLAS cache by vertex buffer handle
     std::unordered_map<VkBuffer, AccelStructureHandle> _blasByVB;
 
     // TLAS + scratch / instance buffer (rebuilt per frame)
     AccelStructureHandle _tlas{};
     AllocatedBuffer _tlasInstanceBuffer{};
     size_t _tlasInstanceCapacity{0};
 
     void ensure_tlas_storage(VkDeviceSize requiredASSize, VkDeviceSize requiredScratch);
 };
 
