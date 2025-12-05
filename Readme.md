# QuaternionEngine
Multipurpose Vulkan render engine specialized for physics simulation and solar system visualization

![](vk.png)

## Introduction
Work-In-Progress Vulkan render engine
Current structure:
- Flexible render graph system with multiple render passes, Hot reloading
- Deferred rendering
- PBR, cascaded shadows, normal mapping (MikkTSpace tangents optional)
- GLTF loading and rendering, primitive creation and rendering
- Supports texture compression(BCn, non glTF standard), LRU reload
- Runtime object clicking, generation, movement
- Multi light system
- SSR
- FXAA
- Bloom

Work-In-Progress
- [ ] Floating origin with double precision coordinate system
- [ ] Planet Rendering

## Build prequsites
- ktx software with libraries
