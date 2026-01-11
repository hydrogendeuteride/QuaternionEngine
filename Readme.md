# QuaternionEngine
Multipurpose Vulkan render engine specialized for physics simulation and solar system visualization

![](gallery/vk2.png)

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
- Floating origin with double precision coordinate system
- Planet Rendering, Cubesphere-quadtree LOD Terrain system, Rayleigh-Mie scattering atmosphere
- Sun

Work-In-Progress
- Physics engine integration (Jolt)
- Celestial mechanics
- ...AND making real game

## Build prequsites
- ktx software with libraries

## Gallery
![](gallery/vk.png)
![](gallery/vk3.png)
![](gallery/vk4.png)
