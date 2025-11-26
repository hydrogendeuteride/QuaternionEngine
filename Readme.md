# CopernicusEngine
Multi-purpose Vulkan render engine specialized for physics simulation and solar system visualization

## Introduction
Work-In-Progress Vulkan render engine
Current structure:
- Flexible render graph system with multiple render passes, Hot reloading
- Deferred rendering
- PBR, cascaded shadows, normal mapping (MikkTSpace tangents optional)
- GLTF loading and rendering, primitive creation and rendering.
- Supports texture compression(BCn, non glTF standard), LRU reload
- Object clicking, generation.
- Multi light system

Work-In-Progress
- [ ] TAA
- [ ] SSR
- [ ] bloom
- [ ] Planet Rendering

## Build prequsites
- ktx software with libraries
