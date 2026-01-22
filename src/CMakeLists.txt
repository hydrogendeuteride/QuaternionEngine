add_executable (vulkan_engine
  main.cpp
  # core root
  core/types.h
  core/world.h
  core/config.h
  core/context.h
  core/context.cpp
  core/engine.h
  core/engine.cpp
  core/engine_ui.cpp
  core/input/input_system.h
  core/input/input_system.cpp
  core/ui/imgui_system.h
  core/ui/imgui_system.cpp
  core/picking/picking_system.h
  core/picking/picking_system.cpp
  core/debug_draw/debug_draw.h
  core/debug_draw/debug_draw.cpp
  core/debug_draw/physics_debug_draw.h
  core/debug_draw/physics_debug_draw.cpp
  core/debug_draw/engine_debug_draw.h
  core/debug_draw/engine_debug_draw.cpp
  core/game_api.h
  core/game_api/game_api.cpp
  core/game_api/game_api_textures.cpp
  core/game_api/game_api_scene.cpp
  core/game_api/game_api_planets.cpp
  core/game_api/game_api_lighting.cpp
  core/game_api/game_api_camera.cpp
  core/game_api/game_api_postfx.cpp
  core/game_api/game_api_particles.cpp
  core/game_api/game_api_volumetrics.cpp
  core/game_api/game_api_debug.cpp
  # core/device
  core/device/device.h
  core/device/device.cpp
  core/device/swapchain.h
  core/device/swapchain.cpp
  core/device/resource.h
  core/device/resource.cpp
  core/device/images.h
  core/device/images.cpp
  # core/descriptor
  core/descriptor/descriptors.h
  core/descriptor/descriptors.cpp
  core/descriptor/manager.h
  core/descriptor/manager.cpp
  # core/pipeline
  core/pipeline/manager.h
  core/pipeline/manager.cpp
  core/pipeline/sampler.h
  core/pipeline/sampler.cpp
  # core/assets
  core/assets/locator.h
  core/assets/locator.cpp
  core/assets/manager.h
  core/assets/manager.cpp
  core/assets/async_loader.h
  core/assets/async_loader.cpp
  core/assets/texture_cache.h
  core/assets/texture_cache.cpp
  core/assets/ktx_loader.h
  core/assets/ktx_loader.cpp
  core/assets/ibl_manager.h
  core/assets/ibl_manager.cpp
  # core/frame
  core/frame/resources.h
  core/frame/resources.cpp
  # core/util
  core/util/initializers.h
  core/util/initializers.cpp
  core/util/debug.h
  core/util/debug.cpp
  # core/raytracing
  core/raytracing/raytracing.h
  core/raytracing/raytracing.cpp
  # render
  render/pipelines.h
  render/pipelines.cpp
  render/renderpass.h
  render/renderpass.cpp
  render/materials.h
  render/materials.cpp
  render/primitives.h
  # render passes
  render/passes/background.h
  render/passes/background.cpp
  render/passes/sun_disk.h
  render/passes/sun_disk.cpp
  render/passes/geometry.h
  render/passes/geometry.cpp
  render/passes/lighting.h
  render/passes/lighting.cpp
  render/passes/shadow.h
  render/passes/shadow.cpp
  render/passes/fxaa.h
  render/passes/fxaa.cpp
  render/passes/ssr.h
  render/passes/ssr.cpp
  render/passes/clouds.h
  render/passes/clouds.cpp
  render/passes/atmosphere.h
  render/passes/atmosphere.cpp
  render/passes/particles.h
  render/passes/particles.cpp
  render/passes/transparent.h
  render/passes/transparent.cpp
  render/passes/imgui_pass.h
  render/passes/imgui_pass.cpp
  render/passes/auto_exposure.h
  render/passes/auto_exposure.cpp
  render/passes/tonemap.h
  render/passes/tonemap.cpp
  render/passes/debug_draw.h
  render/passes/debug_draw.cpp
  # render graph
  render/graph/types.h
  render/graph/graph.h
  render/graph/graph.cpp
  render/graph/builder.h
  render/graph/builder.cpp
  render/graph/resources.h
  render/graph/resources.cpp
  # scene
  scene/vk_scene.h
  scene/vk_scene.cpp
  scene/vk_scene_picking.cpp
  scene/mesh_bvh.h
  scene/mesh_bvh.cpp
  scene/vk_loader.h
  scene/vk_loader.cpp
  scene/tangent_space.h
  scene/tangent_space.cpp
  scene/camera.h
  scene/camera.cpp
  scene/camera/icamera_mode.h
  scene/camera/camera_rig.h
  scene/camera/camera_rig.cpp
  scene/camera/mode_free.h
  scene/camera/mode_free.cpp
  scene/camera/mode_orbit.h
  scene/camera/mode_orbit.cpp
  scene/camera/mode_follow.h
  scene/camera/mode_follow.cpp
  scene/camera/mode_chase.h
  scene/camera/mode_chase.cpp
  scene/camera/mode_fixed.h
  scene/camera/mode_fixed.cpp
  scene/planet/cubesphere.h
  scene/planet/cubesphere.cpp
  scene/planet/planet_heightmap.h
  scene/planet/planet_heightmap.cpp
  scene/planet/planet_quadtree.h
  scene/planet/planet_quadtree.cpp
  scene/planet/planet_system.h
  scene/planet/planet_system.cpp
  # compute
  compute/vk_compute.h
  compute/vk_compute.cpp
  # runtime
  runtime/i_game_callbacks.h
  runtime/time_manager.h
  runtime/time_manager.cpp
  runtime/game_runtime.h
  runtime/game_runtime.cpp
  # physics
  physics/collision_shape.h
  physics/collider_mesh_instance.h
  physics/body_settings.h
  physics/collider_asset.h
  physics/collider_asset.cpp
  physics/gltf_collider_parser.h
  physics/gltf_collider_parser.cpp
  physics/physics_body.h
  physics/physics_body.cpp
  physics/physics_world.h
  physics/jolt/jolt_physics_world.h
  physics/jolt/jolt_physics_world.cpp
  physics/jolt/jolt_query_filters.h
  # game
  game/entity.h
  game/entity.cpp
  game/entity_manager.h
  game/entity_manager.cpp
  game/game_world.h
  game/game_world.cpp
  game/example_game.h
  game/example_game.cpp
)

set_property(TARGET vulkan_engine PROPERTY CXX_STANDARD 20)
target_compile_definitions(vulkan_engine PUBLIC GLM_FORCE_DEPTH_ZERO_TO_ONE)

# ==============================================================================
# Compiler/OS-specific optimization flags
# ==============================================================================

# clang-cl
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_SIMULATE_ID STREQUAL "MSVC")
    set(USING_CLANG_CL TRUE)
else()
    set(USING_CLANG_CL FALSE)
endif()

# build optim
if (CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")

    if (MSVC AND NOT USING_CLANG_CL)
        # MSVC
        target_compile_options(vulkan_engine PRIVATE
            /O2             # Maximum speed optimization
            /Ob2            # Inline expansion (any suitable)
            /Oi             # Enable intrinsic functions
            /Ot             # Favor fast code
            /Oy             # Omit frame pointers
            /GL             # Whole program optimization
            /fp:fast        # Fast floating-point
            /GS-            # Disable buffer security checks (perf)
        )
        target_link_options(vulkan_engine PRIVATE
            /LTCG           # Link-time code generation
            /OPT:REF        # Remove unreferenced functions
            /OPT:ICF        # Identical COMDAT folding
        )
        # Optional: AVX2 for modern CPUs (uncomment if needed)
        # target_compile_options(vulkan_engine PRIVATE /arch:AVX2)

    elseif (USING_CLANG_CL)
        # Clang-cl (Clang with MSVC-compatible flags)
        target_compile_options(vulkan_engine PRIVATE
            /O2
            /Ob2
            /Oi
            /Ot
            /fp:fast
            -flto=thin      # ThinLTO for faster link times
            -fno-strict-aliasing
        )
        target_link_options(vulkan_engine PRIVATE
            -flto=thin
            /OPT:REF
            /OPT:ICF
        )
        # Optional: AVX2
        target_compile_options(vulkan_engine PRIVATE /arch:AVX2)

    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        # GCC (g++)
        target_compile_options(vulkan_engine PRIVATE
            -O3                     # High optimization level
            -march=native           # Optimize for host CPU
            -mtune=native           # Tune for host CPU
            -flto                   # Link-time optimization
            -ffast-math             # Fast floating-point
            -funroll-loops          # Loop unrolling
            -fomit-frame-pointer    # Omit frame pointers
            -fno-strict-aliasing
        )
        target_link_options(vulkan_engine PRIVATE
            -flto
            -s                      # Strip symbols
        )

    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        # Clang (non-Windows / Linux/macOS)
        target_compile_options(vulkan_engine PRIVATE
            -O3
            -march=native
            -mtune=native
            -flto=thin
            -ffast-math
            -funroll-loops
            -fomit-frame-pointer
            -fno-strict-aliasing
        )
        target_link_options(vulkan_engine PRIVATE
            -flto=thin
        )
    endif()

endif()

# --- Debug build options ---
if (CMAKE_BUILD_TYPE STREQUAL "Debug")

    if (MSVC)
        # MSVC / Clang-cl debug
        target_compile_options(vulkan_engine PRIVATE
            /Od             # Disable optimization
            /Zi             # Debug information
            /RTC1           # Runtime checks
            /GS             # Buffer security checks
        )
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        # GCC / Clang debug
        target_compile_options(vulkan_engine PRIVATE
            -O0                     # No optimization
            -g3                     # Maximum debug info
            -fno-omit-frame-pointer # Keep frame pointers for debugging
            -fsanitize=address,undefined   # Optional: sanitizers
        )
        target_link_options(vulkan_engine PRIVATE
            -fsanitize=address,undefined
        )
    endif()

endif()

if (WIN32)
    target_compile_definitions(vulkan_engine PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        _CRT_SECURE_NO_WARNINGS
    )
endif()

if (UNIX AND NOT APPLE)
    target_link_libraries(vulkan_engine PRIVATE pthread dl)
endif()

if (APPLE)
    target_compile_options(vulkan_engine PRIVATE
        -Wno-deprecated-declarations
    )
endif()

target_include_directories(vulkan_engine PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/core"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/device"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/descriptor"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/pipeline"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/assets"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/frame"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/util"
  "${CMAKE_CURRENT_SOURCE_DIR}/core/raytracing"
  "${CMAKE_CURRENT_SOURCE_DIR}/render"
  "${CMAKE_CURRENT_SOURCE_DIR}/render/passes"
  "${CMAKE_CURRENT_SOURCE_DIR}/render/graph"
  "${CMAKE_CURRENT_SOURCE_DIR}/scene"
  "${CMAKE_CURRENT_SOURCE_DIR}/compute"
  "${CMAKE_CURRENT_SOURCE_DIR}/runtime"
  "${CMAKE_CURRENT_SOURCE_DIR}/physics"
  "${CMAKE_CURRENT_SOURCE_DIR}/game"
)

option(ENABLE_MIKKTS "Use MikkTSpace for tangent generation" ON)

target_link_libraries(vulkan_engine PUBLIC vma glm Vulkan::Vulkan fmt::fmt stb_image SDL2::SDL2 vkbootstrap imgui fastgltf::fastgltf ImGuizmo BVH2)
if (VULKAN_ENGINE_USE_JOLT)
  target_link_libraries(vulkan_engine PUBLIC Jolt::Jolt)
  target_compile_definitions(vulkan_engine PUBLIC VULKAN_ENGINE_USE_JOLT=1)
endif()
if (ENABLE_MIKKTS)
  target_link_libraries(vulkan_engine PUBLIC mikktspace)
  target_compile_definitions(vulkan_engine PUBLIC MIKKTS_ENABLE=1)
endif()

add_library(vma_impl OBJECT vma_impl.cpp)
target_include_directories(vma_impl PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/vma")
target_link_libraries(vma_impl PRIVATE Vulkan::Vulkan)
target_link_libraries(vulkan_engine PUBLIC $<TARGET_OBJECTS:vma_impl>)

target_precompile_headers(vulkan_engine PRIVATE
  <optional>
  <vector>
  <memory>
  <string>
  <unordered_map>
  <glm/mat4x4.hpp>
  <glm/vec4.hpp>
  <vulkan/vulkan.h>
)

if(WIN32)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.21)
    add_custom_command(TARGET vulkan_engine POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:vulkan_engine> $<TARGET_FILE_DIR:vulkan_engine>
      COMMAND_EXPAND_LISTS
    )
  else()
    message(WARNING "CMake < 3.21: skipping runtime DLL copy step for vulkan_engine")
  endif()
endif()

# KTX (libktx)
find_package(ktx CONFIG QUIET)

set(_KTX_CMAKE_TARGET "")
if(TARGET ktx::ktx)
  set(_KTX_CMAKE_TARGET ktx::ktx)
elseif(TARGET KTX::ktx)
  set(_KTX_CMAKE_TARGET KTX::ktx)
elseif(TARGET ktx)
  set(_KTX_CMAKE_TARGET ktx)
endif()

set(_KTX_USE_FALLBACK OFF)
if(_KTX_CMAKE_TARGET STREQUAL "")
  set(_KTX_USE_FALLBACK ON)
else()
  get_target_property(_KTX_IMPORTED "${_KTX_CMAKE_TARGET}" IMPORTED)
  if(_KTX_IMPORTED)
    set(_KTX_IMPORTED_LOCATION "")
    get_target_property(_KTX_IMPORTED_LOCATION "${_KTX_CMAKE_TARGET}" IMPORTED_LOCATION)
    if(NOT _KTX_IMPORTED_LOCATION)
      get_target_property(_KTX_IMPORTED_LOCATION "${_KTX_CMAKE_TARGET}" IMPORTED_LOCATION_RELEASE)
    endif()
    if(NOT _KTX_IMPORTED_LOCATION)
      get_target_property(_KTX_IMPORTED_LOCATION "${_KTX_CMAKE_TARGET}" IMPORTED_LOCATION_DEBUG)
    endif()

    if(NOT _KTX_IMPORTED_LOCATION)
      set(_KTX_USE_FALLBACK ON)
    elseif(NOT WIN32 AND _KTX_IMPORTED_LOCATION MATCHES "\\.dll$|\\.lib$")
      set(_KTX_USE_FALLBACK ON)
    endif()
  endif()
endif()

if(_KTX_USE_FALLBACK)
  find_path(KTX_INCLUDE_DIR NAMES ktx.h)
  find_library(KTX_LIBRARY NAMES ktx)
  if(NOT KTX_INCLUDE_DIR OR NOT KTX_LIBRARY)
    message(FATAL_ERROR "libktx not found; please install KTX v2 and make it discoverable via ktx_DIR or CMAKE_PREFIX_PATH")
  endif()
  target_include_directories(vulkan_engine PUBLIC "${KTX_INCLUDE_DIR}")
  target_link_libraries(vulkan_engine PUBLIC "${KTX_LIBRARY}")
else()
  target_link_libraries(vulkan_engine PUBLIC "${_KTX_CMAKE_TARGET}")
endif()
