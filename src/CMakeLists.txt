set(VULKAN_ENGINE_SOURCES)

include(cmake/sources_root.cmake)
include(cmake/sources_core.cmake)
include(cmake/sources_render.cmake)
include(cmake/sources_scene.cmake)
include(cmake/sources_compute.cmake)
include(cmake/sources_runtime.cmake)
include(cmake/sources_physics.cmake)
include(cmake/sources_audio.cmake)
include(cmake/sources_game.cmake)

add_executable(vulkan_engine ${VULKAN_ENGINE_SOURCES})

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
    target_link_libraries(vulkan_engine PRIVATE Dbghelp)
    target_compile_options(vulkan_engine PRIVATE /Zi)
    target_link_options(vulkan_engine PRIVATE /DEBUG:FULL /MAP)
endif()

if (UNIX AND NOT APPLE)
    target_link_libraries(vulkan_engine PRIVATE pthread dl)
endif()

if (APPLE)
    target_compile_options(vulkan_engine PRIVATE
        -Wno-deprecated-declarations
    )
endif()

# Vulkan structs are intentionally partially initialized in many places
# (set sType + only required fields). Suppress this noisy warning category.
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(vulkan_engine PRIVATE
        -Wno-missing-field-initializers
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
  "${CMAKE_CURRENT_SOURCE_DIR}/audio"
  "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/taskflow/3rd-party"
)

# Treat BVH headers as system headers to avoid third-party warning noise.
target_include_directories(vulkan_engine SYSTEM PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/BVH/include"
)

option(ENABLE_MIKKTS "Use MikkTSpace for tangent generation" ON)

target_link_libraries(vulkan_engine PUBLIC vma glm Vulkan::Vulkan fmt::fmt stb_image SDL2::SDL2 vkbootstrap imgui fastgltf::fastgltf ImGuizmo BVH2 orbitsim miniaudio tomlplusplus)
if (TARGET miniaudio_libvorbis)
  target_link_libraries(vulkan_engine PUBLIC miniaudio_libvorbis)
  target_compile_definitions(vulkan_engine PUBLIC VULKAN_ENGINE_AUDIO_LIBVORBIS=1)
endif()
if (TARGET miniaudio_libopus)
  target_link_libraries(vulkan_engine PUBLIC miniaudio_libopus)
  target_compile_definitions(vulkan_engine PUBLIC VULKAN_ENGINE_AUDIO_LIBOPUS=1)
endif()
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
