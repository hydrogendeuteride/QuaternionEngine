cmake_minimum_required(VERSION 3.20)

project(vulkan_engine LANGUAGES C CXX)

if(WIN32)
    if(DEFINED ENV{VULKAN_SDK})
        file(TO_CMAKE_PATH "$ENV{VULKAN_SDK}" _VULKAN_SDK)
        set(Vulkan_INCLUDE_DIR "${_VULKAN_SDK}/Include")
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(Vulkan_LIBRARY "${_VULKAN_SDK}/Lib/vulkan-1.lib")
        else()
            set(Vulkan_LIBRARY "${_VULKAN_SDK}/Lib32/vulkan-1.lib")
        endif()
    elseif(EXISTS "C:/VulkanSDK/1.3.296.0/Include")
        # Fallback for a common SDK install path; update if your SDK version differs.
        set(Vulkan_INCLUDE_DIR "C:/VulkanSDK/1.3.296.0/Include")
        set(Vulkan_LIBRARY "C:/VulkanSDK/1.3.296.0/Lib/vulkan-1.lib")
    endif()
endif()

find_package(Vulkan REQUIRED COMPONENTS glslangValidator)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

# Third-party deps are vendored; keep builds offline-friendly by default.
# BVH2's CMake enables tests by default, which would FetchContent googletest.
set(BVH2_ENABLE_TESTS OFF CACHE BOOL "Disable BVH2 tests (offline builds)" FORCE)

option(VULKAN_ENGINE_USE_JOLT "Build with Jolt Physics (vendored in third_party/JoltPhysics)" ON)
option(VULKAN_ENGINE_JOLT_DOUBLE_PRECISION "Build Jolt with double precision math (JPH_DOUBLE_PRECISION)" ON)
option(VULKAN_ENGINE_BUILD_TESTS "Build unit tests" ON)

add_subdirectory(third_party)

if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_SIMULATE_ID STREQUAL "MSVC")
    foreach(_SDL_TARGET IN ITEMS SDL2 SDL2-static)
        if(TARGET ${_SDL_TARGET})
            target_link_options(${_SDL_TARGET} PRIVATE /DEFAULTLIB:ucrt.lib /DEFAULTLIB:vcruntime.lib)
        endif()
    endforeach()
endif()

add_subdirectory(src)

if (VULKAN_ENGINE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

option(VULKAN_ENGINE_BUILD_SHADERS "Compile GLSL shaders to SPIR-V at build time" ON)

if(VULKAN_ENGINE_BUILD_SHADERS)
    set(GLSL_VALIDATOR "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}")
    if(NOT GLSL_VALIDATOR)
        find_program(GLSL_VALIDATOR glslangValidator HINTS /usr/bin /usr/local/bin)
    endif()

    if(NOT GLSL_VALIDATOR)
        message(FATAL_ERROR "glslangValidator not found. Install the Vulkan SDK or disable shader compilation with -DVULKAN_ENGINE_BUILD_SHADERS=OFF.")
    endif()

    file(GLOB_RECURSE GLSL_SOURCE_FILES CONFIGURE_DEPENDS
        "${PROJECT_SOURCE_DIR}/shaders/*.frag"
        "${PROJECT_SOURCE_DIR}/shaders/*.vert"
        "${PROJECT_SOURCE_DIR}/shaders/*.comp"
    )

    set(SPIRV_BINARY_FILES "")
    foreach(GLSL IN LISTS GLSL_SOURCE_FILES)
        set(SPIRV "${GLSL}.spv")
        add_custom_command(
            OUTPUT "${SPIRV}"
            # Force SPIR-V 1.5 output for broader driver compatibility (some drivers crash on SPIR-V 1.6 modules).
            COMMAND "${GLSL_VALIDATOR}" -V --target-env vulkan1.2 --target-env spirv1.5 "${GLSL}" -o "${SPIRV}"
            DEPENDS "${GLSL}"
            VERBATIM
        )
        list(APPEND SPIRV_BINARY_FILES "${SPIRV}")
    endforeach()

    add_custom_target(shaders_spirv DEPENDS ${SPIRV_BINARY_FILES})
    add_dependencies(vulkan_engine shaders_spirv)
endif()
