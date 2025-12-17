cmake_minimum_required (VERSION 3.8)

project ("vulkan_engine")

if (WIN32)
    set(VULKAN_SDK "$ENV{VULKAN_SDK}")
    set(Vulkan_INCLUDE_DIR "C:/VulkanSDK/1.3.296.0/Include")
    set(Vulkan_LIBRARY "C:/VulkanSDK/1.3.296.0/Lib/vulkan-1.lib")
endif()

find_package(Vulkan REQUIRED)

# Third-party deps are vendored; keep builds offline-friendly by default.
# BVH2's CMake enables tests by default, which would FetchContent googletest.
set(BVH2_ENABLE_TESTS OFF CACHE BOOL "Disable BVH2 tests (offline builds)" FORCE)

add_subdirectory(third_party)

if (MSVC AND CMAKE_C_COMPILER_ID MATCHES "Clang")
    if (TARGET SDL2)
        target_link_options(SDL2 PRIVATE /DEFAULTLIB:ucrt.lib /DEFAULTLIB:vcruntime.lib)
    endif()
    if (TARGET SDL2-static)
        target_link_options(SDL2-static PRIVATE /DEFAULTLIB:ucrt.lib /DEFAULTLIB:vcruntime.lib)
    endif()
endif()

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

add_subdirectory(src)

find_program(GLSL_VALIDATOR glslangValidator HINTS /usr/bin /usr/local/bin $ENV{VULKAN_SDK}/Bin/ $ENV{VULKAN_SDK}/Bin32/)




file(GLOB_RECURSE GLSL_SOURCE_FILES
    "${PROJECT_SOURCE_DIR}/shaders/*.frag"
    "${PROJECT_SOURCE_DIR}/shaders/*.vert"
    "${PROJECT_SOURCE_DIR}/shaders/*.comp"
    )

foreach(GLSL ${GLSL_SOURCE_FILES})
  message(STATUS "BUILDING SHADER")
  get_filename_component(FILE_NAME ${GLSL} NAME)
  set(SPIRV "${PROJECT_SOURCE_DIR}/shaders/${FILE_NAME}.spv")
  message(STATUS ${GLSL})
  add_custom_command(
    OUTPUT ${SPIRV}
    COMMAND ${GLSL_VALIDATOR} -V --target-env vulkan1.2 ${GLSL} -o ${SPIRV}
    DEPENDS ${GLSL})
  list(APPEND SPIRV_BINARY_FILES ${SPIRV})
endforeach(GLSL)

add_custom_target(shaders_spirv DEPENDS ${SPIRV_BINARY_FILES})

# Ensure shaders are built alongside the executable.
add_dependencies(vulkan_engine shaders_spirv)
