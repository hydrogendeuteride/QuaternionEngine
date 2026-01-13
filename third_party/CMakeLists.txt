add_library(vkbootstrap STATIC)
add_library(glm INTERFACE)
add_library(vma INTERFACE)

add_library(stb_image INTERFACE)

add_subdirectory(fastgltf)
add_subdirectory(fmt EXCLUDE_FROM_ALL)
add_subdirectory(SDL EXCLUDE_FROM_ALL)

if (VULKAN_ENGINE_USE_JOLT)
  # Jolt's CMake options are unfortunately generic and live in the cache; set a couple
  # of safe defaults for integration into this engine build.
  set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "Disable Jolt warnings-as-errors in parent project" FORCE)
  set(ENABLE_INSTALL OFF CACHE BOOL "Disable Jolt install rules in parent project" FORCE)
  set(OVERRIDE_CXX_FLAGS OFF CACHE BOOL "Do not let Jolt override global CMAKE_CXX_FLAGS_*" FORCE)
  set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "Use dynamic MSVC runtime (avoid /MT-/MD mismatches)" FORCE)
  set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "Disable Jolt LTO/IPO in parent project (engine controls LTO)" FORCE)
  set(CPP_RTTI_ENABLED ON CACHE BOOL "Enable RTTI (engine headers use dynamic_cast; required for linking)" FORCE)
  set(DOUBLE_PRECISION ${VULKAN_ENGINE_JOLT_DOUBLE_PRECISION} CACHE BOOL "Build Jolt with double precision math" FORCE)

  add_subdirectory(JoltPhysics/Build EXCLUDE_FROM_ALL)
endif()

target_sources(vkbootstrap PRIVATE 
  vkbootstrap/VkBootstrap.h
  vkbootstrap/VkBootstrap.cpp
  )

target_include_directories(vkbootstrap PUBLIC vkbootstrap)
target_link_libraries(vkbootstrap PUBLIC Vulkan::Vulkan $<$<BOOL:UNIX>:${CMAKE_DL_LIBS}>)
set_property(TARGET vkbootstrap PROPERTY CXX_STANDARD 20)

#both vma and glm and header only libs so we only need the include path
target_include_directories(vma INTERFACE vma)

target_include_directories(glm INTERFACE glm)

#add_library(sdl2 INTERFACE)

#target_include_directories(sdl2 INTERFACE $ENV{VULKAN_SDK}/Include/SDL2 )
#target_link_directories(sdl2 INTERFACE $ENV{VULKAN_SDK}/Lib )

#target_link_libraries(sdl2 INTERFACE SDL2 SDL2main)

add_library(imgui STATIC)

target_include_directories(imgui PUBLIC imgui)

target_sources(imgui PRIVATE 
  imgui/imgui.h
  imgui/imgui.cpp

  imgui/imgui_demo.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_widgets.cpp
  imgui/imgui_tables.cpp
  imgui/imgui_impl_vulkan.cpp
  imgui/imgui_impl_sdl2.cpp
  )

target_link_libraries(imgui PUBLIC Vulkan::Vulkan SDL2::SDL2)

add_library(ImGuizmo STATIC
  ImGuizmo/ImGuizmo.cpp
  ImGuizmo/ImSequencer.cpp
  ImGuizmo/ImCurveEdit.cpp
  ImGuizmo/ImGradient.cpp
  ImGuizmo/GraphEditor.cpp
)

target_include_directories(ImGuizmo PUBLIC
  ImGuizmo
  imgui
)

target_link_libraries(ImGuizmo PUBLIC imgui)

add_subdirectory(BVH)
target_include_directories(BVH2 INTERFACE taskflow)

target_include_directories(stb_image INTERFACE stb_image)

# MikkTSpace (optional)
add_library(mikktspace STATIC MikkTSpace/mikktspace.c)
target_include_directories(mikktspace PUBLIC MikkTSpace)
