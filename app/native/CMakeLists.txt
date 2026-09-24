cmake_minimum_required(VERSION 3.21)

project(materia_reference_editor_native VERSION 0.1.0 LANGUAGES C CXX)

if(NOT DEFINED HAXEON_NATIVE_OUTPUT_DIR OR HAXEON_NATIVE_OUTPUT_DIR STREQUAL "")
    set(HAXEON_NATIVE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/native")
endif()
file(MAKE_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")

# The app owns this aggregate provider.  NativeKit and UIKit remain ordinary
# CMake projects, but one Haxeon action configures and builds their complete
# runtime dependency chain together.
set(NK_BUILD_SHARED ON CACHE BOOL "Build shared NativeKit libraries" FORCE)
set(NK_BUILD_GPU ON CACHE BOOL "Build NativeKit GPU" FORCE)
set(NK_BUILD_TESTS OFF CACHE BOOL "Disable dependency tests in the app build" FORCE)
set(NK_BUILD_EXAMPLES OFF CACHE BOOL "Disable dependency examples in the app build" FORCE)
set(NKUI_ENABLE_SECTION_GC ON CACHE BOOL "Keep UIKit section garbage collection enabled" FORCE)

get_filename_component(MATERIA_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(MATERIA_NATIVEKIT_DIR "${MATERIA_SOURCE_ROOT}/nativekit")
set(MATERIA_UIKIT_DIR "${MATERIA_SOURCE_ROOT}/uikit")

add_subdirectory("${MATERIA_NATIVEKIT_DIR}" nativekit)
add_subdirectory("${MATERIA_UIKIT_DIR}" uikit)

# Build the scene model and spatial queries used by the editor document.
set(NKSCENE_NATIVEKIT_DIR "${MATERIA_NATIVEKIT_DIR}")
set(NK_LIBRARY_TYPE SHARED)
add_subdirectory("${MATERIA_SOURCE_ROOT}/scenekit/scene" scene)
add_subdirectory("${MATERIA_SOURCE_ROOT}/scenekit/scene_render" scene_render)

# Sensor editing can atomically compile and rebuild the same RobotKit/SimKit
# runtime used by robotd. Reuse the already-created SceneKit targets instead of
# adding the complete dependency graph a second time.
set(RK_BUILD_SHARED ON CACHE BOOL "Build RobotKit as a shared library" FORCE)
set(RK_BUILD_TESTS OFF CACHE BOOL "Disable RobotKit tests in the app build" FORCE)
set(RK_BUILD_SIMKIT ON CACHE BOOL "Enable the SimKit-backed RobotKit runtime" FORCE)
set(MATERIA_ENABLE_MUJOCO ON CACHE BOOL "Build the MuJoCo backend for application simulation" FORCE)
set(NKSIM_BUILD_MUJOCO ${MATERIA_ENABLE_MUJOCO} CACHE BOOL "Build selectable MuJoCo backend" FORCE)
set(NK_SIM_MUJOCO_USE_VENDORED ON CACHE BOOL "Use SimKit's pinned MuJoCo" FORCE)
set(NK_MUJOCO_SOURCE_DIR "${MATERIA_SOURCE_ROOT}/simkit/vendor/mujoco" CACHE PATH
    "Path to the pinned MuJoCo checkout" FORCE)
set(MUJOCO_ENABLE_LTO OFF CACHE BOOL "Keep MuJoCo compatible with the application linker" FORCE)
set(NK_LIBRARY_TYPE SHARED)
add_subdirectory("${MATERIA_SOURCE_ROOT}/simkit/sim_core" sim_core)
if(MATERIA_ENABLE_MUJOCO)
    add_subdirectory("${MATERIA_SOURCE_ROOT}/simkit/sim_mujoco" sim_mujoco)
endif()
add_subdirectory("${MATERIA_SOURCE_ROOT}/robotkit" robotkit)

# CadKit owns millimetre-based parametric geometry. Keep its pinned OCCT
# toolkits shared, matching OCCT's supported LGPL distribution model.
set(CADKIT_BUILD_TESTS OFF CACHE BOOL "Disable CadKit dependency tests" FORCE)
set(CADKIT_BUILD_OCCT ON CACHE BOOL "Build CadKit's pinned OCCT" FORCE)
set(CADKIT_BUILD_SHARED ON CACHE BOOL "Build the CadKit application library" FORCE)
set(BUILD_LIBRARY_TYPE Shared CACHE STRING "Build OCCT as shared libraries" FORCE)
add_subdirectory("${MATERIA_SOURCE_ROOT}/cadkit" cadkit)
if(UNIX AND NOT APPLE)
    target_link_options(cadkit-core PRIVATE "LINKER:-z,defs")
endif()

set(MATERIA_RUNTIME_TARGETS nativekit nativekit_gpu nativekit_ui nativekit_scene
    nativekit_scene_render nativekit_sim_core robotkit_runtime cadkit-core)
if(MATERIA_ENABLE_MUJOCO)
    list(APPEND MATERIA_RUNTIME_TARGETS nativekit_sim_mujoco)
endif()
foreach(target_name IN LISTS MATERIA_RUNTIME_TARGETS)
    if(NOT TARGET "${target_name}")
        message(FATAL_ERROR "Materia runtime package did not define ${target_name}")
    endif()
    set_target_properties("${target_name}" PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
        RUNTIME_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
        ARCHIVE_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")
endforeach()

# Haxeon's coarse CMake artifact is a build stamp.  The runtime libraries
# loaded by the generated FFI calls are the libnativekit*.so files above.
add_custom_target(haxeon_app_native
    DEPENDS ${MATERIA_RUNTIME_TARGETS}
    COMMAND "${CMAKE_COMMAND}" -E touch "${HAXEON_NATIVE_OUTPUT_DIR}/app.hdll"
    BYPRODUCTS "${HAXEON_NATIVE_OUTPUT_DIR}/app.hdll"
    COMMENT "Building NativeKit, GPU, and UIKit native dependencies")
