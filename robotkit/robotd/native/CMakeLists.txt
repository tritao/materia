cmake_minimum_required(VERSION 3.21)

project(materia_robotd_native LANGUAGES C CXX)

include(CTest)

option(ROBOTD_BUILD_TESTS "Build RobotKit endpoint tests for robotd" OFF)

if(NOT DEFINED HAXEON_NATIVE_OUTPUT_DIR OR HAXEON_NATIVE_OUTPUT_DIR STREQUAL "")
    set(HAXEON_NATIVE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/native")
endif()
file(MAKE_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")

set(RK_BUILD_SHARED ON CACHE BOOL "Build RobotKit libraries as shared libraries" FORCE)
set(RK_BUILD_TESTS ${ROBOTD_BUILD_TESTS} CACHE BOOL
    "Build RobotKit tests in robotd" FORCE)
set(RK_BUILD_SIMKIT ON CACHE BOOL
    "Build the SimKit-backed RobotKit simulation" FORCE)
set(RK_BUILD_INFERENCE ON CACHE BOOL
    "Build the image inference module used by robotd" FORCE)
set(NK_BUILD_SHARED ON CACHE BOOL "Build SimKit dependencies as shared libraries" FORCE)
set(NK_BUILD_TESTS OFF CACHE BOOL "Disable SimKit tests in robotd" FORCE)
set(NK_BUILD_EXAMPLES OFF CACHE BOOL "Disable SimKit examples in robotd" FORCE)
option(NKSIM_BUILD_MUJOCO "Build the optional MuJoCo simulation backend" OFF)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../../simkit" simkit)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../.." robotkit)

foreach(target_name
        nativekit nativekit_scene nativekit_scene_render nativekit_scene_interaction
        nativekit_sim_core nativekit_sim_mujoco trajectory_core visionkit_core
        robotkit_runtime robotkit_policy robotkit_inference)
    if(TARGET "${target_name}")
        set_target_properties("${target_name}" PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
            RUNTIME_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
            ARCHIVE_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")
    endif()
endforeach()

if(TARGET robotkit_policy)
    set(ROBOTD_POLICY_TARGET robotkit_policy)
endif()
if(TARGET visionkit_core)
    set(ROBOTD_VISIONKIT_TARGET visionkit_core)
endif()
if(TARGET robotkit_inference)
    set(ROBOTD_INFERENCE_TARGET robotkit_inference)
endif()

add_custom_target(robotd_native
    DEPENDS trajectory_core robotkit_runtime ${ROBOTD_POLICY_TARGET}
        ${ROBOTD_VISIONKIT_TARGET} ${ROBOTD_INFERENCE_TARGET}
    COMMENT "Building RobotKit native dependencies for robotd")
