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
set(RK_BUILD_SIMKIT_ENDPOINT ON CACHE BOOL
    "Build the SimKit-backed RobotKit endpoint" FORCE)
set(NK_BUILD_SHARED ON CACHE BOOL "Build SimKit dependencies as shared libraries" FORCE)
set(NK_BUILD_TESTS OFF CACHE BOOL "Disable SimKit tests in robotd" FORCE)
set(NK_BUILD_EXAMPLES OFF CACHE BOOL "Disable SimKit examples in robotd" FORCE)
set(NKSIM_BUILD_MUJOCO OFF CACHE BOOL "Disable MuJoCo for the first robotd endpoint" FORCE)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../../simkit" simkit)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../.." robotkit)

foreach(target_name
        nativekit nativekit_scene nativekit_scene_render nativekit_scene_interaction
        nativekit_sim_core robotkit_core robotkit_runtime)
    if(TARGET "${target_name}")
        set_target_properties("${target_name}" PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
            RUNTIME_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
            ARCHIVE_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")
    endif()
endforeach()

add_custom_target(robotd_native
    DEPENDS robotkit_core robotkit_runtime
    COMMAND "${CMAKE_COMMAND}" -E touch "${HAXEON_NATIVE_OUTPUT_DIR}/robotd.hdll"
    BYPRODUCTS "${HAXEON_NATIVE_OUTPUT_DIR}/robotd.hdll"
    COMMENT "Building RobotKit native dependencies for robotd")
