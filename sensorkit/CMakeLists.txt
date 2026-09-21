cmake_minimum_required(VERSION 3.21)

project(SensorKit VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CTest)

option(NK_BUILD_SHARED "Build SensorKit as a shared library" ON)
option(NK_BUILD_TESTS "Build SensorKit tests" ${BUILD_TESTING})
option(NKSENSOR_BUILD_SIM "Build the SimKit truth bridge" ON)
option(NKSENSOR_BUILD_RENDER "Build the SceneKit rendering bridge" ON)
set(NKSENSOR_SIMKIT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../simkit" CACHE PATH
    "Path to the SimKit source checkout")
set(NKSENSOR_SCENEKIT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../scenekit" CACHE PATH
    "Path to the SceneKit source checkout")

set(NK_LIBRARY_TYPE STATIC)
if(NK_BUILD_SHARED AND NOT EMSCRIPTEN)
    set(NK_LIBRARY_TYPE SHARED)
endif()

add_subdirectory(sensor_core)
add_subdirectory(sensor_wire)
add_subdirectory(sensor_stream)

if(NKSENSOR_BUILD_SIM)
    set(NKSENSOR_PARENT_BUILD_TESTS ${NK_BUILD_TESTS})
    set(NK_BUILD_EXAMPLES OFF CACHE BOOL "Build SimKit examples" FORCE)
    set(NK_BUILD_TESTS OFF CACHE BOOL "Build SimKit dependency tests" FORCE)
    set(NKSIM_BUILD_MUJOCO OFF CACHE BOOL "Build the MuJoCo backend" FORCE)
    add_subdirectory("${NKSENSOR_SIMKIT_DIR}" "${CMAKE_BINARY_DIR}/simkit")
    set(NK_BUILD_TESTS ${NKSENSOR_PARENT_BUILD_TESTS} CACHE BOOL
        "Build SensorKit tests" FORCE)
    set(NK_BUILD_TESTS ${NKSENSOR_PARENT_BUILD_TESTS})
    add_subdirectory(sensor_sim)
endif()

if(NKSENSOR_BUILD_RENDER)
    if(NOT TARGET NativeKit::scene_render)
        set(NKSENSOR_PARENT_BUILD_TESTS ${NK_BUILD_TESTS})
        set(NK_BUILD_TESTS OFF CACHE BOOL "Build SceneKit dependency tests" FORCE)
        add_subdirectory("${NKSENSOR_SCENEKIT_DIR}" "${CMAKE_BINARY_DIR}/scenekit")
        set(NK_BUILD_TESTS ${NKSENSOR_PARENT_BUILD_TESTS} CACHE BOOL
            "Build SensorKit tests" FORCE)
        set(NK_BUILD_TESTS ${NKSENSOR_PARENT_BUILD_TESTS})
    endif()
    add_subdirectory(sensor_render)
endif()
