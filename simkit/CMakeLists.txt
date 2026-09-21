cmake_minimum_required(VERSION 3.21)

project(SimKit VERSION 0.1.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CTest)

option(NK_BUILD_SHARED "Build SimKit dependencies as shared libraries" ON)
option(NK_BUILD_TESTS "Build SimKit dependency tests" ${BUILD_TESTING})
option(NK_BUILD_EXAMPLES "Build SimKit examples" ON)
option(NKSIM_BUILD_MUJOCO "Build the MuJoCo backend" ON)
option(NK_SIM_MUJOCO_USE_VENDORED "Use SimKit's pinned MuJoCo checkout" ON)

set(NKSIM_SCENEKIT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../scenekit" CACHE PATH
    "Path to the SceneKit source checkout")
set(NKSCENE_NATIVEKIT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../nativekit" CACHE PATH
    "Path to the NativeKit source checkout")
set(NK_MUJOCO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/mujoco" CACHE PATH
    "Path to the MuJoCo source checkout")

add_subdirectory("${NKSIM_SCENEKIT_DIR}" "${CMAKE_BINARY_DIR}/scenekit")

set(NK_LIBRARY_TYPE STATIC)
if(NK_BUILD_SHARED AND NOT EMSCRIPTEN)
    set(NK_LIBRARY_TYPE SHARED)
endif()

add_subdirectory(sim_core)
if(NKSIM_BUILD_MUJOCO)
    add_subdirectory(sim_mujoco)
endif()
