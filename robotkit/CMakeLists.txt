cmake_minimum_required(VERSION 3.21)

project(RobotKit VERSION 0.1.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CTest)
include(FetchContent)

# Header-only MCAP C++ implementation, pinned for reproducible recordings.
FetchContent_Declare(mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG 1420296ffcfdcde4b6894c0c1aba0ad083f93dde
    GIT_SHALLOW TRUE)
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()
FetchContent_GetProperties(mcap)
if(NOT mcap_POPULATED)
    FetchContent_Populate(mcap)
endif()
add_library(robotkit_mcap_headers INTERFACE)
target_include_directories(robotkit_mcap_headers INTERFACE
    "${mcap_SOURCE_DIR}/cpp/mcap/include")

option(RK_BUILD_SHARED "Build RobotKit libraries as shared libraries" ON)
option(RK_BUILD_TESTS "Build RobotKit tests" ${BUILD_TESTING})
option(RK_BUILD_SIMKIT "Build the SimKit-backed RobotKit simulation" OFF)

set(RK_LIBRARY_TYPE STATIC)
if(RK_BUILD_SHARED AND NOT EMSCRIPTEN)
    set(RK_LIBRARY_TYPE SHARED)
endif()

add_subdirectory(runtime)

if(RK_BUILD_SIMKIT)
    if(NOT TARGET NativeKit::sim_core)
        message(FATAL_ERROR
            "RK_BUILD_SIMKIT requires SimKit's NativeKit::sim_core target")
    endif()
    target_sources(robotkit_runtime PRIVATE
        runtime/src/simulation.cpp
        runtime/src/simulation_robot.cpp
        runtime/src/simulation_c_api.cpp)
    target_compile_definitions(robotkit_runtime PRIVATE RK_HAS_SIMKIT=1)
    target_link_libraries(robotkit_runtime PUBLIC NativeKit::sim_core)
    if(TARGET NativeKit::sim_mujoco)
        target_compile_definitions(robotkit_runtime PRIVATE RK_HAS_MUJOCO=1)
        target_link_libraries(robotkit_runtime PRIVATE NativeKit::sim_mujoco)
    endif()

    if(RK_BUILD_TESTS)
        add_executable(robotkit_simkit_tests runtime/tests/simkit.cpp)
        target_compile_features(robotkit_simkit_tests PRIVATE cxx_std_20)
        target_link_libraries(robotkit_simkit_tests PRIVATE RobotKit::runtime)
        add_test(NAME robotkit_simkit COMMAND robotkit_simkit_tests)
        set_tests_properties(robotkit_simkit PROPERTIES LABELS "robot;runtime;simkit"
            TIMEOUT 30)
        if(TARGET NativeKit::sim_mujoco)
            add_executable(robotkit_mujoco_tests runtime/tests/mujoco.cpp)
            target_link_libraries(robotkit_mujoco_tests PRIVATE RobotKit::runtime)
            add_test(NAME robotkit_mujoco COMMAND robotkit_mujoco_tests)
            set_tests_properties(robotkit_mujoco PROPERTIES LABELS "robot;sensors;mujoco" TIMEOUT 60)
            add_executable(robotkit_mujoco_backend_tests ../simkit/sim_mujoco/tests/mujoco.cpp)
            target_compile_features(robotkit_mujoco_backend_tests PRIVATE cxx_std_20)
            target_link_libraries(robotkit_mujoco_backend_tests PRIVATE NativeKit::sim_mujoco)
            add_test(NAME robotkit_mujoco_backend COMMAND robotkit_mujoco_backend_tests)
            set_tests_properties(robotkit_mujoco_backend PROPERTIES LABELS "sim;mujoco" TIMEOUT 60)
        endif()
    endif()
endif()
