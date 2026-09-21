cmake_minimum_required(VERSION 3.21)

project(RobotKit VERSION 0.1.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CTest)

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

    if(RK_BUILD_TESTS)
        add_executable(robotkit_simkit_tests runtime/tests/simkit.cpp)
        target_compile_features(robotkit_simkit_tests PRIVATE cxx_std_20)
        target_link_libraries(robotkit_simkit_tests PRIVATE RobotKit::runtime)
        add_test(NAME robotkit_simkit COMMAND robotkit_simkit_tests)
        set_tests_properties(robotkit_simkit PROPERTIES LABELS "robot;runtime;simkit"
            TIMEOUT 30)
    endif()
endif()
