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
    $<BUILD_INTERFACE:${mcap_SOURCE_DIR}/cpp/mcap/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# MCAP's LZ4 support needs only four small C sources. A system installation can
# be selected explicitly for distribution builds; source builds stay pinned.
option(RK_USE_SYSTEM_LZ4 "Use a system LZ4 library" OFF)
if(RK_USE_SYSTEM_LZ4)
    find_path(RK_LZ4_INCLUDE_DIR lz4frame.h REQUIRED)
    find_library(RK_LZ4_LIBRARY NAMES lz4 REQUIRED)
    add_library(robotkit_lz4 INTERFACE)
    target_include_directories(robotkit_lz4 INTERFACE
        $<BUILD_INTERFACE:${RK_LZ4_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
    target_link_libraries(robotkit_lz4 INTERFACE
        $<BUILD_INTERFACE:${RK_LZ4_LIBRARY}>
        $<INSTALL_INTERFACE:lz4>)
else()
    FetchContent_Declare(robotkit_lz4_source
        GIT_REPOSITORY https://github.com/lz4/lz4.git
        GIT_TAG ebb370ca83af193212df4dcbadcc5d87bc0de2f0
        GIT_SHALLOW FALSE)
    FetchContent_GetProperties(robotkit_lz4_source)
    if(NOT robotkit_lz4_source_POPULATED)
        FetchContent_Populate(robotkit_lz4_source)
    endif()
    add_library(robotkit_lz4 STATIC
        "${robotkit_lz4_source_SOURCE_DIR}/lib/lz4.c"
        "${robotkit_lz4_source_SOURCE_DIR}/lib/lz4frame.c"
        "${robotkit_lz4_source_SOURCE_DIR}/lib/lz4hc.c"
        "${robotkit_lz4_source_SOURCE_DIR}/lib/xxhash.c")
    target_include_directories(robotkit_lz4 PUBLIC
        $<BUILD_INTERFACE:${robotkit_lz4_source_SOURCE_DIR}/lib>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
    set_target_properties(robotkit_lz4 PROPERTIES POSITION_INDEPENDENT_CODE ON)
    install(FILES "${robotkit_lz4_source_SOURCE_DIR}/lib/lz4.h"
        "${robotkit_lz4_source_SOURCE_DIR}/lib/lz4frame.h"
        "${robotkit_lz4_source_SOURCE_DIR}/lib/lz4hc.h"
        "${robotkit_lz4_source_SOURCE_DIR}/lib/xxhash.h"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
install(TARGETS robotkit_mcap_headers robotkit_lz4 EXPORT RobotKitTargets)
install(DIRECTORY "${mcap_SOURCE_DIR}/cpp/mcap/include/" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

option(RK_BUILD_SHARED "Build RobotKit libraries as shared libraries" ON)
option(RK_BUILD_TESTS "Build RobotKit tests" ${BUILD_TESTING})
option(RK_BUILD_SIMKIT "Build the SimKit-backed RobotKit simulation" OFF)
option(RK_BUILD_POLICY "Build the ONNX Runtime policy library (fetches a pinned prebuilt release)" OFF)
option(RK_BUILD_VISIONKIT "Build the optional VisionKit camera geometry library" OFF)
option(RK_BUILD_INFERENCE "Build the ONNX Runtime image inference library" OFF)

if(RK_BUILD_TESTS)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME robotkit_rkf1_lock
        COMMAND ${Python3_EXECUTABLE} -m tools.wire.rkf1 check
            --source ${CMAKE_CURRENT_SOURCE_DIR}/haxe/robotkit/protocol
            --lock ${CMAKE_CURRENT_SOURCE_DIR}/schema/rkf1.lock.json)
    set_tests_properties(robotkit_rkf1_lock PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/.. LABELS "robot;wire")
endif()

set(RK_LIBRARY_TYPE STATIC)
if(RK_BUILD_SHARED AND NOT EMSCRIPTEN)
    set(RK_LIBRARY_TYPE SHARED)
endif()

if(NOT TARGET motionkit_core)
    set(MK_BUILD_SHARED ${RK_BUILD_SHARED} CACHE BOOL "Build MotionKit with RobotKit" FORCE)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../motionkit/native" motionkit)
endif()

if(RK_BUILD_VISIONKIT AND NOT TARGET visionkit_core)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../visionkit/native" visionkit)
endif()

add_subdirectory(runtime)

if(RK_BUILD_POLICY)
    add_subdirectory(policy)
endif()
if(RK_BUILD_INFERENCE)
    add_subdirectory(inference)
endif()

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
            target_link_libraries(robotkit_mujoco_tests PRIVATE RobotKit::runtime NativeKit::sim_mujoco)
            add_test(NAME robotkit_mujoco COMMAND robotkit_mujoco_tests)
            set_tests_properties(robotkit_mujoco PROPERTIES LABELS "robot;sensors;mujoco" TIMEOUT 60)
            add_executable(robotkit_mixed_scene_tests runtime/tests/mixed_scene.cpp)
            target_compile_features(robotkit_mixed_scene_tests PRIVATE cxx_std_17)
            target_link_libraries(robotkit_mixed_scene_tests PRIVATE RobotKit::runtime NativeKit::sim_mujoco)
            add_test(NAME robotkit_mixed_scene COMMAND robotkit_mixed_scene_tests)
            set_tests_properties(robotkit_mixed_scene PROPERTIES LABELS "robot;mujoco;mixed" TIMEOUT 120)
            add_executable(robotkit_mujoco_backend_tests ../simkit/sim_mujoco/tests/mujoco.cpp)
            target_compile_features(robotkit_mujoco_backend_tests PRIVATE cxx_std_20)
            target_link_libraries(robotkit_mujoco_backend_tests PRIVATE NativeKit::sim_mujoco)
            add_test(NAME robotkit_mujoco_backend COMMAND robotkit_mujoco_backend_tests)
            set_tests_properties(robotkit_mujoco_backend PROPERTIES LABELS "sim;mujoco" TIMEOUT 60)
        endif()
    endif()

    # MJCF import compiles the file with MuJoCo itself (plans/HUMANOID.md, HU-D2).
    if(TARGET mujoco::mujoco)
        add_executable(robotkit_mjcf_import tools/mjcf_import/mjcf_import.cpp)
        # The conformance reference steps the same file in plain MuJoCo.
        add_executable(robotkit_mjcf_reference tools/mjcf_import/mjcf_reference.cpp)
        foreach(tool robotkit_mjcf_import robotkit_mjcf_reference)
            target_compile_features(${tool} PRIVATE cxx_std_17)
            # Mesh decoders (STL, OBJ) register themselves from static
            # constructors that nothing references, so a static MuJoCo must
            # link whole.
            if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
                target_link_libraries(${tool} PRIVATE
                    "$<LINK_LIBRARY:WHOLE_ARCHIVE,mujoco::mujoco>")
            else()
                message(WARNING "CMake 3.24 or newer is needed to load MJCF files with mesh assets")
                target_link_libraries(${tool} PRIVATE mujoco::mujoco)
            endif()
        endforeach()
        if(RK_BUILD_TESTS)
            add_test(NAME robotkit_mjcf_import COMMAND "${CMAKE_COMMAND}"
                "-DIMPORTER=$<TARGET_FILE:robotkit_mjcf_import>"
                "-DMODEL=${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/mjcf/walker.xml"
                "-DEXPECTED=${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/mjcf/walker.robot.json"
                "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/mjcf-walker"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/tools/mjcf_import/check_import.cmake")
            set_tests_properties(robotkit_mjcf_import PROPERTIES LABELS "robot;mujoco;import" TIMEOUT 60)
        endif()
    endif()
endif()

# These tests use assert for setup calls as well as checks. Keep them active in Release.
get_property(_assert_test_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(_assert_test_target IN LISTS _assert_test_targets)
    if(_assert_test_target MATCHES "_tests$")
        if(MSVC)
            target_compile_options(${_assert_test_target} PRIVATE /UNDEBUG)
        else()
            target_compile_options(${_assert_test_target} PRIVATE -UNDEBUG)
        endif()
    endif()
endforeach()
