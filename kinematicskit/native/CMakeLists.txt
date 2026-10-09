cmake_minimum_required(VERSION 3.21)
project(kinematicskit_core VERSION 0.1.0 LANGUAGES C CXX)

option(KK_BUILD_SHARED "Build KinematicsKit core as a shared library" ON)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(KK_BUILD_SHARED)
    set(KK_LIBRARY_TYPE SHARED)
else()
    set(KK_LIBRARY_TYPE STATIC)
endif()

# Eigen stays private to the library (KK-D10): nothing in the C ABI names it.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# proxsuite is used header-only (its dense solver needs only Eigen); see
# THIRD_PARTY.md. Its own CMake, submodules and optional extras are not used.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/proxsuite/include/proxsuite/proxqp/dense/dense.hpp")
    message(FATAL_ERROR "Initialize kinematicskit/native/vendor/proxsuite submodule")
endif()

add_library(kinematicskit_core ${KK_LIBRARY_TYPE}
    src/api.cpp
    src/model.cpp
    src/qp.cpp
)
target_compile_features(kinematicskit_core PUBLIC c_std_11 PRIVATE cxx_std_17)
target_include_directories(kinematicskit_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kinematicskit_core PRIVATE Eigen3::Eigen)
target_include_directories(kinematicskit_core SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/proxsuite/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proxsuite_config)
set_target_properties(kinematicskit_core PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    POSITION_INDEPENDENT_CODE ON)
if(KK_BUILD_SHARED)
    target_compile_definitions(kinematicskit_core PRIVATE KK_BUILDING_LIBRARY=1)
else()
    target_compile_definitions(kinematicskit_core PUBLIC KK_STATIC=1)
endif()
if(MSVC)
    target_compile_options(kinematicskit_core PRIVATE /W4)
else()
    # No fused multiply-add contraction: the native arithmetic must match the
    # Haxe kit operation for operation.
    target_compile_options(kinematicskit_core PRIVATE -Wall -Wextra -ffp-contract=off)
endif()
add_library(kinematicskit::core ALIAS kinematicskit_core)
