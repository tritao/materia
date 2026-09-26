cmake_minimum_required(VERSION 3.21)
project(motionkit_core VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
if(NOT DEFINED RK_BUILD_SHARED)
    set(RK_BUILD_SHARED ON)
endif()
option(MK_BUILD_SHARED "Build MotionKit as a shared library" ${RK_BUILD_SHARED})

if(MK_BUILD_SHARED)
    set(MK_LIBRARY_TYPE SHARED)
else()
    set(MK_LIBRARY_TYPE STATIC)
endif()

add_library(motionkit_core ${MK_LIBRARY_TYPE} src/trajectory.cpp)
target_compile_features(motionkit_core PUBLIC cxx_std_17)
target_include_directories(motionkit_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_definitions(motionkit_core PRIVATE MK_BUILDING_LIBRARY)
if(NOT MK_BUILD_SHARED)
    target_compile_definitions(motionkit_core PUBLIC MK_STATIC)
endif()
set_target_properties(motionkit_core PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(BUILD_TESTING)
    add_executable(motionkit_trajectory_tests tests/trajectory.cpp)
    target_link_libraries(motionkit_trajectory_tests PRIVATE motionkit_core)
    add_test(NAME motionkit_trajectory COMMAND motionkit_trajectory_tests)
endif()
