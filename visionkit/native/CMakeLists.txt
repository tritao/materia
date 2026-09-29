cmake_minimum_required(VERSION 3.21)
project(visionkit_core VERSION 0.1.0 LANGUAGES C CXX)

option(VK_BUILD_TESTS "Build VisionKit native tests" ON)
set(VK_OPENCV_THREADS 1 CACHE STRING "Maximum OpenCV worker threads")
if(NOT VK_OPENCV_THREADS MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "VK_OPENCV_THREADS must be a positive integer")
endif()
include(cmake/OpenCV.cmake)
find_package(ZLIB REQUIRED)

add_library(visionkit_core SHARED src/visionkit.cpp)
add_library(VisionKit::core ALIAS visionkit_core)
target_compile_features(visionkit_core PUBLIC cxx_std_17)
target_include_directories(visionkit_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_definitions(visionkit_core PRIVATE VK_BUILDING_LIBRARY
    VK_OPENCV_THREADS=${VK_OPENCV_THREADS})
target_link_libraries(visionkit_core PRIVATE ${VK_OPENCV_LIBRARIES} ZLIB::ZLIB)
set_target_properties(visionkit_core PROPERTIES
    CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION} SOVERSION 0)

if(VK_BUILD_TESTS)
    enable_testing()
    add_executable(visionkit_smoke tests/smoke.cpp)
    target_link_libraries(visionkit_smoke PRIVATE VisionKit::core)
    add_test(NAME visionkit_smoke COMMAND visionkit_smoke)
    add_executable(visionkit_camera_tests tests/camera.cpp)
    target_link_libraries(visionkit_camera_tests PRIVATE VisionKit::core)
    add_test(NAME visionkit_camera COMMAND visionkit_camera_tests)
    if(MSVC)
        target_compile_options(visionkit_camera_tests PRIVATE /UNDEBUG)
    else()
        target_compile_options(visionkit_camera_tests PRIVATE -UNDEBUG)
    endif()
endif()
