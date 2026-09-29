cmake_minimum_required(VERSION 3.21)
project(visionkit_core VERSION 0.1.0 LANGUAGES C CXX)

option(VK_BUILD_TESTS "Build VisionKit native tests" ON)
set(VK_OPENCV_THREADS 1 CACHE STRING "Maximum OpenCV worker threads")
if(NOT VK_OPENCV_THREADS MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "VK_OPENCV_THREADS must be a positive integer")
endif()
include(cmake/OpenCV.cmake)
find_package(ZLIB REQUIRED)

add_library(visionkit_core SHARED src/visionkit.cpp src/fiducial.cpp src/calibration.cpp)
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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../animkit/native/vendor/stb/stb_image.h")
    add_executable(visionkit_calibrate tools/calibrate/main.cpp)
    target_compile_features(visionkit_calibrate PRIVATE cxx_std_17)
    target_include_directories(visionkit_calibrate PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../../animkit/native/vendor/stb")
    target_link_libraries(visionkit_calibrate PRIVATE VisionKit::core)
else()
    message(WARNING "stb_image submodule absent; visionkit_calibrate target unavailable")
endif()

if(VK_BUILD_TESTS)
    enable_testing()
    add_executable(visionkit_smoke tests/smoke.cpp)
    target_link_libraries(visionkit_smoke PRIVATE VisionKit::core)
    add_test(NAME visionkit_smoke COMMAND visionkit_smoke)
    add_executable(visionkit_camera_tests tests/camera.cpp)
    target_link_libraries(visionkit_camera_tests PRIVATE VisionKit::core)
    add_test(NAME visionkit_camera COMMAND visionkit_camera_tests)
    add_executable(visionkit_fiducial_tests tests/fiducial.cpp)
    target_link_libraries(visionkit_fiducial_tests PRIVATE VisionKit::core
        ${VK_OPENCV_LIBRARIES} ZLIB::ZLIB)
    add_test(NAME visionkit_fiducial COMMAND visionkit_fiducial_tests)
    if(TARGET visionkit_calibrate)
        add_executable(visionkit_calibration_tests tests/calibration.cpp)
        target_include_directories(visionkit_calibration_tests PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/../../animkit/native/vendor/stb")
        target_link_libraries(visionkit_calibration_tests PRIVATE VisionKit::core
            ${VK_OPENCV_LIBRARIES} ZLIB::ZLIB)
        target_compile_options(visionkit_calibration_tests PRIVATE -UNDEBUG)
        add_test(NAME visionkit_calibration COMMAND visionkit_calibration_tests $<TARGET_FILE:visionkit_calibrate>)
    endif()
    if(MSVC)
        target_compile_options(visionkit_camera_tests PRIVATE /UNDEBUG)
        target_compile_options(visionkit_fiducial_tests PRIVATE /UNDEBUG)
    else()
        target_compile_options(visionkit_camera_tests PRIVATE -UNDEBUG)
        target_compile_options(visionkit_fiducial_tests PRIVATE -UNDEBUG)
    endif()
endif()
