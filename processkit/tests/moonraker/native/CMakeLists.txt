cmake_minimum_required(VERSION 3.21)
project(moonraker_native LANGUAGES C CXX)
set(NK_BUILD_SHARED ON CACHE BOOL "" FORCE)
set(NK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(NK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(NK_USE_SYSTEM_CURL ON CACHE BOOL "" FORCE)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(CURL REQUIRED)
endif()
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../../../haxeon/vendor/nativekit" nativekit)

set_target_properties(nativekit PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")
add_custom_target(moonraker_native DEPENDS nativekit)
