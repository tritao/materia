cmake_minimum_required(VERSION 3.21)
project(robotkit_core_package_consumer LANGUAGES C CXX)
set(RK_BUILD_SIMKIT OFF CACHE BOOL "Core consumer has no simulation" FORCE)
set(RK_BUILD_POLICY OFF CACHE BOOL "Core consumer has no policy runtime" FORCE)
set(RK_BUILD_INFERENCE OFF CACHE BOOL "Core consumer has no image inference" FORCE)
set(RK_BUILD_VISIONKIT OFF CACHE BOOL "Core consumer has no vision stack" FORCE)
set(RK_BUILD_TESTS OFF CACHE BOOL "Build only this consumer" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../.." robotkit)

# The haxeon-platform package robotkit-core depends on loads NativeKit (its monotonic clock).
set(NK_BUILD_SHARED ON CACHE BOOL "Build NativeKit as a shared library" FORCE)
set(NK_BUILD_TESTS OFF CACHE BOOL "Core consumer has no NativeKit tests" FORCE)
set(NK_BUILD_EXAMPLES OFF CACHE BOOL "Core consumer has no NativeKit examples" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../../../haxeon/vendor/nativekit" nativekit)
if(DEFINED HAXEON_NATIVE_OUTPUT_DIR)
    set_target_properties(nativekit PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
        RUNTIME_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
        ARCHIVE_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")
endif()
add_custom_target(robotkit_core_consumer_native DEPENDS robotkit_runtime nativekit)
