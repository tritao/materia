cmake_minimum_required(VERSION 3.21)

project(cadkit_native LANGUAGES C CXX)

if(NOT DEFINED HAXEON_NATIVE_OUTPUT_DIR OR HAXEON_NATIVE_OUTPUT_DIR STREQUAL "")
    set(HAXEON_NATIVE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/native")
endif()
file(MAKE_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")

# The shared CadKit core that the Haxe bindings load. Set CADKIT_OCCT_DIR to a prebuilt OCCT install of the pinned
# source; without it this configure builds OCCT, which takes most of an hour.
set(CADKIT_BUILD_TESTS OFF CACHE BOOL "Disable CadKit tests in the Haxeon build" FORCE)
set(CADKIT_BUILD_SHARED ON CACHE BOOL "Build the CadKit core as a shared library" FORCE)
set(BUILD_LIBRARY_TYPE Shared CACHE STRING "Build OCCT as shared libraries" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/.." cadkit)

set_target_properties(cadkit-core PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")
if(UNIX AND NOT APPLE)
    target_link_options(cadkit-core PRIVATE "LINKER:-z,defs")
endif()
