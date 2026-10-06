cmake_minimum_required(VERSION 3.24)
project(animkit_tests_native VERSION 0.1.0 LANGUAGES C CXX)

if(NOT DEFINED HAXEON_NATIVE_OUTPUT_DIR OR HAXEON_NATIVE_OUTPUT_DIR STREQUAL "")
    set(HAXEON_NATIVE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/native")
endif()
file(MAKE_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")

get_filename_component(MATERIA_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(NK_BUILD_SHARED ON CACHE BOOL "" FORCE)
set(NK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(NK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(NK_BUILD_GPU ON CACHE BOOL "" FORCE)
add_subdirectory("${MATERIA_SOURCE_ROOT}/haxeon/vendor/nativekit" nativekit)
set(NKSCENE_NATIVEKIT_DIR "${MATERIA_SOURCE_ROOT}/haxeon/vendor/nativekit")
set(NK_LIBRARY_TYPE SHARED)
add_subdirectory("${MATERIA_SOURCE_ROOT}/scenekit/scene" scene)
add_subdirectory("${MATERIA_SOURCE_ROOT}/scenekit/scene_render" scene_render)
set(BUILD_TESTING OFF)
add_subdirectory("${MATERIA_SOURCE_ROOT}/animkit/native" animkit)

set(ANIMKIT_TEST_RUNTIME_TARGETS nativekit nativekit_gpu nativekit_scene nativekit_scene_render animkit_core)
foreach(target_name IN LISTS ANIMKIT_TEST_RUNTIME_TARGETS)
    set_target_properties("${target_name}" PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
        RUNTIME_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")
endforeach()
add_custom_target(animkit_tests_native DEPENDS ${ANIMKIT_TEST_RUNTIME_TARGETS})
