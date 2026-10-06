cmake_minimum_required(VERSION 3.21)

project(SceneKit VERSION 0.1.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CTest)

option(NK_BUILD_SHARED "Build SceneKit and NativeKit as shared libraries" ON)
option(NK_BUILD_TESTS "Build SceneKit and NativeKit tests" ${BUILD_TESTING})
set(NKSCENE_NATIVEKIT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../haxeon/vendor/nativekit" CACHE PATH
    "Path to the NativeKit source checkout")

set(NK_BUILD_GPU ON CACHE BOOL "Build NativeKit GPU for SceneKit" FORCE)
add_subdirectory("${NKSCENE_NATIVEKIT_DIR}" "${CMAKE_BINARY_DIR}/nativekit")

set(NK_LIBRARY_TYPE STATIC)
if(NK_BUILD_SHARED AND NOT EMSCRIPTEN)
    set(NK_LIBRARY_TYPE SHARED)
endif()

add_subdirectory(scene)
add_subdirectory(scene_render)
add_subdirectory(scene_interaction)

# Test and benchmark executables use assert for setup as well as checks.
if(NK_BUILD_TESTS)
    foreach(scene_dir scene scene_render scene_interaction)
        get_property(scene_targets DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${scene_dir}"
            PROPERTY BUILDSYSTEM_TARGETS)
        foreach(scene_target IN LISTS scene_targets)
            get_target_property(scene_type "${scene_target}" TYPE)
            if(scene_type STREQUAL "EXECUTABLE")
                if(MSVC)
                    target_compile_options("${scene_target}" PRIVATE /UNDEBUG)
                else()
                    target_compile_options("${scene_target}" PRIVATE -UNDEBUG)
                endif()
            endif()
        endforeach()
    endforeach()
endif()
