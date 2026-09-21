cmake_minimum_required(VERSION 3.21)

project(materia_reference_editor_native LANGUAGES C CXX)

if(NOT DEFINED HAXEON_NATIVE_OUTPUT_DIR OR HAXEON_NATIVE_OUTPUT_DIR STREQUAL "")
    set(HAXEON_NATIVE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/native")
endif()
file(MAKE_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")

# The app owns this aggregate provider.  NativeKit and UIKit remain ordinary
# CMake projects, but one Haxeon action configures and builds their complete
# runtime dependency chain together.
set(NK_BUILD_SHARED ON CACHE BOOL "Build shared NativeKit libraries" FORCE)
set(NK_BUILD_GPU ON CACHE BOOL "Build NativeKit GPU" FORCE)
set(NK_BUILD_TESTS OFF CACHE BOOL "Disable dependency tests in the app build" FORCE)
set(NK_BUILD_EXAMPLES OFF CACHE BOOL "Disable dependency examples in the app build" FORCE)
set(NKUI_ENABLE_SECTION_GC ON CACHE BOOL "Keep UIKit section garbage collection enabled" FORCE)

set(MATERIA_NATIVEKIT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../nativekit")
set(MATERIA_UIKIT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../uikit")

add_subdirectory("${MATERIA_NATIVEKIT_DIR}" nativekit)
add_subdirectory("${MATERIA_UIKIT_DIR}" uikit)

foreach(target_name nativekit nativekit_gpu nativekit_ui)
    if(TARGET "${target_name}")
        set_target_properties("${target_name}" PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
            RUNTIME_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
            ARCHIVE_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")
    endif()
endforeach()

# Haxeon's coarse CMake artifact is a build stamp.  The runtime libraries
# loaded by the generated FFI calls are the libnativekit*.so files above.
add_custom_target(haxeon_app_native
    DEPENDS nativekit nativekit_gpu nativekit_ui
    COMMAND "${CMAKE_COMMAND}" -E touch "${HAXEON_NATIVE_OUTPUT_DIR}/app.hdll"
    BYPRODUCTS "${HAXEON_NATIVE_OUTPUT_DIR}/app.hdll"
    COMMENT "Building NativeKit, GPU, and UIKit native dependencies")
