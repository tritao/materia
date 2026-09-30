cmake_minimum_required(VERSION 3.21)

# Emscripten host for the browser build of the reference editor. The editor
# itself is a separate Haxeon wasm32 guest (see build.sh); this executable
# supplies the browser runtime and the native C ABIs the guest imports, all in
# one linear memory that the guest shares through the Haxeon memory contract.
project(materia_web_host VERSION 0.1.0 LANGUAGES C CXX)

if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "app/web is the Emscripten host; configure it with emcmake")
endif()

set(MATERIA_WEB_GUEST_WASM "" CACHE FILEPATH "Haxeon guest module the host exports functions to")
set(MATERIA_WEB_EXPORTS_FILE "" CACHE FILEPATH
    "Emscripten response file listing the C functions the guest imports from linked libraries")
if(NOT EXISTS "${MATERIA_WEB_GUEST_WASM}" OR NOT EXISTS "${MATERIA_WEB_EXPORTS_FILE}")
    message(FATAL_ERROR "Build the browser editor with app/web/build.sh, which compiles the guest first")
endif()

get_filename_component(MATERIA_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

set(NK_BUILD_SHARED OFF CACHE BOOL "Emscripten links one static host" FORCE)
set(NK_BUILD_GPU ON CACHE BOOL "The editor viewport renders through NativeKit GPU" FORCE)
set(NK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(NK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(NK_SOKOL_BACKEND gles3 CACHE STRING "WebGL2" FORCE)
add_subdirectory("${MATERIA_SOURCE_ROOT}/nativekit" nativekit)
add_subdirectory("${MATERIA_SOURCE_ROOT}/uikit" uikit)

set(NKSCENE_NATIVEKIT_DIR "${MATERIA_SOURCE_ROOT}/nativekit")
set(NK_LIBRARY_TYPE STATIC)
add_subdirectory("${MATERIA_SOURCE_ROOT}/scenekit/scene" scene)
add_subdirectory("${MATERIA_SOURCE_ROOT}/scenekit/scene_render" scene_render)

# The host heap ends where the guest's managed heap begins; the guest owns the
# rest of the fixed-size memory. Values are byte offsets, multiples of 64 KiB.
set(MATERIA_WEB_GUEST_MEMORY_LIMIT 805306368 CACHE STRING "Total linear memory in bytes")
set(NKUI_HAXEON_MEMORY_CONTRACT_VERSION 1)
set(NKUI_HAXEON_WASM_PAGE_SIZE 65536)
set(NKUI_HAXEON_HOST_HEAP_BASE 0)
set(NKUI_HAXEON_HOST_HEAP_LIMIT ${NK_WASM_HOST_HEAP_LIMIT})
set(NKUI_HAXEON_GUEST_MEMORY_BASE ${NKUI_HAXEON_HOST_HEAP_LIMIT})
set(NKUI_HAXEON_GUEST_MEMORY_LIMIT ${MATERIA_WEB_GUEST_MEMORY_LIMIT})
set(NKUI_HAXEON_MEMORY_LIMIT ${MATERIA_WEB_GUEST_MEMORY_LIMIT})
configure_file("${MATERIA_SOURCE_ROOT}/uikit/cmake/wasm_memory_contract.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/nativekit_haxeon_memory_contract.h" @ONLY)

add_executable(materia_web
    host.cpp
    "${MATERIA_SOURCE_ROOT}/uikit/src/platform/wasm/haxeon_memory_contract.cpp")
target_include_directories(materia_web PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(materia_web PRIVATE
    NativeKit::nativekit NativeKit::gpu NativeKit::ui NativeKit::scene NativeKit::scene_render
    NativeKit::wasm_host_allocator)
target_link_options(materia_web PRIVATE
    "-sINITIAL_MEMORY=${MATERIA_WEB_GUEST_MEMORY_LIMIT}"
    "-sALLOW_MEMORY_GROWTH=0"
    "-sEXPORTED_FUNCTIONS=@${MATERIA_WEB_EXPORTS_FILE}"
    # The page adds the guest's native callbacks to the function table.
    "-sEXPORTED_RUNTIME_METHODS=addFunction,removeFunction"
    "-sALLOW_TABLE_GROWTH=1"
    "-sNO_EXIT_RUNTIME=1")
set_property(TARGET materia_web APPEND PROPERTY LINK_DEPENDS "${MATERIA_WEB_EXPORTS_FILE}")
