cmake_minimum_required(VERSION 3.21)

# Emscripten host for the browser build of the reference editor. The editor
# itself is a separate Haxeon guest module (see build.sh); this executable
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

# C++ exceptions and C setjmp/longjmp (qhull's error exits) are native Wasm exceptions throughout: MuJoCo's model
# compiler throws and catches its own errors, and without catching every throw aborts the page. Every object and the
# link must agree, so this precedes all libraries.
string(APPEND CMAKE_CXX_FLAGS " -fwasm-exceptions")
string(APPEND CMAKE_C_FLAGS " -fwasm-exceptions")

set(NK_BUILD_SHARED OFF CACHE BOOL "Emscripten links one static host" FORCE)
set(NK_BUILD_GPU ON CACHE BOOL "The editor viewport renders through NativeKit GPU" FORCE)
set(NK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(NK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(NK_SOKOL_BACKEND gles3 CACHE STRING "WebGL2" FORCE)
add_subdirectory("${MATERIA_SOURCE_ROOT}/haxeon/vendor/nativekit" nativekit)
set(NKUI_NATIVEKIT_DIR "${MATERIA_SOURCE_ROOT}/haxeon/vendor/nativekit" CACHE PATH "NativeKit source for the UI package" FORCE)
add_subdirectory("${MATERIA_SOURCE_ROOT}/haxeon/packages/ui" uikit)

set(NKSCENE_NATIVEKIT_DIR "${MATERIA_SOURCE_ROOT}/haxeon/vendor/nativekit")
set(NK_LIBRARY_TYPE STATIC)
add_subdirectory("${MATERIA_SOURCE_ROOT}/scenekit/scene" scene)
add_subdirectory("${MATERIA_SOURCE_ROOT}/scenekit/scene_render" scene_render)

# Simulation: SimKit's core and its MuJoCo backend. The editor steps sessions itself, so neither starts a thread;
# embedded MuJoCo builds single-threaded and without its JavaScript bindings. Its dependencies come through
# FetchContent (see build.sh for offline sources).
set(NKSIM_SCENEKIT_DIR "${MATERIA_SOURCE_ROOT}/scenekit")
set(NK_SIM_MUJOCO_USE_VENDORED ON CACHE BOOL "" FORCE)
set(NK_MUJOCO_SOURCE_DIR "${MATERIA_SOURCE_ROOT}/simkit/vendor/mujoco" CACHE PATH "" FORCE)
add_subdirectory("${MATERIA_SOURCE_ROOT}/simkit/sim_core" sim_core)
add_subdirectory("${MATERIA_SOURCE_ROOT}/simkit/sim_mujoco" sim_mujoco)

# RobotKit's runtime and its SimKit simulation, without the serial and simulated-board devices (no ports or cargo
# here). It brings MotionKit.
# MotionKit needs Eigen, which is header-only: Emscripten's find_package searches only its sysroot, so take the
# host's copy.
find_path(MATERIA_WEB_EIGEN3_DIR Eigen3Config.cmake
    PATHS /usr/share/eigen3/cmake /usr/lib/cmake/eigen3 /usr/local/share/eigen3/cmake /opt/homebrew/share/eigen3/cmake
    NO_CMAKE_FIND_ROOT_PATH)
if(MATERIA_WEB_EIGEN3_DIR)
    set(Eigen3_DIR "${MATERIA_WEB_EIGEN3_DIR}" CACHE PATH "" FORCE)
endif()
set(RK_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(RK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(RK_BUILD_SIMKIT ON CACHE BOOL "" FORCE)
add_subdirectory("${MATERIA_SOURCE_ROOT}/robotkit" robotkit)

# AnimKit (ozz-animation, glTF import) and StockKit (stock removal). StockKit's worker pool runs every job on the
# calling thread in a build without threads.
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(AK_BUILD_SHARED OFF CACHE BOOL "" FORCE)
add_subdirectory("${MATERIA_SOURCE_ROOT}/animkit/native" animkit)
set(SK_BUILD_SHARED OFF CACHE BOOL "" FORCE)
add_subdirectory("${MATERIA_SOURCE_ROOT}/stockkit/core" stockkit)

# CadKit, when given an Emscripten install of the pinned OCCT source (static, built with -fwasm-exceptions like
# everything here). Without one CadKit stays unavailable and the guest's calls throw haxeon.wasm.HostError.
set(MATERIA_WEB_OCCT_DIR "" CACHE PATH "Prefix of an Emscripten install of CadKit's pinned OCCT")
if(MATERIA_WEB_OCCT_DIR)
    # Emscripten confines find_package to its sysroot, so name the package directory itself.
    set(OpenCASCADE_DIR "${MATERIA_WEB_OCCT_DIR}/lib/cmake/opencascade" CACHE PATH "" FORCE)
    set(CADKIT_OCCT_DIR "${MATERIA_WEB_OCCT_DIR}" CACHE PATH "" FORCE)
    set(CADKIT_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(CADKIT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory("${MATERIA_SOURCE_ROOT}/cadkit" cadkit)
endif()

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
configure_file("${MATERIA_SOURCE_ROOT}/haxeon/packages/ui/cmake/wasm_memory_contract.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/nativekit_haxeon_memory_contract.h" @ONLY)

add_executable(materia_web
    host.cpp
    "${MATERIA_SOURCE_ROOT}/haxeon/packages/ui/src/platform/wasm/haxeon_memory_contract.cpp")
target_include_directories(materia_web PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(materia_web PRIVATE
    NativeKit::nativekit NativeKit::gpu NativeKit::ui NativeKit::scene NativeKit::scene_render
    NativeKit::sim_core NativeKit::sim_mujoco RobotKit::runtime animkit_core stockkit::core
    NativeKit::wasm_host_allocator)
if(MATERIA_WEB_OCCT_DIR)
    target_link_libraries(materia_web PRIVATE cadkit::core)
endif()
target_link_options(materia_web PRIVATE
    "-sINITIAL_MEMORY=${MATERIA_WEB_GUEST_MEMORY_LIMIT}"
    "-sALLOW_MEMORY_GROWTH=0"
    "-sEXPORTED_FUNCTIONS=@${MATERIA_WEB_EXPORTS_FILE}"
    # The page adds the guest's native callbacks to the function table.
    "-sEXPORTED_RUNTIME_METHODS=addFunction,removeFunction"
    "-sALLOW_TABLE_GROWTH=1"
    "-sNO_EXIT_RUNTIME=1"
    "-fwasm-exceptions"
    # Emscripten defaults to 64 KiB; 1 MiB matches the smallest desktop main thread (Windows). The cookie check
    # turns an overflow into an abort instead of silent corruption of static data.
    "-sSTACK_SIZE=1MB"
    "-sSTACK_OVERFLOW_CHECK=1")
set_property(TARGET materia_web APPEND PROPERTY LINK_DEPENDS "${MATERIA_WEB_EXPORTS_FILE}")
# Files the kits open by path, embedded into the in-memory filesystem where desktop builds find them relative to the
# working directory (the filesystem root here): AnimKit's default worker model.
foreach(asset animkit/assets/quaternius/worker.glb)
    target_link_options(materia_web PRIVATE "--embed-file=${MATERIA_SOURCE_ROOT}/${asset}@/${asset}")
    set_property(TARGET materia_web APPEND PROPERTY LINK_DEPENDS "${MATERIA_SOURCE_ROOT}/${asset}")
endforeach()
