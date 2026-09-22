if(NK_SIM_MUJOCO_USE_VENDORED)
    if(NOT EXISTS "${NK_MUJOCO_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "Vendored MuJoCo is missing at '${NK_MUJOCO_SOURCE_DIR}'; "
            "run git submodule update --init vendor/mujoco or set "
            "NK_SIM_MUJOCO_USE_VENDORED=OFF")
    endif()

    # NativeKit consumes only the core MuJoCo target. Keep the optional MuJoCo
    # applications and test suites out of the NativeKit build by default.
    set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL
        "Build samples for MuJoCo")
    set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL
        "Build simulate library for MuJoCo")
    set(MUJOCO_BUILD_STUDIO OFF CACHE BOOL
        "Build studio library for MuJoCo")
    set(MUJOCO_BUILD_TESTS OFF CACHE BOOL
        "Build tests for MuJoCo")
    set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL
        "Build and test utility libraries for Python bindings")
    set(MUJOCO_WITH_USD OFF CACHE BOOL
        "Build with OpenUSD")
    set(MUJOCO_USE_FILAMENT OFF CACHE BOOL
        "Use filament rendering")

    add_subdirectory("${NK_MUJOCO_SOURCE_DIR}"
        "${CMAKE_BINARY_DIR}/vendor/mujoco")
    set(NK_MUJOCO_TARGET mujoco::mujoco)
else()
    find_package(mujoco CONFIG REQUIRED)

    set(NK_MUJOCO_TARGET "mujoco::mujoco" CACHE STRING
        "MuJoCo CMake target consumed by NativeKit::sim_mujoco")
endif()

if(NOT TARGET ${NK_MUJOCO_TARGET})
    message(FATAL_ERROR
        "MuJoCo was found, but target '${NK_MUJOCO_TARGET}' is not available")
endif()

add_library(nativekit_sim_mujoco ${NK_LIBRARY_TYPE}
    src/mujoco_backend.cpp)
add_library(NativeKit::sim_mujoco ALIAS nativekit_sim_mujoco)

target_compile_features(nativekit_sim_mujoco PRIVATE cxx_std_20)
target_compile_definitions(nativekit_sim_mujoco PRIVATE NKSIMMUJOCO_BUILDING_LIBRARY)
target_include_directories(nativekit_sim_mujoco
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../sim_core/src)
target_link_libraries(nativekit_sim_mujoco
    PUBLIC NativeKit::sim_core
    PRIVATE ${NK_MUJOCO_TARGET})
if(UNIX AND NOT APPLE)
    # Some Linux toolchains enable --as-needed by default. Keep MuJoCo's
    # DT_NEEDED entry even when the plugin is linked as a shared module, and
    # fail the build instead of shipping a plugin with unresolved mj_* APIs.
    target_link_options(nativekit_sim_mujoco PRIVATE
        "LINKER:--no-as-needed"
        "LINKER:-z,defs")
endif()
set_target_properties(nativekit_sim_mujoco PROPERTIES
    EXPORT_NAME sim_mujoco
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(NK_BUILD_TESTS)
    add_executable(nativekit_sim_mujoco_tests tests/mujoco.cpp)
    target_compile_features(nativekit_sim_mujoco_tests PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sim_mujoco_tests PRIVATE NativeKit::sim_mujoco)
    add_test(NAME sim_mujoco COMMAND nativekit_sim_mujoco_tests)
    set_tests_properties(sim_mujoco PROPERTIES LABELS "sim;mujoco" TIMEOUT 30)
endif()

if(NK_BUILD_EXAMPLES)
    add_executable(nativekit_sim_mujoco_falling_hinge examples/falling_hinge.cpp)
    target_compile_features(nativekit_sim_mujoco_falling_hinge PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sim_mujoco_falling_hinge PRIVATE NativeKit::sim_mujoco)

    add_executable(nativekit_sim_mujoco_stacked_boxes examples/stacked_boxes.cpp)
    target_compile_features(nativekit_sim_mujoco_stacked_boxes PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sim_mujoco_stacked_boxes PRIVATE NativeKit::sim_mujoco)
endif()

if(NK_BUILD_TESTS)
    add_executable(nativekit_sim_mujoco_benchmark bench/benchmark.cpp)
    target_compile_features(nativekit_sim_mujoco_benchmark PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sim_mujoco_benchmark PRIVATE NativeKit::sim_mujoco)
endif()

install(TARGETS nativekit_sim_mujoco EXPORT NativeKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
