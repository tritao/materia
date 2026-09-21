add_library(nativekit_sim_core ${NK_LIBRARY_TYPE}
    src/world.cpp
    src/clock.cpp
    src/body.cpp
    src/joint.cpp
    src/shape.cpp
    src/snapshot.cpp
    src/scene_sync.cpp
    src/physics_backend.cpp
    src/host.cpp)
add_library(NativeKit::sim_core ALIAS nativekit_sim_core)

target_compile_features(nativekit_sim_core PRIVATE cxx_std_20)
target_compile_definitions(nativekit_sim_core PRIVATE NKSIM_BUILDING_LIBRARY)
if(NK_ENABLE_NO_EXCEPTIONS)
    target_compile_definitions(nativekit_sim_core PRIVATE NK_ENABLE_NO_EXCEPTIONS=1)
    if(MSVC)
        target_compile_options(nativekit_sim_core PRIVATE /EHs-c-)
    else()
        target_compile_options(nativekit_sim_core PRIVATE -fno-exceptions)
    endif()
endif()
target_include_directories(nativekit_sim_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${NKSIM_SCENEKIT_DIR}/scene/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE src)
target_link_libraries(nativekit_sim_core PUBLIC NativeKit::scene)
find_package(Threads REQUIRED)
target_link_libraries(nativekit_sim_core PRIVATE Threads::Threads)
set_target_properties(nativekit_sim_core PROPERTIES
    EXPORT_NAME sim_core
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(NK_BUILD_EXAMPLES)
    add_executable(nativekit_sim_core_falling_body examples/falling_body.cpp)
    target_compile_features(nativekit_sim_core_falling_body PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sim_core_falling_body PRIVATE NativeKit::sim_core)
endif()

if(NK_BUILD_TESTS)
    add_executable(nativekit_sim_core_benchmark bench/benchmark.cpp)
    target_compile_features(nativekit_sim_core_benchmark PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sim_core_benchmark PRIVATE NativeKit::sim_core)
endif()

if(NK_BUILD_TESTS)
    add_executable(nativekit_sim_core_tests tests/sim_core.cpp)
    target_compile_features(nativekit_sim_core_tests PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sim_core_tests PRIVATE NativeKit::sim_core)
    add_test(NAME sim_core COMMAND nativekit_sim_core_tests)
    set_tests_properties(sim_core PROPERTIES LABELS "sim" TIMEOUT 30)

    add_executable(nativekit_sim_core_c_api_tests tests/c_api.c)
    set_target_properties(nativekit_sim_core_c_api_tests PROPERTIES LINKER_LANGUAGE CXX)
    target_include_directories(nativekit_sim_core_c_api_tests PRIVATE include)
    target_link_libraries(nativekit_sim_core_c_api_tests PRIVATE NativeKit::sim_core)
    add_test(NAME sim_core_c_api COMMAND nativekit_sim_core_c_api_tests)
    set_tests_properties(sim_core_c_api PROPERTIES LABELS "sim;abi" TIMEOUT 30)

    add_executable(nativekit_sim_core_host_tests tests/host.cpp)
    target_compile_features(nativekit_sim_core_host_tests PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sim_core_host_tests PRIVATE NativeKit::sim_core)
    add_test(NAME sim_core_host COMMAND nativekit_sim_core_host_tests)
    set_tests_properties(sim_core_host PROPERTIES LABELS "sim;threading" TIMEOUT 30)
endif()

install(TARGETS nativekit_sim_core EXPORT NativeKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
