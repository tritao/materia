add_library(nativekit_sensor_render ${NK_LIBRARY_TYPE} src/sensor_render.cpp)
add_library(NativeKit::sensor_render ALIAS nativekit_sensor_render)

target_compile_features(nativekit_sensor_render PRIVATE cxx_std_20)
target_compile_definitions(nativekit_sensor_render PRIVATE NKSENSOR_RENDER_BUILDING_LIBRARY)
target_include_directories(nativekit_sensor_render
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nativekit_sensor_render
    PUBLIC NativeKit::sensor_core NativeKit::scene_render)
set_target_properties(nativekit_sensor_render PROPERTIES
    EXPORT_NAME sensor_render
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(NK_BUILD_TESTS AND UNIX AND NOT APPLE AND NOT ANDROID AND NOT EMSCRIPTEN)
    find_program(NKSENSOR_RENDER_XVFB_RUN xvfb-run)
    if(NKSENSOR_RENDER_XVFB_RUN)
        add_executable(nativekit_sensor_render_tests tests/sensor_render.cpp)
        target_compile_features(nativekit_sensor_render_tests PRIVATE cxx_std_20)
        target_include_directories(nativekit_sensor_render_tests PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/../../scenekit/scene/src")
        target_link_libraries(nativekit_sensor_render_tests PRIVATE NativeKit::sensor_render
            NativeKit::scene NativeKit::gpu NativeKit::nativekit)
        add_test(NAME sensor_render
            COMMAND ${NKSENSOR_RENDER_XVFB_RUN} -a $<TARGET_FILE:nativekit_sensor_render_tests>)
        set_tests_properties(sensor_render PROPERTIES LABELS "sensor;render;graphics;gpu"
            TIMEOUT 30 ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1")
    endif()
endif()

install(TARGETS nativekit_sensor_render EXPORT NativeKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
