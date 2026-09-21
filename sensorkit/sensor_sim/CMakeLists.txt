add_library(nativekit_sensor_sim ${NK_LIBRARY_TYPE} src/sensor_sim.cpp)
add_library(NativeKit::sensor_sim ALIAS nativekit_sensor_sim)

target_compile_features(nativekit_sensor_sim PRIVATE cxx_std_20)
target_compile_definitions(nativekit_sensor_sim PRIVATE NKSENSOR_SIM_BUILDING_LIBRARY)
target_include_directories(nativekit_sensor_sim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE src)
target_link_libraries(nativekit_sensor_sim
    PUBLIC NativeKit::sensor_core NativeKit::sim_core NativeKit::scene_render)
set_target_properties(nativekit_sensor_sim PROPERTIES
    EXPORT_NAME sensor_sim
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(NK_BUILD_TESTS)
    add_executable(nativekit_sensor_sim_tests tests/sensor_sim.cpp)
    target_compile_features(nativekit_sensor_sim_tests PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sensor_sim_tests PRIVATE NativeKit::sensor_sim)
    add_test(NAME sensor_sim COMMAND nativekit_sensor_sim_tests)
    set_tests_properties(sensor_sim PROPERTIES LABELS "sensor;sim" TIMEOUT 30)
endif()

install(TARGETS nativekit_sensor_sim EXPORT NativeKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
