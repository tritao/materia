add_library(nativekit_sensor_stream ${NK_LIBRARY_TYPE}
    src/sensor_stream.cpp)
add_library(NativeKit::sensor_stream ALIAS nativekit_sensor_stream)

target_compile_features(nativekit_sensor_stream PRIVATE cxx_std_20)
target_compile_definitions(nativekit_sensor_stream PRIVATE NKSENSOR_STREAM_BUILDING_LIBRARY)
target_include_directories(nativekit_sensor_stream
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nativekit_sensor_stream PUBLIC NativeKit::sensor_wire)
set_target_properties(nativekit_sensor_stream PROPERTIES
    EXPORT_NAME sensor_stream
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(NK_BUILD_TESTS)
    add_executable(nativekit_sensor_stream_tests tests/sensor_stream.cpp)
    target_compile_features(nativekit_sensor_stream_tests PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sensor_stream_tests PRIVATE NativeKit::sensor_stream)
    add_test(NAME sensor_stream COMMAND nativekit_sensor_stream_tests)
    set_tests_properties(sensor_stream PROPERTIES LABELS "sensor;stream" TIMEOUT 30)
endif()

install(TARGETS nativekit_sensor_stream EXPORT NativeKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
