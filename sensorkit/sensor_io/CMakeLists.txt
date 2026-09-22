add_library(nativekit_sensor_io ${NK_LIBRARY_TYPE}
    src/msgpack.cpp
    src/sensor_wire.cpp
    src/sensor_stream.cpp)
add_library(NativeKit::sensor_io ALIAS nativekit_sensor_io)

target_compile_features(nativekit_sensor_io PRIVATE cxx_std_20)
target_compile_definitions(nativekit_sensor_io PRIVATE
    NKSENSOR_WIRE_BUILDING_LIBRARY
    NKSENSOR_STREAM_BUILDING_LIBRARY)
target_include_directories(nativekit_sensor_io
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nativekit_sensor_io PUBLIC NativeKit::sensor_core)
set_target_properties(nativekit_sensor_io PROPERTIES
    EXPORT_NAME sensor_io
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(NK_BUILD_TESTS)
    add_executable(nativekit_sensor_wire_tests tests/sensor_wire.cpp)
    target_compile_features(nativekit_sensor_wire_tests PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sensor_wire_tests PRIVATE NativeKit::sensor_io)
    add_test(NAME sensor_wire COMMAND nativekit_sensor_wire_tests)
    set_tests_properties(sensor_wire PROPERTIES LABELS "sensor;io;wire" TIMEOUT 30)

    add_executable(nativekit_sensor_stream_tests tests/sensor_stream.cpp)
    target_compile_features(nativekit_sensor_stream_tests PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sensor_stream_tests PRIVATE NativeKit::sensor_io)
    add_test(NAME sensor_stream COMMAND nativekit_sensor_stream_tests)
    set_tests_properties(sensor_stream PROPERTIES LABELS "sensor;io;stream" TIMEOUT 30)
endif()

install(TARGETS nativekit_sensor_io EXPORT NativeKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY haxe/ DESTINATION share/sensorkit/haxe)
