add_library(nativekit_sensor_io ${NK_LIBRARY_TYPE}
    src/msgpack.cpp
    src/sensor_wire.cpp
    generated/cpp/sensor_wire_generated.cpp
    src/sensor_stream.cpp)
add_library(NativeKit::sensor_io ALIAS nativekit_sensor_io)

target_compile_features(nativekit_sensor_io PRIVATE cxx_std_20)
target_compile_definitions(nativekit_sensor_io PRIVATE
    NKSENSOR_WIRE_BUILDING_LIBRARY
    NKSENSOR_STREAM_BUILDING_LIBRARY)
target_include_directories(nativekit_sensor_io
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/generated/cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(nativekit_sensor_io PUBLIC NativeKit::sensor_core)
set_target_properties(nativekit_sensor_io PROPERTIES
    EXPORT_NAME sensor_io
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

find_package(Python3 3.10 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_custom_target(sensor_wire_generate
        COMMAND ${Python3_EXECUTABLE} -B -m tools.wire.generate
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
        COMMENT "Generate SensorKit wire artifacts from the canonical schema")
endif()

set(NKSENSOR_HAXEON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../haxeon" CACHE PATH
    "Haxeon source tree used by the SensorKit wire interop check")
if(EXISTS "${NKSENSOR_HAXEON_DIR}/.tools/haxe/haxe" AND
   EXISTS "${NKSENSOR_HAXEON_DIR}/.tools/hashlink/hl")
    add_custom_target(sensor_wire_haxe_vectors
        COMMAND ${CMAKE_COMMAND} -E env "HAXEON_ROOT=${NKSENSOR_HAXEON_DIR}"
            bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/haxe/run_vectors.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
        COMMENT "Check generated Haxe SensorKit records against shared wire vectors")
endif()

if(NK_BUILD_TESTS)
    if(Python3_Interpreter_FOUND)
        add_test(NAME sensor_wire_codegen
            COMMAND ${Python3_EXECUTABLE} -B -m tools.wire.generate --check)
        set_tests_properties(sensor_wire_codegen PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
            LABELS "sensor;wire;codegen" TIMEOUT 30)
        add_test(NAME sensor_wire_schema_tests
            COMMAND ${Python3_EXECUTABLE} -B -m unittest tools.wire.tests.test_wire)
        set_tests_properties(sensor_wire_schema_tests PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
            LABELS "sensor;wire;codegen" TIMEOUT 30)
    endif()
    if(TARGET sensor_wire_haxe_vectors)
        add_test(NAME sensor_wire_haxe
            COMMAND ${CMAKE_COMMAND} -E env "HAXEON_ROOT=${NKSENSOR_HAXEON_DIR}"
                bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/haxe/run_vectors.sh)
        set_tests_properties(sensor_wire_haxe PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
            LABELS "sensor;wire;haxe" TIMEOUT 60)
    endif()

    add_executable(nativekit_sensor_wire_tests tests/sensor_wire.cpp)
    target_compile_features(nativekit_sensor_wire_tests PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sensor_wire_tests PRIVATE NativeKit::sensor_io)
    target_compile_definitions(nativekit_sensor_wire_tests PRIVATE
        NKSENSOR_SENSOR_WIRE_VECTORS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/sensor_wire_vectors.tsv")
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
