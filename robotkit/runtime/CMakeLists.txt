add_library(robotkit_runtime ${RK_LIBRARY_TYPE}
    src/validation.cpp
    src/runtime.cpp
    src/runtime_c_api.cpp
    src/recording.cpp
    src/device_host_v5.cpp
    src/serial_protocol.cpp
    src/serial_endpoint.cpp)
add_library(RobotKit::runtime ALIAS robotkit_runtime)

target_compile_features(robotkit_runtime PUBLIC cxx_std_20)
target_link_libraries(robotkit_runtime PRIVATE robotkit_mcap_headers)
target_compile_definitions(robotkit_runtime PRIVATE RK_BUILDING_LIBRARY)
if(NOT RK_BUILD_SHARED OR EMSCRIPTEN)
    target_compile_definitions(robotkit_runtime PUBLIC RK_STATIC)
endif()
target_include_directories(robotkit_runtime
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/generated>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(robotkit_runtime PROPERTIES
    EXPORT_NAME runtime
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(RK_BUILD_TESTS)
    find_package(Python3 3.10 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME robotkit_device_wire_codegen
            COMMAND ${Python3_EXECUTABLE} -B -m tools.wire check --config robotkit/wire.json)
        set_tests_properties(robotkit_device_wire_codegen PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
            LABELS "robot;wire;codegen" TIMEOUT 30)
    endif()
    find_program(RK_CARGO_EXECUTABLE cargo)
    if(RK_CARGO_EXECUTABLE)
        add_test(NAME robotkit_device_protocol_rust
            COMMAND ${RK_CARGO_EXECUTABLE} test
                --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/Cargo.toml
                --target-dir ${CMAKE_CURRENT_BINARY_DIR}/cargo-target)
        set_tests_properties(robotkit_device_protocol_rust PROPERTIES
            LABELS "robot;wire;rust" TIMEOUT 120)
    endif()

    add_executable(robotkit_device_wire_tests tests/device_wire.cpp)
    target_compile_features(robotkit_device_wire_tests PRIVATE cxx_std_20)
    target_include_directories(robotkit_device_wire_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/generated)
    add_test(NAME robotkit_device_wire COMMAND robotkit_device_wire_tests
        ${CMAKE_CURRENT_SOURCE_DIR}/../schema/device_wire_vectors.tsv)
    set_tests_properties(robotkit_device_wire PROPERTIES LABELS "robot;wire;codegen" TIMEOUT 30)

    add_executable(robotkit_device_protocol_v5_tests tests/device_protocol_v5.cpp)
    target_compile_features(robotkit_device_protocol_v5_tests PRIVATE cxx_std_20)
    target_include_directories(robotkit_device_protocol_v5_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/generated)
    add_test(NAME robotkit_device_protocol_v5 COMMAND robotkit_device_protocol_v5_tests
        ${CMAKE_CURRENT_SOURCE_DIR}/../schema/device_frame_vectors.tsv)
    set_tests_properties(robotkit_device_protocol_v5 PROPERTIES LABELS "robot;wire;hardware" TIMEOUT 30)

    add_executable(robotkit_validation_tests tests/validation.cpp)
    target_compile_features(robotkit_validation_tests PRIVATE cxx_std_20)
    target_link_libraries(robotkit_validation_tests PRIVATE RobotKit::runtime)
    add_test(NAME robotkit_validation COMMAND robotkit_validation_tests)
    set_tests_properties(robotkit_validation PROPERTIES LABELS "robot;abi" TIMEOUT 30)

    add_executable(robotkit_runtime_tests tests/runtime.cpp)
    target_compile_features(robotkit_runtime_tests PRIVATE cxx_std_20)
    target_link_libraries(robotkit_runtime_tests PRIVATE RobotKit::runtime)
    add_test(NAME robotkit_runtime COMMAND robotkit_runtime_tests)
    set_tests_properties(robotkit_runtime PROPERTIES LABELS "robot;runtime" TIMEOUT 30)

    add_executable(robotkit_runtime_c_api_tests tests/c_api.c)
    set_target_properties(robotkit_runtime_c_api_tests PROPERTIES LINKER_LANGUAGE CXX)
    target_include_directories(robotkit_runtime_c_api_tests PRIVATE include)
    target_link_libraries(robotkit_runtime_c_api_tests PRIVATE RobotKit::runtime)
    add_test(NAME robotkit_runtime_c_api COMMAND robotkit_runtime_c_api_tests)
    set_tests_properties(robotkit_runtime_c_api PROPERTIES LABELS "robot;runtime;abi" TIMEOUT 30)

    add_executable(robotkit_recording_tests tests/recording.cpp)
    target_compile_features(robotkit_recording_tests PRIVATE cxx_std_20)
    target_link_libraries(robotkit_recording_tests PRIVATE RobotKit::runtime)
    add_test(NAME robotkit_recording COMMAND robotkit_recording_tests)
    set_tests_properties(robotkit_recording PROPERTIES LABELS "robot;recording;mcap" TIMEOUT 30)

    if(NOT WIN32)
        if(RK_CARGO_EXECUTABLE)
            add_custom_command(
                OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/cargo-target/debug/pty_device
                COMMAND ${RK_CARGO_EXECUTABLE} build
                    --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/Cargo.toml
                    --target-dir ${CMAKE_CURRENT_BINARY_DIR}/cargo-target
                    --bin pty_device
                DEPENDS
                    ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/Cargo.toml
                    ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/src/bin/pty_device.rs
                    ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/src/lib.rs
                    ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/src/runtime.rs
                    ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/src/device_wire.rs
                VERBATIM)
            add_custom_target(robotkit_pty_device DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/cargo-target/debug/pty_device)
            add_executable(robotkit_device_host_v5_pty_tests tests/device_host_v5_pty.cpp)
            target_compile_features(robotkit_device_host_v5_pty_tests PRIVATE cxx_std_20)
            target_link_libraries(robotkit_device_host_v5_pty_tests PRIVATE RobotKit::runtime)
            add_dependencies(robotkit_device_host_v5_pty_tests robotkit_pty_device)
            add_test(NAME robotkit_device_host_v5_pty COMMAND robotkit_device_host_v5_pty_tests
                ${CMAKE_CURRENT_BINARY_DIR}/cargo-target/debug/pty_device)
            set_tests_properties(robotkit_device_host_v5_pty PROPERTIES LABELS "robot;wire;hardware" TIMEOUT 30)
        endif()

        add_executable(robotkit_serial_protocol_tests tests/serial_protocol.cpp)
        target_compile_features(robotkit_serial_protocol_tests PRIVATE cxx_std_20)
        target_link_libraries(robotkit_serial_protocol_tests PRIVATE RobotKit::runtime)
        add_test(NAME robotkit_serial_protocol COMMAND robotkit_serial_protocol_tests)
        set_tests_properties(robotkit_serial_protocol PROPERTIES LABELS "robot;runtime;hardware"
            TIMEOUT 30)

        add_executable(robotkit_serial_endpoint_tests tests/serial_endpoint.cpp)
        target_compile_features(robotkit_serial_endpoint_tests PRIVATE cxx_std_20)
        target_link_libraries(robotkit_serial_endpoint_tests PRIVATE RobotKit::runtime)
        add_test(NAME robotkit_serial_endpoint COMMAND robotkit_serial_endpoint_tests)
        set_tests_properties(robotkit_serial_endpoint PROPERTIES LABELS "robot;runtime;hardware"
            TIMEOUT 30)
    endif()
endif()

install(TARGETS robotkit_runtime EXPORT RobotKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES generated/device_wire.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
