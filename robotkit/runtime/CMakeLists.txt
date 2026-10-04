if(RK_BUILD_VIRTUAL_DEVICE)
    find_program(RK_VIRTUAL_CARGO_EXECUTABLE cargo REQUIRED)
    set(RK_VIRTUAL_LIB ${CMAKE_CURRENT_BINARY_DIR}/cargo-virtual/debug/librobotkit_device_virtual.a)
    add_custom_command(OUTPUT ${RK_VIRTUAL_LIB}
        COMMAND ${RK_VIRTUAL_CARGO_EXECUTABLE} build
            --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/Cargo.toml
            --target-dir ${CMAKE_CURRENT_BINARY_DIR}/cargo-virtual
        DEPENDS
            ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/Cargo.toml
            ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/src/lib.rs
            ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/src/welder.rs
            ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/boards/welder_retrofit.rs
            ${CMAKE_CURRENT_SOURCE_DIR}/../../processkit/schema/weld_contract.rs
            ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/src/scheduled_core.rs
            ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/src/virtual_board.rs
        VERBATIM)
    add_custom_target(robotkit_device_virtual_build DEPENDS ${RK_VIRTUAL_LIB})
    add_library(robotkit_device_virtual STATIC IMPORTED GLOBAL)
    set_target_properties(robotkit_device_virtual PROPERTIES IMPORTED_LOCATION ${RK_VIRTUAL_LIB})
    add_dependencies(robotkit_device_virtual robotkit_device_virtual_build)
endif()

add_library(robotkit_runtime ${RK_LIBRARY_TYPE}
    src/validation.cpp
    src/runtime.cpp
    src/runtime_c_api.cpp
    src/recording.cpp
    src/clock_estimator6.cpp
    src/device_compiler6.cpp
    src/rkd6_endpoint.cpp
    src/virtual_device_endpoint.cpp)
add_library(RobotKit::runtime ALIAS robotkit_runtime)
# Large structs belong on the heap: worker threads may have only 512 KiB of stack (macOS) and the browser 1 MiB.
if(NOT MSVC)
    target_compile_options(robotkit_runtime PRIVATE -Wframe-larger-than=65536)
endif()
if(RK_BUILD_SERIAL_DEVICE)
    target_sources(robotkit_runtime PRIVATE src/device_serial_endpoint.cpp)
    target_compile_definitions(robotkit_runtime PRIVATE RK_HAS_SERIAL_DEVICE=1)
endif()
if(RK_BUILD_VIRTUAL_DEVICE)
    target_compile_definitions(robotkit_runtime PRIVATE RK_HAS_VIRTUAL_DEVICE=1)
    target_link_libraries(robotkit_runtime PRIVATE robotkit_device_virtual)
else()
    target_sources(robotkit_runtime PRIVATE src/rkd_virtual_unavailable.cpp)
endif()

target_compile_features(robotkit_runtime PUBLIC cxx_std_20)
target_link_libraries(robotkit_runtime PUBLIC trajectory_core
    PRIVATE robotkit_mcap_headers robotkit_lz4)
target_include_directories(robotkit_runtime PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/include)
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
        add_test(NAME robotkit_device_wire6_codegen
            COMMAND ${Python3_EXECUTABLE} -B -m tools.wire check --config robotkit/wire6.json)
        set_tests_properties(robotkit_device_wire6_codegen PROPERTIES
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


    add_executable(robotkit_device_wire6_tests tests/device_wire6.cpp)
    target_compile_features(robotkit_device_wire6_tests PRIVATE cxx_std_20)
    target_include_directories(robotkit_device_wire6_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/generated)
    add_test(NAME robotkit_device_wire6 COMMAND robotkit_device_wire6_tests
        ${CMAKE_CURRENT_SOURCE_DIR}/../schema/device_frame6_vectors.tsv)
    set_tests_properties(robotkit_device_wire6 PROPERTIES LABELS "robot;wire;codegen" TIMEOUT 30)

    add_executable(robotkit_clock_estimator6_tests tests/clock_estimator6.cpp src/clock_estimator6.cpp)
    target_compile_features(robotkit_clock_estimator6_tests PRIVATE cxx_std_20)
    target_include_directories(robotkit_clock_estimator6_tests PRIVATE src)
    add_test(NAME robotkit_clock_estimator6 COMMAND robotkit_clock_estimator6_tests)
    set_tests_properties(robotkit_clock_estimator6 PROPERTIES LABELS "robot;runtime;clock" TIMEOUT 30)

    add_executable(robotkit_segment_vectors_tests tests/segment_vectors.cpp)
    target_compile_features(robotkit_segment_vectors_tests PRIVATE cxx_std_20)
    target_link_libraries(robotkit_segment_vectors_tests PRIVATE trajectory_core)
    add_test(NAME robotkit_segment_vectors COMMAND robotkit_segment_vectors_tests
        ${CMAKE_CURRENT_SOURCE_DIR}/../device_protocol/tests/segment_vectors.tsv)
    set_tests_properties(robotkit_segment_vectors PROPERTIES LABELS "robot;wire;motion" TIMEOUT 30)

    add_executable(robotkit_device_compiler6_tests tests/device_compiler6.cpp
        src/device_compiler6.cpp src/clock_estimator6.cpp)
    target_compile_features(robotkit_device_compiler6_tests PRIVATE cxx_std_20)
    target_include_directories(robotkit_device_compiler6_tests PRIVATE src include generated)
    target_link_libraries(robotkit_device_compiler6_tests PRIVATE trajectory_core)
    add_test(NAME robotkit_device_compiler6 COMMAND robotkit_device_compiler6_tests)
    set_tests_properties(robotkit_device_compiler6 PROPERTIES LABELS "robot;runtime;wire" TIMEOUT 30)

    add_executable(robotkit_rkd6_endpoint_tests tests/rkd6_endpoint.cpp
        src/rkd6_endpoint.cpp src/device_compiler6.cpp src/clock_estimator6.cpp)
    target_compile_features(robotkit_rkd6_endpoint_tests PRIVATE cxx_std_20)
    target_include_directories(robotkit_rkd6_endpoint_tests PRIVATE src include generated)
    target_link_libraries(robotkit_rkd6_endpoint_tests PRIVATE trajectory_core)
    add_test(NAME robotkit_rkd6_endpoint COMMAND robotkit_rkd6_endpoint_tests)
    set_tests_properties(robotkit_rkd6_endpoint PROPERTIES LABELS "robot;runtime;wire" TIMEOUT 30)

    if(RK_BUILD_VIRTUAL_DEVICE)
        add_executable(robotkit_virtual_device_endpoint_tests tests/virtual_device_endpoint.cpp)
        target_compile_features(robotkit_virtual_device_endpoint_tests PRIVATE cxx_std_20)
        target_include_directories(robotkit_virtual_device_endpoint_tests PRIVATE src)
        target_link_libraries(robotkit_virtual_device_endpoint_tests PRIVATE RobotKit::runtime)
        add_test(NAME robotkit_virtual_device_endpoint COMMAND robotkit_virtual_device_endpoint_tests)
        set_tests_properties(robotkit_virtual_device_endpoint PROPERTIES LABELS "robot;runtime;virtual" TIMEOUT 30)
    endif()


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
    target_link_libraries(robotkit_recording_tests PRIVATE RobotKit::runtime robotkit_mcap_headers robotkit_lz4)
    add_test(NAME robotkit_recording COMMAND robotkit_recording_tests)
    set_tests_properties(robotkit_recording PROPERTIES LABELS "robot;recording;mcap" TIMEOUT 30)

    if(NOT WIN32 AND RK_CARGO_EXECUTABLE AND RK_BUILD_SERIAL_DEVICE)
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/cargo-target/debug/pty_device
            COMMAND ${RK_CARGO_EXECUTABLE} build
                --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/Cargo.toml
                --target-dir ${CMAKE_CURRENT_BINARY_DIR}/cargo-target
                --bin pty_device
            DEPENDS
                ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/Cargo.toml
                ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/src/lib.rs
                ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/src/bin/pty_device.rs
                ${CMAKE_CURRENT_SOURCE_DIR}/../device_virtual/src/bin/shared/mod.rs
            VERBATIM)
        add_custom_target(robotkit_pty_device DEPENDS
            ${CMAKE_CURRENT_BINARY_DIR}/cargo-target/debug/pty_device)
        add_executable(robotkit_rkd6_serial_pty_tests tests/rkd6_serial_pty.cpp)
        target_compile_features(robotkit_rkd6_serial_pty_tests PRIVATE cxx_std_20)
        target_include_directories(robotkit_rkd6_serial_pty_tests PRIVATE src)
        target_link_libraries(robotkit_rkd6_serial_pty_tests PRIVATE RobotKit::runtime)
        add_dependencies(robotkit_rkd6_serial_pty_tests robotkit_pty_device)
        add_test(NAME robotkit_rkd6_serial_pty COMMAND robotkit_rkd6_serial_pty_tests
            ${CMAKE_CURRENT_BINARY_DIR}/cargo-target/debug/pty_device)
        set_tests_properties(robotkit_rkd6_serial_pty PROPERTIES LABELS "robot;wire;hardware" TIMEOUT 30)
    endif()

endif()

install(TARGETS robotkit_runtime EXPORT RobotKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES generated/device_wire6.hpp generated/device_frame6.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# These tests use assert for setup calls as well as checks. Keep them active in Release.
get_property(_assert_test_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(_assert_test_target IN LISTS _assert_test_targets)
    if(_assert_test_target MATCHES "_tests$")
        if(MSVC)
            target_compile_options(${_assert_test_target} PRIVATE /UNDEBUG)
        else()
            target_compile_options(${_assert_test_target} PRIVATE -UNDEBUG)
        endif()
    endif()
endforeach()
