# The pendant firmware's core (../pendant_core) as a shared library the editor loads, so the simulated pendant runs the code
# that ships on the pendant. Needs cargo; the browser has no Rust toolchain for it yet.
cmake_minimum_required(VERSION 3.24)
project(machinekit_pendant LANGUAGES C)

find_program(MK_PENDANT_CARGO cargo REQUIRED)
set(MK_PENDANT_RUST_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MK_PENDANT_LIB ${CMAKE_CURRENT_BINARY_DIR}/cargo/release/${CMAKE_STATIC_LIBRARY_PREFIX}pendant_virtual${CMAKE_STATIC_LIBRARY_SUFFIX})
file(GLOB_RECURSE MK_PENDANT_RUST_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.rs
    ${CMAKE_CURRENT_SOURCE_DIR}/../pendant_core/src/*.rs
    ${CMAKE_CURRENT_SOURCE_DIR}/../operator_link/src/*.rs)
add_custom_command(OUTPUT ${MK_PENDANT_LIB}
    COMMAND ${MK_PENDANT_CARGO} build --release --offline
        --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
        --target-dir ${CMAKE_CURRENT_BINARY_DIR}/cargo
    DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
        ${CMAKE_CURRENT_SOURCE_DIR}/../pendant_core/Cargo.toml
        ${CMAKE_CURRENT_SOURCE_DIR}/../operator_link/Cargo.toml
        ${MK_PENDANT_RUST_SOURCES}
    VERBATIM)
add_custom_target(machinekit_pendant_build DEPENDS ${MK_PENDANT_LIB})
add_library(machinekit_pendant_rust STATIC IMPORTED GLOBAL)
set_target_properties(machinekit_pendant_rust PROPERTIES IMPORTED_LOCATION ${MK_PENDANT_LIB})
add_dependencies(machinekit_pendant_rust machinekit_pendant_build)

# The Rust objects go in whole: nothing in the C stub refers to them.
add_library(machinekit_pendant SHARED src/stub.c)
target_include_directories(machinekit_pendant PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(machinekit_pendant PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,machinekit_pendant_rust>")
if(UNIX AND NOT APPLE)
    target_link_libraries(machinekit_pendant PRIVATE dl pthread m)
endif()
