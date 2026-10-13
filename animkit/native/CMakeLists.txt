# Compatibility entry point for Materia's aggregate native builds.
cmake_minimum_required(VERSION 3.24)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../haxeon/packages/animation/native"
                 "${CMAKE_CURRENT_BINARY_DIR}/haxeon-animation")
