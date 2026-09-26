cmake_minimum_required(VERSION 3.21)

# Thin wrapper around robotd/native's own CMakeLists.txt that forces the
# optional MuJoCo simulation backend on, for haxeon's own native.cmake
# integration (see robotkit/tests/mujoco/haxeon.json).
#
# haxeon's NativeCMakeProvider always configures a package's native.cmake
# project with a fixed, hardcoded argument list (-S <source> -B <buildDir>
# -DHAXEON_NATIVE_OUTPUT_DIR=... -DHAXEON_TARGET=... -DCMAKE_BUILD_TYPE=...)
# and has no mechanism to pass extra -D defines per package. robotd/native's
# own CMakeLists.txt declares `option(NKSIM_BUILD_MUJOCO ... OFF)` so every
# other Haxe consumer of the `robotd_native` target (robotkit/tests,
# robotkit/cadbridge/tests, worldd) keeps MuJoCo's extra build time and
# dependency out of its default build. Since CMake's `option()` only sets a
# cache variable when it is not already defined, pre-seeding
# NKSIM_BUILD_MUJOCO in the cache here — before including the real
# robotd/native project as a subdirectory — flips it on for only this one
# haxeon project's own isolated CMake build directory, without touching
# robotd/native's own default for anyone else.
project(materia_robotd_native_mujoco NONE)

set(NKSIM_BUILD_MUJOCO ON CACHE BOOL "Build the optional MuJoCo simulation backend" FORCE)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../native" native)
