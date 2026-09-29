# ONNX Runtime (MIT, Microsoft) from a pinned prebuilt release, as the imported
# target onnxruntime::onnxruntime. It is not built from source.
#
# Offline or pre-provisioned builds set ROBOTKIT_ONNXRUNTIME_DIR (a CMake
# variable or an environment variable) to an unpacked release directory that
# holds include/ and lib/. Otherwise the release for this platform is
# downloaded once into the build tree and checked against the hash below.
#
# Only linux-x64 has been built and run so far; the other archives are pinned
# from the release's published digests.
include_guard(GLOBAL)
include(FetchContent)

set(RK_ONNXRUNTIME_VERSION 1.30.0)
set(_rk_ort_base https://github.com/microsoft/onnxruntime/releases/download/v${RK_ONNXRUNTIME_VERSION})

if(NOT ROBOTKIT_ONNXRUNTIME_DIR AND DEFINED ENV{ROBOTKIT_ONNXRUNTIME_DIR})
    set(ROBOTKIT_ONNXRUNTIME_DIR "$ENV{ROBOTKIT_ONNXRUNTIME_DIR}")
endif()

if(NOT ROBOTKIT_ONNXRUNTIME_DIR)
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _rk_arch)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND _rk_arch MATCHES "^(x86_64|amd64)$")
        set(_rk_ort_archive onnxruntime-linux-x64-${RK_ONNXRUNTIME_VERSION}.tgz)
        set(_rk_ort_sha256 a5ed5a3cac51fbb2e90da632ae43d19212faaa20e76484e62bcb7c23ddb3b3fd)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND _rk_arch MATCHES "^(aarch64|arm64)$")
        set(_rk_ort_archive onnxruntime-linux-aarch64-${RK_ONNXRUNTIME_VERSION}.tgz)
        set(_rk_ort_sha256 e16a27a8ed330bbc698df7330b0cf56e722f354e3bcc92118682c74ef3c3e3da)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin" AND _rk_arch MATCHES "^(arm64|aarch64)$")
        set(_rk_ort_archive onnxruntime-osx-arm64-${RK_ONNXRUNTIME_VERSION}.tgz)
        set(_rk_ort_sha256 6ebb5062a934537c352937821f9fe9718e7de1a2db1122a93dd363ffd53a7012)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows" AND _rk_arch MATCHES "^(amd64|x86_64)$")
        set(_rk_ort_archive onnxruntime-win-x64-${RK_ONNXRUNTIME_VERSION}.zip)
        set(_rk_ort_sha256 c6ba983baf5681af108599675d2a89c2d145512d02de28aed0bff177cd0ba949)
    else()
        message(FATAL_ERROR "No pinned ONNX Runtime release for ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}; "
            "set ROBOTKIT_ONNXRUNTIME_DIR to an unpacked release.")
    endif()
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    FetchContent_Declare(onnxruntime_release
        URL ${_rk_ort_base}/${_rk_ort_archive}
        URL_HASH SHA256=${_rk_ort_sha256})
    FetchContent_GetProperties(onnxruntime_release)
    if(NOT onnxruntime_release_POPULATED)
        cmake_policy(PUSH)
        if(POLICY CMP0169)
            cmake_policy(SET CMP0169 OLD)
        endif()
        FetchContent_Populate(onnxruntime_release)
        cmake_policy(POP)
    endif()
    set(ROBOTKIT_ONNXRUNTIME_DIR "${onnxruntime_release_SOURCE_DIR}")
endif()

if(NOT EXISTS "${ROBOTKIT_ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
    message(FATAL_ERROR "ROBOTKIT_ONNXRUNTIME_DIR=${ROBOTKIT_ONNXRUNTIME_DIR} is not an ONNX Runtime release")
endif()

add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)
if(WIN32)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
        IMPORTED_LOCATION "${ROBOTKIT_ONNXRUNTIME_DIR}/lib/onnxruntime.dll"
        IMPORTED_IMPLIB "${ROBOTKIT_ONNXRUNTIME_DIR}/lib/onnxruntime.lib")
    file(GLOB RK_ONNXRUNTIME_RUNTIME_FILES "${ROBOTKIT_ONNXRUNTIME_DIR}/lib/*.dll")
elseif(APPLE)
    file(GLOB RK_ONNXRUNTIME_RUNTIME_FILES "${ROBOTKIT_ONNXRUNTIME_DIR}/lib/libonnxruntime*.dylib")
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
        IMPORTED_LOCATION "${ROBOTKIT_ONNXRUNTIME_DIR}/lib/libonnxruntime.${RK_ONNXRUNTIME_VERSION}.dylib")
else()
    file(GLOB RK_ONNXRUNTIME_RUNTIME_FILES "${ROBOTKIT_ONNXRUNTIME_DIR}/lib/libonnxruntime*.so*")
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
        IMPORTED_LOCATION "${ROBOTKIT_ONNXRUNTIME_DIR}/lib/libonnxruntime.so.${RK_ONNXRUNTIME_VERSION}"
        IMPORTED_SONAME libonnxruntime.so.1)
endif()
set_target_properties(onnxruntime::onnxruntime PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${ROBOTKIT_ONNXRUNTIME_DIR}/include")
