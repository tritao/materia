# MuJoCo fetches its third-party libraries from GitHub while it configures, which makes every clean configure depend on the
# network and spend half a minute cloning. They are submodules under simkit/vendor/mujoco-deps instead, and FetchContent is
# pointed at them. Each must be at the commit the MuJoCo checkout declares (MUJOCO_DEP_VERSION_*), so updating MuJoCo's
# pins is only complete once the submodule is moved too, and a configure says so rather than building the wrong source.
#
# The pins are read from the MuJoCo checkout's own CMake files, so this module has no copy of them to keep in step; if that
# layout changes the configure stops with "Could not read MuJoCo's pinned version" rather than skipping the check. Only what
# the backend builds is vendored. MuJoCo's tests and benchmarks need gtest, abseil and benchmark: turning them on fails the
# configure (FetchContent is disconnected below) until those are added as submodules here too.

set(_mujoco_deps_dir "${CMAKE_CURRENT_LIST_DIR}/../vendor/mujoco-deps")
file(READ "${NK_MUJOCO_SOURCE_DIR}/cmake/MujocoDependencies.cmake" _mujoco_pins)
file(READ "${NK_MUJOCO_SOURCE_DIR}/cmake/third_party_deps/lodepng.cmake" _mujoco_lodepng_pin)
string(APPEND _mujoco_pins "\n${_mujoco_lodepng_pin}")
find_package(Git QUIET)

# <submodule directory>|<suffix of MuJoCo's MUJOCO_DEP_VERSION_ variable>
foreach(_entry IN ITEMS
        "ccd|ccd" "lodepng|lodepng" "marchingcubecpp|MarchingCubeCpp" "miniz|miniz"
        "qhull|qhull" "tinyobjloader|tinyobjloader" "tinyxml2|tinyxml2")
    string(REPLACE "|" ";" _entry "${_entry}")
    list(GET _entry 0 _name)
    list(GET _entry 1 _version_name)
    set(_source "${_mujoco_deps_dir}/${_name}")
    if(NOT EXISTS "${_source}/.git")
        message(FATAL_ERROR
            "MuJoCo's ${_name} is missing at '${_source}'; run git submodule update --init simkit/vendor/mujoco-deps/${_name}")
    endif()
    if(NOT _mujoco_pins MATCHES "MUJOCO_DEP_VERSION_${_version_name}[ \t\r\n]+([0-9a-f]+)")
        message(FATAL_ERROR "Could not read MuJoCo's pinned version of ${_name} (MUJOCO_DEP_VERSION_${_version_name})")
    endif()
    set(_pinned "${CMAKE_MATCH_1}")
    if(GIT_FOUND)
        execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${_source}" rev-parse HEAD
            OUTPUT_VARIABLE _actual OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _status ERROR_QUIET)
        if(NOT _status EQUAL 0 OR NOT _actual STREQUAL _pinned)
            message(FATAL_ERROR
                "MuJoCo pins ${_name} at ${_pinned} but '${_source}' is at '${_actual}'; "
                "run git submodule update --init simkit/vendor/mujoco-deps/${_name}, or move the submodule to MuJoCo's pin")
        endif()
    endif()
    string(TOUPPER "${_name}" _upper)
    set(FETCHCONTENT_SOURCE_DIR_${_upper} "${_source}" CACHE PATH "Vendored ${_name}" FORCE)
endforeach()

# Nothing else MuJoCo fetches is needed for the backend; failing here beats silently cloning from the network.
set(FETCHCONTENT_FULLY_DISCONNECTED ON)
