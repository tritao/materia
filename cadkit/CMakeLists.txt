cmake_minimum_required(VERSION 3.20)

project(cadkit VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(CADKIT_BUILD_TESTS "Build CadKit tests" ON)
option(CADKIT_BUILD_OCCT "Build the pinned vendored OCCT source" ON)
option(CADKIT_BUILD_SHARED "Build the CadKit core as a shared library" ON)
# Building OCCT takes most of an hour, so a build never starts one by accident. CadKit uses a prebuilt install of the
# pinned source (cadkit/scripts/build-occt-prefix.sh creates it once), found in this order:
#   1. CADKIT_OCCT_DIR (cache variable, or the environment variable of the same name, which lets tools such as haxeon
#      opt in),
#   2. the shared cache next to the checkout: <checkout>/../materia-cache/occt-<pinned revision>-release, where the
#      revision is the last commit that touched third_party/occt (so a changed pin finds nothing and fails below),
# and otherwise configuring stops, unless CADKIT_BUILD_OCCT_FROM_SOURCE (or the environment variable) opts in to
# compiling the vendored OCCT.
option(CADKIT_BUILD_OCCT_FROM_SOURCE "Compile the vendored OCCT instead of using a prebuilt install" OFF)
set(CADKIT_OCCT_DIR "$ENV{CADKIT_OCCT_DIR}" CACHE PATH
    "Prefix of a prebuilt install of the pinned OCCT source, used instead of building it")

if(NOT CADKIT_OCCT_DIR AND CADKIT_BUILD_OCCT AND NOT CADKIT_BUILD_OCCT_FROM_SOURCE AND NOT "$ENV{CADKIT_BUILD_OCCT_FROM_SOURCE}")
    set(_cadkit_occt_cache "$ENV{MATERIA_CACHE_DIR}")
    if(NOT _cadkit_occt_cache)
        get_filename_component(_cadkit_occt_cache "${CMAKE_CURRENT_LIST_DIR}/../../materia-cache" ABSOLUTE)
    endif()
    set(_cadkit_occt_revision "")
    find_package(Git QUIET)
    if(GIT_FOUND)
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" log -1 --abbrev=8 --format=%h -- third_party/occt
            WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
            OUTPUT_VARIABLE _cadkit_occt_revision
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET)
    endif()
    set(_cadkit_occt_candidate "${_cadkit_occt_cache}/occt-${_cadkit_occt_revision}-release")
    if(_cadkit_occt_revision AND EXISTS "${_cadkit_occt_candidate}/lib/cmake/opencascade")
        set(CADKIT_OCCT_DIR "${_cadkit_occt_candidate}" CACHE PATH "" FORCE)
        message(STATUS "CadKit found a prebuilt OCCT at ${_cadkit_occt_candidate}")
    else()
        message(FATAL_ERROR
            "CadKit needs a prebuilt OCCT and none was found.\n"
            "  Expected: ${_cadkit_occt_candidate}\n"
            "  Create it once (about an hour) with: cadkit/scripts/build-occt-prefix.sh\n"
            "  Or point CADKIT_OCCT_DIR at an existing install of the pinned source.\n"
            "  To compile OCCT as part of this build instead, pass -DCADKIT_BUILD_OCCT_FROM_SOURCE=ON.")
    endif()
endif()

if(CADKIT_OCCT_DIR)
    find_package(OpenCASCADE CONFIG REQUIRED PATHS "${CADKIT_OCCT_DIR}" NO_DEFAULT_PATH)
    message(STATUS "CadKit uses prebuilt OCCT ${OpenCASCADE_VERSION} from ${CADKIT_OCCT_DIR}")
    set(CADKIT_OCCT_INCLUDE_DIR "${OpenCASCADE_INCLUDE_DIR}")
    set(CADKIT_BUILD_OCCT OFF)
else()
    set(CADKIT_OCCT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/include/opencascade")
endif()

if(CADKIT_BUILD_OCCT)
    # CadKit core uses OCCT's headless modeling and STEP providers. Keep Draw
    # and the top-level visualization modules disabled; TKDESTEP may pull the
    # OCCT support toolkits required by its implementation.
    set(BUILD_MODULE_ApplicationFramework OFF CACHE BOOL "" FORCE)
    set(BUILD_MODULE_DataExchange OFF CACHE BOOL "" FORCE)
    set(BUILD_MODULE_Draw OFF CACHE BOOL "" FORCE)
    set(BUILD_MODULE_Visualization OFF CACHE BOOL "" FORCE)
    set(BUILD_MODULE_FoundationClasses OFF CACHE BOOL "" FORCE)
    set(BUILD_MODULE_ModelingData OFF CACHE BOOL "" FORCE)
    set(BUILD_MODULE_ModelingAlgorithms OFF CACHE BOOL "" FORCE)
    set(BUILD_ADDITIONAL_TOOLKITS "TKBO;TKDE;TKDESTEP;TKFillet;TKOffset;TKMesh;TKPrim;TKTopAlgo;TKXSBase" CACHE STRING "" FORCE)

    set(BUILD_DOC_Overview OFF CACHE BOOL "" FORCE)
    set(BUILD_DOC_RefMan OFF CACHE BOOL "" FORCE)
    set(BUILD_RESOURCES OFF CACHE BOOL "" FORCE)
    set(INSTALL_TEST_CASES OFF CACHE BOOL "" FORCE)
    set(USE_FREETYPE OFF CACHE BOOL "" FORCE)
    set(USE_OPENGL OFF CACHE BOOL "" FORCE)
    set(USE_XLIB OFF CACHE BOOL "" FORCE)

    add_subdirectory(third_party/occt)

    # OCCT's standalone project generates this version header relative to its
    # own binary directory, while its public targets use the top-level
    # generated include directory when OCCT is embedded as a subdirectory.
    # Mirror that one generated header into the include tree used by OCCT's
    # targets. This keeps the vendored source unmodified.
    set(CADKIT_OCCT_VERSION_HEADER
        "${CMAKE_CURRENT_BINARY_DIR}/third_party/occt/include/opencascade/Standard_Version.hxx")
    if(EXISTS "${CADKIT_OCCT_VERSION_HEADER}")
        file(COPY "${CADKIT_OCCT_VERSION_HEADER}"
             DESTINATION "${CMAKE_BINARY_DIR}/include/opencascade")
    else()
        message(FATAL_ERROR "OCCT did not generate Standard_Version.hxx")
    endif()
endif()

add_subdirectory(core)

if(CADKIT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
