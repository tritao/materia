cmake_minimum_required(VERSION 3.20)

project(cadkit VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(CADKIT_BUILD_TESTS "Build CadKit tests" ON)
option(CADKIT_BUILD_OCCT "Build the pinned vendored OCCT source" ON)
option(CADKIT_BUILD_SHARED "Build the CadKit core as a shared library" ON)

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
    set(BUILD_ADDITIONAL_TOOLKITS "TKBO;TKDE;TKDESTEP;TKFillet;TKMesh;TKPrim;TKTopAlgo;TKXSBase" CACHE STRING "" FORCE)

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
             DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/include/opencascade")
    else()
        message(FATAL_ERROR "OCCT did not generate Standard_Version.hxx")
    endif()
endif()

add_subdirectory(core)

if(CADKIT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
