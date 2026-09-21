if(CADKIT_BUILD_SHARED)
    set(CADKIT_CORE_LIBRARY_TYPE SHARED)
else()
    set(CADKIT_CORE_LIBRARY_TYPE STATIC)
endif()

add_library(cadkit-core ${CADKIT_CORE_LIBRARY_TYPE}
    src/cadkit.cpp
)

if(CADKIT_BUILD_SHARED)
    target_compile_definitions(cadkit-core PRIVATE CADKIT_SHARED=1)
endif()

set_target_properties(cadkit-core PROPERTIES
    OUTPUT_NAME cadkit-core
)

target_compile_features(cadkit-core PUBLIC c_std_11 PRIVATE cxx_std_17)

target_include_directories(cadkit-core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include/opencascade
)

target_link_libraries(cadkit-core
    PRIVATE
        TKBO
        TKDE
        TKDESTEP
        TKFillet
        TKMesh
        TKPrim
        TKTopAlgo
        TKBRep
        TKXSBase
)

if(WIN32 AND CADKIT_BUILD_SHARED)
    set_target_properties(cadkit-core PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

add_library(cadkit::core ALIAS cadkit-core)
