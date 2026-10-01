# Tests use assert for setup calls as well as checks; keep them active in Release.
if(MSVC)
    add_compile_options(/UNDEBUG)
else()
    add_compile_options(-UNDEBUG)
endif()

add_library(cadkit-c-header-compile OBJECT
    c_header_smoke.c
)

target_include_directories(cadkit-c-header-compile
    PRIVATE
        ${PROJECT_SOURCE_DIR}/core/include
)

add_executable(cadkit-core-smoke
    core_smoke.cpp
)

target_link_libraries(cadkit-core-smoke
    PRIVATE
        cadkit::core
)

add_test(NAME cadkit-core-smoke COMMAND cadkit-core-smoke)

add_executable(cadkit-modeling-smoke modeling_smoke.cpp)
target_link_libraries(cadkit-modeling-smoke PRIVATE cadkit::core)
add_test(NAME cadkit-modeling-smoke COMMAND cadkit-modeling-smoke)

add_executable(cadkit-naming-smoke naming_smoke.cpp)
target_link_libraries(cadkit-naming-smoke PRIVATE cadkit::core)
add_test(NAME cadkit-naming-smoke COMMAND cadkit-naming-smoke)

# Not a test: `cadkit-naming-benchmark` times naming against OCCT (TN2's budget).
add_executable(cadkit-naming-benchmark ${PROJECT_SOURCE_DIR}/benchmarks/naming_benchmark.cpp ${PROJECT_SOURCE_DIR}/core/src/naming.cpp)
target_compile_features(cadkit-naming-benchmark PRIVATE cxx_std_17)
target_include_directories(cadkit-naming-benchmark PRIVATE ${CADKIT_OCCT_INCLUDE_DIR})
target_link_libraries(cadkit-naming-benchmark PRIVATE TKBO TKFillet TKPrim TKTopAlgo TKBRep)
