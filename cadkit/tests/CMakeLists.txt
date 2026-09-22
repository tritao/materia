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
