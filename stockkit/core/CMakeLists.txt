cmake_minimum_required(VERSION 3.21)
project(stockkit_core VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
option(SK_BUILD_SHARED "Build StockKit core as a shared library" ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(SK_BUILD_SHARED)
    set(SK_LIBRARY_TYPE SHARED)
else()
    set(SK_LIBRARY_TYPE STATIC)
endif()

add_library(stockkit_core ${SK_LIBRARY_TYPE}
    src/api.cpp
    src/mesh.cpp
    src/pool.cpp
    src/preview.cpp
    src/profile.cpp
    src/stock.cpp
    src/sweep.cpp
    src/tool.cpp
)
target_compile_features(stockkit_core PUBLIC c_std_11 PRIVATE cxx_std_17)
target_include_directories(stockkit_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(Threads REQUIRED)
target_link_libraries(stockkit_core PRIVATE Threads::Threads)
set_target_properties(stockkit_core PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    POSITION_INDEPENDENT_CODE ON)
if(SK_BUILD_SHARED)
    target_compile_definitions(stockkit_core PRIVATE SK_BUILDING_LIBRARY=1)
else()
    target_compile_definitions(stockkit_core PUBLIC SK_STATIC=1)
endif()
if(MSVC)
    target_compile_options(stockkit_core PRIVATE /W4)
else()
    # fma must stay a real fused multiply-add for the exact predicates, and
    # plain operations must not be contracted into one.
    target_compile_options(stockkit_core PRIVATE -Wall -Wextra -ffp-contract=off)
endif()
add_library(stockkit::core ALIAS stockkit_core)

if(BUILD_TESTING)
    add_executable(stockkit_core_tests tests/core_tests.cpp)
    target_link_libraries(stockkit_core_tests PRIVATE stockkit_core)
    target_compile_features(stockkit_core_tests PRIVATE cxx_std_17)
    add_test(NAME stockkit_core_tests COMMAND stockkit_core_tests)

    add_executable(stockkit_core_bench tests/bench.cpp)
    target_link_libraries(stockkit_core_bench PRIVATE stockkit_core)
    target_compile_features(stockkit_core_bench PRIVATE cxx_std_17)
endif()
