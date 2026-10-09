cmake_minimum_required(VERSION 3.21)
project(collisionkit_core VERSION 0.1.0 LANGUAGES C CXX)

# Tests build only when CollisionKit core is the top-level project.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    include(CTest)
endif()
option(CK_BUILD_SHARED "Build CollisionKit core as a shared library" ON)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CK_BUILD_SHARED)
    set(CK_LIBRARY_TYPE SHARED)
else()
    set(CK_LIBRARY_TYPE STATIC)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# coal (collision and distance; see THIRD_PARTY.md) builds as a static core
# with Eigen only, linked privately: nothing in the C ABI names it.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/coal/materia/CMakeLists.txt")
    message(FATAL_ERROR "Initialize collisionkit/native/vendor/coal submodule")
endif()
add_subdirectory(vendor/coal/materia coal EXCLUDE_FROM_ALL)

# V-HACD 4 (convex decomposition; see THIRD_PARTY.md) is a single header,
# compiled once in src/decompose.cpp.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/v-hacd/include/VHACD.h")
    message(FATAL_ERROR "Initialize collisionkit/native/vendor/v-hacd submodule")
endif()
find_package(Threads REQUIRED)

add_library(collisionkit_core ${CK_LIBRARY_TYPE}
    src/api.cpp
    src/decompose.cpp
    src/world.cpp
)
target_compile_features(collisionkit_core PUBLIC c_std_11 PRIVATE cxx_std_17)
target_include_directories(collisionkit_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(collisionkit_core PRIVATE Eigen3::Eigen coal::core Threads::Threads)
target_include_directories(collisionkit_core SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vendor/v-hacd/include)
set_target_properties(collisionkit_core PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    POSITION_INDEPENDENT_CODE ON)
if(CK_BUILD_SHARED)
    target_compile_definitions(collisionkit_core PRIVATE CK_BUILDING_LIBRARY=1)
else()
    target_compile_definitions(collisionkit_core PUBLIC CK_STATIC=1)
endif()
if(MSVC)
    target_compile_options(collisionkit_core PRIVATE /W4)
else()
    target_compile_options(collisionkit_core PRIVATE -Wall -Wextra)
endif()
add_library(collisionkit::core ALIAS collisionkit_core)

# Standalone C++ tests: coal itself, and the collision world through the C ABI.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND BUILD_TESTING)
    add_executable(ck_coal_smoke tests/cpp/coal_smoke.cpp)
    target_link_libraries(ck_coal_smoke PRIVATE coal::core)
    add_test(NAME ck_coal_smoke COMMAND ck_coal_smoke)
    add_executable(ck_collision_world tests/cpp/collision_world.cpp)
    target_link_libraries(ck_collision_world PRIVATE collisionkit_core)
    add_test(NAME ck_collision_world COMMAND ck_collision_world)
    add_executable(ck_decomposition tests/cpp/decomposition.cpp)
    target_link_libraries(ck_decomposition PRIVATE collisionkit_core)
    add_test(NAME ck_decomposition COMMAND ck_decomposition)
endif()
