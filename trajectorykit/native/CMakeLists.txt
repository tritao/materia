cmake_minimum_required(VERSION 3.21)
project(trajectory_core VERSION 0.1.0 LANGUAGES C CXX)
include(CTest)
include(GNUInstallDirs)
option(TC_BUILD_SHARED "Build the shared polynomial evaluator and validator" ON)
set(TC_LIBRARY_TYPE STATIC)
if(TC_BUILD_SHARED AND NOT EMSCRIPTEN)
    set(TC_LIBRARY_TYPE SHARED)
endif()
add_library(trajectory_core ${TC_LIBRARY_TYPE} src/trajectory.cpp src/validation.cpp)
add_library(TrajectoryKit::core ALIAS trajectory_core)
target_compile_features(trajectory_core PUBLIC cxx_std_17)
target_include_directories(trajectory_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_definitions(trajectory_core PRIVATE TC_BUILDING_LIBRARY)
if(NOT TC_BUILD_SHARED OR EMSCRIPTEN)
    target_compile_definitions(trajectory_core PUBLIC TC_STATIC)
endif()
set_target_properties(trajectory_core PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES VERSION ${PROJECT_VERSION} SOVERSION 0)
if(BUILD_TESTING)
    find_package(Threads REQUIRED)
    foreach(name trajectory validation)
        add_executable(trajectory_core_${name}_tests ../../motionkit/native/tests/${name}.cpp)
        target_link_libraries(trajectory_core_${name}_tests PRIVATE trajectory_core Threads::Threads)
        # These checks do not use any planner entry points.
        target_include_directories(trajectory_core_${name}_tests PRIVATE ../../motionkit/native/include)
        if(MSVC)
            target_compile_options(trajectory_core_${name}_tests PRIVATE /UNDEBUG)
        else()
            target_compile_options(trajectory_core_${name}_tests PRIVATE -UNDEBUG)
        endif()
        add_test(NAME trajectory_core_${name} COMMAND trajectory_core_${name}_tests)
    endforeach()
endif()
install(TARGETS trajectory_core EXPORT RobotKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(DEFINED HAXEON_NATIVE_OUTPUT_DIR)
    set_target_properties(trajectory_core PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
        RUNTIME_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}"
        ARCHIVE_OUTPUT_DIRECTORY "${HAXEON_NATIVE_OUTPUT_DIR}")
endif()
