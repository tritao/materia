add_library(robotkit_runtime ${RK_LIBRARY_TYPE}
    src/validation.cpp
    src/runtime.cpp
    src/runtime_c_api.cpp)
add_library(RobotKit::runtime ALIAS robotkit_runtime)

target_compile_features(robotkit_runtime PUBLIC cxx_std_20)
target_compile_definitions(robotkit_runtime PRIVATE RK_BUILDING_LIBRARY)
if(NOT RK_BUILD_SHARED OR EMSCRIPTEN)
    target_compile_definitions(robotkit_runtime PUBLIC RK_STATIC)
endif()
target_include_directories(robotkit_runtime
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(robotkit_runtime PROPERTIES
    EXPORT_NAME runtime
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(RK_BUILD_TESTS)
    add_executable(robotkit_validation_tests tests/validation.cpp)
    target_compile_features(robotkit_validation_tests PRIVATE cxx_std_20)
    target_link_libraries(robotkit_validation_tests PRIVATE RobotKit::runtime)
    add_test(NAME robotkit_validation COMMAND robotkit_validation_tests)
    set_tests_properties(robotkit_validation PROPERTIES LABELS "robot;abi" TIMEOUT 30)

    add_executable(robotkit_runtime_tests tests/runtime.cpp)
    target_compile_features(robotkit_runtime_tests PRIVATE cxx_std_20)
    target_link_libraries(robotkit_runtime_tests PRIVATE RobotKit::runtime)
    add_test(NAME robotkit_runtime COMMAND robotkit_runtime_tests)
    set_tests_properties(robotkit_runtime PROPERTIES LABELS "robot;runtime" TIMEOUT 30)

    add_executable(robotkit_runtime_c_api_tests tests/c_api.c)
    set_target_properties(robotkit_runtime_c_api_tests PROPERTIES LINKER_LANGUAGE CXX)
    target_include_directories(robotkit_runtime_c_api_tests PRIVATE include)
    target_link_libraries(robotkit_runtime_c_api_tests PRIVATE RobotKit::runtime)
    add_test(NAME robotkit_runtime_c_api COMMAND robotkit_runtime_c_api_tests)
    set_tests_properties(robotkit_runtime_c_api PROPERTIES LABELS "robot;runtime;abi" TIMEOUT 30)
endif()

install(TARGETS robotkit_runtime EXPORT RobotKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
