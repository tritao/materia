add_library(robotkit_core ${RK_LIBRARY_TYPE}
    src/core.cpp)
add_library(RobotKit::core ALIAS robotkit_core)

target_compile_features(robotkit_core PUBLIC cxx_std_17)
target_compile_definitions(robotkit_core PRIVATE RK_BUILDING_LIBRARY)
target_include_directories(robotkit_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(robotkit_core PROPERTIES
    EXPORT_NAME core
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(RK_BUILD_TESTS)
    add_executable(robotkit_core_tests tests/core.cpp)
    target_compile_features(robotkit_core_tests PRIVATE cxx_std_17)
    target_link_libraries(robotkit_core_tests PRIVATE RobotKit::core)
    add_test(NAME robotkit_core COMMAND robotkit_core_tests)
    set_tests_properties(robotkit_core PROPERTIES LABELS "robot;abi" TIMEOUT 30)
endif()

install(TARGETS robotkit_core EXPORT RobotKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
