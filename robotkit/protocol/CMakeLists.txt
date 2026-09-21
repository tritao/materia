add_library(robotkit_protocol ${RK_LIBRARY_TYPE}
    src/protocol.cpp)
add_library(RobotKit::protocol ALIAS robotkit_protocol)

target_compile_features(robotkit_protocol PUBLIC cxx_std_20)
target_compile_definitions(robotkit_protocol PRIVATE RK_BUILDING_LIBRARY)
target_include_directories(robotkit_protocol
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(robotkit_protocol PROPERTIES
    EXPORT_NAME protocol
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(RK_BUILD_TESTS)
    add_executable(robotkit_protocol_tests tests/protocol.cpp)
    target_compile_features(robotkit_protocol_tests PRIVATE cxx_std_20)
    target_link_libraries(robotkit_protocol_tests PRIVATE RobotKit::protocol)
    add_test(NAME robotkit_protocol COMMAND robotkit_protocol_tests)
    set_tests_properties(robotkit_protocol PROPERTIES LABELS "robot;protocol" TIMEOUT 30)
endif()

install(TARGETS robotkit_protocol EXPORT RobotKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
