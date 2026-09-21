add_library(nativekit_sensor_core ${NK_LIBRARY_TYPE} src/sensor_core.cpp)
add_library(NativeKit::sensor_core ALIAS nativekit_sensor_core)

target_compile_features(nativekit_sensor_core PRIVATE cxx_std_20)
target_compile_definitions(nativekit_sensor_core PRIVATE NKSENSOR_BUILDING_LIBRARY)
target_include_directories(nativekit_sensor_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE src)
set_target_properties(nativekit_sensor_core PROPERTIES
    EXPORT_NAME sensor_core
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

if(NK_BUILD_TESTS)
    add_executable(nativekit_sensor_core_tests tests/sensor_core.cpp)
    target_compile_features(nativekit_sensor_core_tests PRIVATE cxx_std_20)
    target_link_libraries(nativekit_sensor_core_tests PRIVATE NativeKit::sensor_core)
    add_test(NAME sensor_core COMMAND nativekit_sensor_core_tests)
    set_tests_properties(sensor_core PROPERTIES LABELS "sensor" TIMEOUT 30)
endif()

install(TARGETS nativekit_sensor_core EXPORT NativeKitTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
