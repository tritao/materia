# Learned-policy inference (ONNX Runtime). Optional: RK_BUILD_POLICY.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/OnnxRuntime.cmake)

add_library(robotkit_policy SHARED src/policy.cpp)
add_library(RobotKit::policy ALIAS robotkit_policy)
target_compile_features(robotkit_policy PRIVATE cxx_std_20)
target_link_libraries(robotkit_policy PRIVATE onnxruntime::onnxruntime)
target_include_directories(robotkit_policy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../runtime/include>)
target_compile_definitions(robotkit_policy PRIVATE RK_BUILDING_LIBRARY)
set_target_properties(robotkit_policy PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE)
if(APPLE)
    set_target_properties(robotkit_policy PROPERTIES BUILD_RPATH "@loader_path")
endif()

# robotd's native build gathers its libraries in one directory that the Haxe
# host loads from, so ONNX Runtime goes there beside robotkit_policy.
if(HAXEON_NATIVE_OUTPUT_DIR)
    add_custom_command(TARGET robotkit_policy POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${HAXEON_NATIVE_OUTPUT_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RK_ONNXRUNTIME_RUNTIME_FILES}
            "${HAXEON_NATIVE_OUTPUT_DIR}"
        COMMENT "Staging ONNX Runtime beside robotkit_policy"
        VERBATIM)
endif()

if(RK_BUILD_TESTS)
    add_executable(robotkit_policy_tests tests/policy.cpp)
    target_compile_features(robotkit_policy_tests PRIVATE cxx_std_20)
    target_link_libraries(robotkit_policy_tests PRIVATE robotkit_policy)
    target_compile_definitions(robotkit_policy_tests PRIVATE
        RK_POLICY_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
        RK_POLICY_G1_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../tools/humanoid/policies/unitree-g1/policy.onnx")
    set_target_properties(robotkit_policy_tests PROPERTIES BUILD_RPATH "$ORIGIN")
    add_custom_command(TARGET robotkit_policy_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RK_ONNXRUNTIME_RUNTIME_FILES}
            "$<TARGET_FILE_DIR:robotkit_policy_tests>"
        VERBATIM)
    add_test(NAME robotkit_policy COMMAND robotkit_policy_tests)
    set_tests_properties(robotkit_policy PROPERTIES LABELS "robot;policy;onnx" TIMEOUT 60)
endif()
