include(${CMAKE_CURRENT_SOURCE_DIR}/../policy/cmake/OnnxRuntime.cmake)
get_property(RK_ONNXRUNTIME_RUNTIME_FILES GLOBAL PROPERTY RK_ONNXRUNTIME_RUNTIME_FILES)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)

add_library(robotkit_inference SHARED src/inference.cpp)
add_library(RobotKit::inference ALIAS robotkit_inference)
target_compile_features(robotkit_inference PRIVATE cxx_std_20)
target_link_libraries(robotkit_inference PRIVATE onnxruntime::onnxruntime OpenSSL::Crypto)
target_include_directories(robotkit_inference PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../runtime/include>)
target_compile_definitions(robotkit_inference PRIVATE RK_BUILDING_LIBRARY)
set_target_properties(robotkit_inference PROPERTIES
    CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES
    BUILD_RPATH "$ORIGIN" INSTALL_RPATH "$ORIGIN" BUILD_WITH_INSTALL_RPATH TRUE)
if(APPLE)
    set_target_properties(robotkit_inference PROPERTIES BUILD_RPATH "@loader_path")
endif()
if(HAXEON_NATIVE_OUTPUT_DIR)
    add_custom_command(TARGET robotkit_inference POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${HAXEON_NATIVE_OUTPUT_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RK_ONNXRUNTIME_RUNTIME_FILES}
            "${HAXEON_NATIVE_OUTPUT_DIR}"
        COMMENT "Staging ONNX Runtime beside robotkit_inference" VERBATIM)
endif()
if(RK_BUILD_TESTS)
    add_executable(robotkit_inference_tests tests/inference.cpp)
    target_compile_features(robotkit_inference_tests PRIVATE cxx_std_20)
    target_link_libraries(robotkit_inference_tests PRIVATE robotkit_inference)
    target_compile_definitions(robotkit_inference_tests PRIVATE
        RK_INFERENCE_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/detector.onnx"
        RK_INFERENCE_FIXTURE_NHWC="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/detector_nhwc.onnx"
        RK_INFERENCE_FIXTURE_LARGE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/detector_large.onnx")
    target_compile_definitions(robotkit_inference_tests PRIVATE
        RK_INFERENCE_FIXTURE_UINT8="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/detector_uint8.onnx"
        RK_INFERENCE_FIXTURE_DYNAMIC="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/detector_dynamic.onnx")
    set_target_properties(robotkit_inference_tests PROPERTIES BUILD_RPATH "$ORIGIN")
    add_custom_command(TARGET robotkit_inference_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RK_ONNXRUNTIME_RUNTIME_FILES}
            "$<TARGET_FILE_DIR:robotkit_inference_tests>" VERBATIM)
    add_test(NAME robotkit_inference COMMAND robotkit_inference_tests)
    set_tests_properties(robotkit_inference PROPERTIES LABELS "robot;inference;onnx" TIMEOUT 60)
endif()
