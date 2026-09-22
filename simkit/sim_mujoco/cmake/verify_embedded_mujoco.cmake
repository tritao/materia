if(NOT DEFINED PLUGIN OR NOT EXISTS "${PLUGIN}")
    message(FATAL_ERROR "MuJoCo plugin verification requires an existing PLUGIN")
endif()
if(NOT DEFINED READELF OR NOT EXISTS "${READELF}")
    message(FATAL_ERROR "MuJoCo plugin verification requires CMAKE_READELF")
endif()

execute_process(
    COMMAND "${READELF}" -d "${PLUGIN}"
    RESULT_VARIABLE readelf_result
    OUTPUT_VARIABLE dynamic_section
    ERROR_VARIABLE readelf_error)
if(NOT readelf_result EQUAL 0)
    message(FATAL_ERROR "Could not inspect ${PLUGIN}: ${readelf_error}")
endif()
if(dynamic_section MATCHES "Shared library: \\[libmujoco")
    message(FATAL_ERROR "${PLUGIN} unexpectedly depends on a dynamic MuJoCo runtime")
endif()

message(STATUS "Verified self-contained MuJoCo plugin: ${PLUGIN}")
