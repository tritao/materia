# Imports the walker fixture and compares the RobotModel with the checked-in
# expectation, which RobotWorldTests also decodes and checks semantically.
file(REMOVE_RECURSE "${OUTPUT}")
execute_process(COMMAND "${IMPORTER}" "${MODEL}" "${OUTPUT}"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "import failed (${result}): ${error}")
endif()
message(STATUS "${output}")
foreach(note IN ITEMS "1 visual meshes" "1 IMUs" "not yet stored (H2): armature on 2"
        "2 explicit contact pairs")
    string(FIND "${output}" "${note}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "import summary lacks \"${note}\"")
    endif()
endforeach()
if(NOT EXISTS "${OUTPUT}/meshes/torso.stl")
    message(FATAL_ERROR "the torso's visual mesh was not written")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files
    "${OUTPUT}/robot.json" "${EXPECTED}" RESULT_VARIABLE different)
if(NOT different EQUAL 0)
    message(FATAL_ERROR "${OUTPUT}/robot.json differs from ${EXPECTED}")
endif()
