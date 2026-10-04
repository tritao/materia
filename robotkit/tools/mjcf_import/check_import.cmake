# Imports the walker fixture and compares the RobotModel with the checked-in
# expectation, which RobotWorldTests also decodes and checks semantically.
file(REMOVE_RECURSE "${OUTPUT}")
execute_process(COMMAND "${IMPORTER}" "${MODEL}" "${OUTPUT}"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "import failed (${result}): ${error}")
endif()
message(STATUS "${output}")
foreach(note IN ITEMS "1 visual meshes" "1 IMUs" "joint dynamics: armature on 2"
        "2 contact pairs between links; 0 with world geoms")
    string(FIND "${output}" "${note}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "import summary lacks \"${note}\"")
    endif()
endforeach()
if(NOT EXISTS "${OUTPUT}/meshes/torso.stl")
    message(FATAL_ERROR "the torso's visual mesh was not written")
endif()
file(READ "${OUTPUT}/robot.json" actual_json)
file(READ "${EXPECTED}" expected_json)
# Saved model codecs canonicalize formatting; compare the complete JSON values.
string(JSON actual GET "${actual_json}")
string(JSON expected GET "${expected_json}")
if(NOT actual STREQUAL expected)
    message(FATAL_ERROR "${OUTPUT}/robot.json differs semantically from ${EXPECTED}")
endif()
