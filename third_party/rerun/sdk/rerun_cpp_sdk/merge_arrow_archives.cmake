# Arrow's bundled libraries are inside its build tree. Feed ar relative paths
# from that root so its MRI parser never sees the Chinese/space-containing prefix.
file(READ "${AR_SCRIPT}" script)
string(REPLACE "${AR_WORKING_DIRECTORY}/" "" script "${script}")
file(WRITE "${AR_SCRIPT}.relative" "${script}")
execute_process(
    COMMAND "${AR_TOOL}" -M
    INPUT_FILE "${AR_SCRIPT}.relative"
    WORKING_DIRECTORY "${AR_WORKING_DIRECTORY}"
    RESULT_VARIABLE status
)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Arrow archive merge failed: ${status}")
endif()
