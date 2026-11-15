# Patch only the GNU/Clang archive command; MSVC and macOS keep their tools.
set(build_utils "${ARROW_SOURCE_DIR}/cpp/cmake_modules/BuildUtils.cmake")
file(READ "${build_utils}" content)
set(original "    set(BUNDLE_COMMAND \${ar_tool} -M < \${ar_script_path})")
set(replacement "    set(BUNDLE_COMMAND \${CMAKE_COMMAND}
        \"-DAR_TOOL=\${ar_tool}\"
        \"-DAR_SCRIPT=\${ar_script_path}\"
        \"-DAR_WORKING_DIRECTORY=\${CMAKE_BINARY_DIR}\"
        -P \"${CMAKE_CURRENT_LIST_DIR}/merge_arrow_archives.cmake\")")
string(FIND "${content}" "${original}" original_position)
string(FIND "${content}" "${replacement}" patched_position)
if(NOT original_position EQUAL -1)
    string(REPLACE "${original}" "${replacement}" content "${content}")
    file(WRITE "${build_utils}" "${content}")
elseif(patched_position EQUAL -1)
    message(FATAL_ERROR "Unsupported Arrow archive command in ${build_utils}")
endif()
