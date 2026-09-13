# Ensure helper-only CMake modules remain syntactically valid on every host.
# In particular, CI runs on macOS and would otherwise never include the MSVC helper.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(HELPER_MODULES
    "cmake/compilers/CompilerClangGCC.cmake"
    "cmake/compilers/CompilerMSVC.cmake"
    "cmake/features/PrecompiledHeaders.cmake"
    "cmake/features/UnityBuild.cmake"
    "cmake/project/codegen/FlatBuffers.cmake"
    "cmake/project/ProjectTargets.cmake")

set(PARSE_FAILURES "")
foreach(helper_relative_path IN LISTS HELPER_MODULES)
    set(helper_path "${SOURCE_DIR}/${helper_relative_path}")
    if(NOT EXISTS "${helper_path}")
        list(APPEND PARSE_FAILURES "missing: ${helper_relative_path}")
        continue()
    endif()

    execute_process(
        COMMAND "${CMAKE_COMMAND}" -P "${helper_path}"
        RESULT_VARIABLE helper_result
        OUTPUT_VARIABLE helper_output
        ERROR_VARIABLE helper_error)
    if(NOT helper_result EQUAL 0)
        string(STRIP "${helper_output}\n${helper_error}" helper_diagnostics)
        list(APPEND PARSE_FAILURES
            "${helper_relative_path}: ${helper_diagnostics}")
    endif()
endforeach()

if(PARSE_FAILURES)
    string(REPLACE ";" "\n  " PARSE_FAILURES_TEXT "${PARSE_FAILURES}")
    message(FATAL_ERROR
        "CMake helper parse guard failed:\n  ${PARSE_FAILURES_TEXT}")
endif()

message(STATUS "[cmake-helper-parse] ${HELPER_MODULES} parse successfully")
