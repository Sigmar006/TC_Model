# Keep public CMake presets aligned with the supported dependency provider and
# bootstrap entry point. A present preset is a supported user-facing contract.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(PRESETS_FILE "${SOURCE_DIR}/CMakePresets.json")
set(BOOTSTRAP_FILE "${SOURCE_DIR}/scripts/bootstrap_conan.sh")
file(READ "${PRESETS_FILE}" PRESETS_CONTENT)
file(READ "${BOOTSTRAP_FILE}" BOOTSTRAP_CONTENT)

# PACKAGE_MANAGER=none 预设走 SystemPackages provider，不调用 conan，故不得耦合
# conan toolchain：设置 PACKAGE_MANAGER=none 的 configure preset 不得同时携带
# CMAKE_TOOLCHAIN_FILE（否则会在配置期因找不到 conan_toolchain.cmake 而失败）。
# 该约束由 ProjectSetup.cmake 的 toolchain 存在性校验在配置期兜底，此处不再硬拒绝。

foreach(required_preset IN ITEMS
        llvm-ninja-debug
        llvm-ninja-release)
    if(NOT PRESETS_CONTENT MATCHES "\"name\"[ \t\r\n]*:[ \t\r\n]*\"${required_preset}\"")
        message(FATAL_ERROR "Required configure preset is missing: ${required_preset}")
    endif()
    if(NOT BOOTSTRAP_CONTENT MATCHES "${required_preset}\\)")
        message(FATAL_ERROR
            "bootstrap_conan.sh does not support required preset: ${required_preset}")
    endif()
endforeach()

execute_process(
    COMMAND "${CMAKE_COMMAND}" -S "${SOURCE_DIR}" --list-presets=all
    RESULT_VARIABLE list_presets_result
    OUTPUT_VARIABLE list_presets_output
    ERROR_VARIABLE list_presets_error)
if(NOT list_presets_result EQUAL 0)
    message(FATAL_ERROR
        "CMake could not parse CMakePresets.json:\n${list_presets_output}\n${list_presets_error}")
endif()

message(STATUS "[preset-provider] public presets and bootstrap entry point are consistent")
