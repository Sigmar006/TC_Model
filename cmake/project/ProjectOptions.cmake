# ProjectOptions.cmake
# 项目构建选项入口：集中声明所有用户可配置开关
# cmake/project 的唯一选项边界，业务模块不重复定义缓存变量

# 共享库 / 测试 / 示例 / 安装四个顶层开关。
option(BUILD_SHARED_LIBS "Build shared libraries instead of static" ON)
option(ENABLE_TESTING "Enable testing support" OFF)
option(ENABLE_EXAMPLES "Build example programs" OFF)
option(ENABLE_INSTALL "Enable installation rules" OFF)

# Unity Build 依赖 set_source_files_properties 的 UNITY_BUILD 属性，CMake 3.16 引入。
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
    option(ENABLE_UNITY_BUILD "Enable Unity Build for faster compilation" OFF)
else()
    set(ENABLE_UNITY_BUILD OFF CACHE INTERNAL "Unity Build requires CMake 3.16+")
endif()

# ccache 是 UNIX 下的编译缓存工具，Windows 原生不支持。
if(UNIX)
    option(USE_CCACHE "Use ccache to accelerate rebuilds" ON)
else()
    set(USE_CCACHE OFF CACHE INTERNAL "ccache is primarily for Linux/macOS")
endif()

# 预编译头同样依赖 CMake 3.16 的 target_precompile_headers。
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
    option(ENABLE_PCH "Enable precompiled headers" OFF)
    mark_as_advanced(ENABLE_PCH)
else()
    set(ENABLE_PCH OFF CACHE INTERNAL "Precompiled headers require CMake 3.16+")
endif()

# 编译期告警 / 静态分析两项开关。
option(ENABLE_WARNINGS "Enable additional compiler warnings" ON)
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
mark_as_advanced(ENABLE_CLANG_TIDY)

# clang-tidy 检查项白名单：聚焦 readability/modernize/performance/bugprone 四类，
# 保留可直接机械修复的项，排除需人工判断语义的高噪声检查。
set(CLANG_TIDY_CHECKS
    "-*,readability-braces-around-statements,readability-else-after-return,readability-isolate-declaration,readability-qualified-auto,readability-redundant-access-specifiers,readability-redundant-control-flow,readability-redundant-preprocessor,readability-static-accessed-through-instance,modernize-redundant-void-arg,modernize-use-bool-literals,modernize-use-default-member-init,modernize-use-equals-default,modernize-use-equals-delete,modernize-use-nullptr,modernize-use-override,modernize-use-using,performance-for-range-copy,performance-implicit-conversion-in-loop,performance-move-constructor-init,performance-unnecessary-copy-initialization,performance-unnecessary-value-param,bugprone-macro-parentheses,bugprone-string-constructor,bugprone-string-integer-assignment"
    CACHE STRING "clang-tidy checks whitelist")
mark_as_advanced(CLANG_TIDY_CHECKS)

# 线程栈大小档位：DEFAULT 保留平台默认，RECOMMENDED 适合常规项目，
# LARGE_PROJECT 适配多线程大栈场景，EXTREME_RECURSION 仅用于深递归算法模块。
set(STACK_SIZE_OPTION "RECOMMENDED" CACHE STRING "Stack size configuration")
set_property(CACHE STACK_SIZE_OPTION PROPERTY STRINGS
    "DEFAULT" "RECOMMENDED" "LARGE_PROJECT" "EXTREME_RECURSION")
mark_as_advanced(STACK_SIZE_OPTION)

# 包管理器 / 依赖 provider：
#   conan —— 标准方式，由 scripts/bootstrap_conan.sh 完成 install 并生成 toolchain/CMakeDeps。
#   none  —— 不走 conan：第三方依赖来自 third_party/ 源码，由 scripts/win/fetch_third_party.bat
#            git clone 到位，dependencies/VendorPackages.cmake 以 add_subdirectory / header-only 消费。
# 两个 provider 对外契约相同（相同的 imported target 名与 ONEQ_LINK_DEPENDENCIES 等输出变量），
# 故 src/、tests/、install 等下游消费点不感知 provider 切换。
set(PACKAGE_MANAGER "conan" CACHE STRING
    "1q dependency provider: 'conan' (run scripts/bootstrap_conan.sh) or 'none' (run scripts/win/fetch_third_party.bat)")
set_property(CACHE PACKAGE_MANAGER PROPERTY STRINGS "conan" "none")
if(NOT PACKAGE_MANAGER STREQUAL "conan" AND NOT PACKAGE_MANAGER STREQUAL "none")
    message(FATAL_ERROR
        "Unsupported PACKAGE_MANAGER: '${PACKAGE_MANAGER}'. "
        "1q supports 'conan' (run scripts/bootstrap_conan.sh <preset>) or 'none' "
        "(run scripts/win/fetch_third_party.bat to populate third_party/).")
endif()
if(PACKAGE_MANAGER STREQUAL "none")
    message(STATUS "Package Manager: none (deps from third_party/, run scripts/win/fetch_third_party.bat)")
else()
    message(STATUS "Package Manager: Conan")
endif()

# 机动（maneuver）模块开关：默认关闭，按需启用。
option(ONEQ_ENABLE_FLIGHT_DYNAMIC "Build the flight_dynamic (maneuver) module" OFF)
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    message(STATUS "flight_dynamic module: ENABLED")
else()
    message(STATUS "flight_dynamic module: disabled (set -DONEQ_ENABLE_FLIGHT_DYNAMIC=ON to enable)")
endif()

# 库内部调试日志总闸（PROJECT_LOG_* → 1q_library.log）。默认关闭。
# Windows 保持关闭；macOS 排库算法时才 -DONEQ_ENABLE_FILE_LOG=ON（接 spdlog）。
# 关闭时宏空操作，sink 不编译。
option(ONEQ_ENABLE_FILE_LOG "Enable library debug file log (1q_library.log); default OFF" OFF)
if(ONEQ_ENABLE_FILE_LOG)
    message(STATUS "file log backend: ENABLED (ProjectFileLog)")
else()
    message(STATUS "file log backend: disabled (PROJECT_LOG_* are no-ops)")
endif()

# 分层验收文件日志（默认关闭；宏与派生计算一并剪除）。开启后按验收用例指定文件
# 分别写入（四段同一行），不进 1q_library.log；同一指标被多个文件要求时允许重复写出。
option(ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG
    "Write SatelliteTarget.log, opir_acceptance.log, and IRInternalInterface.log" OFF)
if(ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG)
    message(STATUS "opir logs: ENABLED (SatelliteTarget.log, opir_acceptance.log, IRInternalInterface.log)")
else()
    message(STATUS "opir acceptance log: disabled (set -DONEQ_ENABLE_OPIR_ACCEPTANCE_LOG=ON to enable)")
endif()

option(ONEQ_ENABLE_RIR_ACCEPTANCE_LOG
    "Write RadarOutput.log, rir_acceptance.log, RadarInternalInterface.log, and RIR CSV evidence" OFF)
if(ONEQ_ENABLE_RIR_ACCEPTANCE_LOG)
    message(STATUS "rir logs: ENABLED (RadarOutput.log, rir_acceptance.log, RadarInternalInterface.log)")
else()
    message(STATUS "rir acceptance log: disabled (set -DONEQ_ENABLE_RIR_ACCEPTANCE_LOG=ON to enable)")
endif()

option(ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG
    "Write fusion_acceptance.log (SBIRS items 11-14 plus fused tracks / UKF / handover)" OFF)
if(ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG)
    message(STATUS "fusion acceptance log: ENABLED (fusion_acceptance.log)")
else()
    message(STATUS "fusion acceptance log: disabled (set -DONEQ_ENABLE_FUSION_ACCEPTANCE_LOG=ON to enable)")
endif()

option(ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG
    "Write inference_acceptance.log (trajectory / impact / launch)" OFF)
if(ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG)
    message(STATUS "inference acceptance log: ENABLED (inference_acceptance.log)")
else()
    message(STATUS "inference acceptance log: disabled (set -DONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG=ON to enable)")
endif()

option(ONEQ_ENABLE_PRECISION_EVALUATION_LOG
    "Write precision_acceptance.log (key metrics / AHP)" OFF)
if(ONEQ_ENABLE_PRECISION_EVALUATION_LOG)
    message(STATUS "precision evaluation log: ENABLED (precision_acceptance.log)")
else()
    message(STATUS "precision evaluation log: disabled (set -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON to enable)")
endif()

# 集成方用 CMake 指定日志目录（编进库的默认落盘路径）。
# 空 = 进程当前目录下的 log/。文件名仍按层固定；运行时环境变量可再覆盖单文件。
set(ONEQ_LOG_DIR "" CACHE PATH
    "Directory for library debug and acceptance log files (empty = <cwd>/log)")
if(ONEQ_LOG_DIR)
    file(TO_CMAKE_PATH "${ONEQ_LOG_DIR}" ONEQ_LOG_DIR_NORMALIZED)
    string(REGEX REPLACE "/+$" "" ONEQ_LOG_DIR_NORMALIZED "${ONEQ_LOG_DIR_NORMALIZED}")
    message(STATUS "log dir: ${ONEQ_LOG_DIR_NORMALIZED} (-DONEQ_LOG_DIR)")
else()
    set(ONEQ_LOG_DIR_NORMALIZED "log")
    message(STATUS "log dir: <cwd>/log (set -DONEQ_LOG_DIR=<dir> to pin)")
endif()

function(oneq_log_file out_var filename)
    set("${out_var}" "${ONEQ_LOG_DIR_NORMALIZED}/${filename}" PARENT_SCOPE)
endfunction()
