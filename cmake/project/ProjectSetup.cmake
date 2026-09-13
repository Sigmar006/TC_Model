# ProjectSetup.cmake
# 顶层 CMakeLists.txt 的生命周期宏
# 故意用 macro 而非 function：目录级输出/测试/安装设置必须写入调用方目录作用域

set(ONEQ_PROJECT_SETUP_DIR "${CMAKE_CURRENT_LIST_DIR}")

macro(oneq_prepare_project)
    # 校验 toolchain 文件存在：缺失时给出明确的引导提示而非晦涩的下游报错。
    if(DEFINED CMAKE_TOOLCHAIN_FILE)
        get_filename_component(_requested_toolchain "${CMAKE_TOOLCHAIN_FILE}" ABSOLUTE)
        if(NOT EXISTS "${_requested_toolchain}")
            message(FATAL_ERROR
                "Requested toolchain does not exist: ${CMAKE_TOOLCHAIN_FILE}\n"
                "请先运行 scripts/bootstrap_conan.sh <preset> 生成 conan_toolchain.cmake，"
                "再执行 cmake --preset <preset>。")
        endif()
    endif()

    # C++ 标准默认 17；用户显式指定时优先采用，但不低于 C++11。
    set(PROJECT_DEFAULT_CXX_STANDARD 17)
    if(DEFINED CMAKE_CXX_STANDARD AND NOT CMAKE_CXX_STANDARD STREQUAL "")
        set(PROJECT_CXX_STANDARD "${CMAKE_CXX_STANDARD}")
    else()
        set(PROJECT_CXX_STANDARD "${PROJECT_DEFAULT_CXX_STANDARD}")
    endif()
    if(PROJECT_CXX_STANDARD LESS 11)
        message(FATAL_ERROR "CMAKE_CXX_STANDARD must be at least 11")
    endif()

    # CMP0091：改用 MSVC_RUNTIME_LIBRARY 属性而非 /MD /MDd 全局 flag。
    # CMP0092：关闭 MSVC 默认的 /W3，交由编译器模块自行控制告警等级。
    if(POLICY CMP0091)
        cmake_policy(SET CMP0091 NEW)
    endif()
    if(POLICY CMP0092)
        cmake_policy(SET CMP0092 NEW)
    endif()
endmacro()

macro(oneq_configure_project)
    # 派生目标命名约定：别名 <PROJECT_NAME>::core、核心 target <name>_lib、导出集 <PROJECT_NAME>Targets。
    string(TOLOWER "${PROJECT_NAME}" PROJECT_NAME_LOWER)
    set(PROJECT_ALIAS "${PROJECT_NAME}::core")
    set(PROJECT_CORE_TARGET "${PROJECT_NAME_LOWER}_lib")
    set(PROJECT_EXPORT_SET "${PROJECT_NAME}Targets")

    # 加载顺序敏感：先选项（定义缓存变量），再特性层（函数定义），
    # 然后 codegen 与编译器 flag，最后 project 子模块（依赖、安装、target、汇总、legacy）。
    include("${ONEQ_PROJECT_SETUP_DIR}/ProjectOptions.cmake")
    include("${CMAKE_SOURCE_DIR}/cmake/features/UnityBuild.cmake")
    include("${CMAKE_SOURCE_DIR}/cmake/features/CCache.cmake")
    include("${CMAKE_SOURCE_DIR}/cmake/features/ClangTidy.cmake")
    include("${CMAKE_SOURCE_DIR}/cmake/features/ClangFormat.cmake")
    include("${CMAKE_SOURCE_DIR}/cmake/features/PrecompiledHeaders.cmake")
    include("${ONEQ_PROJECT_SETUP_DIR}/codegen/FlatBuffers.cmake")
    if(MSVC)
        include("${CMAKE_SOURCE_DIR}/cmake/compilers/CompilerMSVC.cmake")
    else()
        include("${CMAKE_SOURCE_DIR}/cmake/compilers/CompilerClangGCC.cmake")
    endif()

    include("${ONEQ_PROJECT_SETUP_DIR}/ProjectDependencies.cmake")
    include("${ONEQ_PROJECT_SETUP_DIR}/ProjectInstall.cmake")
    include("${ONEQ_PROJECT_SETUP_DIR}/ProjectTargets.cmake")
    include("${ONEQ_PROJECT_SETUP_DIR}/ProjectSummary.cmake")

    # 生命周期四步：环境 → 依赖 → 产物目录 → 测试/安装设施。
    oneq_configure_project_environment()
    oneq_find_project_dependencies()
    oneq_configure_project_outputs()
    oneq_configure_project_facilities()
endmacro()

macro(oneq_configure_project_environment)
    set(CMAKE_CXX_STANDARD "${PROJECT_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    # 多配置生成器（如 Visual Studio、Ninja Multi-Config）在配置期不固定 BUILD_TYPE，
    # 需确保 RelWithDebInfo 在候选列表中；单配置生成器则补默认 Release。
    get_property(ONEQ_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    if(ONEQ_IS_MULTI_CONFIG)
        message(STATUS "Multi-config generator detected (${CMAKE_GENERATOR})")
        if(NOT "RelWithDebInfo" IN_LIST CMAKE_CONFIGURATION_TYPES)
            list(APPEND CMAKE_CONFIGURATION_TYPES RelWithDebInfo)
        endif()
    else()
        if(NOT CMAKE_BUILD_TYPE)
            set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build" FORCE)
        endif()
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
        message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
    endif()

    # 32 位目标未完整验证，提前告警。
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        message(WARNING "32-bit compilation detected. This may not be fully supported.")
    endif()
endmacro()

macro(oneq_configure_project_outputs)
    # 多配置：按 CONFIG 后缀分目录（如 lib/Release、bin/Debug），互不覆盖。
    if(ONEQ_IS_MULTI_CONFIG)
        foreach(ONEQ_OUTPUT_CONFIG IN LISTS CMAKE_CONFIGURATION_TYPES)
            string(TOUPPER "${ONEQ_OUTPUT_CONFIG}" ONEQ_OUTPUT_CONFIG_UPPER)
            set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${ONEQ_OUTPUT_CONFIG_UPPER} "${CMAKE_BINARY_DIR}/${ONEQ_OUTPUT_CONFIG}/lib")
            set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${ONEQ_OUTPUT_CONFIG_UPPER} "${CMAKE_BINARY_DIR}/${ONEQ_OUTPUT_CONFIG}/lib")
            set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${ONEQ_OUTPUT_CONFIG_UPPER} "${CMAKE_BINARY_DIR}/${ONEQ_OUTPUT_CONFIG}/bin")
        endforeach()
    else()
        # 单配置：统一归入 lib / bin。
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
    endif()
endmacro()

macro(oneq_configure_project_facilities)
    # 测试设施：启用 CTest 框架。
    if(ENABLE_TESTING)
        include(CTest)
        enable_testing()
    endif()

    # 安装设施：GNUInstallDirs 提供标准路径；若用户未显式指定安装前缀，回退到构建树下的 install/。
    if(ENABLE_INSTALL)
        include(GNUInstallDirs)
        if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
            set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation prefix" FORCE)
        endif()
    endif()
endmacro()
