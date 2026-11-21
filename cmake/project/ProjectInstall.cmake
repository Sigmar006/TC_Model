# ProjectInstall.cmake
# 安装与包导出生命周期入口
# 受 ENABLE_INSTALL 门控；生成 *Config.cmake 供下游 find_package 消费

function(oneq_configure_package_dependencies out_var)
    # 组装下游包配置文件中的 find_dependency(...) 片段。安装树只导出 1q
    # 自身产物；active 依赖须由 consumer 的 toolchain 或 CMAKE_PREFIX_PATH 提供。
    set(config_find_dependencies "")
    if(PACKAGE_MANAGER STREQUAL "conan")
        string(APPEND config_find_dependencies
            "find_dependency(Eigen3 REQUIRED CONFIG)\n"
            "find_dependency(Boost REQUIRED CONFIG)\n"
            "find_dependency(nanoflann REQUIRED CONFIG)\n"
            "find_dependency(flatbuffers REQUIRED CONFIG)\n")
        # spdlog 仅在启用时纳入，避免下游被迫安装未使用的依赖。
        if(PROJECT_ENABLE_SPDLOG)
            string(APPEND config_find_dependencies
                "find_dependency(spdlog REQUIRED CONFIG)\n")
        endif()
        string(APPEND config_find_dependencies "find_dependency(ZLIB REQUIRED CONFIG)\n")
        # SQLite3 虽是 PRIVATE 链接，但静态库的 PRIVATE 依赖会以
        # `$<LINK_ONLY:SQLite::SQLite3>` 的形式进入导出 target 的链接接口；
        # 下游 find_package(1q) 时若未在此处 find_dependency，会直接报
        # `The link interface of target "1q::core" contains: SQLite::SQLite3
        #  but the target was not found`。
        string(APPEND config_find_dependencies
            "find_dependency(SQLite3 REQUIRED CONFIG)\n")
        # jsbsim 仅在启用 flight_dynamic 时纳入，默认关闭时不依赖。
        if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
            string(APPEND config_find_dependencies
                "find_dependency(jsbsim REQUIRED CONFIG)\n")
        endif()
        # HDF5 输出能力依赖 HighFive，仅在 C++17 且包存在时启用。
        if(ONEQ_ENABLE_HDF5_OUTPUT)
            string(APPEND config_find_dependencies
                "find_dependency(HighFive REQUIRED CONFIG)\n")
        endif()
    endif()
    # Threads::Threads 由 oneq_find_project_dependencies() 无条件加入链接闭包，与
    # provider 无关（CMake 内置 FindThreads 模块），因此放在 conan 分支之外。
    # 缺这一句时下游 find_package(1q) 会报
    # `The link interface of target "1q::core" contains: Threads::Threads but the
    #  target was not found`。
    string(APPEND config_find_dependencies "find_dependency(Threads REQUIRED)\n")
    set("${out_var}" "${config_find_dependencies}" PARENT_SCOPE)
endfunction()

function(oneq_install_project)
    if(NOT ENABLE_INSTALL)
        return()
    endif()

    include(CMakePackageConfigHelpers)

    # 安装核心 target 并关联到导出集，按 GNUInstallDirs 标准路径分类产物。
    install(TARGETS ${PROJECT_CORE_TARGET}
        EXPORT ${PROJECT_EXPORT_SET}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # include/1q/ 是受控的公共头边界：镜像安装而非维护第二份模块本地头清单。
    install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/1q/"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/1q"
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
            PATTERN "README.md")
    install(FILES "${PROJECT_GENERATED_EXPORT_HEADER}"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q
    )

    # 导出 target 集为 <PROJECT_NAME>Targets.cmake，下游以 <PROJECT_NAME>:: 命名空间引用。
    install(EXPORT ${PROJECT_EXPORT_SET}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
        FILE ${PROJECT_NAME}Targets.cmake
    )

    # 生成 *Config.cmake.in 模板并 configure 为最终包配置文件。
    oneq_configure_package_dependencies(ONEQ_CONFIG_FIND_DEPENDENCIES)
    set(_oneq_config_template "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake.in")
    file(WRITE "${_oneq_config_template}" [=[@PACKAGE_INIT@

include(CMakeFindDependencyMacro)

@ONEQ_CONFIG_FIND_DEPENDENCIES@

include("${CMAKE_CURRENT_LIST_DIR}/@PROJECT_NAME@Targets.cmake")

if(MSVC AND TARGET @PROJECT_NAME@::@PROJECT_NAME_LOWER@)
    set_property(TARGET @PROJECT_NAME@::@PROJECT_NAME_LOWER@ PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

check_required_components(@PROJECT_NAME@)

set(@PROJECT_NAME@_VERSION @PROJECT_VERSION@)
set(@PROJECT_NAME@_INCLUDE_DIRS "@PACKAGE_CMAKE_INSTALL_INCLUDEDIR@")
set(@PROJECT_NAME@_LIBRARIES @PROJECT_NAME@::@PROJECT_NAME_LOWER@)
]=])

    configure_package_config_file(
        "${_oneq_config_template}"
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
        PATH_VARS CMAKE_INSTALL_INCLUDEDIR
    )

    # 版本兼容策略采用 SameMajorVersion：次版本/补丁升级视为兼容，主版本升级需人工确认。
    write_basic_package_version_file(
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    # 安装两个配置文件到 cmake/ 目录，下游 find_package 即可定位。
    install(FILES
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )

endfunction()
