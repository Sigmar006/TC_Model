# ProjectTargets.cmake
# 产品 target 工厂：统一稳定的构建装配
# 源码归属与直接依赖仍归各业务模块可见

# 按 features 层定义的顺序装配 target：先 Unity Build，再编译器 flag，
# 然后覆盖率插桩，最后预编译头。
function(oneq_apply_target_configuration)
    set(multi_value_args TARGETS LINK_TARGETS)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "oneq_apply_target_configuration() requires TARGETS")
    endif()
    # 未单独指定链接目标时，复用编译目标列表。
    if(NOT ARG_LINK_TARGETS)
        set(ARG_LINK_TARGETS ${ARG_TARGETS})
    endif()

    apply_unity_build(TARGETS ${ARG_TARGETS})
    if(MSVC)
        apply_msvc_options(
            TARGETS ${ARG_TARGETS}
            LINK_TARGETS ${ARG_LINK_TARGETS}
            ENABLE_WARNINGS ${ENABLE_WARNINGS}
            STACK_SIZE_OPTION ${STACK_SIZE_OPTION})
    else()
        apply_clang_gcc_options(
            TARGETS ${ARG_TARGETS}
            LINK_TARGETS ${ARG_LINK_TARGETS}
            ENABLE_WARNINGS ${ENABLE_WARNINGS}
            STACK_SIZE_OPTION ${STACK_SIZE_OPTION})
    endif()
    apply_precompiled_headers(TARGETS ${ARG_TARGETS})
endfunction()

# 装配对外暴露的公共库 target：MSVC 运行时、静态宏、版本号与可见性。
function(oneq_configure_public_library target)
    if(NOT TARGET "${target}")
        message(FATAL_ERROR "oneq_configure_public_library(): target does not exist: ${target}")
    endif()

    # MSVC 运行时覆盖：留空则使用默认动态 CRT（/MD /MDd），可由用户显式指定。
    set(ONEQ_MSVC_RUNTIME_LIBRARY "" CACHE STRING
        "MSVC runtime override. Empty = default dynamic CRT (/MD /MDd).")
    if(MSVC)
        if(ONEQ_MSVC_RUNTIME_LIBRARY STREQUAL "")
            set(_oneq_msvc_rt "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
        else()
            set(_oneq_msvc_rt "${ONEQ_MSVC_RUNTIME_LIBRARY}")
        endif()
        set_property(TARGET "${target}" PROPERTY MSVC_RUNTIME_LIBRARY "${_oneq_msvc_rt}")
    endif()

    # 静态构建时对外暴露 ONEQ_STATIC_DEFINE，供下游头文件正确选择 dllexport/dllimport。
    if(NOT BUILD_SHARED_LIBS)
        target_compile_definitions("${target}" PUBLIC ONEQ_STATIC_DEFINE)
    endif()

    # 统一 target 属性：导出名、debug 后缀、版本号、SOVERSION。
    set_target_properties("${target}" PROPERTIES
        EXPORT_NAME "${PROJECT_NAME_LOWER}"
        OUTPUT_NAME "${PROJECT_NAME_LOWER}"
        DEBUG_POSTFIX d
        VERSION "${PROJECT_VERSION}"
        SOVERSION "${PROJECT_VERSION_MAJOR}")
    # 非 Windows：隐藏非导出符号，缩减二进制体积并强化 ABI 边界。
    if(NOT WIN32)
        set_target_properties("${target}" PROPERTIES
            CXX_VISIBILITY_PRESET hidden
            VISIBILITY_INLINES_HIDDEN YES)
    endif()
endfunction()

# 创建业务组件 OBJECT 库并装配 include 路径、依赖与构建特性。
function(oneq_add_component target)
    set(multi_value_args SOURCES)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})
    if(NOT ARG_SOURCES)
        message(FATAL_ERROR "oneq_add_component(${target}) requires SOURCES")
    endif()
    if(TARGET "${target}")
        message(FATAL_ERROR "1q component target already exists: ${target}")
    endif()

    # 源文件统一转为绝对路径，避免不同目录下 CMakeLists 相对路径歧义。
    set(component_sources)
    foreach(component_source IN LISTS ARG_SOURCES)
        if(IS_ABSOLUTE "${component_source}")
            list(APPEND component_sources "${component_source}")
        else()
            list(APPEND component_sources
                "${CMAKE_CURRENT_SOURCE_DIR}/${component_source}")
        endif()
    endforeach()

    # OBJECT 库：编译为独立目标文件，最终由公共库 target 聚合链接。
    add_library("${target}" OBJECT ${component_sources})
    # Eigen 为 header-only；链入以便 PCH（Eigen/Core）与各 TU 解析共用 include 路径。
    if(TARGET Eigen3::Eigen)
        target_link_libraries("${target}" PRIVATE Eigen3::Eigen)
    endif()
    # 四类 include 路径：src（模块互引）、生成头目录、公共 include、FlatBuffers 生成头。
    target_include_directories("${target}" PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${PROJECT_GENERATED_INCLUDE_DIR}"
        "${CMAKE_SOURCE_DIR}/include"
        "${ONEQ_FLATBUFFERS_GENERATED_DIR}")
    target_compile_features("${target}" PRIVATE "cxx_std_${PROJECT_CXX_STANDARD}")
    if(NOT BUILD_SHARED_LIBS)
        target_compile_definitions("${target}" PRIVATE ONEQ_STATIC_DEFINE)
    endif()
    # 日志后端：启用 spdlog 时链接，否则仅定义开关宏供编译期条件编译。
    if(PROJECT_ENABLE_SPDLOG)
        target_link_libraries("${target}" PRIVATE "${PROJECT_SPDLOG_TARGET}")
    endif()
    # 库调试日志总闸是 ONEQ_ENABLE_FILE_LOG。关掉则两端后端都是 0，PROJECT_LOG_* 空操作。
    # 打开时：有 spdlog（macOS）走 spdlog；无 spdlog（Windows）走 ProjectFileLog。
    # 落盘目录由 ONEQ_LOG_DIR 编进默认路径（空则 <cwd>/log/）。
    oneq_log_file(_oneq_file_log_path "1q_library.log")
    oneq_log_file(_oneq_opir_accept_path "opir_acceptance.log")
    oneq_log_file(_oneq_rir_accept_path "rir_acceptance.log")
    oneq_log_file(_oneq_fusion_accept_path "fusion_acceptance.log")
    oneq_log_file(_oneq_inference_accept_path "inference_acceptance.log")
    oneq_log_file(_oneq_precision_accept_path "precision_acceptance.log")
    oneq_log_file(_oneq_ir_internal_interface_path "IRInternalInterface.log")
    oneq_log_file(_oneq_radar_internal_interface_path "RadarInternalInterface.log")
    oneq_log_file(_oneq_radar_output_path "RadarOutput.log")
    oneq_log_file(_oneq_satellite_target_path "SatelliteTarget.log")
    oneq_log_file(_oneq_rir_antenna_csv_path "RadarAntennaGainPatterndata.csv")
    oneq_log_file(_oneq_rir_scan_csv_path "Rir_scan_pattern.csv")
    target_compile_definitions("${target}" PRIVATE
        PROJECT_LOG_BACKEND_SPDLOG=$<AND:$<BOOL:${PROJECT_ENABLE_SPDLOG}>,$<BOOL:${ONEQ_ENABLE_FILE_LOG}>>
        PROJECT_LOG_BACKEND_FILE=$<AND:$<BOOL:${ONEQ_ENABLE_FILE_LOG}>,$<NOT:$<BOOL:${PROJECT_ENABLE_SPDLOG}>>>
        ONEQ_FILE_LOG_PATH=\"${_oneq_file_log_path}\"
        ONEQ_OPIR_ACCEPTANCE_LOG_PATH=\"${_oneq_opir_accept_path}\"
        ONEQ_RIR_ACCEPTANCE_LOG_PATH=\"${_oneq_rir_accept_path}\"
        ONEQ_FUSION_ACCEPTANCE_LOG_PATH=\"${_oneq_fusion_accept_path}\"
        ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH=\"${_oneq_inference_accept_path}\"
        ONEQ_PRECISION_ACCEPTANCE_LOG_PATH=\"${_oneq_precision_accept_path}\"
        ONEQ_IR_INTERNAL_INTERFACE_LOG_PATH=\"${_oneq_ir_internal_interface_path}\"
        ONEQ_RADAR_INTERNAL_INTERFACE_LOG_PATH=\"${_oneq_radar_internal_interface_path}\"
        ONEQ_RADAR_OUTPUT_LOG_PATH=\"${_oneq_radar_output_path}\"
        ONEQ_SATELLITE_TARGET_LOG_PATH=\"${_oneq_satellite_target_path}\"
        ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH=\"${_oneq_rir_antenna_csv_path}\"
        ONEQ_RIR_SCAN_PATTERN_CSV_PATH=\"${_oneq_rir_scan_csv_path}\"
        ONEQ_HAVE_ZLIB=$<BOOL:${ONEQ_HAVE_ZLIB}>)
    if(ONEQ_HAVE_ZLIB)
        target_link_libraries("${target}" PRIVATE ZLIB::ZLIB)
    endif()
    oneq_apply_target_configuration(TARGETS "${target}")
    # 登记到全局属性，供公共库 target 聚合时枚举。
    set_property(GLOBAL APPEND PROPERTY ONEQ_COMPONENT_TARGETS "${target}")
endfunction()

# 返回所有已登记的组件 target 列表，供公共库 target 聚合链接。
function(oneq_get_component_targets out_var)
    get_property(component_targets GLOBAL PROPERTY ONEQ_COMPONENT_TARGETS)
    set("${out_var}" "${component_targets}" PARENT_SCOPE)
endfunction()
