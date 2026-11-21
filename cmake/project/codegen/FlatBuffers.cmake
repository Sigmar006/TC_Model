# codegen/FlatBuffers.cmake
# FlatBuffers 构建期代码生成配置：定义 flatc 发现与 schema 代码生成函数。
# schemas/replay/*.fbs 在构建期编译为 *_generated.h，flatc 来自 conan flatbuffers 包
# （FlatcTargets.cmake 暴露 flatbuffers::flatc 与 FLATBUFFERS_FLATC_EXECUTABLE），版本与运行时头一致（当前 1.12.0）。
# 本文件只定义函数；实际 flatc 发现与 schema 编译由各模块 CMakeLists 在
# find_package(flatbuffers) 与目标创建之后调用。

# 生成头集中输出目录。按 <sensor>/session/generated/ 子目录布局，使
# `#include "<sensor>/session/generated/<name>_generated.h"`（Style-2）能通过
# 把本目录加入 include path 直接解析。
set(ONEQ_FLATBUFFERS_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated" CACHE INTERNAL
    "FlatBuffers 生成头根目录（构建期由 flatc 产生，不进 git）")

# 发现并校验 flatc 编译器，结果写入 ONEQ_FLATC_EXECUTABLE；必须在 find_package(flatbuffers) 之后调用。
# 优先级：0) 用户显式 -DONEQ_FLATC_EXECUTABLE=/path/to/flatc（交叉编译时指向**宿主** flatc）；
#         1) conan 暴露的 flatbuffers::flatc imported target；
#         2) FLATBUFFERS_FLATC_EXECUTABLE 变量（同来源另一形态）；
#         3) find_program(flatc) fallback（仅用于脱离 conan 的本地诊断）。
# flatc 未找到时触发 FATAL_ERROR。
macro(setup_flatc)
    # 允许外部通过 -DONEQ_FLATC_EXECUTABLE 显式指定：交叉编译（如 x86 主机构建
    # aarch64 目标）时目标平台的 flatc 无法在主机执行，必须用宿主 flatc 生成代码。
    if(DEFINED ONEQ_FLATC_EXECUTABLE AND NOT ONEQ_FLATC_EXECUTABLE STREQUAL ""
       AND EXISTS "${ONEQ_FLATC_EXECUTABLE}")
        set(_oneq_flatc_user "${ONEQ_FLATC_EXECUTABLE}")
    else()
        set(_oneq_flatc_user "")
    endif()
    set(ONEQ_FLATC_EXECUTABLE "")
    if(_oneq_flatc_user)
        set(ONEQ_FLATC_EXECUTABLE "${_oneq_flatc_user}")
    endif()
    if(TARGET flatbuffers::flatc)
        get_target_property(_oneq_flatc_loc flatbuffers::flatc IMPORTED_LOCATION)
        if(_oneq_flatc_loc AND NOT ONEQ_FLATC_EXECUTABLE)
            set(ONEQ_FLATC_EXECUTABLE "${_oneq_flatc_loc}")
        endif()
    endif()
    if(NOT ONEQ_FLATC_EXECUTABLE AND DEFINED FLATBUFFERS_FLATC_EXECUTABLE)
        set(ONEQ_FLATC_EXECUTABLE "${FLATBUFFERS_FLATC_EXECUTABLE}")
    endif()
    if(NOT ONEQ_FLATC_EXECUTABLE)
        find_program(ONEQ_FLATC_EXECUTABLE NAMES flatc)
    endif()
    unset(_oneq_flatc_user)
    unset(_oneq_flatc_loc)

    if(NOT ONEQ_FLATC_EXECUTABLE)
        message(FATAL_ERROR
            "FlatBuffers: flatc 未找到。conan flatbuffers 包应通过 FlatcTargets.cmake "
            "暴露 flatbuffers::flatc；若缺失请检查 conan install 是否成功。"
            "另可用 -DONEQ_FLATC_EXECUTABLE=<宿主 flatc> 显式指定（交叉编译场景必需）。"
            "注意 conanfile.py 必须保持 flatbuffers/*:header_only=False，否则包内无 flatc。")
    endif()

    # 解析结果持久化到 CACHE：flatbuffers_generate() 有独立作用域，需保证拿到一致的工具路径。
    set(ONEQ_FLATC_EXECUTABLE "${ONEQ_FLATC_EXECUTABLE}" CACHE INTERNAL
        "Resolved FlatBuffers compiler executable")

    # 配置期版本探测：仅当 flatc 路径是具体可执行文件时执行。
    # 当来源是 vendored 源码构建（VendorPackages.cmake）时，路径为生成器表达式
    # $<TARGET_FILE:flatc>（构建期才解析），配置期无法执行，故跳过——版本探测只是
    # STATUS 诊断，flatc 实际版本一致性由源码 tag 保证（v1.12.0 与运行时头一致）。
    if(NOT ONEQ_FLATC_EXECUTABLE MATCHES "\\$<")
        execute_process(
            COMMAND ${ONEQ_FLATC_EXECUTABLE} --version
            OUTPUT_VARIABLE _oneq_flatc_version
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
    else()
        set(_oneq_flatc_version "(vendored flatc, version probed at build time)")
    endif()
    message(STATUS "FlatBuffers: 构建期代码生成已启用")
    message(STATUS "  └─ flatc: ${ONEQ_FLATC_EXECUTABLE}")
    if(_oneq_flatc_version)
        message(STATUS "  └─ ${_oneq_flatc_version}")
    endif()
    unset(_oneq_flatc_version)
    unset(_oneq_flatc_loc)
endmacro()

# 为单个 .fbs schema 创建 flatc 代码生成规则。
# schema_file：待编译的 schema 路径；sensor_subdir：src/ 下的 sensor 目录名，
#   用于在输出目录复刻 <sensor>/session/generated/ 层级以支撑 Style-2 include；
# out_var：返回生成头的绝对路径，调用方将其加入 target 的 SOURCES 以建立编译依赖。
# 生成参数 --cpp --gen-object-api 与原 checked-in 头一致（Object API 内容虽未被手写 codec
# 消费，但保留以维持字节级一致，避免无谓 diff）。
function(flatbuffers_generate schema_file sensor_subdir out_var)
    get_filename_component(_schema_name "${schema_file}" NAME_WE)
    get_filename_component(_schema_abs "${schema_file}" ABSOLUTE)

    set(_out_subdir "${ONEQ_FLATBUFFERS_GENERATED_DIR}/${sensor_subdir}/session/generated")
    set(_out_header "${_out_subdir}/${_schema_name}_generated.h")

    # add_custom_command 要求 OUTPUT 的目录在命令执行前存在（部分生成器）。
    add_custom_command(
        OUTPUT "${_out_header}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${_out_subdir}"
        COMMAND ${ONEQ_FLATC_EXECUTABLE}
                --cpp --gen-object-api
                -o "${_out_subdir}"
                "${_schema_abs}"
        DEPENDS "${_schema_abs}"
        COMMENT "FlatBuffers: generating ${_schema_name}_generated.h from ${schema_file}"
        VERBATIM
    )

    set(${out_var} "${_out_header}" PARENT_SCOPE)
endfunction()

# 模块自有的 schema 注册：刻意不设中央 schema-to-target 清单，
# 各业务模块在自身 target 的源码与依赖旁声明所用的 schema。
function(oneq_add_flatbuffers_schemas)
    set(one_value_args TARGET MODULE)
    set(multi_value_args SCHEMAS)
    cmake_parse_arguments(ARG "" "${one_value_args}" "${multi_value_args}" ${ARGN})
    if(NOT ARG_TARGET OR NOT ARG_MODULE OR NOT ARG_SCHEMAS)
        message(FATAL_ERROR
            "oneq_add_flatbuffers_schemas() requires TARGET, MODULE and SCHEMAS")
    endif()
    if(NOT TARGET "${ARG_TARGET}")
        message(FATAL_ERROR "FlatBuffers owner target does not exist: ${ARG_TARGET}")
    endif()

    # 首次注册时才解析 flatc，避免重复执行工具发现。
    get_property(flatc_ready GLOBAL PROPERTY ONEQ_FLATC_READY)
    if(NOT flatc_ready)
        setup_flatc()
        set_property(GLOBAL PROPERTY ONEQ_FLATC_READY TRUE)
    endif()

    # 逐个 schema 生成头并挂到 owner target，同时登记到全局列表供收尾聚合。
    foreach(schema_name IN LISTS ARG_SCHEMAS)
        set(schema_path "${CMAKE_SOURCE_DIR}/schemas/replay/${schema_name}")
        if(NOT EXISTS "${schema_path}")
            message(FATAL_ERROR "FlatBuffers schema does not exist: ${schema_path}")
        endif()
        flatbuffers_generate("${schema_path}" "${ARG_MODULE}" generated_header)
        target_sources("${ARG_TARGET}" PRIVATE "${generated_header}")
        set_property(GLOBAL APPEND PROPERTY ONEQ_FLATBUFFERS_GENERATED_HEADERS
            "${generated_header}")
    endforeach()
    target_include_directories("${ARG_TARGET}" PRIVATE
        "${ONEQ_FLATBUFFERS_GENERATED_DIR}")
endfunction()

# 收尾：若有生成头，创建伞形 header target 汇总所有生成头的构建依赖。
function(oneq_finalize_flatbuffers)
    get_property(generated_headers GLOBAL PROPERTY ONEQ_FLATBUFFERS_GENERATED_HEADERS)
    if(NOT generated_headers)
        return()
    endif()
    add_custom_target(oneq_flatbuffers_headers DEPENDS ${generated_headers})
    set(ONEQ_FLATBUFFERS_HEADERS_TARGET oneq_flatbuffers_headers PARENT_SCOPE)
endfunction()
