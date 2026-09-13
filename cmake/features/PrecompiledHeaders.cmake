# PrecompiledHeaders.cmake
# 定义项目自有 target 的预编译头（PCH）策略函数。
# 与 UnityBuild 同属通用编译特性层，不绑定特定项目。
#
# 每个 target 用同一份头清单、各自的编译命令行编出自己的 PCH。刻意不做
# REUSE_FROM 共享 anchor PCH：本仓库各 target 有大量私有 define（验收日志路径、
# 日志后端开关），共享 PCH 与 TU 命令行不一致会触发 MSVC C4651，且该警告由
# PCH 一致性校验发出、/wd4651 无法屏蔽（已实测）。每 target 独立 PCH 全量重建
# 多出的只是若干个可并行的小 PCH 编译。

function(apply_precompiled_headers)
    set(multi_value_args TARGETS)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})

    if(NOT ENABLE_PCH)
        message(STATUS "Precompiled Headers: Disabled")
        return()
    endif()
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "apply_precompiled_headers() requires TARGETS")
    endif()

    set(_oneq_pch_headers
        <vector>
        <string>
        <array>
        <deque>
        <list>
        <map>
        <set>
        <unordered_map>
        <unordered_set>
        <memory>
        <algorithm>
        <numeric>
        <utility>
        <functional>
        <tuple>
        <sstream>
        <cmath>
        <cstdint>
        <cstring>
        <chrono>)
    if(CMAKE_CXX_STANDARD GREATER_EQUAL 17)
        list(APPEND _oneq_pch_headers
            <optional>
            <variant>)
    endif()
    # Eigen：Kalman/LLT 主路径高频头；PCH 摊销重复解析（Windows preset 默认 ENABLE_PCH=ON）。
    # 仅给链接了 Eigen 的 target 预编 Eigen 头——消费者 target（如 batch_validation）
    # 不链接 Eigen，其 PCH 编译期没有 Eigen include 路径，预含 Eigen 头会 C1083。
    set(_oneq_pch_eigen_headers)
    if(TARGET Eigen3::Eigen)
        set(_oneq_pch_eigen_headers
            <Eigen/Core>
            <Eigen/Cholesky>)
    endif()

    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "PCH target does not exist: ${_target}")
        endif()
        set(_oneq_target_pch_headers "${_oneq_pch_headers}")
        if(_oneq_pch_eigen_headers)
            get_target_property(_oneq_t_links "${_target}" LINK_LIBRARIES)
            get_target_property(_oneq_t_iface_links "${_target}" INTERFACE_LINK_LIBRARIES)
            set(_oneq_t_link_text "")
            if(_oneq_t_links)
                string(JOIN ";" _oneq_t_link_text ${_oneq_t_links})
            endif()
            if(_oneq_t_iface_links)
                string(JOIN ";" _oneq_t_link_text "${_oneq_t_link_text};${_oneq_t_iface_links}")
            endif()
            if(_oneq_t_link_text MATCHES "Eigen3::Eigen")
                list(APPEND _oneq_target_pch_headers ${_oneq_pch_eigen_headers})
            endif()
            unset(_oneq_t_links)
            unset(_oneq_t_iface_links)
            unset(_oneq_t_link_text)
        endif()
        target_precompile_headers("${_target}" PRIVATE ${_oneq_target_pch_headers})
        unset(_oneq_target_pch_headers)
    endforeach()

    message(STATUS "Precompiled Headers: Enabled (per-target)")
endfunction()
