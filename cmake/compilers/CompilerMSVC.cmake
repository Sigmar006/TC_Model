# CompilerMSVC.cmake
# 为 MSVC 编译器定义 target 级编译/链接选项应用函数。
# 本文件仅定义函数，不直接调用 add_compile_options() / add_link_options()，

function(apply_msvc_options)
    set(one_value_args ENABLE_WARNINGS STACK_SIZE_OPTION)
    set(multi_value_args TARGETS LINK_TARGETS)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(NOT MSVC)
        message(FATAL_ERROR "apply_msvc_options() requires an MSVC compiler")
    endif()
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "apply_msvc_options() requires TARGETS")
    endif()
    if(NOT DEFINED ARG_STACK_SIZE_OPTION OR ARG_STACK_SIZE_OPTION STREQUAL "")
        set(ARG_STACK_SIZE_OPTION "DEFAULT")
    endif()
    if(NOT ARG_LINK_TARGETS)
        set(ARG_LINK_TARGETS ${ARG_TARGETS})
    endif()

    message(STATUS "Configuring for MSVC compiler")
    message(STATUS "  └─ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

    # /MP：多源文件并行编译；/Zc:inline：剔除 COMDAT 重复定义，缩减代码体积。
    # /Zc:inline 需 VS2015+（MSVC 1900），VS2013（v120/1800）不识别，故按版本追加。
    # 编码说明：仓库全部 C/C++ 源文件统一携带 UTF-8 BOM（scripts/utf8_bom.py 维护），
    # cl.exe 对带 BOM 文件无条件按 UTF-8 解码，不依赖 /utf-8 选项、pragma 或
    # VS2015 Update 等级；交付库面向无 /utf-8 的 VS2015 消费方（BOM 自描述）。
    set(_msvc_common_compile_options /MP)
    if(MSVC_VERSION GREATER_EQUAL 1900)
        list(APPEND _msvc_common_compile_options /Zc:inline)
    endif()
    # /permissive- 与 /Zc:referenceBinding 需 VS2017+（MSVC 1910）。
    if(MSVC_VERSION GREATER_EQUAL 1910)
        list(APPEND _msvc_common_compile_options
            /permissive-           # 严格遵循标准 C++，禁用历史非标准扩展
            /Zc:referenceBinding)  # 禁止临时量绑定到非 const 左值引用（标准要求）
    endif()

    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Compiler target does not exist: ${_target}")
        endif()
        target_compile_options("${_target}" PRIVATE ${_msvc_common_compile_options})
        # Eigen 的 warning(push) 只盖住头文件解析；pop 之后本 TU 实例化 LLT/三角求解
        # 时 C4127 会重新打开。永久宏让实例化阶段也保持关闭。
        # _USE_MATH_DEFINES：MSVC 下 M_PI 等常量需要它，且必须在首个 <cmath> 之前
        # 生效。PCH 强制预包含标准头，TU 内再 #define 已晚（include guard 已触发），
        # 故升为 target 编译定义，让 PCH 与 TU 命令行一致。
        target_compile_definitions("${_target}" PRIVATE
            EIGEN_PERMANENTLY_DISABLE_STUPID_WARNINGS
            _USE_MATH_DEFINES)
        # VS2013（MSVC<1900）：noexcept/constexpr 关键字兼容层（见该头注释）。
        # _ALLOW_KEYWORD_MACROS 关闭 CRT 的禁止宏化关键字检查，/FI 保证先于任何头生效。
        if(MSVC_VERSION LESS 1900)
            target_compile_definitions("${_target}" PRIVATE _ALLOW_KEYWORD_MACROS)
            target_compile_options("${_target}" PRIVATE
                "/FI${CMAKE_SOURCE_DIR}/cmake/compat/oneq_vs2013_compat.h")
        endif()
        if(ARG_ENABLE_WARNINGS)
            target_compile_options("${_target}" PRIVATE
                /W4      # 最高信息级别警告（启用绝大多数警告，含 C4xxx 系列）
                /WX-     # 警告不视为错误（保留告警但允许编译继续）
                /wd4127  # Eigen 模板 if(Size==N) 在 /W4 下发 C4127，并附带
                         # SolveTriangular 等「正在编译类模板」note 洪流，
                         # v141 上单 TU 可刷满日志、表现为构建卡住
                /w14242  # 从 size_t 隐式截断转换（如 sizeof 结果赋给 int）
                /w14254  # 运算符转换可能导致数据丢失
                /w14263  # 成员函数签名与基类虚函数不一致（虚表未覆盖）
                /w14265  # 类有虚函数但析构函数非 virtual（多态资源泄漏风险）
                /w14287  # 非 const 成员函数调用非 static 数据成员的 this 误用
                /we4289  # 将 for-range 局部变量声明错误使用为错误（标准兼容）
                /w14296  # 表达式始终为 false（疑似逻辑错误）
                /w14311  # 指针截断转换为更小整型（64→32 位指针丢失高位）
                /w14545  # 函数体之前的不确定语句（逗号表达式误写）
                /w14546  # 在形参列表前误用逗号（误写为函数调用）
                /w14547  # 在实参列表前误用逗号
                /w14549  # 形参实参类型不一致（signed/unsigned 提醒）
                /w14555  # 表达式无副作用（疑似缺赋值运算符）
                /w14619  # pragma warning 描述格式错误
                /w14640  # 线程不安全静态局部变量的使用风险提醒
                /w14826  # 从 size_t 到带符号整型的转换可能溢出
                /w14905  # 宽字符串字面量拼接为窄字符串（类型不一致）
                /w14906  # 字符串字面量拼接跨字符串类型不一致
                /w14928) # 异常规范误用（如 throw() 旧式标注）
        else()
            target_compile_options("${_target}" PRIVATE /W3)  # 标准信息级别警告
        endif()

        target_compile_options("${_target}" PRIVATE
            $<$<CONFIG:Debug>:/Z7>                                                # 生成旧式完整调试信息（嵌入 .obj，便于调试）
            $<$<CONFIG:Debug>:/Od>                                                # 关闭优化，变量观察值与源码一致
            $<$<CONFIG:Debug>:/RTC1>                                              # 运行期检查：未初始化栈变量 + 栈帧校验
            $<$<AND:$<CONFIG:Debug>,$<VERSION_GREATER_EQUAL:${MSVC_VERSION},1910>>:/JMC>  # 启用 Just My Code 调试（仅步入用户代码）
            $<$<CONFIG:Release>:/O2>                                              # 标准优化（真发布档：不保留调试产物）
            $<$<CONFIG:Release>:/Ob2>                                             # 任意内联
            $<$<CONFIG:Release>:/Oi>                                              # 启用内建函数
            $<$<CONFIG:Release>:/GS->                                             # 关闭缓冲区安全检查（性能向，牺牲溢出防护）
            $<$<CONFIG:RelWithDebInfo>:/O2>                                       # 标准优化，兼顾性能与可调试性
            $<$<CONFIG:RelWithDebInfo>:/Ob2>                                      # 任意内联
            $<$<CONFIG:RelWithDebInfo>:/Oi>                                       # 启用内建函数
            $<$<CONFIG:RelWithDebInfo>:/Z7>                                       # 生成完整调试信息
            $<$<CONFIG:MinSizeRel>:/O1>                                           # 以代码体积为目标优化
            $<$<CONFIG:MinSizeRel>:/Os>)                                          # 优化时优先代码体积（而非速度）
    endforeach()

    foreach(_target IN LISTS ARG_LINK_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Compiler link target does not exist: ${_target}")
        endif()
        if(ARG_STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
            # 设置栈保留空间为 2MB（默认仅 1MB）
            target_link_options("${_target}" PRIVATE /STACK:2097152)
        elseif(ARG_STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
            # 设置栈保留空间为 4MB，适配较深调用栈的大中型项目
            target_link_options("${_target}" PRIVATE /STACK:4194304)
        elseif(ARG_STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
            # 设置栈保留空间为 8MB，适配深度递归 / 模板实例化场景
            target_link_options("${_target}" PRIVATE /STACK:8388608)
        endif()

        target_link_options("${_target}" PRIVATE
            $<$<CONFIG:Debug>:/DEBUG:FULL>          # 生成完整 PDB 调试符号
            $<$<CONFIG:Debug>:/INCREMENTAL>         # 启用增量链接，加快反复链接速度
            $<$<CONFIG:Release>:/OPT:REF>           # 移除未引用函数/数据（真发布档：无 PDB/调试信息）
            $<$<CONFIG:Release>:/OPT:ICF>           # 折叠等价 COMDAT 段
            $<$<CONFIG:Release>:/INCREMENTAL:NO>    # 关闭增量链接，配合 /OPT 优化
            $<$<CONFIG:RelWithDebInfo>:/DEBUG:FULL> # 生成完整 PDB 调试符号
            $<$<CONFIG:RelWithDebInfo>:/OPT:REF>    # 移除未引用函数/数据（需 /Gy 段级编译）
            $<$<CONFIG:RelWithDebInfo>:/OPT:ICF>    # 折叠等价 COMDAT 段（Identical COMDAT Folding）
            $<$<CONFIG:RelWithDebInfo>:/INCREMENTAL:NO>  # 关闭增量链接，配合 /OPT 优化
            $<$<CONFIG:MinSizeRel>:/OPT:REF>        # 移除未引用函数/数据
            $<$<CONFIG:MinSizeRel>:/OPT:ICF>)       # 折叠等价 COMDAT 段，进一步缩减体积
    endforeach()

    if(ARG_STACK_SIZE_OPTION STREQUAL "DEFAULT")
        message(STATUS "  └─ Stack size: Using system default (1MB)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
        message(STATUS "  └─ Stack size: 2MB (Recommended)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
        message(STATUS "  └─ Stack size: 4MB (Large Project)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
        message(STATUS "  └─ Stack size: 8MB (Extreme Recursion)")
    endif()

    if(ARG_ENABLE_WARNINGS)
        message(STATUS "  └─ Warnings: Enhanced (/W4 + additional checks)")
    else()
        message(STATUS "  └─ Warnings: Standard (/W3)")
    endif()
endfunction()
