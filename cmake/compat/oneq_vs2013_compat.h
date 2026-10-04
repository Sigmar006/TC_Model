/**
 * @file oneq_vs2013_compat.h
 * @brief VS2013（MSVC 1800）关键字兼容层：以宏替代该编译器尚未支持的关键字。
 *
 * VS2013 不识别 C++11 的 `noexcept` 与 `constexpr`（二者自 VS2015 / MSVC 1900 起支持），
 * 且 `noexcept` 在 MSVC 2013 下不是关键字（被当作普通标识符，故 `S(S&&) noexcept;`
 * 报 C3646），`constexpr` 声明则报 C2144 等。启用本层后：
 *   - `noexcept` → 空。MSVC 2013 既无该关键字，也不允许在 `= default` 的显式默认化
 *     函数上书写异常规范（`throw() = default` 报 C2610），故整体抹除；损失的是异常
 *     规范语义，而 MSVC 2013 的标准库本就不据 `is_nothrow_*` 做移动优化。
 *   - `constexpr` → `const`。本工程全部 constexpr 用法均为变量声明（无 constexpr
 *     函数/构造函数），转为 const 后：整型/指针/枚举仍是常量表达式（数组界、模板实参
 *     等照常可用）；少数 `static constexpr` 非整型成员已另按 C++11 规则改为
 *     类外定义。
 *
 * 该头仅由 CMake 在 MSVC < 1900 时经 /FI 强制包含（见 cmake/compilers/CompilerMSVC.cmake），
 * 并配合 `_ALLOW_KEYWORD_MACROS` 关闭 CRT 的“禁止宏化关键字”检查（VC/include/xkeycheck.h）。
 * 不进入 VS2015+ 构建，也不影响第三方依赖（Eigen/Boost 1.68/nanoflann 在 MSVC 1800 下
 * 均以特性宏规避这些关键字）。
 */
#ifndef ONEQ_CMAKE_COMPAT_ONEQ_VS2013_COMPAT_H_
#define ONEQ_CMAKE_COMPAT_ONEQ_VS2013_COMPAT_H_

#if defined(_MSC_VER) && _MSC_VER < 1900
#ifndef _ALLOW_KEYWORD_MACROS
#define _ALLOW_KEYWORD_MACROS
#endif
#define noexcept
#define constexpr const
#endif

#endif  // ONEQ_CMAKE_COMPAT_ONEQ_VS2013_COMPAT_H_
