/**
 * @file OneqPlatformCompat.h
 * @brief 跨编译器平台兼容宏（VS2013 与 VS2015+ / clang/gcc 的差异收敛点）。
 *
 * VS2013（MSVC 1800）缺少若干 C++11/C99 设施，或实现不完整，集中在此以宏适配：
 *   - 无 `thread_local`（VS2015 引入）→ 用 `__declspec(thread)`。注意该存储类不
 *     支持非平凡类型（含动态初始化），非平凡线程局部量请改用线程局部指针托管。
 *   - 无 `snprintf` / `std::snprintf`（VS2015 引入）→ 用 `_snprintf_s(..., _TRUNCATE, ...)`
 *     等价实现（截断并保证以 '\0' 结尾）。
 *
 * 非 MSVC<1900 一律走标准设施，故本头对 VS2015+ / clang/gcc 是零成本转发。
 */
#ifndef ONEQ_COMMON_PLATFORM_ONEQ_PLATFORM_COMPAT_H_
#define ONEQ_COMMON_PLATFORM_ONEQ_PLATFORM_COMPAT_H_

#include <cstddef>
#include <cstdio>

#if defined(_MSC_VER) && _MSC_VER < 1900
#define ONEQ_THREAD_LOCAL __declspec(thread)
#define ONEQ_SNPRINTF(buffer, size, ...) \
    _snprintf_s((buffer), (size), _TRUNCATE, __VA_ARGS__)
#else
#define ONEQ_THREAD_LOCAL thread_local
#define ONEQ_SNPRINTF(buffer, size, ...) std::snprintf((buffer), (size), __VA_ARGS__)
#endif

#endif  // ONEQ_COMMON_PLATFORM_ONEQ_PLATFORM_COMPAT_H_
