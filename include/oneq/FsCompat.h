// FsCompat.h
// std::filesystem 的编译期兼容层（内部头，不安装、不属于 1q 公共 API）
//
// 背景：本工程要求源码可在 GCC 7.3（银河麒麟 V10 基线工具链）上编译。GCC 7.3 的
// libstdc++ **没有** <filesystem>，只有 <experimental/filesystem>；GCC 8 才有
// <filesystem>，但仍需显式链接 libstdc++fs；GCC 9.1 起头文件与库均并入主线。
// 因此不能按 __cplusplus 猜分支（GCC 7.3 在 -std=c++17 下 __cplusplus 已是 201703L，
// 但头文件并不存在，会直接 fatal error），必须用 __has_include 探测**头文件是否存在**。
//
// 使用方式：
//   #include <oneq/FsCompat.h>
//   oneq_fs::create_directories(path, ec);
//
// 注意：
//  - 该头置于 include/oneq/（而非 include/1q/），因此不会被安装到下游，也不会被
//    tests/compatibility/public_api/check_public_header_cxx11.cmake 的公共头
//    C++11 禁令清单拦截。
//  - 链接期：GCC < 9.1 需要 libstdc++fs，由 cmake/compilers/CompilerClangGCC.cmake
//    统一为所有非 OBJECT 目标补加，调用方无需关心。

#ifndef ONEQ_FSCOMPAT_H_
#define ONEQ_FSCOMPAT_H_

#if defined(__has_include)
#  if __has_include(<filesystem>)
#    include <filesystem>
namespace oneq_fs = std::filesystem;
#    define ONEQ_HAS_STD_FILESYSTEM 1
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
namespace oneq_fs = std::experimental::filesystem;
#    define ONEQ_HAS_STD_EXPERIMENTAL_FILESYSTEM 1
#  else
#    error "本工具链既无 <filesystem> 也无 <experimental/filesystem>，无法提供目录操作能力"
#  endif
#else
// 不支持 __has_include 的老编译器（MSVC 2015/2017 早期等）：退回按版本判定。
#  if defined(_MSC_VER) && _MSC_VER < 1910
#    include <experimental/filesystem>
namespace oneq_fs = std::experimental::filesystem;
#  else
#    include <filesystem>
namespace oneq_fs = std::filesystem;
#    define ONEQ_HAS_STD_FILESYSTEM 1
#  endif
#endif

#endif  // ONEQ_FSCOMPAT_H_