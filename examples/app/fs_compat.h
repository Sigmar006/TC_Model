/**
 * @file fs_compat.h
 * @brief std::filesystem 跨编译器别名（app_fs），示例装配层共用。
 *
 * 实际分支判定收敛到 include/oneq/FsCompat.h：按 __has_include 探测头文件是否存在，
 * 而不是按 __cplusplus 猜。GCC 7.3（银河麒麟 V10 基线）在 -std=c++17 下
 * __cplusplus 已经是 201703L，但 libstdc++ 并没有 <filesystem>，此前这里会走进
 * 第一个 else 分支直接 fatal error。
 */

#ifndef EXAMPLES_APP_FS_COMPAT_H_
#define EXAMPLES_APP_FS_COMPAT_H_

#include <oneq/FsCompat.h>

namespace app_fs = oneq_fs;

#endif  // EXAMPLES_APP_FS_COMPAT_H_