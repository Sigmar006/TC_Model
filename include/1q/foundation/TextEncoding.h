/**
 * @file TextEncoding.h
 * @brief 外部字符串的编码归一工具（1q 边界统一入口）。
 *
 * 背景（2026-11-06）：平台/模型库给 1q 的单元名称等字符串是**本地代码页（GBK）**字节，
 * 而 1q 的验收日志按 **UTF-8** 写出；两者混在同一条日志行里时，写出通道只能"整行猜一种
 * 编码"，结果把已经是 UTF-8 的部分又按 GBK 转了一遍，出现 `鍗槦-1-1`（应为 `卫星-1-1`）
 * 这类"合法但错"的乱码。
 *
 * 解决办法：**在外部字符串进入 1q 的边界就归一到 UTF-8**，此后内部只存在一种编码，
 * 写出通道无需再做任何猜测式转换。
 *
 * 使用约束：
 * - 只用于"外部输入的文本"（单元名、目标名、来源名等），不要用于内部常量；
 * - 归一失败时**保留原串**（宁可不转，也不清空）；
 * - 定长字段请配合 `TruncateUtf8` 截断，避免切坏多字节字符。
 */

#ifndef ONEQ_FOUNDATION_TEXT_ENCODING_H_
#define ONEQ_FOUNDATION_TEXT_ENCODING_H_

#include <cstddef>
#include <string>

#include "1q/api.hpp"

namespace oneq {
namespace foundation {

/**
 * @brief 判断字节串是否为合法 UTF-8（Windows 下用 MB_ERR_INVALID_CHARS 严格校验）。
 * @param[in] text 待判定内容。
 * @return 合法 UTF-8 返回 true；空串视为合法；非 Windows 平台恒为 true。
 */
ONEQ_API bool IsValidUtf8(const std::string& text);

/**
 * @brief 把外部字符串归一为 UTF-8。
 *
 * 归一顺序：
 *   1. 已是合法 UTF-8 —— 原样返回；
 *   2. 按 CP_ACP（GBK）解码再编码为 UTF-8；
 *   3. 再按 GB18030 尝试；
 *   4. 仍失败则把非法字节替换为 '?'，**绝不返回坏字节**。
 *
 * @param[in] text 外部来源的字节串。
 * @return UTF-8 编码的字符串；输入为空时返回空串。
 */
ONEQ_API std::string NormalizeExternalText(const std::string& text);

/**
 * @brief 按 UTF-8 字符边界把字符串截断到不超过 max_bytes 字节。
 *
 * 用于写入定长字段（例如固定长度的 char 数组）：截断点若落在多字节字符中间，会回退到
 * 该字符的起始字节，保证结果仍是合法 UTF-8。
 *
 * @param[in] text      UTF-8 字符串。
 * @param[in] max_bytes 允许的最大字节数。
 * @return 截断后的字符串（不超过 max_bytes 字节，且不切坏字符）。
 */
ONEQ_API std::string TruncateUtf8(const std::string& text, std::size_t max_bytes);

}  // namespace foundation
}  // namespace oneq

#endif  // ONEQ_FOUNDATION_TEXT_ENCODING_H_
