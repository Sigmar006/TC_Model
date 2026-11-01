/**
 * @file AcceptanceLogWriter.h
 * @brief 集成方写验收行的公共入口（1q 库统一落盘编码/换行/跨模块互斥）。
 */

#ifndef ONEQ_LOGGING_ACCEPTANCE_LOG_WRITER_H_
#define ONEQ_LOGGING_ACCEPTANCE_LOG_WRITER_H_

#include <string>

namespace oneq {
namespace logging {

/**
 * @brief 把一行验收文本写入 `SatelliteTarget.log` 通道。
 *
 * @param[in] line 行内容（**系统 ANSI/GBK 编码、不带行尾换行**）。库内会做
 *                 ANSI→UTF-8 转码、补 `'\n'`、并在跨 DLL 命名互斥体保护下落盘。
 *
 * @note 专供"库出字符串、集成方落盘"的用法（如融合数据处理器把精度评估的
 *       第 26/27 项行写到宿主日志）。集成方**不要**自己 `fopen` 追加：那样会与
 *       本通道的库内流形成两条独立文件句柄，实测出现过字节被覆盖、整行消失。
 * @note 已编码成 UTF-8 的文本不要送进来——库内还会再按 ANSI 转一次。
 */
void WriteSatelliteTargetAcceptanceLine(const std::string& line);

}  // namespace logging
}  // namespace oneq

#endif  // ONEQ_LOGGING_ACCEPTANCE_LOG_WRITER_H_
