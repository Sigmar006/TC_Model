/**
 * @file SbirsExternalOutputAdapter.h
 * @brief 定义 SBIRS-inspired 外部输出适配入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_OUTPUT_ADAPTER_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_OUTPUT_ADAPTER_H_

#include <cstddef>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 统计输出帧中的检测记录数量。
 * @param[in] frame 输出帧
 * @return 检测记录条数
 */
ONEQ_API std::size_t CountSbirsDetections(const SbirsOutputFrame& frame);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_EXTERNAL_OUTPUT_ADAPTER_H_
