/**
 * @file SbirsCycleOutputAdapter.h
 * @brief 定义 SBIRS-inspired 输出适配入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_OUTPUT_ADAPTER_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_OUTPUT_ADAPTER_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 检查输出帧是否仅包含原生 SBIRS 观测字段（契约边界守卫）。
 * @param[in] frame 待检查的输出帧
 * @return 仅含原生字段返回 true，否则返回 false
 * @note 用于输出边界测试，确保归属/真值等字段不混入 raw output。
 */
ONEQ_API bool SbirsOutputFrameContainsOnlyNativeFields(const SbirsOutputFrame& frame);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_OUTPUT_ADAPTER_H_
