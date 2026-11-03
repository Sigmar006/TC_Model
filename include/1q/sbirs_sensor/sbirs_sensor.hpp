/**
 * @file sbirs_sensor.hpp
 * @brief SBIRS-inspired 传感器模块稳定会话入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SBIRS_SENSOR_HPP_
#define ONEQ_SBIRS_SENSOR_SBIRS_SENSOR_HPP_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/sbirs_sensor_config.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleOutputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsExternalInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsExternalOutputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace oneq {
namespace sbirs_sensor {

/**
 * @brief 关闭/开启"SNR↔测角误差线性标定"（默认开启）。
 *
 * @details 默认口径：逐帧测角 σ **只**由线性 SNR 标定决定，便于外部验证程序把
 *   "该行 SNR ⇒ 应有的 σ" 与实测误差逐行对账，不必了解内部实现：
 *     SNR 35 dB → σ 1.0 µrad；SNR 10 dB → σ 5.0 µrad；dB 域线性插值，区间外钳位。
 *   传入 false 时回落到物理模型（仪器地板 ⊕ 焦面光子噪声 IFOV/(√SNR)），两个锚点
 *   与物理常数均在库内固定，**不提供额外配置项**。
 *
 * @param[in] enabled true = 使用线性标定；false = 使用物理模型。
 * @return 切换前的状态（true = 切换前为线性标定）。
 */
bool ONEQ_API SetLinearSnrAngleSigmaEnabled(bool enabled);

/**
 * @brief 查询当前是否处于"SNR↔测角误差线性标定"口径。
 * @return true = 线性标定；false = 物理模型。
 */
bool ONEQ_API IsLinearSnrAngleSigmaEnabled();

}  // namespace sbirs_sensor
}  // namespace oneq

#endif  // ONEQ_SBIRS_SENSOR_SBIRS_SENSOR_HPP_
