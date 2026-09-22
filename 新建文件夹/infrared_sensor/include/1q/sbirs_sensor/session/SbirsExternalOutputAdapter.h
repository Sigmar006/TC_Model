/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsExternalOutputAdapter.
 * Model: SBIRS infrared sensor.
 * Layer: session.
 * File role: interface declarations and data contracts.
 * Responsibility: public cycle contract, validation, adapters, recording, and replay.
 * Upstream boundary: external cycle data and validated session configuration.
 * Downstream boundary: stable output frames, issues, debug views, and replay records.
 * State ownership: state remains with the enclosing session or controller.
 * Units and frames: preserve the explicit units and coordinate-frame contracts.
 * Validation: reject invalid boundary data before mutating persistent state.
 * Failure behavior: preserve the declared neutral-result or fail-closed path.
 * Dependency rule: use public DTOs at boundaries and keep private types internal.
 * Maintenance rule: update comments when inputs, outputs, or ownership change.
 */

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
