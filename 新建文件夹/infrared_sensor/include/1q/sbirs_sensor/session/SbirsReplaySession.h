/*
 * Supplemental architecture notes for acceptance review.
 * Unit: SbirsReplaySession.
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
 * @file SbirsReplaySession.h
 * @brief 提供 SBIRS-inspired replay trace 回放入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_SESSION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief SBIRS-inspired replay 回放结果，携带回放报告、播放结果与首个错误信息。
 * @note 当 trace 中存在 failure marker 时，回填 `reached_failure_marker` 及相关字段。
 */
struct ONEQ_API SbirsReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};     /**< 回放报告 */
  oneq::replay::ReplayTracePlaybackResult playback{}; /**< 播放结果 */
  bool ok{false};                                     /**< 回放是否成功 */
  bool reached_failure_marker{false};                 /**< 是否到达 failure marker */
  std::string failure_marker_payload{};               /**< failure marker 原始 payload */
  oneq::replay::ReplayTraceFailure failure_marker_data{}; /**< failure marker 解析数据 */
  std::string first_error{};                          /**< 首个错误信息 */
};

/**
 * @brief 回放指定目录下的 SBIRS replay trace。
 * @param[in] trace_dir trace 目录路径
 * @return 回放结果，含报告、播放结果与错误信息
 */
ONEQ_API SbirsReplaySessionResult ReplaySbirsTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_REPLAY_SESSION_H_
