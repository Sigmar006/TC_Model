/**
 * @file RirCycleResult.h
 * @brief 远程识别雷达单周期执行结果类型。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_RESULT_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_RESULT_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/remote_identification_radar/session/RirTrackRequest.h"

namespace remote_identification_radar {
namespace session {

/** @brief 远程识别雷达单周期执行状态；该枚举是结果有效性的唯一真相。 */
enum class RirCycleStatus : std::uint8_t {
  kCompleted = 0,        /**< 周期完成。 */
  kPoweredOff,           /**< 传感器关机（只推进世界时间）。 */
  kRejectedInvalidInput, /**< 输入校验拒绝。 */
  kRejectedInvalidConfig,/**< 配置校验拒绝。 */
  kRejectedExecution,    /**< 执行阶段失败。 */
};

/**
 * @brief 指定识别任务回退成因（镜像 AR `ArDesignationRevertReason` 形状）。
 */
enum class RirDesignationRevertReason : std::uint8_t {
  kNone = 0,          /**< 无回退（指定识别驻留正常/任务完成）。 */
  kNotRecognized = 1, /**< 指定目标不在场景（驻留期间回扫描等待）。 */
  kAcquisitionTimeout = 2, /**< 限时窗口耗尽仍未识别：任务作废（回到扫描，终态）。 */
  kOutsideSteerableVolume = 3 /**< 目标视线越出 scan_center + 可扫描体积（驻留回扫描；转台重新瞄准后可恢复）。 */
};

/**
 * @brief 本周期驻留指向来源（加性字段）。
 * @note 外部跟踪（`kExternalTrack`）与扫描基准（`scan_center`）是**两种不同语义**：
 *       前者每周期把首个驻留对准目标方向，后者只是把扫描波位表的方位原点平移。
 *       实测：仅用扫描基准时零偏移、完美指向下仍 0/25 周期检出（且失败是静默的）；
 *       用外部跟踪时 25/25 检出。见
 *       docs/review/ir_external_position_driven_rir_e2e_2026-10-04.md。
 */
enum class RirPointingSource : std::uint8_t {
  kScan = 0,          /**< 扫描波位（区域搜索）。 */
  kTrack = 1,         /**< 库内确认航迹跟踪驻留。 */
  kDesignation = 2,   /**< 人工指定识别目标。 */
  kExternalTrack = 3, /**< 外部跟踪请求（中心 + 邻域窗口）。 */
};

/**
 * @brief RirCycleResult 描述单周期执行后的聚合结果。
 * @note 只有 `status == kCompleted` 携带识别输出；拒绝周期不复用上一帧。
 *       `kIdentify` 且 RF 链解析成功时，`emission_frame` 携带本周期全部驻留的
 *       实际 RIR 发射（逐驻留一条，指向 = 该驻留波位，单驻留周期恰一条；
 *       供编排层汇集全球 RF scene；与 AR `ArCycleResult::emission_frame` 同契约）。
 */
struct ONEQ_API RirCycleResult {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号，用于失败结果与 trace 归属 */
  RirCycleStatus status{RirCycleStatus::kRejectedInvalidInput}; /**< 周期执行状态。 */
  RirOutputFrame output_frame{}; /**< 当前调用返回的识别输出帧。 */
  oneq::electromagnetics::RfEmissionFrame emission_frame{}; /**< 本周期实际 RIR 发射。 */
  RirIssueList issues{}; /**< 统一问题列表（规则 14：校验问题 phase=kInputValidation 在前 +
                               执行诊断 phase=kExecution/kOutputContract 在后） */
  RirCycleAbortReason abort_reason{
      RirCycleAbortReason::kNone}; /**< 若周期 abort，给出结构化原因 */
  bool has_recognition_summary{false}; /**< 本周期是否发布了识别效能摘要 */
  RirRecognitionCycleSummary recognition_summary{}; /**< 本周期识别效能摘要；未执行时保持默认值 */
  std::vector<RirTrackAttributionRecord> track_attributions{}; /**< 航迹归属视图（加性字段，
      默认空；仅 kCompleted 周期携带，非执行周期不复用上一周期列表） */

  /** @brief 当前指定识别目标外部 ID（未指定/任务完成/作废后为 0）。 */
  std::uint64_t designated_target_id{0U};
  /** @brief 本周期是否对指定目标执行识别驻留（任务窗口内且目标在场景）。 */
  bool designation_active{false};
  /** @brief 本周期指定任务未达成（目标缺席）回扫描；作废沿亦为 true
   *         （成因 kAcquisitionTimeout，其后指定清零）。 */
  bool designation_reverted_to_scan{false};
  /** @brief 回退成因（仅 designation_reverted_to_scan == true 时有意义）。 */
  RirDesignationRevertReason designation_revert_reason{RirDesignationRevertReason::kNone};
  /** @brief 本周期驻留波束中心（扫描波位或指定目标指向；雷达局部 az/el，deg）。 */
  config::RirAzimuthElevationDeg dwell_center_deg{};

  /** @brief 本周期驻留指向来源（加性字段；默认扫描）。 */
  RirPointingSource pointing_source{RirPointingSource::kScan};

  /** @brief 本周期外推后的外部照射中心（雷达局部 ENU az/el，deg）；未启用外部跟踪时为默认值。
   *  @note 与 `dwell_center_deg` 未必相同：后者是首个非跟踪驻留，本字段是中心本身。
   *        集成方据此核对自己提交的位置/速度是否被正确消费。 */
  config::RirAzimuthElevationDeg external_track_center_deg{};

  /** @brief 本周期外部跟踪阶段（加性字段；未启用为 kNone）。 */
  RirExternalTrackPhase external_track_phase{RirExternalTrackPhase::kNone};

  /** @brief 外部跟踪邻域窗口的波位推进序号（每周期随窗口波位重排而递增）。 */
  std::uint32_t external_track_sweep_index{0U};

  /**
   * @brief 本周期驻留计划的种类计数（加性字段，四项之和 = 本周期实际驻留数）。
   *
   * @note 用途：① 验收第 37/66 项"搜索/指定/跟踪"分布可由结果层直接读出，不必再数日志；
   *       ② 回归"有外部引导时全部槽位给外部窗口"的调度策略（2026-10-22 外部优先）：
   *       外部跟踪窗口吃满预算的周期里 `search_dwell_count == 0` 且
   *       `internal_track_dwell_count == 0`。
   */
  std::uint32_t search_dwell_count{0U};
  std::uint32_t designate_dwell_count{0U};
  std::uint32_t internal_track_dwell_count{0U};
  std::uint32_t external_track_dwell_count{0U};

  /**
   * @brief 本周期外部 cue 被合理性门判为不可信（位置跳变 > 10 km/s × 间隔 + 50 km）。
   *
   * @note 只作标注：降级**不改变**槽位分配（外部窗口照样吃满预算，见 2026-10-22 外部优先
   *       策略）。现场据此判断引导质量——数千公里外的目标，融合位置噪声 0.5°~1° 就折合
   *       数十至上百公里，会频繁触发本条，故不能拿它当"引导不可用"的判据。
   */
  bool external_cue_degraded{false};

  /**
   * @brief 本周期「实际有效目标最大斜距」（m）：本周期持有航迹的目标里最大输入几何斜距。
   *
   * 与 `mission.max_range_m`（径向粗筛门，仅过滤超远候选）语义不同：本量反映
   * SNR 链路预算下真正被探到/成航迹的目标能达到的最远斜距，是给外部集成方
   * 判断「实际探测距离」的观测口径。本周期无任何航迹时为 0。
   */
  float max_detected_slant_range_m{0.0f};
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_RESULT_H_
