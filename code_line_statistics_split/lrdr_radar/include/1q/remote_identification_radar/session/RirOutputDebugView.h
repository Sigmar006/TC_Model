/**
 * @file RirOutputDebugView.h
 * @brief 远程识别雷达目标输出调试视图类型集合。
 *
 * 目标输出调试视图（开发可读逐目标状态合成）
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_OUTPUT_DEBUG_VIEW_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_OUTPUT_DEBUG_VIEW_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/remote_identification_radar/session/RirCycleResult.h"

namespace remote_identification_radar {
namespace session {

// 前向声明：Build 参数为 const 引用，header 无需完整类型，避免拉入输入重依赖。
struct RirCycleInput;

/**
 * @brief 调试视图下的目标状态分类（含周期未执行/输出中缺失等诊断态）。
 * @note 对齐 AR 航迹调试态；识别结论有无不另开状态位（走识别诊断字段）。
 */
enum class ONEQ_API RirDebugTargetStatus {
  kConfirmed = 0,        /**< 目标对应航迹处于已确认状态。 */
  kTentative = 1,        /**< 目标对应航迹处于候选状态。 */
  kLost = 2,             /**< 目标对应航迹处于丢失状态。 */
  kNotInOutput = 3,      /**< 输入目标在本周期归属表中无对应航迹。 */
  kCycleNotCompleted = 4 /**< 本周期未成功完成，无法判定状态。 */
};

/**
 * @brief 单个输入目标在调试视图下的合成状态。
 */
struct ONEQ_API RirDebugTargetState {
  std::uint64_t external_target_id{0U};                        /**< 输入目标外部标识符 */
  std::string target_name{};                                   /**< 目标名称（人读标签） */
  RirDebugTargetStatus status{RirDebugTargetStatus::kNotInOutput}; /**< 调试状态分类 */
  bool present_in_input{false};                                /**< 是否出现在本周期输入目标表 */
  bool has_track{false};                                       /**< 本周期归属表中是否存在对应航迹 */
  std::uint64_t association_key{0U};                           /**< 关联航迹的关联键 */
  std::uint32_t hit_count{0U};                                 /**< 航迹命中累计计数 */
  double position_enu_x_m{0.0};                                /**< 滤波位置 ENU x（m；有航迹时） */
  double position_enu_y_m{0.0};                                /**< 滤波位置 ENU y（m；有航迹时） */
  double position_enu_z_m{0.0};                                /**< 滤波位置 ENU z（m；有航迹时） */
  double speed_m_per_s{0.0};                                   /**< 滤波速度模长（m/s；有航迹时） */
  double slant_range_m{0.0};                                   /**< 输入几何斜距回填（m，规则 12） */
  double look_az_deg{0.0};                                     /**< 输入几何视线方位回填（deg） */
  double look_el_deg{0.0};                                     /**< 输入几何视线俯仰回填（deg） */
  bool has_recognition_output{false};                          /**< 识别链是否有本航迹结论 */
  RirRecognitionState recognition_state{RirRecognitionState::kDisabled}; /**< 识别状态机 */
  RirRecognitionCategory target_category{RirRecognitionCategory::kUnknown}; /**< 识别大类 */
  RirBallisticSubclass ballistic_subclass{RirBallisticSubclass::kUnknown}; /**< 弹道细分类型（判据未冻结恒 kUnknown） */
  std::string target_model{};                                  /**< 识别型号（未确认为空） */
  float confidence{0.0f};                                      /**< 识别置信度，[0, 1] */
  std::uint32_t observation_count{0U};                         /**< 识别证据积累量 */
  bool has_cycle_detection_snr{false}; /**< 本周期检测链是否产出 SNR（单脉冲口径，dB）。 */
  double cycle_detection_snr_db{0.0};  /**< 本周期检测 SNR（dB）；未检出时无效。 */
};

/**
 * @brief 单周期的目标输出调试视图聚合。
 */
struct ONEQ_API RirOutputDebugView {
  std::uint32_t input_cycle_index{0U};    /**< 本周期输入周期号 */
  bool executed_this_cycle{false};        /**< 本周期是否成功完成（status == kCompleted） */
  RirCycleAbortReason abort_reason{RirCycleAbortReason::kNone}; /**< 中止原因（未中止为 kNone） */
  std::vector<RirDebugTargetState> targets{}; /**< 逐输入目标的调试状态 */
  RirIssueList issues{};                  /**< 统一问题列表（规则 14；周期 issue 条目转写） */

  /** @brief 指定任务镜像（自信封逐字转写，见 RirCycleResult）。 */
  std::uint64_t designated_target_id{0U};
  bool designation_active{false};
  bool designation_reverted_to_scan{false};
  RirDesignationRevertReason designation_revert_reason{RirDesignationRevertReason::kNone};
  config::RirAzimuthElevationDeg dwell_center_deg{};
};

/**
 * @brief 把识别输出帧、执行结果与输入目标表合成为开发可读目标状态。
 *
 * 该构建器只读组合（信封归属对照 + 识别结论 + 输入几何回填），不反向影响
 * 检测/跟踪/识别链，不替代信封 track_attributions；无状态、无 Attach。
 */
class ONEQ_API RirOutputDebugViewBuilder {
 public:
  /**
   * @brief 把单周期结果与场景目标事实合成为开发可读目标状态。
   *
   * 只读组合；按输入场景目标表顺序逐项合成（零 ID 目标无法按 ID 关联航迹，
   * 落 kNotInOutput 诊断行，与 AR 行为一致）。
   *
   * @param[in] input 当前周期输入（用于遍历输入目标表与几何回填）。
   * @param[in] result 当前周期结果（用于查询归属航迹与识别结论）。
   * @return 合成的单周期目标输出调试视图。
   */
  static RirOutputDebugView Build(const RirCycleInput& input, const RirCycleResult& result);
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_OUTPUT_DEBUG_VIEW_H_
