/**
 * @file ground_station_fusion_component.h
 * @brief 消息驱动地面站融合组件（订阅探测事件 → FusionEngine::Update）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_GROUND_STATION_FUSION_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_GROUND_STATION_FUSION_COMPONENT_H_

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/precision_evaluation/PrecisionEvaluationSession.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "core/component.h"
#include "core/events.h"
#include "core/signals.h"

namespace component_attachment {

class World;

/**
 * @brief 地面站融合枢纽：消息收件箱 + 融合引擎（可选双星精度评估叠加）。
 */
class GroundStationFusionComponent : public Component {
 public:
  /**
   * @brief 仅融合引擎（无精度评估叠加）。
   * @param[in] engine 融合引擎所有权（main 创建 FusionEngine 后 std::move 传入）。
   *            本场景见 sbirs_triple_sat_fix_messages/main.cpp：config.fusion 来自
   *            PrecisionEvaluationConfig 库默认值（JSON 无 fusion 字段）；通用 runner
   *            场景则读 session_config.fusion。
   */
  explicit GroundStationFusionComponent(std::unique_ptr<fusion::FusionEngine> engine);
  /**
   * @brief 融合引擎 + 精度评估会话。
   * @param[in] engine 同上（本场景 JSON 无 fusion，用 config.fusion 库默认）。
   * @param[in] evaluation 评估会话所有权（main 用 scene.config 创建；JSON 填
   *            satellites[] / targets[] / inference_horizon_sec 等，见 LoadScene）。
   * @param[in] evaluation_source_ids 双星评估 source_id（本场景用 config 默认
   *            4 / 104，JSON 无此字段；主辅冲突时 main 给辅星 +100）。
   *            融合 inbox 仍处理本周期全部卫星，不限两颗。
   */
  GroundStationFusionComponent(
      std::unique_ptr<fusion::FusionEngine> engine,
      std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation,
      std::vector<std::uint32_t> evaluation_source_ids);
  ~GroundStationFusionComponent() override = default;

  GroundStationFusionComponent(const GroundStationFusionComponent&) = delete;
  GroundStationFusionComponent& operator=(const GroundStationFusionComponent&) = delete;

  const char* Name() const override { return "GroundStationFusion"; }
  void Step(World& world, double dt_sec) override;

  /** @brief 新周期开始前清空收件箱并确保信号已连接（须在 World::Step 之前调用）。 */
  void BeginCycle(World& world, std::uint64_t cycle);

  /**
   * @brief 写入本周期评估对照输入（每周期 World::Step 前由 main 注入）。
   * @param[in] ephemeris 双星星历（本场景由 JSON satellites[] 装入 scene.ephemeris）。
   * @param[in] truth 真值目标（本场景由 JSON targets[] 装入 scene.truth，main 每周期推进）。
   */
  void SetEvaluationInputs(const precision_evaluation::DualSatEphemerisInput& ephemeris,
                           const std::vector<precision_evaluation::EvaluationTruthTarget>& truth);

  const std::vector<fusion::FusedTarget>& targets() const { return targets_; }

  /** @brief 本周期已收卫星帧数（World::Step 之后、下次 BeginCycle 之前有效）。 */
  std::size_t last_sbirs_frame_count() const { return sbirs_inbox_.size(); }

  const precision_evaluation::PrecisionEvaluationCycleResult& last_evaluation() const {
    return last_evaluation_;
  }

  /** @brief 本周期已收卫星帧（World::Step 之后、下次 BeginCycle 之前有效）。 */
  const std::vector<SbirsFrameSubmittedEvent>& last_sbirs_frames() const { return sbirs_inbox_; }

  precision_evaluation::PrecisionEvaluationReport SummarizeEvaluation() const;

  /** @brief 设置双星视线交会配对窗口（周期数，0 为严格同周期，>=1 允许跨周期滑窗交会）。 */
  void SetPairWindowCycles(std::uint32_t window) { pair_window_cycles_ = window; }
  std::uint32_t pair_window_cycles() const { return pair_window_cycles_; }

 private:
  struct TargetObservationAnchor {
    bool valid{false};
    std::uint64_t cycle{0U};
    double measurement_time_s{0.0};
    oneq::coordinate::EcefPositionM satellite_position_ecef{};
    oneq::coordinate::Vector3d direction_ecef{};
    float infrared_snr_linear{0.0f};
    std::uint32_t source_id{0U};
  };

  void EnsureSignalConnections(World& world);
  void OnDetectionBatchSubmitted(const DetectionBatchSubmittedEvent& event);
  void OnSbirsFrameSubmitted(const SbirsFrameSubmittedEvent& event);
  void OnSbirsCrossCue(const SbirsCrossCueEvent& event);

  std::unique_ptr<fusion::FusionEngine> engine_;
  std::vector<fusion::FusedTarget> targets_{};

  std::uint64_t inbox_cycle_{0U};
  std::vector<FusionDetectionSample> detection_inbox_{};
  std::vector<SbirsFrameSubmittedEvent> sbirs_inbox_{};

  std::uint32_t pair_window_cycles_{1U};
  std::map<std::uint64_t, std::map<std::uint32_t, TargetObservationAnchor>> target_observations_{};

  std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation_;
  std::vector<std::uint32_t> evaluation_source_ids_{};
  precision_evaluation::DualSatEphemerisInput evaluation_ephemeris_{};
  std::vector<precision_evaluation::EvaluationTruthTarget> evaluation_truth_{};
  precision_evaluation::PrecisionEvaluationCycleResult last_evaluation_{};

  boost::signals2::scoped_connection detection_batch_connection_{};
  boost::signals2::scoped_connection sbirs_frame_connection_{};
  boost::signals2::scoped_connection cross_cue_connection_{}; /**< cross-cue 收件连接 */
  std::vector<SbirsCrossCueEvent> cross_cue_relay_buffer_{}; /**< 已收待转发递话（BeginCycle 冲刷） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_GROUND_STATION_FUSION_COMPONENT_H_
