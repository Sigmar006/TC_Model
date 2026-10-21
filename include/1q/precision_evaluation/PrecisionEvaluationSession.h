/**
 * @file PrecisionEvaluationSession.h
 * @brief 定义精度评估会话：对照双星探测、融合航迹与推演关键点提取误差（需求 3.2.1.6.3）。
 */

#ifndef ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_SESSION_H_
#define ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_SESSION_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "1q/api.hpp"
#include "1q/fusion/FusedTarget.h"
#include "1q/precision_evaluation/PrecisionEvaluationConfig.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace precision_evaluation {

/**
 * @brief 精度评估会话（评估层入口）。
 * @details 不驱动传感器、不握融合引擎。每周期 Step 消费调用方已产出的双星
 *          SBIRS 周期结果与融合航迹：① 红外定位角度误差；② 双星双视线交会
 *          位置误差；③ 航迹运动学估计 → 速度误差；④ 按间隔以估计状态与真值
 *          状态分别喂推演引擎 → 落点/发射点预测误差。Summarize 聚合五指标并
 *          求 AHP 权重与综合得分。
 * @note 评估只对照、不回写任何被评估模块；`[PrecisionEval]` 日志由编译期开关
 *       `ONEQ_ENABLE_PRECISION_EVALUATION_LOG`（默认 OFF）门控。
 * @note 落点/发射点误差口径 = 估计状态与真值状态经**同一推演引擎**的关键点之差，
 *       衡量状态误差传播，不含弹道模型自身偏差。
 */
class ONEQ_API PrecisionEvaluationSession {
 public:
  /**
   * @brief 构造评估会话（推演引擎 + 指标体系按配置装配）。
   * @param[in] config 评估聚合配置（本会话使用 inference / AHP / 参考误差；
   *                   双星会话配置与 FusionConfig 由地面站组件消费）。
   */
  explicit PrecisionEvaluationSession(const PrecisionEvaluationConfig& config = {});

  ~PrecisionEvaluationSession();

  PrecisionEvaluationSession(PrecisionEvaluationSession&&) noexcept;
  PrecisionEvaluationSession& operator=(PrecisionEvaluationSession&&) noexcept;

  PrecisionEvaluationSession(const PrecisionEvaluationSession&) = delete;
  PrecisionEvaluationSession& operator=(const PrecisionEvaluationSession&) = delete;

  /**
   * @brief 对照本周期双星探测与融合航迹，提取四类误差样本。
   * @param[in] cycle_index 周期号（调用方单调递增）
   * @param[in] dt_sec 步长（s）
   * @param[in] utc_julian_day UTC 儒略日（GMST 计算）
   * @param[in] ephemeris 双星星历（交会几何用 ECEF 位置）
   * @param[in] truth_targets 本周期真值目标列表（键须与各层产品可关联）
   * @param[in] result_a 主星本周期 SBIRS 结果
   * @param[in] result_b 辅星本周期 SBIRS 结果
   * @param[in] tracks 本周期融合航迹（须已开逐航迹滤波，否则无速度/推演样本）
   * @return 本周期四类误差样本
   */
  PrecisionEvaluationCycleResult Step(
      std::uint32_t cycle_index, float dt_sec, double utc_julian_day,
      const DualSatEphemerisInput& ephemeris,
      const std::vector<EvaluationTruthTarget>& truth_targets,
      const sbirs_sensor::session::SbirsCycleResult& result_a,
      const sbirs_sensor::session::SbirsCycleResult& result_b,
      const std::vector<fusion::FusedTarget>& tracks);

  /**
   * @brief 聚合全程误差样本并求 AHP 综合评分（不改变会话状态，可重复调用）。
   * @return 五指标汇总 + AHP 权重/一致性 + 综合得分；AHP 矩阵非法时 ahp_valid=false
   *         且综合分保持 0（不静默退化为等权）；无样本指标按 0 分计入综合
   *         （零证据=零分，不因空序列 rmse=0 而得满分）。
   */
  PrecisionEvaluationReport Summarize() const;

  /** @brief 诊断用：当前已缓存"最近一拍误差快照"的目标数（只读，不改会话状态）。 */
  std::size_t DebugLatestErrorCount() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_SESSION_H_
