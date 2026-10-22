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
 * @brief 把一条验收行转成落盘编码（Windows：ANSI -> UTF-8；其它平台原样返回）。
 *
 * @note 供"库出字符串、集成方落盘"的用法：集成方自己写文件时先过本函数，落盘字节才与
 *       库内 `WriteAcceptanceLog` 写出的一致（UTF-8）。与库内用的是同一个转换，重复调用
 *       不会二次转码已有 UTF-8 的风险由调用方保证（每行只调一次）。
 */
std::string EncodeAcceptanceLineForFile(const std::string& line);


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

  /**
   * @brief 最近一次 `Summarize()` 产生的第 26/27 项验收行（完整行，已含前四段格式）。
   *
   * @return 行列表：第 26 项逐目标各一行（无样本目标不产生），随后第 27 项一行
   *         （AHP 非法时该行不产生）。未被 `Summarize()` 调用过时为空。
   *
   * @note 为什么单独暴露：2026-10-22 现场定位确认，宿主进程里由评估线程直接
   *       `WriteAcceptanceLog` 写出的行始终不可见（写调用确实执行、sink 指向现场日志、
   *       流 healthy、tellp 增长；同目录另开文件正常、同一 DLL 的推演行正常），而
   *       `Summarize()` 本身是纯计算。故把"算/格式化"与"写"拆开：库出字符串，
   *       集成方在**自己确认可写盘的线程**上按原样落盘即可，行内容与库内写盘逐字节一致。
   *       库内仍保留原写盘路径（单 DLL 场景/示例程序不受影响）。
   */
  const std::vector<std::string>& LastAcceptanceLines() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_SESSION_H_
