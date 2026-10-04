/**
 * @file InferenceAcceptanceRecords.h
 * @brief 推演层验收行拼装。
 */

#ifndef ONEQ_SRC_TARGET_INFERENCE_INFERENCE_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_TARGET_INFERENCE_INFERENCE_ACCEPTANCE_RECORDS_H_

#include <array>
#include <cstdint>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/target_inference/InferenceResult.h"
#include "1q/target_inference/InferenceTrackState.h"
#include "1q/target_inference/TargetInferenceConfig.h"

namespace target_inference {

/**
 * @brief 比机械能（单位：J/kg）：ε = ½|v|² − μ/|r|。
 * @param[in] position       ECEF 位置（m）。
 * @param[in] velocity_ecef  ECEF 速度（m/s）。
 * @param[in] earth_mu_m3_per_s2 地球引力参数（m³/s²，引擎配置单源传入）。
 * @return 位置范数 > 0 时返回比机械能；位置为零矢时返回 0（不可用，调用方跳过）。
 * @note 不受验收日志宏门控，恒编译可测；μ 不在本文件新造常数。
 */
double SpecificMechanicalEnergyJPerKg(const oneq::coordinate::EcefPositionM& position,
                                      const std::array<double, 3U>& velocity_ecef_m_per_s,
                                      double earth_mu_m3_per_s2);

/** @brief 关机点判定输出的相态（甲方 2026-08-22 批注「判断其势能+动能最大的地方」）。 */
enum class BurnoutPhase {
  kObserving = 0,       /**< 证据不足暂态（前几拍或混合信号），如实写观测中。 */
  kBoosting = 1,        /**< 助推中：本采样命中助推特征（关机点未到）。 */
  kConfirmed = 2,       /**< 已关机：观测到助推段且其后连续无助推（关机点在窗口内）。 */
  kBeforeWindow = 3,    /**< 关机点在观测窗口外：从未见助推且能量自首采样平稳。 */
  kBeforeTrackStart = 4 /**< 关机点早于跟踪起点：能量自首采样即下降（旧分支语义保留）。 */
};

/**
 * @brief 单航迹关机点判定状态（会话内逐采样累计；纯数据，无静态状态）。
 * @note 峰值字段沿用旧 EnergyPeak 语义：助推段 ε 单调升，峰值自然落在最后一个
 *       助推采样——关机时刻即峰值时刻。
 */
struct BurnoutTrackerState {
  double energy_j_per_kg{0.0};   /**< 至今比机械能峰值。 */
  double time_sec{0.0};          /**< 峰值采样时刻。 */
  oneq::coordinate::EcefPositionM position{}; /**< 峰值位置。 */
  double radius_m{0.0};          /**< 峰值地心距（阈值换算用）。 */
  double speed_mps{0.0};         /**< 峰值速度模。 */
  bool peak_is_first_sample{false}; /**< 峰值仍为首采样（下降即意味关机早于跟踪起点）。 */
  bool valid{false};             /**< 已收到首个有效采样。 */
  bool ever_boosted{false};      /**< 观测窗口内出现过助推采样。 */
  std::uint32_t coast_samples{0U};  /**< 助推后连续非助推采样数（≥2 确认关机）。 */
  std::uint32_t flat_samples{0U};   /**< 无助推且能量平稳的连续采样数（≥5 判窗口外）。 */
  std::uint32_t rise_streak{0U};    /**< 连续能量上涨采样数（防抖：≥2 才计助推）。 */
  bool confirmed{false};         /**< 下降沿确认（ε 自峰值累计降幅超 0.1% 势能尺度）。 */
  double first_energy_j_per_kg{0.0}; /**< 首采样比机械能（平稳带锚点）。 */
  std::array<double, 3U> last_velocity_m_per_s = std::array<double, 3U>(); /**< 上一采样速度（加速度通道）。 */
  double last_time_sec{0.0};     /**< 上一采样时刻。 */
  double last_energy_j_per_kg{0.0}; /**< 上一采样比机械能（能量通道比较基准）。 */
};

/**
 * @brief 关机点判定状态机单步推进（纯函数，不受验收日志宏门控）。
 * @param[in,out] state 单航迹累计状态（调用方按航迹键持有）。
 * @param[in] position/velocity_ecef 本采样 ECEF 状态；t_sec 采样仿真时间。
 * @param[in] earth_mu_m3_per_s2 地球引力参数（m³/s²，配置单源；重力尺度 μ/r² 由此推导）。
 * @param[in] velocity_sigma_m 速度观测 1-σ（m/s；0 = 调用方无协方差信息，不设防）。
 * @return 本采样后的相态。
 * @note 助推采样双通道（任一命中）：|Δv|/Δt > 2.5·μ/r²（推力远超重力，滤波噪声
 *       几 m/s 够不着），或 ε 逐拍上涨超 1e-4·μ/r（噪声尺度）且连续 ≥2 拍（防抖）。
 *       关机确认双路径（粘性）：见过助推后连续 2 拍无助推；或 ε 自峰值累计下降超
 *       1e-3·μ/r（旧下降沿口径，覆盖慢推力缓升场景）。确认期间峰值锚点冻结
 *       （关机时刻不随确认后缓升漂移）；再次出现助推特征则撤销结论重开
 *       （多脉冲/再次加速语义）。从未助推且 ε 距首采样不超 1e-4·μ/r 连续 5 拍
 *       判窗口外。物理依据：无阻力滑行 dε/dt = 0。阈值与边界见
 *       algorithms.md「关机点判定」；相态为当拍最优证据结论，后续证据出现时按
 *       周期改判。
 * @note 评审 2026-08-27 条3 速度 σ 护栏：velocity_sigma_m ≥ 5 m/s（kBurnout-
 *       VelocitySigmaM，方法常数，依据：加速判别门 2.5g≈20 m/s² 在 1s 步长下
 *       需速度噪声远低于此）时不推进任何状态、返回观测中——仅方位/弱可观测
 *       航迹的速度欠估计收敛缓升与助推特征动力学不可分，判了必是假关机。
 */
BurnoutPhase UpdateBurnoutTracker(BurnoutTrackerState& state,
                                  const oneq::coordinate::EcefPositionM& position,
                                  const std::array<double, 3U>& velocity_ecef_m_per_s,
                                  double t_sec, double earth_mu_m3_per_s2,
                                  double velocity_sigma_m = 0.0);

/**
 * @brief Resolves a terminal burnout phase to a printable point estimate.
 * @param[in] phase Burnout state-machine phase.
 * @param[in] state Accumulated state; confirmed phases use the observed energy peak.
 * @param[in] current_time_sec Current absolute simulation time.
 * @param[in] trajectory Current trajectory solution; pre-observation phases use backtracking.
 * @param[out] burnout_time_sec Estimated absolute burnout time.
 * @param[out] burnout_position Estimated burnout position in geodetic coordinates.
 * @return True for an observed confirmation or a valid precision-gated backtrack.
 * @note The backtracked 100 km atmosphere-interface stop is the modeled upper boundary of
 *       powered flight. It is an estimate rather than an observed burnout confirmation.
 */
bool TryResolveBurnoutEstimate(BurnoutPhase phase, const BurnoutTrackerState& state,
                               double current_time_sec, const TrajectoryPrediction& trajectory,
                               double* burnout_time_sec,
                               oneq::coordinate::LlaPositionDegM* burnout_position);

/**
 * @brief 推演验收行写入。μ 与地球半径取引擎配置（TargetInferenceConfig 单源）。
 * @note results 以非 const 传入：关机点确认分支把敏度传播得到的关机点 1-σ 回填
 *       到 `TrajectoryPrediction::burnout_position_sigma_m`（评审 2026-08-26 条10）。
 */
void WriteInferenceAcceptance(const std::vector<InferenceTrackState>& tracks,
                              std::vector<TargetInferenceResult>& results,
                              const TargetInferenceConfig& config);

}  // namespace target_inference

#endif
