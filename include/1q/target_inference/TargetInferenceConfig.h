/**
 * @file TargetInferenceConfig.h
 * @brief 定义目标推演引擎配置（弹道模型与判决阈值）。
 */

#ifndef ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_CONFIG_H_
#define ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_CONFIG_H_

#include <cstdint>
#include <string>

#include "1q/api.hpp"

namespace target_inference {

/**
 * @brief 目标推演配置。
 * @note 弹道模型为中心引力 + 可选指数大气阻力（ballistic coefficient ≤ 0 关闭）；
 *       助推段未建模——发射点定义为回推弹道的大气界面停机点（回推状态高度
 *       降至 launch_atmosphere_interface_altitude_m 即停机；关机前目标不 estimable，
 *       停机点即"助推段上界"估计），发射点 1σ 含随回推时长线性增长的模型误差项
 *       （见 algorithms.md 边界）。
 */
struct ONEQ_API TargetInferenceConfig {
  double earth_mu_m3_per_s2{3.986004418e14}; /**< 地球引力参数（单位：m³/s²）。 */
  double earth_radius_m{6378137.0};          /**< 地球半径（单位：m）。 */
  double prediction_horizon_sec{300.0};      /**< 轨迹预测时域（单位：s）。 */
  double integration_step_sec{1.0};          /**< 弹道积分步长（单位：s）。 */
  double waypoint_interval_sec{10.0};        /**< 航迹点输出间隔（单位：s）。 */
  double launch_max_backtrack_sec{900.0};    /**< 回推时长上限（单位：s）。 */
  double launch_atmosphere_interface_altitude_m{100000.0}; /**< 回推大气界面停机高度（单位：m）；回推状态地心高低于该值即停机，发射点估计=停机点（助推段未建模的物理停机门）。 */
  double launch_model_error_rate_m_per_s{50.0}; /**< 发射点模型误差增长率（单位：m/s）；1σ 主项=该速率×回推时长（助推段/模型未建模误差随回推时长增长）。 */
  double drag_ballistic_coefficient_m2_per_kg{0.0}; /**< 弹道系数（≤ 0 关闭阻力）。 */
  double kinematic_type_weight{0.5};         /**< 运动学先验 vs 外部证据的融合权重。 */
  double high_energy_speed_threshold_m_per_s{1200.0}; /**< 高能量速度门（单位：m/s）。 */
  double high_altitude_threshold_m{30000.0}; /**< 高空门（单位：m）。 */
  /** 落点预报分发通道名（如事件信号名）。装配层接好外发通道后填入，验收行据此写
   *  「分发状态=已发布」；留空 = 未接外发，写「已封装待分发」。 */
  std::string impact_distribution_channel{};
  /** 落点预报分发来源实体 ID（验收判定标准 第25项：发布方实体 ID，不用通道名代替；
   *  0 = 装配层未标注，已发布行省略该字段）。 */
  std::uint64_t impact_distribution_source_id{0U};
  /** 落点预报分发对象实体 ID（接收方；0 = 未标注，省略）。 */
  std::uint64_t impact_distribution_target_id{0U};
  /**
   * @brief 落点预报的标准化封装版本（验收判定标准 第25项 `标准化封装版本=`）。
   *
   * @details 判据要求"模拟预报告信息的标准化封装与分发"必须留下封装要素，版本号属封装头，
   *   由装配层配置；缺省 1.0 表示首版格式。
   */
  // v120 限制：类类型成员不能用 {} 初始化（C2797），改用 = 拷贝初始化。
  std::string impact_distribution_message_version = std::string("1.0");
  /**
   * @brief 落点预报消息类型（验收判定标准 第25项 `消息类型=`）。
   *
   * @details 同一分发通道将来可能承载多类消息，类型字段用于接收方分流；本板只有落点预报。
   */
  std::string impact_distribution_message_type = std::string("落点预报");
  /**
   * @brief 落点预报对外报出的置信度（验收判定标准 第25项 `置信度=`）。
   *
   * @details 与 `落点误差范围1σ=` 并列输出：误差范围给出 1-σ，置信度给出该报文声明的
   *   置信水平（缺省 0.95）。两者口径不同、不得互相换算后替换。
   */
  double impact_distribution_confidence{0.95};
};

}  // namespace target_inference

#endif  // ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_CONFIG_H_
