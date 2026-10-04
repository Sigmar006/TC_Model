/**
 * @file ArHardwareConfig.h
 * @brief 机载雷达硬件域主配置类型集合。
 *
 * 硬件域配置（探测链路物理参数、天线方向图、RCS 物理建模、信号处理增益偏置等）
 * 的主头文件。
 *
 * @note 硬件域包含硬件能力参数（发射机、天线、接收机、RCS 物理）与装备级
 *       信号处理增益偏置。检测判决门限（minimum_snr_db、pfa、pulse_count、
 *       minimum_detection_margin_db）属于策略域 ArPolicyConfig::detection，不在此类型中。
 *       内部通过 MapSessionToExecution() 将 hardware + policy.detection 合并为
 *       engineering::DetectionConfig。
 *
 * @note 原硬件/方向图/RCS/跟踪/生命周期语义档位（ArHardwareProfile 等）已由
 *       ArProfileConstants.h 中的预定义结构体常量取代，不再提供枚举。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_HARDWARE_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_HARDWARE_CONFIG_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/api.hpp"
#include "1q/electromagnetics/RfLinkBudget.h"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace config {

namespace profiles {

/**
 * @brief 目标雷达散射截面起伏统计模型。
 *
 * 不同 Swerling 模型对应不同的目标 RCS 起伏统计特性，
 * 影响多脉冲检测概率的计算方式。
 */
enum class ONEQ_API SwerlingModel {
  kSwerling0 = 0, /**< 不起伏——目标 RCS 在各次观测中恒定。 */
  kSwerling1 = 1, /**< 扫描间慢起伏，单次扫描内 RCS 恒定。 */
  kSwerling2 = 2, /**< 脉冲间快起伏，每个脉冲 RCS 独立采样。 */
  kSwerling3 = 3, /**< 扫描间慢起伏，RCS 服从 2 自由度 chi-squared 分布。 */
  kSwerling4 = 4  /**< 脉冲间快起伏，RCS 服从 2 自由度 chi-squared 分布。 */
};

}  // namespace profiles

namespace detection {

/**
 * @brief 方向图模型类型。
 */
enum class ONEQ_API AntennaPatternModelType {
  kGaussianMainLobe = 0,  /**< 高斯主瓣近似。 */
  kParabolicMainLobe = 1, /**< 抛物线主瓣近似。 */
  kCosinePower = 2,       /**< 余弦幂方向图近似。 */
  kSincPattern = 3        /**< sinc² 方向图（均匀孔径理论解，需物理孔径尺寸）。 */
};

/**
 * @brief 天线方向图参数。
 */
struct ONEQ_API AntennaPatternConfig {
  AntennaPatternModelType model_type{
      AntennaPatternModelType::kGaussianMainLobe};    /**< 主瓣模型类型。 */
  float max_sidelobe_level_db{-20.0f};                /**< 最大旁瓣电平。 */
  float backlobe_level_db{-35.0f};                    /**< 后瓣电平。 */
  float scan_loss_coeff_db_per_deg2{0.0f};            /**< 扫描损失系数。 */
  float max_scan_loss_db{6.0f};                       /**< 扫描损失上限。 */
  config::AzimuthElevationDeg boresight_offset_deg{}; /**< 方向图相对安装轴偏置。 */
};

/**
 * @brief 天线工程参数。
 */
struct ONEQ_API AntennaConfig {
  float main_beam_gain_db{35.0f}; /**< 主瓣峰值增益。 */
  float nominal_az_beamwidth_deg{
      4.0f}; /**< 名义方位波束宽度；正值直接生效，0 表示从有效物理孔径推导。 */
  float nominal_el_beamwidth_deg{
      4.0f}; /**< 名义俯仰波束宽度；正值直接生效，0 表示从有效物理孔径推导。 */
  float antenna_length_m{
      0.0f}; /**< 物理方位孔径尺寸；正值参与波束推导和 sinc² 模式，0 表示未配置。 */
  float antenna_width_m{
      0.0f}; /**< 物理俯仰孔径尺寸；正值参与波束推导和 sinc² 模式，0 表示未配置。 */
  AntennaPatternConfig pattern{};         /**< 方向图参数。 */
  bool enable_directional_pattern{false}; /**< 是否启用离轴方向图评估。 */
};

/**
 * @brief 接收机工程参数。
 */
struct ONEQ_API ReceiverConfig {
  float noise_figure_db{4.0f};    /**< 接收机噪声系数。 */
  float receive_loss_db{2.0f};    /**< 接收链路损耗。 */
  float cross_polarization_isolation_db{30.0f};             /**< 正交极化隔离（dB）。 */
  float minimum_far_field_range_m{1.0f};                    /**< 远场公式最小适用距离（m）。 */
  bool has_co_site_isolation{false};                        /**< 是否配置同平台耦合隔离。 */
  float co_site_isolation_db{0.0f};                         /**< 同平台耦合路径隔离（dB）。 */
  float maximum_linear_input_power_w{1.0e-3f};              /**< 线性接收上限（W）。 */
  float preselector_bandwidth_hz{20.0e6f};                  /**< 宽带前端预选器带宽（Hz）。 */
  float interference_observation_jn_gate_db{6.0f};          /**< 干扰观测发布 J/N 门限（dB）。 */
  oneq::electromagnetics::RfScenePolarization scene_polarization{
      oneq::electromagnetics::RfScenePolarization::kHorizontal}; /**< RF v2 接收极化。 */
  std::vector<oneq::electromagnetics::RfCoSiteIsolationPath> co_site_paths{};
  /**< 伙伴发射源的显式有向隔离路径（端点 = 对方传感器实体 ID）。自身发射泄漏
       隔离不在此配置：会话组装接收状态时按当前实体身份自动附加（显式配置
       同自身实体 ID 的路径可覆盖缺省隔离值）。 */
};

/**
 * @brief 发射机工程参数。
 */
struct ONEQ_API TransmitterConfig {
  float peak_power_w{1e6f};            /**< 峰值发射功率。 */
  float frequency_hz{3e9f};            /**< 工作频率。 */
  float bandwidth_hz{4.5e6f};          /**< 发射带宽。 */
  float pulse_width_s{13e-6f};         /**< 脉宽。 */
  float prf_hz{300.0f};                /**< 脉冲重复频率。 */
  float transmit_loss_db{3.5f};        /**< 发射链路损耗。 */
  float maximum_peak_power_w{1.2e6f};  /**< 烧穿等控制不可越过的峰值功率上限。 */
  float maximum_duty_cycle{0.10f};     /**< 允许的最大发射占空比，范围 (0, 1]。 */
  float maximum_pulse_energy_j{20.0f}; /**< 单脉冲能量上限（J）。 */
  std::vector<double> frequency_plan_hz = std::vector<double>(1U, 3.0e9); /**< 允许周期执行选择的离散载频表。 */
};

/**
 * @brief RCS 物理建模参数。
 */
struct ONEQ_API RcsPhysicsConfig {
  bool enable_physical_rcs{true};       /**< 是否启用物理 RCS 估计；默认开启。 */
  float physics_mix_ratio{1.0f};        /**< 物理估计占比 [0,1]：0=只用场景输入 RCS，1=完全物理估计。与扫描无关。 */
  float cylinder_weight{0.5f};          /**< 圆柱散射模型权重。 */
  float min_equivalent_radius_m{0.05f}; /**< 等效半径下界。 */
  float max_equivalent_radius_m{5.0f};  /**< 等效半径上界。 */
  float min_rcs_m2{0.01f};              /**< RCS 裁剪下界。 */
  float max_rcs_m2{1000.0f};            /**< RCS 裁剪上界。 */
  float bistatic_psi_offset_deg{5.0f};  /**< 双站角偏移补偿。 */
};

/**
 * @brief 信号处理增益偏置（四增益分项账本，缺省 0 dB 等于保守账本）。
 *
 * 四个 dB 偏置叠加在检测单元分项 SINR 账本上；**缺省 0 dB 时与保守账本
 * 逐位一致**。脉压（B·τ）与积累（N）增益永远自动派生，禁止把派生量手填进
 * 偏置（防双算）；额外链路损耗继续走 `transmit_loss_db`/`receive_loss_db`。
 * 符号约定：改善因子正 dB 为优（target/clutter/jamming），噪声代价正 dB 为劣
 * （noise，与 `noise_figure_db` 同向）。值域 [0, 40] dB（配置校验拒绝越界）。
 */
struct ONEQ_API SignalProcessingConfig {
  float target_processing_gain_db{0.0f}; /**< 目标信号额外处理增益；正 = 提升 SINR（账本分子）。 */
  float noise_processing_gain_db{0.0f}; /**< 噪声代价；正 = 抬高噪声底（账本分母）。 */
  float clutter_suppression_gain_db{0.0f}; /**< 杂波抑制（MTI 改善因子）；正 = 抑制杂波（分母）。 */
  float jamming_suppression_gain_db{0.0f}; /**< 干扰抑制（旁瓣对消改善因子）；正 = 抑制干扰（分母）。 */
};

/**
 * @brief 探测硬件能力聚合配置。
 *
 * 包含硬件能力参数（发射机、天线、接收机、RCS 物理建模）与装备级信号处理
 * 增益偏置。检测判决门限不在此结构中，参见 ArPolicyConfig::detection。
 */
struct ONEQ_API DetectionConfig {
  TransmitterConfig transmitter{}; /**< 发射机参数。 */
  AntennaConfig antenna{};         /**< 天线参数。 */
  ReceiverConfig receiver{};       /**< 接收机参数。 */
  RcsPhysicsConfig rcs_physics{};  /**< RCS 物理建模参数。 */
  SignalProcessingConfig signal_processing{}; /**< 信号处理增益偏置（默认全 0 dB）。 */
};

}  // namespace detection

using detection::AntennaPatternModelType;
using detection::DetectionConfig;

/**
 * @brief 雷达硬件域配置——DetectionConfig 别名。
 *
 * 包含硬件能力与装备级信号处理增益偏置。检测门限（minimum_snr_db 等）在
 * ArPolicyConfig::detection 中。
 */
using ArHardwareConfig = detection::DetectionConfig;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_HARDWARE_CONFIG_H_
