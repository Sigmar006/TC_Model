/**
 * @file RirHardwareConfig.h
 * @brief 远程识别雷达硬件域主配置类型。
 *
 * 硬件域仅包含物理硬件能力参数（发射机、天线、接收机），服务于识别链路的
 * 效能级 SNR 计算与距离像分辨率推导（带宽）。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_HARDWARE_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_HARDWARE_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/electromagnetics/RfScene.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirAzimuthElevationDeg 雷达局部坐标系的方位-俯仰二维角度（单位：度）。
 */
struct ONEQ_API RirAzimuthElevationDeg {
  float az_deg{0.0f}; /**< 方位角（deg）：从 +x 向 +y，[-180, 180]。 */
  float el_deg{0.0f}; /**< 俯仰角（deg）：相对水平面，[-90, 90]，正值向上。 */

  RirAzimuthElevationDeg() = default;
  RirAzimuthElevationDeg(float az_deg_in, float el_deg_in)
      : az_deg(az_deg_in), el_deg(el_deg_in) {}
};

/**
 * @brief RirAzimuthElevationLimitsDeg 方位-俯仰扫描限位（单位：度）。
 */
struct ONEQ_API RirAzimuthElevationLimitsDeg {
  float az_min_deg{-60.0f}; /**< 方位最小扫描角（单位：度）。 */
  float az_max_deg{60.0f};  /**< 方位最大扫描角（单位：度）。 */
  float el_min_deg{-30.0f}; /**< 俯仰最小扫描角（单位：度）。 */
  float el_max_deg{30.0f};  /**< 俯仰最大扫描角（单位：度）。 */

  RirAzimuthElevationLimitsDeg() = default;
  RirAzimuthElevationLimitsDeg(float az_min_deg_in, float az_max_deg_in,
                               float el_min_deg_in, float el_max_deg_in)
      : az_min_deg(az_min_deg_in), az_max_deg(az_max_deg_in),
        el_min_deg(el_min_deg_in), el_max_deg(el_max_deg_in) {}
};

namespace hardware {

/**
 * @brief 方向图模型类型：只影响离波束中心越远增益掉多快，不改变波束中心指向。
 */
enum class ONEQ_API RirAntennaPatternModelType {
  kGaussianMainLobe = 0,  /**< 高斯主瓣近似。 */
  kCosinePower = 2,       /**< 余弦幂方向图近似。 */
  kSincPattern = 3        /**< sinc² 方向图（均匀孔径理论解，需物理孔径尺寸）。 */
};

/**
 * @brief 天线工程参数（含方向图形状）。
 * @note 扫描损失：波束偏离安装轴时额外扣增益，
 *       `scan_loss_coeff_db_per_deg2` × 偏角²（默认 0 = 不扣），
 *       再被 `max_scan_loss_db` 封顶（默认 6 dB）。
 */
struct ONEQ_API RirAntennaConfig {
  float main_beam_gain_db{35.0f}; /**< 主瓣峰值增益。 */
  float nominal_az_beamwidth_deg{
      4.0f}; /**< 名义方位波束宽度；正值直接生效，0 表示从有效物理孔径推导。 */
  float nominal_el_beamwidth_deg{
      4.0f}; /**< 名义俯仰波束宽度；正值直接生效，0 表示从有效物理孔径推导。 */
  float antenna_length_m{
      1.2f}; /**< 物理方位孔径（m）；>0 可推波束宽 / 供 sinc²；0=未配置。 */
  float antenna_width_m{
      1.2f}; /**< 物理俯仰孔径（m）；>0 可推波束宽 / 供 sinc²；0=未配置。 */
  RirAntennaPatternModelType model_type{
      RirAntennaPatternModelType::kGaussianMainLobe}; /**< 主瓣模型类型。 */
  float max_sidelobe_level_db{-20.0f};                /**< 最大旁瓣电平。 */
  float backlobe_level_db{-35.0f};                    /**< 后瓣电平。 */
  float scan_loss_coeff_db_per_deg2{0.0f};            /**< 扫描损失系数（dB/deg²）。 */
  float max_scan_loss_db{6.0f};                       /**< 扫描损失封顶（dB）。 */
  config::RirAzimuthElevationDeg boresight_offset_deg{}; /**< 方向图相对安装轴偏置。 */
};

/**
 * @brief 接收机工程参数。
 */
struct ONEQ_API RirReceiverConfig {
  float noise_figure_db{4.0f};    /**< 接收机噪声系数。 */
  float receive_loss_db{2.0f};    /**< 接收链路损耗。 */
  float cross_polarization_isolation_db{30.0f}; /**< 正交极化隔离（dB）。 */
  float maximum_linear_input_power_w{1.0e-3f};  /**< 线性接收上限（W）。 */
  oneq::electromagnetics::RfScenePolarization scene_polarization{
      oneq::electromagnetics::RfScenePolarization::kHorizontal}; /**< RF v2 接收极化。 */
};

/**
 * @brief 发射机工程参数。
 */
struct ONEQ_API RirTransmitterConfig {
  float peak_power_w{1e6f};       /**< 峰值发射功率。 */
  float frequency_hz{3e9f};       /**< 工作频率。 */
  float bandwidth_hz{4.5e6f};     /**< 发射带宽。 */
  float pulse_width_s{13e-6f};    /**< 脉宽。 */
  float prf_hz{300.0f};           /**< 脉冲重复频率。 */
  float transmit_loss_db{3.5f};   /**< 发射链路损耗。 */
};

/**
 * @brief RCS 物理建模参数。
 */
struct ONEQ_API RirRcsPhysicsConfig {
  bool enable_physical_rcs{false};      /**< 是否启用物理 RCS 估计；默认关闭。 */
  float physics_mix_ratio{1.0f};        /**< 物理估计占比 [0,1]：0=只用场景输入 RCS，1=完全物理估计。与扫描无关。 */
  float cylinder_weight{0.5f};          /**< 圆柱散射模型权重。 */
  float min_equivalent_radius_m{0.05f}; /**< 等效半径下界。 */
  float max_equivalent_radius_m{5.0f};  /**< 等效半径上界。 */
  float min_rcs_m2{0.01f};              /**< RCS 裁剪下界。 */
  float max_rcs_m2{1000.0f};            /**< RCS 裁剪上界。 */
  float bistatic_psi_offset_deg{5.0f};  /**< 双站角偏移补偿。 */
};

/**
 * @brief 信号处理增益偏置（dB，值域 [0, 40]）。
 */
struct ONEQ_API RirSignalProcessingConfig {
  float target_processing_gain_db{3.0f}; /**< 目标信号额外处理增益；正 = 提升 SINR。 */
  float noise_processing_gain_db{1.0f}; /**< 噪声代价；正 = 抬高噪声底。 */
  float clutter_suppression_gain_db{10.0f}; /**< 杂波抑制（MTI 改善因子）；正 = 抑制杂波。 */
  float jamming_suppression_gain_db{8.0f}; /**< 干扰抑制；正 = 抑制干扰。 */
  /**
   * @brief 是否把 MTI/MTD 响应与相干积累并入检测 SINR（2026-10-22）。
   * @details 缺省 false：沿用"四标量处理增益"的单脉冲账本，行为与改动前逐位一致。
   *   置 true 且检测积累脉冲数≥2 时，2 脉冲 MTI 响应、MTD 跨 bin 损耗与相干积累增益 N
   *   一并进入 SINR（盲速处如实丢检测）。本字段不进回放序列化，回放默认为 false。
   */
  bool enable_mti_mtd_effects{false};
};

}  // namespace hardware

/**
 * @brief RirHardwareConfig 远程识别雷达硬件域配置。
 */
struct ONEQ_API RirHardwareConfig {
  hardware::RirTransmitterConfig transmitter{};      /**< 发射机参数。 */
  hardware::RirAntennaConfig antenna{};              /**< 天线参数。 */
  hardware::RirReceiverConfig receiver{};            /**< 接收机参数。 */
  hardware::RirRcsPhysicsConfig rcs_physics{};       /**< RCS 物理建模参数。 */
  hardware::RirSignalProcessingConfig signal_processing{}; /**< 信号处理增益偏置。 */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_HARDWARE_CONFIG_H_
