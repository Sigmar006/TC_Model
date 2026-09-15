/**
 * @file SbirsPolicyConfig.h
 * @brief 定义 SBIRS-inspired 检测、误差和调度策略。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

/** @brief 检测门限策略（线性 IR SNR 门限，对应 `T = μ + k·σ` 中的门限取值）。 */
struct ONEQ_API SbirsDetectionPolicyConfig {
  float wide_min_snr_linear{4.0f};  /**< WFOV 检测门限（k≈4） */
  float narrow_min_snr_linear{6.0f}; /**< NFOV 捕获门限（k≈5~6，要求更低虚警率） */
};

// 2.10 误差模型：5 类物理误差各自的 1-σ。
// sigma 单位：角度类为 deg，距离类为无量纲比例。
// random_seed 驱动可注入的确定性高斯采样源（保证 replay 可复现）。
/**
 * @brief 误差模型配置，定义 WFOV 带误差位置生成所用的 5 类物理误差 1-σ。
 * @note 轨道、姿态和视场三项 sigma 按 RSS 合成为有效角误差；三项均为 0 时不注入随机角误差。
 *       方位/俯仰独立采样，`random_seed` 保证 replay 可复现。
 */
struct ONEQ_API SbirsErrorModelConfig {
  float range_fraction_sigma{0.001f};    /**< 距离乘法误差比例 1-σ（约 0.1%），无量纲 */
  std::uint32_t random_seed{1U};         /**< 固定 32 位随机源种子，驱动可注入高斯采样 */
  float orbit_sigma_deg{0.00005f};       /**< 卫星轨道误差角度 1-σ，单位 deg */
  float attitude_sigma_deg{0.0001f};     /**< 卫星姿态误差角度 1-σ，单位 deg */
  float fov_sigma_deg{0.00005f};         /**< 探测器视场（像元/畸变）误差 1-σ，单位 deg */
  float detector_bandwidth_hz{100.0f};   /**< 探测器带宽，用于动态滞后误差，单位 Hz */
  float nav_position_sigma_m{50.0f};     /**< 卫星导航定位径向误差 1-σ（典型 ≈50m），仅验收旁路 ECEF 误差行使用 */
  /** @note 甲方 2026-08-27 指标：红外系统测角误差 ≤3 μrad（≈0.000172°）。三项
   *        σ 默认 RSS ≈ 1.95 μrad（3×10⁻⁵°/10⁻⁴°/5×10⁻⁵° 组合），注入即在指标
   *        内且留 35% 余量；经 NFOV 滤波后验收行（红外系统测角误差 RMSE）更低。
   *        旧默认姿态 0.01°（≈174 μrad）实测 RMSE ≈0.0017°，超指标约 10 倍。 */
};

/**
 * @brief 实际光学中心的时间相关姿态与逐 NFOV 通道指向扰动配置。
 * @note 共模项同时作用于 WFOV/NFOV；通道项仅作用于对应 NFOV。全部幅值默认 0，
 *       与 `SbirsErrorModelConfig` 的量测误差及 mission 静态 settle error 相互独立。
 */
struct ONEQ_API SbirsPointingDisturbanceConfig {
  float common_attitude_sigma_deg{0.0f};           /**< 共模 GM 各角轴平稳 1-σ，单位 deg */
  float common_attitude_correlation_time_s{1.0f};  /**< 共模 GM 相关时间，单位 s */
  float channel_pointing_sigma_deg{0.0f};          /**< 逐通道 GM 各角轴平稳 1-σ，单位 deg */
  float channel_pointing_correlation_time_s{1.0f}; /**< 逐通道 GM 相关时间，单位 s */
  float channel_vibration_amplitude_deg{0.0f};     /**< 逐通道确定性振动各轴峰值，单位 deg */
  float channel_vibration_frequency_hz{0.0f};      /**< 逐通道确定性振动频率，单位 Hz */
  std::uint32_t random_seed{1U};                   /**< 共模与逐通道独立随机流的 32 位基础种子 */
};

/**
 * @brief NFOV 资源调度器配置（单镜筒，2026-09-02 起）。
 * @note 窄场只有一个镜筒（单执行器分时轮转）：可同时保持的精跟条数不再可配置——
 *       由轮转物理涌现（分离目标轮空超过跟踪门容忍即丢锁）。历史配置项
 *       `max_concurrent_nfov_locks` 已删除（等效多镜筒的虚构能力）。场景 JSON 残留
 *       该键：sbirs 场景装载器显式拒绝报错；通用装载器（examples config_loader）
 *       按忽略未知键的既有策略静默跳过。
 */
struct ONEQ_API SbirsSchedulerConfig {
  /**
   * @brief WFOV→NFOV 切换所需的连续 WFOV 检测命中次数（>=1）。
   * @note 宽窄切换前置条件：目标连续 N 个周期通过 WFOV 几何+SNR 门后才允许进入
   *       NFOV 调度。默认 1 与既有单次命中即调度行为逐位一致；计数在进入跟踪时清零，
   *       丢锁回宽场后需重新积累。连续命中计数随验收日志输出（3.2.1.3.2.1）。
   */
  int wide_to_narrow_required_consecutive_hits{1};
};

/** @brief 首次 NFOV 捕获成功后采用的互斥跟踪模式。 */
enum class ONEQ_API SbirsTrackingMode {
  kEstimated = 0,                 /**< 使用估计器预测、校正并生成跟踪输出 */
  kStrictTruthAssisted,           /**< 真值 LOS 驱动指向并输出精确真值 */
  kSensorLikeTruthAssisted        /**< 真值 LOS 驱动指向，输出使用传感器误差模型 */
};

/** @brief Estimated 跟踪模式采用的生产估计后端。 */
enum class ONEQ_API SbirsEstimatedTrackingBackend {
  kEkf = 0, /**< 单 EKF 后端 */
  kImm,     /**< IMM(EKF) 后端 */
  kAngleCvKf /**< 实验：4 维角度 CV 线性标准 KF；非默认 */
};

/**
 * @brief 三种互斥跟踪模式及 Estimated 后端配置。
 * @details 默认进入 Estimated，并在 EKF/IMM 中选择后端；两个 TruthAssisted 模式使用独立状态与
 *          输出来源。SNR/可探测性始终使用物理真值链路。初始协方差 P0 由位置/速度 1-σ 构造。
 */
struct ONEQ_API SbirsTrackingConfig {
  SbirsTrackingMode tracking_mode{SbirsTrackingMode::kEstimated};
  SbirsEstimatedTrackingBackend estimated_backend{SbirsEstimatedTrackingBackend::kEkf};
  float process_noise_diff_coeff{1.0f}; /**< 过程噪声扩散系数 q。EKF/IMM：CV 加速度白噪声（m²/s³）；kAngleCvKf：视线角加速度白噪声（rad²/s³） */
  float initial_position_std_m{1000.0f};       /**< 初始位置 1-σ（米），构造 P0 位置对角元 */
  float initial_velocity_std_m_per_s{100.0f};  /**< 初始速度 1-σ（m/s），构造 P0 速度对角元 */
  unsigned int nis_gate_loss_cycles{0U}; /**< 连续 NIS 超过 2 维 95% 门限后丢锁的周期数；0 表示禁用 */
  unsigned int nfov_tracking_gate_loss_cycles{2U}; /**< NFOV 几何/SNR 门连续失败后丢锁周期数（>=1） */
  std::vector<float> imm_model_noise_diff_coeffs{}; /**< IMM 各模型的过程噪声扩散系数；空向量表示使用默认 [1.0, 100.0] */
};

/**
 * @brief SBIRS-inspired 策略聚合配置，组合检测、误差、调度和跟踪四域策略。
 * @note 纯数据类型 (POD)，作为 `SbirsSessionConfig::policy` 的子配置。
 */
struct ONEQ_API SbirsPolicyConfig {
  SbirsDetectionPolicyConfig detection{};  /**< 检测门限策略 */
  SbirsErrorModelConfig error_model{};     /**< 误差模型策略 */
  SbirsPointingDisturbanceConfig pointing_disturbance{}; /**< 实际光轴时间相关扰动 */
  SbirsSchedulerConfig scheduler{};        /**< NFOV 资源调度策略 */
  SbirsTrackingConfig tracking{};          /**< EKF 滤波测量跟踪策略 */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
