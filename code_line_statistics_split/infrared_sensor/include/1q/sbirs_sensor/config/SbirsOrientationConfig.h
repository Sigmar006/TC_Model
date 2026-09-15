/**
 * @file SbirsOrientationConfig.h
 * @brief 定义 SBIRS-inspired 传感器安装指向与稳定配置（对齐 ArOrientationConfig 语义）。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ORIENTATION_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ORIENTATION_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace sbirs_sensor {
namespace config {

/**
 * @brief SbirsScanLimitsDeg 表示传感器系方位/俯仰扫描限位（单位：deg）。
 * @note 限位在传感器坐标系内约束 WFOV 扫描中心与 NFOV ATP 指向命令；默认
 *       az [-180, 180]、el [-90, 90] 等效不约束（保证零姿态 + 零安装角配置下
 *       行为与历史版本逐位一致）。AR 的机械/电子限位默认 ±60/±30 会收紧既有
 *       场景，故此处默认取全开、仅对齐语义。
 */
struct ONEQ_API SbirsScanLimitsDeg {
  float az_min_deg{-180.0f}; /**< 方位最小扫描角（单位：deg） */
  float az_max_deg{180.0f};  /**< 方位最大扫描角（单位：deg） */
  float el_min_deg{-90.0f};  /**< 俯仰最小扫描角（单位：deg） */
  float el_max_deg{90.0f};   /**< 俯仰最大扫描角（单位：deg） */
};

/**
 * @brief SbirsStabilizationMode 表示传感器扫描稳定方式。
 * @note 不引入 AR 的 kGroundStabilized：天基传感器无对地稳定语义
 *       （AR 的 ground 模式在无地理参考时本就等效惯性稳定）。
 */
enum class ONEQ_API SbirsStabilizationMode {
  kBodyStabilized = 0,     /**< 随体稳定：扫描参数为传感器系角度，光轴足迹随卫星姿态变化 */
  kInertialStabilized = 1, /**< 惯性稳定：扫描参数为 ECI 参考方向，经链路反解到传感器系实现 */
};

/**
 * @brief SbirsMisalignmentModel 表示安装失准角误差模型（阶段 3）：常值偏置 +
 *        运行期一次抽取的常值随机微扰。
 * @note 语义边界（与量测域 `attitude_sigma_deg` 及扰动共模的谱正交）：
 *       - 安装失准 = 静态误差（运行内常值；bias + 一次 N(0,σ) 抽取合成，运行期不重抽）；
 *       - 扰动共模 = 时变 GM（tau>0）——静态 vs 时变谱不重叠，无双重计模；
 *       - 量测域 `attitude_sigma_deg` = 时刻输出误差（污染量测，不进链）。
 *       失准误差合成进 boresight 链（R_sensor_to_eci = R_att · R_mount⁻¹ ·
 *       R_misalign⁻¹），影响实际光轴足迹与门控；不污染量测输出、不进
 *       BuildMeasurementCovariance。
 * @note 本结构为初始化静态配置，不进入运行期 RuntimeConfigPatch；random_seed
 *       驱动运行期一次高斯抽取（pipeline 构造/ApplyConfig 时），保证 replay 可复现。
 */
struct ONEQ_API SbirsMisalignmentModel {
  oneq::coordinate::EulerAnglesDeg bias_deg{}; /**< 常值失准偏置（单位：deg，Z-Y-X，Body->Sensor） */
  float random_sigma_deg{0.0f};                /**< 随机微扰 1-σ（单位：deg；0 时不抽取随机流） */
  std::uint32_t random_seed{1U};               /**< 微扰流 32 位种子（固定，保证 replay 可复现） */
};

/**
 * @brief SbirsOrientationConfig 表示 SBIRS-inspired 传感器安装指向配置。
 * @note 静态基准组合关系（对齐 ArOrientationConfig 头注释）：
 *       actual_boresight = attitude(Body->ECI) ∘ mount(Body->Sensor)
 *                          ∘ misalignment⁻¹ ∘ scan(传感器系)。
 * @note 本结构体默认全零安装/全零失准（库内几何/单测逐位不变）。示例与场景
 *       JSON 加载层另有非零默认集 SbirsOrientationDefaults()
 *       （examples config_loader_detail.h，验收演示误差参数）：JSON 缺键时
 *       继承该默认集，与本结构体默认无关。
 * @note 本结构为初始化静态配置，不进入运行期 RuntimeConfigPatch；
 *       扫描参数（scan_start_az/span/el）的参考系由 stabilization_mode 决定
 *       （体稳定=传感器系；惯性稳定=ECI 参考定义）。
 */
struct ONEQ_API SbirsOrientationConfig {
  /**
   * @brief 传感器安装偏置角（单位：deg，参考系：Body -> Sensor，Z-Y-X 欧拉）。
   * @note 与周期输入的卫星姿态（Body->ECI）通过旋转矩阵复合得 ECI->Sensor
   *       等效姿态；静态配置（可动安装座如需运行期变化须扩展 patch 通道）。
   */
  oneq::coordinate::EulerAnglesDeg mount_angles_deg;
  SbirsScanLimitsDeg sensor_scan_limits_deg; /**< 传感器系扫描限位 */
  SbirsStabilizationMode stabilization_mode{
      SbirsStabilizationMode::kBodyStabilized}; /**< 扫描稳定方式 */
  SbirsMisalignmentModel misalignment{}; /**< 安装失准角误差模型（常值偏置 + 运行期一次随机抽取） */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_ORIENTATION_CONFIG_H_
